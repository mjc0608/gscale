/*
 * vGT core module
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2011 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2011 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/hash.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <asm/bitops.h>
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_drv.h"
#include "vgt_devtable.h"
#include <xen/vgt-if.h>

/*
 * NOTE list:
 * 	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
 * 	  also the hooks in AGP driver
 * 	- need a check on "unsigned long" vs. "u64" usage
 * 	- need consider cache related issues, e.g. Linux/Windows may have different
 * 	  TLB invalidation mode setting, which may impact vGT's context switch logic
 */
static bool vgt_restore_context (struct vgt_device *vgt);
static bool vgt_save_context (struct vgt_device *vgt);

bool hvm_render_owner = false;
static int __init hvm_render_setup(char *str)
{
	hvm_render_owner = true;
	return 1;
}
__setup("hvm_render_owner", hvm_render_setup);

static bool hvm_dpy_owner = false;
static int __init hvm_dpy_setup(char *str)
{
	hvm_dpy_owner = true;
	return 1;
}
__setup("hvm_dpy_owner", hvm_dpy_setup);

static int __init hvm_owner_setup(char *str)
{
	hvm_dpy_owner = true;
	hvm_render_owner = true;

	return 1;
}
__setup("hvm_owner", hvm_owner_setup);

static bool vgt_primary = false;
static int __init vgt_primary_setup(char *str)
{
	vgt_primary = true;
	return 1;
}
__setup("vgt_primary", vgt_primary_setup);

bool vgt_debug;
static int __init vgt_debug_setup(char *str)
{
	vgt_debug = true;
	return 1;
}
__setup("vgt_debug", vgt_debug_setup);

bool novgt = false;
static int __init vgt_novgt_setup(char *str)
{
	novgt = true;
	return 1;
}
__setup("novgt", vgt_novgt_setup);

static int start_period = 10; /* in unit of second */
static int __init period_setup(char *str)
{
	start_period = simple_strtoul(str, NULL, 10);
	return 1;
}
__setup("vgt_start_period=", period_setup);

static int fastmode = 1;
static int __init mode_setup(char *str)
{
	fastmode = 1;
	return 1;
}
__setup("vgt_fastmode", mode_setup);

/*
 * FIXME: now video ring switch has weird issue. The cmd
 * parser may enter endless loop even when head/tail is
 * zero. earlier posting read doesn't solve the issue.
 * so disable it for now.
 */
static int enable_video_switch = 0;
static int __init video_switch_setup(char *str)
{
	enable_video_switch = 1;
	return 1;
}

__setup("enable_video_switch", video_switch_setup);

/* enable this to use the old style switch context */
static int use_old_ctx_switch = false;
static int __init use_old_ctx_switch_setup(char *str)
{
    use_old_ctx_switch = true;
    return 1;
}
__setup("use_old_ctx_switch", use_old_ctx_switch_setup);

LIST_HEAD(pgt_devices);
static struct pgt_device default_device = {
	.bus = 0,
	.devfn = 0x10,		/* BDF: 0:2:0 */
};

static int dom0_aperture_sz = 64;	//in MB.
static int dom0_gm_sz = 64;			//in MB. Dom0 has no hidden gm.

static int __init dom0_aperture_sz_setup(char *str)
{
	int t;
	if (sscanf(str, "%d", &t) == 1 && t > 0 && t <= 256)
		dom0_aperture_sz = t;
	else {
		printk("vGT: dom0_aperture_sz: invalid value ignored.\n");
	}

	if (dom0_gm_sz < dom0_aperture_sz)
		dom0_gm_sz = dom0_aperture_sz;
	return 1;
}
__setup("dom0_aperture_sz=", dom0_aperture_sz_setup);

static int __init dom0_gm_sz_setup(char *str)
{
	int t;
	if (sscanf(str, "%d", &t) == 1 && t > 0 && t <= 2048)
		dom0_gm_sz = t;
	else {
		printk("vGT: dom0_gm_sz: invalid value ignored.\n");
	}

	if (dom0_gm_sz < dom0_aperture_sz)
		dom0_gm_sz = dom0_aperture_sz;
	return 1;
}
__setup("dom0_gm_sz=", dom0_gm_sz_setup);

static int dom0_fence_sz = 4;
static int __init dom0_fence_sz_setup(char *str)
{
	int t;
	if (sscanf(str, "%d", &t) == 1 && t > 0 && t <= 16)
		dom0_fence_sz = t;
	else {
		printk("vGT: dom0_fence_sz: invalid value ignored.\n");
	}

	return 1;
}
__setup("dom0_fence_sz=", dom0_fence_sz_setup);

/* Before using the bitmap to manage the allocation of GM space dynamically
 * (hence Dom0's aperture starts at 0 of GM space, we used static fixed
 * allocation and Dom0's aperture starts at 128MB of GM space.
 * If you want to switch to the old 128MB location anyway, enable this kernel
 * parameter.
 */
static int dom0_aperture_starts_at_128MB;
static int __init dom0_aperture_starts_at_128MB_setup(char *str)
{
	dom0_aperture_starts_at_128MB = 1;
	return 1;
}
__setup("dom0_aperture_starts_at_128MB", dom0_aperture_starts_at_128MB_setup);

int vgt_ctx_switch = 1;

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
void show_debug(struct pgt_device *pdev, int ring_id)
{
	vgt_reg_t reg;
	struct vgt_device *vgt_dom1 = default_device.device[1];

	if (vgt_dom1) {
		vgt_show_irq_state(vgt_dom0);
		vgt_show_irq_state(vgt_dom1);
		printk("DERRMR: %x\n", VGT_MMIO_READ(pdev, 0x44050));
		printk("VBLANK_A: %x\n", VGT_MMIO_READ(pdev, _REG_VBLANK_A));
		printk("VBLANK_B: %x\n", VGT_MMIO_READ(pdev, _REG_VBLANK_B));
	}

	printk("debug registers(ring-%d),reg maked with <*> may not apply to every ring):\n", ring_id);
	printk("....EIR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_RCS_EIR));
	printk("....EMR: %x\n", VGT_MMIO_READ(pdev, _REG_RCS_EMR));
	printk("....ESR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_RCS_ESR));
	printk("....blit EIR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_BCS_EIR));
	printk("....blit ESR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_BCS_ESR));
	printk("....IPEIR(last executed inst): %x\n", VGT_MMIO_READ(pdev, 0x2064 + 0x10000*ring_id));
	printk("....IPEHR(last executed inst): 0x%x\n", VGT_MMIO_READ(pdev, 0x2068 + 0x10000*ring_id));
	reg = VGT_MMIO_READ(pdev, 0x2070 + 0x10000*ring_id);
	printk("....INSTPS* (parser state): 0x%x :\n", reg);
	printk("....ACTHD(active header): 0x%x\n", VGT_MMIO_READ(pdev, 0x2074 + 0x10000*ring_id));
	printk("....UHPTR(pending header): %x\n", VGT_MMIO_READ(pdev, _REG_RCS_UHPTR));
	printk("....DMA_FADD_P(current fetch DMA): 0x%x\n", VGT_MMIO_READ(pdev, 0x2078 + 0x10000*ring_id));
	printk("....CSCMDOP* (instruction DWORD): 0x%x\n", VGT_MMIO_READ(pdev, 0x220C + 0x10000*ring_id));
	printk("....CSCMDVLD* (command buffer valid): 0x%x\n", VGT_MMIO_READ(pdev, 0x2210 + 0x10000*ring_id));
	printk("(informative)\n");
	printk("....INSTDONE_1(FYI)*: 0x%x\n", VGT_MMIO_READ(pdev, 0x206C + 0x10000*ring_id));
	printk("....INSTDONE_2*: 0x%x\n", VGT_MMIO_READ(pdev, 0x207C + 0x10000*ring_id));
}

/*
 * Show some global register settings, if we care about bits
 * in those registers.
 *
 * normally invoked from initialization phase, and mmio emulation
 * logic
 */
static void show_mode_settings(struct pgt_device *pdev)
{
	vgt_reg_t val;
	struct vgt_device *vgt1 = default_device.device[1];

	if (current_render_owner(pdev))
		printk("Current render owner: %d\n", current_render_owner(pdev)->vgt_id);

#define SHOW_MODE(reg)		\
	do{				\
		val = VGT_MMIO_READ(pdev, reg);	\
		printk("vGT: "#reg"(%x): p(%x), 0(%x), 1(%x)\n",	\
			reg, val, __sreg(vgt_dom0, reg), vgt1 ? __sreg(vgt1, reg) : 0);	\
	} while (0);
	SHOW_MODE(_REG_RCS_MI_MODE);
	SHOW_MODE(_REG_VCS_MI_MODE);
	SHOW_MODE(_REG_BCS_MI_MODE);
	SHOW_MODE(_REG_GFX_MODE);
	SHOW_MODE(_REG_ARB_MODE);
	SHOW_MODE(_REG_GT_MODE);
	SHOW_MODE(_REG_RCS_INSTPM);
	SHOW_MODE(_REG_VCS_INSTPM);
	SHOW_MODE(_REG_BCS_INSTPM);
	SHOW_MODE(_REG_CACHE_MODE_0);
	SHOW_MODE(_REG_CACHE_MODE_1);
	SHOW_MODE(_REG_TILECTL);
	if (pdev->is_ivybridge) {
		SHOW_MODE(_REG_RCS_GFX_MODE_IVB);
		SHOW_MODE(_REG_BCS_BLT_MODE_IVB);
		SHOW_MODE(_REG_VCS_MFX_MODE_IVB);
	}
}

#if 0
/*
 * Show the content of a saved render context
 */
static void show_context(struct vgt_device *vgt, uint64_t context, bool clobber)
{
	struct pgt_device *pdev = vgt->pdev;
	uint64_t ptr;
	u32 *vptr;
	int i;

	/* GM is not trapped. So safe to access it directly */
	ptr = (uint64_t)phys_aperture_vbase(pdev) + context;
	printk("===================\n");
	printk("Context-vgt%d (%llx, %llx): %s\n", vgt->vgt_id, context, ptr, clobber ? "clobbered" : "");

	vptr = (u32 *)ptr;
	if (clobber) {
		printk("Clobber the context!\n");
		*vptr = 0x12345678;
		*(vptr + 0x4A) = 0x12345678;	// CACHE_MODE_0
		*(vptr + 0x4D)= 0x12345678;	// INSTPM
		*(vptr + 0x4F)= 0x12345678;	// MI_MODE
		*(vptr + 0x5C)= 0x12345678;	// TIMESTAMP
		*(vptr + 0x70)= 0x12345678;	// STATE_SIP
	}
	for (i = 0; i < 0x100; i++) {
		if (!(i % 8))
			printk("\n[%03x]:", i);
		printk(" %8x", *(vptr + i));
	}
	printk("\n");
	printk("===================\n");
}
#endif

/*
 * TODO: the context layout could be different on generations.
 * e.g. ring head/tail, ccid, etc. when PPGTT is enabled
 */
#define OFF_CACHE_MODE_0       0x4A
#define OFF_CACHE_MODE_1       0x4B
#define OFF_INSTPM             0x4D
#define OFF_EXCC               0x4E
#define OFF_MI_MODE            0x4F
static void update_context(struct vgt_device *vgt, uint64_t context)
{
	struct pgt_device *pdev = vgt->pdev;
	uint64_t ptr;
	u32 *vptr;

	ptr = (uint64_t)phys_aperture_vbase(pdev) + context;
	vptr = (u32 *)ptr;
#define UPDATE_FIELD(off, reg) \
	*(vptr + off) = 0xFFFF0000 | (__sreg(vgt, reg) & 0xFFFF);

	UPDATE_FIELD(OFF_CACHE_MODE_0, _REG_CACHE_MODE_0);
	UPDATE_FIELD(OFF_CACHE_MODE_1, _REG_CACHE_MODE_1);
	UPDATE_FIELD(OFF_INSTPM, _REG_RCS_INSTPM);
	UPDATE_FIELD(OFF_EXCC, _REG_RCS_EXCC);
	UPDATE_FIELD(OFF_MI_MODE, _REG_RCS_MI_MODE);
}

struct vgt_device *next_display_owner;
atomic_t display_switched = ATOMIC_INIT(0);
struct vgt_device *vgt_dom0;
struct mmio_hash_table	*mtable[MHASH_SIZE];
struct mmio_hash_table gtt_mmio_handler={
	.read = gtt_mmio_read,
	.write = gtt_mmio_write,
	.mmio_base = 0x200000,
};

static void vgt_hash_add_mtable(struct vgt_device *vgt, int table, int index, struct mmio_hash_table *mht)
{
	struct mmio_hash_table **t = NULL;

	switch(table) {
	case VGT_HASH_MMIO:
		t = &mtable[index];
		break;
	case VGT_HASH_WP_PAGE:
		t = &vgt->wp_table[index];
		break;
	}

	if (!*t) {
		*t = mht;
		mht->next = NULL;
	} else {
		mht->next = *t;
		*t = mht;
	}
}

struct mmio_hash_table *vgt_hash_lookup_mtable(struct vgt_device *vgt, int table, int item)
{
	int index;
	struct mmio_hash_table *mht = NULL;

	index = mhash(item);

	switch(table) {
	case VGT_HASH_MMIO:
		if (item >= gtt_mmio_handler.mmio_base)
			return &gtt_mmio_handler;
		mht = mtable[index];
		item &= ~3;
		break;
	case VGT_HASH_WP_PAGE:
		mht = vgt->wp_table[index];
		break;
	}

	while (mht) {
		if (mht->mmio_base == item)
			return mht;
		else
			mht = mht->next;
	}
	return NULL;
}

static void free_mtable_chain(struct mmio_hash_table *chain)
{
	if (!chain) {
		free_mtable_chain(chain->next);
		kfree(chain);
	}
}

void vgt_hash_free_mtable(struct vgt_device *vgt, int table)
{
	int i;

	if (table == VGT_HASH_MMIO) {
		for (i = 0; i < MHASH_SIZE; i++)
			free_mtable_chain(mtable[i]);
		memset(mtable, 0, sizeof(mtable));
	} else if (table == VGT_HASH_WP_PAGE) {
		for (i = 0; i < MHASH_SIZE; i++)
			free_mtable_chain(vgt->wp_table[i]);
		memset(vgt->wp_table, 0, sizeof(vgt->wp_table));
	}
}

void vgt_hash_register_entry(struct vgt_device *vgt, int table, struct mmio_hash_table *mht)
{
	int index = mhash(mht->mmio_base);

	if (table == VGT_HASH_MMIO)
		ASSERT ((mht->mmio_base & 3) == 0);
	vgt_hash_add_mtable(vgt, table, index, mht);
}

void vgt_hash_remove_entry(struct vgt_device *vgt, int table, int key)
{
	int index;
	struct mmio_hash_table *mht, *p;

	index = mhash(key);
	mht = NULL;

	switch(table) {
	case VGT_HASH_MMIO:
		mht = mtable[index];
		break;
	case VGT_HASH_WP_PAGE:
		mht = vgt->wp_table[index];
		break;
	}

	p = mht;
	while(mht) {
		if (mht->mmio_base == key) {
			p = mht->next;
			kfree(mht);
			break;
		} else {
			p = mht;
			mht = mht->next;
		}
	}
}

#if 0
static void _add_mtable(int index, struct mmio_hash_table *mht)
{
	vgt_hash_add_mtable(NULL, VGT_HASH_MMIO, index, mht);
}
#endif

static struct mmio_hash_table *lookup_mtable(int mmio_base)
{
	return vgt_hash_lookup_mtable(NULL, VGT_HASH_MMIO, mmio_base);
}

static void free_mtable(void)
{
	vgt_hash_free_mtable(NULL, VGT_HASH_MMIO);
}

static void register_mhash_entry(struct mmio_hash_table *mht)
{
	vgt_hash_register_entry(NULL, VGT_HASH_MMIO, mht);
}

bool vgt_register_mmio_handler(int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write)
{
	int i, end;
	struct mmio_hash_table *mht;

	end = start + bytes -1;

	printk("start=0x%x end=0x%x\n", start, end);

	ASSERT((start & 3) == 0);
	ASSERT(((end+1) & 3) == 0);

	for ( i = start; i < end; i += 4 ) {
		mht = kmalloc(sizeof(*mht), GFP_KERNEL);
		if (mht == NULL) {
			printk("Insufficient memory in %s\n", __FUNCTION__);
			free_mtable();
			return false;
		}
		mht->mmio_base = i;
		mht->read = read;
		mht->write = write;
		register_mhash_entry(mht);
	}
	return true;
}

/*
 * bitmap of allocated vgt_ids.
 * bit = 0 means free ID, =1 means allocated ID.
 */
unsigned long vgt_id_alloc_bitmap;

/*TODO: may need lock to protect default_deivce */
struct vgt_device *vmid_2_vgt_device(int vmid)
{
    unsigned int bit;
    struct vgt_device *vgt;
    /* TODO: check if vgt_id_alloc_bitmap is ~0UL */
    for_each_set_bit(bit, &vgt_id_alloc_bitmap, (8 * sizeof(unsigned long))) {
        vgt = default_device.device[bit];
        if (vgt->vm_id == vmid)
            return vgt;
    }
    return NULL;
}

int allocate_vgt_id(void)
{
	unsigned long bit_index;

	do {
		bit_index = ffz (vgt_id_alloc_bitmap);
		if (bit_index >= VGT_MAX_VMS) {
			printk("vGT: allocate_vgt_id() failed\n");
			return -ENOSPC;
		}
	} while (test_and_set_bit(bit_index, &vgt_id_alloc_bitmap) != 0);

	return bit_index;
}

void free_vgt_id(int vgt_id)
{
	ASSERT(vgt_id >= 0 && vgt_id < VGT_MAX_VMS);
	ASSERT(vgt_id_alloc_bitmap & (1UL << vgt_id));
	clear_bit(vgt_id, &vgt_id_alloc_bitmap);
}

/*
 * Guest to host GMADR (include aperture) converting.
 *
 * handle in 4 bytes granule
 */
vgt_reg_t mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t g_value)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t h_value;
	vgt_reg_t mask;

	if (!reg_addr_fix(pdev, reg))
		return g_value;

	ASSERT((reg < _REG_FENCE_0_LOW) || (reg > _REG_FENCE_15_HIGH));

	mask = pdev->vgt_addr_table[reg_addr_index(pdev, reg)];
	dprintk("vGT: address fix g->h for reg (0x%lx) value (0x%x) mask (0x%x)\n", reg, g_value, mask);
	/*
	 * NOTE: address ZERO is special, and sometimes the driver may hard
	 * code address ZERO, e.g. in curbase setting (when the cursor becomes
	 * invisible). So we always translate address ZERO into the valid
	 * range of the VM. If this doesn't work, we need change the driver!
	 */
	if (!(g_value & mask)) {
		dprintk("vGT(%d): translate address ZERO for reg (%lx)\n",
			vgt->vgt_id, reg);
		g_value = (vgt_guest_visible_gm_base(vgt) & mask) |
			  (g_value & ~mask);
	}
	/* FIXME: there may have some complex mask pattern */
	h_value = g2h_gm(vgt, g_value & mask);
	dprintk("....(g)%x->(h)%x\n", g_value, (h_value & mask) | (g_value & ~mask));

	return (h_value & mask) | (g_value & ~mask);
}

/*
 * Host to guest GMADR (include aperture) converting.
 *
 * handle in 4 bytes granule
 */
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t g_value;
	vgt_reg_t mask;

	if (!reg_addr_fix(pdev, reg))
		return h_value;

	dprintk("vGT: address fix h->g for reg (%lx)(%x)\n", reg, h_value);
	mask = pdev->vgt_addr_table[reg_addr_index(pdev, reg)];

	/*
	 * it's possible the initial state may not contain a valid address
	 * vm's range. In such case fake a valid address since the value there
	 * doesn't matter.
	 */
	if (!h_gm_is_visible(vgt, h_value & mask) && !h_gm_is_hidden(vgt, h_value & mask)) {
		dprintk("!!!vGT: reg (%lx) doesn't contain a valid host address (%x)\n", reg, h_value);
		h_value = (vgt_visible_gm_base(vgt) & mask) | (h_value & ~mask);
	}

	/* FIXME: there may have some complex mask pattern */
	g_value = h2g_gm(vgt, h_value & mask);
	dprintk("....(h)%x->(g)%x\n", h_value, (g_value & mask) | (h_value & ~mask));
	return (g_value & mask) | (h_value & ~mask);
}

#if 0
/* show Linux specific seqno fields for all ringbuffers */
static void show_seqno(struct pgt_device *pdev)
{
	char *p_contents;
	vgt_reg_t addr;

	addr = VGT_MMIO_READ(pdev, _REG_RCS_HWS_PGA);
	p_contents = phys_aperture_vbase(pdev) + addr;
	printk("RCS HWS PGA: %x,%x,%x isr: %x, index: %x, seqno: %d\n",
		addr, __vreg(vgt_dom0, _REG_RCS_HWS_PGA),
		__sreg(vgt_dom0, _REG_RCS_HWS_PGA), *(u32*)(p_contents),
		*(u32*)(p_contents + 0x20 * 4), *(u32*)(p_contents + 0x21 * 4));

	addr = VGT_MMIO_READ(pdev, _REG_VCS_HWS_PGA);
	p_contents = phys_aperture_vbase(pdev) + addr;
	printk("RCS HWS PGA: %x,%x,%x isr: %x, index: %x, seqno: %d\n",
		addr, __vreg(vgt_dom0, _REG_VCS_HWS_PGA),
		__sreg(vgt_dom0, _REG_VCS_HWS_PGA), *(u32*)(p_contents),
		*(u32*)(p_contents + 0x20 * 4), *(u32*)(p_contents + 0x21 * 4));

	addr = VGT_MMIO_READ(pdev, _REG_BCS_HWS_PGA);
	p_contents = phys_aperture_vbase(pdev) + addr;
	printk("RCS HWS PGA: %x,%x,%x isr: %x, index: %x, seqno: %d\n",
		addr, __vreg(vgt_dom0, _REG_BCS_HWS_PGA),
		__sreg(vgt_dom0, _REG_BCS_HWS_PGA), *(u32*)(p_contents),
		*(u32*)(p_contents + 0x20 * 4), *(u32*)(p_contents + 0x21 * 4));
}
#endif

/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
void show_ringbuffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	vgt_reg_t p_tail, p_head, p_start;
	char *p_contents;
	int i;
	struct vgt_device *vgt = current_render_owner(pdev);
	u32* cur;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	p_start = VGT_MMIO_READ(pdev, RB_START(pdev, ring_id));
	printk("ring buffer(%d): head (%x) tail(%x), start(%x)\n", ring_id,
		p_head, p_tail, p_start);
	printk("psmi idle:(%d), mi_mode idle:(%d)\n",
		VGT_MMIO_READ(pdev, pdev->ring_psmi[ring_id]) & _REGBIT_PSMI_IDLE_INDICATOR,
		VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_RINGS_IDLE);

	p_head &= RB_HEAD_OFF_MASK;
	p_contents = phys_aperture_vbase(pdev) + p_start + p_head;
	printk("p_contents(%lx)\n", (unsigned long)p_contents);
	/* FIXME: consider wrap */
	for (i = -(bytes/4); i < bytes/4; i++) {
		if (!(i % 8))
			printk("\n[%08x]:", p_head + i * 4);
		printk(" %8x", *((u32*)p_contents + i));
		if (!i)
			printk("(*)");
	}
	printk("\n");

	cur = (u32*)p_contents - 2;
	if ((*cur & 0xfffff000) == 0x18800000 && vgt) {
		u32 val, h_val;
		u64 mfn;
		int rc;
		extern int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val);

		cur++;
		printk("Hang in batch buffer (%x)\n", *cur);
		val = vgt->vgtt[GTT_INDEX(pdev, *cur)];
		printk("vGTT: %x\n", val);
		rc = gtt_p2m(vgt, val, &h_val);
		if (rc < 0) {
			printk("failed to translate\n");
		} else {
			mfn = gtt_pte_get_pfn((gtt_pte_t *)&h_val);
			printk("MACH: %x %llx\n", h_val, mfn);
		}
		printk("Actual pGTT: %x\n",
			vgt_read_gtt(pdev, GTT_INDEX(pdev, *cur)));
	}
}

static inline unsigned long vgt_get_passthrough_reg(struct vgt_device *vgt,
		unsigned int reg)
{
	__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	return __vreg(vgt, reg);
}

static unsigned long vgt_get_reg(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_update(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg(vgt, reg);
	else
		return __vreg(vgt, reg);
}

static inline unsigned long vgt_get_passthrough_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	__sreg64(vgt, reg) = VGT_MMIO_READ_BYTES(vgt->pdev, reg, 8);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	__vreg(vgt, reg + 4) = mmio_h2g_gmadr(vgt, reg + 4, __sreg(vgt, reg + 4));
	return __vreg64(vgt, reg);
}
/*
 * for 64bit reg access, we split into two 32bit accesses since each part may
 * require address fix
 *
 * TODO: any side effect with the split? or instead install specific handler
 * for 64bit regs like fence?
 */
static unsigned long vgt_get_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_update(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg_64(vgt, reg);
	else
		return __vreg64(vgt, reg);
}

static int display_pointer_id = 0;
void vgt_set_display_pointer(int vm_id)
{
	struct vgt_device *vgt = vmid_2_vgt_device(vm_id);

	if (!vgt) {
		printk("vGT: invalid vm_id (%d)\n", vm_id);
		return;
	}

	VGT_MMIO_WRITE(vgt->pdev, _REG_DSPASURF, __sreg(vgt, _REG_DSPASURF));
	VGT_MMIO_WRITE(vgt->pdev, _REG_CURABASE, __sreg(vgt, _REG_CURABASE));
	printk("vGT: set display to vgt(%d) with (%x, %x)\n", vm_id,
		__sreg(vgt, _REG_DSPASURF), __sreg(vgt, _REG_CURABASE));
	display_pointer_id = vm_id;
}

ssize_t vgt_get_display_pointer(char *buf)
{
	struct vgt_device *vgt = vmid_2_vgt_device(display_pointer_id);

	return sprintf(buf, "Current pointer: id [%d] sReg[%x,%x] pReg[%x,%x]\n",
			display_pointer_id,
			__sreg(vgt, _REG_DSPASURF), __sreg(vgt, _REG_CURABASE),
			VGT_MMIO_READ(vgt->pdev, _REG_DSPASURF),
			VGT_MMIO_READ(vgt->pdev, _REG_CURABASE));
}

static void vgt_update_reg(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	if (reg == _REG_DSPASURF)
		dprintk("%s: =======: write vReg(%x), sReg(%x)\n", __func__, __vreg(vgt, reg), __sreg(vgt, reg));
	if (reg_hw_access(vgt, reg))
		VGT_MMIO_WRITE(pdev, reg, __sreg(vgt, reg));
}

static void vgt_update_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	__sreg(vgt, reg + 4) = mmio_g2h_gmadr(vgt, reg + 4, __vreg(vgt, reg + 4));
	if (reg_hw_access(vgt, reg))
			VGT_MMIO_WRITE_BYTES(pdev, reg, __sreg64(vgt, reg), 8);
}

static void vgt_gen6_force_wake(struct pgt_device *pdev)
{
	int count = 0;

	if (VGT_MMIO_READ(pdev, _REG_FORCEWAKE) == 0) {
		VGT_MMIO_WRITE(pdev, _REG_FORCEWAKE, 1);
		while (count < 50 && !(VGT_MMIO_READ(pdev, _REG_FORCEWAKE_ACK) & 1))
			count++;
		printk("vGT: 1st forcewake set to %d(%x)\n",
			VGT_MMIO_READ(pdev, _REG_FORCEWAKE),
			VGT_MMIO_READ(pdev, _REG_FORCEWAKE_ACK));
	}
}

static void vgt_gen6_mul_force_wake(struct pgt_device *pdev)
{
	int count = 0;

	while (count < 50 && (VGT_MMIO_READ(pdev, _REG_MUL_FORCEWAKE_ACK) & 1))
		count++;

	VGT_MMIO_WRITE(pdev, _REG_MUL_FORCEWAKE, (1 | (1 << 16)));
	(void)VGT_MMIO_READ(pdev, _REG_MUL_FORCEWAKE);

	count = 0;

	while (count < 50 && !(VGT_MMIO_READ(pdev, _REG_MUL_FORCEWAKE_ACK) & 1))
		count++;
}

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_reg(vgt, reg);
	} else {
		wvalue = vgt_get_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

bool default_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	memcpy((char *)vgt->state.vReg + offset,
			p_data, bytes);

	offset &= ~(bytes - 1);
	if (bytes <= 4)
		vgt_update_reg(vgt, offset);
	else
		vgt_update_reg_64(vgt, offset);

	return true;
}

bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_passthrough_reg(vgt, reg);
	} else {
		wvalue = vgt_get_passthrough_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

#define PCI_BAR_ADDR_MASK (~0xFUL)  /* 4 LSB bits are not address */

static inline unsigned int vgt_pa_to_mmio_offset(struct vgt_device *vgt, unsigned long pa)
{
	return (vgt->vm_id == 0)?
		pa - vgt->pdev->gttmmio_base :
		pa - ( (*(uint64_t*)(vgt->state.cfg_space + VGT_REG_CFG_SPACE_BAR0))
				& PCI_BAR_ADDR_MASK );
}

/*
 * Emulate the VGT MMIO register read ops.
 * Return : true/false
 * */
bool vgt_emulate_read(struct vgt_device *vgt, unsigned int pa, void *p_data,int bytes)
{
	struct mmio_hash_table *mht;
	struct pgt_device *pdev = vgt->pdev;
	unsigned int offset;
	unsigned long flags;

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	/* for single-VM UP dom0 case, no nest is expected */
	ASSERT(!spin_is_locked(&pdev->lock));

//	ASSERT (offset + bytes <= vgt->state.regNum *
//				sizeof(vgt->state.vReg[0]));
	ASSERT (bytes <= 8);
//	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);
	if (!VGT_REG_IS_ALIGNED(offset, bytes)){
		printk("unaligned reg %x, bytes=%d\n", offset, bytes);
		offset = VGT_REG_ALIGN(offset, bytes);
	}

	if (bytes > 4)
		dprintk("vGT: capture >4 bytes read to %x\n", offset);

	spin_lock_irqsave(&pdev->lock, flags);

	pdev->dev_func.force_wake(pdev);

	mht = lookup_mtable(offset);
	if ( mht && mht->read )
		mht->read(vgt, offset, p_data, bytes);
	else {
		default_mmio_read(vgt, offset, p_data, bytes);
	}

#if 0
	if (vgt->vm_id) {
		if (VGT_MMIO_READ(pdev, offset) != *(vgt_reg_t *)p_data)
			printk("vGT: read reg(%x), p(%x), v(%x)\n",
				offset, VGT_MMIO_READ(pdev, offset), *(vgt_reg_t *)p_data);
	}
#endif
	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

/*
 * TODO: most mode ctl registers are render specific. In such case
 * we only need to ensure the mask bits set correctly when doing the
 * save/restore. On the other hand, need study later whether some
 * ctl registers may have wider impact e.g. on both render and
 * display. That would be tricky.
 *
 * Also, there may have more such registers in newer generations.
 */
vgt_reg_t vgt_mode_ctl_regs[] = {
	_REG_GFX_MODE,
	_REG_ARB_MODE,

	_REG_RCS_MI_MODE,
	_REG_VCS_MI_MODE,
	_REG_BCS_MI_MODE,

	_REG_RCS_INSTPM,
	_REG_VCS_INSTPM,
	_REG_BCS_INSTPM,

	_REG_GT_MODE,
	_REG_CACHE_MODE_0,
	_REG_CACHE_MODE_1,
};

vgt_reg_t vgt_mode_mask_regs[16] = {0};

/* FIXME: need a better way to handle this generation difference */
vgt_reg_t vgt_gen7_mode_ctl_regs[] = {
	_REG_BCS_MI_MODE,
	_REG_BCS_BLT_MODE_IVB,
	_REG_RCS_MI_MODE,
	_REG_RCS_GFX_MODE_IVB,
	_REG_VCS_MI_MODE,
	_REG_VCS_MFX_MODE_IVB,
};

/*
 * Emulate the VGT MMIO register write ops.
 * Return : true/false
 * */
bool vgt_emulate_write(struct vgt_device *vgt, unsigned int pa,
	void *p_data, int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct mmio_hash_table *mht;
	unsigned int offset;
	unsigned long flags;
	vgt_reg_t old_vreg=0, old_sreg=0;

	/* XXX PPGTT PTE WP comes here too. */
	if (pdev->enable_ppgtt) {
		mht = vgt_hash_lookup_mtable(vgt, VGT_HASH_WP_PAGE, pa >> PAGE_SHIFT);
		if (mht && mht->write) {
			/* XXX lock? */
			mht->write(vgt, pa, p_data, bytes);
			return true;
		}
	}

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	ASSERT(!spin_is_locked(&pdev->lock));
//	ASSERT (offset + bytes <= vgt->state.regNum *
//				sizeof(vgt->state.vReg[0]));
	/* at least FENCE registers are accessed in 8 bytes */
	ASSERT (bytes <= 8);
	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);

	if (bytes > 4)
		dprintk("vGT: capture >4 bytes write to %x with val (%lx)\n", offset, *(unsigned long*)p_data);
/*
	if (reg_rdonly(pdev, offset & (~(bytes - 1)))) {
		printk("vGT: captured write to read-only reg (%x)\n", offset);
		return true;
	}
*/

	spin_lock_irqsave(&pdev->lock, flags);

	if (offset < VGT_MMIO_REG_NUM && reg_mode_ctl(pdev, offset)) {
		old_vreg = __vreg(vgt, offset);
		old_sreg = __sreg(vgt, offset);
	}

	pdev->dev_func.force_wake(pdev);

	mht = lookup_mtable(offset);
	if ( mht && mht->write )
		mht->write(vgt, offset, p_data, bytes);
	else {
		default_mmio_write(vgt, offset, p_data, bytes);
	}

	if (offset == _REG_DSPASURF || offset == _REG_DSPBSURF) {
		dprintk("vGT(%d): write to surface base (%x) with (%x), pReg(%x)\n",
			vgt->vgt_id, offset, __vreg(vgt, offset),
			VGT_MMIO_READ(pdev, offset));
		/* update live reg as vm may wait on the update */
		if (!reg_hw_access(vgt, offset)) {
			__vreg(vgt, offset + 0x10) = __vreg(vgt, offset);
			__sreg(vgt, offset + 0x10) = __sreg(vgt, offset);
		}
	}

	/* higher 16bits of mode ctl regs are mask bits for change */
	if (offset < VGT_MMIO_REG_NUM && reg_mode_ctl(pdev, offset)) {
		u32 mask = __vreg(vgt, offset) >> 16;
		int j;

		for (j = 0; j < ARRAY_NUM(vgt_mode_ctl_regs); j++) {
			if (vgt_mode_ctl_regs[j] == offset)
				break;
		}
		dprintk("old mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset), vgt_mode_mask_regs[j]);
		/*
		 * share the global mask among VMs, since having one VM touch a bit
		 * not changed by another VM should be still saved/restored later
		 */
		if (j != ARRAY_NUM(vgt_mode_ctl_regs))
			vgt_mode_mask_regs[j] |= mask << 16;
		__vreg(vgt, offset) = (old_vreg & ~mask) | (__vreg(vgt, offset) & mask);
		__sreg(vgt, offset) = (old_sreg & ~mask) | (__sreg(vgt, offset) & mask);
		dprintk("new mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset), vgt_mode_mask_regs[j]);
		//show_mode_settings(vgt->pdev);
	}

	if (offset == _REG_RCS_UHPTR)
		printk("vGT: write to UHPTR (%x,%x)\n", __vreg(vgt, offset), __sreg(vgt, offset));


	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

bool is_rendering_engine_empty(struct pgt_device *pdev, int ring_id)
{
	if ( is_ring_enabled(pdev, ring_id) && !is_ring_empty(pdev, ring_id) )
		return false;

	/*
	 * FIXME: it turns out that psmi idle status bit check may not be
	 * always true when both dom0/Linux VM runs glxgears in parallel. Not
	 * sure the reason yet. So disable this check for now, but need revise
	 * in the future
	 */
#if 0
	if (!(VGT_MMIO_READ(pdev, pdev->ring_psmi[ring_id]) & _REGBIT_PSMI_IDLE_INDICATOR))
		return false;
#endif

	if (!(VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_RINGS_IDLE))
		return false;

	return true;
}

bool is_context_switch_done(struct pgt_device *pdev, int ring_id)
{
	u32 *ptr;

	ptr = (u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev));
	if (*ptr != pdev->magic)
		return false;

	return true;
}

/*
 * Wait for the empty of RB.
 * TODO: Empty of RB doesn't mean the commands are retired. May need a STORE_IMM
 * after MI_FLUSH, but that needs our own hardware satus page.
 */
static bool ring_wait_for_empty(struct pgt_device *pdev, int ring_id, bool ctx_switch, char *str)
{
	static u64 max = 200;
	u64 count = 0;

	/* wait to be completed */
	while (true) {
		if (ctx_switch && is_context_switch_done(pdev, ring_id))
			break;
		if (!ctx_switch && is_rendering_engine_empty(pdev, ring_id))
			break;
		sleep_us(1);		/* 1us delay */
		count++;
		if (!(count % 10000000)) {
			printk("vGT(%s): wait %lld seconds for ring(%d)\n",
				str, count / 1000000, ring_id);

			printk("vGT-cur(%d): head(%x), tail(%x), start(%x)\n",
				current_render_owner(pdev)->vgt_id,
				current_render_owner(pdev)->rb[ring_id].sring.head,
				current_render_owner(pdev)->rb[ring_id].sring.tail,
				current_render_owner(pdev)->rb[ring_id].sring.start);
			printk("vGT-dom0(%d): head(%x), tail(%x), start(%x)\n",
				vgt_dom0->vgt_id,
				vgt_dom0->rb[ring_id].sring.head,
				vgt_dom0->rb[ring_id].sring.tail,
				vgt_dom0->rb[ring_id].sring.start);
			show_debug(pdev, ring_id);
			show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		}
	}

	if (count > 2000 || count > max)
		printk("vGT(%s): ring (%d) has timeout (%lldus), max(%lldus)\n",
			ctx_switch ? "ctx-switch" : "wait-empty",
			ring_id, count, max);

	if (count > max)
		max = count;

	return true;
}

bool is_rendering_engines_empty(struct pgt_device *pdev, int *ring_id)
{
	int i;

	/*
	 * TODO: timeout for 3 engines are not synchronous. Need suspend
	 * command parser later
	 */
	for (i=0; i < pdev->max_engines; i++) {
		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		if ( !ring_wait_for_empty(pdev, i, false, "wait-empty") ) {
			*ring_id = i;
			return false;
		}
	}
	return true;
}

/*
 * Request from user level daemon/IOCTL
 */
void vgt_request_display_owner_switch(struct vgt_device *vgt)
{
	if (next_display_owner != current_display_owner(vgt->pdev))
		next_display_owner = vgt;
}

/*
 * Do monitor owner switch.
 */
void vgt_switch_display_owner(struct vgt_device *prev,
    struct vgt_device *next)
{
    vgt_save_state(prev);
    vgt_restore_state(next);
}

void do_vgt_display_switch(struct pgt_device *pdev)
{
	unsigned long flags;
    //struct vgt_device *cur, *pre;
	printk(KERN_WARNING"xuanhua: vGT: display switched\n");
	printk(KERN_WARNING"xuanhua: vGT: current display owner: %p; next display owner: %p\n",
			current_display_owner(pdev), next_display_owner);

	/* TODO: Because of assert in vgt_emulate_write/read,
	 * we cannot use this lock in case deadlock */
	/* FIXME: we do need other locks for this ??? */
	spin_lock_irqsave(&pdev->lock, flags);
	dprintk("before irq save\n");
	vgt_irq_save_context(current_display_owner(pdev),
			VGT_OT_DISPLAY);
	dprintk("after irq save\n");
	vgt_switch_display_owner(current_display_owner(pdev),
			next_display_owner);
	previous_display_owner(pdev) = current_display_owner(pdev);
	current_display_owner(pdev) = next_display_owner;
	//next_display_owner = NULL;
	dprintk("before irq restore\n");
	vgt_irq_restore_context(next_display_owner, VGT_OT_DISPLAY);
	spin_unlock_irqrestore(&pdev->lock, flags);
	dprintk("after irq restore\n");
	/*
	 * Virtual interrupts pending right after display switch
	 * Need send to both prev and next owner.
	 */
	if (pdev->request & VGT_REQUEST_IRQ) {
		printk("vGT: handle pending interrupt in the display context switch time\n");
		spin_lock_irqsave(&pdev->lock, flags);
		clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
		vgt_handle_virtual_interrupt(pdev, VGT_OT_DISPLAY);
		spin_unlock_irqrestore(&pdev->lock, flags);
	}
#if 0
    cur = next_display_owner;
    pre = current_display_owner(pdev);
    if (cur == pre)
        return;
    else {
        current_display_owner(pdev) = cur;
        previous_display_owner(pdev) = pre;
    }

    /* double buffered */
    VGT_MMIO_WRITE(cur->pdev, _REG_DSPASURF, __sreg(cur, _REG_DSPASURF));
    VGT_MMIO_WRITE(cur->pdev, _REG_DSPASURF, __sreg(cur, _REG_DSPASURF));
    printk("XXXX: display switch to dom %d\n", cur->vgt_id);
#endif
}

static struct vgt_device *next_vgt(
	struct list_head *head, struct vgt_device *vgt)
{
	struct list_head *next;

	next = vgt->list.next;
	/* wrap the list */
	if (next == head)
		next = head->next;
	return list_entry(next, struct vgt_device, list);
}

/* update the tail pointer after all the context is restored */
static void vgt_resume_ringbuffers(struct vgt_device *vgt)
{
	int i;

	for (i = 0; i < vgt->pdev->max_engines; i++) {
		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		if (!(vgt->rb[i].sring.ctl & _RING_CTL_ENABLE)) {
			printk("vGT: ring (%d) not enabled. exit resume\n", i);
			continue;
		}
		VGT_MMIO_WRITE(vgt->pdev, RB_TAIL(vgt->pdev, i), vgt->rb[i].sring.tail);
	}
}

/*
 * random GTT entry check
 */
static void check_gtt(struct pgt_device *pdev)
{
	static unsigned int addr[] = {
	0x00000000, 0x02000000, 0x04000000, 0x08000000,
	0x0C000000, 0x0FFFF000, 0x10000000, 0x20000000,
	0x40000000, 0x60000000, 0x7FFFF000 };

	int i;

	for (i = 0; i < ARRAY_SIZE(addr); i++)
		printk("GMADR: 0x08%x, GTT INDEX: %x, GTT VALUE: %x\n",
			addr[i], GTT_INDEX(pdev, addr[i]),
			vgt_read_gtt(pdev, GTT_INDEX(pdev, addr[i])));
}

void vgt_toggle_ctx_switch(bool enable)
{
	/*
	 * No need to hold lock as this will be observed
	 * in the next check in kthread.
	 */
	if (enable)
		vgt_ctx_switch = 1;
	else
		vgt_ctx_switch = 0;
}

static int period = HZ/5;	/* default slow mode in 200ms */

/*
 * The thread to perform the VGT ownership switch.
 *
 * We need to handle race conditions from different paths around
 * vreg/sreg/hwreg. So far there're 4 paths at least:
 *   a) the vgt thread to conduct context switch
 *   b) the GP handler to emulate MMIO for dom0
 *   c) the event handler to emulate MMIO for other VMs
 *   d) the interrupt handler to do interrupt virtualization
 *
 * Now d) is removed from the race path, because we adopt a delayed
 * injection mechanism. Physical interrupt handler only saves pending
 * IIR bits, and then wake up the vgt thread. Later the vgt thread
 * checks the pending bits to do the actual virq injection. This approach
 * allows vgt thread to handle ownership switch cleanly.
 *
 * So it's possible for other 3 paths to touch vreg/sreg/hwreg:
 *   a) the vgt thread may need to update HW updated regs into
 *      vreg/sreg of the prev owner
 *   b) the GP handler and event handler always updates vreg/sreg,
 *      and may touch hwreg if vgt is the current owner
 *      and then update vreg for interrupt virtualization
 *
 * To simplify the lock design, we make below assumptions:
 *   a) the vgt thread doesn't trigger GP fault itself, i.e. always
 *      issues hypercall to do hwreg access
 *   b) the event handler simply notifies another kernel thread, leaving
 *      to that thread for actual MMIO emulation
 *
 * Given above assumption, no nest would happen among 4 paths, and a
 * simple global spinlock now should be enough to protect the whole
 * vreg/sreg/ hwreg. In the future we can futher tune this part on
 * a necessary base.
 *
 * TODO: display switch time is long in seconds, which should be split
 * from the main thread, and also minimize its lock granularity.
 */
int vgt_thread(void *priv)
{
	struct vgt_device *next, *vgt = priv, *prev;
	struct pgt_device *pdev = vgt->pdev;
	int threshold = 2; /* print every 10s */
	long wait = 0;
	int ring_id;
	cycles_t start, end;

	//ASSERT(current_render_owner(pdev));
	printk("vGT: start kthread for dev (%x, %x)\n", pdev->bus, pdev->devfn);
	printk("vGT: dexuan: use %s style context switch\n", use_old_ctx_switch ? "old": "new");
	if (fastmode) {
		printk("vGT: fastmode switch (in 16ms)\n");
		period = HZ/64;
	}

	threshold = (10 * HZ) /period;
	wait = HZ*start_period;
	while (!kthread_should_stop()) {
		/*
		 * TODO: Use high priority task and timeout based event
		 * 	mechanism for QoS. schedule in 50ms now.
		 */
		wait = wait_event_timeout(pdev->wq, pdev->request, wait);

		if (!pdev->request && wait) {
			printk("vGT: main thread waken up by unknown reasons!\n");
			continue;
		}

		/* Handle virtual interrupt injection to current owner */
		if (test_and_clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request)) {
			spin_lock_irq(&pdev->lock);
			vgt_handle_virtual_interrupt(pdev, VGT_OT_INVALID);
			spin_unlock_irq(&pdev->lock);
		}

		/* Send uevent to userspace */
		if (test_and_clear_bit(VGT_REQUEST_UEVENT, (void *)&pdev->request)) {
			/* TODO: Give a new type of request when not all
			 *	 uevents are for hotplug
			 */
			vgt_probe_edid(pdev, -1);
			vgt_signal_uevent(pdev);
		}

		if (test_and_clear_bit(VGT_REQUEST_PPGTT_INIT, (void *)&pdev->request)) {
			int i;
			for (i = 0; i < VGT_MAX_VMS; i++) {
				if (pdev->device[i] && pdev->device[i]->need_ppgtt_setup) {
					vgt_setup_ppgtt(pdev->device[i]);
					pdev->device[i]->need_ppgtt_setup = false;
				}
			}
		}

		/* context switch timeout hasn't expired */
		if (wait)
			continue;

		wait = period;

		if (!vgt_ctx_switch)
			continue;

		if (!(vgt_ctx_check(pdev) % threshold))
			printk("vGT: %lldth checks, %lld switches\n",
				vgt_ctx_check(pdev), vgt_ctx_switch(pdev));
		vgt_ctx_check(pdev)++;

		pdev->dev_func.force_wake(pdev);

		/* Response to the monitor switch request. */
		/* vgt display switch moved out rendering context switch. */
		if (list_empty(&pdev->rendering_runq_head)) {
			/* Idle now, and no pending activity */
			printk("....idle\n");
			continue;
		}

		/*
		 * disable interrupt which is sufficient to prevent more
		 * cmds submitted by the current owner, when dom0 is UP.
		 * if the mmio handler for HVM is made into a thread,
		 * simply a spinlock is enough
		 */
		spin_lock_irq(&pdev->lock);
		if (is_rendering_engines_empty(pdev, &ring_id)) {
			next = next_vgt(&pdev->rendering_runq_head, current_render_owner(pdev));
			dprintk("vGT: next vgt (%d)\n", next->vgt_id);
			if ( next != current_render_owner(pdev) )
			{
				rdtsc_barrier();
				start = get_cycles();
				rdtsc_barrier();
				/*
				 * FIXME: now acquire the lock with interrupt disabled.
				 * So far vGT's own interrupt handler hasn't been
				 * registered, so that a GEN interrupt may comes in the
				 * middle of the context switch. the i915 interrupt
				 * handler requires GP fault for emulation, and thus
				 * result in dead lock.
				 *
				 * Later remove the irq disable when integrating
				 * interrupt part.
				 */
				prev = current_render_owner(pdev);
				vgt_ctx_switch(pdev)++;

				//show_seqno(pdev);
				if (!vgt_save_context(prev)) {
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("vGT: (%lldth checks %lldth switch<%d->%d>): fail to save context\n",
						vgt_ctx_check(pdev),
						vgt_ctx_switch(pdev),
						prev->vgt_id,
						next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					local_irq_enable();
					break;
				}

				vgt_irq_save_context(prev, VGT_OT_RENDER);

				if (!vgt_restore_context(next)) {
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("vGT: (%lldth checks %lldth switch<%d->%d>): fail to restore context\n",
						vgt_ctx_check(pdev),
						vgt_ctx_switch(pdev),
						prev->vgt_id,
						next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					local_irq_enable();
					break;
				}

				if (prev->exit_req_from_render_switch) {
					vgt_deactive(prev->pdev, &prev->list);
					complete(&prev->exit_from_render_switch);
				}

				previous_render_owner(pdev) = current_render_owner(pdev);
				current_render_owner(pdev) = next;
				vgt_irq_restore_context(next, VGT_OT_RENDER);
				//show_seqno(pdev);
				vgt_resume_ringbuffers(next);

				rdtsc_barrier();
				end = get_cycles();
				rdtsc_barrier();
				//printk("vGT: take %lld cycles\n", end - start);
			}
			else
				dprintk("....no other instance\n");
		} else {
			printk("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
				vgt_ctx_switch(pdev), ring_id,
				current_render_owner(pdev)->vgt_id);
			show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		}
		spin_unlock_irq(&pdev->lock);
		/* Virtual interrupts pending right after render switch */
		if (pdev->request & VGT_REQUEST_IRQ) {
			printk("vGT: handle pending interrupt in the render context switch time\n");
			spin_lock_irq(&pdev->lock);
			clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
			vgt_handle_virtual_interrupt(pdev, VGT_OT_RENDER);
			spin_unlock_irq(&pdev->lock);
		}
	}
	return 0;
}

/*
 * Only need to save head, since it's the only one updated by hw
 */
void ring_phys_2_shadow(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	dprintk("old head(%x), old tail(%x)\n", srb->head, srb->tail);
	srb->head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	dprintk("new head(%x), new tail(%x)\n", srb->head, srb->tail);
#if 0
	srb->tail = VGT_MMIO_READ(pdev, &prb->tail);
	srb->start = VGT_MMIO_READ(pdev, &prb->start);
	srb->ctl = VGT_MMIO_READ(pdev, &prb->ctl);
#endif
}

/* Rewind the head/tail registers */
void rewind_ring(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, ring_id), srb->tail);
	VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, ring_id), srb->head);
}

static inline void stop_ring(struct pgt_device *pdev, int ring_id)
{
	/* wait for ring idle */
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[ring_id],
		       _REGBIT_MI_STOP_RINGS | (_REGBIT_MI_STOP_RINGS << 16));
	ring_wait_for_empty(pdev, ring_id, false, "stop-ring");
}

static inline void resume_ring(struct pgt_device *pdev, int ring_id)
{
	/* make sure ring resumed */
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[ring_id],
		       _REGBIT_MI_STOP_RINGS << 16);
	if (VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_STOP_RINGS)
		printk("!!!!!!!!!failed to clear stop ring bit\n");
}

/*
 * write to head is undefined when ring is enabled.
 *
 * so always invoke this disable action when recovering a new ring setting
 */
static inline void disable_ring(struct pgt_device *pdev, int ring_id)
{
	stop_ring(pdev, ring_id);
	/* disable the ring */
	VGT_MMIO_WRITE(pdev, RB_CTL(pdev, ring_id), 0);
	/* by ktian1. no source for this trick */
	VGT_POST_READ(pdev, RB_CTL(pdev, ring_id));
}

static inline void enable_ring(struct pgt_device *pdev, int ring_id, vgt_reg_t val)
{
	ASSERT(val & _RING_CTL_ENABLE);
	VGT_MMIO_WRITE(pdev, RB_CTL(pdev, ring_id), val);
	VGT_POST_READ(pdev, RB_CTL(pdev, ring_id));
	resume_ring(pdev, ring_id);
}

void stop_rings(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < pdev->max_engines; i++)
		stop_ring(pdev, i);
}

void resume_rings(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < pdev->max_engines; i++)
		resume_ring(pdev, i);
}

/*
 * to restore to a new ring buffer, we need restore all ring regs including head.
 */
void ring_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	dprintk("shadow 2 phys: [%x, %x, %x, %x] \n", srb->head, srb->tail,
		VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id)),
		VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id)));

	if (!(srb->ctl & _RING_CTL_ENABLE)) {
		printk("vGT/switch-%lld: ring (%d) not enabled. exit restore\n",
			vgt_ctx_switch(pdev), ring_id);
		VGT_MMIO_WRITE(pdev, RB_CTL(pdev, ring_id), 0);
		return;
	}

	disable_ring(pdev, ring_id);

	VGT_MMIO_WRITE(pdev, RB_START(pdev, ring_id), srb->start);

	/* make head==tail when enabling the ring buffer */
	VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, ring_id), srb->head);
	VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, ring_id), srb->head);

	enable_ring(pdev, ring_id, srb->ctl);

	/*
	 * FIXME: One weird issue observed when switching between dom0
	 * and win8 VM. The video ring #1 is not used by both dom0 and
	 * win8 (head=tail=0), however sometimes after switching back
	 * to win8 the video ring may enter a weird state that VCS cmd
	 * parser continues to parse the whole ring (fulled with ZERO).
	 * Sometimes it ends for one whole loop when head reaches back
	 * to 0. Sometimes it may parse indefinitely so that there's
	 * no way to wait for the ring empty.
	 *
	 * Add a posted read works around the issue. In the future we
	 * can further optimize by not switching unused ring.
	 */
	VGT_POST_READ(pdev, RB_HEAD(pdev, ring_id));
	dprintk("shadow 2 phys: [%x, %x]\n",
		VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id)));
}

#if 0
/*
 * Load sring from vring.
 *
 * FIXME: Do we require this? suppose vreg->sreg is done in mmio handler already?
 */
static void vring_2_sring(struct vgt_device *vgt, vgt_state_ring_t *rb)
{
	rb->sring.head = rb->vring.head;
	rb->sring.tail = rb->vring.tail;
	/* Aperture fixes */
	rb->sring.start = mmio_g2h_gmadr(vgt, rb->vring.start);

	rb->sring.ctl = rb->vring.start;

	/* TODO: QoS  control to advance tail reg. */
}
#endif

/*
 * s2v only needs to update head register.
 *
 * Now invoked from context switch time, assuming that 50ms quantum for
 * a VM won't fill all the ring buffer. This is the only place where
 * vr->head is updated.
 */
static void sring_2_vring(struct vgt_device *vgt, int ring_id,
	vgt_ringbuffer_t *sr, vgt_ringbuffer_t *vr)
{
	vr->head = sr->head;
}

/*
 * FIXME:
 * Now we reuse VM's ringbuffer to submit context switch commands,
 * which can save the extra costs to program MMIO regs. Ideally
 * context switch is kicked in only when ring buffer is empty, so
 * that no valid content exists. But for safety now we still save
 * original content replaced by vGT's commands, and retore it later.
 *
 * However this could be costful, since aperture is mapped as WC.
 * Later we may want to skip the save/restore or use vGT's own
 * aperture instead.
 */
static  void ring_save_commands (vgt_state_ring_t *rb,
	char *p_aperture, char *buf, int bytes)
{
	char	*p_contents;
	vgt_reg_t	rbtail;
	vgt_reg_t  ring_size, to_tail;

	ASSERT ((bytes & 3) == 0);
	p_contents = p_aperture + rb->sring.start;
	rbtail = rb->sring.head & RB_HEAD_OFF_MASK; /* in byte unit */

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;
	dprintk("p_contents(save): %lx, rbtail: %x, ring_size: %x, to_tail: %x, start: %x\n",
		(unsigned long)p_contents, rbtail, ring_size, to_tail,
		rb->sring.start);

#if 0
	printk("current buffer: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", (*((u32*)buf + i)));
	printk("\n");
	printk("current ring buffer: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", *((u32*)p_contents + rbtail/4 + i));
	printk("\n");
#endif

	if ( likely(to_tail >= bytes) )
	{
		memcpy (buf, p_contents + rbtail, bytes);
	}
	else {
		memcpy (buf, p_contents + rbtail, to_tail);
		memcpy (buf + to_tail, p_contents, bytes - to_tail);
	}

#if 0
	printk("saved content: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", (*((u32*)buf + i)));
	printk("\n");
#endif
}

static void ring_load_commands(vgt_state_ring_t *rb,
	char *p_aperture, char *buf, int bytes)
{
	char	*p_contents;
	vgt_reg_t	rbtail;
	vgt_reg_t  ring_size, to_tail;	/* bytes */

	p_contents = p_aperture + rb->sring.start;
	/* reset to the head for every load */
	rbtail = rb->phys_tail = rb->sring.head & RB_HEAD_OFF_MASK; /* in byte unit */

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;
	dprintk("p_contents(restore): %lx, rbtail: %x, ring_size: %x, to_tail: %x, start: %x\n",
		(unsigned long)p_contents, rbtail, ring_size, to_tail,
		rb->sring.start);

#if 0
	printk("current command: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", (*((u32*)buf + i)));
	printk("\n");
	printk("current ring buffer: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", *((u32*)p_contents + rbtail/4 + i));
	printk("\n");

	printk("copy to %lx\n", (unsigned long)((rb_dword *)p_contents + rbtail));
#endif

	if ( likely(to_tail >= bytes) )
	{
		/* FIXME: need using VGT_MMIO_WRITE */
		memcpy (p_contents + rbtail, buf, bytes);
		rb->phys_tail += bytes;
	} else {
		memcpy (p_contents + rbtail, buf, to_tail);
		memcpy (p_contents, buf + to_tail, bytes - to_tail);
		rb->phys_tail = bytes - to_tail;
	}

#if 0
	//wbinvd();
	printk("updated ring buffer: ");
	for (i = 0; i < bytes/4; i++)
		printk(" %x", *((u32*)p_contents + rbtail/4 + i));
	printk("\n");
	printk("phys_tail: %x\n", rb->phys_tail);
#endif
}

static inline void save_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	dprintk("<vgt-%d>save ring buffer\n", vgt->vgt_id);
	ring_save_commands (&vgt->rb[ring_id],
			phys_aperture_vbase(vgt->pdev),
			(char*)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void restore_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	dprintk("<vgt-%d>restore ring buffer\n", vgt->vgt_id);
	ring_load_commands (&vgt->rb[ring_id],
			phys_aperture_vbase(vgt->pdev),
			(char *)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

#if 0
static void disable_power_management(struct vgt_device *vgt)
{
	/* Save the power state and froce wakeup. */
	vgt->saved_wakeup = VGT_MMIO_READ(vgt->pdev, _REG_FORCEWAKE);
	VGT_MMIO_WRITE(vgt->pdev, _REG_FORCEWAKE, 1);
	VGT_POST_READ(vgt->pdev, _REG_FORCEWAKE);	/* why this ? */
}

static void restore_power_management(struct vgt_device *vgt)
{
	/* Restore the saved power state. */
	VGT_MMIO_WRITE(vgt->pdev, _REG_FORCEWAKE, vgt->saved_wakeup);
	VGT_POST_READ(vgt->pdev, _REG_FORCEWAKE);	/* why this ? */
}
#endif

/*
 * TODO:
 * MI_SUSPEND_FLUSH is necessary for GEN5, but not SNB.
 * IVB furthers requires MI_ARB_ON_OFF to disable preemption
 * (from intel-gfx community)
 *
 * cmds_save_context should be a multiple of 8-bytes.
 * let's use sizeof(rb_dword)*16=64 bytes.
 */
static rb_dword	cmds_save_context[16] = {
	MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP,
	MI_STORE_DATA_IMM | MI_SDI_USE_GTT, MI_NOOP, MI_NOOP, MI_NOOP,
	MI_NOOP,MI_NOOP,MI_NOOP,MI_NOOP,
};

/* TODO: MI_FORCE_RESTORE is only required for initialization */
/* cmds_restore_context should be a multiple of 8-bytes.
 * let's use sizeof(rb_dword)*16=64 bytes.
 */
static rb_dword	cmds_restore_context[16] = {
	MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_MM_SPACE_GTT | MI_FORCE_RESTORE,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP,
	MI_STORE_DATA_IMM | MI_SDI_USE_GTT, MI_NOOP, MI_NOOP, MI_NOOP,
	MI_NOOP,MI_NOOP,MI_NOOP,MI_NOOP,
};

/*
 * Submit a series of context save/restore commands to ring engine,
 * and wait till it is executed.
 *
 * FIXME: currently only one series of commands can be issued, and a
 * ring buffer reset is required to issue another set of commands
 */
bool rcs_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes)
{
	vgt_state_ring_t	*rb;
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t	ccid;

	//ASSERT ((ccid_addr & ~GTT_PAGE_MASK) == 0 );

	rb = &vgt->rb[ring_id];

	dprintk("vGT: CCID is %x, new cmd is %x\n",
		VGT_MMIO_READ(pdev, _REG_CCID), cmds[2]);
	/*
	 * No need to program CCID. Per the PRM, CCID will be
	 * updated as the result of MI_SET_CONTEXT. In such
	 * case, if current CCID is valid, meaning that VM is using
	 * it, later MI_SET_CONTEXT will effectively save current
	 * context to the VM's area, and then update new ID pointing
	 * to vGT's area. Otherwise, it will be purely an ID change.
	 */

	dprintk("before load [%x, %x]\n",
		VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id)));

	dprintk("old magic number: %d\n",
		*(u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));

	if (!use_old_ctx_switch) {
		char *p_aperture;

		disable_ring(pdev, ring_id);

		p_aperture = phys_aperture_vbase(pdev) + pdev->ctx_switch_rb_page;
		memcpy(p_aperture, cmds, bytes);

		VGT_MMIO_WRITE(pdev, RB_START(pdev, ring_id), pdev->ctx_switch_rb_page);
		VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, ring_id), 0);
		VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, ring_id), 0);

		// ctx_switch_rb_page has a size of 1 page.
		enable_ring(pdev, ring_id, _RING_CTL_ENABLE);

		VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, ring_id), bytes);
	} else {
		//prepare for switching to the new ring
		ASSERT((cmds == cmds_save_context)||(cmds == cmds_restore_context));
		if (cmds == cmds_restore_context)
			//vring_2_sring(vgt, rb);
			ring_shadow_2_phys (pdev, ring_id, &rb->sring);

		// save 32 dwords of the ring
		save_ring_buffer (vgt, ring_id);

		ring_load_commands (rb, phys_aperture_vbase(pdev), (char*)cmds, bytes);
		VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, ring_id), rb->phys_tail);		/* TODO: Lock in future */
	}
	//mdelay(1);

	if (!ring_wait_for_empty(pdev, ring_id, true, "ctx-switch")) {
		printk("vGT: context switch commands unfinished\n");
		show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		return false;
	}

	if (use_old_ctx_switch)
		// restore 32 dwords of the ring
		restore_ring_buffer (vgt, ring_id);

	dprintk("new magic number: %d\n",
		*(u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));

	/* still confirm the CCID for safety. May remove in the future */
	ccid = VGT_MMIO_READ (pdev, _REG_CCID);
	if ((ccid & GTT_PAGE_MASK) != (cmds[2] & GTT_PAGE_MASK)) {
		printk("vGT: CCID isn't changed [%x, %x]\n", ccid, cmds[2]);
		return false;
	}

	return true;
}

bool default_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes)
{
	printk("vGT: unsupported command submit for engine (%d)\n", ring_id);
	return false;
}

/* FIXME: need audit all render resources carefully */
vgt_reg_t vgt_render_regs[] = {
	/* mode ctl regs. sync with vgt_mode_ctl_regs */
	_REG_ARB_MODE,

	_REG_CACHE_MODE_0,
	_REG_RCS_MI_MODE,
	_REG_GFX_MODE,

	_REG_VCS_MI_MODE,
	_REG_BCS_MI_MODE,

	_REG_RCS_INSTPM,
	_REG_VCS_INSTPM,
	_REG_BCS_INSTPM,

	_REG_GT_MODE,
	_REG_CACHE_MODE_1,

	/* other regs */

	_REG_RCS_HWSTAM,
	_REG_BCS_HWSTAM,
	_REG_VCS_HWSTAM,

	_REG_RCS_HWS_PGA,
	_REG_BCS_HWS_PGA,
	_REG_VCS_HWS_PGA,

	_REG_RCS_EXCC,
	_REG_BCS_EXCC,
	_REG_VCS_EXCC,

	_REG_RCS_UHPTR,
	_REG_BCS_UHPTR,
	_REG_VCS_UHPTR,

	_REG_TILECTL,
};

static void vgt_setup_render_regs(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_NUM(vgt_render_regs); i++) {
		reg_set_owner(pdev, vgt_render_regs[i], VGT_OT_RENDER);
		reg_set_pt(pdev, vgt_render_regs[i]);
	}

	/*
	 * FIXME: this should be whitelisted in vgt_render_regs, or else
	 * give pt permission w/o save/restore is wrong
	 */
	/* RCS */
	for (i = 0x2000; i <= 0x2FFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_RENDER);
		reg_set_pt(pdev, i);
	}

	/* VCS */
	for (i = 0x12000; i <= 0x12FFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_RENDER);
		reg_set_pt(pdev, i);
	}

	/* BCS */
	for (i = 0x22000; i <= 0x22FFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_RENDER);
		reg_set_pt(pdev, i);
	}

	if (pdev->is_ivybridge) {
		for (i = 0; i < ARRAY_NUM(vgt_gen7_mode_ctl_regs); i++) {
			reg_set_mode_ctl(pdev, vgt_gen7_mode_ctl_regs[i]);
		}
	} else {
		for (i = 0; i < ARRAY_NUM(vgt_mode_ctl_regs); i++)
			reg_set_mode_ctl(pdev, vgt_mode_ctl_regs[i]);
	}
}

/* TODO: lots of to fill */
vgt_reg_t vgt_display_regs[] = {
	_REG_CURACNTR	,
	_REG_CURABASE	,
	_REG_CURAPOS	,
	_REG_CURAVGAPOPUPBASE,
	_REG_CURAPALET_0,
	_REG_CURAPALET_1,
	_REG_CURAPALET_2,
	_REG_CURAPALET_3,
	_REG_CURASURFLIVE,

	_REG_CURBCNTR	,
	_REG_CURBBASE	,
	_REG_CURBPOS	,
	_REG_CURBPALET_0,
	_REG_CURBPALET_1,
	_REG_CURBPALET_2,
	_REG_CURBPALET_3,
	_REG_CURBSURFLIVE,

	_REG_DSPACNTR	,
	_REG_DSPALINOFF	,
	_REG_DSPASTRIDE	,
	_REG_DSPASURF	,
	_REG_DSPATILEOFF,
	_REG_DSPASURFLIVE,

	_REG_DSPBCNTR	,
	_REG_DSPBLINOFF	,
	_REG_DSPBSTRIDE	,
	_REG_DSPBSURF	,
	_REG_DSPBTILEOFF,
	_REG_DSPBSURFLIVE,

	_REG_DVSACNTR	,
	_REG_DVSALINOFF	,
	_REG_DVSASTRIDE	,
	_REG_DVSAPOS	,
	_REG_DVSASIZE	,
	_REG_DVSAKEYVAL	,
	_REG_DVSAKEYMSK	,
	_REG_DVSASURF	,
	_REG_DVSAKEYMAXVAL,
	_REG_DVSATILEOFF,
	_REG_DVSASURFLIVE,
	_REG_DVSASCALE	,

	_REG_DVSBCNTR	,
	_REG_DVSBLINOFF	,
	_REG_DVSBSTRIDE	,
	_REG_DVSBPOS	,
	_REG_DVSBSIZE	,
	_REG_DVSBKEYVAL	,
	_REG_DVSBKEYMSK	,
	_REG_DVSBSURF	,
	_REG_DVSBKEYMAXVAL,
	_REG_DVSBTILEOFF,
	_REG_DVSBSURFLIVE,
	_REG_DVSBSCALE	,
};

static void vgt_setup_display_regs(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_NUM(vgt_display_regs); i++) {
		reg_set_owner(pdev, vgt_display_regs[i], VGT_OT_DISPLAY);
		reg_set_pt(pdev, vgt_display_regs[i]);
	}

	/* display pallete registers */
	for (i = 0x4A000; i <= 0x4CFFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* PIPE control */
	for (i = 0x60000; i <= 0x6FFFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* Plane and cursor control */
	for (i = 0x70000; i <= 0x7FFFF; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* ============================== */
	/* !!!below need double confirm in the future */
	reg_set_owner(pdev, _REG_DISP_ARB_CTL, VGT_OT_DISPLAY);
	reg_set_pt(pdev, _REG_DISP_ARB_CTL);

	reg_set_owner(pdev, _REG_DISP_ARB_CTL2, VGT_OT_DISPLAY);
	reg_set_pt(pdev, _REG_DISP_ARB_CTL2);

	/* display watermark */
	for (i = 0x45100; i <= 0x45130; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* backlight */
	for (i = 0x48250; i <= 0x48270; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* panel power sequence */
	for (i = 0xc7200; i <= 0xc7210; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	reg_set_owner(pdev, 0xe1180, VGT_OT_DISPLAY); /* PCH_LVDS */
	reg_set_pt(pdev, 0xe1180);

	/* PCH shared functions (gmbus, gpio, clock, power seq, backlight) */
	for (i = 0xc0000; i <= 0xc7210; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* gmbus are fully virtualized */
	/* FIXME: The pt is still set for below registers who are fully
	 *	  virtualized. It is not a problem since the reg_hw_access()
	 *	  will not check "pt" if reg is "always_virt". In future,
	 *	  above loop should be modified to handle display registers
	 *	  one by one.
	 */

#ifndef ENABLE_GPIO_EMULATION
	for (i = _REG_PCH_GPIOA; i <= _REG_PCH_GPIOF; i += REG_SIZE) {
		reg_set_always_virt(pdev, i);
	}
#endif /* ENABLE_GPIO_EMULATION */

	for (i = _REG_PCH_GMBUS0; i <= _REG_PCH_GMBUS3; i += REG_SIZE) {
		reg_set_always_virt(pdev, i);
	}

	/* PCH transcoder and port control */
	for (i = 0xe0000; i <= 0xe4fff; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* PCH transcoder and FDI control */
	for (i = 0xf0000; i <= 0xf2fff; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}

	/* FDI PLL control */
	for (i = 0xee000; i <= 0xee007; i += REG_SIZE) {
		reg_set_owner(pdev, i, VGT_OT_DISPLAY);
		reg_set_pt(pdev, i);
	}
}

/* TODO: lots of to fill */
vgt_reg_t vgt_pm_regs[] = {
	_REG_GT_THREAD_STATUS,
	_REG_GT_CORE_STATUS,
	_REG_FORCEWAKE,
	_REG_FORCEWAKE_ACK,
	_REG_RC_STATE_CTRL_1,
	_REG_RC_STATE_CTRL_2,
};

static void vgt_setup_pm_regs(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_NUM(vgt_pm_regs); i++)
		reg_set_owner(pdev, vgt_pm_regs[i], VGT_OT_PM);
}

/* TODO: lots of to fill */
vgt_reg_t vgt_mgmt_regs[] = {
};

static void vgt_setup_mgmt_regs(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_NUM(vgt_mgmt_regs); i++)
		reg_set_owner(pdev, vgt_mgmt_regs[i], VGT_OT_MGMT);
}

void vgt_rendering_save_mmio(struct vgt_device *vgt)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int num = ARRAY_NUM(vgt_render_regs);
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		int reg = vgt_render_regs[i];
		//if (reg_hw_update(vgt->pdev, reg)) {
		/* FIXME: only hw update reg needs save */
		if (!reg_mode_ctl(vgt->pdev, reg))
		{
			__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
			__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
			dprintk("....save mmio (%x) with (%x)\n", reg, __sreg(vgt, reg));
		}
	}
}

/*
 * Rstore MMIO registers per rendering context.
 * (Not include ring buffer registers).
 */
void vgt_rendering_restore_mmio(struct vgt_device *vgt)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int num = ARRAY_NUM(vgt_render_regs);
	int i, j;
	struct pgt_device *pdev = vgt->pdev;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		int reg = vgt_render_regs[i];
		vgt_reg_t val = __sreg(vgt, reg);
		if (reg_mode_ctl(pdev, reg)) {
			for (j = 0; j < ARRAY_NUM(vgt_mode_ctl_regs); j++) {
				if (vgt_mode_ctl_regs[j] == reg) {
					val |= vgt_mode_mask_regs[j];
					break;
				}
			}
			if (j == ARRAY_NUM(vgt_mode_ctl_regs))
				val |= 0xFFFF0000;
		}

		/*
		 * FIXME: there's regs only with some bits updated by HW. Need
		 * OR vm's update with hw's bits?
		 */
		//if (!reg_hw_update(vgt->pdev, reg))
		if (__sreg(vgt, _REG_RCS_UHPTR) & 1) {
			printk("!!!!!UHPTR is valid after resuming. Clear the valid bit\n");
			__sreg(vgt, _REG_RCS_UHPTR) &= ~1;
			__vreg(vgt, _REG_RCS_UHPTR) &= ~1;
		}
		VGT_MMIO_WRITE(vgt->pdev, reg, val);
		dprintk("....restore mmio (%x) with (%x)\n", reg, val);
	}
}

/*
 * Rendering engine context switch
 *
 */

static bool vgt_save_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int 			i;
	vgt_state_ring_t	*rb;
	bool rc = true;
	vgt_reg_t old_tail;

	if (vgt == NULL)
		return false;

	//disable_power_management(vgt);

	vgt_rendering_save_mmio(vgt);

	/* save rendering engines */
	for (i=0; i < pdev->max_engines; i++) {
		rb = &vgt->rb[i];
		old_tail = rb->sring.tail;
		ring_phys_2_shadow (pdev, i, &rb->sring);

		sring_2_vring(vgt, i, &rb->sring, &rb->vring);

		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		/* for stateless engine, no need to save/restore context */
		if (rb->stateless)
			continue;

		/* No need to submit ArbOnOffInstruction */

		/*
		 * Switch context ID to the area allocated by vGT.
		 *
		 * If VM already has valid context ID in CCID, this will cause
		 * the current context saved to the VM's area; Or else this is
		 * purely an ID pointer change.
		 *
		 * Context save to vGT's area happens in the restore phase.
		 */
		switch (i) {
		case RING_BUFFER_RCS:
			//rb->context_save_area = 0xC2000000;
			/* Does VM want the ext state to be saved? */
			cmds_save_context[2] =
				MI_RESTORE_INHIBIT |
				MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN |
				MI_RESTORE_EXT_STATE_EN |
				rb->context_save_area;
			cmds_save_context[10] = vgt_data_ctx_magic(pdev);
			pdev->magic++;
			cmds_save_context[11] = pdev->magic;
			break;
		default:
			printk("vGT: unsupported engine (%d) switch \n", i);
			break;
		}

		rc = (*pdev->submit_context_command[i]) (vgt, i, cmds_save_context,
				sizeof(cmds_save_context));

		if (rc)
			rb->initialized = true;

		if (old_tail != rb->sring.tail)
			printk("!!!!!!!!!(save ring-%d, %llx switch) tail moved from %x to %x\n",
				i, vgt_ctx_switch(pdev), rb->sring.tail, old_tail);

		dprintk("<vgt-%d>vgt_save_context done\n", vgt->vgt_id);

	}
	return rc;
}

static bool vgt_restore_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_state_ring_t	*rb;
	bool rc;
	vgt_reg_t old_tail;

	if (vgt == NULL)
		return false;

	vgt_addr_fix_restore();

	for (i=0; i < pdev->max_engines; i++) {
		rb = &vgt->rb[i];

		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		if (rb->stateless)
			continue;

		old_tail = rb->sring.tail;

		switch (i) {
			case RING_BUFFER_RCS:
				if (rb->initialized) {
					cmds_restore_context[2] =
						rb->context_save_area |
						MI_MM_SPACE_GTT |
						MI_SAVE_EXT_STATE_EN |
						MI_RESTORE_EXT_STATE_EN |
						MI_FORCE_RESTORE;
				} else {
					printk("vGT(%d): first initialization. switch to dummy context.\n",
						vgt->vgt_id);
					cmds_restore_context[2] =
						pdev->dummy_area |
						MI_MM_SPACE_GTT |
						MI_SAVE_EXT_STATE_EN |
						MI_RESTORE_EXT_STATE_EN |
						MI_RESTORE_INHIBIT;
				}
				cmds_restore_context[10] = vgt_data_ctx_magic(pdev);
				pdev->magic++;
				cmds_restore_context[11] = pdev->magic;
				break;
			default:
				printk("vGT: unsupported engine (%d) switch \n", i);
				break;
		}
		/* some mode control registers can be only restored through this command */
		update_context(vgt, rb->context_save_area);
		rc = (*pdev->submit_context_command[i]) (vgt, i, cmds_restore_context,
			sizeof(cmds_restore_context));

		if (!rc)
			goto err;

		if (old_tail != rb->sring.tail)
			printk("!!!!!!!!!(restore ring-%d, %llx switch) tail moved from %x to %x\n",
				i, vgt_ctx_switch(pdev), rb->sring.tail, old_tail);
	}

	/* Restore ring registers */
	for (i=0; i < pdev->max_engines; i++) {
		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		rb = &vgt->rb[i];
		/* vring->sring */
		//vring_2_sring(vgt, rb);
		ring_shadow_2_phys (pdev, i, &rb->sring);
	}

	stop_rings(pdev);

	vgt_rendering_restore_mmio(vgt);

	resume_rings(pdev);

	/* Restore the PM */
	//restore_power_management(vgt);
	dprintk("<vgt-%d>vgt_restore_context done\n", vgt->vgt_id);
	return true;
err:
	/* TODO: need fall back to original VM's context */
	return false;
}

static void state_vreg_init(struct vgt_device *vgt)
{
	memcpy (vgt->state.vReg, vgt->pdev->initial_mmio_state, VGT_MMIO_SPACE_SZ);

	/* set the bit 0:2 (Thread C-State) to C0
	 * TODO: consider other bit 3:31
	 */
	__vreg(vgt, _REG_GT_THREAD_STATUS) = 0;

	/* set the bit 0:2(Core C-State ) to C0 */
	__vreg(vgt, _REG_GT_CORE_STATUS) = 0;

	/*TODO: init other regs that need different value from pdev */
}

/* TODO: figure out any security holes by giving the whole initial state */
static void state_sreg_init(struct vgt_device *vgt)
{
	vgt_reg_t *sreg;

	sreg = vgt->state.sReg;
	memcpy (sreg, vgt->pdev->initial_mmio_state, VGT_MMIO_SPACE_SZ);

	/*
	 * Do we really need address fix for initial state? Any address information
	 * there is meaningless to a VM, unless that address is related to allocated
	 * GM space to the VM. Translate a host address '0' to a guest GM address
	 * is just a joke.
	 */
#if 0
	/* FIXME: add off in addr table to avoid checking all regs */
	for (i = 0; i < VGT_MMIO_REG_NUM; i++) {
		if (reg_addr_fix(vgt->pdev, i * REG_SIZE)) {
			__sreg(vgt, i) = mmio_g2h_gmadr(vgt, i, __vreg(vgt, i));
			dprintk("vGT: address fix for reg (%x): (%x->%x)\n",
				i, __vreg(vgt, i), __sreg(vgt, i));
		}
	}
#endif
}

/*
 * Initialize the vgt state instance.
 * Return:   0: failed
 * 	     1: success
 *
 */
static int create_state_instance(struct vgt_device *vgt)
{
	vgt_state_t	*state;
	int i;

	dprintk("create_state_instance\n");
	state = &vgt->state;
	state->vReg = kmalloc (state->regNum * REG_SIZE, GFP_KERNEL);
	state->sReg = kmalloc (state->regNum * REG_SIZE, GFP_KERNEL);
	if ( state->vReg == NULL || state->sReg == NULL )
	{
		printk("VGT: insufficient memory allocation at %s\n", __FUNCTION__);
		if ( state->vReg )
			kfree (state->vReg);
		if ( state->sReg )
			kfree (state->sReg);
		state->sReg = state->vReg = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < VGT_BAR_NUM; i++)
		state->bar_mapped[i] = 0;
	return 0;
}

static int allocate_vm_aperture_gm_and_fence(struct vgt_device *vgt, vgt_params_t vp)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned long *gm_bitmap = pdev->gm_bitmap;
	unsigned long *fence_bitmap = pdev->fence_bitmap;
	unsigned long guard = hidden_gm_base(vgt->pdev)/SIZE_1MB;
	unsigned long gm_bitmap_total_bits = VGT_GM_BITMAP_BITS;
	unsigned long aperture_search_start = 0;
	unsigned long visable_gm_start, hidden_gm_start = guard;
	unsigned long fence_base;

	ASSERT(vgt->aperture_base == 0); /* not allocated yet*/
	ASSERT(vp.aperture_sz > 0 && vp.aperture_sz <= vp.gm_sz);
	ASSERT(vp.fence_sz > 0);

	if (vgt->vm_id == 0) {
		if (dom0_aperture_starts_at_128MB)
			aperture_search_start = 128;
		printk("vGT: dom0 aperture starts at %ldMB.\n",
			aperture_search_start);
	}

	visable_gm_start = bitmap_find_next_zero_area(gm_bitmap, guard,
				aperture_search_start, vp.aperture_sz, 0);
	if (visable_gm_start >= guard)
		return -ENOMEM;

	if (vp.gm_sz > vp.aperture_sz) {
		hidden_gm_start = bitmap_find_next_zero_area(gm_bitmap,
				gm_bitmap_total_bits, guard, vp.gm_sz - vp.aperture_sz, 0);
		if (hidden_gm_start >= gm_bitmap_total_bits)
			return -ENOMEM;
	}
	fence_base = bitmap_find_next_zero_area(fence_bitmap,
				VGT_FENCE_BITMAP_BITS, 0, vp.fence_sz, 0);
	if (fence_base >= VGT_MAX_NUM_FENCES)
		return -ENOMEM;

	vgt->aperture_base = phys_aperture_base(vgt->pdev) +
			(visable_gm_start * SIZE_1MB);
	vgt->aperture_sz = vp.aperture_sz * SIZE_1MB;
	vgt->gm_sz = vp.gm_sz * SIZE_1MB;
	vgt->hidden_gm_offset = hidden_gm_start * SIZE_1MB;
	vgt->fence_base = fence_base;
	vgt->fence_sz = vp.fence_sz;

	/* mark the related areas as BUSY. */
	bitmap_set(gm_bitmap, visable_gm_start, vp.aperture_sz);
	if (vp.gm_sz > vp.aperture_sz)
		bitmap_set(gm_bitmap, hidden_gm_start, vp.gm_sz - vp.aperture_sz);
	bitmap_set(fence_bitmap, fence_base, vp.fence_sz);
	return 0;
}

static void free_vm_aperture_gm_and_fence(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned long *gm_bitmap = pdev->gm_bitmap;
	unsigned long *fence_bitmap = pdev->fence_bitmap;
	unsigned long visable_gm_start =
		aperture_2_gm(vgt->pdev, vgt->aperture_base)/SIZE_1MB;
	unsigned long hidden_gm_start = vgt->hidden_gm_offset/SIZE_1MB;

	ASSERT(vgt->aperture_sz > 0 && vgt->aperture_sz <= vgt->gm_sz);

	/* mark the related areas as available */
	bitmap_clear(gm_bitmap, visable_gm_start, vgt->aperture_sz/SIZE_1MB);
	if (vgt->gm_sz > vgt->aperture_sz)
		bitmap_clear(gm_bitmap, hidden_gm_start,
			(vgt->gm_sz - vgt->aperture_sz)/SIZE_1MB);
	bitmap_clear(fence_bitmap, vgt->fence_base,  vgt->fence_sz);
}

static void initialize_gm_fence_allocation_bitmaps(struct pgt_device *pdev)
{
	unsigned long *gm_bitmap = pdev->gm_bitmap;

	printk("vGT: total aperture: 0x%x bytes, total GM space: 0x%llx bytes\n",
		phys_aperture_sz(pdev), gm_sz(pdev));

	ASSERT(phys_aperture_sz(pdev) % SIZE_1MB == 0);
	ASSERT(gm_sz(pdev) % SIZE_1MB == 0);
	ASSERT(phys_aperture_sz(pdev) <= gm_sz(pdev) && gm_sz(pdev) <= VGT_MAX_GM_SIZE);

	pdev->rsvd_aperture_sz = VGT_RSVD_APERTURE_SZ;
	pdev->rsvd_aperture_base = phys_aperture_base(pdev) + hidden_gm_base(pdev) -
								pdev->rsvd_aperture_sz;

	// mark the rsvd aperture as not-available.
	bitmap_set(gm_bitmap, aperture_2_gm(pdev, pdev->rsvd_aperture_base)/SIZE_1MB,
				pdev->rsvd_aperture_sz/SIZE_1MB);

	printk("vGT: reserved aperture: [0x%llx, 0x%llx)\n",
			pdev->rsvd_aperture_base,
			pdev->rsvd_aperture_base + pdev->rsvd_aperture_sz);

	/* serve as a simple linear allocator */
	pdev->rsvd_aperture_pos = 0;
}

/*
 * priv: VCPU ?
 */
int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp)
{
	int i;
	struct vgt_device *vgt;
	vgt_state_ring_t	*rb;
	char *cfg_space;
	int rc = -ENOMEM;

	printk("vGT: %s: vm_id=%d, aperture_sz=%dMB, gm_sz=%dMB, fence_sz=%d\n",
		__func__, vp.vm_id, vp.aperture_sz, vp.gm_sz, vp.fence_sz);

	vgt = kzalloc (sizeof(*vgt), GFP_KERNEL);
	if (vgt == NULL) {
		printk("Insufficient memory for vgt_device in %s\n", __FUNCTION__);
		return rc;
	}

	if ((rc = vgt->vgt_id = allocate_vgt_id()) < 0 )
		goto err;

	vgt->vm_id = vp.vm_id;
	vgt->pdev = pdev;

	vgt->exit_req_from_render_switch = 0;
	init_completion(&vgt->exit_from_render_switch);

	vgt->state.regNum = VGT_MMIO_REG_NUM;
	INIT_LIST_HEAD(&vgt->list);

	if ((rc = create_state_instance(vgt)) < 0)
		goto err;

	/* TODO: hard code ballooning now. We can support non-ballooning too in the future */
	vgt->ballooning = true;

	/* present aperture to the guest at the same host address */
	vgt->state.aperture_base = phys_aperture_base(pdev);

	/* init aperture/gm ranges allocated to this vgt */
	if ((rc = allocate_vm_aperture_gm_and_fence(vgt, vp)) < 0) {
		printk("vGT: %s: no enough available aperture/gm/fence!\n", __func__);
		goto err;
	}

	vgt->aperture_offset = aperture_2_gm(pdev, vgt->aperture_base);
	vgt->aperture_base_va = phys_aperture_vbase(pdev) +
		vgt->aperture_offset;

	if (vgt->ballooning)
		vgt->vgtt_sz = (gm_sz(pdev) >> GTT_PAGE_SHIFT) * GTT_ENTRY_SIZE;
	else
		vgt->vgtt_sz = (vgt->gm_sz >> GTT_PAGE_SHIFT) * GTT_ENTRY_SIZE;
	printk("vGT:   Virtual GTT size: 0x%lx\n", (long)vgt->vgtt_sz);
	vgt->vgtt = kzalloc(vgt->vgtt_sz, GFP_KERNEL);
	if (!vgt->vgtt) {
		printk("vGT: failed to allocate virtual GTT table\n");
		goto err;
	}

	vgt->rsvd_aperture_base = pdev->rsvd_aperture_base + pdev->rsvd_aperture_pos;
	pdev->rsvd_aperture_pos += VGT_APERTURE_PER_INSTANCE_SZ;
	printk("vGT:   rsvd_aperture_base: 0x%llx\n", vgt->rsvd_aperture_base);

	for (i=0; i< pdev->max_engines; i++) {
		rb = &vgt->rb[i];
		rb->context_save_area = aperture_2_gm(pdev, vgt->rsvd_aperture_base +
			i * SZ_CONTEXT_AREA_PER_RING);
		rb->initialized = false;
	}
	/* TODO */
	pdev->dummy_area = aperture_2_gm(pdev, vgt->rsvd_aperture_base +
                        pdev->max_engines * SZ_CONTEXT_AREA_PER_RING);

	vgt->rb[RING_BUFFER_RCS].stateless = 0;	/* RCS */
	vgt->rb[RING_BUFFER_VCS].stateless = 1;	/* BCS */
	vgt->rb[RING_BUFFER_BCS].stateless = 1;	/* VCS */

	vgt->state.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	vgt->state.bar_size[1] =			/* Aperture */
		vgt->ballooning ? pdev->bar_size[1] : vgt_aperture_sz(vgt);
	vgt->state.bar_size[2] = pdev->bar_size[2];	/* PIO */
	vgt->state.bar_size[3] = pdev->bar_size[3];	/* ROM */

	/* Set initial configuration space and MMIO space registers. */
	cfg_space = &vgt->state.cfg_space[0];
	memcpy (cfg_space, pdev->initial_cfg_space, VGT_CFG_SPACE_SZ);
	cfg_space[VGT_REG_CFG_SPACE_MSAC] = vgt->state.bar_size[1];
	vgt_pci_bar_write_32(vgt, VGT_REG_CFG_SPACE_BAR1, phys_aperture_base(pdev) );

	printk("vGT:   aperture: [0x%llx, 0x%llx] guest [0x%llx, 0x%llx] "
		"va(0x%llx)\n",
		vgt_aperture_base(vgt),
		vgt_aperture_end(vgt),
		vgt_guest_aperture_base(vgt),
		vgt_guest_aperture_end(vgt),
		(uint64_t)vgt->aperture_base_va);

	printk("vGT:   GM: [0x%llx, 0x%llx], [0x%llx, 0x%llx], "
		"guest[0x%llx, 0x%llx], [0x%llx, 0x%llx]\n",
		vgt_visible_gm_base(vgt),
		vgt_visible_gm_end(vgt),
		vgt_hidden_gm_base(vgt),
		vgt_hidden_gm_end(vgt),
		vgt_guest_visible_gm_base(vgt),
		vgt_guest_visible_gm_end(vgt),
		vgt_guest_hidden_gm_base(vgt),
		vgt_guest_hidden_gm_end(vgt));

	if (vgt->vm_id != 0 && !vgt_primary){
		/* Mark vgt device as non primary VGA */
		cfg_space[VGT_REG_CFG_CLASS_CODE] = VGT_PCI_CLASS_VGA;
		cfg_space[VGT_REG_CFG_SUB_CLASS_CODE] = VGT_PCI_CLASS_VGA_OTHER;
		cfg_space[VGT_REG_CFG_CLASS_PROG_IF] = VGT_PCI_CLASS_VGA_OTHER;
	}

	state_sreg_init (vgt);
	state_vreg_init(vgt);

	if ((rc = vgt_vstate_irq_init(vgt)) < 0)
		goto err;

	/* setup the ballooning information */
	if (vgt->ballooning) {
		__vreg64(vgt, vgt_info_off(magic)) = VGT_MAGIC;
		__vreg(vgt, vgt_info_off(version_major)) = 1;
		__vreg(vgt, vgt_info_off(version_minor)) = 0;
		__vreg(vgt, vgt_info_off(display_ready)) = 0;
		__vreg(vgt, vgt_info_off(vgt_id)) = vgt->vgt_id;
		__vreg(vgt, vgt_info_off(avail_rs.low_gmadr.my_base)) = vgt_visible_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.low_gmadr.my_size)) = vgt_aperture_sz(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.high_gmadr.my_base)) = vgt_hidden_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.high_gmadr.my_size)) = vgt_hidden_gm_sz(vgt);

		__vreg(vgt, vgt_info_off(avail_rs.fence_num)) = vgt->fence_sz;
		printk("vGT: filling VGT_PVINFO_PAGE for dom%d:\n"
			"   visable_gm_base=0x%llx, size=0x%llx\n"
			"   hidden_gm_base=0x%llx, size=0x%llx\n"
			"   fence_base=%d, num=%d\n",
			vgt->vm_id,
			vgt_visible_gm_base(vgt), vgt_aperture_sz(vgt),
			vgt_hidden_gm_base(vgt), vgt_hidden_gm_sz(vgt),
			vgt->fence_base, vgt->fence_sz);
	}

	pdev->device[vgt->vgt_id] = vgt;
	list_add(&vgt->list, &pdev->rendering_idleq_head);

	if (vgt->vm_id != 0){
		/* HVM specific init */
		if ((rc = vgt_hvm_info_init(vgt)) < 0 ||
			(rc = vgt_hvm_io_init(vgt)) < 0 ||
			(rc = vgt_hvm_enable(vgt)) < 0)
			goto err;
		if (pdev->enable_ppgtt)
			vgt_init_shadow_ppgtt(vgt);
	}

	if (vgt->vm_id) {
		vgt_ops->boot_time = 0;

		/* a special debug mode to give full access to hvm guest */
		if (hvm_render_owner)
			current_render_owner(pdev) = vgt;

		if (hvm_dpy_owner)
			current_display_owner(pdev) = vgt;
	}

	/* create debugfs interface */
	(void)vgt_init_debugfs();
	/* create debugfs per vgt */
	(void)vgt_create_debugfs(vgt);
	/* initialize i2c states */
	vgt_init_i2c_bus(&vgt->vgt_i2c_bus);
	/* assign aux_ch vregs for aux_ch virtualization */
	vgt_init_aux_ch_vregs(&vgt->vgt_i2c_bus, vgt->state.vReg);
	vgt_propagate_edid(vgt, -1);

	*ptr_vgt = vgt;
	return 0;
err:
	if ( vgt->aperture_base > 0)
		free_vm_aperture_gm_and_fence(vgt);
	kfree(vgt->vgtt);
	kfree(vgt->state.vReg);
	kfree(vgt->state.sReg);
	if (vgt->vgt_id >= 0)
		free_vgt_id(vgt->vgt_id);
	kfree(vgt);
	return rc;
}

void vgt_release_instance(struct vgt_device *vgt)
{
	int i;
	unsigned long flags;
	bool in_rendering_rq = false;
	struct list_head *pos;

	vgt_destroy_debugfs(vgt);

	/* switch the display owner to Dom0 if needed */
	if (current_display_owner(vgt->pdev) != vgt_dom0) {
		next_display_owner = vgt_dom0;
		do_vgt_display_switch(&default_device);
	}

	if (vgt_ctx_switch) {
		spin_lock_irqsave(&vgt->pdev->lock, flags);
		list_for_each (pos, &vgt->pdev->rendering_runq_head)
			if (pos == &vgt->list) {
				in_rendering_rq = true;
				vgt->exit_req_from_render_switch = 1;
				break;
			}
		spin_unlock_irqrestore(&vgt->pdev->lock, flags);

		if (in_rendering_rq)
			wait_for_completion(&vgt->exit_from_render_switch);

		/* The vgt may be still in executing. */
		while ( is_current_render_owner(vgt) )
			schedule();
	}

	vgt_hvm_info_deinit(vgt);
	vgt->pdev->device[vgt->vgt_id] = NULL;

	vgt_vstate_irq_exit(vgt);
	/* already idle */
	list_del(&vgt->list);

	for (i = 0; i < EDID_NUM; ++ i) {
		if (vgt->vgt_edids[i]) {
			kfree(vgt->vgt_edids[i]);
		}
	}

	if (vgt->pdev->enable_ppgtt)
		vgt_destroy_shadow_ppgtt(vgt);

	free_vm_aperture_gm_and_fence(vgt);
	kfree(vgt->vgtt);
	kfree(vgt->state.vReg);
	kfree(vgt->state.sReg);
	free_vgt_id(vgt->vgt_id);
	kfree(vgt);
	printk("vGT: vgt_release_instance done for dom%d: vgt_id=%d\n",
		vgt->vm_id, vgt->vgt_id);
}

static uint32_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off)
{
	unsigned long bar_s, bar_size=0;
	struct pci_dev *dev = pdev->pdev;

	pci_read_config_dword(dev,  bar_off, (uint32_t *)&bar_s);
	pci_write_config_dword(dev, bar_off, 0xFFFFFFFF);

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_size);
dprintk("read back bar_size %lx\n", bar_size);
	bar_size &= ~0xf;       /* bit 4-31 */
dprintk("read back bar_size1 %lx\n", bar_size);
	bar_size = 1 << find_first_bit(&bar_size, BITS_PER_LONG);
dprintk("read back bar_size2 %lx\n", bar_size);

	pci_write_config_dword(dev, bar_off, bar_s);

#if 0
        bar_s = pci_conf_read32( 0, vgt_bus, vgt_dev, vgt_fun, bar_off);
        pci_conf_write32(0, vgt_bus, vgt_dev, vgt_fun, bar_off, 0xFFFFFFFF);

        bar_size = pci_conf_read32(0, vgt_bus, vgt_dev, vgt_fun, bar_off);
        bar_size &= ~0xf;       /* bit 4-31 */
        bar_size = 1 << find_first_bit(&bar_size, sizeof(bar_size));

        pci_conf_write32(0, vgt_bus, vgt_dev, vgt_fun, bar_offset, bar_s);
#endif
        return bar_size;
}

static bool save_vbios(struct pgt_device *pdev)
{
	char *ptr = __va(0xC0000);
	u64 size;
	int i, pages, cnt, rest, j, vbt_start = -1;
	char sum = 0;
	struct page *page;
	char *vbios;

	pdev->vbios = NULL;
	/* allocate 64KB buffer */
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(VGT_VBIOS_PAGES));
	if (!page) {
		printk("vGT: no enough memory for vBIOS\n");
		return false;
	}
	pdev->vbios = page;
	/* FIXME: not sure why the __va doesn't work sanely here */
#if 0
	vbios = __va(page_to_phys(page));
	printk("vGT: save vbios at %lx\n", (uint64_t)vbios);
	for (i = 0; i < VGT_VBIOS_PAGES; i++) {
		printk("%d: pa(%lx), mfn (%lx), va1(%lx), va2(%lx)\n",
			i, page_to_phys(page + i), g2m_pfn(0, page_to_pfn(page + i)) << PAGE_SHIFT,
			pfn_to_kaddr(page_to_pfn(page + i)), __va(page_to_phys(page + i)));
		*(char*)(__va(page_to_phys(page + i))) = 0xaa;
	}
#endif

	if (*(uint16_t *)ptr != 0xAA55) {
		printk("vGT: no valid VBIOS found!\n");
		return false;
	}

	printk("vGT: found a valid VBIOS\n");
	size = ptr[2] * 512;
	ASSERT_NUM(size && (size < (VGT_VBIOS_PAGES << PAGE_SHIFT)), size);

	pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	pages = 1;
	printk("vGT: VBIOS size: %llx (%d pages)\n", size, pages);

	for (i = 0; i + 4 < size; i++)
		if (!memcmp(ptr + i, "$VBT", 4)) {
			printk("vGT: find VBT table at %x\n", 0xC0000+i);
			vbt_start = i;
			break;
		}

	rest = size;
	cnt = PAGE_SIZE;
	for (i = 0; i < pages; i++) {
		if (rest < PAGE_SIZE)
			cnt = rest;
		vbios = (char *)kmap(page + i);
		printk("vGT: copy %dth vbios page (pa-%llx, va-%llx, cnt-%x)\n", i,
			(u64)page_to_phys(page + i), (u64)vbios, cnt);
		memcpy(vbios, ptr + i * PAGE_SIZE, cnt);
		/*
		 * FIXME: now not sure the reason. only the 1st page can be correctly
		 * scanned by the 2nd VM. fortunately the vbt size is 0xf61 on this
		 * platform. So we move the vbt table to be fully within the 1st page
		 * as a workaround.
		 */
		if (i == 0 && vbt_start != -1)
			memcpy(vbios + 0x60, ptr + vbt_start, 0xf80);
		for (j = 0; j < cnt; j++)
			sum += vbios[j];
		if (i == pages - 1 && sum + vbios[j-1] != 0) {
			printk("vGT: adjust VBIOS checksum (%x->%x)\n", (u32)vbios[j - 1], (u32)-sum);
			vbios[j - 1] = -sum;
		}
#if 0
		/* check 1200B VBT table */
		if (i == 0)
			for (j = 0xab0; j < 0xab0 + 0x4b0; j += 4) {
				if (!(j % 16))
					printk("\n[%4x]:", j);
				printk(" %4x", *(uint32_t *)(vbios + j));
			}
#endif
		kunmap(page + i);
		rest -= PAGE_SIZE;
	}

	/*
	 * FIXME: ROM BAR on the physical device may be disabled, when the GEN is
	 * used as the boot device. Hard code to 64KB now
	 */
	pdev->bar_size[3] = VGT_VBIOS_PAGES << PAGE_SHIFT;
	pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR_ROM] &= 0x1; /* enabled */
	return true;
}

bool initial_phys_states(struct pgt_device *pdev)
{
	int i;
	uint64_t	bar0, bar1;
	struct pci_dev *dev = pdev->pdev;

dprintk("VGT: Initial_phys_states\n");
	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4)
		pci_read_config_dword(dev, i,
				(uint32_t *)&pdev->initial_cfg_space[i]);
	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4) {
		if (!(i % 16))
			dprintk("\n[%2x]: ", i);

		dprintk("%02x %02x %02x %02x ",
			*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff00) >> 8,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff0000) >> 16,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff000000) >> 24);
	}
	for (i=0; i < 3; i++) {
		pdev->bar_size[i] = pci_bar_size(pdev, VGT_REG_CFG_SPACE_BAR0 + 8*i);
		printk("bar-%d size: %x\n", i, pdev->bar_size[i]);
	}

	bar0 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR0];
	bar1 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR1];
	dprintk("bar0: 0x%llx, Bar1: 0x%llx\n", bar0, bar1);

	ASSERT ((bar0 & 7) == 4);
	/* memory, 64 bits bar0 */
	pdev->gttmmio_base = bar0 & ~0xf;

	ASSERT ((bar1 & 7) == 4);
	/* memory, 64 bits bar */
	pdev->gmadr_base = bar1 & ~0xf;
	printk("gttmmio: 0x%llx, gmadr: 0x%llx\n", pdev->gttmmio_base, pdev->gmadr_base);

	/* TODO: no need for this mapping since hypercall is used */
	pdev->gttmmio_base_va = ioremap (pdev->gttmmio_base, 2 * VGT_MMIO_SPACE_SZ);
	if ( pdev->gttmmio_base_va == NULL ) {
		printk("Insufficient memory for ioremap1\n");
		return false;
	}
	printk("gttmmio_base_va: 0x%llx\n", (uint64_t)pdev->gttmmio_base_va);

#if 1		// TODO: runtime sanity check warning...
	//pdev->phys_gmadr_va = ioremap (pdev->gmadr_base, VGT_TOTAL_APERTURE_SZ);
	pdev->gmadr_va = ioremap (pdev->gmadr_base, pdev->bar_size[1]);
	if ( pdev->gmadr_va == NULL ) {
		iounmap(pdev->gttmmio_base_va);
		printk("Insufficient memory for ioremap2\n");
		return false;
	}
	printk("gmadr_va: 0x%llx\n", (uint64_t)pdev->gmadr_va);
#endif

#if 0
	/* TODO: Extend VCPUOP_request_io_emulation hypercall to handle
	 * trunk data read request, and use hypercall here.
	 * Or enable "rep movsx" support.
	 */
	memcpy (pdev->initial_mmio_state, pdev->gttmmio_base_va,
			VGT_MMIO_SPACE_SZ);

#else
	for (i = 0; i < VGT_MMIO_REG_NUM; i++) {
		pdev->initial_mmio_state[i] = *((vgt_reg_t *)pdev->gttmmio_base_va + i);
	}
#endif

	/* FIXME: GMBUS2 has an in-use bit as the hw semaphore, and we should recover
	 * it after the snapshot. Remove this workaround after GMBUS virtualization
	 */
	{
		u32 val = VGT_MMIO_READ(pdev, 0xc5108);
		printk("vGT: GMBUS2 init value: %x, %x\n", pdev->initial_mmio_state[REG_INDEX(0xc5100)], val);
		VGT_MMIO_WRITE(pdev, 0xc5108, val | 0x8000);
	}

	return save_vbios(pdev);
}

/*
 * model specific reg policy setup here
 *
 * based on search of the keyword "GraphicsAddress" in PRM
 */
static void vgt_setup_addr_fix_info(struct pgt_device *pdev)
{
	vgt_set_addr_mask(pdev, _REG_RCS_START, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_BCS_START, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_VCS_START, 0xFFFFF000);

	/*
	 * FIXME: a separate handler maybe required, since a blind address
	 * translation when valid bit is cleared is problematic
	 */
	vgt_set_addr_mask(pdev, _REG_RCS_BB_ADDR, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_VCS_BB_ADDR, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_BCS_BB_ADDR, 0xFFFFF000);

	vgt_set_addr_mask(pdev, _REG_RCS_HWS_PGA, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_VCS_HWS_PGA, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_BCS_HWS_PGA, 0xFFFFF000);

	vgt_set_addr_mask(pdev, _REG_RCS_BB_PREEMPT_ADDR, 0xFFFFF000);
	//vgt_set_addr_mask(pdev, _REG_RCS_BB_ADDR_DIFF, 0xFFFFF000);
	//vgt_set_addr_mask(pdev, _REG_RCS_BB_OFFSET, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_CCID, 0xFFFFF000);

	vgt_set_addr_mask(pdev, _REG_RCS_FBC_RT_BASE_ADDR, 0xFFFFF000);

	if (pdev->enable_ppgtt) {
		vgt_set_addr_mask(pdev, _REG_RCS_PP_DIR_BASE_IVB, 0xFFFF0000);
		vgt_set_addr_mask(pdev, _REG_VCS_PP_DIR_BASE, 0xFFFF0000);
		vgt_set_addr_mask(pdev, _REG_BCS_PP_DIR_BASE, 0xFFFF0000);
	}

	vgt_set_addr_mask(pdev, _REG_CURABASE, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_CURBBASE, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DSPASURF, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DSPASURFLIVE, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DSPBSURF, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DSPBSURFLIVE, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DVSASURF, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DVSASURFLIVE, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DVSBSURF, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_DVSBSURFLIVE, 0xFFFFF000);

	/* ===== things to be further studied ====== */
	/* PP_PFD: 32 PPGTT page fault data registers */
	/* VCS context workaround ponter */
	/* TLB registers */
	/* performance statistics registers like OABUFFER */
	/* debug registers */
}

static void vgt_setup_always_virt(struct pgt_device *pdev)
{
	int i;

	for (i = VGT_PVINFO_PAGE; i < VGT_PVINFO_PAGE + VGT_PVINFO_SIZE; i += REG_SIZE)
		reg_set_always_virt(pdev, i);
	/*
	 * FIXME: rinbbuffer registers may return ZERO when power management
	 * is active. We tried to disable pm logic from i915 driver, but it
	 * looks that it may not work well. So set the forcewake always virtualized
	 * and force enabling it. In the future we need to fix this in fine-grained
	 * level. One problem is that operating forcewake reg at this point has no
	 * effect, so we postpone to the time at the 1st context switch
	 */
	reg_set_always_virt(pdev, _REG_FORCEWAKE);
	reg_set_always_virt(pdev, _REG_FORCEWAKE_ACK);
	reg_set_always_virt(pdev, _REG_GT_CORE_STATUS);
	reg_set_always_virt(pdev, _REG_GT_THREAD_STATUS);
	reg_set_always_virt(pdev, _REG_RC_STATE_CTRL_1);
	reg_set_always_virt(pdev, _REG_RC_STATE_CTRL_2);

	reg_set_always_virt(pdev, _REG_MUL_FORCEWAKE);
}

static void vgt_setup_hw_update_regs(struct pgt_device *pdev)
{
	//reg_set_hw_update(pdev, TIMESTAMP);
	reg_set_hw_update(pdev, _REG_RCS_UHPTR);
	reg_set_hw_update(pdev, _REG_VCS_UHPTR);
	reg_set_hw_update(pdev, _REG_BCS_UHPTR);

	reg_set_hw_update(pdev, _REG_CURASURFLIVE);
	reg_set_hw_update(pdev, _REG_CURBSURFLIVE);
	reg_set_hw_update(pdev, _REG_DSPASURFLIVE);
	reg_set_hw_update(pdev, _REG_DSPBSURFLIVE);
	reg_set_hw_update(pdev, _REG_DVSASURFLIVE);
	reg_set_hw_update(pdev, _REG_DVSBSURFLIVE);
}

uint64_t vgt_get_gtt_size(struct pci_bus *bus)
{
	uint16_t gmch_ctrl;

	ASSERT(!bus->number);
	/* GTT size is within GMCH. */
	pci_bus_read_config_word(bus, 0, _REG_GMCH_CONTRL, &gmch_ctrl);
	switch ( (gmch_ctrl >> 8) & 3 ) {
	case	1:
		return 1 * SIZE_1MB;
	case	2:
		return 2 * SIZE_1MB;
	default:
		printk("Wrong GTT memory size\n");
		break;
	}
	return 0;
}

static void vgt_set_device_type(struct pgt_device *pdev)
{
	pdev->is_sandybridge = _is_sandybridge(pdev->pdev->device);
	if ( pdev->is_sandybridge )
		printk("Detected Sandybridge\n");
	pdev->is_ivybridge = _is_ivybridge(pdev->pdev->device);
	if ( pdev->is_ivybridge )
		printk("Detected Ivybridge\n");
	pdev->is_haswell = _is_haswell(pdev->pdev->device);
	if ( pdev->is_haswell )
		printk("Detected Haswell\n");
}

static bool vgt_initialize_pgt_device(struct pci_dev *dev, struct pgt_device *pdev)
{
	pdev->pdev = dev;
	pdev->pbus = dev->bus;

	vgt_set_device_type(pdev);

	/* check PPGTT enabling. now always enable on IVB. */
	if (pdev->is_ivybridge)
		pdev->enable_ppgtt = 1;

	INIT_LIST_HEAD(&pdev->rendering_runq_head);
	INIT_LIST_HEAD(&pdev->rendering_idleq_head);

	pdev->reg_info = kzalloc (VGT_MMIO_REG_NUM * sizeof(reg_info_t),
				GFP_KERNEL);
	if (!pdev->reg_info) {
		printk("vGT: failed to allocate reg_info\n");
		return false;
	}

	gm_sz(pdev) = vgt_get_gtt_size(pdev->pbus) * 1024;
	/* first setup the reg ownership mapping */
	vgt_setup_render_regs(pdev);
	vgt_setup_display_regs(pdev);
	vgt_setup_pm_regs(pdev);
	vgt_setup_mgmt_regs(pdev);

	/* then setup always virtualized reg */
	vgt_setup_always_virt(pdev);

	/* then mark regs updated by hw */
	vgt_setup_hw_update_regs(pdev);

	/* then add addr fix info for pass-through regs */
	vgt_setup_addr_fix_info(pdev);

	/* clean port status, 0 means not plugged in */
	memset(pdev->port_detect_status, 0, sizeof(pdev->port_detect_status));

	/* TODO: add ivb/hsw difference later */
	pdev->max_engines = 3;
	pdev->ring_mmio_base[RING_BUFFER_RCS] = _REG_RCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_VCS] = _REG_VCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_BCS] = _REG_BCS_TAIL;

	pdev->ring_psmi[RING_BUFFER_RCS] = _REG_RCS_PSMI;
	pdev->ring_psmi[RING_BUFFER_VCS] = _REG_VCS_PSMI;
	pdev->ring_psmi[RING_BUFFER_BCS] = _REG_BCS_PSMI;

	pdev->ring_mi_mode[RING_BUFFER_RCS] = _REG_RCS_MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_VCS] = _REG_VCS_MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_BCS] = _REG_BCS_MI_MODE;

	pdev->submit_context_command[RING_BUFFER_RCS] =
		rcs_submit_context_command;
	pdev->submit_context_command[RING_BUFFER_VCS] =
		default_submit_context_command;
	pdev->submit_context_command[RING_BUFFER_BCS] =
		default_submit_context_command;

	return true;
}

/* Setup device specific handler for different functions. */
static bool vgt_init_device_func (struct pgt_device *pdev)
{
	/* force wake handler */
	pdev->dev_func.force_wake = vgt_gen6_force_wake;

	if (pdev->is_ivybridge || pdev->is_haswell) {
		/* FIXME To check if enable multithread force wake, we have to
		 * probe ECOBUS (0xa180) bit5. But it looks current hyper call MMIO
		 * read and pdev->initial_mmio_state both return 0, which is different
		 * from native value (e.g 0x84100020). I don't know why...
		 *
		 * Always use MT force wake now.
		 */
#if 0
		u32 temp;
		vgt_gen6_mul_force_wake(pdev);
		temp = VGT_MMIO_READ(pdev, _REG_ECOBUS);
		if (temp & _REGBIT_MUL_FORCEWAKE_ENABLE) {
			/* enable multithread force wake */
		}
#endif
		printk("vGT: Use MT force wake!\n");
		pdev->dev_func.force_wake = vgt_gen6_mul_force_wake;
	}
	return true;
}

/* FIXME: allocate instead of static */
#define VGT_APERTURE_PAGES	(VGT_RSVD_APERTURE_SZ >> GTT_PAGE_SHIFT)
static struct page *pages[VGT_APERTURE_PAGES];
struct page *dummy_page;
dma_addr_t dummy_addr;
/* TODO: check license. May move to another file */
static int setup_gtt(struct pgt_device *pdev)
{
	struct page *page;
	int i, ret, index;
	dma_addr_t dma_addr;

	check_gtt(pdev);
	printk("vGT: clear all GTT entries.\n");
	dummy_page = alloc_page(GFP_KERNEL | __GFP_ZERO | GFP_DMA32);
	if (!dummy_page)
		return -ENOMEM;

	get_page(dummy_page);
	set_pages_uc(dummy_page, 1);
	dma_addr = pci_map_page(pdev->pdev, dummy_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	if (pci_dma_mapping_error(pdev->pdev, dma_addr))
		return -EINVAL;

	dma_addr |= (dma_addr >> 28) & 0xff0;
	dma_addr |= 0x1;	/* UC, valid */
	printk("....dummy page (0x%llx, 0x%llx)\n", page_to_phys(dummy_page), dma_addr);
	dummy_addr = dma_addr;

	/* for debug purpose */
	memset(pfn_to_kaddr(page_to_pfn(dummy_page)), 0x77, PAGE_SIZE);

	/* clear all GM space, instead of only aperture */
	for (i = 0; i < gm_pages(pdev); i++)
		vgt_write_gtt(pdev, i, dma_addr);

	dprintk("content at 0x0: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x0));
	dprintk("content at 0x64000: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x64000));
	dprintk("content at 0x8064000: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x8064000));

	check_gtt(pdev);
	printk("vGT: allocate vGT aperture\n");
	/* Fill GTT range owned by vGT driver */
	index = GTT_INDEX(pdev, aperture_2_gm(pdev, pdev->rsvd_aperture_base));
	for (i = 0; i < VGT_APERTURE_PAGES; i++) {
		/* need a DMA flag? */
		page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!page) {
			ret = -ENOMEM;
			goto err_out;
		}

		get_page(page);
		/* use wc instead! */
		set_pages_uc(page, 1);

		pages[i] = page;

		/* dom0 needs DMAR anyway */
		dma_addr = pci_map_page(pdev->pdev, page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
			ret = -EINVAL;
			goto err_out;
		}

		dma_addr |= (dma_addr >> 28) & 0xff0;
		dma_addr |= 0x1;	/* UC, valid */
		vgt_write_gtt(pdev, index + i, dma_addr);

		if (!(i % 1024))
			printk("vGT: write GTT-%x phys: %llx, dma: %llx\n",
				index + i, page_to_phys(page), dma_addr);
	}

	check_gtt(pdev);
	/* any cache flush required here? */
	return 0;
err_out:
	printk("vGT: error in GTT initialization\n");
	for (i = 0; i < VGT_APERTURE_PAGES; i++)
		if (pages[i]) {
			put_page(pages[i]);
			__free_page(pages[i]);
		}

	return ret;
}

/*
 * Initialize the vgt driver.
 *  return 0: success
 *	-1: error
 */
//int vgt_add_state_sysfs(struct vgt_device *vgt);
int vgt_initialize(struct pci_dev *dev)
{
	struct pgt_device *pdev = &default_device;
	struct task_struct *p_thread;
	vgt_params_t vp;

	if (novgt)
		return 0;

	spin_lock_init(&pdev->lock);

	memset (mtable, 0, sizeof(mtable));

	if (!vgt_initialize_pgt_device(dev, pdev))
		goto err;
	if (!vgt_initialize_mmio_hooks(pdev))
		goto err;
	if (!initial_phys_states(pdev))
		goto err;

	initialize_gm_fence_allocation_bitmaps(pdev);

	vgt_init_device_func(pdev);

	if (vgt_irq_init(pdev) != 0)
		goto err;

	/* setup the scratch page for the context switch */
	pdev->scratch_page = aperture_2_gm(pdev, pdev->rsvd_aperture_base +
		pdev->rsvd_aperture_pos);
	printk("scratch page is allocated at gm(0x%llx)\n", pdev->scratch_page);
	/* reserve the 1st trunk for vGT's general usage */
	pdev->rsvd_aperture_pos += VGT_APERTURE_PER_INSTANCE_SZ;

	/* reserve 1 page as the temporary ring buffer for context switch */
	pdev->ctx_switch_rb_page = aperture_2_gm(pdev, pdev->rsvd_aperture_base +
		pdev->rsvd_aperture_pos);
	pdev->rsvd_aperture_pos += PAGE_SIZE;
	printk("dexuan: ctx_switch_rb_page is allocated at gm(%llx)\n", pdev->ctx_switch_rb_page);

	/* initialize EDID data */
	vgt_probe_edid(pdev, -1);

	/* create domain 0 instance */
	vp.vm_id = 0;
	vp.aperture_sz = dom0_aperture_sz;
	vp.gm_sz = dom0_gm_sz;
	vp.fence_sz = dom0_fence_sz;
	if (create_vgt_instance(pdev, &vgt_dom0, vp) < 0)
		goto err;

	pdev->owner[VGT_OT_DISPLAY] = vgt_dom0;
	dprintk("create dom0 instance succeeds\n");

	show_mode_settings(pdev);

	if (setup_gtt(pdev))
		goto err;

	xen_vgt_dom0_ready(vgt_dom0);

	/* "hvm_owner" is a special mode where we give all the ownerships to the hvm guest */
	if (!hvm_render_owner)
		current_render_owner(pdev) = vgt_dom0;
	else
		vgt_ctx_switch = 0;
	current_display_owner(pdev) = vgt_dom0;
	current_pm_owner(pdev) = vgt_dom0;
	current_mgmt_owner(pdev) = vgt_dom0;
	pdev->ctx_check = 0;
	pdev->ctx_switch = 0;
	pdev->magic = 0;

	init_waitqueue_head(&pdev->wq);
	p_thread = kthread_run(vgt_thread, vgt_dom0, "vgt_thread");
	if (!p_thread) {
		goto err;
	}
	pdev->p_thread = p_thread;
	show_debug(pdev, 0);

	list_add(&pdev->list, &pgt_devices);

	/* FIXME: only support ONE vgt device now,
	 * you cannot call this function more than
	 * once
	 */
	//vgt_add_state_sysfs(vgt_dom0);
	vgt_init_sysfs(pdev);

#ifdef VGT_DEBUGFS_DUMP_FB
	/* There is anytime only one instance of the workqueue,
	 * and NON_REENTRANT
	 */
	pdev->pgt_wq = alloc_workqueue("vgt_workqueue",
			WQ_UNBOUND,
			1);
	if (!pdev->pgt_wq) {
		printk("vGT: failed to create kthread: vgt_workqueue.\n");
		goto err;
	}
#endif

	printk("vgt_initialize succeeds.\n");
	return 0;
err:
	printk("vgt_initialize failed.\n");
	vgt_destroy();
	return -1;
}

void vgt_destroy(void)
{
	struct list_head *pos, *next;
	struct vgt_device *vgt;
	struct pgt_device *pdev = &default_device;
	int i;

	list_del(&pdev->list);

	/* do we need the thread actually stopped? */
	kthread_stop(pdev->p_thread);

	vgt_irq_exit(pdev);

	/* Deactive all VGTs */
	while ( !list_empty(&pdev->rendering_runq_head) ) {
		list_for_each (pos, &pdev->rendering_runq_head)
			vgt_deactive(pdev, pos);
	};

#ifdef VGT_DEBUGFS_DUMP_FB
	/* Destruct pgt_wq */
	destroy_workqueue(pdev->pgt_wq);
#endif

	/* Destruct all vgt_debugfs */
	vgt_release_debugfs();

	intel_gtt_clear_range(0,
		(phys_aperture_sz(pdev) - GTT_PAGE_SIZE)/PAGE_SIZE);
	for (i = 0; i < phys_aperture_pages(pdev); i++)
		if (pages[i]) {
			put_page(pages[i]);
			__free_page(pages[i]);
		}

	if (pdev->gttmmio_base_va)
		iounmap(pdev->gttmmio_base_va);
	if (pdev->gmadr_va)
		iounmap(pdev->gmadr_va);
	while ( !list_empty(&pdev->rendering_idleq_head)) {
		for (pos = pdev->rendering_idleq_head.next;
			pos != &pdev->rendering_idleq_head; pos = next) {
			next = pos->next;
			vgt = list_entry (pos, struct vgt_device, list);
			vgt_release_instance(vgt);
		}
	}
	free_mtable();
	kfree(pdev->reg_info);
	if (pdev->vbios)
		__free_pages(pdev->vbios, get_order(VGT_VBIOS_PAGES));

	for (i = 0; i < EDID_NUM; ++ i) {
		if (pdev->pdev_edids[i]) {
			kfree(pdev->pdev_edids[i]);
			pdev->pdev_edids[i] = NULL;
		}
	}
}


