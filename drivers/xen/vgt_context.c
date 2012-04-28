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
#include <asm/bitops.h>
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_reg.h"

/* uncomment this macro so that dom0's aperture/GM starts from non-zero */
//#define DOM0_NON_IDENTICAL

/*
 * a temporary trick:
 *
 * uncomment this macro so that display/cursor bases are fixed with non-zero
 * base, while GPU rendering still happens to zero-based range. The trick is
 * to duplicate whole [0, 64M] GTT entries into [128M, 192M] range. All MMIO
 * addresses are fixed into [128M, 192M], except fence registers.
 */
//#define DOM0_NONIDEN_DISPLAY_ONLY

/*
 * NOTE list:
 * 	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
 * 	  also the hooks in AGP driver
 * 	- GTT aperture and gfx memory size check (now hardcode from intel-gtt.c)
 * 	- need a check on "unsigned long" vs. "u64" usage
 * 	- need consider cache related issues, e.g. Linux/Windows may have different
 * 	  TLB invalidation mode setting, which may impact vGT's context switch logic
 * 	- Need another way to ensure ring commands finished. Now check head==tail
 * 	- different GEN may have different register address. Caution to support IVB
 */
bool vgt_restore_context (struct vgt_device *vgt);
bool vgt_save_context (struct vgt_device *vgt);

unsigned int ring_mmio_base [MAX_ENGINES] = {
	/* must be in the order of ring ID definition */
	_REG_RCS_TAIL,
	_REG_VCS_TAIL,
	_REG_BCS_TAIL,
#ifndef SANDY_BRIDGE
	_REG_VECS_TAIL,	// HSW+
	_REG_VCS2_TAIL,	// BDW
#endif
};

submit_context_command_t submit_context_command[MAX_ENGINES] = {
	rcs_submit_context_command,
	default_submit_context_command,
	default_submit_context_command,
#ifndef SANDY_BRIDGE
	default_submit_context_command,
	default_submit_context_command,
#endif
};

LIST_HEAD(pgt_devices);

static struct pgt_device default_device = {
	.bus = 0,
	.devfn = 0x10,		/* BDF: 0:2:0 */
};

/* FIXME: move to pdev */
/* contains mask info for regs requiring addr fix */
vgt_addr_mask_t vgt_addr_table[VGT_ADDR_FIX_NUM];
int ai_index;

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
static void show_debug(struct pgt_device *pdev)
{
	vgt_reg_t reg;
	printk("debug registers:\n");
	printk("....EIR: %x\n", VGT_MMIO_READ(pdev, _REG_RCS_EIR));
	printk("....ESR: %x\n", VGT_MMIO_READ(pdev, _REG_RCS_ESR));
	printk("....blit EIR: %x\n", VGT_MMIO_READ(pdev, _REG_BCS_EIR));
	printk("....blit ESR: %x\n", VGT_MMIO_READ(pdev, _REG_BCS_ESR));
	printk("....IPEHR(last executed inst): %x\n", VGT_MMIO_READ(pdev, 0x2068));
	reg = VGT_MMIO_READ(pdev, 0x2070);
	printk("....INSTPS (parser state): %x :\n", reg);
	printk("....ACTHD(active header): %x\n", VGT_MMIO_READ(pdev, 0x2074));
	printk("....DMA_FADD_P(current fetch DMA): %x\n", VGT_MMIO_READ(pdev, 0x2078));
	printk("....CSCMDOP (instruction DWORD): %x\n", VGT_MMIO_READ(pdev, 0x220C));
	printk("....CSCMDVLD (command buffer valid): %x\n", VGT_MMIO_READ(pdev, 0x2210));
	printk("(informative)\n");
	printk("....INSTDONE_1(FYI): %x\n", VGT_MMIO_READ(pdev, 0x206C));
	printk("....INSTDONE_2: %x\n", VGT_MMIO_READ(pdev, 0x207C));
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
	vgt_reg_t reg;

#define ENABLED_STR(val, bit)		\
	((val & bit) ? "enabled" : "disabled")
	reg = VGT_MMIO_READ(pdev, _REG_GFX_MODE);
	printk("vGT: GFX_MODE: (%x)\n", reg);
	printk("....(%x)Flush TLB invalidation mode: %s\n",
		_REGBIT_FLUSH_TLB_INVALIDATION_MODE,
		ENABLED_STR(reg, _REGBIT_FLUSH_TLB_INVALIDATION_MODE));
	printk("....(%x)Replay mode: %x\n",
		_REGBIT_REPLAY_MODE, reg & _REGBIT_REPLAY_MODE);
	printk("....(%x)PPGTT: %s\n", _REGBIT_PPGTT,
		ENABLED_STR(reg, _REGBIT_PPGTT));

	reg = VGT_MMIO_READ(pdev, _REG_MI_MODE);
	printk("VGT: MI_MODE: (%x)\n", reg);
	printk("....(%x)Async Flip Performance mode: %s\n",
		_REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE,
		ENABLED_STR(reg, _REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE));
	printk("....(%x)Flush performance mode: %s\n",
		_REGBIT_MI_FLUSH_PERFORMANCE_MODE,
		ENABLED_STR(reg, _REGBIT_MI_FLUSH_PERFORMANCE_MODE));
	printk("....(%x)MI_FLUSH: %s\n",
		_REGBIT_MI_FLUSH,
		ENABLED_STR(reg, _REGBIT_MI_FLUSH));
	printk("....MI_FLUSH enable may be problematic on some "
	       "    some platforms, which should be abandoned\n"
	       "    On SNB it's parsed regardless of this bit\n");
	printk("....(%x)Invalidate UHPTR: %s\n",
		_REGBIT_MI_INVALIDATE_UHPTR,
		ENABLED_STR(reg, _REGBIT_MI_INVALIDATE_UHPTR));

	reg = VGT_MMIO_READ(pdev, _REG_ARB_MODE);
	printk("VGT: ARB_MODE: (%x)\n", reg);
	printk("....address swizzling: %s\n",
		(reg & _REGBIT_ADDRESS_SWIZZLING) ? "bit 6 used" : "no");
}

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
	ptr = (uint64_t)aperture_vbase(pdev) + context;
	printk("===================\n");
	printk("Context (%llx, %llx): %s\n", context, ptr, clobber ? "clobbered" : "");

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

/*
 * Global mode setting that vGT needs to ensure
 *
 * Now leave it empty
 */
static void enforce_mode_setting(struct pgt_device *pdev)
{
#if 0
	vgt_reg_t reg;
	reg = VGT_MMIO_READ(pdev, _REG_MI_MODE);
	if (!(reg & _REGBIT_MI_FLUSH)) {
		printk("(vGT): force enabling MI_FLUSH\n");
		reg |= (_REGBIT_MI_FLUSH << 16) | _REGBIT_MI_FLUSH;
		VGT_MMIO_WRITE(pdev, _REG_MI_MODE, reg);
		printk("(vGT): new value : %x\n", VGT_MMIO_READ(pdev, _REG_MI_MODE));
	}
#endif
}

#ifndef SINGLE_VM_DEBUG
struct vgt_device *next_display_owner;
atomic_t display_switched = ATOMIC_INIT(0);
#endif
static struct vgt_device *vgt_dom0;
struct mmio_hash_table	*mtable[MHASH_SIZE];
struct mmio_hash_table gtt_mmio_handler={
	.read = gtt_mmio_read,
	.write = gtt_mmio_write,
	.mmio_base = 0x200000,
};

static void _add_mtable(int index, struct mmio_hash_table *mht)
{
	if ( !mtable[index] ) {
		mtable[index] = mht;
		mht->next = NULL;
	}
	else {
		mht->next = mtable[index];
		mtable[index] = mht;
	}
}

static struct mmio_hash_table *lookup_mtable(int mmio_base)
{
	int index;
	struct mmio_hash_table *mht;

	if(mmio_base >= gtt_mmio_handler.mmio_base)
		return &gtt_mmio_handler;

	mmio_base &= ~3;
	index = mhash(mmio_base);
	mht = mtable[index];

	while (mht != NULL) {
		if (mht->mmio_base == mmio_base)
			return mht;
		else
			mht = mht->next;
	};
	return NULL;
}

static void free_mtable_chain(struct mmio_hash_table *chain)
{
	if (!chain) {
		free_mtable_chain(chain->next);
		kfree(chain);
	}
}

static void free_mtable(void)
{
	int i;

	for (i=0; i<MHASH_SIZE; i++) {
		if ( mtable[i] )
			free_mtable_chain(mtable[i]);
	}
	memset(mtable, 0, sizeof(mtable));
}

static void register_mhash_entry(struct mmio_hash_table *mht)
{
	int index = mhash(mht->mmio_base);

	ASSERT ((mht->mmio_base & 3) == 0);
	_add_mtable(index, mht);
}

bool vgt_register_mmio_handler(int start, int end,
	vgt_mmio_read read, vgt_mmio_write write)
{
	int i;
	struct mmio_hash_table *mht;

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
    unsigned int nvms, vgt_id;
    struct vgt_device *vgt;
    /* TODO: check if vgt_id_alloc_bitmap is ~0UL */
    nvms = ffz(vgt_id_alloc_bitmap);
    ASSERT(nvms <= VGT_MAX_VMS);
    for (vgt_id = 0; vgt_id < nvms; vgt_id++) {
        vgt = default_device.device[vgt_id];
        if (vgt->vm_id == vmid)
            return vgt;
    }
    return NULL;
}

int allocate_vgt_id(void)
{
	unsigned long bit_index;

	ASSERT((vgt_id_alloc_bitmap & VGT_ID_ALLOC_BITMAP)
		!= VGT_ID_ALLOC_BITMAP);

	do {
		bit_index = ffz (vgt_id_alloc_bitmap);
		ASSERT (bit_index < VGT_MAX_VMS);
		if (bit_index >= VGT_MAX_VMS)
			return -1;
	}
	while ( test_and_set_bit(bit_index, &vgt_id_alloc_bitmap) != 0);

	return bit_index;
}

void free_vgt_id(int vgt_id)
{
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

#if 0
	if (reg == 0x7019C) {
		printk("vGT: capture DSPA setting (%x)\n", g_value);
		return g_value;
	}
#endif

	dprintk("vGT: address fix g->h for reg (%lx)(%x)\n", reg, g_value);
	mask = vgt_addr_table[reg_addr_index(pdev, reg)];
	/* FIXME: there may have some complex mask pattern */
	h_value = g2h_gm(vgt, g_value & mask);
	dprintk("....(g)%x->(h)%x\n", g_value, (h_value & mask) | (g_value & ~mask));

#ifdef DOM0_NONIDEN_DISPLAY_ONLY
	/* a workaround to test display part, before command parser is ready */
	if (reg == _REG_DSPASURF || reg == _REG_CURABASE) {
		int g_index = GTT_INDEX(pdev, (g_value & mask));
		int h_index = GTT_INDEX(pdev, h_value);
		int i;

		dprintk("index (%x)(%x), value (%x)(%x)\n", g_index, h_index,
			vgt_read_gtt(pdev, g_index),
			vgt_read_gtt(pdev, h_index));
		dprintk("content at 0x0: %lx\n", *(unsigned long *)((char *)aperture_vbase(pdev) + 0x0));
		dprintk("content at 0x%x: %lx\n", g_value & mask, *(unsigned long *)((char *)aperture_vbase(pdev) + (g_value & mask)));
		dprintk("content at 0x%x: %lx\n", h_value & mask, *(unsigned long *)((char *)aperture_vbase(pdev) + (h_value & mask)));
		dprintk("DSPATILEOFF: %x, DSPLINOFF: %x, SIZE: %x\n",
			VGT_MMIO_READ(pdev, _REG_DSPATILEOFF), VGT_MMIO_READ(pdev, _REG_DSPALINOFF),
			VGT_MMIO_READ(pdev, 0x70190));
		dprintk("_REG_DSPACNTR: %x, _REG_DSPASURFLIVE: %x\n",
			VGT_MMIO_READ(pdev, _REG_DSPACNTR), VGT_MMIO_READ(pdev, _REG_DSPASURFLIVE));
		/*
		 * duplicate GTT entries to the same memory page
		 * of course it may only work for display part, while 3D objects
		 * are allocated dynamically in the middle
		 *
		 * now cover 16M
		 */
		for (i = 0; i < 4096 * 4; i++)
			vgt_write_gtt(pdev, h_index - g_index + i, vgt_read_gtt(pdev, i));

		dprintk("index (%x)(%x), value (%x)(%x)\n", g_index, h_index,
			vgt_read_gtt(pdev, g_index),
			vgt_read_gtt(pdev, h_index));
		dprintk("DSPATILEOFF: %x, DSPLINOFF: %x, SIZE: %x\n",
			VGT_MMIO_READ(pdev, _REG_DSPATILEOFF), VGT_MMIO_READ(pdev, _REG_DSPALINOFF),
			VGT_MMIO_READ(pdev, 0x70190));
		dprintk("DSPATILEOFF: %x, DSPLINOFF: %x\n",
			VGT_MMIO_READ(pdev, _REG_DSPATILEOFF), VGT_MMIO_READ(pdev, _REG_DSPALINOFF));
		dprintk("_REG_DSPACNTR: %x, _REG_DSPASURFLIVE: %x\n",
			VGT_MMIO_READ(pdev, _REG_DSPACNTR), VGT_MMIO_READ(pdev, _REG_DSPASURFLIVE));
		dprintk("content at 0x0: %lx\n", *(unsigned long *)((char *)aperture_vbase(pdev) + 0x0));
		dprintk("content at 0x%x: %lx\n", g_value & mask, *(unsigned long *)((char *)aperture_vbase(pdev) + (g_value & mask)));
		dprintk("content at 0x%x: %lx\n", h_value & mask, *(unsigned long *)((char *)aperture_vbase(pdev) + (h_value & mask)));
	}
#endif
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
	mask = vgt_addr_table[reg_addr_index(pdev, reg)];

	/* FIXME: it's possible the initial state may not contain valid address */
	if (!h_gm_is_visible(vgt, h_value & mask) && !h_gm_is_hidden(vgt, h_value & mask)) {
		printk("!!!vGT: reg (%lx) doesn't contain a valid host address (%x)\n", reg, h_value);
		return h_value;
	}

	/* FIXME: there may have some complex mask pattern */
	g_value = h2g_gm(vgt, h_value & mask);
	dprintk("....(h)%x->(g)%x\n", h_value, (g_value & mask) | (h_value & ~mask));
	return (g_value & mask) | (h_value & ~mask);
}

/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
static void show_ringbuffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	vgt_reg_t p_tail, p_head, p_start;
	char *p_contents;
	int i;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(ring_id));
	p_start = VGT_MMIO_READ(pdev, RB_START(ring_id));
	printk("ring buffer(%d): head (%x) tail(%x), start(%x)\n", ring_id,
		p_head, p_tail, p_start);

	p_head &= RB_HEAD_OFF_MASK;
	p_contents = aperture_vbase(pdev) + p_start + p_head;
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
}

static unsigned long vgt_get_reg(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/* check whether to update vreg from HW */
//	if (reg_hw_update(pdev, reg) &&
	if (reg_pt(pdev, reg) &&
	    (vgt_ops->boot_time || reg_is_owner(vgt, reg))) {
		__sreg(vgt, reg) = VGT_MMIO_READ(pdev, reg);
		__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	}

	return __vreg(vgt, reg);
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
	struct pgt_device *pdev = vgt->pdev;
	/* check whether to update vreg from HW */
//	if (reg_hw_update(pdev, reg) &&
	if (reg_pt(pdev, reg) &&
	    (vgt_ops->boot_time || reg_is_owner(vgt, reg))) {
		__sreg64(vgt, reg) = VGT_MMIO_READ_BYTES(pdev, reg, 8);
		__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
		__vreg(vgt, reg + 4) = mmio_h2g_gmadr(vgt, reg + 4, __sreg(vgt, reg + 4));
	}

	return __vreg64(vgt, reg);
}

static void vgt_update_reg(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	if (reg_pt(pdev, reg)) {
		__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));

		if (vgt_ops->boot_time || reg_is_owner(vgt, reg))
			VGT_MMIO_WRITE(pdev, reg, __sreg(vgt, reg));
	}
}

static void vgt_update_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	if (reg_pt(pdev, reg)) {
		__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
		__sreg(vgt, reg + 4) = mmio_g2h_gmadr(vgt, reg + 4, __vreg(vgt, reg + 4));

		if (vgt_ops->boot_time || reg_is_owner(vgt, reg))
			VGT_MMIO_WRITE_BYTES(pdev, reg, __sreg64(vgt, reg), 8);
	}
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
	unsigned int off2;
	unsigned long wvalue;
	unsigned int offset;
	unsigned long flags;

	offset = vgt_pa_to_mmio_offset(vgt, pa);

//#ifdef SINGLE_VM_DEBUG
	/* for single-VM UP dom0 case, no nest is expected */
	ASSERT(!spin_is_locked(&pdev->lock));
//#endif

//	ASSERT (offset + bytes <= vgt->state.regNum *
//				sizeof(vgt->state.vReg[0]));
	ASSERT (bytes <= 8);
	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);

	if (bytes > 4)
		dprintk("vGT: capture >4 bytes read to %x\n", offset);

	spin_lock_irqsave(&pdev->lock, flags);
	mht = lookup_mtable(offset);
	if ( mht && mht->read )
		mht->read(vgt, offset, p_data, bytes);
	else {
		off2 = offset & ~(bytes - 1);

		if (bytes <= 4) {
			wvalue = vgt_get_reg(vgt, off2);
		} else {
			wvalue = vgt_get_reg_64(vgt, off2);
		}

		memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);
	}

	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

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

	offset = vgt_pa_to_mmio_offset(vgt, pa);

//#ifdef SINGLE_VM_DEBUG
	ASSERT(!spin_is_locked(&pdev->lock));
//#endif
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
	mht = lookup_mtable(offset);
	if ( mht && mht->write )
		mht->write(vgt, offset, p_data, bytes);
	else {
		memcpy((char *)vgt->state.vReg + offset,
				p_data, bytes);

		offset &= ~(bytes - 1);
		if (bytes <= 4)
			vgt_update_reg(vgt, offset);
		else
			vgt_update_reg_64(vgt, offset);
	}

	if (offset == _REG_GFX_MODE || offset == _REG_MI_MODE || offset == _REG_ARB_MODE) {
		printk("vGT: write to global registers (%x)\n", offset);
		enforce_mode_setting(vgt->pdev);
		show_mode_settings(vgt->pdev);
	}

	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

bool is_rendering_engine_empty(struct pgt_device *pdev, int ring_id)
{
	if ( is_ring_enabled(pdev, ring_id) && !is_ring_empty(pdev, ring_id) )
		return false;

	return true;
}

bool is_context_switch_done(struct pgt_device *pdev, int ring_id)
{
	u32 *ptr;

	ptr = (u32 *)(aperture_vbase(pdev) + vgt_data_ctx_magic(pdev));
	if (*ptr != pdev->magic)
		return false;

	return true;
}

/*
 * Wait for the empty of RB.
 * TODO: Empty of RB doesn't mean the commands are retired. May need a STORE_IMM
 * after MI_FLUSH, but that needs our own hardware satus page.
 */
static bool ring_wait_for_empty(struct pgt_device *pdev, int ring_id, bool ctx_switch)
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
				ctx_switch ? "ctx-switch" : "wait-empty",
				count / 1000000, ring_id);
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
	for (i=0; i < MAX_ENGINES; i++)
		if ( !ring_wait_for_empty(pdev, i, false) ) {
			*ring_id = i;
			return false;
		}
	return true;
}

#ifndef SINGLE_VM_DEBUG
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
#endif

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

/*
 * random GTT entry check
 */
static void check_gtt(struct pgt_device *pdev)
{
	printk("GMADR: 0x00000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x00000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x00000000)));
	printk("GMADR: 0x02000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x02000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x02000000)));
	printk("GMADR: 0x04000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x04000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x04000000)));
	printk("GMADR: 0x08000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x08000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x08000000)));
	printk("GMADR: 0x0C000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x0C000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x0C000000)));
	printk("GMADR: 0x0FFFF000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x0FFFF000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x0FFFF000)));
	printk("GMADR: 0x10000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x10000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x10000000)));
	printk("GMADR: 0x10000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x10000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x10000000)));
	printk("GMADR: 0x20000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x20000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x20000000)));
	printk("GMADR: 0x40000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x20000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x40000000)));
	printk("GMADR: 0x60000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x60000000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x60000000)));
	printk("GMADR: 0x7ffff000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0x7ffff000),
		vgt_read_gtt(pdev, GTT_INDEX(pdev, 0x7ffff000)));
}

static int start_period = 10; /* in unit of second */
static int __init period_setup(char *str)
{
	start_period = simple_strtoul(str, NULL, 10);
	return 1;
}
__setup("vgt_start_period=", period_setup);

static int fastmode = 0;
static int __init mode_setup(char *str)
{
	fastmode = 1;
	return 1;
}
__setup("vgt_fastmode", mode_setup);

static int vgt_ctx_switch = 0;
static int __init ctx_switch_setup(char *str)
{
	vgt_ctx_switch = 1;
	return 1;
}
__setup("vgt_ctx_switch", ctx_switch_setup);

static int period = 5*HZ;	/* default slow mode */
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

	ASSERT(current_render_owner(pdev));
	printk("vGT: start kthread for dev (%x, %x)\n", pdev->bus, pdev->devfn);
	if (fastmode) {
		printk("vGT: fastmode switch (in 50ms)\n");
		period = HZ/20;
		threshold = 200;
	}

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
		if (test_and_clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request))
			vgt_handle_virtual_interrupt(pdev, VGT_OT_INVALID);

		/* context switch timeout hasn't expired */
		if (wait)
			continue;

		wait = period;
		if (!(vgt_ctx_check(pdev) % threshold))
			printk("vGT: %lldth checks, %lld switches\n",
				vgt_ctx_check(pdev), vgt_ctx_switch(pdev));
		vgt_ctx_check(pdev)++;

#ifndef SINGLE_VM_DEBUG
		/* Response to the monitor switch request. */
		if (atomic_read(&display_switched)) {

			printk(KERN_WARNING"xuanhua: vGT: display switched\n");
			printk(KERN_WARNING"xuanhua: vGT: current display owner: %p; next display owner: %p\n",
				current_display_owner(pdev), next_display_owner);
			spin_lock_irq(&pdev->lock);
			vgt_irq_save_context(current_display_owner(pdev),
				VGT_OT_DISPLAY);
			vgt_switch_display_owner(current_display_owner(pdev),
					next_display_owner);
			previous_display_owner(pdev) = current_display_owner(pdev);
			current_display_owner(pdev) = next_display_owner;
			//next_display_owner = NULL;
			atomic_dec(&display_switched);
			vgt_irq_restore_context(next_display_owner, VGT_OT_DISPLAY);
			/*
			 * Virtual interrupts pending right after display switch
			 * Need send to both prev and next owner.
			 */
			if (pdev->request & VGT_REQUEST_IRQ) {
				clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
				vgt_handle_virtual_interrupt(pdev, VGT_OT_DISPLAY);
			}
			spin_unlock_irq(&pdev->lock);
		}
#endif

		if (list_empty(&pdev->rendering_runq_head)) {
			/* Idle now, and no pending activity */
			dprintk("....idle\n");
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
			next = next_vgt(&pdev->rendering_runq_head, vgt);
#ifndef SINGLE_VM_DEBUG
			if ( next != current_render_owner(pdev) )
#endif
			{
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

				previous_render_owner(pdev) = current_render_owner(pdev);
				current_render_owner(pdev) = next;
				vgt_irq_restore_context(next, VGT_OT_RENDER);
			}
#ifndef SINGLE_VM_DEBUG
			else
				dprintk("....no other instance\n");
#endif
		} else {
			printk("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
				vgt_ctx_switch(pdev), ring_id,
				current_render_owner(pdev)->vgt_id);
			show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		}
		spin_unlock_irq(&pdev->lock);
#ifndef SINGLE_VM_DEBUG
		/* Virtual interrupts pending right after render switch */
		if (pdev->request & VGT_REQUEST_IRQ) {
			spin_lock_irq(&pdev->lock);
			clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
			vgt_handle_virtual_interrupt(pdev, VGT_OT_RENDER);
			spin_unlock_irq(&pdev->lock);
		}
#endif
	}
	return 0;
}

/*
 * Only need to save head, since it's the only one updated by hw
 */
void ring_phys_2_shadow(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	dprintk("old head(%x), old tail(%x)\n", srb->head, srb->tail);
	srb->head = VGT_MMIO_READ(pdev, RB_HEAD(ring_id));
	dprintk("new head(%x), new tail(%x)\n", srb->head, VGT_MMIO_READ(pdev, RB_TAIL(ring_id)));
#if 0
	srb->tail = VGT_MMIO_READ(pdev, &prb->tail);
	srb->start = VGT_MMIO_READ(pdev, &prb->start);
	srb->ctl = VGT_MMIO_READ(pdev, &prb->ctl);
#endif
}

/* Rewind the head/tail registers */
void rewind_ring(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	VGT_MMIO_WRITE(pdev, RB_TAIL(ring_id), srb->tail);
	VGT_MMIO_WRITE(pdev, RB_HEAD(ring_id), srb->head);
}

/*
 * write to head is undefined when ring is enabled.
 *
 * so always invoke this disable action when recovering a new ring setting
 */
static inline void disable_ring(struct pgt_device *pdev, int ring_id)
{
	VGT_MMIO_WRITE(pdev, RB_CTL(ring_id), 0);
	/* by ktian1. no source for this trick */
	VGT_POST_READ(pdev, RB_CTL(ring_id));
}

/*
 * to restore to a new ring buffer, we need restore all ring regs including head.
 */
void ring_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	dprintk("shadow 2 phys: [%x, %x]\n", srb->head, srb->tail);

	ASSERT(srb->ctl & _RING_CTL_ENABLE);
	disable_ring(pdev, ring_id);
	VGT_MMIO_WRITE(pdev, RB_TAIL(ring_id), srb->tail);
	VGT_MMIO_WRITE(pdev, RB_HEAD(ring_id), srb->head);
	VGT_MMIO_WRITE(pdev, RB_START(ring_id), srb->start);
	VGT_MMIO_WRITE(pdev, RB_CTL(ring_id), srb->ctl);
	VGT_POST_READ(pdev, RB_CTL(ring_id)); /* by ktian1 */

	dprintk("shadow 2 phys: [%x, %x]\n", VGT_MMIO_READ(pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(ring_id)));
}

/*
 * A pre-step to restore a new ring buffer. We can't restore both head/tail pointers,
 * if ctl reg is enabled, or else hw will start parsing it before we actually restore
 * the context. This pre-step restores to the new ring buffer, but with head==tail
 */
void ring_pre_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	dprintk("pre shadow 2 phys: [%x, %x]\n", srb->head, srb->tail);

	ASSERT(srb->ctl & _RING_CTL_ENABLE);
	disable_ring(pdev, ring_id);
	VGT_MMIO_WRITE(pdev, RB_TAIL(ring_id), srb->head);
	VGT_MMIO_WRITE(pdev, RB_HEAD(ring_id), srb->head);
	VGT_MMIO_WRITE(pdev, RB_START(ring_id), srb->start);
	VGT_MMIO_WRITE(pdev, RB_CTL(ring_id), srb->ctl);
	VGT_POST_READ(pdev, RB_CTL(ring_id));	/* by ktian1 */

	dprintk("pre shadow 2 phys: [%x, %x]\n", VGT_MMIO_READ(pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(ring_id)));
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
	rbtail = rb->sring.tail; /* in byte unit */

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;
	dprintk("p_contents: %lx, rbtail: %x, ring_size: %x, to_tail: %x, start: %x\n",
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
	/* reset to the tail for every load */
	rbtail = rb->phys_tail = rb->sring.tail; /* in byte unit */

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;
	dprintk("p_contents: %lx, rbtail: %x, ring_size: %x, to_tail: %x, start: %x\n",
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
			aperture_vbase(vgt->pdev),
			(char*)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void restore_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	dprintk("<vgt-%d>restore ring buffer\n", vgt->vgt_id);
	ring_load_commands (&vgt->rb[ring_id],
			aperture_vbase(vgt->pdev),
			(char *)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

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

/*
 * TODO:
 * MI_SUSPEND_FLUSH is necessary for GEN5, but not SNB.
 * IVB furthers requires MI_ARB_ON_OFF to disable preemption
 * (from intel-gfx community)
 */
static rb_dword	cmds_save_context[] = {
	MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP,
	MI_STORE_DATA_IMM | MI_SDI_USE_GTT, MI_NOOP, MI_NOOP, MI_NOOP,
	MI_NOOP,
};

/* TODO: MI_FORCE_RESTORE is only required for initialization */
static rb_dword	cmds_restore_context[] = {
	MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_MM_SPACE_GTT | MI_FORCE_RESTORE,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP,
	MI_STORE_DATA_IMM | MI_SDI_USE_GTT, MI_NOOP, MI_NOOP, MI_NOOP,
	MI_NOOP,
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
		VGT_MMIO_READ(pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(ring_id)));

	dprintk("old magic number: %d\n",
		*(u32 *)(aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));
	ring_load_commands (rb, aperture_vbase(pdev), (char*)cmds, bytes);
	VGT_MMIO_WRITE(pdev, RB_TAIL(ring_id), rb->phys_tail);		/* TODO: Lock in future */
	//mdelay(1);

	if (!ring_wait_for_empty(pdev, ring_id, true)) {
		printk("vGT: context switch commands unfinished\n");
		show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		return false;
	}
	dprintk("new magic number: %d\n",
		*(u32 *)(aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));

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
	_REG_FENCE_0_LOW,
	_REG_FENCE_0_HIGH,
	_REG_FENCE_1_LOW,
	_REG_FENCE_1_HIGH,
	_REG_FENCE_2_LOW,
	_REG_FENCE_2_HIGH,
	_REG_FENCE_3_LOW,
	_REG_FENCE_3_HIGH,
	_REG_FENCE_4_LOW,
	_REG_FENCE_4_HIGH,
	_REG_FENCE_5_LOW,
	_REG_FENCE_5_HIGH,
	_REG_FENCE_6_LOW,
	_REG_FENCE_6_HIGH,
	_REG_FENCE_7_LOW,
	_REG_FENCE_7_HIGH,
	_REG_FENCE_8_LOW,
	_REG_FENCE_8_HIGH,
	_REG_FENCE_9_LOW,
	_REG_FENCE_9_HIGH,
	_REG_FENCE_10_LOW,
	_REG_FENCE_10_HIGH,
	_REG_FENCE_11_LOW,
	_REG_FENCE_11_HIGH,
	_REG_FENCE_12_LOW,
	_REG_FENCE_12_HIGH,
	_REG_FENCE_13_LOW,
	_REG_FENCE_13_HIGH,
	_REG_FENCE_14_LOW,
	_REG_FENCE_14_HIGH,
	_REG_FENCE_15_LOW,
	_REG_FENCE_15_HIGH,

	_REG_RCS_HWSTAM,
	_REG_BCS_HWSTAM,
	_REG_VCS_HWSTAM,

	_REG_RCS_HWS_PGA,
	_REG_BCS_HWS_PGA,
	_REG_VCS_HWS_PGA,

	_REG_RCS_INSTPM,
	_REG_BCS_INSTPM,
	_REG_VCS_INSTPM,

	_REG_RCS_EXCC,
	_REG_BCS_EXCC,
	_REG_VCS_EXCC,

	_REG_RCS_UHPTR,
	_REG_BCS_UHPTR,
	_REG_VCS_UHPTR,

	_REG_RCS_IMR,
	_REG_BCS_IMR,
	_REG_VCS_IMR,
};

static void vgt_setup_render_regs(struct pgt_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_NUM(vgt_render_regs); i++)
		reg_set_owner(pdev, vgt_render_regs[i], VGT_OT_RENDER);
}

/* TODO: lots of to fill */
vgt_reg_t vgt_display_regs[] = {
	_REG_FENCE_0_LOW,
	_REG_FENCE_0_HIGH,
	_REG_FENCE_1_LOW,
	_REG_FENCE_1_HIGH,
	_REG_FENCE_2_LOW,
	_REG_FENCE_2_HIGH,
	_REG_FENCE_3_LOW,
	_REG_FENCE_3_HIGH,
	_REG_FENCE_4_LOW,
	_REG_FENCE_4_HIGH,
	_REG_FENCE_5_LOW,
	_REG_FENCE_5_HIGH,
	_REG_FENCE_6_LOW,
	_REG_FENCE_6_HIGH,
	_REG_FENCE_7_LOW,
	_REG_FENCE_7_HIGH,
	_REG_FENCE_8_LOW,
	_REG_FENCE_8_HIGH,
	_REG_FENCE_9_LOW,
	_REG_FENCE_9_HIGH,
	_REG_FENCE_10_LOW,
	_REG_FENCE_10_HIGH,
	_REG_FENCE_11_LOW,
	_REG_FENCE_11_HIGH,
	_REG_FENCE_12_LOW,
	_REG_FENCE_12_HIGH,
	_REG_FENCE_13_LOW,
	_REG_FENCE_13_HIGH,
	_REG_FENCE_14_LOW,
	_REG_FENCE_14_HIGH,
	_REG_FENCE_15_LOW,
	_REG_FENCE_15_HIGH,

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

	for (i = 0; i < ARRAY_NUM(vgt_display_regs); i++)
		reg_set_owner(pdev, vgt_display_regs[i], VGT_OT_DISPLAY);
}

/* TODO: lots of to fill */
vgt_reg_t vgt_pm_regs[] = {
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
		reg_set_owner(pdev, vgt_mgmt_regs[i], VGT_OT_PM);
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
		if (reg_hw_update(vgt->pdev, reg)) {
			__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
			__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
			printk("....save mmio (%x) with (%x)\n", reg, __sreg(vgt, reg));
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
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		int reg = vgt_render_regs[i];
		dprintk("....restore mmio (%x) with (%x)\n", reg, __sreg(vgt, reg));
		/*
		 * FIXME: there's regs only with some bits updated by HW. Need
		 * OR vm's update with hw's bits?
		 */
		if (!reg_hw_update(vgt->pdev, reg))
			VGT_MMIO_WRITE(vgt->pdev, reg, __sreg(vgt, reg));
	}
}

/*
 * Rendering engine context switch
 *
 */

bool vgt_save_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int 			i;
	vgt_state_ring_t	*rb;
	bool rc = true;

	if (vgt == NULL)
		return false;

	disable_power_management(vgt);

	vgt_rendering_save_mmio(vgt);

	/* save rendering engines */
	for (i=0; i < MAX_ENGINES; i++) {
		rb = &vgt->rb[i];
		ring_phys_2_shadow (pdev, i, &rb->sring);

		sring_2_vring(vgt, i, &rb->sring, &rb->vring);

		/* for stateless engine, no need to save/restore context */
		if (rb->stateless)
			continue;

		/* save 32 dwords of the ring */
		save_ring_buffer (vgt, i);

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

		rc = (*submit_context_command[i]) (vgt, i, cmds_save_context,
				sizeof(cmds_save_context));
		restore_ring_buffer (vgt, i);

		if (rc)
			rb->initialized = true;
		dprintk("<vgt-%d>vgt_save_context done\n", vgt->vgt_id);

	}
	return rc;
}

bool vgt_restore_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_state_ring_t	*rb;
	bool rc;

	if (vgt == NULL)
		return false;

	vgt_addr_fix_restore();

	for (i=0; i < MAX_ENGINES; i++) {
		rb = &vgt->rb[i];

		/* stateless engine doesn't have this flag set */
		if (rb->initialized ) {	/* has saved context */
			//vring_2_sring(vgt, rb);
			ring_pre_shadow_2_phys (pdev, i, &rb->sring);

			/* save 32 dwords of the ring */
			save_ring_buffer (vgt, i);

#ifdef SINGLE_VM_DEBUG
			/*
			 * for single VM debug, we need a dummy context to make sure
			 * context save actually conducted
			 */
			dprintk("dummy switch\n");
			cmds_save_context[2] = MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN | MI_RESTORE_EXT_STATE_EN | 0xE000000;
			pdev->magic++;
			cmds_save_context[11] = pdev->magic;
			rc = (*submit_context_command[i]) (vgt, i, cmds_save_context,
				sizeof(cmds_save_context));

			/* reset the head/tail */
			ring_pre_shadow_2_phys (pdev, i, &rb->sring);

			if (!rc)
				goto err;

			dprintk("real switch\n");
#endif

			/*
			 * Save current context to prev's vGT area, and restore
			 * context from next's vGT area.
			 */
			switch (i) {
				case RING_BUFFER_RCS:
					cmds_restore_context[2] =
						rb->context_save_area |
						MI_MM_SPACE_GTT |
						MI_SAVE_EXT_STATE_EN |
						MI_RESTORE_EXT_STATE_EN |
						MI_FORCE_RESTORE;
					cmds_restore_context[10] = vgt_data_ctx_magic(pdev);
					pdev->magic++;
					cmds_restore_context[11] = pdev->magic;
					break;
				default:
					printk("vGT: unsupported engine (%d) switch \n", i);
					break;
			}
			rc = (*submit_context_command[i]) (vgt, i, cmds_restore_context,
				sizeof(cmds_restore_context));

			/* restore 32 dwords of the ring */
			restore_ring_buffer (vgt, i);

			if (!rc)
				goto err;
		}
	}

	vgt_rendering_restore_mmio(vgt);

	/* Restore ring registers */
	for (i=0; i < MAX_ENGINES; i++) {
		rb = &vgt->rb[i];
		/* vring->sring */
		//vring_2_sring(vgt, rb);
		ring_shadow_2_phys (pdev, i, &rb->sring);
	}

	/* Restore the PM */
	restore_power_management(vgt);
	dprintk("<vgt-%d>vgt_restore_context done\n", vgt->vgt_id);
	return true;
err:
	/* TODO: need fall back to original VM's context */
	return false;
}

/* TODO: figure out any security holes by giving the whole initial state */
static void state_reg_v2s(struct vgt_device *vgt)
{
	vgt_reg_t *vreg, *sreg;

	vreg = vgt->state.vReg;
	sreg = vgt->state.sReg;
	memcpy (sreg, vreg, VGT_MMIO_SPACE_SZ);

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
static bool create_state_instance(struct vgt_device *vgt)
{
	vgt_state_t	*state;

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
		return false;
	}
	return true;
}

/*
 * priv: VCPU ?
 */
struct vgt_device *create_vgt_instance(struct pgt_device *pdev, int vm_id)
{
	int i, vgt_id;
	struct vgt_device *vgt;
	vgt_state_ring_t	*rb;
	char *cfg_space;

	printk("create_vgt_instance\n");
	vgt = kmalloc (sizeof(*vgt), GFP_KERNEL);
	if (vgt == NULL) {
		printk("Insufficient memory for vgt_device in %s\n", __FUNCTION__);
		return NULL;
	}
	memset(vgt, 0, sizeof(*vgt));
	/* TODO: check format of aperture size */
	vgt_id = allocate_vgt_id();
	if (vgt_id < 0) {
		kfree (vgt);
		return NULL;
	}
	vgt->vgt_id = vgt_id;
	vgt->vm_id = vm_id;

	vgt->state.regNum = VGT_MMIO_REG_NUM;
	INIT_LIST_HEAD(&vgt->list);

	if ( !create_state_instance(vgt) ) {
		free_vgt_id(vgt_id);
		kfree (vgt);
		return NULL;
	}

	/* present aperture to the guest at the same host address */
	vgt->state.aperture_base = aperture_base(pdev);

	/* init aperture/gm ranges allocated to this vgt */
	if (vgt->vgt_id == 0) {
		vgt->aperture_base = dom0_aperture_base(pdev);
		vgt->aperture_sz = dom0_aperture_sz(pdev);
		vgt->gm_sz = dom0_aperture_sz(pdev);
		vgt->hidden_gm_offset = gm_sz(pdev);	/* dom0 has no hidden part */
	} else {
		/*
		 * TODO: Use sysfs for dynamic configuration.
		 */
		vgt->aperture_base = get_vm_aperture_base(pdev, vgt->vgt_id);
		vgt->aperture_sz = vm_aperture_sz(pdev);
		vgt->gm_sz = vm_gm_sz(pdev);
		vgt->hidden_gm_offset = get_vm_hidden_gm_base(pdev, vgt->vgt_id);
	}

	vgt->aperture_offset = aperture_2_gm(pdev, vgt->aperture_base);
	vgt->aperture_base_va = aperture_vbase(pdev) +
		vgt->aperture_offset;

	vgt->vgtt_sz = (vgt->gm_sz >> GTT_PAGE_SHIFT) * GTT_ENTRY_SIZE;
	printk("Virtual GTT size: 0x%lx\n", (long)vgt->vgtt_sz);
	vgt->vgtt = kzalloc(vgt->vgtt_sz, GFP_KERNEL);
	if (!vgt->vgtt) {
		printk("vGT: failed to allocate virtual GTT table\n");
		kfree(vgt->state.vReg);
		kfree(vgt->state.sReg);
		free_vgt_id(vgt_id);
		kfree (vgt);
		return NULL;
	}

	printk("Aperture: [%llx, %llx] guest [%llx, %llx] va(%llx)\n",
		vgt_aperture_base(vgt),
		vgt_aperture_end(vgt),
		vgt_guest_aperture_base(vgt),
		vgt_guest_aperture_end(vgt),
		(uint64_t)vgt->aperture_base_va);

	printk("GM: [%llx, %llx], [%llx, %llx], guest[%llx, %llx]\n",
		vgt_visible_gm_base(vgt),
		vgt_visible_gm_end(vgt),
		vgt_hidden_gm_base(vgt),
		vgt_hidden_gm_end(vgt),
		vgt_guest_gm_base(vgt),
		vgt_guest_gm_base(vgt) + vgt_gm_sz(vgt) - 1);

	vgt->rsvd_aperture_base = rsvd_aperture_base(pdev) + pdev->rsvd_aperture_pos;
	pdev->rsvd_aperture_pos += VGT_APERTURE_PER_INSTANCE_SZ;
	printk("rsvd_aperture_base: %llx\n", vgt->rsvd_aperture_base);

	for (i=0; i< MAX_ENGINES; i++) {
		rb = &vgt->rb[i];
		rb->context_save_area = aperture_2_gm(pdev, vgt->rsvd_aperture_base +
			i * SZ_CONTEXT_AREA_PER_RING);
		rb->initialized = false;
	}
	vgt->rb[RING_BUFFER_RCS].stateless = 0;
	vgt->rb[RING_BUFFER_VCS].stateless = 1;
	vgt->rb[RING_BUFFER_BCS].stateless = 1;

	vgt->state.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	vgt->state.bar_size[1] = vgt_aperture_sz(vgt);	/* Aperture */
	vgt->state.bar_size[2] = pdev->bar_size[2];	/* PIO */

	/* Set initial configuration space and MMIO space registers. */
	cfg_space = &vgt->state.cfg_space[0];
	memcpy (cfg_space, pdev->initial_cfg_space, VGT_CFG_SPACE_SZ);
	cfg_space[VGT_REG_CFG_SPACE_MSAC] = vgt->state.bar_size[1];
	*(uint32_t *)(cfg_space + VGT_REG_CFG_SPACE_BAR1) =
		vgt_guest_aperture_base(vgt) | 0x4;	/* 64-bit MMIO bar */

	memcpy (vgt->state.vReg, pdev->initial_mmio_state, VGT_MMIO_SPACE_SZ);
	vgt->pdev = pdev;
	state_reg_v2s (vgt);

	if (vgt_vstate_irq_init(vgt) != 0)
		return NULL;

	pdev->device[vgt->vgt_id] = vgt;
	list_add(&vgt->list, &pdev->rendering_idleq_head);

	/* TODO: do clean up if vgt_hvm_init() failed */
	if (vgt->vm_id != 0)
		vgt_hvm_info_init(vgt);

	if (vgt->vm_id && vgt_ops->boot_time)
		vgt_ops->boot_time = 0;

	return vgt;
}

void vgt_release_instance(struct vgt_device *vgt)
{
	struct list_head *pos;

	list_for_each (pos, &vgt->pdev->rendering_runq_head)
		if (pos == &vgt->list) {
			printk("Couldn't release an active vgt instance\n");
			return ;
		}
	/* The vgt may be still in executing. */
	while ( is_current_render_owner(vgt) )
		schedule();

	vgt_hvm_info_deinit(vgt);
	vgt->pdev->device[vgt->vgt_id] = NULL;

	vgt_vstate_irq_exit(vgt);
	/* already idle */
	list_del(&vgt->list);

	kfree(vgt->vgtt);
	kfree(vgt->state.vReg);
	kfree(vgt->state.sReg);
	free_vgt_id(vgt->vgt_id);
	kfree(vgt);
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
	dprintk("bar0: %llx, Bar1: %llx\n", bar0, bar1);

	ASSERT ((bar0 & 7) == 4);
	/* memory, 64 bits bar0 */
	pdev->gttmmio_base = bar0 & ~0xf;

	ASSERT ((bar1 & 7) == 4);
	/* memory, 64 bits bar */
	pdev->gmadr_base = bar1 & ~0xf;
	printk("gttmmio: %llx, gmadr:%llx\n", pdev->gttmmio_base, pdev->gmadr_base);

	/* TODO: no need for this mapping since hypercall is used */
	pdev->gttmmio_base_va = ioremap (pdev->gttmmio_base, 2 * VGT_MMIO_SPACE_SZ);
	if ( pdev->gttmmio_base_va == NULL ) {
		printk("Insufficient memory for ioremap1\n");
		return false;
	}
	printk("gttmmio_base_va: %llx\n", (uint64_t)pdev->gttmmio_base_va);

#if 1		// TODO: runtime sanity check warning...
	//pdev->phys_gmadr_va = ioremap (pdev->gmadr_base, VGT_TOTAL_APERTURE_SZ);
	pdev->gmadr_va = ioremap (pdev->gmadr_base, pdev->bar_size[1]);
	if ( pdev->gmadr_va == NULL ) {
		iounmap(pdev->gttmmio_base_va);
		printk("Insufficient memory for ioremap2\n");
		return false;
	}
	printk("gmadr_va: %llx\n", (uint64_t)pdev->gmadr_va);
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

	return true;
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

	vgt_set_addr_mask(pdev, _REG_RCS_UHPTR, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_VCS_UHPTR, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_BCS_UHPTR, 0xFFFFF000);


	vgt_set_addr_mask(pdev, _REG_RCS_BB_PREEMPT_ADDR, 0xFFFFF000);
	//vgt_set_addr_mask(pdev, _REG_RCS_BB_ADDR_DIFF, 0xFFFFF000);
	//vgt_set_addr_mask(pdev, _REG_RCS_BB_OFFSET, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_CCID, 0xFFFFF000);

	vgt_set_addr_mask(pdev, _REG_RCS_FBC_RT_BASE_ADDR, 0xFFFFF000);

	vgt_set_addr_mask(pdev, _REG_RCS_PP_DIR_BASE_READ, 0xFFFF0000);
	vgt_set_addr_mask(pdev, _REG_RCS_PP_DIR_BASE_WRITE, 0xFFFF0000);
	vgt_set_addr_mask(pdev, _REG_VCS_PP_DIR_BASE, 0xFFFF0000);
	vgt_set_addr_mask(pdev, _REG_BCS_PP_DIR_BASE, 0xFFFF0000);

#ifndef DOM0_NONIDEN_DISPLAY_ONLY
	/* FIXME: similarly, a valid bit exists */
	vgt_set_addr_mask(pdev, _REG_FENCE_0_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_0_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_1_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_1_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_2_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_2_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_3_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_3_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_4_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_4_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_5_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_5_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_6_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_6_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_7_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_7_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_8_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_8_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_9_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_9_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_10_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_10_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_11_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_11_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_12_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_12_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_13_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_13_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_14_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_14_HIGH, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_15_LOW, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_FENCE_15_HIGH, 0xFFFFF000);
#endif

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
	vgt_set_addr_mask(pdev, _REG_RCS_ACTHD, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_VCS_ACTHD, 0xFFFFF000);
	vgt_set_addr_mask(pdev, _REG_BCS_ACTHD, 0xFFFFF000);
}

static void vgt_setup_virt_regs(struct pgt_device *pdev)
{
}

/*
 * Is this really required? If HW just says unexpected behavior, should
 * we just allow it?
 */
static void vgt_setup_rdonly(struct pgt_device *pdev)
{
	reg_set_rdonly(pdev, _REG_RCS_BB_ADDR);
	reg_set_rdonly(pdev, _REG_VCS_BB_ADDR);
	reg_set_rdonly(pdev, _REG_BCS_BB_ADDR);

	reg_set_rdonly(pdev, _REG_RCS_BB_PREEMPT_ADDR);
	reg_set_rdonly(pdev, _REG_RCS_BB_ADDR_DIFF);

	reg_set_rdonly(pdev, _REG_RCS_PP_DIR_BASE_READ);
}

static void vgt_setup_hw_update_regs(struct pgt_device *pdev)
{
	//reg_set_hw_update(pdev, TIMESTAMP);
}

static bool vgt_initialize_pgt_device(struct pci_dev *dev, struct pgt_device *pdev)
{
	pdev->pdev = dev;
	pdev->pbus = dev->bus;

	INIT_LIST_HEAD(&pdev->rendering_runq_head);
	INIT_LIST_HEAD(&pdev->rendering_idleq_head);

	pdev->reg_info = kzalloc (VGT_MMIO_REG_NUM * sizeof(reg_info_t),
				GFP_KERNEL);
	if (!pdev->reg_info) {
		printk("vGT: failed to allocate reg_info\n");
		return false;
	}

	/* first setup the reg ownership mapping */
	vgt_setup_render_regs(pdev);
	vgt_setup_display_regs(pdev);
	vgt_setup_pm_regs(pdev);
	vgt_setup_mgmt_regs(pdev);

	/* then enable virt-only flag */
	vgt_setup_virt_regs(pdev);

	/* then setup read-only reg */
	vgt_setup_rdonly(pdev);

	/* then mark regs updated by hw */
	vgt_setup_hw_update_regs(pdev);

	/* then add addr fix info for pass-through regs */
	vgt_setup_addr_fix_info(pdev);
	return true;
}

/* FIXME: allocate instead of static */
#define VGT_APERTURE_PAGES	VGT_RSVD_APERTURE_SZ >> GTT_PAGE_SHIFT
static struct page *pages[VGT_APERTURE_PAGES];
static struct page *dummy_page;
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
	printk("....dummy page (%llx, %llx)\n", page_to_phys(dummy_page), dma_addr);

	/* for debug purpose */
	memset(pfn_to_kaddr(page_to_pfn(dummy_page)), 0x77, PAGE_SIZE);

	/* clear all GM space, instead of only aperture */
	for (i = 0; i < gm_pages(pdev); i++)
		vgt_write_gtt(pdev, i, dma_addr);

	dprintk("content at 0x0: %lx\n", *(unsigned long *)((char *)aperture_vbase(pdev) + 0x0));
	dprintk("content at 0x64000: %lx\n", *(unsigned long *)((char *)aperture_vbase(pdev) + 0x64000));
	dprintk("content at 0x8064000: %lx\n", *(unsigned long *)((char *)aperture_vbase(pdev) + 0x8064000));

	check_gtt(pdev);
	printk("vGT: allocate vGT aperture\n");
	/* Fill GTT range owned by vGT driver */
	index = GTT_INDEX(pdev, aperture_2_gm(pdev, rsvd_aperture_base(pdev)));
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

/* FIXME: invoked by AGP GTT code */
static uint64_t tot_gm_size;
void vgt_update_gtt_info(uint64_t gm_size)
{
	printk("GTT: tell vGT about total gm_size: %llx\n", gm_size);
	tot_gm_size = gm_size;
}

unsigned long int vgt_dom0_aper_offset(void)
{
	/*FIXME: remove the hard code 128M */
#ifdef DOM0_NON_IDENTICAL
	return SIZE_1MB * 128;
#else
	return 0;
#endif
}

EXPORT_SYMBOL(vgt_dom0_aper_offset);

void vgt_calculate_max_vms(struct pgt_device *pdev)
{
	uint64_t avail_ap, avail_gm;
	int possible_ap, possible_gm, possible;
	int i;
	uint64_t dom0_start = aperture_base(pdev);

	if (!tot_gm_size) {
		printk("vGT: ZERO GM space !!!!\n");
		tot_gm_size = aperture_sz(pdev);
	}
	gm_sz(pdev) = tot_gm_size;
	printk("vGT: total aperture (%x), total GM space (%llx)\n",
		aperture_sz(pdev), gm_sz(pdev));

#ifndef DOM0_NON_IDENTICAL
	pdev->max_vms = 1;		/* dom0 only */
#else
	pdev->max_vms = VGT_MAX_VMS;
#endif

	rsvd_aperture_sz(pdev) = VGT_RSVD_APERTURE_SZ;
	dom0_aperture_sz(pdev) = VGT_DOM0_APERTURE_SZ;

	avail_ap = aperture_sz(pdev) - rsvd_aperture_sz(pdev) -
			dom0_aperture_sz(pdev);
	possible_ap = avail_ap / VGT_MIN_APERTURE_SZ;

	avail_gm = gm_sz(pdev) - rsvd_aperture_sz(pdev) -
			dom0_aperture_sz(pdev);
	possible_gm = avail_gm / VGT_MIN_GM_SZ;

	possible = (possible_ap < possible_gm) ? possible_ap : possible_gm;
	possible++;	/* count on dom0 */
	if (possible < pdev->max_vms) {
		printk("vGT: request to support %d VMs, but only %d VMs can be allowed\n",
			VGT_MAX_VMS, possible);
		pdev->max_vms = possible;
	}

	printk("vGT: support %d VMs:\n", pdev->max_vms);

	/* TODO: instead of using min size, calculate an optimal size for requested VMs */
	vm_aperture_sz(pdev) = VGT_MIN_APERTURE_SZ;
	vm_gm_sz(pdev) = VGT_MIN_GM_SZ;
	for (i = 0; i < pdev->max_vms - 1; i++) {
		printk("....VM#%d:\n", i);
		printk("........aperture: [%llx, %llx]\n",
			get_vm_aperture_base(pdev, i),
			get_vm_aperture_end(pdev, i));
		printk("........gm: [%llx, %llx], [%llx, %llx]\n",
			get_vm_visible_gm_base(pdev, i),
			get_vm_visible_gm_end(pdev, i),
			get_vm_hidden_gm_base(pdev, i),
			get_vm_hidden_gm_end(pdev, i));

		dom0_start = get_vm_aperture_end(pdev, i) + 1;
	}

	ASSERT(dom0_start + dom0_aperture_sz(pdev) +
		rsvd_aperture_sz(pdev) <=
		aperture_base(pdev) + aperture_sz(pdev));

	dom0_aperture_base(pdev) = dom0_start;
	printk("....dom0 aperture: [%llx, %llx]\n",
			dom0_aperture_base(pdev),
			dom0_aperture_end(pdev));

	rsvd_aperture_base(pdev) = dom0_aperture_end(pdev) + 1;
	printk("....reserved aperture: [%llx, %llx]\n",
			rsvd_aperture_base(pdev),
			rsvd_aperture_end(pdev) );

	/* serve as a simple linear allocator */
	pdev->rsvd_aperture_pos = 0;

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

	spin_lock_init(&pdev->lock);

	memset (mtable, 0, sizeof(mtable));

	if ( !vgt_initialize_pgt_device(dev, pdev) )
		goto err;
	if ( !vgt_initialize_mmio_hooks() )
		goto err;
	if ( !initial_phys_states(pdev) )
		goto err;

	vgt_calculate_max_vms(pdev);

	if ( vgt_irq_init(pdev) != 0)
		goto err;

	/* setup the scratch page for the context switch */
	pdev->scratch_page = aperture_2_gm(pdev, pdev->rsvd_aperture_base +
		pdev->rsvd_aperture_pos);
	printk("scratch page is allocated at gm(%llx)\n", pdev->scratch_page);
	/* reserve the 1st trunk for vGT's general usage */
	pdev->rsvd_aperture_pos += VGT_APERTURE_PER_INSTANCE_SZ;

	/* create domain 0 instance */
	vgt_dom0 = create_vgt_instance(pdev, 0);   /* TODO: */
	if (vgt_dom0 == NULL)
		goto err;
#ifndef SINGLE_VM_DEBUG
	pdev->owner[VGT_OT_DISPLAY] = vgt_dom0;
#endif
	dprintk("create dom0 instance succeeds\n");

	/* FIXME: not sure why? update MI_MODE at this point has no effect! */
	enforce_mode_setting(pdev);
	show_mode_settings(pdev);

	if (setup_gtt(pdev))
		goto err;

	if (xen_register_vgt_device(0, vgt_dom0) != 0) {
		xen_deregister_vgt_device(vgt_dom0);
		goto err;
	}

	current_render_owner(pdev) = vgt_dom0;
	current_display_owner(pdev) = vgt_dom0;
	current_pm_owner(pdev) = vgt_dom0;
	current_mgmt_owner(pdev) = vgt_dom0;
	pdev->ctx_check = 0;
	pdev->ctx_switch = 0;
	pdev->magic = 0;

	init_waitqueue_head(&pdev->wq);
	if (vgt_ctx_switch) {
		p_thread = kthread_run(vgt_thread, vgt_dom0, "vgt_thread");
		if (!p_thread) {
			xen_deregister_vgt_device(vgt_dom0);
			goto err;
		}
		pdev->p_thread = p_thread;
	}
	show_debug(pdev);

	list_add(&pdev->list, &pgt_devices);

    /* FIXME: only support ONE vgt device now,
     * you cannot call this function more than
     * once
     */
    //vgt_add_state_sysfs(vgt_dom0);
    vgt_init_sysfs(pdev);

	printk("vgt_initialize succeeds.\n");
	return 0;
err:
    printk("vgt_initialize failed.\n");
    vgt_destroy();
    return -1;
}

void vgt_destroy()
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

	intel_gtt_clear_range(0, aperture_sz(pdev) - GTT_PAGE_SIZE);
	for (i = 0; i < aperture_pages(pdev); i++)
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
}


/*
 * TODO: PIO BAR.
 */
