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
#include "vgt_reg.h"
#include "vgt_wr.c"

/*
 * NOTE list:
 * 	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
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

static struct pgt_device default_device = {
	.bus = 0,
	.devfn = 0x10,		/* BDF: 0:2:0 */
};

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
	printk("....EIR: %x\n", VGT_MMIO_READ(pdev, _REG_RDR_EIR));
	printk("....ESR: %x\n", VGT_MMIO_READ(pdev, _REG_RDR_ESR));
	printk("....blit EIR: %x\n", VGT_MMIO_READ(pdev, _REG_BLIT_EIR));
	printk("....blit ESR: %x\n", VGT_MMIO_READ(pdev, _REG_BLIT_ESR));
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
	ptr = (uint64_t)pdev->gmadr_va + context;
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
#define	INVLID_MONITOR_SW_REQ	(-1)	/* -1: means no request */
int next_monitor_owner;
#endif
static struct vgt_device *vgt_dom0;
struct mmio_hash_table	*mtable[MHASH_SIZE];

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
 */
vgt_reg_t g2h_gmadr(struct vgt_device *vgt, vgt_reg_t g_gm_addr)
{
	/* TODO: for offseting */
	return g_gm_addr;
}

/*
 * Host to guest GMADR (include aperture) converting.
 */
vgt_reg_t h2g_gmadr(struct vgt_device *vgt, vgt_reg_t h_gm_addr)
{
	/* TODO: for offseting */
	return h_gm_addr;
}

/*
 * ABANDON IN THE FUTURE!!!! we should use explict VGT_MMIO interfaces
 * to avoid unnecessary GP faults
 *
 * Get the VA of vgt guest aperture base.
 * (NOTES: Aperture base is equal to GMADR base)
 */
static inline char *__aperture(struct vgt_device *vgt)
{
	char *p_contents;

	p_contents = vgt_aperture_vbase(vgt);
	return p_contents;
}

/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
static void show_ringbuffer(struct vgt_device *vgt, int ring_id, int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[ring_id];
	vgt_reg_t p_tail, p_head;
	char *p_contents;
	int i;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(ring_id));
	printk("ring buffer: p[%x, %x], s[%x, %x]\n", p_head, p_tail,
		rb->sring.head, rb->sring.tail);

	p_head &= RB_HEAD_OFF_MASK;
	p_contents = __aperture(vgt) + rb->sring.start + p_head;
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

/*
 * Emulate the VGT MMIO register read ops.
 * Return : true/false
 * */
bool vgt_emulate_read(struct vgt_device *vgt, unsigned int offset, void *p_data,int bytes)
{
	struct mmio_hash_table *mht;
	struct pgt_device *pdev = vgt->pdev;
	int id;
	unsigned int flags=0, off2;
	unsigned long wvalue;

#ifdef SINGLE_VM_DEBUG
	/* for single-VM UP dom0 case, no nest is expected */
	ASSERT(!spin_is_locked(&pdev->lock));
#endif
	spin_lock(&pdev->lock);
	offset -= pdev->gttmmio_base;
	ASSERT (offset + bytes <= vgt->state.regNum *
				sizeof(vgt->state.vReg[0]));
	ASSERT (bytes <= 8);
	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);

	if (bytes > 4)
		dprintk("vGT: capture 8 bytes read to %x\n", offset);

	mht = lookup_mtable(offset);
	if ( mht && mht->read )
		mht->read(vgt, offset, p_data, bytes);
	else {
		off2 = offset & ~(bytes - 1);
		id = gpuRegIndex(off2);
		if ( id >= 0 )
			flags = gpuregs[id].flags;

		/*
		 * PIPE registers are pass thru, and need to come
		 * from real HW register.
		 */
		if (vgt_ops->boot_time ||
		    (flags & (I915_REG_FLAG_PIPE_A | I915_REG_FLAG_PIPE_B))
#ifndef SINGLE_VM_DEBUG
            && (vgt->vgt_id == curr_monitor_owner(pdev))
#endif
		) {
			/* need to update hardware */
			wvalue = VGT_MMIO_READ_BYTES(pdev, off2, bytes);
			/* FIXME: only address fix, not the whole reg val */
			if (flags & I915_REG_FLAG_GRAPHICS_ADDRESS)
				wvalue = h2g_gmadr(vgt, wvalue);

		} else {
			if (bytes <= 4)
				wvalue = (unsigned long)__vreg(vgt, off2);
			else
				wvalue = __vreg64(vgt, off2);
		}

		/* FIXME: also need to find other registers updaetd by HW, which should be passed through too */

		memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);
	}

	spin_unlock(&pdev->lock);
	return true;
}

vgt_reg_t mmio_address_v2p(
	struct vgt_device *vgt, int id, vgt_reg_t vreg, int shadow_only)
{
	unsigned int flags = 0;
	vgt_reg_t	sreg = vreg;

	if ( id >= 0 )
		flags = gpuregs[id].flags;

	/*
	 * We need to fix address for GRAPHICS register here.
	 * But we want to ignore the ring buffer register.
	 * The pipe registers must be passthru
	 */
	if (flags & I915_REG_FLAG_GRAPHICS_ADDRESS)
		sreg = g2h_gmadr(vgt, vreg);

	if (!shadow_only &&
		(flags & (I915_REG_FLAG_PIPE_A | I915_REG_FLAG_PIPE_B)))
		/* need to update hardware */
		VGT_MMIO_WRITE(vgt->pdev, gpuregs[id].offset, sreg);
	return sreg;
}

/*
 * Emulate the VGT MMIO register write ops.
 * Return : true/false
 * */
bool vgt_emulate_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct mmio_hash_table *mht;
	int id;

#ifdef SINGLE_VM_DEBUG
	ASSERT(!spin_is_locked(&pdev->lock));
#endif
	spin_lock(&pdev->lock);
	offset -= pdev->gttmmio_base;
	ASSERT (offset + bytes <= vgt->state.regNum *
				sizeof(vgt->state.vReg[0]));
	/* at least FENCE registers are accessed in 8 bytes */
	ASSERT (bytes <= 8);
	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);

	if (bytes > 4)
		dprintk("vGT: capture 8 bytes write to %x with val (%lx)\n", offset, *(unsigned long*)p_data);

	mht = lookup_mtable(offset);
	if ( mht && mht->write )
		mht->write(vgt, offset, p_data, bytes);
	else {
		vgt_reg_t	sreg;
		unsigned long	sreg_64;

		memcpy((char *)vgt->state.vReg + offset,
				p_data, bytes);
		offset &= ~(bytes - 1);
		id = gpuRegIndex(offset);

		if (bytes <= 4) {
			sreg = mmio_address_v2p (vgt, id, __vreg(vgt, offset), 0);
			if (vgt_ops->boot_time)
				__sreg(vgt, offset) = sreg;
			sreg_64 = (unsigned long)sreg;
		} else {
			/* FIXME: need a 64bit version of mmio_address_v2p */
			//sreg_64 = (unsigned long)mmio_address_v2p (vgt, id, __vreg(vgt, offset), 0);
			sreg_64 = (unsigned long)__vreg64(vgt, offset);
			if (vgt_ops->boot_time)
				__sreg64(vgt, offset) = sreg_64;
		}

		/*
		 * Before the 2nd VM is started, we think the system is in
		 * boot time, where we'd like all dom0's MMIO writes flushed
		 * to the hardware since vGT driver itself doesn't do the
		 * initialization work. After the boot phase, passed through
		 * MMIOs are switched at ownership switch
		 */
		if (vgt_ops->boot_time)
			VGT_MMIO_WRITE_BYTES(pdev, offset, sreg_64, bytes);

		/* TODO: figure out pass through registers */
	}

	if (offset == _REG_GFX_MODE || offset == _REG_MI_MODE || offset == _REG_ARB_MODE) {
		printk("vGT: write to global registers (%x)\n", offset);
		enforce_mode_setting(vgt->pdev);
		show_mode_settings(vgt->pdev);
	}

	spin_unlock(&pdev->lock);
	return true;
}

bool is_rendering_engine_empty(struct pgt_device *pdev, int ring_id)
{
	if ( is_ring_enabled(pdev, ring_id) && !is_ring_empty(pdev, ring_id) )
		return false;

	return true;
}

/*
 * Wait for the empty of RB.
 * TODO: Empty of RB doesn't mean the commands are retired. May need a STORE_IMM
 * after MI_FLUSH, but that needs our own hardware satus page.
 */
static bool ring_wait_for_empty(struct pgt_device *pdev, int ring_id, int timeout)
{
	bool r = true;

	/* wait to be completed: TO CHECK: WR uses CCID register */
	while (--timeout > 0 ) {
		if (is_rendering_engine_empty(pdev, ring_id))
			break;
		sleep_us(1);		/* 1us delay */
	}

	if (timeout <= 0)
		r = false;

	return r;
}

bool is_rendering_engines_empty(struct pgt_device *pdev, int timeout)
{
	int i;

	/*
	 * TODO: timeout for 3 engines are not synchronous. Need suspend
	 * command parser later
	 */
	for (i=0; i < MAX_ENGINES; i++)
		if ( !ring_wait_for_empty(pdev, i, timeout) )
			return false;
	return true;
}

#ifndef SINGLE_VM_DEBUG
/*
 * Request from user level daemon/IOCTL
 */
void vgt_request_monitor_owner_switch(int vgt_id)
{
	if (next_monitor_owner != curr_monitor_owner(pdev))
		next_monitor_owner = vgt_id;
}

/*
 * Do monitor owner switch.
 */
void vgt_switch_monitor_owner(int prev_id, int next_id)
{
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
	printk("GMADR: 0xC0000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0xC0000000), vgt_read_gtt(pdev, GTT_INDEX(pdev, 0xC0000000)));
	printk("GMADR: 0xC2000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0xC2000000), vgt_read_gtt(pdev, GTT_INDEX(pdev, 0xC2000000)));
	printk("GMADR: 0xC8000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0xC8000000), vgt_read_gtt(pdev, GTT_INDEX(pdev, 0xC8000000)));
	printk("GMADR: 0xCC000000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0xCC000000), vgt_read_gtt(pdev, GTT_INDEX(pdev, 0xCC000000)));
	printk("GMADR: 0xCFFFF000, GTT INDEX: %x, GTT VALUE: %x\n",
		GTT_INDEX(pdev, 0xCFFFF000), vgt_read_gtt(pdev, GTT_INDEX(pdev, 0xCFFFF000)));
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
 * It's possible for all 4 paths to touch vreg/sreg/hwreg:
 *   a) the vgt thread may need to update HW updated regs into
 *      vreg/sreg of the prev owner
 *   b) the GP handler and event handler always updates vreg/sreg,
 *      and may touch hwreg if vgt is the current owner
 *   c) the interrupt handler touches hwreg to clear physical events,
 *      and then update vreg for interrupt virtualization
 *
 * To simplify the lock design, we make below assumptions:
 *   a) the vgt thread doesn't trigger GP fault itself, i.e. always
 *      issues hypercall to do hwreg access
 *   b) the event handler simply notifies another kernel thread, leaving
 *      to that thread for actual MMIO emulation
 *   c) the interrupt handler is registered as the interrupt thread
 *
 * Given above assumption, no nest would happen among 4 paths, and a
 * simple global spinlock now should be enough to protect the whole
 * vreg/sreg/ hwreg. In the future we can futher tune this part on
 * a necessary base.
 */
int vgt_thread(void *priv)
{
	struct vgt_device *next, *vgt = priv, *prev;
	struct pgt_device *pdev = vgt->pdev;
	static u64 cnt = 0, switched = 0;
	static int first = 0;
	int timeout = 100; /* microsecond */
	int threshold = 2; /* print every 10s */

	ASSERT(current_render_owner(pdev));
	printk("vGT: start kthread for dev (%x, %x)\n", pdev->bus, pdev->devfn);
	if (fastmode) {
		printk("vGT: fastmode switch (in 50ms)\n");
		period = HZ/20;
		threshold = 200;
	}

	while (!kthread_should_stop()) {
		/*
		 * TODO: Use high priority task and timeout based event
		 * 	mechanism for QoS. schedule in 50ms now.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		if (!first) {
			schedule_timeout(HZ*start_period);
			first = 1;
		} else
			schedule_timeout(period);

		if (!(cnt % threshold))
			printk("vGT: %lldth checks, %lld switches\n", cnt, switched);
		cnt++;
#ifndef SINGLE_VM_DEBUG
		/* Response to the monitor switch request. */
		if (next_monitor_owner != INVLID_MONITOR_SW_REQ) {
			/* has pending request */
			vgt_switch_monitor_owner(curr_monitor_owner(pdev),
					next_monitor_owner);
			curr_monitor_owner(pdev) = next_monitor_owner;
			next_monitor_owner = INVLID_MONITOR_SW_REQ;
		}
#endif

		if (list_empty(&pdev->rendering_runq_head)) {
			/* Idle now, and no pending activity */
			dprintk("....idle\n");
			continue;
		}

		/* TODO: need stop command parser from further adding content */

		if (is_rendering_engines_empty(pdev, timeout)) {
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
				spin_lock_irq(&pdev->lock);
				prev = current_render_owner(pdev);
				switched++;

				if (!vgt_save_context(prev)) {
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("vGT: (%lldth checks %lldth switch<%d->%d>): fail to save context\n",
						cnt, switched, prev->vgt_id, next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					break;
				}

				if (!vgt_restore_context(next)) {
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
					printk("vGT: (%lldth checks %lldth switch<%d->%d>): fail to restore context\n",
						cnt, switched, prev->vgt_id, next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					break;
				}

				current_render_owner(pdev) = next;
				spin_unlock_irq(&pdev->lock);
			}
#ifndef SINGLE_VM_DEBUG
			else
				dprintk("....no other instance\n");
#endif
		} else {
			printk("vGT: (%lldth switch<%d>)...ring is busy for %dus\n",
				switched, current_render_owner(pdev)->vgt_id, timeout);
			show_ringbuffer(vgt, 0, 16 * sizeof(vgt_reg_t));
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
	rb->sring.start = g2h_gmadr(vgt, rb->vring.start);

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
static void sring_2_vring(struct vgt_device *vgt,
	vgt_ringbuffer_t *sr, vgt_ringbuffer_t *vr)
{
	/* Fix address */
	vr->head = h2g_gmadr(vgt, sr->head);
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
			__aperture(vgt),
			(char*)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void restore_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	dprintk("<vgt-%d>restore ring buffer\n", vgt->vgt_id);
	ring_load_commands (&vgt->rb[ring_id],
			__aperture(vgt),
			(char *)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void disable_power_management(struct vgt_device *vgt)
{
	vgt_reg_t val;
	/* Save the power state and froce wakeup. */
	vgt->saved_wakeup = VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);
	VGT_MMIO_WRITE(vgt->pdev, I915_REG_FORCEWAKE_OFFSET, 1);
	val = VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);	/* why this ? */
}

static void restore_power_management(struct vgt_device *vgt)
{
	vgt_reg_t val;
	/* Restore the saved power state. */
	VGT_MMIO_WRITE(vgt->pdev, I915_REG_FORCEWAKE_OFFSET, vgt->saved_wakeup);
	val = VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);	/* why this ? */
}

/*
 * TODO:
 * MI_SUSPEND_FLUSH is necessary for GEN5, but not SNB.
 * IVB furthers requires MI_ARB_ON_OFF to disable preemption
 * (from intel-gfx community)
 */
static rb_dword	cmds_save_context[8] =
	{MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP};

/* TODO: MI_FORCE_RESTORE is only required for initialization */
static rb_dword	cmds_restore_context[8] =
	{MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_MM_SPACE_GTT | MI_FORCE_RESTORE,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP};

#if 0
/*
 * CCID change doesn't implicate the finish of all the commands.
 *
 * don't use this interface
 */
static bool wait_ccid_to_renew(struct pgt_device *pdev, vgt_reg_t new_ccid)
{
	int	timeout;
	vgt_reg_t ccid;

	/* wait for the register to be updated */
	timeout = CCID_TIMEOUT_LIMIT;
	while (--timeout > 0 ) {
		ccid = VGT_MMIO_READ (pdev, _REG_CCID);
		if ((ccid & GTT_PAGE_MASK) == (new_ccid & GTT_PAGE_MASK))
			break;
		sleep_us(1);		/* 1us delay */
	}
	if (timeout <= 0) {
		printk("XXXX: Update CCID failed at %s %d %x %x\n",
			__FUNCTION__, __LINE__,	ccid, new_ccid);
		return false;
	}
	dprintk("XXXX: Update CCID successfully to %x\n", ccid);
	return true;
}
#endif

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
#if 0
	/* WR ignore extended state, and MBO bits, why ? */
	ccid =  ccid_addr | CCID_MBO_BITS |
		CCID_VALID | CCID_EXTENDED_STATE_SAVE_ENABLE;
	VGT_MMIO_WRITE(vgt->pdev, _REG_CCID, ccid);

	if ( !wait_ccid_to_renew (vgt->pdev, ccid) )
		return false;
#endif

	dprintk("before load [%x, %x]\n",
		VGT_MMIO_READ(pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(ring_id)));

	ring_load_commands (rb, __aperture(vgt), (char*)cmds, bytes);
	VGT_MMIO_WRITE(pdev, RB_TAIL(ring_id), rb->phys_tail);		/* TODO: Lock in future */
	//mdelay(1);

	if (!ring_wait_for_empty(pdev, ring_id, 100)) {
		printk("vGT: context switch commands unfinished\n");
		show_ringbuffer(vgt, ring_id, 16 * sizeof(vgt_reg_t));
		return false;
	}

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

void vgt_rendering_save_mmio(struct vgt_device *vgt)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int num = ARRAY_NUM(rendering_ctx_regs);
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		ASSERT (rendering_ctx_regs[i] < vgt->state.regNum * REG_SIZE);
		/* TODO: only update __sreg for registers updated by HW */
		__sreg(vgt, rendering_ctx_regs[i]) =
			VGT_MMIO_READ(vgt->pdev, rendering_ctx_regs[i]);

		dprintk("....save mmio (%x) with (%x)\n", rendering_ctx_regs[i],
			__sreg(vgt, rendering_ctx_regs[i]));
		/*
		 * TODO: Fix address.
		 */
		/*
		 * If a register may be updated by HW as well,
		 * how to coordinate w/ pure SW emulation?
		 */
		__vreg(vgt, rendering_ctx_regs[i]) =
			__sreg(vgt, rendering_ctx_regs[i]);
	}
}

/*
 * Rstore MMIO registers per rendering context.
 * (Not include ring buffer registers).
 */
void vgt_rendering_restore_mmio(struct vgt_device *vgt)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int num = ARRAY_NUM(rendering_ctx_regs);
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		dprintk("....restore mmio (%x) with (%x)\n", rendering_ctx_regs[i],
			__sreg(vgt, rendering_ctx_regs[i]));
		/* Address is fixed previously */
		VGT_MMIO_WRITE(vgt->pdev, rendering_ctx_regs[i],
			__sreg(vgt, rendering_ctx_regs[i]));
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
	/* disable Power */
	disable_power_management(vgt);

	/* save MMIO: IntelGpuRegSave in WR */
	vgt_rendering_save_mmio(vgt);

	/*
	 * FIXME: VCS and BCS has different context switch methods, relying on
	 * MI_ARB_CHECK? Now just limit to the rendering engine only.
	 */
	/* save rendering engines */
#if 0
	for (i=0; i < MAX_ENGINES; i++) {
#else
	for (i=0; i < 1; i++) {
#endif
		rb = &vgt->rb[i];
		ring_phys_2_shadow (pdev, i, &rb->sring);

		sring_2_vring(vgt, &rb->sring, &rb->vring);

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
				aperture_2_gm(pdev, rb->context_save_area);
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

#if 0
	for (i=0; i < MAX_ENGINES; i++) {
#else
	for (i=0; i < 1; i++) {
#endif
		rb = &vgt->rb[i];

		if (rb->initialized ) {	/* has saved context */
			//vring_2_sring(vgt, rb);
			ring_pre_shadow_2_phys (pdev, i, &rb->sring);

			/* save 32 dwords of the ring */
			save_ring_buffer (vgt, i);

			/*
			 * Save current context to prev's vGT area, and restore
			 * context from next's vGT area.
			 */
			switch (i) {
				case RING_BUFFER_RCS:
					cmds_restore_context[2] =
						aperture_2_gm(pdev, rb->context_save_area) |
						MI_MM_SPACE_GTT |
						MI_SAVE_EXT_STATE_EN |
						MI_RESTORE_EXT_STATE_EN |
						MI_FORCE_RESTORE;
					break;
				default:
					printk("vGT: unsupported engine (%d) switch \n", i);
					break;
			}
#ifdef SINGLE_VM_DEBUG
			/*
			 * for single VM debug, we need a dummy context to make sure
			 * context save actually conducted
			 */
			dprintk("dummy switch\n");
			cmds_save_context[2] = MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN | MI_RESTORE_EXT_STATE_EN | 0xE000000;
			rc = (*submit_context_command[i]) (vgt, i, cmds_save_context,
				sizeof(cmds_save_context));

			/* reset the head/tail */
			ring_pre_shadow_2_phys (pdev, i, &rb->sring);

			if (!rc)
				goto err;

			dprintk("real switch\n");
#endif
			rc = (*submit_context_command[i]) (vgt, i, cmds_restore_context,
				sizeof(cmds_restore_context));

			/* restore 32 dwords of the ring */
			restore_ring_buffer (vgt, i);

			if (!rc)
				goto err;
		}
	}
	/* MMIO restore: intelGpuRegRestore in WR */
	vgt_rendering_restore_mmio(vgt);

	/* Restore ring registers */
#if 0
	for (i=0; i < MAX_ENGINES; i++) {
#else
	for (i=0; i < 1; i++) {
#endif
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

static void state_reg_v2s(struct vgt_device *vgt)
{
	int i, off;
	vgt_reg_t *vreg, *sreg;

	vreg = vgt->state.vReg;
	sreg = vgt->state.sReg;
	memcpy (sreg, vreg, VGT_MMIO_SPACE_SZ);

	/* Address fix */
	for (i = 0; i < gpuRegEntries; i++) {
		off = gpuregs[i].offset;
		/* reuse vgt_emulate_write logic to fix address. */
		__sreg(vgt, off) =
			mmio_address_v2p (vgt, i, __vreg(vgt, off), 1);
		dprintk("vGT: address fix (%d) for reg (%x): (%x->%x)\n",
			i, off, __vreg(vgt, off), __sreg(vgt, off));

	}
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
struct vgt_device *create_vgt_instance(struct pgt_device *pdev)
{
	int i;
	struct vgt_device *vgt;
	vgt_state_ring_t	*rb;
	char *cfg_space;

	dprintk("create_vgt_instance\n");
	vgt = kmalloc (sizeof(*vgt), GFP_KERNEL);
	if (vgt == NULL) {
		printk("Insufficient memory for vgt_device in %s\n", __FUNCTION__);
		return NULL;
	}
	/* TODO: check format of aperture size */
#ifdef SINGLE_VM_DEBUG
	vgt->vgt_id = 0;
	vgt->vm_id = 0;
#else
	vgt_id = allocate_vgt_id();
	if (vgt_id < 0) {
		kfree (vgt);
		return NULL;
	}
	vgt->vgt_id = vgt_id;
	vgt->vm_id = ...;
#endif
	vgt->state.regNum = VGT_MMIO_REG_NUM;
	INIT_LIST_HEAD(&vgt->list);

	if ( !create_state_instance(vgt) ) {
#ifndef SINGLE_VM_DEBUG
		free_vgt_id(vgt_id);
#endif
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
		vgt->hidden_gm_offset = 0;	/* dom0 has no hidden part */
	} else {
		vgt->aperture_base = get_vm_aperture_base(pdev, vgt->vgt_id);
		vgt->aperture_sz = vm_aperture_sz(pdev);
		vgt->gm_sz = vm_gm_sz(pdev);
		vgt->hidden_gm_offset = get_vm_hidden_gm_base(pdev, vgt->vgt_id);
	}

	vgt->aperture_offset = aperture_2_gm(pdev, vgt->aperture_base);
	vgt->aperture_base_va = aperture_vbase(pdev) +
		vgt->aperture_offset;

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
		vgt_gm_sz(vgt));

	vgt->rsvd_aperture_base = rsvd_aperture_base(pdev) +
		vgt->vgt_id * VGT_APERTURE_PER_INSTANCE_SZ;
	printk("rsvd_aperture_base: %llx\n", vgt->rsvd_aperture_base);

	for (i=0; i< MAX_ENGINES; i++) {
		rb = &vgt->rb[i];
		rb->context_save_area = vgt->rsvd_aperture_base +
			i * SZ_CONTEXT_AREA_PER_RING;
		rb->initialized = false;
	}

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
	state_reg_v2s (vgt);

	vgt->pdev = pdev;
	list_add(&vgt->list, &pdev->rendering_idleq_head);
	/* TODO: per register special handling. */
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

	/* already idle */
	list_del(&vgt->list);

	kfree(vgt->state.vReg);
	kfree(vgt->state.sReg);
#ifndef SINGLE_VM_DEBUG
	free_vgt_id(vgt->vgt_id);
#endif
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

static void vgt_initialize_pgt_device(struct pci_dev *dev, struct pgt_device *pdev)
{
	pdev->pdev = dev;
	pdev->pbus = dev->bus;

	INIT_LIST_HEAD(&pdev->rendering_runq_head);
	INIT_LIST_HEAD(&pdev->rendering_idleq_head);
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
	for (i = 0; i < aperture_pages(pdev); i++)
		vgt_write_gtt(pdev, i, dma_addr);

	check_gtt(pdev);
	printk("vGT: allocate vGT aperture\n");
	/* Fill GTT range owned by vGT driver */
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
		index = GTT_INDEX(pdev, rsvd_aperture_base(pdev)) + i;
		vgt_write_gtt(pdev, index, dma_addr);

		if (!(i % (VGT_APERTURE_PAGES / 20)))
			printk("vGT: write GTT-%x phys: %llx, dma: %llx\n",
				index, page_to_phys(page), dma_addr);
	}

	check_gtt(pdev);
	/* any cache flush required here? */
	return 0;
err_out:
	for (i = 0; i < VGT_APERTURE_PAGES; i++)
		if (pages[i]) {
			put_page(pages[i]);
			__free_page(pages[i]);
		}

	return ret;
}

void vgt_calculate_max_vms(struct pgt_device *pdev)
{
	uint64_t avail_ap, avail_gm;
	int possible_ap, possible_gm, possible;
	int i;
	uint64_t dom0_start = aperture_base(pdev);

	printk("vGT: total aperture (%x), total GM space (%llx)\n",
		aperture_sz(pdev), gm_sz(pdev));

#ifdef SINGLE_VM_DEBUG
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
	if (possible < VGT_MAX_VMS) {
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
}

/*
 * Initialize the vgt driver.
 *  return 0: success
 *	-1: error
 */
int vgt_initialize(struct pci_dev *dev)
{
	int i;
	struct pgt_device *pdev = &default_device;
	struct task_struct *p_thread;

	spin_lock_init(&pdev->lock);

	memset (mtable, 0, sizeof(mtable));

	vgt_initialize_pgt_device(dev, pdev);
	if ( !vgt_initialize_mmio_hooks() )
		goto err;
	if ( !initial_phys_states(pdev) )
		goto err;

	vgt_calculate_max_vms(pdev);

	/* TODO: no need for this mapping since hypercall is used */
	for (i=0; i < MAX_ENGINES; i++) {
		pdev->ring_base_vaddr[i] =
			(vgt_ringbuffer_t *) _vgt_mmio_va(pdev, ring_mmio_base[i]);
		printk("ring_base_vaddr[%d]: %llx\n", i, (uint64_t)pdev->ring_base_vaddr[i]);
	}
	/* create domain 0 instance */
	vgt_dom0 = create_vgt_instance(pdev);   /* TODO: */
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
	p_thread = kthread_run(vgt_thread, vgt_dom0, "vgt_thread");
	if (!p_thread) {
		xen_deregister_vgt_device(vgt_dom0);
		goto err;
	}
	pdev->p_thread = p_thread;
	show_debug(pdev);

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

	/* do we need the thread actually stopped? */
	kthread_stop(pdev->p_thread);

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
}


/*
 * TODO: PIO BAR.
 */
