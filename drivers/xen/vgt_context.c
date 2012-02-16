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
#include <asm/bitops.h>
#include <xen/vgt.h>
#include "vgt_reg.h"
#include "vgt_wr.c"

/*
 * WORKAROUND list:
 * 	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
 * 	- GTT aperture and gfx memory size check (now hardcode from intel-gtt.c)
 */
void vgt_restore_context (struct vgt_device *vgt);
void vgt_save_context (struct vgt_device *vgt);

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
 * Gfx ownership
 */
static inline bool is_current_vgt(struct vgt_device *vgt)
{
	return vgt == current_render_owner(vgt->pdev);
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
 * Get the VA of vgt guest aperture base.
 * (NOTES: Aperture base is equal to GMADR base)
 */
static inline char *__aperture(struct vgt_device *vgt)
{
	char *p_contents;

	p_contents = vgt->aperture_base_va;
	/* TODO: check */
	p_contents += vgt->aperture_offset;	/* WR use "-" */
	return p_contents;
}

/*
 * Emulate the VGT MMIO register read ops.
 * Return : true/false
 * */
bool vgt_emulate_read(struct vgt_device *vgt, unsigned int offset, void *p_data,int bytes)
{
	struct mmio_hash_table *mht;
	int id;
	unsigned int flags=0, off2;
	vgt_reg_t wvalue;

	offset -= vgt->pdev->gttmmio_base;
	ASSERT (offset + bytes <= vgt->state.regNum *
				sizeof(vgt->state.vReg[0]));
	ASSERT (bytes <= 4);
	ASSERT ((offset & 3) + bytes <= 4);

{
static int i = 0;

if (i++ < 10)
	printk("vGT: captured read emulation for (%x)\n", offset);
}
	mht = lookup_mtable(offset);
	if ( mht && mht->read )
		mht->read(vgt, offset, p_data, bytes);
	else {
		off2 = offset & ~3;
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
			wvalue = VGT_MMIO_READ(vgt->pdev, off2);
			if (flags & I915_REG_FLAG_GRAPHICS_ADDRESS)
				wvalue = h2g_gmadr(vgt, wvalue);

		} else
		/* FIXME: any emulation required for PIPE to make it forward progress */
			wvalue = __vreg(vgt, off2);

		/* FIXME: also need to find other registers updaetd by HW, which should be passed through too */

		memcpy(p_data, &wvalue + (offset & 3), bytes);
	}
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
	struct mmio_hash_table *mht;
	int id;

	offset -= vgt->pdev->gttmmio_base;
	ASSERT (offset + bytes <= vgt->state.regNum *
				sizeof(vgt->state.vReg[0]));
	ASSERT (bytes <= 4);

{
static int i = 0;

if (i++ < 10)
	printk("vGT: captured write emulation for (%x)\n", offset);
}
	mht = lookup_mtable(offset);
	if ( mht && mht->write )
		mht->write(vgt, offset, p_data, bytes);
	else {
		vgt_reg_t	sreg;

		memcpy((char *)vgt->state.vReg + offset,
				p_data, bytes);
		offset &= ~3;
		id = gpuRegIndex(offset);


		sreg = mmio_address_v2p (vgt, id, __vreg(vgt, offset), 0);

		/*
		 * Before the 2nd VM is started, we think the system is in
		 * boot time, where we'd like all dom0's MMIO writes flushed
		 * to the hardware since vGT driver itself doesn't do the
		 * initialization work. After the boot phase, passed through
		 * MMIOs are switched at ownership switch
		 */
		if (vgt_ops->boot_time) {
			__sreg(vgt, offset) = sreg;
			VGT_MMIO_WRITE(vgt->pdev, offset, sreg);
		}

		/* TODO: figure out pass through registers */
	}


	return true;
}

bool is_rendering_engine_empty(struct pgt_device *pdev, int ring_id)
{
	vgt_ringbuffer_t	*prb;

	prb = pdev->ring_base_vaddr[ring_id];
	if ( is_ring_enabled(prb) && !is_ring_empty(prb) )
		return false;
	return true;
}

bool is_rendering_engines_empty(struct pgt_device *pdev)
{
	int i;

	for (i=0; i < VGT_MAX_VMS; i++)
		if ( !is_rendering_engine_empty(pdev, i) )
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
	if (next == head)
		return NULL;
	return list_entry(next, struct vgt_device, list);
}
/*
 * The thread to perform the VGT ownership switch.
 *
 */
int vgt_thread(void *priv)
{
	struct vgt_device *next, *vgt=priv, *prev;
	struct pgt_device *pdev = vgt->pdev;
	static u64 cnt = 0, switched = 0;

	while (!kthread_should_stop()) {
		/*
		 * TODO: Use high priority task and timeout based event
		 * 	mechanism for QoS. schedule in 50ms now.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		//schedule_timeout(HZ/20);
		schedule_timeout(HZ);

		cnt++;
		printk("vGT(%lld): check context switch\n", cnt);
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

		if ((current_render_owner(pdev) == NULL) &&
			list_empty(&pdev->rendering_runq_head)) {
			/* Idle now, and no pending activity */
			printk("....idle\n");
			continue;
		}

		/* TODO: need stop command parser from further adding content */

		if (is_rendering_engines_empty(pdev)) {
			next = next_vgt(&pdev->rendering_runq_head, vgt);
			if ( next && next != current_render_owner(pdev) ) {
				prev = current_render_owner(pdev);
				switched++;
				printk("....the %lldth switch (%d->%d)\n", switched, prev->vgt_id, next->vgt_id);

				vgt_save_context(prev);
				vgt_restore_context(next);
				current_render_owner(pdev) = next;
			}
		} else
			printk("....ring is busy\n");
	}
	return 0;
}

/*
 * Only need to save head, since it's the only one updated by hw
 */
void ring_phys_2_shadow(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_ringbuffer_t *prb = pdev->ring_base_vaddr[ring_id];

	srb->head = _REG_READ_(&prb->head);
#if 0
	srb->tail = _REG_READ_(&prb->tail);
	srb->start = _REG_READ_(&prb->start);
	srb->ctl = _REG_READ_(&prb->ctl);
#endif
}

/* Rewind the head/tail registers */
void rewind_ring(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_ringbuffer_t *prb = pdev->ring_base_vaddr[ring_id];

	_REG_WRITE_(&prb->tail, srb->tail);
	_REG_WRITE_(&prb->head, srb->head);
}

/*
 * to restore to a new ring buffer, we need restore all ring regs including head.
 */
void ring_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_ringbuffer_t *prb = pdev->ring_base_vaddr[ring_id];

	_REG_WRITE_(&prb->tail, srb->tail);
	_REG_WRITE_(&prb->head, srb->head);
	_REG_WRITE_(&prb->start, srb->start);
	_REG_WRITE_(&prb->ctl, srb->ctl);
}

/*
 * A pre-step to restore a new ring buffer. We can't restore both head/tail pointers,
 * if ctl reg is enabled, or else hw will start parsing it before we actually restore
 * the context. This pre-step restores to the new ring buffer, but with head==tail
 */
void ring_pre_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_ringbuffer_t *prb = pdev->ring_base_vaddr[ring_id];

	_REG_WRITE_(&prb->tail, srb->head);
	_REG_WRITE_(&prb->head, srb->head);
	_REG_WRITE_(&prb->start, srb->start);
	_REG_WRITE_(&prb->ctl, srb->ctl);
}

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
	rbtail = rb->phys_tail;	/* in byte unit */

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;

	if ( likely(to_tail >= bytes) )
	{
		memcpy (buf, p_contents + rbtail, bytes);
	}
	else {
		memcpy (buf, p_contents + rbtail, to_tail);
		memcpy (buf + to_tail, p_contents, bytes - to_tail);
	}
}

static void ring_load_commands(vgt_state_ring_t *rb,
	char *p_aperture, char *buf, int bytes)
{
	char	*p_contents;
	vgt_reg_t	rbtail;
	vgt_reg_t  ring_size, to_tail;	/* bytes */

	p_contents = p_aperture + rb->sring.start;
	rbtail = rb->phys_tail;

	ring_size = _RING_CTL_BUF_SIZE(rb->sring.ctl);
	to_tail = ring_size - rbtail;

	if ( likely(to_tail >= bytes) )
	{
		memcpy ((rb_dword *)p_contents + rbtail, buf, bytes);
		rb->phys_tail += bytes;
	} else {
		memcpy ((rb_dword *)p_contents + rbtail, buf, to_tail);
		memcpy (p_contents, buf + to_tail, bytes - to_tail);
		rb->phys_tail = bytes - to_tail;
	}
}

static inline void save_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	ring_save_commands (&vgt->rb[ring_id],
			__aperture(vgt),
			(char*)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void restore_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	ring_load_commands (&vgt->rb[ring_id],
			__aperture(vgt),
			(char *)vgt->rb[ring_id].save_buffer,
			sizeof(vgt->rb[ring_id].save_buffer));
}

static void disable_power_management(struct vgt_device *vgt)
{
	/* Save the power state and froce wakeup. */
	vgt->saved_wakeup = VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);
	VGT_MMIO_WRITE(vgt->pdev, I915_REG_FORCEWAKE_OFFSET, 1);
	VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);	/* why this ? */
}

static void restore_power_management(struct vgt_device *vgt)
{
	/* Restore the saved power state. */
	VGT_MMIO_WRITE(vgt->pdev, I915_REG_FORCEWAKE_OFFSET, vgt->saved_wakeup);
	VGT_MMIO_READ(vgt->pdev, I915_REG_FORCEWAKE_OFFSET);	/* why this ? */
}


static rb_dword	cmds_save_context[8] =
	{MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP};

static rb_dword	cmds_restore_context[8] =
	{MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN,
	MI_SET_CONTEXT, MI_MM_SPACE_GTT | MI_FORCE_RESTORE,
	MI_NOOP,
	MI_SUSPEND_FLUSH,
	MI_NOOP,
	MI_FLUSH,
	MI_NOOP};

/*
 * Wait for the empty of RB.
 * NOTES: Empty of RB doesn't mean the commands are retired.
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
	if (timeout <= 0) {
		printk("ring_wait_for_empty timeout\n");
		ASSERT(0);
		r = false;
	}
	return r;
}

static bool wait_ccid_to_renew(struct pgt_device *pdev, vgt_reg_t new_ccid)
{
	int	timeout;
	vgt_reg_t ccid;

	/* wait for the register to be updated */
	timeout = CCID_TIMEOUT_LIMIT;
	while (--timeout > 0 ) {
		ccid = VGT_MMIO_READ (pdev, _REG_CCID);
		if ((ccid & GPU_PAGE_MASK) == (new_ccid & GPU_PAGE_MASK))
			break;
		sleep_us(1);		/* 1us delay */
	}
	if (timeout <= 0) {
		printk("Update CCID failed at %s %d %xi %x\n",
			__FUNCTION__, __LINE__,	ccid, new_ccid);
		return false;
	}
	return true;
}

/*
 * Submit a series of context save/restore commands to ring engine,
 * and wait till it is executed.
 */
bool rcs_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes)
{
	vgt_state_ring_t	*rb;
//	vgt_reg_t	ccid;
	vgt_ringbuffer_t *prb = vgt->pdev->ring_base_vaddr[ring_id];

	//ASSERT ((ccid_addr & ~GPU_PAGE_MASK) == 0 );

	rb = &vgt->rb[ring_id];

	printk("vGT: CCID is %x, new cmd is %x\n",
		VGT_MMIO_READ(vgt->pdev, _REG_CCID), cmds[2]);
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

	ring_load_commands (rb, __aperture(vgt), (char*)cmds, bytes);
	_REG_WRITE_(&prb->tail, rb->phys_tail);		/* TODO: Lock in future */

	return wait_ccid_to_renew(vgt->pdev, cmds[2]);
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
		ASSERT (rendering_ctx_regs[i] < vgt->state.regNum);
		/* TODO: only update __sreg for registers updated by HW */
		__sreg(vgt, rendering_ctx_regs[i]) =
			VGT_MMIO_READ(vgt->pdev, rendering_ctx_regs[i]);
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
		/* Address is fixed previously */
		VGT_MMIO_WRITE(vgt->pdev, rendering_ctx_regs[i],
			__sreg(vgt, rendering_ctx_regs[i]));
	}
}

/*
 * Rendering engine context switch
 *
 */

void vgt_save_context (struct vgt_device *vgt)
{
	int 			i;
	vgt_state_ring_t	*rb;

	if (vgt == NULL)
		return;
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
		ring_phys_2_shadow (vgt->pdev, i, &rb->sring);
		/* cache the tail reg. */
		rb->phys_tail = rb->sring.tail;

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
			/* Does VM want the ext state to be saved? */
			cmds_save_context[2] = MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN | rb->context_save_area;
			break;
		default:
			printk("vGT: unsupported engine (%d) switch \n", i);
			break;
		}
		(*submit_context_command[i]) (vgt, i, cmds_save_context,
				sizeof(cmds_save_context));
		restore_ring_buffer (vgt, i);
		rb->initialized = true;
	}
}

void vgt_restore_context (struct vgt_device *vgt)
{
	int i;
	vgt_state_ring_t	*rb;

	if (vgt == NULL)
		return ;
	/* Restore rb registers */
#if 0
	for (i=0; i < MAX_ENGINES; i++) {
#else
	for (i=0; i < 0; i++) {
#endif
		rb = &vgt->rb[i];

		if (rb->initialized ) {	/* has saved context */
			//vring_2_sring(vgt, rb);
			ring_pre_shadow_2_phys (vgt->pdev, i, &rb->sring);
			rb->phys_tail = rb->sring.tail;

			/* save 32 dwords of the ring */
			save_ring_buffer (vgt, i);

			/*
			 * Save current context to prev's vGT area, and restore
			 * context from next's vGT area.
			 */
			switch (i) {
				case RING_BUFFER_RCS:
					cmds_restore_context[2] = rb->context_save_area |
						MI_MM_SPACE_GTT | MI_FORCE_RESTORE | MI_RESTORE_EXT_STATE_EN;
					break;
				default:
					printk("vGT: unsupported engine (%d) switch \n", i);
					break;
			}
			(*submit_context_command[i]) (vgt, i, cmds_restore_context,
				sizeof(cmds_restore_context));

			/* restore 32 dwords of the ring */
			restore_ring_buffer (vgt, i);
		}
	}
	/* MMIO restore: intelGpuRegRestore in WR */
	vgt_rendering_restore_mmio(vgt);

	/* Restore ring registers */
#if 0
	for (i=0; i < MAX_ENGINES; i++) {
#else
	for (i=0; i < MAX_ENGINES; i++) {
#endif
		rb = &vgt->rb[i];
		/* vring->sring */
		vring_2_sring(vgt, rb);
		ring_shadow_2_phys (vgt->pdev, i, &rb->sring);
	}

	/* Restore the PM */
	restore_power_management(vgt);
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
		printk("vGT: address fix (%d) for reg (%x): (%x->%x)\n",
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

printk("create_state_instance\n");
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
struct vgt_device *create_vgt_instance(struct pgt_device *pdev, void *priv)
{
	int i;
	struct vgt_device *vgt;
	vgt_state_ring_t	*rb;
	char *cfg_space;

printk("create_vgt_instance\n");
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
	vgt->priv = priv;
	vgt->state.regNum = VGT_MMIO_REG_NUM;
	INIT_LIST_HEAD(&vgt->list);
	list_add(&vgt->list, &pdev->rendering_idleq_head);

	if ( !create_state_instance(vgt) ) {
#ifndef SINGLE_VM_DEBUG
		free_vgt_id(vgt_id);
#endif
		kfree (vgt);
		return NULL;
	}

	if (vgt->vgt_id == 0) {	/* dom0 GFX driver */
		vgt->state.aperture_base_pa = pdev->gmadr_base +
				VGT_DOM0_GFX_APERTURE_BASE;
		vgt->aperture_base_va = pdev->phys_gmadr_va +
				VGT_DOM0_GFX_APERTURE_BASE;
//		vgt->state.gt_gmadr_base = ;
	} else {
		vgt->state.aperture_base_pa = pdev->gmadr_base +
				VGT_VM1_APERTURE_BASE +
				VGT_GUEST_APERTURE_SZ * (vgt->vgt_id-1);
		vgt->aperture_base_va = pdev->phys_gmadr_va +
				VGT_VM1_APERTURE_BASE +
				VGT_GUEST_APERTURE_SZ * (vgt->vgt_id-1);
	}
printk("aperture_base_pa: %llx, va: %llx\n", vgt->state.aperture_base_pa, (uint64_t)vgt->aperture_base_va);
	vgt->aperture_offset = 0;

	vgt->vgt_aperture_base = pdev->gmadr_base + VGT_APERTURE_BASE +
		vgt->vgt_id * VGT_APERTURE_PER_INSTANCE_SZ;
printk("vgt_aperture_base: %llx\n", vgt->vgt_aperture_base);

	for (i=0; i< MAX_ENGINES; i++) {
		rb = &vgt->rb[i];
		rb->context_save_area = vgt->vgt_aperture_base +
			i * SZ_CONTEXT_AREA_PER_RING;
		rb->initialized = false;
	}

	vgt->state.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	if (vgt->vgt_id == 0)
		vgt->state.bar_size[1] = VGT_DOM0_APERTURE_SZ;	/* GMADR */
	else
		vgt->state.bar_size[1] = VGT_GUEST_APERTURE_SZ;	/* GMADR */
	vgt->state.bar_size[2] = pdev->bar_size[2];	/* PIO */

	/* Set initial configuration space and MMIO space registers. */
	cfg_space = &vgt->state.cfg_space[0];
	memcpy (cfg_space, pdev->initial_cfg_space, VGT_CFG_SPACE_SZ);
	cfg_space[VGT_REG_CFG_SPACE_MSAC] = vgt->state.bar_size[1];
	*(uint32_t *)(cfg_space + VGT_REG_CFG_SPACE_BAR1) =
		vgt->state.aperture_base_pa | 0x4;	/* 64-bit MMIO bar */

	memcpy (vgt->state.vReg, pdev->initial_mmio_state, VGT_MMIO_SPACE_SZ);
	state_reg_v2s (vgt);

	vgt->pdev = pdev;
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
	while ( is_current_vgt(vgt) )
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
printk("read back bar_size %lx\n", bar_size);
	bar_size &= ~0xf;       /* bit 4-31 */
printk("read back bar_size1 %lx\n", bar_size);
	bar_size = 1 << find_first_bit(&bar_size, BITS_PER_LONG);
printk("read back bar_size2 %lx\n", bar_size);

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

printk("VGT: Initial_phys_states\n");
	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4)
		pci_read_config_dword(dev, i,
				(uint32_t *)&pdev->initial_cfg_space[i]);
	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4) {
		if (!(i % 16))
			printk("\n[%2x]: ", i);

		printk("%02x %02x %02x %02x ",
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
	dprintk("gttmmio: %llx, gmadr:%llx\n",
			pdev->gttmmio_base, pdev->gmadr_base);
	pdev->gttmmio_base_va = ioremap (pdev->gttmmio_base, 2 * VGT_MMIO_SPACE_SZ);
	if ( pdev->gttmmio_base_va == NULL ) {
		printk("Insufficient memory for ioremap1\n");
		return false;
	}
	pdev->gtt_base_va = pdev->gttmmio_base_va + VGT_MMIO_SPACE_SZ;
	printk("gttmmio_base_va: %llx, gtt_base_va, %llx\n", (uint64_t)pdev->gttmmio_base_va, (uint64_t)pdev->gtt_base_va);
#if 1		// TODO: runtime sanity check warning...
	//pdev->phys_gmadr_va = ioremap (pdev->gmadr_base, VGT_TOTAL_APERTURE_SZ);
	pdev->phys_gmadr_va = ioremap (pdev->gmadr_base, pdev->bar_size[1]);
	if ( pdev->phys_gmadr_va == NULL ) {
		iounmap(pdev->gttmmio_base_va);
		printk("Insufficient memory for ioremap2\n");
		return false;
	}
	printk("gmadr_va: %llx\n", (uint64_t)pdev->phys_gmadr_va);
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

/*
 * Initialize the vgt driver.
 *  return 0: success
 *	-1: error
 */
int vgt_initialize(struct pci_dev *dev)
{
	int i;
	struct pgt_device *pdev = &default_device;

	memset (mtable, 0, sizeof(mtable));

	vgt_initialize_pgt_device(dev, pdev);
	if ( !vgt_initialize_mmio_hooks() )
		goto err;
	if ( !initial_phys_states(pdev) )
		goto err;

	for (i=0; i < MAX_ENGINES; i++) {
		pdev->ring_base_vaddr[i] =
			(vgt_ringbuffer_t *) _vgt_mmio_va(pdev, ring_mmio_base[i]);
		printk("ring_base_vaddr[%d]: %llx\n", i, (uint64_t)pdev->ring_base_vaddr[i]);
	}
	/* create domain 0 instance */
	vgt_dom0 = create_vgt_instance(pdev, NULL);   /* TODO: */
	if (vgt_dom0 == NULL)
		goto err;
#ifndef SINGLE_VM_DEBUG
	pdev->owner[VGT_OT_DISPLAY] = vgt_dom0;
#endif
	dprintk("create dom0 instance succeeds\n");

    if (xen_register_vgt_device(0, vgt_dom0) != 0) {
        xen_deregister_vgt_device(vgt_dom0);
        goto err;
    }
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

	/* Deactive all VGTs */
	while ( !list_empty(&pdev->rendering_runq_head) ) {
		list_for_each (pos, &pdev->rendering_runq_head)
			vgt_deactive(pdev, pos);
	};
	if (pdev->gttmmio_base_va)
		iounmap(pdev->gttmmio_base_va);
	if (pdev->phys_gmadr_va)
		iounmap(pdev->phys_gmadr_va);
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
