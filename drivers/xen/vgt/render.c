/*
 * Render context management
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include "vgt.h"

/*
 * NOTE list:
 *	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
 *	 also the hooks in AGP driver
 *	- need a check on "unsigned long" vs. "u64" usage
 *	- need consider cache related issues, e.g. Linux/Windows may have different
 *	 TLB invalidation mode setting, which may impact vGT's context switch logic
 */
static bool vgt_restore_context (struct vgt_device *vgt);
static bool vgt_save_context (struct vgt_device *vgt);
u64	context_switch_cost = 0;
u64	context_switch_num = 0;

int vgt_ctx_switch = 1;
bool vgt_validate_ctx_switch = false;

/*
 * TODO: the context layout could be different on generations.
 * e.g. ring head/tail, ccid, etc. when PPGTT is enabled
 */
#define OFF_CACHE_MODE_0	0x4A
#define OFF_CACHE_MODE_1	0x4B
#define OFF_INSTPM		0x4D
#define OFF_EXCC		0x4E
#define OFF_MI_MODE		0x4F
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

bool vgt_magic_done(struct pgt_device *pdev, int ring_id)
{
	u32 *ptr;

	ptr = (u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev));
	if (*ptr != pdev->magic)
		return false;

	return true;
}

static bool ring_wait_for_magic(struct pgt_device *pdev, int ring_id)
{
	if (wait_for_atomic(vgt_magic_done(pdev, ring_id), 100) != 0) {
		vgt_err("wait for magic fail on ring %d\n", ring_id);
		return false;
	}
	return true;
}

/*
 * Wait for the empty of RB.
 * TODO: Empty of RB doesn't mean the commands are retired. May need a STORE_IMM
 * after MI_FLUSH, but that needs our own hardware satus page.
 */
static bool ring_wait_for_empty(struct pgt_device *pdev, int ring_id, char *str)
{
	/* wait to be completed */
	if (wait_for_atomic(is_rendering_engine_empty(pdev, ring_id), 100) != 0) {
		vgt_err("(%s): timeout wait 100 ms for ring(%d)\n", str, ring_id);

		vgt_err("vGT-cur(%d): head(%x), tail(%x), start(%x)\n",
				current_render_owner(pdev)->vgt_id,
				current_render_owner(pdev)->rb[ring_id].sring.head,
				current_render_owner(pdev)->rb[ring_id].sring.tail,
				current_render_owner(pdev)->rb[ring_id].sring.start);
		vgt_err("vGT-dom0(%d): head(%x), tail(%x), start(%x)\n",
				vgt_dom0->vgt_id,
				vgt_dom0->rb[ring_id].sring.head,
				vgt_dom0->rb[ring_id].sring.tail,
				vgt_dom0->rb[ring_id].sring.start);
		show_debug(pdev, ring_id);
		show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
		return false;
	}

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

		if ( !ring_wait_for_empty(pdev, i, "wait-empty") ) {
			*ring_id = i;
			return false;
		}
	}
	return true;
}

/* update the tail pointer after all the context is restored */
void vgt_resume_ringbuffers(struct vgt_device *vgt)
{
	int i;

	for (i = 0; i < vgt->pdev->max_engines; i++) {
		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		if (!(vgt->rb[i].sring.ctl & _RING_CTL_ENABLE)) {
			vgt_dbg("vGT: ring (%d) not enabled. exit resume\n", i);
			continue;
		}
		VGT_MMIO_WRITE(vgt->pdev, RB_TAIL(vgt->pdev, i), vgt->rb[i].sring.tail);
	}
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

/*
 * The thread to perform the VGT ownership switch.
 *
 * We need to handle race conditions from different paths around
 * vreg/sreg/hwreg. So far there're 4 paths at least:
 *   a) the vgt thread to conduct context switch
 *   b) the GP handler to emulate MMIO for dom0
 *   c) the event handler to emulate MMIO for other VMs
 *   d) the interrupt handler to do interrupt virtualization
 *   e) /sysfs interaction from userland program
 *
 * Now d) is removed from the race path, because we adopt a delayed
 * injection mechanism. Physical interrupt handler only saves pending
 * IIR bits, and then wake up the vgt thread. Later the vgt thread
 * checks the pending bits to do the actual virq injection. This approach
 * allows vgt thread to handle ownership switch cleanly.
 *
 * So it's possible for other 3 paths to touch vreg/sreg/hwreg:
 *   a) the vgt thread may need to update HW updated regs into
 *	  vreg/sreg of the prev owner
 *   b) the GP handler and event handler always updates vreg/sreg,
 *	  and may touch hwreg if vgt is the current owner
 *	  and then update vreg for interrupt virtualization
 *
 * To simplify the lock design, we make below assumptions:
 *   a) the vgt thread doesn't trigger GP fault itself, i.e. always
 *	  issues hypercall to do hwreg access
 *   b) the event handler simply notifies another kernel thread, leaving
 *	  to that thread for actual MMIO emulation
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
	int threshold = 500; /* print every 500 times */
	int ring_id;
	cycles_t t0, t1, t2, t3;//t4 = 0, t5;

	//ASSERT(current_render_owner(pdev));
	printk("vGT: start kthread for dev (%x, %x)\n", pdev->bus, pdev->devfn);

	while (!kthread_should_stop()) {
		/*
		 * TODO: Use high priority task and timeout based event
		 *	mechanism for QoS. schedule in 50ms now.
		 */
		/* vgt_thread can only be waken up when there is a request */
		wait_event(pdev->event_wq, pdev->request);

		t0 = vgt_get_cycles();
		if (!pdev->request) {
			printk("vGT: main thread waken up by unknown reasons!\n");
			continue;
		}


		/* Handle virtual interrupt injection to current owner */
		if (test_and_clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request)) {
			spin_lock_irq(&pdev->lock);
			vgt_forward_events(pdev);
			spin_unlock_irq(&pdev->lock);
		}

		/* Send uevent to userspace */
		if (test_and_clear_bit(VGT_REQUEST_UEVENT, (void *)&pdev->request)) {
			vgt_signal_uevent(pdev);
		}

		if (!test_and_clear_bit(VGT_REQUEST_CTX_SWITCH,
					(void *)&pdev->request))
			continue;

		if (!vgt_ctx_switch)
			continue;

		if (!(vgt_ctx_check(pdev) % threshold))
			vgt_dbg("vGT: %lldth checks, %lld switches\n",
				vgt_ctx_check(pdev), vgt_ctx_switch(pdev));
		vgt_ctx_check(pdev)++;

		/* FIXME: this should be guaranteed by ctx scheduler.
		 * But for robustness, let's keep it temporarily */
		if (vgt_runq_is_empty(pdev)) {
			/* Idle now, and no pending activity */
			vgt_dbg("....idle\n");
			continue;
		}

		/*
		 * disable interrupt which is sufficient to prevent more
		 * cmds submitted by the current owner, when dom0 is UP.
		 * if the mmio handler for HVM is made into a thread,
		 * simply a spinlock is enough
		 */
		spin_lock_irq(&pdev->lock);

		ASSERT(next_sched_vgt);
		next = next_sched_vgt;

		if ( next != (prev = current_render_owner(pdev)) )
		{
			if (is_rendering_engines_empty(pdev, &ring_id)) {
				vgt_dbg("vGT: next vgt (%d)\n", next->vgt_id);

				/* variable exported by debugfs */
				context_switch_num ++;
				t1 = vgt_get_cycles();
				/* Records actual tsc when all rendering engines
				 * are stopped */
				if (event_based_qos) {
					ctx_actual_end_time(current_render_owner(pdev)) = t1;
				}
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
				//prev = current_render_owner(pdev);
				if ( prev )
					prev->stat.allocated_cycles +=
						(t1 - prev->stat.schedule_in_time);
				vgt_ctx_switch(pdev)++;

				//show_seqno(pdev);
				if (!vgt_save_context(prev)) {
					vgt_err("vGT: (%lldth checks %lldth switch<%d->%d>): fail to save context\n",
						vgt_ctx_check(pdev),
						vgt_ctx_switch(pdev),
						prev->vgt_id,
						next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					local_irq_enable();
					break;
				}

				if (!vgt_restore_context(next)) {
					vgt_err("vGT: (%lldth checks %lldth switch<%d->%d>): fail to restore context\n",
						vgt_ctx_check(pdev),
						vgt_ctx_switch(pdev),
						prev->vgt_id,
						next->vgt_id);

					/* TODO: any recovery to do here. Now simply exits the thread */
					local_irq_enable();
					break;
				}

				current_render_owner(pdev) = next;
				//show_seqno(pdev);

				if (pdev->enable_ppgtt && next->ppgtt_initialized)
					vgt_ppgtt_switch(next);

				vgt_resume_ringbuffers(next);

				/* request to check IRQ when ctx switch happens */
				if (prev->force_removal ||
					bitmap_empty(prev->enabled_rings, MAX_ENGINES)) {
					printk("Disable render for vgt(%d) from kthread\n",
						prev->vgt_id);
					vgt_disable_render(prev);
					wmb();
					if (prev->force_removal) {
						prev->force_removal = false;
						if (waitqueue_active(&pdev->destroy_wq))
							wake_up(&pdev->destroy_wq);
					}
					/* no need to check if prev is to be destroyed */
				}

				t2 = vgt_get_cycles();
				next->stat.schedule_in_time = t2;
				//printk("vGT: take %lld cycles\n", t2 - t1);

				/* setup countdown for next vgt context */
				if (event_based_qos) {
					vgt_setup_countdown(next);
				}

			} else {
				printk("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
					vgt_ctx_switch(pdev),
					current_render_owner(pdev)->vgt_id, ring_id);
				show_ringbuffer(pdev, ring_id, 16 * sizeof(vgt_reg_t));
			}
		}

		spin_unlock_irq(&pdev->lock);

		t3 = vgt_get_cycles();
		context_switch_cost += (t3-t0);
	}
	return 0;
}

/*
 * Only need to save head, since it's the only one updated by hw
 */
void ring_phys_2_shadow(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_dbg("old head(%x), old tail(%x)\n", srb->head, srb->tail);
	srb->head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	vgt_dbg("new head(%x), new tail(%x)\n", srb->head, srb->tail);
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
	ring_wait_for_empty(pdev, ring_id, "stop-ring");
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[ring_id],
			_REGBIT_MI_STOP_RINGS | (_REGBIT_MI_STOP_RINGS << 16));
}

static inline void resume_ring(struct pgt_device *pdev, int ring_id)
{
	/* make sure ring resumed */
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[ring_id],
			_REGBIT_MI_STOP_RINGS << 16);
	if (VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_STOP_RINGS)
		vgt_warn("!!!!!!!!!failed to clear stop ring bit\n");
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


bool vgt_vrings_empty(struct vgt_device *vgt)
{
	int ring_id;
	vgt_ringbuffer_t *vring;
	for (ring_id = 0; ring_id < vgt->pdev->max_engines; ring_id++)
		if (test_bit(ring_id, vgt->enabled_rings)) {
			vring = &vgt->rb[ring_id].vring;
			if (!RB_HEAD_TAIL_EQUAL(vring->head, vring->tail))
				return false;
		}

	return true;
}

/*
 * to restore to a new ring buffer, we need restore all ring regs including head.
 */
void ring_shadow_2_phys(struct pgt_device *pdev, int ring_id, vgt_ringbuffer_t *srb)
{
	vgt_dbg("shadow 2 phys: [%x, %x, %x, %x] \n", srb->head, srb->tail,
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
	vgt_dbg("shadow 2 phys: [%x, %x]\n",
		VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id)),
		VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id)));
}

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

	_REG_BRSYNC,
	_REG_BVSYNC,
	_REG_RBSYNC,
	_REG_RVSYNC,
	_REG_VBSYNC,
	_REG_VRSYNC,
};

vgt_reg_t vgt_gen7_render_regs[] = {
	/* Add IVB register, so they all got pass-through */

	_REG_ARB_MODE,

	_REG_BCS_HWS_PGA_GEN7,
	_REG_RCS_HWS_PGA,
	_REG_VCS_HWS_PGA,

	_REG_GT_MODE_IVB,
	_REG_CACHE_MODE_0_IVB,
	_REG_CACHE_MODE_1_IVB,

	_REG_BCS_MI_MODE,
	_REG_BCS_BLT_MODE_IVB,
	_REG_BCS_INSTPM,
	_REG_BCS_HWSTAM,
	_REG_BCS_EXCC,
	_REG_BCS_UHPTR,
	_REG_BRSYNC,
	_REG_BVSYNC,

	_REG_RCS_MI_MODE,
	_REG_RCS_GFX_MODE_IVB,
	_REG_RCS_INSTPM,
	_REG_RCS_HWSTAM,
	_REG_RCS_EXCC,
	_REG_RCS_UHPTR,
	_REG_RBSYNC,
	_REG_RVSYNC,

	_REG_VCS_MI_MODE,
	_REG_VCS_MFX_MODE_IVB,
	_REG_VCS_INSTPM,
	_REG_VCS_HWSTAM,
	_REG_VCS_EXCC,
	_REG_VCS_UHPTR,
	_REG_VBSYNC,
	_REG_VRSYNC,

	_REG_TILECTL,
};

static void __vgt_rendering_save(struct vgt_device *vgt, int num, vgt_reg_t *regs)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		int reg = regs[i];
		//if (reg_hw_status(vgt->pdev, reg)) {
		/* FIXME: only hw update reg needs save */
		if (!reg_mode_ctl(vgt->pdev, reg))
		{
			__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
			__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
			vgt_dbg("....save mmio (%x) with (%x)\n", reg, __sreg(vgt, reg));
		}
	}
}

/* For save/restore global states difference between VMs.
 * Other context states should be covered by normal context switch later. */
void vgt_rendering_save_mmio(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	/*
	 * both save/restore refer to the same array, so it's
	 * enough to track only save part
	 */
	pdev->in_ctx_switch = 1;
	if (IS_SNB(pdev))
		__vgt_rendering_save(vgt, ARRAY_NUM(vgt_render_regs), &vgt_render_regs[0]);
	else if (IS_IVB(pdev) || IS_HSW(pdev))
		__vgt_rendering_save(vgt, ARRAY_NUM(vgt_gen7_render_regs), &vgt_gen7_render_regs[0]);
	pdev->in_ctx_switch = 0;
}

static void __vgt_rendering_restore (struct vgt_device *vgt, int num_render_regs, vgt_reg_t *render_regs)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	vgt_reg_t  res_val; /*restored value of mmio register*/
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i = 0; i < num_render_regs; i++) {
		int reg = render_regs[i];
		vgt_reg_t val = __sreg(vgt, reg);

		if (reg_mode_ctl(pdev, reg) && reg_aux_mode_mask(pdev, reg))
			val |= reg_aux_mode_mask(pdev, reg);

		/*
		 * FIXME: there's regs only with some bits updated by HW. Need
		 * OR vm's update with hw's bits?
		 */
		//if (!reg_hw_status(vgt->pdev, reg))
		VGT_MMIO_WRITE(vgt->pdev, reg, val);
		vgt_dbg("....restore mmio (%x) with (%x)\n", reg, val);

		if(!vgt_validate_ctx_switch)
			continue;
		res_val = VGT_MMIO_READ(vgt->pdev, reg);
		if(res_val == val)
			continue;
		if (!reg_mode_ctl(pdev, reg) ||
			 ((res_val ^ val) & (reg_aux_mode_mask(pdev, reg) >> 16)))
			vgt_warn("restore %x: failed:  val=%x, val_read_back=%x\n",
				reg, val, res_val);
	}
}

/*
 * Rstore MMIO registers per rendering context.
 * (Not include ring buffer registers).
 */
void vgt_rendering_restore_mmio(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (IS_SNB(pdev))
		__vgt_rendering_restore(vgt, ARRAY_NUM(vgt_render_regs), &vgt_render_regs[0]);
	else if (IS_IVB(pdev) || IS_HSW(pdev))
		__vgt_rendering_restore(vgt, ARRAY_NUM(vgt_gen7_render_regs), &vgt_gen7_render_regs[0]);
}

/*
 * Rendering engine context switch
 *
 */

static bool vgt_save_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_state_ring_t *rb;
	vgt_reg_t old_tail;

	if (vgt == NULL)
		return false;

	//disable_power_management(vgt);

	vgt_rendering_save_mmio(vgt);

	/* save rendering engines */
	for (i=0; i < pdev->max_engines; i++) {
		struct vgt_ring_buffer *ring = pdev->ring_buffer;
		vgt_reg_t	ccid;

		rb = &vgt->rb[i];
		old_tail = rb->sring.tail;
		/* Save head */
		ring_phys_2_shadow (pdev, i, &rb->sring);

		sring_2_vring(vgt, i, &rb->sring, &rb->vring);

		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		/* for stateless engine, no need to save/restore context */
		if (rb->stateless)
			continue;

		if (i != RING_BUFFER_RCS)
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

		if (vgt->has_context) {
			rb->active_vm_context = VGT_MMIO_READ(pdev, _REG_CCID);
			rb->active_vm_context &= 0xfffff000;
			vgt_dbg("VM %d CCID 0x%x\n", vgt->vm_id, rb->active_vm_context);
		}

		disable_ring(pdev, i);

		vgt_ring_start(ring);

		/* XXX disable_ring()->stop_ring() will disable ring in
		 * MI_MODE */
		resume_ring(pdev, i);

		vgt_ring_begin(ring, 14);
		vgt_ring_emit(ring, MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN);
		vgt_ring_emit(ring, MI_SET_CONTEXT);
		vgt_ring_emit(ring, MI_RESTORE_INHIBIT | MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN |
				MI_RESTORE_EXT_STATE_EN |
				rb->context_save_area);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_SUSPEND_FLUSH);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_FLUSH);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_STORE_DATA_IMM | MI_SDI_USE_GTT);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
		vgt_ring_emit(ring, ++pdev->magic);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_advance(ring);

		if (!ring_wait_for_magic(pdev, i)) {
			vgt_err("save context commands unfinished\n");
			show_ringbuffer(pdev, i, 16 * sizeof(vgt_reg_t));
			return false;
		}

		vgt_dbg("new magic number: %d\n",
				*(u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));

		/* still confirm the CCID for safety. May remove in the future */
		ccid = VGT_MMIO_READ (pdev, _REG_CCID);
		if ((ccid & GTT_PAGE_MASK) != (rb->context_save_area & GTT_PAGE_MASK)) {
			printk("vGT: CCID isn't changed [%x, %lx]\n", ccid, (unsigned long)rb->context_save_area);
		}

		rb->initialized = true;

		vgt_dbg("<vgt-%d>vgt_save_context done\n", vgt->vgt_id);

	}
	return true;
}

static bool vgt_restore_context (struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_state_ring_t	*rb;
	vgt_reg_t old_tail;

	if (vgt == NULL)
		return false;

	for (i=0; i < pdev->max_engines; i++) {
		struct vgt_ring_buffer *ring = pdev->ring_buffer;

		rb = &vgt->rb[i];

		if (!enable_video_switch && i == RING_BUFFER_VCS)
			continue;

		if (rb->stateless)
			continue;

		if (i != RING_BUFFER_RCS)
			continue;

		old_tail = rb->sring.tail;

		/* some mode control registers can be only restored through this command */
		update_context(vgt, rb->context_save_area);

		vgt_ring_begin(ring, 12);
		vgt_ring_emit(ring, MI_SUSPEND_FLUSH | MI_SUSPEND_FLUSH_EN);
		vgt_ring_emit(ring, MI_SET_CONTEXT);
		if (rb->initialized)
			vgt_ring_emit(ring, rb->context_save_area |
					MI_MM_SPACE_GTT |
					MI_SAVE_EXT_STATE_EN |
					MI_RESTORE_EXT_STATE_EN |
					MI_FORCE_RESTORE);
		else {
			printk("vGT(%d): first initialization. switch to dummy context.\n",
					vgt->vgt_id);
			vgt_ring_emit(ring, pdev->dummy_area |
					MI_MM_SPACE_GTT |
					MI_SAVE_EXT_STATE_EN |
					MI_RESTORE_EXT_STATE_EN |
					MI_RESTORE_INHIBIT);
		}

		vgt_ring_emit(ring, MI_SUSPEND_FLUSH);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_FLUSH);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_STORE_DATA_IMM | MI_SDI_USE_GTT);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
		vgt_ring_emit(ring, ++pdev->magic);
		vgt_ring_emit(ring, 0);
		vgt_ring_advance(ring);

		if (!ring_wait_for_magic(pdev, i)) {
			vgt_err("restore context switch commands unfinished\n");
			show_ringbuffer(pdev, i, 16 * sizeof(vgt_reg_t));
			return false;
		}

		/* restore VM context */
		if (vgt->has_context && rb->active_vm_context) {
			vgt_ring_begin(ring, 8);
			vgt_ring_emit(ring, MI_SET_CONTEXT);
			vgt_ring_emit(ring, rb->active_vm_context |
					MI_MM_SPACE_GTT |
					MI_SAVE_EXT_STATE_EN |
					MI_RESTORE_EXT_STATE_EN |
					MI_FORCE_RESTORE);

			vgt_ring_emit(ring, MI_STORE_DATA_IMM | MI_SDI_USE_GTT);
			vgt_ring_emit(ring, 0);
			vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
			vgt_ring_emit(ring, ++pdev->magic);
			vgt_ring_emit(ring, 0);
			vgt_ring_emit(ring, 0);
			vgt_ring_advance(ring);

			if (!ring_wait_for_magic(pdev, i)) {
				vgt_err("change to VM context switch commands unfinished\n");
				show_ringbuffer(pdev, i, 16 * sizeof(vgt_reg_t));
				return false;
			}
		}
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
	vgt_dbg("<vgt-%d>vgt_restore_context done\n", vgt->vgt_id);
	return true;
}

static inline int tail_to_ring_id(struct pgt_device *pdev, unsigned int tail_off)
{
	int i;

	for (i = 0; i < pdev->max_engines; i++) {
		if ( pdev->ring_mmio_base[i] == tail_off )
			return i;
	}
	printk("Wrong tail register %s\n", __FUNCTION__);
	ASSERT(0);
	return 0;
}

u64 ring_mmio_rcnt=0;
u64 ring_mmio_wcnt=0;
u64 ring_tail_mmio_wcnt=0;
u64 ring_tail_mmio_wcycles=0;

bool ring_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;

	ring_mmio_rcnt++;

	ASSERT(bytes <= 4 && !(off & (bytes - 1)));
	//printk("vGT:ring_mmio_read (%x)\n", off);

	if ((hvm_render_owner && (vgt->vm_id != 0)) || reg_hw_access(vgt, off)){
		unsigned long data;
		data = VGT_MMIO_READ_BYTES(vgt->pdev, off, bytes);
		memcpy(p_data, &data, bytes);
		return true;
	}

	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_to_ring_id (vgt->pdev, _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	memcpy(p_data, (char *)vring + rel_off, bytes);
	//ring_debug(vgt, ring_id);
	return true;
}

bool ring_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;
	vgt_ringbuffer_t	*sring;
	struct vgt_tailq *tailq = NULL;
	vgt_reg_t	oval;
	cycles_t	t0, t1;
	struct pgt_device *pdev = vgt->pdev;

	ring_mmio_wcnt++;
	ASSERT(bytes <= 4);
	vgt_dbg("vGT:ring_mmio_write (0x%x) with val (0x%x)\n", off, *((u32 *)p_data));
	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ASSERT(!(rel_off & (bytes - 1)));

	ring_id = tail_to_ring_id (pdev, _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;
	sring = &vgt->rb[ring_id].sring;

	if (shadow_tail_based_qos)
		tailq = &vgt->rb_tailq[ring_id];

	if (ring_id == RING_BUFFER_VECS)
		vgt->vebox_support = true;

	oval = *(vgt_reg_t *)((char *)vring + rel_off);
	memcpy((char *)vring + rel_off, p_data, bytes);

	switch (rel_off) {
	case RB_OFFSET_TAIL:
		t0 = get_cycles();
		ring_tail_mmio_wcnt++;

		/* enable hvm tailq after the ring enabled */
		if (shadow_tail_based_qos) {
			if (test_bit(ring_id, vgt->enabled_rings))
				vgt_tailq_pushback(tailq, vring->tail, 0);
		} else
			sring->tail = vring->tail;

#if 0
		if (shadow_tail_based_qos) {
			if (vgt->vgt_id > 0) {
				if (enable_hvm_tailq && !vgt->force_removal)
					vgt_tailq_pushback(tailq, vring->tail, 0);
			} else
				vgt_tailq_pushback(tailq, vring->tail, 0);
		} else
			sring->tail = vring->tail;
#endif


		if ( !bypass_scan )
			vgt_scan_vring(vgt, ring_id);
		t1 = get_cycles();
		ring_tail_mmio_wcycles += (t1-t0);

		if (shadow_tail_based_qos) {
			if (vgt_tailq_last_stail(tailq)
					&& !test_and_set_bit(ring_id, (void *)vgt->started_rings))
				printk("Ring-%d starts work for vgt-%d\n",
						ring_id, vgt->vgt_id);
			/* When a ring is enabled, tail value
			 * can never write to real hardware */
			return true;
		} else {
			if (sring->tail &&
					!test_and_set_bit(ring_id, (void *)vgt->started_rings))
				printk("Ring-%d starts work for vgt-%d\n",
						ring_id, vgt->vgt_id);
		}

		break;
	case RB_OFFSET_HEAD:
		//debug
		//vring->head |= 0x200000;
		sring->head = vring->head;
		break;
	case RB_OFFSET_START:
		sring->start = mmio_g2h_gmadr(vgt, off, vring->start);
		break;
	case RB_OFFSET_CTL:
		sring->ctl = vring->ctl;

		if ( (oval & _RING_CTL_ENABLE) &&
			!(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: deactivate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_disable_ring(vgt, ring_id);
		}
		else if ( !(oval & _RING_CTL_ENABLE) &&
			(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: activate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_enable_ring(vgt, ring_id);
		}
		if (!bypass_scan && (vring->ctl & _RING_CTL_ENABLE)) {
			vgt->last_scan_head[ring_id] =
				vring->head & RB_HEAD_OFF_MASK;
			vgt_scan_vring(vgt, ring_id);
		}
		break;
	default:
		ASSERT(0);
		break;
	}

	/* TODO: lock with kthread? */
	/*
	 * FIXME: Linux VM doesn't read head register directly. Instead it relies on
	 * automatic head reporting mechanism. Later with command parser, there's no
	 * problem since all commands are translated and filled by command parser. for
	 * now it's possible for dom0 to fill over than a full ring in a scheduled
	 * quantum
	 */
	if (reg_hw_access(vgt, off))
		VGT_MMIO_WRITE(pdev, off, *(vgt_reg_t*)((char *)sring + rel_off));
	//ring_debug(vgt, ring_id);
	return true;
}

u64	ring_0_idle = 0;
u64	ring_0_busy = 0;
struct pgt_device *perf_pgt = NULL;

void vgt_gpu_perf_sample(void)
{
	int	ring_id = 0;

	if ( perf_pgt ) {
		if ( is_rendering_engine_empty(perf_pgt, ring_id) )
			ring_0_idle ++;
		else
			ring_0_busy ++;
	}
}
EXPORT_SYMBOL_GPL(vgt_gpu_perf_sample);
