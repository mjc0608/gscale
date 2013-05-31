/*
 * Render context management
 *
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
u64	context_switch_cost = 0;
u64	context_switch_num = 0;

int vgt_ctx_switch = 1;
bool vgt_validate_ctx_switch = false;

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

static bool ring_is_empty(struct pgt_device *pdev,
	int id)
{
	if ( is_ring_enabled(pdev, id) && !is_ring_empty(pdev, id) )
		return false;

	return true;
}

static bool ring_is_idle(struct pgt_device *pdev,
	int id)
{
	/*
	* FIXME: it turns out that psmi idle status bit check may not be
	 * always true when both dom0/Linux VM runs glxgears in parallel. Not
	 * sure the reason yet. So disable this check for now, but need revise
	 * in the future
	 */
#if 0
	if (!(VGT_MMIO_READ(pdev, pdev->ring_psmi[id]) & _REGBIT_PSMI_IDLE_INDICATOR))
		return false;
#endif
	return true;
}

static bool ring_is_stopped(struct pgt_device *pdev, int id)
{
	vgt_reg_t val;

	val = VGT_MMIO_READ(pdev, pdev->ring_mi_mode[id]);
	if ((val & (_REGBIT_MI_STOP_RINGS | _REGBIT_MI_RINGS_IDLE)) ==
	    (_REGBIT_MI_STOP_RINGS | _REGBIT_MI_RINGS_IDLE))
		return true;

	return false;
}

static bool ring_wait_for_completion(struct pgt_device *pdev, int id)
{
	u32 *ptr;

	/* now a single magic number, because only RCS supports hw switch */
	if (id != RING_BUFFER_RCS)
		return true;

	ptr = (u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev));
	if (wait_for_atomic((*ptr == pdev->magic), VGT_RING_TIMEOUT) != 0) {
		vgt_err("Timeout %d ms for CMD comletion on ring %d\n",
			VGT_RING_TIMEOUT, id);
		return false;
	}

	return true;
}

/* make a render engine idle */
static bool idle_render_engine(struct pgt_device *pdev, int id)
{
	if (wait_for_atomic(ring_is_empty(pdev, id), VGT_RING_TIMEOUT) != 0) {
		vgt_err("Timeout wait %d ms for ring(%d) empty\n",
			VGT_RING_TIMEOUT, id);
		return false;
	}

	/* may do some jobs here to make sure ring idle */

	if (wait_for_atomic(ring_is_idle(pdev, id), VGT_RING_TIMEOUT) != 0) {
		vgt_err("Timeout wait %d ms for ring(%d) idle\n",
			VGT_RING_TIMEOUT, id);
		return false;
	}

	return true;
}

static bool idle_rendering_engines(struct pgt_device *pdev, int *id)
{
	int i;

	/*
	 * Though engines are checked in order, no further CMDs
	 * are allowed from VM due to holding the big lock.
	 */
	for (i=0; i < pdev->max_engines; i++) {
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];

		if (!ring->need_switch)
			continue;

		if ( !idle_render_engine(pdev, i) ) {
			*id = i;
			return false;
		}
	}
	return true;
}

static inline void stop_ring(struct pgt_device *pdev, int id)
{
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[id],
			_REGBIT_MI_STOP_RINGS | (_REGBIT_MI_STOP_RINGS << 16));

	ASSERT(!wait_for_atomic(ring_is_stopped(pdev, id), VGT_RING_TIMEOUT));
}

static inline void start_ring(struct pgt_device *pdev, int id)
{
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[id],
			_REGBIT_MI_STOP_RINGS << 16);
	VGT_POST_READ(pdev, pdev->ring_mi_mode[id]);
}

/*
 * write to head is undefined when ring is enabled.
 *
 * so always invoke this disable action when recovering a new ring setting
 */
static inline void disable_ring(struct pgt_device *pdev, int id)
{
	/* disable the ring */
	VGT_WRITE_CTL(pdev, id, 0);
	/* by ktian1. no source for this trick */
	VGT_POST_READ_CTL(pdev, id);
}

static inline void enable_ring(struct pgt_device *pdev, int id, vgt_reg_t val)
{
	ASSERT(val & _RING_CTL_ENABLE);
	VGT_WRITE_CTL(pdev, id, val);
	VGT_POST_READ_CTL(pdev, id);
}

bool vgt_vrings_empty(struct vgt_device *vgt)
{
	int id;
	vgt_ringbuffer_t *vring;
	for (id = 0; id < vgt->pdev->max_engines; id++)
		if (test_bit(id, vgt->enabled_rings)) {
			vring = &vgt->rb[id].vring;
			if (!RB_HEAD_TAIL_EQUAL(vring->head, vring->tail))
				return false;
		}

	return true;
}

static void vgt_save_ringbuffer(struct vgt_device *vgt, int id)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[id];

	/* only head is HW updated */
	rb->sring.head = VGT_READ_HEAD(pdev, id);
	rb->vring.head = rb->sring.head;
}

/* restore ring buffer structures to a empty state (head==tail) */
static void vgt_restore_ringbuffer(struct vgt_device *vgt, int id)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_ringbuffer_t *srb = &vgt->rb[id].sring;
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	if (!ring->need_switch)
		return;

	vgt_dbg("restore ring: [%x, %x, %x, %x] \n", srb->head, srb->tail,
		VGT_READ_HEAD(pdev, id),
		VGT_READ_TAIL(pdev, id));

	if (!(srb->ctl & _RING_CTL_ENABLE)) {
		printk("vGT/switch-%lld: ring (%d) not enabled. exit restore\n",
			vgt_ctx_switch(pdev), id);
		VGT_WRITE_CTL(pdev, id, 0);
		return;
	}

	disable_ring(pdev, id);

	VGT_WRITE_START(pdev, id, srb->start);

	/* make head==tail when enabling the ring buffer */
	VGT_WRITE_HEAD(pdev, id, srb->head);
	VGT_WRITE_TAIL(pdev, id, srb->head);

	enable_ring(pdev, id, srb->ctl);

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
	VGT_POST_READ_HEAD(pdev, id);
	vgt_dbg("restore ring: [%x, %x]\n",
		VGT_READ_HEAD(pdev, id),
		VGT_READ_TAIL(pdev, id));
}

void vgt_kick_ringbuffers(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;

	for (i = 0; i < pdev->max_engines; i++) {
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];
		vgt_ringbuffer_t *srb = &vgt->rb[i].sring;

		if (!ring->need_switch)
			continue;

		start_ring(pdev, i);
		VGT_WRITE_TAIL(pdev, i, srb->tail);
	}
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
static void vgt_rendering_save_mmio(struct vgt_device *vgt)
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
 * Restore MMIO registers per rendering context.
 * (Not include ring buffer registers).
 */
static void vgt_rendering_restore_mmio(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (IS_SNB(pdev))
		__vgt_rendering_restore(vgt, ARRAY_NUM(vgt_render_regs), &vgt_render_regs[0]);
	else if (IS_IVB(pdev) || IS_HSW(pdev))
		__vgt_rendering_restore(vgt, ARRAY_NUM(vgt_gen7_render_regs), &vgt_gen7_render_regs[0]);
}

/*
 * ring buffer usage in vGT driver is simple. We allocate a ring buffer
 * big enough to contain all emitted CMDs in a render context switch,
 * and thus emit function is implemented simply by sequentially advancing
 * tail point, w/o the wrap handling requirement.
 */
static inline void vgt_ring_emit(struct vgt_rsvd_ring *ring,
				u32 data)
{
	ASSERT(ring->tail <= ring->size);
	writel(data, ring->virtual_start + ring->tail);
	ring->tail += 4;
}

void vgt_ring_init(struct pgt_device *pdev, int id)
{
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	ring->pdev = pdev;
	ring->id = id;
	ring->size = VGT_RSVD_RING_SIZE;
	ring->start = aperture_2_gm(pdev,
			rsvd_aperture_alloc(pdev, ring->size));
	ring->virtual_start = v_aperture(pdev, ring->start);
	ring->head = 0;
	ring->tail = 0;

	switch (id) {
	case RING_BUFFER_RCS:
		ring->stateless = 0;
		ring->need_switch = 1;
		break;
	case RING_BUFFER_VCS:
	case RING_BUFFER_VECS:
		ring->stateless = 1;
		if (enable_video_switch)
			ring->need_switch = 1;
		else
			ring->need_switch = 0;
		break;
	case RING_BUFFER_BCS:
		ring->stateless = 1;
		ring->need_switch = 1;
		break;
	default:
		vgt_err("Unknown ring ID (%d)\n", id);
		ASSERT(0);
		break;
	}
}

static void vgt_setup_rsvd_ring(struct vgt_rsvd_ring *ring)
{
	struct pgt_device *pdev = ring->pdev;
	u32 head;
	int id = ring->id;

	ring->head = 0;
	ring->tail = 0;

	disable_ring(pdev, id);

	/* execute our ring */
	VGT_WRITE_HEAD(pdev, id, 0);
	VGT_WRITE_TAIL(pdev, id, 0);

	head = VGT_READ_HEAD(pdev, id);
	if (head != 0) {
		VGT_WRITE_HEAD(pdev, id, 0);
	}

	VGT_WRITE_START(pdev, id, ring->start);

	enable_ring(pdev, id, ((ring->size - PAGE_SIZE) & 0x1FF000) | 1);

	ASSERT(!wait_for(((VGT_READ_CTL(pdev, id) & 1) != 0 &&
			VGT_READ_START(pdev, id) == ring->start &&
			(VGT_READ_HEAD(pdev, id) & RB_HEAD_OFF_MASK) == 0),
			VGT_RING_TIMEOUT));
	vgt_dbg("start vgt ring at 0x%x\n", ring->start);
}

static void vgt_ring_advance(struct vgt_rsvd_ring *ring)
{
	ring->tail &= ring->size - 1;
	VGT_WRITE_TAIL(ring->pdev, ring->id, ring->tail);
}

static bool vgt_save_hw_context(int id, struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[id];
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];
	vgt_reg_t	ccid;

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

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("save context commands unfinished\n");
		return false;
	}

	vgt_dbg("new magic number: %d\n",
			*(u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev)));

	/* still confirm the CCID for safety. May remove in the future */
	ccid = VGT_MMIO_READ (pdev, _REG_CCID);
	if ((ccid & GTT_PAGE_MASK) != (rb->context_save_area & GTT_PAGE_MASK)) {
		printk("vGT: CCID isn't changed [%x, %lx]\n", ccid, (unsigned long)rb->context_save_area);
		return false;
	}

	rb->initialized = true;

	return true;
}

static bool vgt_restore_hw_context(int id, struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t	*rb = &vgt->rb[id];
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	/* sync between vReg and saved context */
	update_context(vgt, rb->context_save_area);

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

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("restore context switch commands unfinished\n");
		return false;
	}

	/* restore VM context */
	if (vgt->has_context && rb->active_vm_context) {
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

		if (!ring_wait_for_completion(pdev, id)) {
			vgt_err("change to VM context switch commands unfinished\n");
			return false;
		}
	}

	return true;
}

bool vgt_do_render_context_switch(struct pgt_device *pdev)
{
	struct vgt_device *next, *prev;
	int threshold = 500; /* print every 500 times */
	int i;
	cycles_t t0, t1;

	if (!(vgt_ctx_check(pdev) % threshold))
		vgt_dbg("vGT: %lldth checks, %lld switches\n",
			vgt_ctx_check(pdev), vgt_ctx_switch(pdev));
	vgt_ctx_check(pdev)++;

	ASSERT(!vgt_runq_is_empty(pdev));

	/*
	 * disable interrupt which is sufficient to prevent more
	 * cmds submitted by the current owner, when dom0 is UP.
	 * if the mmio handler for HVM is made into a thread,
	 * simply a spinlock is enough. IRQ handler is another
	 * race point
	 */
	spin_lock_irq(&pdev->lock);

	ASSERT(next_sched_vgt);
	next = next_sched_vgt;
	prev = current_render_owner(pdev);

	if (next == prev)
		goto out;

	if (!idle_rendering_engines(pdev, &i)) {
		vgt_err("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
			vgt_ctx_switch(pdev),
			current_render_owner(pdev)->vgt_id, i);
		goto err;
	}

	vgt_dbg("vGT: next vgt (%d)\n", next->vgt_id);

	/* variable exported by debugfs */
	context_switch_num ++;
	t0 = vgt_get_cycles();
	/* Records actual tsc when all rendering engines
	 * are stopped */
	if (event_based_qos) {
		ctx_actual_end_time(current_render_owner(pdev)) = t0;
	}

	if ( prev )
		prev->stat.allocated_cycles +=
			(t0 - prev->stat.schedule_in_time);
	vgt_ctx_switch(pdev)++;

	/* STEP-1: manually save render context */
	vgt_rendering_save_mmio(prev);

	/* STEP-2: HW render context switch */
	for (i=0; i < pdev->max_engines; i++) {
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];

		if (!ring->need_switch)
			continue;

		/* STEP-2a: stop the ring */
		stop_ring(pdev, i);

		/* STEP-2b: save current ring buffer structure */
		vgt_save_ringbuffer(prev, i);

		if (ring->stateless)
			continue;

		/* STEP-2c: switch to vGT ring buffer */
		vgt_setup_rsvd_ring(ring);
		start_ring(pdev, i);

		/* STEP-2d: save HW render context for prev */
		if (!vgt_save_hw_context(i, prev)) {
			vgt_err("Ring-%d: (%lldth checks %lldth switch<%d->%d>): fail to save context\n",
				i, vgt_ctx_check(pdev), vgt_ctx_switch(pdev),
				prev->vgt_id, next->vgt_id);
			goto err;
		}

		/* STEP-2e: restore HW render context for next */
		if (!vgt_restore_hw_context(i, next)) {
			vgt_err("Ring-%d: (%lldth checks %lldth switch<%d->%d>): fail to restore context\n",
				i, vgt_ctx_check(pdev), vgt_ctx_switch(pdev),
				prev->vgt_id, next->vgt_id);
			goto err;
		}

		/* STEP-2f: idle and stop ring at the end of HW switch */
		if (!idle_render_engine(pdev, i)) {
			vgt_err("fail to idle ring-%d after ctx restore\n", i);
			goto err;
		}
		stop_ring(pdev, i);
	}

	/* STEP-3: manually restore render context */
	vgt_rendering_restore_mmio(next);

	/* STEP-4: restore ring buffer structure */
	for (i = 0; i < pdev->max_engines; i++)
		vgt_restore_ringbuffer(next, i);

	/* STEP-5: switch PPGTT */
	current_render_owner(pdev) = next;
	/* ppgtt switch must be done after render owner switch */
	if (pdev->enable_ppgtt && next->ppgtt_initialized)
		vgt_ppgtt_switch(next);

	/* STEP-6: ctx switch ends, and then kicks of new tail */
	vgt_kick_ringbuffers(next);

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

	next->stat.schedule_in_time = vgt_get_cycles();

	/* setup countdown for next vgt context */
	if (event_based_qos) {
		vgt_setup_countdown(next);
	}

	t1 = vgt_get_cycles();
	context_switch_cost += (t1-t0);
out:
	spin_unlock_irq(&pdev->lock);
	return true;
err:
	/* TODO: any cleanup for context switch errors? */
	vgt_err("FAIL on ring-%d\n", i);
	vgt_err("cur(%d): head(%x), tail(%x), start(%x)\n",
			current_render_owner(pdev)->vgt_id,
			current_render_owner(pdev)->rb[i].sring.head,
			current_render_owner(pdev)->rb[i].sring.tail,
			current_render_owner(pdev)->rb[i].sring.start);
	vgt_err("dom0(%d): head(%x), tail(%x), start(%x)\n",
			vgt_dom0->vgt_id,
			vgt_dom0->rb[i].sring.head,
			vgt_dom0->rb[i].sring.tail,
			vgt_dom0->rb[i].sring.start);
	show_debug(pdev, i);
	show_ringbuffer(pdev, i, 16 * sizeof(vgt_reg_t));
	spin_unlock_irq(&pdev->lock);
	/* crash system now, to avoid causing more confusing errors */
	ASSERT(0);
	return false;
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
		if ( ring_is_empty(perf_pgt, ring_id) )
			ring_0_idle ++;
		else
			ring_0_busy ++;
	}
}
EXPORT_SYMBOL_GPL(vgt_gpu_perf_sample);
