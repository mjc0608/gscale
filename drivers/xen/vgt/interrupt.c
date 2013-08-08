/*
 * vGT interrupt handler
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

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <xen/events.h>
#include <xen/interface/vcpu.h>
#include <xen/interface/hvm/hvm_op.h>
#include "vgt.h"

/*
 * TODO:
 *   - IIR could store two pending interrupts. need emulate the behavior
 *   - GT has 2nd level IMR registers (render/blitter/video)
 *   - Handle more events (like hdmi/dp hotplug, pipe-c, watchdog, etc.)
 */

/*
 * Below are necessary steps to add a new event handling:
 *   a) (device specific) add bit<->event mapping information in
 *      vgt_base_init_irq
 *
 *   b) (event specific) add event forwarding policy in vgt_init_events
 *
 *      Normally those are the only steps required, if the event is only
 *      associated to the 1st leve interrupt control registers (iir/ier
 *      imr/isr). The default handler will take care automatically
 *
 *      In the case where the event is associated with status/control
 *      bits in other registers (e.g. monitor hotplug), you'll provide
 *      specific handler for both physical event and virtual event
 *
 *   c) create a vgt_handle_XXX_phys handler, which deals with any required
 *      housekeeping, and may optionally cache some state to be forwarded
 *      to a VM
 *
 *   d) create a vgt_handle_XXX_virt handler, which emulates a virtual
 *      event generation with any required state emulated accordingly, may
 *      optionally use cached state from p_handler
 *
 *   e) setup virt/phys handler in vgt_init_events
 */
static void vgt_handle_events(struct vgt_irq_host_state *hstate, void *iir,
	enum vgt_irq_type type);

static int vgt_irq_warn_once[VGT_MAX_VMS+1][EVENT_MAX];

char *vgt_irq_name[EVENT_MAX] = {
	// GT
	[RCS_MI_USER_INTERRUPT] = "Render Command Streamer MI USER INTERRUPT",
	[RCS_DEBUG] = "Render EU debug from SVG",
	[RCS_MMIO_SYNC_FLUSH] = "Render MMIO sync flush status",
	[RCS_CMD_STREAMER_ERR] = "Render Command Streamer error interrupt",
	[RCS_PIPE_CONTROL] = "Render PIPE CONTROL notify",
	[RCS_WATCHDOG_EXCEEDED] = "Render Command Streamer Watchdog counter exceeded",
	[RCS_PAGE_DIRECTORY_FAULT] = "Render page directory faults",
	[RCS_AS_CONTEXT_SWITCH] = "Render AS Context Switch Interrupt",

	[VCS_MI_USER_INTERRUPT] = "Video Command Streamer MI USER INTERRUPT",
	[VCS_MMIO_SYNC_FLUSH] = "Video MMIO sync flush status",
	[VCS_CMD_STREAMER_ERR] = "Video Command Streamer error interrupt",
	[VCS_MI_FLUSH_DW] = "Video MI FLUSH DW notify",
	[VCS_WATCHDOG_EXCEEDED] = "Video Command Streamer Watchdog counter exceeded",
	[VCS_PAGE_DIRECTORY_FAULT] = "Video page directory faults",
	[VCS_AS_CONTEXT_SWITCH] = "Video AS Context Switch Interrupt",

	[BCS_MI_USER_INTERRUPT] = "Blitter Command Streamer MI USER INTERRUPT",
	[BCS_MMIO_SYNC_FLUSH] = "Billter MMIO sync flush status",
	[BCS_CMD_STREAMER_ERR] = "Blitter Command Streamer error interrupt",
	[BCS_MI_FLUSH_DW] = "Blitter MI FLUSH DW notify",
	[BCS_PAGE_DIRECTORY_FAULT] = "Blitter page directory faults",
	[BCS_AS_CONTEXT_SWITCH] = "Blitter AS Context Switch Interrupt",

	[VECS_MI_FLUSH_DW] = "Video Enhanced Streamer MI FLUSH DW notify",

	// DISPLAY
	[PIPE_A_FIFO_UNDERRUN] = "Pipe A FIFO underrun",
	[PIPE_A_CRC_ERR] = "Pipe A CRC error",
	[PIPE_A_CRC_DONE] = "Pipe A CRC done",
	[PIPE_A_VSYNC] = "Pipe A vsync",
	[PIPE_A_LINE_COMPARE] = "Pipe A line compare",
	[PIPE_A_ODD_FIELD] = "Pipe A odd field",
	[PIPE_A_EVEN_FIELD] = "Pipe A even field",
	[PIPE_A_VBLANK] = "Pipe A vblank",
	[PIPE_B_FIFO_UNDERRUN] = "Pipe B FIFO underrun",
	[PIPE_B_CRC_ERR] = "Pipe B CRC error",
	[PIPE_B_CRC_DONE] = "Pipe B CRC done",
	[PIPE_B_VSYNC] = "Pipe B vsync",
	[PIPE_B_LINE_COMPARE] = "Pipe B line compare",
	[PIPE_B_ODD_FIELD] = "Pipe B odd field",
	[PIPE_B_EVEN_FIELD] = "Pipe B even field",
	[PIPE_B_VBLANK] = "Pipe B vblank",
	[DPST_PHASE_IN] = "DPST phase in event",
	[DPST_HISTOGRAM] = "DPST histogram event",
	[GSE] = "GSE",
	[DP_A_HOTPLUG] = "DP A Hotplug",
	[AUX_CHANNEL_A] = "AUX Channel A",
	[PCH_IRQ] = "PCH Display interrupt event",
	[PERF_COUNTER] = "Performance counter",
	[POISON] = "Poison",
	[GTT_FAULT] = "GTT fault",
	[PRIMARY_A_FLIP_DONE] = "Primary Plane A flip done",
	[PRIMARY_B_FLIP_DONE] = "Primary Plane B flip done",
	[SPRITE_A_FLIP_DONE] = "Sprite Plane A flip done",
	[SPRITE_B_FLIP_DONE] = "Sprite Plane B flip done",

	// PM
	[GV_DOWN_INTERVAL] = "Render geyserville Down evaluation interval interrupt",
	[GV_UP_INTERVAL] = "Render geyserville UP evaluation interval interrupt",
	[RP_DOWN_THRESHOLD] = "RP DOWN threshold interrupt",
	[RP_UP_THRESHOLD] = "RP UP threshold interrupt",
	[FREQ_DOWNWARD_TIMEOUT_RC6] = "Render Frequency Downward Timeout During RC6 interrupt",
	[PCU_THERMAL] = "PCU Thermal Event",
	[PCU_PCODE2DRIVER_MAILBOX] = "PCU pcode2driver mailbox event",

	// PCH
	[FDI_RX_INTERRUPTS_TRANSCODER_A] = "FDI RX Interrupts Combined A",
	[AUDIO_CP_CHANGE_TRANSCODER_A] = "Audio CP Change Transcoder A",
	[AUDIO_CP_REQUEST_TRANSCODER_A] = "Audio CP Request Transcoder A",
	[FDI_RX_INTERRUPTS_TRANSCODER_B] = "FDI RX Interrupts Combined B",
	[AUDIO_CP_CHANGE_TRANSCODER_B] = "Audio CP Change Transcoder B",
	[AUDIO_CP_REQUEST_TRANSCODER_B] = "Audio CP Request Transcoder B",
	[FDI_RX_INTERRUPTS_TRANSCODER_C] = "FDI RX Interrupts Combined C",
	[AUDIO_CP_CHANGE_TRANSCODER_C] = "Audio CP Change Transcoder C",
	[AUDIO_CP_REQUEST_TRANSCODER_C] = "Audio CP Request Transcoder C",
	[ERR_AND_DBG] = "South Error and Debug Interupts Combined",
	[GMBUS] = "Gmbus",
	[SDVO_B_HOTPLUG] = "SDVO B hotplug",
	[CRT_HOTPLUG] = "CRT Hotplug",
	[DP_B_HOTPLUG] = "DisplayPort/HDMI/DVI B Hotplug",
	[DP_C_HOTPLUG] = "DisplayPort/HDMI/DVI C Hotplug",
	[DP_D_HOTPLUG] = "DisplayPort/HDMI/DVI D Hotplug",
	[AUX_CHENNEL_B] = "AUX Channel B",
	[AUX_CHENNEL_C] = "AUX Channel C",
	[AUX_CHENNEL_D] = "AUX Channel D",
	[AUDIO_POWER_STATE_CHANGE_B] = "Audio Power State change Port B",
	[AUDIO_POWER_STATE_CHANGE_C] = "Audio Power State change Port C",
	[AUDIO_POWER_STATE_CHANGE_D] = "Audio Power State change Port D",

	[EVENT_RESERVED] = "RESERVED EVENTS!!!",
};

/* =======================IRR/IMR/IER handlers===================== */

/* Now we have physical mask bits generated by ANDing virtual
 * mask bits from all VMs. That means, the event is physically unmasked
 * as long as a VM wants it. This is safe because we still use a single
 * big lock for all critical paths, but not efficient.
 */
u32 vgt_recalculate_mask_bits(struct pgt_device *pdev, unsigned int reg)
{
	int i;
	u32 imr = 0xffffffff;

	ASSERT(spin_is_locked(&pdev->lock));
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i])
			imr &= __vreg(pdev->device[i], reg);
	}

	return imr;
}

/*
 * Now we have physical enabling bits generated by ORing virtual
 * enabling bits from all VMs. That means, the event is physically enabled
 * as long as a VM wants it. This is safe because we still use a single
 * big lock for all critical paths, but not efficient.
 */
static u32 vgt_recalculate_ier(struct pgt_device *pdev, unsigned int reg)
{
	int i;
	u32 ier = 0;

	ASSERT(spin_is_locked(&pdev->lock));
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i])
			ier |= __vreg(pdev->device[i], reg);
	}

	return ier;
}

/* general write handler for all level-1 imr registers */
bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, masked, unmasked;
	uint32_t imr = *(u32 *)p_data, new_imr;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	vgt_dbg("IRQ: capture IMR write on reg (%x) with val (%x)\n",
		reg, imr);

	vgt_dbg("IRQ: old vIMR(%x), pIMR(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));

	/* figure out newly masked/unmasked bits */
	changed = __vreg(vgt, reg) ^ imr;
	changed &= ~_REGBIT_MASTER_INTERRUPT;
	masked = (__vreg(vgt, reg) & changed) ^ changed;
	unmasked = masked ^ changed;

	vgt_dbg("IRQ: changed (%x), masked(%x), unmasked (%x)\n",
		changed, masked, unmasked);

	__vreg(vgt, reg) = imr;

	if (changed) {
		new_imr = vgt_recalculate_mask_bits(pdev, reg);
		/*
		 * may optimize by caching the old imr, and then only update
		 * pReg when AND-ed value changes. but that requires link to
		 * device specific irq info. So avoid the complexity here
		 */
		VGT_MMIO_WRITE(pdev, reg, new_imr);
		VGT_POST_READ(pdev, reg);
	}

	ops->check_pending_irq(vgt);
	vgt_dbg("IRQ: new vIMR(%x), pIMR(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));
	return true;
}

/* general write handler for all level-1 ier registers */
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, enabled, disabled;
	uint32_t ier = *(u32 *)p_data, new_ier;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	vgt_dbg("IRQ: capture IER write on reg (%x) with val (%x)\n",
		reg, ier);

	vgt_dbg("IRQ: old vIER(%x), pIER(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));

	/* figure out newly enabled/disable bits */
	changed = __vreg(vgt, reg) ^ ier;
	enabled = (__vreg(vgt, reg) & changed) ^ changed;
	disabled = enabled ^ changed;

	vgt_dbg("vGT_IRQ: changed (%x), enabled(%x), disabled(%x)\n",
		changed, enabled, disabled);
	__vreg(vgt, reg) = ier;

	if (changed) {
		new_ier = vgt_recalculate_ier(pdev, reg);
		/*
		 * may optimize by caching the old ier, and then only update
		 * pReg when OR-ed value changes. but that requires link to
		 * device specific irq info. So avoid the complexity here
		 */
		VGT_MMIO_WRITE(pdev, reg, new_ier);
		VGT_POST_READ(pdev, reg);
	}

	ops->check_pending_irq(vgt);
	vgt_dbg("IRQ: new vIER(%x), pIER(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));
	return true;
}

bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t iir = *(vgt_reg_t *)p_data;

	vgt_dbg("IRQ: capture IIR write on reg (%x) with val (%x)\n",
		reg, iir);

	/* TODO: need use an atomic operation. Now it's safe due to big lock */
	__vreg(vgt, reg) &= ~iir;
	return true;
}

bool vgt_reg_isr_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_dbg("IRQ: capture ISR write on reg (%x) with val (%x)." \
		" Will be ignored!\n", reg, *(vgt_reg_t *)p_data);

	return true;
}

/* =======================vEvent injection===================== */

extern int resend_irq_on_evtchn(unsigned int i915_irq);

static void inject_dom0_virtual_interrupt(struct vgt_device *vgt)
{
	unsigned long flags;
	int i915_irq = vgt->pdev->irq_hstate->i915_irq;

	vgt_dbg("vGT: resend irq for dom0\n");
	/* resend irq may unmask events which requires irq disabled */
	local_irq_save(flags);
	resend_irq_on_evtchn(i915_irq);
	local_irq_restore(flags);
}

#define MSI_CAP_OFFSET 0x90	/* FIXME. need to get from cfg emulation */
#define MSI_CAP_CONTROL (MSI_CAP_OFFSET + 2)
#define MSI_CAP_ADDRESS (MSI_CAP_OFFSET + 4)
#define MSI_CAP_DATA	(MSI_CAP_OFFSET + 8)
static void inject_hvm_virtual_interrupt(struct vgt_device *vgt)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint16_t control = *(uint16_t *)(cfg_space + MSI_CAP_CONTROL);
	struct xen_hvm_inject_msi info;
	int r;

	/* FIXME: now only handle one MSI format */
	ASSERT_NUM(!(control & 0xfffe), control);
	info.domid = vgt->vm_id;
	info.addr = *(uint32_t *)(cfg_space + MSI_CAP_ADDRESS);
	info.data = *(uint16_t *)(cfg_space + MSI_CAP_DATA);
	vgt_dbg("vGT: VM(%d): hvm injections. address (%llx) data(%x)!\n",
		vgt->vm_id, info.addr, info.data);
	r = HYPERVISOR_hvm_op(HVMOP_inject_msi, &info);
	if (r < 0)
		vgt_err("vGT(%d): failed to inject vmsi\n", vgt->vgt_id);
}

static int vgt_inject_virtual_interrupt(struct vgt_device *vgt)
{
	if (vgt->vm_id)
		inject_hvm_virtual_interrupt(vgt);
	else
		inject_dom0_virtual_interrupt(vgt);

	vgt->stat.irq_num++;
	vgt->stat.last_injection = get_cycles();
	return 0;
}

static void vgt_propagate_event(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	int bit;
	struct vgt_irq_info *info;
	unsigned int reg_base;

	info = vgt_get_irq_info(hstate, event);
	if (!info) {
		vgt_err("IRQ(%d): virt-inject: no irq reg info!!!\n",
			vgt->vm_id);
		return;
	}

	reg_base = info->reg_base;
	bit = hstate->events[event].bit;

	/*
         * this function call is equivalent to a rising edge ISR
         * TODO: need check 2nd level IMR for render events
         */
	if (!test_bit(bit, (void*)vgt_vreg(vgt, regbase_to_imr(reg_base)))) {
		vgt_dbg("IRQ: set bit (%d) for (%s) for VM (%d)\n",
			bit, vgt_irq_name[event], vgt->vm_id);
		set_bit(bit, (void*)vgt_vreg(vgt, regbase_to_iir(reg_base)));

		/* enabled PCH events needs queue in level-1 display */
		if (info == hstate->info[IRQ_INFO_PCH] &&
			test_bit(bit, (void*)vgt_vreg(vgt, regbase_to_ier(reg_base))))
			vgt_propagate_event(hstate, PCH_IRQ, vgt);
	}
}

/* =======================vEvent Handlers===================== */

static void vgt_handle_default_event_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	if (!vgt_irq_warn_once[vgt->vgt_id][event]) {
		vgt_info("IRQ: VM(%d) receive event (%s)\n",
			vgt->vm_id, vgt_irq_name[event]);
		vgt_irq_warn_once[vgt->vgt_id][event] = 1;
	}
	vgt_propagate_event(hstate, event, vgt);
	vgt->stat.events[event]++;
}

static void vgt_handle_phase_in_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	__vreg(vgt, _REG_BLC_PWM_CTL2) |= _REGBIT_PHASE_IN_IRQ_STATUS;
	vgt_handle_default_event_virt(hstate, event, vgt);
}

static void vgt_handle_histogram_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	__vreg(vgt, _REG_HISTOGRAM_THRSH) |= _REGBIT_HISTOGRAM_IRQ_STATUS;
	vgt_handle_default_event_virt(hstate, event, vgt);
}

static void vgt_handle_crt_hotplug_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	/* update channel status */
	if (__vreg(vgt, _REG_PCH_ADPA) & _REGBIT_ADPA_CRT_HOTPLUG_ENABLE) {

		__vreg(vgt, _REG_PCH_ADPA) &=
				~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;

		__vreg(vgt, _REG_PCH_ADPA) |=
					vgt_get_event_val(hstate, event) &
					_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;

		vgt_handle_default_event_virt(hstate, event, vgt);
	}
}

static void vgt_handle_port_hotplug_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	vgt_reg_t enable_mask, status_mask;

	if (event == DP_B_HOTPLUG) {
		enable_mask = _REGBIT_DP_B_ENABLE;
		status_mask = _REGBIT_DP_B_STATUS;
	} else if (event == DP_C_HOTPLUG) {
		enable_mask = _REGBIT_DP_C_ENABLE;
		status_mask = _REGBIT_DP_C_STATUS;
	} else {
		ASSERT(event == DP_D_HOTPLUG);
		enable_mask = _REGBIT_DP_D_ENABLE;
		status_mask = _REGBIT_DP_D_STATUS;
	}

	if (__vreg(vgt, _REG_SHOTPLUG_CTL) & enable_mask) {

		__vreg(vgt, _REG_SHOTPLUG_CTL) &= ~status_mask;
		__vreg(vgt, _REG_SHOTPLUG_CTL) |=
				vgt_get_event_val(hstate, event) & status_mask;

		vgt_handle_default_event_virt(hstate, event, vgt);
	}
}

/* =======================pEvent Handlers===================== */

static void vgt_handle_default_event_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	if (!vgt_irq_warn_once[VGT_MAX_VMS][event]) {
		vgt_info("IRQ: receive event (%s)\n",
				vgt_irq_name[event]);
		vgt_irq_warn_once[VGT_MAX_VMS][event] = 1;
	}
}

static void vgt_handle_phase_in_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	uint32_t val;
	struct pgt_device *pdev = hstate->pdev;

	val = VGT_MMIO_READ(pdev, _REG_BLC_PWM_CTL2);
	val &= ~_REGBIT_PHASE_IN_IRQ_STATUS;
	VGT_MMIO_WRITE(pdev, _REG_BLC_PWM_CTL2, val);

	vgt_handle_default_event_phys(hstate, event);
}

static void vgt_handle_histogram_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	uint32_t val;
	struct pgt_device *pdev = hstate->pdev;

	val = VGT_MMIO_READ(pdev, _REG_HISTOGRAM_THRSH);
	val &= ~_REGBIT_HISTOGRAM_IRQ_STATUS;
	VGT_MMIO_WRITE(pdev, _REG_HISTOGRAM_THRSH, val);

	vgt_handle_default_event_phys(hstate, event);
}

/*
 * It's said that CRT hotplug detection through below method does not
 * always work. For example in Linux i915 not hotplug handler is installed
 * for CRT (likely through some other polling method). But let's use this
 * as the example for how hotplug event is generally handled here.
 */
static void vgt_handle_crt_hotplug_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	vgt_reg_t adpa_ctrl;
	struct pgt_device *pdev = hstate->pdev;

	adpa_ctrl = VGT_MMIO_READ(pdev, _REG_PCH_ADPA);
	if (!(adpa_ctrl & _REGBIT_ADPA_DAC_ENABLE)) {
		vgt_warn("IRQ: captured CRT hotplug event when CRT is disabled\n");
	}

	/* write back value to clear channel status */
	VGT_MMIO_WRITE(pdev, _REG_PCH_ADPA, adpa_ctrl);

	/* check blue/green channel status for attachment status */
	if (adpa_ctrl & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK) {
		vgt_info("IRQ: detect crt insert event!\n");
		vgt_set_uevent(vgt_dom0, CRT_HOTPLUG_IN);
	} else {
		vgt_info("IRQ: detect crt removal event!\n");
		vgt_set_uevent(vgt_dom0, CRT_HOTPLUG_OUT);
	}

	vgt_set_event_val(hstate, event, adpa_ctrl);
	/* send out udev events when handling physical interruts */
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);

	vgt_handle_default_event_phys(hstate, event);
}

static void vgt_handle_port_hotplug_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	vgt_reg_t hotplug_ctrl;
	vgt_reg_t enable_mask, status_mask, tmp;
	enum vgt_uevent_type hotplug_event;
	struct pgt_device *pdev = hstate->pdev;

	if (event == DP_B_HOTPLUG) {
		enable_mask = _REGBIT_DP_B_ENABLE;
		status_mask = _REGBIT_DP_B_STATUS;
		hotplug_event = PORT_B_HOTPLUG_IN;
	} else if (event == DP_C_HOTPLUG) {
		enable_mask = _REGBIT_DP_C_ENABLE;
		status_mask = _REGBIT_DP_C_STATUS;
		hotplug_event = PORT_C_HOTPLUG_IN;
	} else {
		ASSERT(event == DP_D_HOTPLUG);
		enable_mask = _REGBIT_DP_D_ENABLE;
		status_mask = _REGBIT_DP_D_STATUS;
		hotplug_event = PORT_D_HOTPLUG_IN;
	}

	hotplug_ctrl = VGT_MMIO_READ(pdev, _REG_SHOTPLUG_CTL);

	if (!(hotplug_ctrl & enable_mask)) {
		vgt_warn("IRQ: captured port hotplug event when HPD is disabled\n");
	}

	tmp = hotplug_ctrl & ~(_REGBIT_DP_B_STATUS |
				_REGBIT_DP_C_STATUS |
				_REGBIT_DP_D_STATUS);
	tmp |= hotplug_ctrl & status_mask;
	/* write back value to clear specific port status */
	VGT_MMIO_WRITE(pdev, _REG_SHOTPLUG_CTL, tmp);

	if (hotplug_ctrl & status_mask) {
		vgt_info("IRQ: detect monitor insert event on port!\n");
		vgt_set_uevent(vgt_dom0, hotplug_event);
	} else {
		vgt_info("IRQ: detect monitor removal eventon port!\n");
		vgt_set_uevent(vgt_dom0, hotplug_event + 1);
	}

	vgt_set_event_val(hstate, event, hotplug_ctrl);
	/* send out udev events when handling physical interruts */
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);

	vgt_handle_default_event_phys(hstate, event);
}

/* =====================GEN specific logic======================= */

/*
 * Here we only check IIR/IER. IMR/ISR is not checked
 * because only rising-edge of ISR is captured as an event,
 * so that current value of vISR doesn't matter.
 */
static void vgt_base_check_pending_irq(struct vgt_device *vgt)
{
	struct vgt_irq_host_state *hstate = vgt->pdev->irq_hstate;

	if (!(__vreg(vgt, _REG_DEIER) & _REGBIT_MASTER_INTERRUPT))
		return;

	/* first try 2nd level PCH pending events */
	if ((__vreg(vgt, _REG_SDEIIR) & __vreg(vgt, _REG_SDEIER)))
		vgt_propagate_event(hstate, PCH_IRQ, vgt);

	/* then check 1st level pending events */
	if ((__vreg(vgt, _REG_DEIIR) & __vreg(vgt, _REG_DEIER)) ||
	    (__vreg(vgt, _REG_GTIIR) & __vreg(vgt, _REG_GTIER)) ||
	    (__vreg(vgt, _REG_PMIIR) & __vreg(vgt, _REG_PMIER))) {
		vgt_inject_virtual_interrupt(vgt);
	}
}

/* base interrupt handler, for snb/ivb/hsw */
static irqreturn_t vgt_base_irq_handler(struct vgt_irq_host_state *hstate)
{
	u32 gt_iir, pm_iir, de_iir, pch_iir, de_iir_tmp;
	int pch_bit;
	struct pgt_device *pdev = hstate->pdev;

	/* read physical IIRs */
	gt_iir = VGT_MMIO_READ(pdev, _REG_GTIIR);
	de_iir = VGT_MMIO_READ(pdev, _REG_DEIIR);
	pm_iir = VGT_MMIO_READ(pdev, _REG_PMIIR);
	pch_iir = VGT_MMIO_READ(pdev, _REG_SDEIIR);

	if (!gt_iir && !de_iir && !pm_iir)
		return IRQ_NONE;

	vgt_handle_events(hstate, &gt_iir, IRQ_INFO_GT);

	pch_bit = hstate->events[PCH_IRQ].bit;
	ASSERT(hstate->events[PCH_IRQ].info);
	de_iir_tmp = de_iir & (~(1 << pch_bit));
	vgt_handle_events(hstate, &de_iir_tmp, IRQ_INFO_DPY);

	vgt_handle_events(hstate, &pm_iir, IRQ_INFO_PM);

	if (de_iir & (1 << pch_bit)) {
		vgt_handle_events(hstate, &pch_iir, IRQ_INFO_PCH);
		VGT_MMIO_WRITE(pdev, _REG_SDEIIR, pch_iir);
	}

	VGT_MMIO_WRITE(pdev, _REG_GTIIR, gt_iir);
	VGT_MMIO_WRITE(pdev, _REG_PMIIR, pm_iir);
	VGT_MMIO_WRITE(pdev, _REG_DEIIR, de_iir);

	return IRQ_HANDLED;
}

/* SNB/IVB/HSW share the similar interrupt register scheme */
static struct vgt_irq_info vgt_base_gt_info = {
	.name = "GT-IRQ",
	.reg_base = _REG_GTISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_dpy_info = {
	.name = "DPY-IRQ",
	.reg_base = _REG_DEISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_pch_info = {
	.name = "PCH-IRQ",
	.reg_base = _REG_SDEISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_pm_info = {
	.name = "PM-IRQ",
	.reg_base = _REG_PMISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

/* associate gen specific register bits to general events */
/* TODO: add all hardware bit definitions */
static void vgt_base_init_irq(
	struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

#define SET_BIT_INFO(s, b, e, i)		\
	do {					\
		s->events[e].bit = b;		\
		s->events[e].info = s->info[i];	\
		s->info[i]->bit_to_event[b] = e;\
	} while (0);

	hstate->info[IRQ_INFO_GT] = &vgt_base_gt_info;
	hstate->info[IRQ_INFO_DPY] = &vgt_base_dpy_info;
	hstate->info[IRQ_INFO_PCH] = &vgt_base_pch_info;
	hstate->info[IRQ_INFO_PM] = &vgt_base_pm_info;

	/* Render events */
	SET_BIT_INFO(hstate, 0, RCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 4, RCS_PIPE_CONTROL, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 12, VCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 16, VCS_MI_FLUSH_DW, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 22, BCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 26, BCS_MI_FLUSH_DW, IRQ_INFO_GT);
	/* No space in GT, so put it in PM */
	SET_BIT_INFO(hstate, 13, VECS_MI_FLUSH_DW, IRQ_INFO_PM);

	/* Display events */
	if (IS_IVB(pdev) || IS_HSW(pdev)) {
		SET_BIT_INFO(hstate, 0, PIPE_A_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 3, PRIMARY_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 4, SPRITE_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 5, PIPE_B_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 8, PRIMARY_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 9, SPRITE_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 10, PIPE_C_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 13, PRIMARY_C_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 14, SPRITE_C_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 24, DPST_PHASE_IN, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 25, DPST_HISTOGRAM, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 26, AUX_CHANNEL_A, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 27, DP_A_HOTPLUG, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 28, PCH_IRQ, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 29, GSE, IRQ_INFO_DPY);
	} else if (IS_SNB(pdev)) {
		SET_BIT_INFO(hstate, 7, PIPE_A_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 15, PIPE_B_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 16, DPST_PHASE_IN, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 17, DPST_HISTOGRAM, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 18, GSE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 19, DP_A_HOTPLUG, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 20, AUX_CHANNEL_A, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 21, PCH_IRQ, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 26, PRIMARY_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 27, PRIMARY_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 28, SPRITE_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 29, SPRITE_B_FLIP_DONE, IRQ_INFO_DPY);
	}

	/* PM events */
	SET_BIT_INFO(hstate, 1, GV_DOWN_INTERVAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 2, GV_UP_INTERVAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 4, RP_DOWN_THRESHOLD, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 5, RP_UP_THRESHOLD, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 6, FREQ_DOWNWARD_TIMEOUT_RC6, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 24, PCU_THERMAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 25, PCU_PCODE2DRIVER_MAILBOX, IRQ_INFO_PM);

	/* PCH events */
	SET_BIT_INFO(hstate, 17, GMBUS, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 19, CRT_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 21, DP_B_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 22, DP_C_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 23, DP_D_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 25, AUX_CHENNEL_B, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 26, AUX_CHENNEL_C, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 27, AUX_CHENNEL_D, IRQ_INFO_PCH);
}

struct vgt_irq_ops vgt_base_irq_ops = {
	.irq_handler = vgt_base_irq_handler,
	.init_irq = vgt_base_init_irq,
	.check_pending_irq = vgt_base_check_pending_irq,
};

/* ======================common event logic====================== */

/*
 * Trigger a virtual event which comes from other requests like hotplug agent
 * instead of from pirq.
 */
void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	vgt_event_virt_handler_t handler;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	ASSERT(spin_is_locked(&pdev->lock));

	handler = vgt_get_event_virt_handler(hstate, event);
	ASSERT(handler);

	handler(hstate, event, vgt);

	ops->check_pending_irq(vgt);
}

/*
 * Forward cached physical events to VMs, invoked from kernel thread
 */
void vgt_forward_events(struct pgt_device *pdev)
{
	int i, event;
	cycles_t delay;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	vgt_event_virt_handler_t handler;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	/* WARING: this should be under lock protection */
	//raise_ctx_sched(vgt_dom0);

	pdev->stat.last_virq = get_cycles();
	delay = pdev->stat.last_virq - pdev->stat.last_pirq;

	/*
	 * it's possible a new pirq coming before last request is handled.
	 * or the irq may come before kthread is ready. So skip the 1st 5.
	 */
	if (delay > 0 && pdev->stat.irq_num > 5)
		pdev->stat.irq_delay_cycles += delay;

	ASSERT(spin_is_locked(&pdev->lock));
	for_each_set_bit(event, hstate->pending_events, EVENT_MAX) {
		clear_bit(event, hstate->pending_events);

		handler = vgt_get_event_virt_handler(hstate, event);
		ASSERT(handler);

		switch (vgt_get_event_policy(hstate, event)) {
		case EVENT_FW_ALL:
			for (i = 0; i < VGT_MAX_VMS; i++) {
				if (pdev->device[i])
					handler(hstate, event, pdev->device[i]);
			}
			break;
		case EVENT_FW_DOM0:
			handler(hstate, event, vgt_dom0);
			break;
		case EVENT_FW_NONE:
		default:
			break;
		}
	}

	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i])
			ops->check_pending_irq(pdev->device[i]);
	}

	pdev->stat.virq_cycles += get_cycles() - pdev->stat.last_virq;
}

/*
 * Scan all pending events in the specified category, and then invoke
 * registered handler accordingly
 */
static void vgt_handle_events(struct vgt_irq_host_state *hstate, void *iir,
	enum vgt_irq_type type)
{
	int bit;
	enum vgt_event_type event;
	struct vgt_irq_info *info = hstate->info[type];
	vgt_event_phys_handler_t handler;
	struct pgt_device *pdev = hstate->pdev;

	ASSERT(spin_is_locked(&pdev->lock));

	for_each_set_bit(bit, iir, VGT_IRQ_BITWIDTH) {
		event = info->bit_to_event[bit];
		pdev->stat.events[event]++;

		if (unlikely(event == EVENT_RESERVED)) {
			vgt_err("IRQ: abandon non-registered [%s, bit-%d] event (%s)\n",
				info->name, bit, vgt_irq_name[event]);
			continue;
		}

		handler = vgt_get_event_phys_handler(hstate, event);
		ASSERT(handler);

		handler(hstate, event);
		set_bit(event, hstate->pending_events);
	}
}

/*
 * Physical interrupt handler for Intel HD serious graphics
 *   - handle various interrupt reasons
 *   - may trigger virtual interrupt instances to dom0 or other VMs
 */
static irqreturn_t vgt_interrupt(int irq, void *data)
{
	struct pgt_device *pdev = (struct pgt_device *)data;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	u32 de_ier;
	irqreturn_t ret;

	pdev->stat.irq_num++;
	pdev->stat.last_pirq = get_cycles();

	spin_lock(&pdev->lock);
	vgt_dbg("IRQ: receive interrupt (de-%x, gt-%x, pch-%x, pm-%x)\n",
		VGT_MMIO_READ(pdev, _REG_DEIIR),
		VGT_MMIO_READ(pdev, _REG_GTIIR),
		VGT_MMIO_READ(pdev, _REG_SDEIIR),
		VGT_MMIO_READ(pdev, _REG_PMIIR));

	/* avoid nested handling by disabling master interrupt */
	de_ier = VGT_MMIO_READ(pdev, _REG_DEIER);
	VGT_MMIO_WRITE(pdev, _REG_DEIER, de_ier & ~_REGBIT_MASTER_INTERRUPT);

	ret = hstate->ops->irq_handler(hstate);
	if (ret == IRQ_NONE) {
		vgt_dbg("Spurious interrupt received (or shared vector)\n");
		goto out;
	}

	vgt_raise_request(pdev, VGT_REQUEST_IRQ);

out:
	/* re-enable master interrupt */
	VGT_MMIO_WRITE(pdev, _REG_DEIER, de_ier);
	spin_unlock(&pdev->lock);

	pdev->stat.pirq_cycles += get_cycles() - pdev->stat.last_pirq;
	return IRQ_HANDLED;
}

/* default handler will be invoked, if not explicitly specified here */
static void vgt_init_events(
	struct vgt_irq_host_state *hstate)
{
	int i;

#define SET_POLICY_ALL(h, e)	\
	((h)->events[e].policy = EVENT_FW_ALL)
#define SET_POLICY_DOM0(h, e)	\
	((h)->events[e].policy = EVENT_FW_DOM0)
#define SET_POLICY_NONE(h, e)	\
	((h)->events[e].policy = EVENT_FW_NONE)
#define SET_P_HANDLER(s, e, h)	\
	((s)->events[e].p_handler = h)
#define SET_V_HANDLER(s, e, h)	\
	((s)->events[e].v_handler = h)

	for (i = 0; i < EVENT_MAX; i++) {
		hstate->events[i].info = NULL;
		/* Default forwarding to all VMs (render and most display events) */
		SET_POLICY_ALL(hstate, i);
		hstate->events[i].p_handler = vgt_handle_default_event_phys;
		hstate->events[i].v_handler = vgt_handle_default_event_virt;;
	}

	SET_P_HANDLER(hstate, DPST_PHASE_IN, vgt_handle_phase_in_phys);
	SET_P_HANDLER(hstate, DPST_HISTOGRAM, vgt_handle_histogram_phys);
	SET_P_HANDLER(hstate, CRT_HOTPLUG, vgt_handle_crt_hotplug_phys);
	SET_P_HANDLER(hstate, DP_B_HOTPLUG, vgt_handle_port_hotplug_phys);
	SET_P_HANDLER(hstate, DP_C_HOTPLUG, vgt_handle_port_hotplug_phys);
	SET_P_HANDLER(hstate, DP_D_HOTPLUG, vgt_handle_port_hotplug_phys);

	SET_V_HANDLER(hstate, DPST_PHASE_IN, vgt_handle_phase_in_virt);
	SET_V_HANDLER(hstate, DPST_HISTOGRAM, vgt_handle_histogram_virt);
	SET_V_HANDLER(hstate, CRT_HOTPLUG, vgt_handle_crt_hotplug_virt);
	SET_V_HANDLER(hstate, DP_B_HOTPLUG, vgt_handle_port_hotplug_virt);
	SET_V_HANDLER(hstate, DP_C_HOTPLUG, vgt_handle_port_hotplug_virt);
	SET_V_HANDLER(hstate, DP_D_HOTPLUG, vgt_handle_port_hotplug_virt);

	/* for engine specific reset */
	SET_POLICY_DOM0(hstate, RCS_WATCHDOG_EXCEEDED);
	SET_POLICY_DOM0(hstate, VCS_WATCHDOG_EXCEEDED);

	/* ACPI OpRegion belongs to dom0 */
	SET_POLICY_DOM0(hstate, GSE);

	/* render-p/c fully owned by Dom0 */
	SET_POLICY_DOM0(hstate, GV_DOWN_INTERVAL);
	SET_POLICY_DOM0(hstate, GV_UP_INTERVAL);
	SET_POLICY_DOM0(hstate, RP_DOWN_THRESHOLD);
	SET_POLICY_DOM0(hstate, RP_UP_THRESHOLD);
	SET_POLICY_DOM0(hstate, FREQ_DOWNWARD_TIMEOUT_RC6);
	SET_POLICY_DOM0(hstate, PCU_THERMAL);
	SET_POLICY_DOM0(hstate, PCU_PCODE2DRIVER_MAILBOX);

	/* Audio owned by Dom0 */
	SET_POLICY_DOM0(hstate, AUDIO_CP_CHANGE_TRANSCODER_A);
	SET_POLICY_DOM0(hstate, AUDIO_CP_REQUEST_TRANSCODER_A);
	SET_POLICY_DOM0(hstate, AUDIO_CP_CHANGE_TRANSCODER_B);
	SET_POLICY_DOM0(hstate, AUDIO_CP_REQUEST_TRANSCODER_B);
	SET_POLICY_DOM0(hstate, AUDIO_CP_CHANGE_TRANSCODER_C);
	SET_POLICY_DOM0(hstate, AUDIO_CP_REQUEST_TRANSCODER_C);

	/* Monitor interfaces are controlled by XenGT driver */
	SET_POLICY_NONE(hstate, DP_A_HOTPLUG);
	SET_POLICY_NONE(hstate, DP_B_HOTPLUG);
	SET_POLICY_NONE(hstate, DP_C_HOTPLUG);
	SET_POLICY_NONE(hstate, DP_D_HOTPLUG);
	SET_POLICY_NONE(hstate, SDVO_B_HOTPLUG);
	SET_POLICY_NONE(hstate, CRT_HOTPLUG);
	SET_POLICY_NONE(hstate, AUX_CHANNEL_A);
	SET_POLICY_NONE(hstate, AUX_CHENNEL_B);
	SET_POLICY_NONE(hstate, AUX_CHENNEL_C);
	SET_POLICY_NONE(hstate, AUX_CHENNEL_D);
	SET_POLICY_NONE(hstate, GMBUS);
}

/*
 * Do interrupt initialization for vGT driver
 */
int vgt_irq_init(struct pgt_device *pdev)
{
	struct vgt_irq_host_state *hstate;

	hstate = kzalloc(sizeof(struct vgt_irq_host_state), GFP_KERNEL);
	if (hstate == NULL)
		return -ENOMEM;

	if (IS_SNB(pdev) || IS_IVB(pdev) || IS_HSW(pdev))
		hstate->ops = &vgt_base_irq_ops;
	else {
		vgt_err("Unsupported device\n");
		kfree(hstate);
		return -EINVAL;
	}

	hstate->pdev = pdev;
	//hstate.i915_irq = IRQ_INVALID;
	//hstate.pirq = IRQ_INVALID;

	/* common event initialization */
	vgt_init_events(hstate);

	/* gen specific initialization */
	hstate->ops->init_irq(hstate);

	pdev->irq_hstate = hstate;
	vgt_dbg("Interrupt initialization completes\n");
	return 0;
}

void vgt_irq_exit(struct pgt_device *pdev)
{
	free_irq(pdev->irq_hstate->pirq, pdev);

	/* TODO: recover i915 handler? */
	//unbind_from_irq(vgt_i915_irq(pdev));

	kfree(pdev->irq_hstate);
}

void vgt_install_irq(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;
	int irq, ret;
	struct vgt_irq_host_state *hstate;

	if (!xen_initial_domain() || !vgt_enabled)
		return;

	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered when installing irq\n");
		return;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device when registering irq\n");
		return;
	}

	printk("vGT: found matching pgt_device when registering irq for dev (0x%x)\n", pdev->devfn);

	irq = bind_virq_to_irq(VIRQ_VGT_GFX, 0);
	if (irq < 0) {
		printk("vGT: fail to bind virq\n");
		return;
	}

	ret = request_irq(pdev->irq, vgt_interrupt, IRQF_SHARED, "vgt", pgt);
	if (ret < 0) {
		printk("vGT: error on request_irq (%d)\n", ret);
		//unbind_from_irq(irq);
		return;
	}

	hstate = pgt->irq_hstate;
	hstate->pirq = pdev->irq;
	hstate->i915_irq = irq;
	pdev->irq = irq;

	printk("vGT: allocate virq (%d) for i915, while keep original irq (%d) for vgt\n",
		hstate->i915_irq, hstate->pirq);
}

void vgt_uninstall_irq(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;
	struct vgt_irq_host_state *hstate;

	if (!xen_initial_domain() || !vgt_enabled)
		return;

	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered when installing irq\n");
		return;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device when registering irq\n");
		return;
	}

	/* Mask all GEN interrupts */
	VGT_MMIO_WRITE(pgt, _REG_DEIER,
		VGT_MMIO_READ(pgt, _REG_DEIER) & ~_REGBIT_MASTER_INTERRUPT);

	hstate = pgt->irq_hstate;

	free_irq(hstate->pirq, pgt);
	//unbind_from_irq(hstate->pirq);

	pdev->irq = hstate->pirq; /* needed by __pci_restore_msi_state() */
}

EXPORT_SYMBOL(vgt_install_irq);
EXPORT_SYMBOL(vgt_uninstall_irq);
