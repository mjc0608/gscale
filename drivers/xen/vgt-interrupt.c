/*
 * vGT interrupt handler
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
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <xen/events.h>
#include <xen/vgt.h>
#include "vgt_reg.h"

/*
 * Major tasks of this file:
 *   - handle physical GEN interrupt
 *   - interrupt virtualization to dom0 and other GT VMs
 *     o forward events carried in a physical interrupt, if it's
 *       programmed by a VM
 *     o emulate events if a given VM is not allowed to program them
 *     o interrupt register virtualization
 *   - track the permission of which events can be programmed by which VM
 *
 * Micellaneous tasks:
 *   - initialization of interrupt control registers
 *   - statistics
 */

/*
 * TODO-must:
 *   - virtual interrupt injection
 *   - display context switch
 *   - lock consideration. there may be events requiring more register
 *     updates other than IIR/ISR. Could they be done in a delayed context?
 *
 * TODO:
 *   - IIR could store two pending interrupts. need emulate the behavior
 *   - pipe control
 *   - error handling (related registers)
 *	page fault (GFX_ARB_ERROR_RPT, PP_PFIR)
 *	cmd error (IPEHR)
 *   - reserved bit checking in vREG?
 *   - watchdog timer control (write 1 to reset, and write 0 to start)
 *	PR_CTR_CTL, BCS_CTR_THRSH, VCS_ECOSKPD, VCS_CNTR
 *	interresting that blitter has no such interrupt
 */

/* simply forward interrupt to dom0 i915, for testing the hook mechanism */
//#define VGT_IRQ_FORWARD_MODE

/* only handle IIR/IMR/IER, with all events handled by default handler */
#define VGT_IRQ_DEFAULT_HANDLER

/* for debug purpose */
uint8_t vgt_irq_warn_once[IRQ_MAX] = {0};
char *vgt_irq_name[IRQ_MAX] = {
	// GT
	[IRQ_RCS_MI_USER_INTERRUPT] = "Render Command Streamer MI USER INTERRUPT",
	[IRQ_RCS_DEBUG] = "Render EU debug from SVG",
	[IRQ_RCS_MMIO_SYNC_FLUSH] = "Render MMIO sync flush status",
	[IRQ_RCS_CMD_STREAMER_ERR] = "Render Command Streamer error interrupt",
	[IRQ_RCS_PIPE_CONTROL] = "ender PIPE CONTROL notify",
	[IRQ_RCS_WATCHDOG_EXCEEDED] = "Render Command Streamer Watchdog counter exceeded",
	[IRQ_RCS_PAGE_DIRECTORY_FAULT] = "Render page directory faults",
	[IRQ_RCS_AS_CONTEXT_SWITCH] = "Render AS Context Switch Interrupt",

	[IRQ_VCS_MI_USER_INTERRUPT] = "Video Command Streamer MI USER INTERRUPT",
	[IRQ_VCS_MMIO_SYNC_FLUSH] = "Video MMIO sync flush status",
	[IRQ_VCS_CMD_STREAMER_ERR] = "Video Command Streamer error interrupt",
	[IRQ_VCS_MI_FLUSH_DW] = "Video MI FLUSH DW notify",
	[IRQ_VCS_WATCHDOG_EXCEEDED] = "Video Command Streamer Watchdog counter exceeded",
	[IRQ_VCS_PAGE_DIRECTORY_FAULT] = "Video page directory faults",
	[IRQ_VCS_AS_CONTEXT_SWITCH] = "Video AS Context Switch Interrupt",

	[IRQ_BCS_MI_USER_INTERRUPT] = "Blitter Command Streamer MI USER INTERRUPT",
	[IRQ_BCS_MMIO_SYNC_FLUSH] = "Billter MMIO sync flush status",
	[IRQ_BCS_CMD_STREAMER_ERR] = "Blitter Command Streamer error interrupt",
	[IRQ_BCS_MI_FLUSH_DW] = "Blitter MI FLUSH DW notify",
	[IRQ_BCS_PAGE_DIRECTORY_FAULT] = "Blitter page directory faults",
	[IRQ_BCS_AS_CONTEXT_SWITCH] = "Blitter AS Context Switch Interrupt",

	// DISPLAY
	[IRQ_PIPE_A_FIFO_UNDERRUN] = "Pipe A FIFO underrun",
	[IRQ_PIPE_A_CRC_ERR] = "Pipe A CRC error",
	[IRQ_PIPE_A_CRC_DONE] = "Pipe A CRC done",
	[IRQ_PIPE_A_VSYNC] = "Pipe A vsync",
	[IRQ_PIPE_A_LINE_COMPARE] = "Pipe A line compare",
	[IRQ_PIPE_A_ODD_FIELD] = "Pipe A odd field",
	[IRQ_PIPE_A_EVEN_FIELD] = "Pipe A even field",
	[IRQ_PIPE_A_VBLANK] = "Pipe A vblank",
	[IRQ_PIPE_B_FIFO_UNDERRUN] = "Pipe B FIFO underrun",
	[IRQ_PIPE_B_CRC_ERR] = "Pipe B CRC error",
	[IRQ_PIPE_B_CRC_DONE] = "Pipe B CRC done",
	[IRQ_PIPE_B_VSYNC] = "Pipe B vsync",
	[IRQ_PIPE_B_LINE_COMPARE] = "Pipe B line compare",
	[IRQ_PIPE_B_ODD_FIELD] = "Pipe B odd field",
	[IRQ_PIPE_B_EVEN_FIELD] = "Pipe B even field",
	[IRQ_PIPE_B_VBLANK] = "Pipe B vblank",
	[IRQ_DPST_PHASE_IN] = "DPST phase in event",
	[IRQ_DPST_HISTOGRAM] = "DPST histogram event",
	[IRQ_GSE] = "GSE",
	[IRQ_DP_A_HOTPLUG] = "DP A Hotplug",
	[IRQ_AUX_CHANNEL_A] = "AUX Channel A",
	[IRQ_PCH_IRQ] = "PCH Display interrupt event",
	[IRQ_PERF_COUNTER] = "Performance counter",
	[IRQ_POISON] = "Poison",
	[IRQ_GTT_FAULT] = "GTT fault",
	[IRQ_PRIMARY_A_FLIP_DONE] = "Primary Plane A flip done",
	[IRQ_PRIMARY_B_FLIP_DONE] = "Primary Plane B flip done",
	[IRQ_SPRITE_A_FLIP_DONE] = "Sprite Plane A flip done",
	[IRQ_SPRITE_B_FLIP_DONE] = "Sprite Plane B flip done",

	// PM
	[IRQ_GV_DOWN_INTERVAL] = "Render geyserville Down evaluation interval interrupt",
	[IRQ_GV_UP_INTERVAL] = "Render geyserville UP evaluation interval interrupt",
	[IRQ_RP_DOWN_THRESHOLD] = "RP DOWN threshold interrupt",
	[IRQ_RP_UP_THRESHOLD] = "RP UP threshold interrupt",
	[IRQ_FREQ_DOWNWARD_TIMEOUT_RC6] = "Render Frequency Downward Timeout During RC6 interrupt",
	[IRQ_PCU_THERMAL] = "PCU Thermal Event",
	[IRQ_PCU_PCODE2DRIVER_MAILBOX] = "PCU pcode2driver mailbox event",

	// PCH
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A] = "FDI RX Interrupts Combined A",
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_A] = "Audio CP Change Transcoder A",
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_A] = "Audio CP Request Transcoder A",
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_B] = "FDI RX Interrupts Combined B",
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_B] = "Audio CP Change Transcoder B",
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_B] = "Audio CP Request Transcoder B",
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_C] = "FDI RX Interrupts Combined C",
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_C] = "Audio CP Change Transcoder C",
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_C] = "Audio CP Request Transcoder C",
	[IRQ_ERR_AND_DBG] = "South Error and Debug Interupts Combined",
	[IRQ_GMBUS] = "Gmbus",
	[IRQ_SDVO_B_HOTPLUG] = "SDVO B hotplug",
	[IRQ_CRT_HOTPLUG] = "CRT Hotplug",
	[IRQ_DP_B_HOTPLUG] = "DisplayPort/HDMI/DVI B Hotplug",
	[IRQ_DP_C_HOTPLUG] = "DisplayPort/HDMI/DVI C Hotplug",
	[IRQ_DP_D_HOTPLUG] = "DisplayPort/HDMI/DVI D Hotplug",
	[IRQ_AUX_CHENNEL_B] = "AUX Channel B",
	[IRQ_AUX_CHENNEL_C] = "AUX Channel C",
	[IRQ_AUX_CHENNEL_D] = "AUX Channel D",
	[IRQ_AUDIO_POWER_STATE_CHANGE_B] = "Audio Power State change Port B",
	[IRQ_AUDIO_POWER_STATE_CHANGE_C] = "Audio Power State change Port C",
	[IRQ_AUDIO_POWER_STATE_CHANGE_D] = "Audio Power State change Port D",

	[IRQ_RESERVED] = "RESERVED EVENTS!!!",
};

/* default event owner mapping table. may be changed dynamically */
enum vgt_owner_type vgt_default_event_owner_table[IRQ_MAX] = {
	// GT
	[IRQ_RCS_MI_USER_INTERRUPT] = VGT_OT_RENDER,
	[IRQ_RCS_DEBUG] = VGT_OT_RENDER,
	[IRQ_RCS_MMIO_SYNC_FLUSH] = VGT_OT_RENDER,
	[IRQ_RCS_CMD_STREAMER_ERR] = VGT_OT_RENDER,
	[IRQ_RCS_PIPE_CONTROL] = VGT_OT_RENDER,
	[IRQ_RCS_WATCHDOG_EXCEEDED] = VGT_OT_RENDER,
	[IRQ_RCS_PAGE_DIRECTORY_FAULT] = VGT_OT_RENDER,
	[IRQ_RCS_AS_CONTEXT_SWITCH] = VGT_OT_RENDER,

	[IRQ_VCS_MI_USER_INTERRUPT] = VGT_OT_RENDER,
	[IRQ_VCS_MMIO_SYNC_FLUSH] = VGT_OT_RENDER,
	[IRQ_VCS_CMD_STREAMER_ERR] = VGT_OT_RENDER,
	[IRQ_VCS_MI_FLUSH_DW] = VGT_OT_RENDER,
	[IRQ_VCS_WATCHDOG_EXCEEDED] = VGT_OT_RENDER,
	[IRQ_VCS_PAGE_DIRECTORY_FAULT] = VGT_OT_RENDER,
	[IRQ_VCS_AS_CONTEXT_SWITCH] = VGT_OT_RENDER,

	[IRQ_BCS_MI_USER_INTERRUPT] = VGT_OT_RENDER,
	[IRQ_BCS_MMIO_SYNC_FLUSH] = VGT_OT_RENDER,
	[IRQ_BCS_CMD_STREAMER_ERR] = VGT_OT_RENDER,
	[IRQ_BCS_MI_FLUSH_DW] = VGT_OT_RENDER,
	[IRQ_BCS_PAGE_DIRECTORY_FAULT] = VGT_OT_RENDER,
	[IRQ_BCS_AS_CONTEXT_SWITCH] = VGT_OT_RENDER,

	// DISPLAY
	[IRQ_PIPE_A_FIFO_UNDERRUN] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_CRC_ERR] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_CRC_DONE] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_VSYNC] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_LINE_COMPARE] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_ODD_FIELD] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_EVEN_FIELD] = VGT_OT_DISPLAY,
	[IRQ_PIPE_A_VBLANK] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_FIFO_UNDERRUN] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_CRC_ERR] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_CRC_DONE] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_VSYNC] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_LINE_COMPARE] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_ODD_FIELD] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_EVEN_FIELD] = VGT_OT_DISPLAY,
	[IRQ_PIPE_B_VBLANK] = VGT_OT_DISPLAY,
	[IRQ_DPST_PHASE_IN] = VGT_OT_DISPLAY,	// ???
	[IRQ_DPST_HISTOGRAM] = VGT_OT_DISPLAY,	// ???
	[IRQ_GSE] = VGT_OT_MGMT,
	[IRQ_DP_A_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_AUX_CHANNEL_A] = VGT_OT_MGMT,
	[IRQ_PCH_IRQ] = VGT_OT_INVALID,		// 2nd level events
	[IRQ_PERF_COUNTER] = VGT_OT_DISPLAY,
	[IRQ_POISON] = VGT_OT_DISPLAY,		// ???
	[IRQ_GTT_FAULT] = VGT_OT_DISPLAY,	// ???
	[IRQ_PRIMARY_A_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_PRIMARY_B_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_SPRITE_A_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_SPRITE_B_FLIP_DONE] = VGT_OT_DISPLAY,

	// PM
	[IRQ_GV_DOWN_INTERVAL] = VGT_OT_PM,
	[IRQ_GV_UP_INTERVAL] = VGT_OT_PM,
	[IRQ_RP_DOWN_THRESHOLD] = VGT_OT_PM,
	[IRQ_RP_UP_THRESHOLD] = VGT_OT_PM,
	[IRQ_FREQ_DOWNWARD_TIMEOUT_RC6] = VGT_OT_PM,
	[IRQ_PCU_THERMAL] = VGT_OT_PM,
	[IRQ_PCU_PCODE2DRIVER_MAILBOX] = VGT_OT_PM,

	// PCH
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A] = VGT_OT_DISPLAY,	// ???
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_A] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_A] = VGT_OT_DISPLAY,
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_B] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_B] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_B] = VGT_OT_DISPLAY,
	[IRQ_FDI_RX_INTERRUPTS_TRANSCODER_C] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_CP_CHANGE_TRANSCODER_C] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_CP_REQUEST_TRANSCODER_C] = VGT_OT_DISPLAY,
	[IRQ_ERR_AND_DBG] = VGT_OT_DISPLAY,	// ???
	[IRQ_GMBUS] = VGT_OT_MGMT,
	[IRQ_SDVO_B_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_CRT_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_DP_B_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_DP_C_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_DP_D_HOTPLUG] = VGT_OT_MGMT,
	[IRQ_AUX_CHENNEL_B] = VGT_OT_MGMT,
	[IRQ_AUX_CHENNEL_C] = VGT_OT_MGMT,
	[IRQ_AUX_CHENNEL_D] = VGT_OT_MGMT,
	[IRQ_AUDIO_POWER_STATE_CHANGE_B] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_POWER_STATE_CHANGE_C] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_POWER_STATE_CHANGE_D] = VGT_OT_DISPLAY,

	[IRQ_RESERVED] = VGT_OT_INVALID,
};

static void vgt_run_emul(struct vgt_device *vstate,
		enum vgt_event_type event, int bit, bool enable);
/* =============Configurations (statc/dynamic)================ */

#if 0
/*
 * force enabling a hw event regardless of the owner setting
 * not optimal for multiple bits change
 */
static void vgt_enable_hw_event(struct pgt_device *dev,
	enum vgt_event_type event, int bit)
{
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	ops->toggle_hw_event(dev, event, bit, true);
}

/* force disabling a hw event regardless of the owner setting */
static void vgt_disable_hw_event(struct pgt_device *dev,
	enum vgt_event_type event, int bit)
{
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	ops->toggle_hw_event(dev, event, bit, false);
}

/*
 * register an event handler for vGT core itself
 *
 * TODO: Currently no lock protection here. It's assumed that registered
 * handler should prepare for handling spurious interrupt even after
 * unregisteration, since it's possible for a vGT-core-caused event coming
 * after the unregistration.
 */
static int vgt_core_register_event(struct vgt_device *dev,
	enum vgt_event_type event, vgt_core_handler_t core_handler)
{
	/* initialize core handlers info at the 1st registration */
	if (!vgt_core_event_handlers(dev)) {
		void *handlers;

		handlers = kzalloc(IRQ_MAX * sizeof(vgt_core_handler_t), GFP_KERNEL);
		if (!handlers) {
			dprintk("vGT: no enough memory for core handler table\n");
			return -ENOMEM;
		}
		vgt_core_event_handlers(dev) = handlers;
	}

	if (vgt_core_event_handler(dev, event)) {
		dprintk("vGT: existing handler for event (%s)\n", vgt_irq_name[event]);
		return -EBUSY;
	}

	vgt_core_event_handler(dev, event) = core_handler;
	vgt_enable_hw_event(dev, event, VGT_IRQ_BITWIDTH);
}

static int vgt_core_register_event(struct vgt_device *dev, enum vgt_event_type event)
{
	ASSERT(vgt_core_event_handlers(dev));
	ASSERT(vgt_core_event_handler(dev, event));

	vgt_disable_hw_event(dev, event, VGT_IRQ_BITWIDTH);
	vgt_core_event_handler(dev, event) = NULL;
}
#endif

void vgt_irq_toggle_emulations(struct vgt_device *vstate,
		enum vgt_owner_type owner, bool enable)
{
	int event;
	struct pgt_device *dev = vstate->pdev;

	for_each_set_bit(event, vgt_state_emulated_events(vstate), IRQ_MAX) {
		if (vgt_get_event_owner_type(dev, event) == owner &&
		    !test_bit(event, vgt_always_emulated_events(dev))) {
			if (enable) {
				vgt_run_emul(vstate, event, VGT_IRQ_BITWIDTH, true);
			} else {
				if (test_bit(event, vgt_state_emulated_events(vstate)))
					vgt_run_emul(vstate, event, VGT_IRQ_BITWIDTH, false);
			}
		}
	}
}

/*
 * Invoked from vGT core when the ownership gets changed
 * INPUT:
 * 	vstate: instance context to be poked
 * 	type: ownership type
 */
void vgt_irq_save_context(struct vgt_device *vstate, enum vgt_owner_type owner)
{
	struct pgt_device *pdev = vstate->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	if (owner != VGT_OT_RENDER || owner != VGT_OT_DISPLAY) {
		dprintk("Dynamic ownership update for type (%d) is prohibited\n", owner);
		return;
	}

	if (ops->save)
		ops->save(vstate, owner);

	/* start emulation for prev owner */
	if (owner == VGT_OT_DISPLAY)
		vgt_irq_toggle_emulations(vstate, owner, true);
}

/*
 * Invoked from vGT core when the ownership gets changed
 * INPUT:
 * 	vstate: instance context to be poked
 * 	owner: ownership type
 */
void vgt_irq_restore_context(struct vgt_device *vstate, enum vgt_owner_type owner)
{
	struct pgt_device *dev = vstate->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	if (owner != VGT_OT_RENDER || owner != VGT_OT_DISPLAY) {
		dprintk("Dynamic ownership update for type (%d) is prohibited\n", owner);
		return;
	}

	/* disable emulation for the new owner */
	if (owner == VGT_OT_DISPLAY)
		vgt_irq_toggle_emulations(vstate, owner, false);

	if (ops->restore)
		ops->restore(vstate, owner);
}

#if 0
/*
 * There may have requirements to update event<->owner relationship
 * in different scenarios, e.g. pass through a display hotplug control
 * to a primary VM. In such case an interface is required to allow
 * updating the mapping table dynamically.
 *
 * Since a rare case, leave it empty now
 */
int vgt_irq_update_event_ownership(struct vgt_device *dev,
	enum vgt_event_type event, enum vgt_owner_type owner)
{
	dprintk("Event ownership mapping change is not supported now\n");
	return -EINVAL;
}
#endif

/* ========================virq injection===================== */

extern int resend_irq_on_evtchn(unsigned int i915_irq);

void inject_dom0_virtual_interrupt(struct vgt_device *vgt)
{
	unsigned long flags;
	dprintk("vGT: resend irq for dom0\n");
	/* resend irq may unmask events which requires irq disabled */
	local_irq_save(flags);
	resend_irq_on_evtchn(vgt_i915_irq(vgt->pdev));
	local_irq_restore(flags);
}

void inject_hvm_virtual_interrupt(struct vgt_device *vgt)
{
	printk("vGT: hvm injection not supported yet!\n");
}

static int vgt_inject_virtual_interrupt(struct vgt_device *vstate)
{
	if (vstate->vgt_id)
		inject_hvm_virtual_interrupt(vstate);
	else
		inject_dom0_virtual_interrupt(vstate);

#ifndef VGT_IRQ_FORWARD_MODE
	vgt_clear_irq_pending(vstate);
#endif

	return 0;
}

/* ========================Emulations========================= */

static void vgt_run_emul(struct vgt_device *vstate,
		enum vgt_event_type event, int bit, bool enable)
{
	struct vgt_irq_info *info;
	struct vgt_irq_info_entry *entry;
	struct pgt_device *dev = vstate->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	printk("vGT-IRQ(%d): %s emul handler for event %s\n", vstate->vgt_id,
		enable ? "enable" : "disable", vgt_irq_name[event]);
	info = ops->get_irq_info_from_event(dev, event);
	ASSERT(info);

	entry = info->table + bit;
	if (entry->emul_handler) {
		if (!enable)
			clear_bit(event, vgt_state_emulated_events(vstate));

		entry->emul_handler(vstate, event, enable);

		if (enable)
			set_bit(event, vgt_state_emulated_events(vstate));
	} else
		VGT_IRQ_WARN(info, event, "No emulation handler\n");
}

#if 0
static void vgt_toggle_reg_event(struct vgt_device *vstate,
	enum vgt_event_type event, int bit, bool enable)
{
	struct pgt_device *dev = vstate->pdev;

	if (enable &&
		!test_and_set_bit(event, vgt_state_enabled_events(vstate))) {
		if (vgt_get_event_owner(dev, event) == vstate)
			vgt_enable_hw_event(dev, event, bit);
		else
			vgt_run_emul(vstate, event, bit, true);
	}

	if (!enable &&
		test_and_clear_bit(event, vgt_state_enabled_events(vstate))) {
		if (vgt_get_event_owner(dev, event) == vstate)
			vgt_disable_hw_event(dev, event, bit);
		else
			vgt_run_emul(vstate, event, bit, false);
	}
}
#endif

/* given a reg bits, toggle emulated handlers */
static void vgt_toggle_emulated_bits(struct vgt_device *vgt,
	unsigned int reg, uint32_t bits, bool enable)
{
	int bit;
	enum vgt_event_type event;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	if (((reg == _REG_DEIER) || (reg == _REG_DEIMR)))
		bits &= ~(_REGBIT_PCH | _REGBIT_MASTER_INTERRUPT);

	for_each_set_bit(bit, (void *)&bits, VGT_IRQ_BITWIDTH) {
		event = ops->get_event_type_from_bit(pdev, reg, bit);
		if (event == IRQ_RESERVED)
			continue;

		vgt_run_emul(vgt, event, bit, enable);
	}
}

/* given a reg bits, only keep bits relevant to the owners */
static uint32_t vgt_keep_owner_bits(struct vgt_device *vgt,
	unsigned int reg, uint32_t bits)
{
	int bit;
	enum vgt_event_type event;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	uint32_t val;

	val = 0;
	/* PCH and maste interrupt are shared */
	if (((reg == _REG_DEIER) || (reg == _REG_DEIMR))) {
		if (bits & _REGBIT_PCH)
			val |= _REGBIT_PCH;
		if (bits & _REGBIT_MASTER_INTERRUPT)
			val |= _REGBIT_MASTER_INTERRUPT;

		bits &= ~(_REGBIT_PCH | _REGBIT_MASTER_INTERRUPT);
	}

	for_each_set_bit(bit, (void *)&bits, VGT_IRQ_BITWIDTH) {
		event = ops->get_event_type_from_bit(pdev, reg, bit);

		/* TODO: reserved events should be always masked/disabled */
		if ((event == IRQ_RESERVED) ||
		    vgt_get_event_owner(pdev, event) == vgt)
			val |= 1U << bit;
	}

	return val;
}

/* write handler for imr */
bool vgt_reg_imr_handler(struct vgt_device *state,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, masked, unmasked;
	uint32_t enabled, disabled, ier, iir, isr, val;
	unsigned long imr = *(unsigned long *)p_data;
	struct pgt_device *pdev = state->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	dprintk("vGT-IRQ: capture IMR write on reg (%s) with val (%lx)\n",
		ops->get_reg_name(pdev, reg), imr);

	dprintk("vGT-IRQ: old vIER(%x), vIMR(%x)\n",
		__vreg(state, reg + 8), __vreg(state, reg));
	dprintk("vGT-IRQ: old IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg + 8), VGT_MMIO_READ(pdev, reg));

	/* figure out newly masked/unmasked bits */
	changed = __vreg(state, reg) ^ imr;
	changed &= ~_REGBIT_MASTER_INTERRUPT;
	masked = (__vreg(state, reg) & changed) ^ changed;
	unmasked = masked ^ changed;

	/* figure out newly enabled/disabled bits */
	ier = vgt_imr_to_ier(state, reg);
	enabled = ier & unmasked;
	disabled = ier & masked;

	dprintk("vGT-IRQ: changed (%x), masked(%x), unmasked (%x), enabled(%x), disabled (%x)\n",
		changed, masked, unmasked, enabled, disabled);
	__vreg(state, reg) = imr;

	/* handle pending virtual events */
	if (enabled) {
		isr = vgt_imr_to_isr(state, reg);
		if (isr & ~imr) {
			vgt_imr_to_iir(state, reg) |= isr & ~imr;
			iir = vgt_imr_to_iir(state, reg);
			vgt_imr_to_isr(state, reg) &= ~(isr & ~imr);
			if (iir & ier) {
				printk("vGT-IRQ: catch pending iir (%x)\n", iir);
				vgt_set_irq_pending(state);
				vgt_inject_virtual_interrupt(state);
			}
		}
	}

	/* merge pch bits */
	if (reg == _REG_DEIMR) {
		if (masked & _REGBIT_PCH) {
			printk("vGT-IRQ(%d): newly mask PCH\n", state->vgt_id);
			clear_bit(state->vgt_id, (void *)&vgt_pch_unmask(pdev));
			/* only mask when all VMs mask */
			if (vgt_pch_unmask(pdev))
				masked &= ~_REGBIT_PCH;
		}

		if (unmasked & _REGBIT_PCH) {
			printk("vGT-IRQ(%d): newly unmask PCH\n", state->vgt_id);
			set_bit(state->vgt_id, (void *)&vgt_pch_unmask(pdev));
		}

	}

	/*
	 * handle changed bits if it's the owner
	 *
	 * FIXME: we happen to observe that IER/IMR changes may not take effect
	 * when master bit in DEIER is enabled on SNB. This may be a hardware
	 * issue to be verified in the future. Now to make it forward progress,
	 * we manually disable master bit when IER/IMR bits are poked by the
	 * owner, and re-enable master bit at the end of the emulation.
	 *
	 * Even this workaround is required, no need to always touch master bit
	 * if the current VM only touches emulated events
	 */
	masked = vgt_keep_owner_bits(state, reg, masked);
	unmasked = vgt_keep_owner_bits(state, reg, unmasked);
	if (masked || unmasked) {
		val = VGT_MMIO_READ(pdev, reg);
		if (masked)
			val |= masked;
		if (unmasked)
			val &= ~unmasked;

		VGT_MMIO_WRITE(pdev, reg, val);
		VGT_POST_READ(pdev, reg);
	}

	/* Then handle emulated events */
	enabled &= ~unmasked;
	disabled &= ~masked;
	if (enabled)
		vgt_toggle_emulated_bits(state, reg, enabled, true);
	if (disabled)
		vgt_toggle_emulated_bits(state, reg, disabled, false);

	dprintk("vGT-IRQ: new vIER(%x), vIMR(%x)\n",
		__vreg(state, reg + 8), __vreg(state, reg));
	dprintk("vGT-IRQ: new IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg + 8), VGT_MMIO_READ(pdev, reg));
	/* TODO: throw out an early warning if no handlers for enabled events */
	return true;
}

/*
 * we only handle the cases which cause an physically enabled event
 * to be disabled, or vice versa. This means under the scope of
 * currently unmasked IMR bits.
 */
bool vgt_reg_ier_handler(struct vgt_device *state,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, imr, iir;
	uint32_t enabled, disabled, ier_enabled, ier_disabled;
	unsigned long ier = *(unsigned long *)p_data;
	struct pgt_device *pdev = state->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	vgt_reg_t val;

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	dprintk("vGT-IRQ: capture IER write on reg (%s) with val (%lx)\n",
		ops->get_reg_name(pdev, reg), ier);

	dprintk("vGT-IRQ: old vIER(%x), vIMR(%x)\n",
		__vreg(state, reg), __vreg(state, reg - 8));
	dprintk("vGT-IRQ: old IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg), VGT_MMIO_READ(pdev, reg - 8));

	/* figure out newly enabled/disable bits */
	changed = __vreg(state, reg) ^ ier;
	ier_enabled = (__vreg(state, reg) & changed) ^ changed;
	ier_disabled = ier_enabled ^ changed;

	/* figure out really enabled/disabled bits */
	imr = vgt_ier_to_imr(state, reg);
	enabled = ier_enabled & ~imr;
	disabled = ier_disabled & ~imr;

	dprintk("vGT_IRQ: changed (%x), i-enabled(%x), i-disabled (%x), enabled(%x), disabled(%x)\n",
		changed, ier_enabled, ier_disabled, enabled, disabled);
	__vreg(state, reg) = ier;

	/* handle pending virtual events */
	if (enabled) {
		iir = vgt_ier_to_iir(state, reg);
		if (iir & ier) {
			printk("vGT-IRQ: catch pending iir (%x)\n", iir);
			vgt_set_irq_pending(state);
			vgt_inject_virtual_interrupt(state);
		}
	}

	/* merge pch bits */
	if (reg == _REG_DEIER) {
		if (ier_enabled & _REGBIT_PCH) {
			printk("vGT-IRQ(%d): newly enable PCH\n", state->vgt_id);
			set_bit(state->vgt_id, (void *)&vgt_pch_enable(pdev));
		}

		if (ier_disabled & _REGBIT_PCH) {
			printk("vGT-IRQ(%d): newly disable PCH\n", state->vgt_id);
			clear_bit(state->vgt_id, (void *)&vgt_pch_enable(pdev));
			if (vgt_pch_enable(pdev))
				ier_disabled &= ~_REGBIT_PCH;
		}
	}

	/*
	 * handle changed bits if it's the owner
	 *
	 * Same trick as in imr handler
	 */
	ier_enabled = vgt_keep_owner_bits(state, reg, ier_enabled);
	ier_disabled = vgt_keep_owner_bits(state, reg, ier_disabled);
	if (ier_enabled || ier_disabled) {
		if (reg == _REG_DEIER) {
			if (ier_enabled & _REGBIT_MASTER_INTERRUPT) {
				dprintk("vGT-IRQ(%d): newly enable MASTER\n", state->vgt_id);
				set_bit(state->vgt_id, (void *)&vgt_master_enable(pdev));
				/* leave to vgt_enable_master_interrupt */
				ier_enabled &= ~_REGBIT_MASTER_INTERRUPT;
			}

			if (ier_disabled & _REGBIT_MASTER_INTERRUPT) {
				dprintk("vGT-IRQ(%d): newly disable MASTER\n", state->vgt_id);
				clear_bit(state->vgt_id, (void *)&vgt_master_enable(pdev));
				ier_disabled &= ~_REGBIT_MASTER_INTERRUPT;
			}

		}

		val = VGT_MMIO_READ(pdev, reg);
		if (ier_enabled)
			val |= ier_enabled;
		if (ier_disabled)
			val &= ~ier_disabled;
		if (vgt_master_enable(pdev))
			val |= _REGBIT_MASTER_INTERRUPT;
		else
			val &= ~_REGBIT_MASTER_INTERRUPT;
		VGT_MMIO_WRITE(pdev, reg, val);
		VGT_POST_READ(pdev, reg);
	}

	/* Then handle emulated events */
	enabled &= ~ier_enabled;
	disabled &= ~ier_disabled;
	if (enabled)
		vgt_toggle_emulated_bits(state, reg, enabled, true);
	if (disabled)
		vgt_toggle_emulated_bits(state, reg, disabled, false);

	dprintk("vGT-IRQ: new vIER(%x), vIMR(%x)\n",
		__vreg(state, reg), __vreg(state, reg - 8));
	dprintk("vGT-IRQ: new IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg), VGT_MMIO_READ(pdev, reg - 8));
	/* TODO: throw out an early warning if no handlers for enabled events */
	return true;
}

/*
 * write handler for IIR.
 *
 * TODO: IIR can cache two pending events
 */
bool vgt_reg_irr_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t iir = *(vgt_reg_t *)p_data;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(vgt->pdev);

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	dprintk("vGT-IRQ: capture IIR write on reg (%s) with val (%lx)\n",
		ops->get_reg_name(vgt->pdev, reg), val);


	dprintk("vGT: update %s: %x -> %x\n", ops->get_reg_name(vgt->pdev, reg),
		__vreg(vgt, reg), __vreg(vgt, reg) & (~iir));
	/* write to clear IIR */
	__vreg(vgt, reg) &= ~iir;

	return true;
}

void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...)
{
	printk("!!!vGT: capture watchdog operations (%s on reg %x). Not emulated yet!\n",
			write ? "write" : "read", reg);
}

static enum hrtimer_restart vgt_dpy_timer_fn(struct hrtimer *data)
{
	struct vgt_emul_timer *dpy_timer = container_of(data, struct vgt_emul_timer, timer);
	struct vgt_irq_virt_state *virq = container_of(dpy_timer, struct vgt_irq_virt_state, dpy_timer);
	struct vgt_device *vstate = virq->vgt;

	/* carry all display status events in one timer */
	if (test_bit(IRQ_PIPE_A_VSYNC, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_A_VSYNC);
	if (test_bit(IRQ_PIPE_A_LINE_COMPARE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_A_LINE_COMPARE);
	if (test_bit(IRQ_PIPE_A_ODD_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_A_ODD_FIELD);
	if (test_bit(IRQ_PIPE_A_EVEN_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_A_EVEN_FIELD);
	if (test_bit(IRQ_PIPE_A_VBLANK, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_A_VBLANK);
	if (test_bit(IRQ_PIPE_B_VSYNC, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_B_VSYNC);
	if (test_bit(IRQ_PIPE_B_LINE_COMPARE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_B_LINE_COMPARE);
	if (test_bit(IRQ_PIPE_B_ODD_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_B_ODD_FIELD);
	if (test_bit(IRQ_PIPE_B_EVEN_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_B_EVEN_FIELD);
	if (test_bit(IRQ_PIPE_B_VBLANK, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PIPE_B_VBLANK);
	if (test_bit(IRQ_PRIMARY_A_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PRIMARY_A_FLIP_DONE);
	if (test_bit(IRQ_PRIMARY_B_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_PRIMARY_B_FLIP_DONE);
	if (test_bit(IRQ_SPRITE_A_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_SPRITE_A_FLIP_DONE);
	if (test_bit(IRQ_SPRITE_B_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propogate_emulated_event(vstate, IRQ_SPRITE_B_FLIP_DONE);

	if (vgt_has_irq_pending(vstate))
		vgt_inject_virtual_interrupt(vstate);
	hrtimer_add_expires_ns(&dpy_timer->timer, dpy_timer->period);
	return HRTIMER_RESTART;
}

/*
 * TODO: currently we use a single timer to emulate all timer-based events.
 * Need further study case-by-case in the future
 */
void vgt_emulate_dpy_status(struct vgt_device *vstate, enum vgt_event_type event, bool enable)
{
	if (!enable)
		clear_bit(event, vgt_dpy_timer(vstate).events);

	if (bitmap_empty(vgt_dpy_timer(vstate).events, IRQ_MAX)) {
		/* TODO: use range timer */
		if (enable)
			/* FIXME : check interface */
			hrtimer_start(&vgt_dpy_timer(vstate).timer,
				      ktime_add_ns(ktime_get(), vgt_dpy_timer(vstate).period),
				      HRTIMER_MODE_ABS);
		else
			hrtimer_cancel(&vgt_dpy_timer(vstate).timer);
	}

	if (enable)
		set_bit(event, vgt_dpy_timer(vstate).events);
}

void vgt_emulate_watchdog(struct vgt_device *vstate, enum vgt_event_type event, bool enable)
{
	dprintk("vGT: watch dog emulation is not supported yet\n");
}

/* =======================pEvent Handlers===================== */

/*
 * the default handler for most events, w/o extra physical housekeeping
 * required except IIR clearing
 */
void vgt_default_event_handler(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	/* no action for physical handler */
	if (physical)
		return;

	/* propagate to the owner */
	vgt_propogate_virtual_event(vgt, bit, info);
}

/* in the case that PCH events are queued in another set of control registers */
void vgt_handle_chained_pch_events(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	//uint32_t sde_iir;

	/* FIXME: need consider how delayed event is done for PCH */
	VGT_IRQ_WARN_ONCE(info, entry->event, "PCH!!!\n");

#if 0
	if (action != VGT_IRQ_HANDLE_FULL)
		VGT_IRQ_WARN_ONCE(info, entry->event, "!!!!!!\n");

#if 0
	/* loop pending events with PCH */
	sde_iir = VGT_MMIO_READ(dev, _REG_SDEIIR);
	vgt_irq_handle_event(dev, &sde_iir, &snb_pch_irq_info);
	VGT_MMIO_WRITE(dev, _REG_SDEIIR, sde_iir);
#endif

	vstate = vgt_get_event_owner(dev, entry->event);
	/* propogate to level-1 control registers */
	if (vgt_has_pch_irq_pending(vstate)) {
		vgt_propogate_virtual_event(vgt_get_event_owner(dev, entry->event), bit, info);
		vgt_clear_pch_irq_pending(vstate);
	}
#endif
}

/* not assumed to be invoked! */
void vgt_handle_unexpected_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "not assumed to happen!!!\n");
	vgt_default_event_handler(dev, bit, entry, info, physical, vgt);
}

/* events we don't expect programmed by VM, such as watchdog timer */
void vgt_handle_host_only_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "host-only event!!!\n");
}

/*
 * unlike NULL handler which we know won't handle for now, weak handlers
 * are for those which may be triggered but the detail is not very clear
 * at this stage.
 */
void vgt_handle_weak_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "more consideration required on this handler~~\n");
	vgt_default_event_handler(dev, bit, entry, info, physical, vgt);
}

/*
 * Erros is now injected into the current owner.
 *
 * need to consider whether recovery of vGT driver is required
 *
 * there's further error information in some debug registers. no forward now
 */
void vgt_handle_cmd_stream_error(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	uint32_t reg, val;

	/* always warn for errors */
	VGT_IRQ_WARN(info, entry->event, "ERROR ERROR!!!\n");

	/* check error status */
	switch (entry->event) {
		case IRQ_RCS_CMD_STREAMER_ERR:
			reg = _REG_RCS_EIR;
		case IRQ_BCS_CMD_STREAMER_ERR:
			reg = _REG_BCS_EIR;
		default:
			printk("no reg info to propogate\n");
			reg = _REG_INVALID;
	};
	ASSERT(reg != _REG_INVALID);

	val = VGT_MMIO_READ(dev, reg);
	VGT_MMIO_WRITE(dev, reg, val);

	/* FIXME */
	*vgt_vreg(vgt, reg) = val;

	/*
	 * FIXME:
	 * there're several conditions contributing to the error condition
	 * such as command error, page table error, etc. However next level
	 * error information for the condition is only for debug-only
	 * purpose. I didn't find consistent information cross manual, and
	 * thus leave them unhandled for now.
	 */

	vgt_propogate_virtual_event(vgt, bit, info);
}

void vgt_handle_phase_in(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	uint32_t val;

	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured Phase-In event!!!\n");

	val = VGT_MMIO_READ(dev, _REG_BLC_PWM_CTL2);
	val &= ~_REGBIT_PHASE_IN_IRQ_STATUS;
	VGT_MMIO_WRITE(dev, _REG_BLC_PWM_CTL2, val);

	/* FIXME */
	*vgt_vreg(vgt, _REG_BLC_PWM_CTL2) |= _REGBIT_PHASE_IN_IRQ_STATUS;

	vgt_propogate_virtual_event(vgt, bit, info);
}

void vgt_handle_histogram(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	uint32_t val;

	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured Histogram event!!!\n")

	val = VGT_MMIO_READ(dev, _REG_HISTOGRAM_THRSH);
	val &= ~_REGBIT_HISTOGRAM_IRQ_STATUS;
	VGT_MMIO_WRITE(dev, _REG_HISTOGRAM_THRSH, val);

	*vgt_vreg(vgt, _REG_HISTOGRAM_THRSH) |= _REGBIT_HISTOGRAM_IRQ_STATUS;

	vgt_propogate_virtual_event(vgt, bit, info);
}

void vgt_handle_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured hotplug event (no handler)!!!\n")
}

void vgt_handle_aux_channel(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured aux channel event (no handler)!!!\n")
}

void vgt_handle_gmbus(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured gmbus event (no handler)!!!\n")
}

/* core event handling loop for a given IIR */
void vgt_irq_handle_event(struct pgt_device *dev, void *iir,
	struct vgt_irq_info *info, bool physical,
	enum vgt_owner_type o_type)
{
	int bit;
	struct vgt_irq_info_entry *entry;

	for_each_set_bit(bit, iir, info->table_size) {
		/* clear cached pending bits */
		if (!physical)
			clear_bit(bit, iir);

		entry = info->table + bit;
		//vgt_trace_irq_event(info, entry->event);

		if (entry->event == IRQ_RESERVED ||
		    entry->event >= IRQ_MAX) {
			VGT_IRQ_WARN(info, entry->event, "UNKNOWN!!!\n");
			return;
		}

#ifndef VGT_IRQ_DEFAULT_HANDLER
		if (unlikely(!entry->event_handler)) {
			VGT_IRQ_WARN(info, entry->event, "No handler!!!\n");
			return;
		}

		/*
		 * the event comes when the owner is in switch.
		 * need to inject into both the prev and curr owner
		 */
		if (vgt_get_event_owner_type(dev, entry->event) == o_type) {
			printk("vGT: inject event (%s) to previous owner (%d)\n",
				vgt_irq_name(entry->event),
				vgt_get_previous_owner(dev, o_type));
			entry->event_handler(dev, bit, entry, info, physical,
				vgt_get_previous_owner(dev, o_type));
		}

		/* inject to the current owner */
		entry->event_handler(dev, bit, entry, info, physical,
			vgt_get_event_owner(dev, entry->event));
#else
		if (entry->event == IRQ_PCH_IRQ)
			printk("!!!vGT-IRQ: PCH interrupt is caught\n");
		vgt_default_event_handler(dev, bit, entry, info, physical,
			vgt_get_event_owner(dev, entry->event));
#endif
	}
}

/*
 * VGT_OT_INVALID indicates normal injection, and other valid types indicate injections
 * to both prev/next owners
 */
void vgt_handle_virtual_interrupt(struct pgt_device *pdev, enum vgt_owner_type type)
{
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	int i;

	ops->handle_virtual_interrupt(pdev, type);

	/* check pending virtual interrupt for active VMs. may instead put on a delayed work? */
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i] && vgt_has_irq_pending(pdev->device[i]))
			vgt_inject_virtual_interrupt(pdev->device[i]);
	}
}

/*
 * Physical interrupt handler for Intel HD serious graphics
 *   - handle various interrupt reasons
 *   - may trigger virtual interrupt instances to dom0 or other VMs
 */
static irqreturn_t vgt_interrupt(int irq, void *data)
{
	struct pgt_device *dev = (struct pgt_device *)data;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);
	irqreturn_t ret;
	u32 de_ier;

	dprintk("vGT: receive interrupt (de-%x, gt-%x, pch-%x, pm-%x)\n",
		VGT_MMIO_READ(dev, _REG_DEIIR),
		VGT_MMIO_READ(dev, _REG_GTIIR),
		VGT_MMIO_READ(dev, _REG_SDEIIR),
		VGT_MMIO_READ(dev, _REG_PMIIR));

#ifndef VGT_IRQ_FORWARD_MODE
	/* avoid nested handling by disabling master interrupt */
	de_ier = VGT_MMIO_READ(dev, _REG_DEIER);
	VGT_MMIO_WRITE(dev, _REG_DEIER, de_ier & ~_REGBIT_MASTER_INTERRUPT);

	ret = ops->interrupt(dev);
	if (ret == IRQ_NONE) {
		printk("Spurious interrupt received (or shared vector)\n");
		VGT_MMIO_WRITE(dev, _REG_DEIER, de_ier);
		return IRQ_HANDLED;
	}

	vgt_raise_request(dev, VGT_REQUEST_IRQ);

	/* re-enable master interrupt */
	VGT_MMIO_WRITE(dev, _REG_DEIER, de_ier);
#else
	if (dev->device[0])
		vgt_inject_virtual_interrupt(dev->device[0]);
	else
		printk("vGT: no owner for this interrupt \n");
#endif

	return IRQ_HANDLED;
}

/* =====================Initializations======================= */

static void vgt_initialize_always_emulated_events(struct pgt_device *dev)
{
	/* timers are always emulated */
	set_bit(IRQ_RCS_WATCHDOG_EXCEEDED, vgt_always_emulated_events(dev));
	set_bit(IRQ_VCS_WATCHDOG_EXCEEDED, vgt_always_emulated_events(dev));
	vgt_get_event_owner_type(dev, IRQ_RCS_WATCHDOG_EXCEEDED) = VGT_OT_INVALID;
	vgt_get_event_owner_type(dev, IRQ_VCS_WATCHDOG_EXCEEDED) = VGT_OT_INVALID;
}

/*
 * Do interrupt initialization for vGT driver
 */
int vgt_irq_init(struct pgt_device *dev)
{
	enum vgt_owner_type *o_table;
	struct vgt_irq_ops *ops;
	struct vgt_irq_host_state *irq_hstate;

	irq_hstate = kzalloc(sizeof(struct vgt_irq_host_state), GFP_KERNEL);
	ASSERT(irq_hstate);

	dev->irq_hstate = irq_hstate;

	spin_lock_init(&(dev->irq_hstate->lock));

	if (snb_device(dev))
		dev->irq_hstate->ops = &snb_irq_ops;
	else {
		dprintk("vGT: no irq ops found!\n");
		return -EINVAL;
	}

	/* Initialize ownership table, based on a default policy table */
	o_table = kmalloc(IRQ_MAX * sizeof(enum vgt_owner_type), GFP_KERNEL);
	if (!o_table) {
		dprintk("vGT: no enough memory for owner table\n");
		return -ENOMEM;
	}
	memcpy((void *)o_table, (void *)vgt_default_event_owner_table,
		IRQ_MAX * sizeof(enum vgt_owner_type));
	vgt_event_owner_table(dev) = o_table;

	ops = vgt_get_irq_ops(dev);
#ifndef VGT_IRQ_FORWARD_MODE
	ops->init(dev);
#endif

	vgt_initialize_always_emulated_events(dev);

	vgt_master_enable(dev) = 0;
	vgt_pch_enable(dev) = 0;
	vgt_pch_unmask(dev) = 0;
	printk("vGT: interrupt initialization completes\n");
	return 0;
}

void vgt_irq_exit(struct pgt_device *dev)
{
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	free_irq(vgt_pirq(dev), dev);
	/* TODO: recover i915 handler? */
	//unbind_from_irq(vgt_i915_irq(dev));
	ops->exit(dev);
	kfree(vgt_event_owner_table(dev));
}

int vgt_vstate_irq_init(struct vgt_device *vstate)
{
	struct vgt_emul_timer *dpy_timer;
	struct vgt_irq_virt_state *irq_vstate;

	irq_vstate = kzalloc(sizeof(struct vgt_irq_virt_state), GFP_KERNEL);
	ASSERT(irq_vstate);

	dpy_timer = &irq_vstate->dpy_timer;

	hrtimer_init(&dpy_timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dpy_timer->timer.function = vgt_dpy_timer_fn;
	dpy_timer->period = VGT_DPY_EMUL_PERIOD;

	irq_vstate->vgt = vstate;
	vstate->irq_vstate = irq_vstate;
	/* Assume basic domain information has been retrieved already */
	printk("vGT: interrupt initialization for domain (%d) completes\n", vstate->vm_id);
	return 0;
}

void vgt_vstate_irq_exit(struct vgt_device *vstate)
{
	// leave it empty for now
}

void vgt_install_irq(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;
	int irq, ret;

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

	vgt_pirq(pgt) = pdev->irq;
	vgt_i915_irq(pgt) = irq;
	pdev->irq = irq;

	printk("vGT: allocate virq (%d) for i915, while keep original irq (%d) for vgt\n",
		vgt_i915_irq(pgt), vgt_pirq(pgt));
}
EXPORT_SYMBOL(vgt_install_irq);
