/*
 * vGT interrupt info table for SNB
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
#include <linux/bitops.h>
#include <xen/vgt.h>
#include "vgt_reg.h"

/*
 * GT related irq info
 *
 * by default no emulation is required for GT events, since all those
 * events are related to instruction execution. But watchdog timer is
 * special, which we want to let vGT fully control it and then emulate
 * for each instance.
 */
static struct vgt_irq_info snb_render_irq_info = {
	.name = "SNB GT IRQ",
	.reg_base = _REG_GTISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.table = {
		{IRQ_RCS_MI_USER_INTERRUPT,	vgt_default_event_handler, 	NULL},	// bit 0
		{IRQ_RCS_DEBUG,			vgt_default_event_handler, 	NULL},	// bit 1
		/* not expect to generate interrupt for sync flush */
		{IRQ_RCS_MMIO_SYNC_FLUSH,	vgt_handle_unexpected_event,	NULL},	// bit 2
		{IRQ_RCS_CMD_STREAMER_ERR,	vgt_handle_cmd_stream_error,	NULL},	// bit 3
		/* PIPE_CONTROL may need some special handling for ID */
		{IRQ_RCS_PIPE_CONTROL, 		vgt_handle_weak_event,		NULL},	// bit 4

		{IRQ_RESERVED, 			NULL, 				NULL},

		{IRQ_RCS_WATCHDOG_EXCEEDED,	vgt_handle_host_only_event,	vgt_emulate_watchdog},	// bit 6
		/* unclear with page fault context yet */
		{IRQ_RCS_PAGE_DIRECTORY_FAULT,	vgt_handle_weak_event, 		NULL},	// bit 7
		/* not expect to use context switch interrupt on snb, though listed in PRM */
		{IRQ_RCS_AS_CONTEXT_SWITCH,	vgt_handle_unexpected_event, 	NULL},	// bit 8

		{IRQ_RESERVED, 			NULL,				NULL},
		{IRQ_RESERVED, 			NULL, 				NULL},
		{IRQ_RESERVED, 			NULL, 				NULL},

		{IRQ_VCS_MI_USER_INTERRUPT, 	vgt_default_event_handler,	NULL},	// bit 12

		{IRQ_RESERVED, 			NULL,				NULL},

		{IRQ_VCS_MMIO_SYNC_FLUSH, 	vgt_handle_unexpected_event,	NULL},	// bit 14
		{IRQ_VCS_CMD_STREAMER_ERR, 	vgt_handle_cmd_stream_error,	NULL},	// bit 15
		{IRQ_VCS_MI_FLUSH_DW, 	vgt_handle_weak_event,		NULL},	// bit 16

		{IRQ_RESERVED, 			NULL,				NULL},

		{IRQ_VCS_WATCHDOG_EXCEEDED, 	vgt_handle_host_only_event,	vgt_emulate_watchdog},	// bit 18
		{IRQ_VCS_PAGE_DIRECTORY_FAULT, vgt_handle_weak_event,		NULL},	// bit 19
		{IRQ_VCS_AS_CONTEXT_SWITCH, 	vgt_handle_unexpected_event,	NULL},	// bit 20

		{IRQ_RESERVED, 			NULL,				NULL},

		{IRQ_BCS_MI_USER_INTERRUPT, 	vgt_default_event_handler,	NULL},	// bit 22

		{IRQ_RESERVED,			NULL,				NULL},

		{IRQ_BCS_MMIO_SYNC_FLUSH, 	vgt_handle_unexpected_event,	NULL}, 	// bit 24
		{IRQ_BCS_CMD_STREAMER_ERR, 	vgt_handle_cmd_stream_error,	NULL}, 	// bit 25
		{IRQ_BCS_MI_FLUSH_DW, 		vgt_handle_weak_event,		NULL},	// bit 26

		{IRQ_RESERVED, 			NULL,				NULL},
		{IRQ_RESERVED, 			NULL,				NULL},

		{IRQ_BCS_PAGE_DIRECTORY_FAULT, vgt_handle_weak_event,		NULL},	// bit 29
		{IRQ_BCS_AS_CONTEXT_SWITCH, 	vgt_handle_unexpected_event,	NULL},	// bit 30

		{IRQ_RESERVED, 			NULL,				NULL},	// bit 31
	},
};

/*
 * display related irq info
 */
static struct vgt_irq_info snb_dpy_irq_info = {
	.name = "SNB DISPLAY IRQ",
	.reg_base = _REG_DEISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.table = {
		{IRQ_PIPE_A_FIFO_UNDERRUN,	vgt_default_event_handler,	NULL},	// bit 0
		{IRQ_PIPE_A_CRC_ERR, 		vgt_default_event_handler,	NULL},	// bit 1
		{IRQ_PIPE_A_CRC_DONE, 		vgt_default_event_handler,	NULL},	// bit 2
		{IRQ_PIPE_A_VSYNC, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 3
		{IRQ_PIPE_A_LINE_COMPARE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 4
		{IRQ_PIPE_A_ODD_FIELD, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 5
		{IRQ_PIPE_A_EVEN_FIELD, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 6
		{IRQ_PIPE_A_VBLANK, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 7
		{IRQ_PIPE_B_FIFO_UNDERRUN, 	vgt_default_event_handler,	NULL},	// bit 8
		{IRQ_PIPE_B_CRC_ERR, 		vgt_default_event_handler,	NULL},	// bit 9
		{IRQ_PIPE_B_CRC_DONE, 		vgt_default_event_handler,	NULL},	// bit 10
		{IRQ_PIPE_B_VSYNC, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 11
		{IRQ_PIPE_B_LINE_COMPARE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 12
		{IRQ_PIPE_B_ODD_FIELD, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 13
		{IRQ_PIPE_B_EVEN_FIELD, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 14
		{IRQ_PIPE_B_VBLANK, 		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 15
		{IRQ_DPST_PHASE_IN, 		vgt_handle_phase_in,	NULL},		// bit 16
		{IRQ_DPST_HISTOGRAM, 		vgt_handle_histogram,	NULL},		// bit 17
		{IRQ_GSE, 			NULL,	NULL},				// bit 18
		{IRQ_DP_A_HOTPLUG, 		vgt_handle_hotplug,	NULL},		// bit 19
		{IRQ_AUX_CHANNEL_A, 		vgt_handle_aux_channel,	NULL},	// bit 20
		{IRQ_PCH_IRQ,			vgt_handle_chained_pch_events,	NULL},	// bit 21
		{IRQ_PERF_COUNTER, 		NULL,	NULL},		// bit 22
		{IRQ_POISON, 			NULL,	NULL},		// bit 23
		{IRQ_GTT_FAULT, 		NULL,	NULL},		// bit 24

		{IRQ_RESERVED, NULL,	NULL},

		{IRQ_PRIMARY_A_FLIP_DONE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 26
		{IRQ_PRIMARY_B_FLIP_DONE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 27
		{IRQ_SPRITE_A_FLIP_DONE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 28
		{IRQ_SPRITE_B_FLIP_DONE, 	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 29

		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},			// bit 31
	},
};

/*
 * leave PM handlers all NULL for now
 *
 * there's no need for emulation for PM events, assuming that dom 0 is the
 * exclusive owner of this category
 */
static struct vgt_irq_info snb_pm_irq_info = {
	.name = "SNB PM IRQ",
	.reg_base = _REG_PMISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.table = {
		{IRQ_RESERVED,			NULL,	NULL},			// bit 0

		{IRQ_GV_DOWN_INTERVAL, 		NULL,	NULL},		// bit 1
		{IRQ_GV_UP_INTERVAL, 		NULL,	NULL},		// bit 2

		{IRQ_RESERVED, 			NULL,	NULL},

		{IRQ_RP_DOWN_THRESHOLD, 	NULL,	NULL},		// bit 4
		{IRQ_RP_UP_THRESHOLD, 		NULL,	NULL},		// bit 5
		{IRQ_FREQ_DOWNWARD_TIMEOUT_RC6, NULL,	NULL},	// bit 6

		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},

		{IRQ_PCU_THERMAL, 		NULL,	NULL},		// bit 24
		{IRQ_PCU_PCODE2DRIVER_MAILBOX,	NULL,	NULL},	// bit 25

		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},			// bit 31
	},
};

/*
 * PCH  related irq info
 */
static struct vgt_irq_info snb_pch_irq_info = {
	.name = "SNB PCH IRQ",
	.reg_base = _REG_SDEISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.table = {
		{IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A,	NULL,	NULL},	// bit 0
		{IRQ_AUDIO_CP_CHANGE_TRANSCODER_A, 	NULL,	NULL},	// bit 1
		{IRQ_AUDIO_CP_REQUEST_TRANSCODER_A, 	NULL,	NULL},	// bit 2

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_FDI_RX_INTERRUPTS_TRANSCODER_B, 	NULL,	NULL},	// bit 4
		{IRQ_AUDIO_CP_CHANGE_TRANSCODER_B, 	NULL,	NULL},	// bit 5
		{IRQ_AUDIO_CP_REQUEST_TRANSCODER_B, 	NULL,	NULL},	// bit 6

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_FDI_RX_INTERRUPTS_TRANSCODER_C, 	NULL,	NULL},	// bit 8
		{IRQ_AUDIO_CP_CHANGE_TRANSCODER_C, 	NULL,	NULL},	// bit 9
		{IRQ_AUDIO_CP_REQUEST_TRANSCODER_C, 	NULL,	NULL},	// bit 10

		{IRQ_RESERVED, 				NULL,	NULL},
		{IRQ_RESERVED, 				NULL,	NULL},
		{IRQ_RESERVED, 				NULL,	NULL},
		{IRQ_RESERVED, 				NULL,	NULL},
		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_ERR_AND_DBG, 			NULL,	NULL},			// bit 16
		{IRQ_GMBUS, 				vgt_handle_gmbus,	NULL},		// bit 17
		{IRQ_SDVO_B_HOTPLUG, 			vgt_handle_hotplug,	NULL},		// bit 18
		{IRQ_CRT_HOTPLUG, 			vgt_handle_hotplug,	NULL},		// bit 19

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_DP_B_HOTPLUG, 			vgt_handle_hotplug,	NULL},		// bit 21
		{IRQ_DP_C_HOTPLUG, 			vgt_handle_hotplug,	NULL},		// bit 22
		{IRQ_DP_D_HOTPLUG, 			vgt_handle_hotplug,	NULL},		// bit 23

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_AUX_CHENNEL_B, 			vgt_handle_aux_channel,	NULL},	// bit 25
		{IRQ_AUX_CHENNEL_C, 			vgt_handle_aux_channel,	NULL},	// bit 26
		{IRQ_AUX_CHENNEL_D, 			vgt_handle_aux_channel,	NULL},	// bit 27

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_AUDIO_POWER_STATE_CHANGE_B, 	NULL,	NULL},		// bit 29
		{IRQ_AUDIO_POWER_STATE_CHANGE_C, 	NULL,	NULL},		// bit 30
		{IRQ_AUDIO_POWER_STATE_CHANGE_D, 	NULL,	NULL},		// bit 31
	},
};

static struct vgt_irq_info* vgt_snb_get_irq_info_from_event(struct pgt_device *dev, enum vgt_event_type event)
{
	if (VGT_RENDER_EVENT(event))
		return &snb_render_irq_info;
	else if (VGT_DPY_EVENT(event))
		return &snb_dpy_irq_info;
	else if (VGT_PCH_EVENT(event))
		return &snb_pch_irq_info;
	else if (VGT_PM_EVENT(event))
		return &snb_pm_irq_info;
	else
		return NULL;
}

static inline struct vgt_irq_info* vgt_snb_get_irq_info_from_reg(int reg)
{
	if (reg >= _REG_GTISR && reg < _REG_GTIER + 4)
		return &snb_render_irq_info;
	else if (reg >= _REG_DEISR && reg < _REG_DEIER + 4)
		return &snb_dpy_irq_info;
	else if (reg >= _REG_SDEISR && reg < _REG_SDEIER + 4)
		return &snb_pch_irq_info;
	else if (reg >= _REG_PMISR && reg < _REG_PMIER + 4)
		return &snb_pm_irq_info;
	else
		return NULL;
}

/*
 * slow path to search the info table, when there is no bit information
 * is available. performance critical path should have bit information
 * already, by carrying from pReg
 */
static int vgt_snb_get_bit_from_event(struct pgt_device *dev,
	enum vgt_event_type event, struct vgt_irq_info *info)
{
	int i;

	for (i = 0; i < info->table_size; i++) {
		if (info->table[i].event == event)
			break;
	}

	ASSERT(i < info->table_size);
	return i;
}

/* Enable a hardware event by operating IER/IMR. */
static inline void vgt_snb_toggle_hw_event(struct pgt_device *dev,
	enum vgt_event_type event, int bit, bool enable)
{
	struct vgt_irq_info *info;

	info = vgt_snb_get_irq_info_from_event(dev, event);
	ASSERT(info != NULL);

	if (bit == VGT_IRQ_BITWIDTH)
		bit = vgt_snb_get_bit_from_event(dev, event, info);

	if (enable) {
		if (VGT_PCH_EVENT(event))
			vgt_snb_toggle_hw_event(dev, IRQ_PCH_IRQ, bit, true);

		vgt_set_reg_bit(dev, vgt_ier(info->reg_base), bit);
	} else {
		vgt_clear_reg_bit(dev, vgt_ier(info->reg_base), bit);

		if (VGT_PCH_EVENT(event))
			vgt_snb_toggle_hw_event(dev, IRQ_PCH_IRQ, bit, false);
	}
}

/*
 * Physical interrupt handler for Intel HD serious graphics
 *   - handle various interrupt reasons
 *   - may trigger virtual interrupt instances to dom0 or other VMs
 * Now SNB specific.
 */
static irqreturn_t vgt_snb_interrupt(struct pgt_device *dev)
{
	uint32_t gt_iir, de_iir, pm_iir;

	/* read physical IIRs */
	gt_iir = VGT_MMIO_READ(dev, _REG_GTIIR);
	de_iir = VGT_MMIO_READ(dev, _REG_DEIIR);
	pm_iir = VGT_MMIO_READ(dev, _REG_PMIIR);

	if (!gt_iir && !de_iir && !pm_iir)
		return IRQ_NONE;

	vgt_irq_handle_event(dev, &gt_iir, &snb_render_irq_info);
	vgt_irq_handle_event(dev, &de_iir, &snb_dpy_irq_info);
	vgt_irq_handle_event(dev, &pm_iir, &snb_pm_irq_info);

	/* clear physical IIRs in the end, after lower level causes are cleared */
	VGT_MMIO_WRITE(dev, _REG_GTIIR, 0);
	VGT_MMIO_WRITE(dev, _REG_PMIIR, 0);
	VGT_MMIO_WRITE(dev, _REG_DEIIR, 0);

	return IRQ_HANDLED;
}

static void vgt_snb_irq_init(struct pgt_device *dev)
{
	/*
	 * Initially disable all the events. IER/IMR actually plays a
	 * similar role, thus here we want to always unmask all IMR
	 * bits, so that IER is the only register to be populated later
	 * at run-time.
	 */
	VGT_MMIO_WRITE(dev, _REG_DEIER, 0U);	/* disable all events */
	VGT_MMIO_WRITE(dev, _REG_DEIMR, 0U);	/* but unmask all events */
	VGT_MMIO_WRITE(dev, _REG_SDEIER, 0U);
	VGT_MMIO_WRITE(dev, _REG_SDEIMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_SDEIIR, VGT_MMIO_READ(dev, _REG_SDEIIR));
	VGT_MMIO_WRITE(dev, _REG_DEIIR, VGT_MMIO_READ(dev, _REG_DEIIR));

	VGT_MMIO_WRITE(dev, _REG_GTIER, 0U);
	VGT_MMIO_WRITE(dev, _REG_GTIMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_RCS_IMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_VCS_IMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_BCS_IMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_GTIIR, VGT_MMIO_READ(dev, _REG_GTIIR));

	VGT_MMIO_WRITE(dev, _REG_PMIER, 0U);
	VGT_MMIO_WRITE(dev, _REG_PMIMR, 0U);
	VGT_MMIO_WRITE(dev, _REG_PMIIR, VGT_MMIO_READ(dev, _REG_PMIIR));

	/* enable master interrupt bit */
	VGT_MMIO_WRITE(dev, _REG_DEIER, _REGBIT_MASTER_INTERRUPT);

	/* Install vreg handlers */
#if 0
	vgt_register_vreg_handler(_REG_DEIMR, vgt_reg_imr_handler);
	vgt_register_vreg_handler(_REG_DEIER, vgt_reg_ier_handler);
	vgt_register_vreg_handler(_REG_PMIMR, vgt_reg_imr_handler);
	vgt_register_vreg_handler(_REG_PMIER, vgt_reg_ier_handler);
	vgt_register_vreg_handler(_REG_GTIMR, vgt_reg_imr_handler);
	vgt_register_vreg_handler(_REG_GTIER, vgt_reg_ier_handler);
	vgt_register_vreg_handler(_REG_SDEIMR, vgt_reg_imr_handler);
	vgt_register_vreg_handler(_REG_SDEIER, vgt_reg_ier_handler);

	vgt_register_vreg_handler(_REG_RCS_WATCHDOG_CTL, vgt_reg_watchdog_handler);
	vgt_register_vreg_handler(_REG_RCS_WATCHDOG_THRSH, vgt_reg_watchdog_handler);
	vgt_register_vreg_handler(_REG_RCS_WATCHDOG_CTR, vgt_reg_watchdog_handler);
	vgt_register_vreg_handler(_REG_VCS_WATCHDOG_CTR, vgt_reg_watchdog_handler);
	vgt_register_vreg_handler(_REG_VCS_WATCHDOG_THRSH, vgt_reg_watchdog_handler);
#endif
	/* Set a list of pass-through regs */
	//vgt_set_vreg_policy(..., ...);
}

static void vgt_snb_irq_exit(struct pgt_device *dev){
	// leave empty for now
}

static void vgt_snb_irq_save(struct vgt_device *vstate,
		enum vgt_owner_type owner)
{
	switch (owner) {
		case VGT_OT_DISPLAY:
			/* mask display IER in the switch? */
			break;
		default:
			break;
	}
}

static void vgt_snb_irq_restore(struct vgt_device *vstate,
		enum vgt_owner_type owner)
{
	switch (owner) {
		case VGT_OT_RENDER:
			vgt_restore_vreg(vstate, _REG_RCS_HWSTAM);
			vgt_restore_vreg(vstate, _REG_VCS_HWSTAM);
			vgt_restore_vreg(vstate, _REG_BCS_HWSTAM);

			/*
			 * GT is always owned by a single VM at a time, so it's
			 * safe to simply restore whole IER from new owner
			 */
			vgt_restore_vreg(vstate, _REG_GTIER);

			/* no emulation/pass-through switch for GT events */
			break;
		case VGT_OT_DISPLAY:
			{
				//uint32_t deier;

				/* invoked after display context switch */
				dprintk("Dynamic ownership change for display requested (->%d)\n",
					vgt_get_id(vstate));

				/* display IER may be shared by multiple VMs */

				break;
			}
		default:
			break;
	}
}

enum vgt_event_type vgt_snb_get_event_type_from_bit(struct pgt_device *dev, uint32_t reg, uint32_t bit)
{
	struct vgt_irq_info *info = vgt_snb_get_irq_info_from_reg(reg);

	ASSERT(info != NULL);
	return info->table[bit].event;
}

struct vgt_irq_ops snb_irq_ops = {
	.init = vgt_snb_irq_init,
	.exit = vgt_snb_irq_exit,
	.interrupt = vgt_snb_interrupt,
	.toggle_hw_event = vgt_snb_toggle_hw_event,
	.save = vgt_snb_irq_save,
	.restore = vgt_snb_irq_restore,
	.get_event_type_from_bit = vgt_snb_get_event_type_from_bit,
	.get_bit_from_event = vgt_snb_get_bit_from_event,
	.get_irq_info_from_event = vgt_snb_get_irq_info_from_event,
};
