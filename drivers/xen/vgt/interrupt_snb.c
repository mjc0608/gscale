/*
 * vGT interrupt info table for SNB
 *
 * This file is provided under GPLv2 license.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 */

#include <linux/module.h>
#include <xen/vgt.h>
#include "vgt.h"

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
	.propagate_virtual_event = vgt_propagate_virtual_event,
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
	.propagate_virtual_event = vgt_propagate_virtual_event,
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
		{IRQ_GSE, 			vgt_default_event_handler,	NULL},	// bit 18
		{IRQ_DP_A_HOTPLUG, 		NULL,	NULL},		// bit 19
		{IRQ_AUX_CHANNEL_A, 		vgt_handle_aux_channel,	NULL},	// bit 20
		{IRQ_PCH_IRQ,			NULL,	NULL},		// bit 21
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
 * exclusive owner of this category.
 *
 * also due to the same reason, the default event handler is enough even
 * when there're related registers other than pm_iir.
 */
static struct vgt_irq_info snb_pm_irq_info = {
	.name = "SNB PM IRQ",
	.reg_base = _REG_PMISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.propagate_virtual_event = vgt_propagate_virtual_event,
	.table = {
		{IRQ_RESERVED,			NULL,	NULL},				// bit 0

		{IRQ_GV_DOWN_INTERVAL, 		vgt_default_event_handler,	NULL},	// bit 1
		{IRQ_GV_UP_INTERVAL, 		vgt_default_event_handler,	NULL},	// bit 2

		{IRQ_RESERVED, 			NULL,	NULL},

		{IRQ_RP_DOWN_THRESHOLD, 	vgt_default_event_handler,	NULL},	// bit 4
		{IRQ_RP_UP_THRESHOLD, 		vgt_default_event_handler,	NULL},	// bit 5
		{IRQ_FREQ_DOWNWARD_TIMEOUT_RC6, vgt_default_event_handler,	NULL},	// bit 6

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

		{IRQ_PCU_THERMAL, 		vgt_default_event_handler,	NULL},	// bit 24
		{IRQ_PCU_PCODE2DRIVER_MAILBOX,	vgt_default_event_handler,	NULL},	// bit 25

		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},
		{IRQ_RESERVED, NULL,	NULL},			// bit 31
	},
};

/*
 * PCH related irq info
 */
static struct vgt_irq_info snb_pch_irq_info = {
	.name = "SNB PCH IRQ",
	.reg_base = _REG_SDEISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.propagate_virtual_event = vgt_propagate_pch_virtual_event,
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
		{IRQ_SDVO_B_HOTPLUG,			NULL,	NULL},		// bit 18
		{IRQ_CRT_HOTPLUG, 			vgt_handle_crt_hotplug,	NULL},		// bit 19

		{IRQ_RESERVED, 				NULL,	NULL},

		{IRQ_DP_B_HOTPLUG, 			NULL,	NULL},		// bit 21
		{IRQ_DP_C_HOTPLUG, 			NULL,	NULL},		// bit 22
		{IRQ_DP_D_HOTPLUG, 			NULL,	NULL},		// bit 23

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

static struct vgt_irq_info gen7_de_irq_info = {
	.name = "Gen7 DE IRQ",
	.reg_base = _REG_DEISR,
	.table_size = VGT_IRQ_BITWIDTH,
	.propagate_virtual_event = vgt_propagate_virtual_event,
	.table = {
		{IRQ_PIPE_A_VBLANK,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 0
		{IRQ_PIPE_A_VSYNC,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 1
		{IRQ_PIPE_A_LINE_COMPARE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 2
		{IRQ_PRIMARY_A_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 3
		{IRQ_SPRITE_A_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 4
		{IRQ_PIPE_B_VBLANK,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 5
		{IRQ_PIPE_B_VSYNC,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 6
		{IRQ_PIPE_B_LINE_COMPARE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 7
		{IRQ_PRIMARY_B_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 8
		{IRQ_SPRITE_B_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 9
		{IRQ_PIPE_C_VBLANK,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 10
		{IRQ_PIPE_C_VSYNC,		vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 11
		{IRQ_PIPE_C_LINE_COMPARE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 12
		{IRQ_PRIMARY_C_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 13
		{IRQ_SPRITE_C_FLIP_DONE,	vgt_default_event_handler,	vgt_emulate_dpy_status},	// bit 14
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 15 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 16 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 17 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 18 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 19 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 20 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 21 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 22 reserved */
		{IRQ_RESERVED,			NULL,				NULL},				/* bit 23 reserved */
		{IRQ_DPST_PHASE_IN,		vgt_handle_phase_in,		NULL},				// bit 24
		{IRQ_DPST_HISTOGRAM,		vgt_handle_histogram,		NULL},				// bit 25
		{IRQ_AUX_CHANNEL_A,		vgt_handle_aux_channel,		NULL},				// bit 26
		{IRQ_DP_A_HOTPLUG,		NULL,	NULL},				// bit 27
		{IRQ_PCH_IRQ,			NULL,				NULL},				// bit 28
		{IRQ_GSE,			vgt_default_event_handler,	NULL},				// bit 29
		{IRQ_ERROR_INTERRUPT_COMBINED,	NULL,				NULL},				// bit 30
		{IRQ_RESERVED,			NULL,				NULL},				// bit 31
	},
};

static struct vgt_irq_info* vgt_snb_get_irq_info_from_event(struct pgt_device *dev, enum vgt_event_type event)
{
	if (VGT_RENDER_EVENT(event))
		return &snb_render_irq_info;
	else if (VGT_DPY_EVENT(event)) {
		if (IS_SNB(dev))
			return &snb_dpy_irq_info;
		else if (IS_IVB(dev) || IS_HSW(dev))
			return &gen7_de_irq_info;
	}
	else if (VGT_PCH_EVENT(event))
		return &snb_pch_irq_info;
	else if (VGT_PM_EVENT(event))
		return &snb_pm_irq_info;
	else
		return NULL;
	/* can't reach */
	return NULL;
}

static struct vgt_irq_info* vgt_snb_get_irq_info_from_owner(struct pgt_device *dev, enum vgt_owner_type owner)
{
	if (owner == VGT_OT_RENDER)
		return &snb_render_irq_info;
	/* FIXME: both de and pch should be returned */
	if (owner == VGT_OT_DISPLAY || owner == VGT_OT_MGMT) {
		if (IS_SNB(dev))
			return &snb_dpy_irq_info;
		else if (IS_IVB(dev) || IS_HSW(dev))
			return &gen7_de_irq_info;
	}
	if (owner == VGT_OT_PM)
		return &snb_pm_irq_info;
	return NULL;
}

static inline struct vgt_irq_info* vgt_snb_get_irq_info_from_reg(struct pgt_device *dev, int reg)
{
	if (reg >= _REG_GTISR && reg < _REG_GTIER + 4)
		return &snb_render_irq_info;
	else if (reg >= _REG_DEISR && reg < _REG_DEIER + 4) {
		if (IS_SNB(dev))
			return &snb_dpy_irq_info;
		else if (IS_IVB(dev) || IS_HSW(dev))
			return &gen7_de_irq_info;
	}
	else if (reg >= _REG_SDEISR && reg < _REG_SDEIER + 4)
		return &snb_pch_irq_info;
	else if (reg >= _REG_PMISR && reg < _REG_PMIER + 4)
		return &snb_pm_irq_info;
	else
		return NULL;
	/* can't reach */
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

/* Enable a hardware event by operating IER. */
static inline void vgt_snb_toggle_hw_event(struct pgt_device *dev,
	enum vgt_event_type event, int bit, bool enable)
{
	struct vgt_irq_info *info;

	printk("vGT-IRQ-SNB: %s event (%s) on bit (%d)\n",
		enable ? "enable" : "disable", vgt_irq_name[event], bit);
	info = vgt_snb_get_irq_info_from_event(dev, event);
	ASSERT(info != NULL);

	if (bit == VGT_IRQ_BITWIDTH)
		bit = vgt_snb_get_bit_from_event(dev, event, info);

	if (enable) {
		vgt_clear_reg_bit(dev, vgt_imr(info->reg_base), bit);

		/*
		 * for a vGT enabled PCH event, we need propagate to DEIER.
		 * but for VM enabled PCH event, the VM itself will enable
		 * DEIER. For safety, since vGT's own requirement is unclear
		 * yet, let's disable this logic for now
		 */
#if 0
		if (VGT_PCH_EVENT(event))
			vgt_snb_toggle_hw_event(dev, IRQ_PCH_IRQ, bit, true);
#endif
	} else {
#if 0
		if (VGT_PCH_EVENT(event))
			vgt_snb_toggle_hw_event(dev, IRQ_PCH_IRQ, bit, false);
#endif

		vgt_set_reg_bit(dev, vgt_imr(info->reg_base), bit);
	}
}

static void vgt_snb_handle_virtual_interrupt(struct pgt_device *dev, enum vgt_owner_type type)
{
	int i;
	if (vgt_gt_iir(dev)) {
		vgt_dbg("vGT-IRQ-SNB: handle virtual gt_iir(%x)\n", vgt_gt_iir(dev));
		vgt_irq_handle_event(dev, &vgt_gt_iir(dev), &snb_render_irq_info, false, type);
	}

	if (vgt_pm_iir(dev)) {
		vgt_dbg("vGT-IRQ-SNB: handle virtual pm_iir(%x)\n", vgt_pm_iir(dev));
		vgt_irq_handle_event(dev, &vgt_pm_iir(dev), &snb_pm_irq_info, false, type);
	}

	if (vgt_sde_iir(dev)) {
		vgt_dbg("vGT-IRQ-SNB: handle virtual sde_iir(%x)\n", vgt_sde_iir(dev));
		vgt_irq_handle_event(dev, &vgt_sde_iir(dev), &snb_pch_irq_info, false, type);
	}

	if (vgt_de_iir(dev)) {
		vgt_dbg("vGT-IRQ-SNB: handle virtual de_iir(%x)\n", vgt_de_iir(dev));
		if (IS_SNB(dev))
			vgt_irq_handle_event(dev, &vgt_de_iir(dev), &snb_dpy_irq_info, false, type);
		else if (IS_IVB(dev) || IS_HSW(dev))
			vgt_irq_handle_event(dev, &vgt_de_iir(dev), &gen7_de_irq_info, false, type);
	}

	/* check pending virtual PCH interrupt for active VMs */
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (dev->device[i] && vgt_has_pch_irq_pending(dev->device[i])) {
			if (IS_SNB(dev))
				vgt_propagate_virtual_event(dev->device[i],
								_REGSHIFT_PCH, &snb_dpy_irq_info);
			else if (IS_IVB(dev) || IS_HSW(dev))
				vgt_propagate_virtual_event(dev->device[i],
								_REGSHIFT_PCH_GEN7, &gen7_de_irq_info);
			vgt_clear_pch_irq_pending(dev->device[i]);
		}
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
	u32 gt_iir, pm_iir, de_iir, sde_iir, tmp_de_iir;
	int pch_irq = 0;

	/* read physical IIRs */
	gt_iir = VGT_MMIO_READ(dev, _REG_GTIIR);
	de_iir = VGT_MMIO_READ(dev, _REG_DEIIR);
	pm_iir = VGT_MMIO_READ(dev, _REG_PMIIR);

	if (!gt_iir && !de_iir && !pm_iir)
		return IRQ_NONE;

	vgt_dbg("vGT-IRQ-SNB: handle gt_iir(%x)\n", gt_iir);
	vgt_irq_handle_event(dev, &gt_iir, &snb_render_irq_info, true, VGT_OT_NONE);

	vgt_dbg("vGT-IRQ-SNB: handle de_iir(%x), tmp_de_iir(%x)\n", de_iir, tmp_de_iir);
	if (IS_SNB(dev)) {
		if (de_iir & _REGBIT_PCH)
			pch_irq = 1;
		tmp_de_iir = de_iir & ~_REGBIT_PCH;
		vgt_irq_handle_event(dev, &tmp_de_iir, &snb_dpy_irq_info, true, VGT_OT_NONE);
	} else if (IS_IVB(dev) || IS_HSW(dev)) {
		if (de_iir & _REGBIT_PCH_GEN7)
			pch_irq = 1;
		tmp_de_iir = de_iir & ~_REGBIT_PCH_GEN7;
		vgt_irq_handle_event(dev, &tmp_de_iir, &gen7_de_irq_info, true, VGT_OT_NONE);
	}

	vgt_dbg("vGT-IRQ-SNB: handle pm_iir(%x)\n", pm_iir);
	vgt_irq_handle_event(dev, &pm_iir, &snb_pm_irq_info, true, VGT_OT_NONE);

	if (pch_irq) {
		sde_iir = VGT_MMIO_READ(dev, _REG_SDEIIR);
		vgt_dbg("vGT-IRQ-SNB: handle sde_iir(%x)\n", sde_iir);
		vgt_irq_handle_event(dev, &sde_iir, &snb_pch_irq_info, true, VGT_OT_NONE);
		VGT_MMIO_WRITE(dev, _REG_SDEIIR, sde_iir);
	}

	/* clear physical IIRs in the end, after lower level causes are cleared */
	VGT_MMIO_WRITE(dev, _REG_GTIIR, gt_iir);
	VGT_MMIO_WRITE(dev, _REG_PMIIR, pm_iir);
	VGT_MMIO_WRITE(dev, _REG_DEIIR, de_iir);

	vgt_gt_iir(dev) |= gt_iir;
	vgt_pm_iir(dev) |= pm_iir;
	if (pch_irq)
		vgt_sde_iir(dev) |= sde_iir;
	vgt_de_iir(dev) |= tmp_de_iir;
	return IRQ_HANDLED;
}

static void vgt_snb_irq_init(struct pgt_device *dev)
{
	int i;
	struct vgt_irq_info *info = NULL;
	printk("vGT: snb irq init\n");

	printk("vGT: DEISR is %x, DEIIR is %x, DEIMR is %x, DEIER is %x\n",
		VGT_MMIO_READ(dev, _REG_DEISR),
		VGT_MMIO_READ(dev, _REG_DEIIR),
		VGT_MMIO_READ(dev, _REG_DEIMR),
		VGT_MMIO_READ(dev, _REG_DEIER));
	printk("vGT: SDEISR is %x, SDEIIR is %x, SDEIMR is %x, SDEIER is %x\n",
		VGT_MMIO_READ(dev, _REG_SDEISR),
		VGT_MMIO_READ(dev, _REG_SDEIIR),
		VGT_MMIO_READ(dev, _REG_SDEIMR),
		VGT_MMIO_READ(dev, _REG_SDEIER));
	printk("vGT: GTISR is %x, GTIIR is %x, GTIMR is %x, GTIER is %x\n",
		VGT_MMIO_READ(dev, _REG_GTISR),
		VGT_MMIO_READ(dev, _REG_GTIIR),
		VGT_MMIO_READ(dev, _REG_GTIMR),
		VGT_MMIO_READ(dev, _REG_GTIER));
	printk("vGT: PMISR is %x, PMIIR is %x, PMIMR is %x, PMIER is %x\n",
		VGT_MMIO_READ(dev, _REG_PMISR),
		VGT_MMIO_READ(dev, _REG_PMIIR),
		VGT_MMIO_READ(dev, _REG_PMIMR),
		VGT_MMIO_READ(dev, _REG_PMIER));
	printk("vGT: RCS_IMR is %x, VCS_IMR is %x, BCS_IMR is %x\n",
		VGT_MMIO_READ(dev, _REG_RCS_IMR),
		VGT_MMIO_READ(dev, _REG_VCS_IMR),
		VGT_MMIO_READ(dev, _REG_BCS_IMR));

	/* follow i915's setting at the boot time */
#if 0
	VGT_MMIO_WRITE(dev, _REG_SDEIER, 0);
	VGT_MMIO_WRITE(dev, _REG_SDEIMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_SDEIIR, VGT_MMIO_READ(dev, _REG_SDEIIR));

	VGT_MMIO_WRITE(dev, _REG_GTIER, 0);
	VGT_MMIO_WRITE(dev, _REG_GTIMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_RCS_IMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_VCS_IMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_BCS_IMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_GTIIR, VGT_MMIO_READ(dev, _REG_GTIIR));

	/* TODO: This may be delayed until dom0 i915 manipulates them */
	VGT_MMIO_WRITE(dev, _REG_PMIER, 0);
	VGT_MMIO_WRITE(dev, _REG_PMIMR, 0xffffffffU);
	VGT_MMIO_WRITE(dev, _REG_PMIIR, VGT_MMIO_READ(dev, _REG_PMIIR));

	VGT_MMIO_WRITE(dev, _REG_DEIER, 0);
	VGT_MMIO_WRITE(dev, _REG_DEIMR, 0x7fffffffU); /* keep master interrupt unmasked */
	VGT_MMIO_WRITE(dev, _REG_DEIIR, VGT_MMIO_READ(dev, _REG_DEIIR));
#endif

	/* Install vreg handlers */
#if 0
	vgt_register_mmio_simple(_REG_RCS_WATCHDOG_CTL, vgt_reg_watchdog_handler);
	vgt_register_mmio_simple(_REG_RCS_WATCHDOG_THRSH, vgt_reg_watchdog_handler);
	vgt_register_mmio_simple(_REG_RCS_WATCHDOG_CTR, vgt_reg_watchdog_handler);
	vgt_register_mmio_simple(_REG_VCS_WATCHDOG_CTR, vgt_reg_watchdog_handler);
	vgt_register_mmio_simple(_REG_VCS_WATCHDOG_THRSH, vgt_reg_watchdog_handler);
#endif
	/* Set a list of pass-through regs */
	//vgt_set_vreg_policy(..., ...);

	vgt_de_dpy_mask(dev) = 0;
	vgt_de_mgmt_mask(dev) = 0;

	if (IS_SNB(dev))
		info = &snb_dpy_irq_info;
	else if (IS_IVB(dev) || IS_HSW(dev))
		info = &gen7_de_irq_info;

	for (i = 0; i < info->table_size; i++) {
		if (info->table[i].event == IRQ_RESERVED)
			continue;
		switch (vgt_get_event_owner_type(dev, info->table[i].event)) {
		case VGT_OT_DISPLAY:
			set_bit(i, (void *)&vgt_de_dpy_mask(dev));
			break;
		case VGT_OT_MGMT:
			set_bit(i, (void *)&vgt_de_mgmt_mask(dev));
			break;
		default:
			break;
		}
	}

	printk("vGT-IRQ-SNB: DE display mask (%x), DE mgmt mask (%x)\n",
		vgt_de_dpy_mask(dev), vgt_de_mgmt_mask(dev));

	vgt_pch_dpy_mask(dev) = 0;
	vgt_pch_mgmt_mask(dev) = 0;
	info = &snb_pch_irq_info;
	for (i = 0; i < info->table_size; i++) {
		if (info->table[i].event == IRQ_RESERVED)
			continue;
		switch (vgt_get_event_owner_type(dev, info->table[i].event)) {
		case VGT_OT_DISPLAY:
			set_bit(i, (void *)&vgt_pch_dpy_mask(dev));
			break;
		case VGT_OT_MGMT:
			set_bit(i, (void *)&vgt_pch_mgmt_mask(dev));
			break;
		default:
			break;
		}
	}

	printk("vGT-IRQ-SNB: PCH display mask (%x), PCH mgmt mask (%x)\n",
		vgt_pch_dpy_mask(dev), vgt_pch_mgmt_mask(dev));
}

static void vgt_snb_irq_exit(struct pgt_device *dev){
	// leave empty for now
}

static void vgt_snb_irq_save(struct vgt_device *vgt,
		enum vgt_owner_type owner)
{
	vgt_reg_t val;
	unsigned long flags;
	struct pgt_device *pdev = vgt->pdev;

	local_irq_save(flags);
	switch (owner) {
		case VGT_OT_RENDER:
			/* disable all GT events */
			VGT_MMIO_WRITE(pdev, _REG_GTIER, 0U);
			VGT_POST_READ(pdev, _REG_GTIER);
			break;
		case VGT_OT_DISPLAY:
			/* disable all display events from DE and PCH. */
			val = VGT_MMIO_READ(pdev, _REG_DEIER);
			val &= ~vgt_de_dpy_mask(pdev);
			VGT_MMIO_WRITE(pdev, _REG_DEIER, val);
			VGT_POST_READ(pdev, _REG_DEIER);

			val = VGT_MMIO_READ(pdev, _REG_SDEIER);
			val &= ~vgt_pch_dpy_mask(pdev);
			VGT_MMIO_WRITE(pdev, _REG_SDEIER, val);
			VGT_POST_READ(pdev, _REG_SDEIER);
			break;
		default:
			break;
	}
	local_irq_restore(flags);
}

static void vgt_snb_irq_restore(struct vgt_device *vgt,
		enum vgt_owner_type owner)
{
	vgt_reg_t val;
	unsigned long flags;
	struct pgt_device *pdev = vgt->pdev;

	local_irq_save(flags);
	switch (owner) {
		case VGT_OT_RENDER:
			VGT_MMIO_WRITE(vgt->pdev, _REG_RCS_IMR,
				__vreg(vgt, _REG_RCS_IMR));
			VGT_POST_READ(pdev, _REG_RCS_IMR);

			VGT_MMIO_WRITE(vgt->pdev, _REG_BCS_IMR,
				__vreg(vgt, _REG_BCS_IMR));
			VGT_POST_READ(pdev, _REG_BCS_IMR);

			VGT_MMIO_WRITE(vgt->pdev, _REG_VCS_IMR,
				__vreg(vgt, _REG_VCS_IMR));
			VGT_POST_READ(pdev, _REG_VCS_IMR);

			VGT_MMIO_WRITE(vgt->pdev, _REG_GTIMR,
				__vreg(vgt, _REG_GTIMR));
			VGT_POST_READ(pdev, _REG_GTIMR);

			VGT_MMIO_WRITE(vgt->pdev, _REG_GTIER,
				__vreg(vgt, _REG_GTIER));
			VGT_POST_READ(pdev, _REG_GTIER);
			break;
		case VGT_OT_DISPLAY:
			/* merge new owner's display bits with other bits */
			val = VGT_MMIO_READ(vgt->pdev, _REG_DEIMR);
			val = (val & ~vgt_de_dpy_mask(vgt->pdev)) |
				(__vreg(vgt, _REG_DEIMR) & vgt_de_dpy_mask(vgt->pdev));
			VGT_MMIO_WRITE(vgt->pdev, _REG_DEIMR, val);
			VGT_POST_READ(pdev, _REG_DEIMR);

			val = VGT_MMIO_READ(vgt->pdev, _REG_SDEIMR);
			val = (val & ~vgt_pch_dpy_mask(vgt->pdev)) |
				(__vreg(vgt, _REG_SDEIMR) & vgt_pch_dpy_mask(vgt->pdev));
			VGT_MMIO_WRITE(vgt->pdev, _REG_SDEIMR, val);
			VGT_POST_READ(pdev, _REG_SDEIMR);

			val = VGT_MMIO_READ(vgt->pdev, _REG_SDEIER);
			val = (val & ~vgt_pch_dpy_mask(vgt->pdev)) |
				(__vreg(vgt, _REG_SDEIER) & vgt_pch_dpy_mask(vgt->pdev));
			VGT_MMIO_WRITE(vgt->pdev, _REG_SDEIER, val);
			VGT_POST_READ(pdev, _REG_SDEIER);

			val = VGT_MMIO_READ(vgt->pdev, _REG_DEIER);
			val = (val & ~vgt_de_dpy_mask(vgt->pdev)) |
				(__vreg(vgt, _REG_DEIER) & vgt_de_dpy_mask(vgt->pdev));
			VGT_MMIO_WRITE(vgt->pdev, _REG_DEIER, val);
			VGT_POST_READ(pdev, _REG_DEIER);

			printk("vGT: after restore deimr(%x) deier(%x) deiir(%x) deisr(%x)\n",
					VGT_MMIO_READ(vgt->pdev, _REG_DEIMR),
					VGT_MMIO_READ(vgt->pdev, _REG_DEIER),
					VGT_MMIO_READ(vgt->pdev, _REG_DEIIR),
					VGT_MMIO_READ(vgt->pdev, _REG_DEISR));
			printk("vGT: after restore sdeimr(%x) sdeier(%x) sdeiir(%x) sdeisr(%x)\n",
					VGT_MMIO_READ(vgt->pdev, _REG_SDEIMR),
					VGT_MMIO_READ(vgt->pdev, _REG_SDEIER),
					VGT_MMIO_READ(vgt->pdev, _REG_SDEIIR),
					VGT_MMIO_READ(vgt->pdev, _REG_SDEISR));
			break;
		default:
			break;
	}
	local_irq_restore(flags);
}

enum vgt_event_type vgt_snb_get_event_type_from_bit(struct pgt_device *dev, uint32_t reg, uint32_t bit)
{
	struct vgt_irq_info *info = vgt_snb_get_irq_info_from_reg(dev, reg);

	ASSERT(info != NULL);
	vgt_dbg("vGT-IRQ-SNB: search table (%s) for bit(%d) with type(%d, %s)\n",
		info->name, bit, info->table[bit].event, vgt_irq_name[info->table[bit].event]);
	return info->table[bit].event;
}

static char *vgt_snb_get_reg_name(struct pgt_device *dev, uint32_t reg)
{
	switch (reg) {
		case _REG_GTIIR: return "GTIIR";
		case _REG_GTIMR: return "GTIMR";
		case _REG_GTIER: return "GTIER";
		case _REG_GTISR: return "GTISR";
		case _REG_DEIIR: return "DEIIR";
		case _REG_DEIMR: return "DEIMR";
		case _REG_DEIER: return "DEIER";
		case _REG_DEISR: return "DEISR";
		case _REG_SDEIIR: return "SDEIIR";
		case _REG_SDEIMR: return "SDEIMR";
		case _REG_SDEIER: return "SDEIER";
		case _REG_SDEISR: return "SDEISR";
		case _REG_PMIIR: return "PMIIR";
		case _REG_PMIMR: return "PMIMR";
		case _REG_PMIER: return "PMIER";
		case _REG_PMISR: return "PMISR";
	}
	return "UNKNOWN";
}

struct vgt_irq_ops snb_irq_ops = {
	.init = vgt_snb_irq_init,
	.exit = vgt_snb_irq_exit,
	.interrupt = vgt_snb_interrupt,
	.handle_virtual_interrupt = vgt_snb_handle_virtual_interrupt,
	.toggle_hw_event = vgt_snb_toggle_hw_event,
	.save = vgt_snb_irq_save,
	.restore = vgt_snb_irq_restore,
	.get_event_type_from_bit = vgt_snb_get_event_type_from_bit,
	.get_bit_from_event = vgt_snb_get_bit_from_event,
	.get_irq_info_from_event = vgt_snb_get_irq_info_from_event,
	.get_irq_info_from_owner = vgt_snb_get_irq_info_from_owner,
	.get_reg_name = vgt_snb_get_reg_name,
};
