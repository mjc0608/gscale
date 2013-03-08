/*
 * vGT interrupt handler
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
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
#include <xen/interface/vcpu.h>
#include "vgt_drv.h"

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
 * TODO:
 *   - IIR could store two pending interrupts. need emulate the behavior
 *   - pipe control
 *   - error handling (related registers)
 *	page fault (GFX_ARB_ERROR_RPT, PP_PFIR)
 *	cmd error (IPEHR)
 *   - reserved bit checking in vREG?
 *   - watchdog timer control (write 1 to reset, and write 0 to start)
 *	PR_CTR_CTL, BCS_CTR_THRSH, VCS_CNTR
 *	interresting that blitter has no such interrupt
 */

/* simply forward interrupt to dom0 i915, for testing the hook mechanism */
//#define VGT_IRQ_FORWARD_MODE

/* only handle IIR/IMR/IER, with all events handled by default handler */
//#define VGT_IRQ_DEFAULT_HANDLER

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
	[IRQ_RCS_L3_PARITY_ERR] = VGT_OT_RENDER,
	[IRQ_RCS_WATCHDOG_EXCEEDED] = VGT_OT_RENDER,
	[IRQ_RCS_PAGE_DIRECTORY_FAULT] = VGT_OT_RENDER,
	[IRQ_RCS_AS_CONTEXT_SWITCH] = VGT_OT_RENDER,
	[IRQ_RCS_MONITOR_BUFF_HALF_FULL] = VGT_OT_RENDER,

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
	[IRQ_DP_A_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_AUX_CHANNEL_A] = VGT_OT_MGMT,
	[IRQ_PCH_IRQ] = VGT_OT_NONE,		// 2nd level events
	[IRQ_PERF_COUNTER] = VGT_OT_DISPLAY,
	[IRQ_POISON] = VGT_OT_DISPLAY,		// ???
	[IRQ_GTT_FAULT] = VGT_OT_DISPLAY,	// ???
	[IRQ_PRIMARY_A_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_PRIMARY_B_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_SPRITE_A_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_SPRITE_B_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_PIPE_C_VBLANK] = VGT_OT_DISPLAY,
	[IRQ_PIPE_C_VSYNC] = VGT_OT_DISPLAY,
	[IRQ_PIPE_C_LINE_COMPARE] = VGT_OT_DISPLAY,
	[IRQ_PRIMARY_C_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_SPRITE_C_FLIP_DONE] = VGT_OT_DISPLAY,
	[IRQ_ERROR_INTERRUPT_COMBINED] = VGT_OT_MGMT,

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
	[IRQ_SDVO_B_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_CRT_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_DP_B_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_DP_C_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_DP_D_HOTPLUG] = VGT_OT_DISPLAY,
	[IRQ_AUX_CHENNEL_B] = VGT_OT_MGMT,
	[IRQ_AUX_CHENNEL_C] = VGT_OT_MGMT,
	[IRQ_AUX_CHENNEL_D] = VGT_OT_MGMT,
	[IRQ_AUDIO_POWER_STATE_CHANGE_B] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_POWER_STATE_CHANGE_C] = VGT_OT_DISPLAY,
	[IRQ_AUDIO_POWER_STATE_CHANGE_D] = VGT_OT_DISPLAY,

	[IRQ_RESERVED] = VGT_OT_NONE,
};

DECLARE_BITMAP(vgt_uevents_bitmap, UEVENT_MAX);
extern struct kobject *vgt_ctrl_kobj;

bool vgt_default_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *dev)
{
    int retval;
    retval = kobject_uevent_env(vgt_ctrl_kobj, uevent_entry->action, uevent_entry->env_var_table);
    if (retval == 0)
        return true;
    else
        return false;
}

bool vgt_hotplug_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *dev)
{
	vgt_probe_edid(dev, -1);
	return vgt_default_uevent_handler(uevent_entry, dev);
}

bool vgt_vga_stat_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *dev)
{
	/* Add vmid */
	int retval;
	char vmid_str[20];
	retval = snprintf(vmid_str, 20, "VMID=%d", uevent_entry->vm_id);
	uevent_entry->env_var_table[1] = vmid_str;
	return vgt_default_uevent_handler(uevent_entry, dev);
}

/*
 When you add new uevents or add new environmental variable,
 you should following rules:
 Now you can at most define VGT_MAX_UEVENT_VARS environmental
 variables with the form like "VAR=VALUE", all the
 pointer of string are stored in env_var_table (below).
struct vgt_uevent_info {
	...
    char *env_var_table[VGT_MAX_UEVENT_VARS];
	...
};
 You should place a NULL as the termination of variable
 definition, or function add_uevent_var() in line 219
 of lib/kobject_uevent.c will fail.
*/

static struct vgt_uevent_info vgt_default_uevent_info_table[UEVENT_MAX] = {
	{"CRT insert", -1, KOBJ_ADD, {"CRT_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"CRT remove", -1, KOBJ_REMOVE, {"CRT_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP A insert", -1, KOBJ_ADD, {"DP_A_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP A remove", -1,KOBJ_REMOVE, {"DP_A_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"SDVO B insert", -1, KOBJ_ADD, {"SDVO_B_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"SDVO B remove", -1, KOBJ_REMOVE, {"SDVO_B_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP B insert", -1, KOBJ_ADD, {"DP_B_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP B remove", -1, KOBJ_REMOVE, {"DP_B_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP C insert", -1, KOBJ_ADD, {"DP_C_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP C remove", -1, KOBJ_REMOVE, {"DP_C_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP D insert", -1, KOBJ_ADD, {"DP_D_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"DP D remove", -1, KOBJ_REMOVE, {"DP_D_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI B insert", -1, KOBJ_ADD, {"HDMI_B_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI B remove", -1, KOBJ_REMOVE, {"HDMI_B_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI C insert", -1, KOBJ_ADD, {"HDMI_C_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI C remove", -1, KOBJ_REMOVE, {"HDMI_C_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI D insert", -1, KOBJ_ADD, {"HDMI_D_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"HDMI D remove", -1, KOBJ_REMOVE, {"HDMI_D_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"VGT enable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=1", NULL, NULL}, vgt_vga_stat_uevent_handler},
	{"VGT disable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=0", NULL, NULL}, vgt_vga_stat_uevent_handler},
};

void inline vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent)
{
	struct vgt_uevent_info *entry;

	ASSERT(uevent < UEVENT_MAX);

	entry = &vgt_default_uevent_info_table[uevent];
	entry->vm_id = vgt->vm_id;

	set_bit(uevent, vgt_uevents_bitmap);
}

void vgt_signal_uevent(struct pgt_device *dev)
{
    struct vgt_uevent_info *info_entry;
    bool rc;
    int bit;

    for_each_set_bit(bit, vgt_uevents_bitmap, UEVENT_MAX) {
        clear_bit(bit, vgt_uevents_bitmap);

        info_entry = &vgt_default_uevent_info_table[bit];

        ASSERT(info_entry);
        ASSERT(info_entry->vgt_uevent_handler);

        rc = info_entry->vgt_uevent_handler(info_entry, dev);
        if (rc == false)
            printk("%s: %d: vGT: failed to send uevent [%s]!\n",
                    __func__, __LINE__, info_entry->uevent_name);
    }
}

static void vgt_run_emul(struct vgt_device *vstate,
		enum vgt_event_type event, int bit, bool enable);

void vgt_show_irq_state(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	printk("--------------------\n");

	printk("....vreg (deier: %x, deiir: %x, deimr: %x, deisr: %x)\n",
			__vreg(vgt, _REG_DEIER),
			__vreg(vgt, _REG_DEIIR),
			__vreg(vgt, _REG_DEIMR),
			__vreg(vgt, _REG_DEISR));
	printk("....physical (deier: %x, deiir: %x, deimr: %x, deisr: %x)\n",
			VGT_MMIO_READ(pdev, _REG_DEIER),
			VGT_MMIO_READ(pdev, _REG_DEIIR),
			VGT_MMIO_READ(pdev, _REG_DEIMR),
			VGT_MMIO_READ(pdev, _REG_DEISR));
	printk("....vreg (gtier: %x, gtiir: %x, gtimr: %x, gtisr: %x)\n",
			__vreg(vgt, _REG_GTIER),
			__vreg(vgt, _REG_GTIIR),
			__vreg(vgt, _REG_GTIMR),
			__vreg(vgt, _REG_GTISR));
	printk("....physical (gtier: %x, gtiir: %x, gtimr: %x, gtisr: %x)\n",
			VGT_MMIO_READ(pdev, _REG_GTIER),
			VGT_MMIO_READ(pdev, _REG_GTIIR),
			VGT_MMIO_READ(pdev, _REG_GTIMR),
			VGT_MMIO_READ(pdev, _REG_GTISR));
	printk("....pch unmask(%llx)\n", vgt_pch_unmask(pdev));
	printk("....pch enable(%llx)\n", vgt_pch_enable(pdev));
	printk("....master enable(%llx)\n", vgt_master_enable(pdev));
}

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
			vgt_dbg("vGT: no enough memory for core handler table\n");
			return -ENOMEM;
		}
		vgt_core_event_handlers(dev) = handlers;
	}

	if (vgt_core_event_handler(dev, event)) {
		vgt_dbg("vGT: existing handler for event (%s)\n", vgt_irq_name[event]);
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
	int event,bit;
	struct pgt_device *dev = vstate->pdev;
	struct vgt_irq_info *info;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	printk("vGT: toggle event emulations at context switch\n");
	info = ops->get_irq_info_from_owner(dev, owner);
	for (bit = 0; bit < info->table_size; bit++) {
		if (info->table[bit].emul_handler) {
			event = info->table[bit].event;
			if (test_bit(event, vgt_always_emulated_events(dev)))
				continue;

			if (enable &&
			    !test_bit(bit, (void*)vgt_vreg(vstate, vgt_imr(info->reg_base))) &&
			    test_bit(bit, (void*)vgt_vreg(vstate, vgt_ier(info->reg_base)))) {
				vgt_run_emul(vstate, event, bit, true);
			} else {
				if (test_bit(event, vgt_state_emulated_events(vstate)))
					vgt_run_emul(vstate, event, bit, false);
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

	if (owner != VGT_OT_RENDER && owner != VGT_OT_DISPLAY) {
		vgt_dbg("Dynamic ownership update for type (%d) is prohibited\n", owner);
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

	if (owner != VGT_OT_RENDER && owner != VGT_OT_DISPLAY) {
		vgt_dbg("Dynamic ownership update for type (%d) is prohibited\n", owner);
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
	vgt_dbg("Event ownership mapping change is not supported now\n");
	return -EINVAL;
}
#endif

/* ========================virq injection===================== */

extern int resend_irq_on_evtchn(unsigned int i915_irq);

/*
 * assumptions:
 *   - rising edge to trigger an event to next level
 *   - only cache one instance for IIR now
 */
void vgt_propagate_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info)
{
	vgt_dbg("vGT: visr(%x), vimr(%x), viir(%x), vier(%x), deier(%x)\n",
		__vreg(vstate, vgt_isr(info->reg_base)),
		__vreg(vstate, vgt_imr(info->reg_base)),
		__vreg(vstate, vgt_iir(info->reg_base)),
		__vreg(vstate, vgt_ier(info->reg_base)),
		__vreg(vstate, _REG_DEIER));
	/* Rising edge ISR triggers IIR. so no need to touch ISR */
//	if (!test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base))) &&
	if (!test_bit(bit, (void*)vgt_vreg(vstate, vgt_imr(info->reg_base))) &&
	    !test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))) &&
	    test_bit(bit, (void*)vgt_vreg(vstate, vgt_ier(info->reg_base))) &&
	    test_bit(_REGSHIFT_MASTER_INTERRUPT, (void*)vgt_vreg(vstate, _REG_DEIER))) {
		if (vstate->vgt_id)
			vgt_dbg("vGT: set bit (%d) for (%s) for VM (%d)\n",
				bit, info->name, vstate->vgt_id);
		vgt_set_irq_pending(vstate);
		vstate->stat.last_propagation = get_cycles();
		vstate->stat.events[info->table[bit].event]++;
	} else {
		if (vstate->vgt_id) {
			vgt_dbg("vGT: propagate bit (%d) for (%s) for VM (%d) w/o injection\n",
				bit, info->name, vstate->vgt_id);
			vgt_dbg("vGT: visr(%x), vimr(%x), viir(%x), vier(%x), deier(%x)\n",
				__vreg(vstate, vgt_isr(info->reg_base)),
				__vreg(vstate, vgt_imr(info->reg_base)),
				__vreg(vstate, vgt_iir(info->reg_base)),
				__vreg(vstate, vgt_ier(info->reg_base)),
				__vreg(vstate, _REG_DEIER));
		}
		vstate->stat.last_blocked_propagation = get_cycles();
	}

#if 0
	if (test_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))))
		clear_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base)));
#endif
}

/*
 * propagate PCH specific event, which will be chained to level-1 ISR later
 * similarly need consider IIR which can store two pending instances
 */
void vgt_propagate_pch_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info)
{
	/* Rising edge ISR triggers IIR. so no need to touch ISR */
//	if (!test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base))) &&
	if (!test_bit(bit, (void*)vgt_vreg(vstate, vgt_imr(info->reg_base))) &&
	    !test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))) &&
	    test_bit(bit, (void*)vgt_vreg(vstate, vgt_ier(info->reg_base)))) {
		vgt_dbg("vGT: set pch bit (%d) for VM (%d)\n", bit, vstate->vgt_id);
		vgt_set_pch_irq_pending(vstate);
	} else {
		vgt_dbg("vGT: propagate pch bit (%d) for VM (%d) w/o injection\n", bit, vstate->vgt_id);
		vgt_dbg("vGT: visr(%x), vimr(%x), viir(%x), vier(%x)i\n",
			__vreg(vstate, vgt_isr(info->reg_base)),
			__vreg(vstate, vgt_imr(info->reg_base)),
			__vreg(vstate, vgt_iir(info->reg_base)),
			__vreg(vstate, vgt_ier(info->reg_base)));
	}

#if 0
	if (test_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))))
		clear_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base)));
#endif
}

/*
 * FIXME: need to handle PCH propagation. Also it'd be good to share
 * same handler as in physical interrupt path, since this can only
 * handle IIR-only events.
 */
void vgt_propagate_emulated_event(struct vgt_device *vstate,
	enum vgt_event_type event)
{
	int bit;
	struct pgt_device *dev = vstate->pdev;
	struct vgt_irq_info *info;
	struct vgt_irq_info_entry *entry;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	info = ops->get_irq_info_from_event(dev, event);
	bit = ops->get_bit_from_event(dev, event, info);
	entry = info->table + bit;
	ASSERT(entry->event == event);
	vgt_propagate_virtual_event(vstate, bit, info);
}

void inject_dom0_virtual_interrupt(struct vgt_device *vgt)
{
	unsigned long flags;
	vgt_dbg("vGT: resend irq for dom0\n");
	/* resend irq may unmask events which requires irq disabled */
	local_irq_save(flags);
	resend_irq_on_evtchn(vgt_i915_irq(vgt->pdev));
	local_irq_restore(flags);
}

#define MSI_CAP_OFFSET 0x90	/* FIXME. need to get from cfg emulation */
#define MSI_CAP_CONTROL (MSI_CAP_OFFSET + 2)
#define MSI_CAP_ADDRESS (MSI_CAP_OFFSET + 4)
#define MSI_CAP_DATA	(MSI_CAP_OFFSET + 8)
void inject_hvm_virtual_interrupt(struct vgt_device *vgt)
{
	struct vcpu_raw_msi_info info;
	char *cfg_space = &vgt->state.cfg_space[0];
	uint16_t control = *(uint16_t *)(cfg_space + MSI_CAP_CONTROL);

	/* FIXME: now only handle one MSI format */
	ASSERT_NUM(!(control & 0xfffe), control);
	info.dom = vgt->vm_id;
	info.address = *(uint32_t *)(cfg_space + MSI_CAP_ADDRESS);
	info.data = *(uint16_t *)(cfg_space + MSI_CAP_DATA);
	vgt_dbg("vGT(%d): hvm injections. address (%x) data(%x)!\n",
		vgt->vgt_id, info.address, info.data);

	if (HYPERVISOR_vcpu_op(VCPUOP_inject_raw_msi,
                        smp_processor_id(), &info) < 0)
		printk("vGT(%d): failed to inject vmsi\n", vgt->vgt_id);
}

static int vgt_inject_virtual_interrupt(struct vgt_device *vstate)
{
	if (vstate->vm_id)
		inject_hvm_virtual_interrupt(vstate);
	else
		inject_dom0_virtual_interrupt(vstate);

#ifndef VGT_IRQ_FORWARD_MODE
	vgt_clear_irq_pending(vstate);
#endif

	vstate->stat.irq_num++;
	vstate->stat.last_injection = get_cycles();
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

	info = ops->get_irq_info_from_event(dev, event);
	ASSERT(info);

	entry = info->table + bit;
	if (entry->emul_handler) {
		vgt_dbg("vGT-IRQ(%d): %s emul handler for event %s\n", vstate->vgt_id,
				enable ? "enable" : "disable", vgt_irq_name[event]);
		if (!enable)
			clear_bit(event, vgt_state_emulated_events(vstate));

		entry->emul_handler(vstate, event, enable);

		if (enable)
			set_bit(event, vgt_state_emulated_events(vstate));
	}
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

	vgt_dbg("vGT: toggle event emulations at imr/ier emulation for reg (%s) bits (%x)\n",
		ops->get_reg_name(pdev, reg), bits);
	if (((reg == _REG_DEIER) || (reg == _REG_DEIMR))) {
		if (pdev->is_sandybridge)
			bits &= ~(_REGBIT_PCH | _REGBIT_MASTER_INTERRUPT);
		else if (pdev->is_ivybridge || pdev->is_haswell)
			bits &= ~(_REGBIT_PCH_GEN7 | _REGBIT_MASTER_INTERRUPT);
	}

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
		if (bits & _REGBIT_MASTER_INTERRUPT)
			val |= _REGBIT_MASTER_INTERRUPT;

		if (pdev->is_sandybridge) {
			if (bits & _REGBIT_PCH)
				val |= _REGBIT_PCH;
			bits &= ~(_REGBIT_PCH | _REGBIT_MASTER_INTERRUPT);
		} else if (pdev->is_ivybridge || pdev->is_haswell) {
			if (bits & _REGBIT_PCH_GEN7)
				val |= _REGBIT_PCH_GEN7;
			bits &= ~(_REGBIT_PCH_GEN7 | _REGBIT_MASTER_INTERRUPT);
		}
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

static void vgt_check_pending_events(struct vgt_device *vgt)
{
	if (!(__vreg(vgt, _REG_DEIER) & _REGBIT_MASTER_INTERRUPT))
		return;

	/*
	 * Here we only check IIR/IER. IMR/ISR is not checked
	 * because only rising-edge of ISR is captured as an event,
	 * so that current value of vISR doesn't matter.
	 */
	if ((__vreg(vgt, _REG_DEIIR) & __vreg(vgt, _REG_DEIER)) ||
	    (__vreg(vgt, _REG_GTIIR) & __vreg(vgt, _REG_GTIER)) ||
	    (__vreg(vgt, _REG_PMIIR) & __vreg(vgt, _REG_PMIER))) {
		vgt_dbg("vGT-IRQ: catch pending iir\n");
		vgt_set_irq_pending(vgt);
		vgt->stat.pending_events++;
		vgt_inject_virtual_interrupt(vgt);
	}
}

/* write handler for imr */
bool vgt_reg_imr_handler(struct vgt_device *state,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, masked, unmasked;
	uint32_t enabled, disabled, ier, val;
	unsigned long imr = *(unsigned long *)p_data;
	struct pgt_device *pdev = state->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	u32 pch_irq_mask = 0;

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	vgt_dbg("vGT-IRQ: capture IMR write on reg (%s) with val (%lx)\n",
		ops->get_reg_name(pdev, reg), imr);

	vgt_dbg("vGT-IRQ: old vIER(%x), vIMR(%x)\n",
		__vreg(state, reg + 8), __vreg(state, reg));
	vgt_dbg("vGT-IRQ: old IER(%x), IMR(%x)\n",
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

	vgt_dbg("vGT-IRQ: changed (%x), masked(%x), unmasked (%x), enabled(%x), disabled (%x)\n",
		changed, masked, unmasked, enabled, disabled);
	__vreg(state, reg) = imr;

	vgt_check_pending_events(state);

	if (pdev->is_sandybridge)
		pch_irq_mask = _REGBIT_PCH;
	else if (pdev->is_ivybridge || pdev->is_haswell)
		pch_irq_mask = _REGBIT_PCH_GEN7;

	/* merge pch bits */
	if (reg == _REG_DEIMR) {
		if (masked & pch_irq_mask) {
			printk("vGT-IRQ(%d): newly mask PCH\n", state->vgt_id);
			clear_bit(state->vgt_id, (void *)&vgt_pch_unmask(pdev));
			/* only mask when all VMs mask */
			if (vgt_pch_unmask(pdev))
				masked &= ~pch_irq_mask;
		}

		if (unmasked & pch_irq_mask) {
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

		if (reg == _REG_DEIMR) {
			if (masked & 0x88)
				vgt_dbg("XXX: mask vblank/vsync (%x)\n", masked);
			if (unmasked & 0x88)
				vgt_dbg("XXX: unmask vblank/vsync (%x)\n", unmasked);
		}
		VGT_MMIO_WRITE(pdev, reg, val);
		VGT_POST_READ(pdev, reg);
	}

	/* Then handle emulated events */
	enabled &= ~(unmasked | pch_irq_mask | _REGBIT_MASTER_INTERRUPT);
	disabled &= ~(masked | pch_irq_mask | _REGBIT_MASTER_INTERRUPT);
	if (enabled)
		vgt_toggle_emulated_bits(state, reg, enabled, true);
	if (disabled)
		vgt_toggle_emulated_bits(state, reg, disabled, false);

	vgt_dbg("vGT-IRQ: new vIER(%x), vIMR(%x)\n",
		__vreg(state, reg + 8), __vreg(state, reg));
	vgt_dbg("vGT-IRQ: new IER(%x), IMR(%x)\n",
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
	uint32_t changed, imr;
	uint32_t enabled, disabled, ier_enabled, ier_disabled;
	unsigned long ier = *(unsigned long *)p_data;
	struct pgt_device *pdev = state->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	vgt_reg_t val;
	u32 pch_irq_mask = 0;

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	vgt_dbg("vGT-IRQ: capture IER write on reg (%s) with val (%lx)\n",
		ops->get_reg_name(pdev, reg), ier);

	vgt_dbg("vGT-IRQ: old vIER(%x), vIMR(%x)\n",
		__vreg(state, reg), __vreg(state, reg - 8));
	vgt_dbg("vGT-IRQ: old IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg), VGT_MMIO_READ(pdev, reg - 8));

	/* figure out newly enabled/disable bits */
	changed = __vreg(state, reg) ^ ier;
	ier_enabled = (__vreg(state, reg) & changed) ^ changed;
	ier_disabled = ier_enabled ^ changed;

	/* figure out really enabled/disabled bits */
	imr = vgt_ier_to_imr(state, reg);
	enabled = ier_enabled & ~imr;
	disabled = ier_disabled & ~imr;

	vgt_dbg("vGT_IRQ: changed (%x), i-enabled(%x), i-disabled (%x), enabled(%x), disabled(%x)\n",
		changed, ier_enabled, ier_disabled, enabled, disabled);
	__vreg(state, reg) = ier;

	vgt_check_pending_events(state);

	if (pdev->is_sandybridge)
		pch_irq_mask = _REGBIT_PCH;
	else if (pdev->is_ivybridge || pdev->is_haswell)
		pch_irq_mask = _REGBIT_PCH_GEN7;

	/* merge pch bits */
	if (reg == _REG_DEIER) {
		if (ier_enabled & pch_irq_mask) {
			printk("vGT-IRQ(%d): newly enable PCH\n", state->vgt_id);
			set_bit(state->vgt_id, (void *)&vgt_pch_enable(pdev));
		}

		if (ier_disabled & pch_irq_mask) {
			printk("vGT-IRQ(%d): newly disable PCH\n", state->vgt_id);
			clear_bit(state->vgt_id, (void *)&vgt_pch_enable(pdev));
			if (vgt_pch_enable(pdev))
				ier_disabled &= ~pch_irq_mask;
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
				vgt_dbg("vGT-IRQ(%d): newly enable MASTER\n", state->vgt_id);
				set_bit(state->vgt_id, (void *)&vgt_master_enable(pdev));
				/* leave to vgt_enable_master_interrupt */
				ier_enabled &= ~_REGBIT_MASTER_INTERRUPT;
			}

			if (ier_disabled & _REGBIT_MASTER_INTERRUPT) {
				vgt_dbg("vGT-IRQ(%d): newly disable MASTER\n", state->vgt_id);
				clear_bit(state->vgt_id, (void *)&vgt_master_enable(pdev));
				ier_disabled &= ~_REGBIT_MASTER_INTERRUPT;
			}

		}

		val = VGT_MMIO_READ(pdev, reg);
		if (ier_enabled)
			val |= ier_enabled;
		if (ier_disabled)
			val &= ~ier_disabled;
		if (reg == _REG_DEIER) {
			if (ier_disabled & 0x88)
				vgt_dbg("XXX: disable vblank/vsync (%x)\n", ier_disabled);
			if (ier_enabled & 0x88)
				vgt_dbg("XXX: enable vblank/vsync (%x)\n", ier_enabled);
		}
		if (vgt_master_enable(pdev))
			val |= _REGBIT_MASTER_INTERRUPT;
		else
			val &= ~_REGBIT_MASTER_INTERRUPT;
		VGT_MMIO_WRITE(pdev, reg, val);
		VGT_POST_READ(pdev, reg);
	}

	/* Then handle emulated events */
	enabled &= ~(ier_enabled | pch_irq_mask | _REGBIT_MASTER_INTERRUPT);
	disabled &= ~(ier_disabled | pch_irq_mask | _REGBIT_MASTER_INTERRUPT);
	if (enabled)
		vgt_toggle_emulated_bits(state, reg, enabled, true);
	if (disabled)
		vgt_toggle_emulated_bits(state, reg, disabled, false);

	vgt_dbg("vGT-IRQ: new vIER(%x), vIMR(%x)\n",
		__vreg(state, reg), __vreg(state, reg - 8));
	vgt_dbg("vGT-IRQ: new IER(%x), IMR(%x)\n",
		VGT_MMIO_READ(pdev, reg), VGT_MMIO_READ(pdev, reg - 8));
	/* TODO: throw out an early warning if no handlers for enabled events */
	return true;
}

/*
 * write handler for IIR.
 *
 * TODO: IIR can cache two pending events
 */
bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t iir = *(vgt_reg_t *)p_data;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(vgt->pdev);

	ASSERT(bytes <= 4 && !(reg & (bytes - 1)));

	vgt_dbg("vGT-IRQ: capture IIR write on reg (%s) with val (%x)\n",
		ops->get_reg_name(vgt->pdev, reg), iir);


	vgt_dbg("vGT: update %s: %x -> %x\n", ops->get_reg_name(vgt->pdev, reg),
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
	struct pgt_device *pdev = vstate->pdev;

	set_bit(vstate->vgt_id, pdev->dpy_emul_request);
	vgt_raise_request(pdev, VGT_REQUEST_EMUL_DPY_IRQ);

	hrtimer_add_expires_ns(&dpy_timer->timer, dpy_timer->period);
	return HRTIMER_RESTART;
}

static void vgt_emul_dpy_virq(struct vgt_device *vstate)
{
/* carry all display status events in one timer */
	if (test_bit(IRQ_PIPE_A_VSYNC, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_A_VSYNC);
	if (test_bit(IRQ_PIPE_A_LINE_COMPARE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_A_LINE_COMPARE);
	if (test_bit(IRQ_PIPE_A_ODD_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_A_ODD_FIELD);
	if (test_bit(IRQ_PIPE_A_EVEN_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_A_EVEN_FIELD);
	if (test_bit(IRQ_PIPE_A_VBLANK, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_A_VBLANK);
#if 0
	if (test_bit(IRQ_PIPE_B_VSYNC, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_B_VSYNC);
	if (test_bit(IRQ_PIPE_B_LINE_COMPARE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_B_LINE_COMPARE);
	if (test_bit(IRQ_PIPE_B_ODD_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_B_ODD_FIELD);
	if (test_bit(IRQ_PIPE_B_EVEN_FIELD, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_B_EVEN_FIELD);
	if (test_bit(IRQ_PIPE_B_VBLANK, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PIPE_B_VBLANK);
#endif
	if (test_bit(IRQ_PRIMARY_A_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PRIMARY_A_FLIP_DONE);
	if (test_bit(IRQ_PRIMARY_B_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_PRIMARY_B_FLIP_DONE);
	if (test_bit(IRQ_SPRITE_A_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_SPRITE_A_FLIP_DONE);
	if (test_bit(IRQ_SPRITE_B_FLIP_DONE, vgt_state_emulated_events(vstate)))
		vgt_propagate_emulated_event(vstate, IRQ_SPRITE_B_FLIP_DONE);

}

void vgt_emul_and_inject_dpy_virq(struct pgt_device *pdev)
{
	struct vgt_device *vgt;
	int i;
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt && test_and_clear_bit(vgt->vgt_id, pdev->dpy_emul_request)) {
			vgt_emul_dpy_virq(vgt);
			if (vgt_has_irq_pending(vgt))
				vgt_inject_virtual_interrupt(vgt);
		}

	}
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
	vgt_dbg("vGT: watch dog emulation is not supported yet\n");
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
	info->propagate_virtual_event(vgt, bit, info);
}

/* not assumed to be invoked, e.g. 'sync flush' expected to be polled! */
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
	union vgt_event_state state = vgt_event_state(dev, entry->event);

	/* always warn for errors */
	VGT_IRQ_WARN(info, entry->event, "ERROR ERROR!!!\n");

	/* check error status */
	switch (entry->event) {
		case IRQ_RCS_CMD_STREAMER_ERR:
			reg = _REG_RCS_EIR;
		case IRQ_BCS_CMD_STREAMER_ERR:
			reg = _REG_BCS_EIR;
		default:
			printk("no reg info to propagate\n");
			reg = _REG_INVALID;
	};
	ASSERT(reg != _REG_INVALID);

	if (physical) {
		/* clear EIR bits */
		val = VGT_MMIO_READ(dev, reg);
		VGT_MMIO_WRITE(dev, reg, val);

		/* save error states */
		state.cmd_err.eir_reg = reg;
		state.cmd_err.eir_val = val;
		return;
	}

	/*
	 * This update should be safe, even when there's another instance of
	 * the same event comes. The current pending bit of the event has been
	 * cleared before invoking this handler. So either this update gets
	 * the old version, or even get the new version, it doesn't matter.
	 */
	__vreg(vgt, state.cmd_err.eir_reg) = state.cmd_err.eir_val;

	/*
	 * FIXME:
	 * there're several conditions contributing to the error condition
	 * such as command error, page table error, etc. However next level
	 * error information for the condition is only for debug-only
	 * purpose. I didn't find consistent information cross manual, and
	 * thus leave them unhandled for now.
	 */

	info->propagate_virtual_event(vgt, bit, info);
}

void vgt_handle_phase_in(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	uint32_t val;

	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured Phase-In event!!!\n");

	if (physical) {
		val = VGT_MMIO_READ(dev, _REG_BLC_PWM_CTL2);
		val &= ~_REGBIT_PHASE_IN_IRQ_STATUS;
		VGT_MMIO_WRITE(dev, _REG_BLC_PWM_CTL2, val);
		return;
	}

	__vreg(vgt, _REG_BLC_PWM_CTL2) |= _REGBIT_PHASE_IN_IRQ_STATUS;
	info->propagate_virtual_event(vgt, bit, info);
}

void vgt_handle_histogram(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	uint32_t val;

	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured Histogram event!!!\n")

	if (physical) {
		val = VGT_MMIO_READ(dev, _REG_HISTOGRAM_THRSH);
		val &= ~_REGBIT_HISTOGRAM_IRQ_STATUS;
		VGT_MMIO_WRITE(dev, _REG_HISTOGRAM_THRSH, val);
		return;
	}

	__vreg(vgt, _REG_HISTOGRAM_THRSH) |= _REGBIT_HISTOGRAM_IRQ_STATUS;
	info->propagate_virtual_event(vgt, bit, info);
}

void vgt_clear_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
{
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	ASSERT(!(offset & 0x3));

	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) & (~value);
			__vreg(vgt, offset) = vreg_data;
		}
	}
}

void vgt_set_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
{
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	ASSERT(!(offset & 0x3));

	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) | value;
			__vreg(vgt, offset) = vreg_data;
		}
	}
}

/*
 * TODO: It's said that CRT hotplug detection through below method does not
 * always work. For example in Linux i915 not hotplug handler is installed
 * for CRT (likely through some other polling method). But let's use this
 * as the example for how hotplug event is generally handled here.
 */
void vgt_handle_crt_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	vgt_reg_t adpa_ctrl;
	struct pgt_device *pdev = vgt->pdev;

	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured CRT hotplug event!!!\n")

	if (physical) {
		pdev->probe_ports = true;

		adpa_ctrl = VGT_MMIO_READ(pdev, _REG_PCH_ADPA);
		if (!(adpa_ctrl & _REGBIT_ADPA_DAC_ENABLE))
			printk("vGT: captured CRT hotplug event when CRT is disabled\n");

		/* write back value to clear channel status */
		VGT_MMIO_WRITE(pdev, _REG_PCH_ADPA, adpa_ctrl);

		/* check blue/green channel status for attachment status */
		if (adpa_ctrl & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK) {
			printk("%s: %d: vGT: detect crt insert event!\n", __func__, __LINE__);

			if (test_and_set_bit(VGT_CRT, dev->port_detect_status))
				printk("vGT: capture CRT hot-plug when it's attached!\n");
			vgt_set_uevent(vgt, CRT_HOTPLUG_IN);
		} else {
			printk("%s: %d: vGT: detect crt removal event!\n", __func__, __LINE__);

			if (!test_and_clear_bit(VGT_CRT, dev->port_detect_status))
				printk("vGT: capture CRT hot-removal when it's disattached!\n");
			vgt_set_uevent(vgt, CRT_HOTPLUG_OUT);
		}

		vgt_event_state(pdev, entry->event).val = adpa_ctrl;
		/* send out udev events when handling physical interruts */
		vgt_raise_request(dev, VGT_REQUEST_UEVENT);
		return;
	}

	/* update channel status */
	__vreg(vgt, _REG_PCH_ADPA) &= ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
	__vreg(vgt, _REG_PCH_ADPA) |= vgt_event_state(pdev, entry->event).val &
				      _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
	info->propagate_virtual_event(vgt, bit, info);
}

void vgt_handle_aux_channel(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured aux channel event (no handler)!!!\n")
	vgt_default_event_handler(dev, bit, entry, info, physical, vgt);
}

void vgt_handle_gmbus(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt)
{
	VGT_IRQ_WARN_ONCE(info, entry->event, "Captured gmbus event (no handler)!!!\n")
	vgt_default_event_handler(dev, bit, entry, info, physical, vgt);
}

/* core event handling loop for a given IIR */
void vgt_irq_handle_event(struct pgt_device *dev, void *iir,
	struct vgt_irq_info *info, bool physical,
	enum vgt_owner_type o_type)
{
	int bit;
	struct vgt_irq_info_entry *entry;

	ASSERT(spin_is_locked(&dev->lock));

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

		if (physical) {
			dev->stat.events[entry->event]++;
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
		if (!physical &&
		    vgt_get_event_owner_type(dev, entry->event) == o_type &&
		    vgt_get_previous_owner(dev, o_type) != NULL) {
			vgt_dbg("vGT: inject event (%s) to previous owner (%d)\n",
				vgt_irq_name[entry->event],
				vgt_get_previous_owner(dev, o_type)->vgt_id);
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
 * Trigger a virtual event which comes from other requests like hotplug agent
 * instead of from pirq.
 */
void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event, bool check)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_info *info;
	struct vgt_irq_info_entry *entry;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	int bit;

	ASSERT(spin_is_locked(&pdev->lock));

	info = ops->get_irq_info_from_event(pdev, event);
	ASSERT(info);
	bit = ops->get_bit_from_event(pdev, event, info);
	entry = info->table + bit;

	/* invoke the event handler indicating a virtual event */
	if (entry->event_handler)
		entry->event_handler(pdev, bit, entry, info, false, vgt);
	else {
		printk("vGT: trigger a virtual event w/o handler. Use default\n");

		vgt_default_event_handler(pdev, bit, entry, info, false, vgt);
	}

	/* forward to DE if any PCH event pending */
	if (VGT_PCH_EVENT(event) &&
	    vgt_has_pch_irq_pending(vgt)) {
		int bit_de;
		struct vgt_irq_info *info_de;

		vgt_clear_pch_irq_pending(vgt);
		info_de = ops->get_irq_info_from_event(pdev, IRQ_PCH_IRQ);
		ASSERT(info_de);
		bit_de = ops->get_bit_from_event(pdev, IRQ_PCH_IRQ, info_de);
		vgt_propagate_virtual_event(vgt, bit_de, info_de);
	}

	if (check && vgt_has_irq_pending(vgt))
		vgt_inject_virtual_interrupt(vgt);
}

/*
 * VGT_OT_NONE indicates normal injection, and other valid types indicate injections
 * to both prev/next owners
 */
void vgt_handle_virtual_interrupt(struct pgt_device *pdev, enum vgt_owner_type type)
{
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	int i;
	cycles_t delay;

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
	ops->handle_virtual_interrupt(pdev, type);

	/* check pending virtual interrupt for active VMs. */
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i] && vgt_has_irq_pending(pdev->device[i]))
			vgt_inject_virtual_interrupt(pdev->device[i]);
	}
	pdev->stat.virq_cycles += get_cycles() - pdev->stat.last_virq;
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

	spin_lock(&dev->lock);
	dev->stat.irq_num++;
	dev->stat.last_pirq = get_cycles();
	vgt_dbg("vGT: receive interrupt (de-%x, gt-%x, pch-%x, pm-%x)\n",
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
		vgt_dbg("Spurious interrupt received (or shared vector)\n");
		VGT_MMIO_WRITE(dev, _REG_DEIER, de_ier);
		spin_unlock(&dev->lock);
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

	dev->stat.pirq_cycles += get_cycles() - dev->stat.last_pirq;
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}


static inline int get_event_and_edid_info(vgt_hotplug_cmd_t cmd,
				enum vgt_event_type *pevent,
				edid_index_t *pedid_idx)
{
	int ret = 0;
	switch(cmd.port_sel) {
	case 0:
		*pedid_idx = EDID_VGA;
		*pevent = IRQ_CRT_HOTPLUG;
		break;
	case 1:
		*pedid_idx = EDID_MAX;
		*pevent = IRQ_MAX;
		printk("vGT: No support for hot plug type: DP_A!\n");
		ret = -EINVAL;
		break;
	case 2:
		*pedid_idx = EDID_DPB;
		*pevent = IRQ_DP_B_HOTPLUG;
		break;
	case 3:
		*pedid_idx = EDID_DPC;
		*pevent = IRQ_DP_C_HOTPLUG;
		break;
	case 4:
		*pedid_idx = EDID_DPD;
		*pevent = IRQ_DP_D_HOTPLUG;
		break;
	default:
		*pedid_idx = EDID_MAX;
		*pevent = IRQ_MAX;
		printk("vGT: Not supported hot plug type: 0x%x!\n",
			cmd.port_sel);
		ret = -EINVAL;
		break;
	}
	return ret;
}

void vgt_trigger_display_hot_plug(struct pgt_device *dev,
		vgt_hotplug_cmd_t  hotplug_cmd)
{
	int i;
	enum vgt_event_type event = IRQ_MAX;
	edid_index_t edid_idx = EDID_MAX;

	if (get_event_and_edid_info(hotplug_cmd, &event, &edid_idx) < 0)
		return;

	/* Default: send hotplug virtual interrupts to all VMs currently.
	 * Since 'vmid' has no concern with vgt_id, e.g.  if you have a HVM
	 * with vmid = 1 and after destroy & recreate it, its vmid become 2
	 * we need to use vmid_2_vgt_device() to map vmid to vgt_device if
	 * we need to send these hotplug virtual interrupts to a specific vm
	 */
	spin_lock_irq(&dev->lock);
	for (i = 0; i < VGT_MAX_VMS; ++ i) {
		struct vgt_device *vgt = dev->device[i];

		if (!vgt)
			continue;

		if (hotplug_cmd.action == 0x1) {
			/* plug in */
			vgt_propagate_edid(vgt, edid_idx);
		} else {
			/* pull out */
			vgt_clear_edid(vgt, edid_idx);
		}

		vgt_trigger_virtual_event(vgt, event, true);
	}

	spin_unlock_irq(&dev->lock);
	return;
}

/* =====================Initializations======================= */

static void vgt_initialize_always_emulated_events(struct pgt_device *dev)
{
	/* timers are always emulated */
	set_bit(IRQ_RCS_WATCHDOG_EXCEEDED, vgt_always_emulated_events(dev));
	set_bit(IRQ_VCS_WATCHDOG_EXCEEDED, vgt_always_emulated_events(dev));
	vgt_get_event_owner_type(dev, IRQ_RCS_WATCHDOG_EXCEEDED) = VGT_OT_NONE;
	vgt_get_event_owner_type(dev, IRQ_VCS_WATCHDOG_EXCEEDED) = VGT_OT_NONE;
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
	if (irq_hstate == NULL)
		return -ENOMEM;

	dev->irq_hstate = irq_hstate;

	/* FIXME IVB: check any difference */
	if (dev->is_sandybridge || dev->is_ivybridge || dev->is_haswell)
		dev->irq_hstate->ops = &snb_irq_ops;
	else {
		vgt_dbg("vGT: no irq ops found!\n");
		return -EINVAL;
	}

	/* Initialize ownership table, based on a default policy table */
	o_table = kmalloc(IRQ_MAX * sizeof(enum vgt_owner_type), GFP_KERNEL);
	if (!o_table) {
		kfree(dev->irq_hstate);
		dev->irq_hstate = NULL;
		vgt_dbg("vGT: no enough memory for owner table\n");
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
	if (dev->irq_hstate != NULL) {
		kfree(vgt_event_owner_table(dev));
		kfree(dev->irq_hstate);
	}
}

int vgt_vstate_irq_init(struct vgt_device *vgt)
{
	struct vgt_emul_timer *dpy_timer;
	struct vgt_irq_virt_state *irq_vstate;

	irq_vstate = kzalloc(sizeof(struct vgt_irq_virt_state), GFP_KERNEL);
	if(irq_vstate == NULL)
		return -ENOMEM;

	dpy_timer = &irq_vstate->dpy_timer;

	hrtimer_init(&dpy_timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dpy_timer->timer.function = vgt_dpy_timer_fn;
	dpy_timer->period = VGT_DPY_EMUL_PERIOD;

	irq_vstate->vgt = vgt;
	vgt->irq_vstate = irq_vstate;
	/* Assume basic domain information has been retrieved already */
	printk("vGT: interrupt initialization for domain (%d) completes\n", vgt->vm_id);
	return 0;
}

void vgt_vstate_irq_exit(struct vgt_device *vgt)
{
	hrtimer_cancel(&vgt_dpy_timer(vgt).timer);
	kfree(vgt->irq_vstate);
}

void vgt_install_irq(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;
	int irq, ret;

	if (!xen_initial_domain() || novgt)
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

	vgt_pirq(pgt) = pdev->irq;
	vgt_i915_irq(pgt) = irq;
	pdev->irq = irq;

	printk("vGT: allocate virq (%d) for i915, while keep original irq (%d) for vgt\n",
		vgt_i915_irq(pgt), vgt_pirq(pgt));
}
EXPORT_SYMBOL(vgt_install_irq);
