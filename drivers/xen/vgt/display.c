/*
 * Display context switch
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
 * Copyright 2008 (c) Intel Corporation
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
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

#include <linux/slab.h>
#include <linux/delay.h>
#include "vgt.h"

#define vgt_restore_sreg(reg)	\
	do {	\
		VGT_MMIO_WRITE(vgt->pdev, (reg), __sreg(vgt, (reg))); \
	} while (0);

static int vgt_restore_state(struct vgt_device *vgt, enum vgt_pipe pipe)
{
#if 0
	unsigned int pipe_ctrl = VGT_MMIO_READ(vgt->pdev, VGT_PIPECONF(pipe));
	if (pipe_ctrl & _REGBIT_PIPE_ENABLE) {
#endif
		vgt_dbg ("start to restore pipe %d.\n", pipe + 1);
		vgt_restore_sreg(VGT_DSPCNTR(pipe));
		vgt_restore_sreg(VGT_DSPSTRIDE(pipe));
		vgt_restore_sreg(VGT_DSPSURF(pipe));
		vgt_restore_sreg(VGT_DSPTILEOFF(pipe));
		vgt_restore_sreg(VGT_DSPLINOFF(pipe));

		vgt_restore_sreg(VGT_CURPOS(pipe));
		vgt_restore_sreg(VGT_CURCNTR(pipe));
		vgt_restore_sreg(VGT_CURBASE(pipe));
		vgt_dbg ("finished pipe %d restore.\n", pipe + 1);
#if 0
	} else {
		vgt_dbg ("pipe %d is not enabled.\n", pipe + 1);
	}
#endif
	return 0;
}

/*
 * Do foreground vm switch.
 */
void do_vgt_fast_display_switch(struct vgt_device *to_vgt)
{
	struct pgt_device *pdev = to_vgt->pdev;
	enum vgt_pipe pipe;

	vgt_dbg("vGT: doing display switch: from %p to %p\n",
			current_foreground_vm(pdev), to_vgt);

	ASSERT(fastpath_dpy_switch);
	ASSERT(spin_is_locked(&pdev->lock));

	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; ++ pipe) {
		vgt_restore_state(to_vgt, pipe);
	}

	current_foreground_vm(pdev) = to_vgt;
}

static int display_pointer_id = 0;
void vgt_set_display_pointer(int vm_id)
{
	struct vgt_device *vgt = vmid_2_vgt_device(vm_id);

	if (!vgt) {
		vgt_dbg("vGT: invalid vm_id (%d)\n", vm_id);
		return;
	}

	VGT_MMIO_WRITE(vgt->pdev, _REG_DSPASURF, __sreg(vgt, _REG_DSPASURF));
	VGT_MMIO_WRITE(vgt->pdev, _REG_CURABASE, __sreg(vgt, _REG_CURABASE));
	vgt_dbg("vGT: set display to VM(%d) with (%x, %x)\n", vm_id,
		__sreg(vgt, _REG_DSPASURF), __sreg(vgt, _REG_CURABASE));
	display_pointer_id = vm_id;
}

ssize_t vgt_get_display_pointer(char *buf)
{
	struct vgt_device *vgt = vmid_2_vgt_device(display_pointer_id);

	return sprintf(buf, "Current pointer: id [%d] sReg[%x,%x] pReg[%x,%x]\n",
			display_pointer_id,
			__sreg(vgt, _REG_DSPASURF), __sreg(vgt, _REG_CURABASE),
			VGT_MMIO_READ(vgt->pdev, _REG_DSPASURF),
			VGT_MMIO_READ(vgt->pdev, _REG_CURABASE));
}

static inline int get_event_and_edid_info(vgt_hotplug_cmd_t cmd,
				enum vgt_event_type *pevent,
				edid_index_t *pedid_idx)
{
	int ret = 0;
	switch(cmd.port_sel) {
	case 0:
		*pedid_idx = EDID_VGA;
		*pevent = CRT_HOTPLUG;
		break;
	case 1:
		*pedid_idx = EDID_MAX;
		*pevent = EVENT_MAX;
		printk("vGT: No support for hot plug type: DP_A!\n");
		ret = -EINVAL;
		break;
	case 2:
		*pedid_idx = EDID_DPB;
		*pevent = DP_B_HOTPLUG;
		break;
	case 3:
		*pedid_idx = EDID_DPC;
		*pevent = DP_C_HOTPLUG;
		break;
	case 4:
		*pedid_idx = EDID_DPD;
		*pevent = DP_D_HOTPLUG;
		break;
	default:
		*pedid_idx = EDID_MAX;
		*pevent = EVENT_MAX;
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
	enum vgt_event_type event = EVENT_MAX;
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

		vgt_trigger_virtual_event(vgt, event);
	}

	spin_unlock_irq(&dev->lock);
	return;
}

DECLARE_BITMAP(vgt_uevents_bitmap, UEVENT_MAX);
extern struct kobject *vgt_ctrl_kobj;

bool vgt_default_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *pdev)
{
	int retval;
	retval = kobject_uevent_env(vgt_ctrl_kobj, uevent_entry->action, uevent_entry->env_var_table);
	if (retval == 0)
		return true;
	else
		return false;
}

bool vgt_hotplug_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *pdev)
{
	vgt_probe_edid(pdev, -1);
	return vgt_default_uevent_handler(uevent_entry, pdev);
}

bool vgt_vga_stat_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *pdev)
{
	/* Add vmid */
	int retval;
	char vmid_str[20];
	retval = snprintf(vmid_str, 20, "VMID=%d", uevent_entry->vm_id);
	uevent_entry->env_var_table[1] = vmid_str;
	return vgt_default_uevent_handler(uevent_entry, pdev);
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

void vgt_signal_uevent(struct pgt_device *pdev)
{
	struct vgt_uevent_info *info_entry;
	bool rc;
	int bit;

	for_each_set_bit(bit, vgt_uevents_bitmap, UEVENT_MAX) {
		clear_bit(bit, vgt_uevents_bitmap);

		info_entry = &vgt_default_uevent_info_table[bit];

		ASSERT(info_entry);
		ASSERT(info_entry->vgt_uevent_handler);

		rc = info_entry->vgt_uevent_handler(info_entry, pdev);
		if (rc == false)
			printk("%s: %d: vGT: failed to send uevent [%s]!\n",
					__func__, __LINE__, info_entry->uevent_name);
	}
}
