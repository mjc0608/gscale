/*
 * Display context switch
 *
 * Copyright 2008 (c) Intel Corporation
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
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
				enum vgt_port_type *pedid_idx)
{
	int ret = 0;
	switch(cmd.port_sel) {
	case 0:
		*pedid_idx = VGT_CRT;
		*pevent = CRT_HOTPLUG;
		break;
	case 1:
		*pedid_idx = VGT_PORT_MAX;
		*pevent = EVENT_MAX;
		printk("vGT: No support for hot plug type: DP_A!\n");
		ret = -EINVAL;
		break;
	case 2:
		*pedid_idx = VGT_DP_B;
		*pevent = DP_B_HOTPLUG;
		break;
	case 3:
		*pedid_idx = VGT_DP_C;
		*pevent = DP_C_HOTPLUG;
		break;
	case 4:
		*pedid_idx = VGT_DP_D;
		*pevent = DP_D_HOTPLUG;
		break;
	default:
		*pedid_idx = VGT_PORT_MAX;
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
	enum vgt_port_type port_idx = VGT_PORT_MAX;

	if (get_event_and_edid_info(hotplug_cmd, &event, &port_idx) < 0)
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
			vgt_propagate_edid(vgt, port_idx);
			vgt_propagate_dpcd(vgt, port_idx);
		} else {
			/* pull out */
			vgt_clear_edid(vgt, port_idx);
			vgt_clear_dpcd(vgt, port_idx);
		}

		vgt_update_monitor_status(vgt);
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
	vgt_probe_dpcd(pdev, -1);
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
	{"PORT A insert", -1, KOBJ_ADD, {"PORT_A_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT A remove", -1,KOBJ_REMOVE, {"PORT_A_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT B insert", -1, KOBJ_ADD, {"PORT_B_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT B remove", -1, KOBJ_REMOVE, {"PORT_B_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT C insert", -1, KOBJ_ADD, {"PORT_C_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT C remove", -1, KOBJ_REMOVE, {"PORT_C_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT D insert", -1, KOBJ_ADD, {"PORT_D_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT D remove", -1, KOBJ_REMOVE, {"PORT_D_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"VGT enable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=1", NULL, NULL}, vgt_vga_stat_uevent_handler},
	{"VGT disable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=0", NULL, NULL}, vgt_vga_stat_uevent_handler},
};

void vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent)
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

void vgt_update_monitor_status(struct vgt_device *vgt)
{
	__vreg(vgt, _REG_SDEISR) &= ~(_REGBIT_DP_B_HOTPLUG |
					_REGBIT_DP_C_HOTPLUG |
					_REGBIT_DP_D_HOTPLUG);

	if (test_bit(VGT_DP_B, vgt->presented_ports) ||
		test_bit(VGT_HDMI_B, vgt->presented_ports)) {
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_B_HOTPLUG;
	}
	if (test_bit(VGT_DP_C, vgt->presented_ports) ||
		test_bit(VGT_HDMI_C, vgt->presented_ports)) {
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_C_HOTPLUG;
	}
	if (test_bit(VGT_DP_D, vgt->presented_ports) ||
		test_bit(VGT_HDMI_D, vgt->presented_ports)) {
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_D_HOTPLUG;
	}
}


static bool is_same_port(vgt_reg_t trans_coder_ctl1, vgt_reg_t trans_coder_ctl2)
{
	return (( trans_coder_ctl1 ^ trans_coder_ctl2 ) & ( _REGBIT_TRANS_DDI_PORT_MASK | _REGBIT_TRANS_DDI_MODE_SELECT_MASK ));
}

static enum vgt_pipe get_edp_input(uint32_t wr_data)
{
	enum vgt_pipe pipe;
	switch (wr_data & _REGBIT_TRANS_DDI_EDP_INPUT_MASK)
	{
		case _REGBIT_TRANS_DDI_EDP_INPUT_A_ON:
		case _REGBIT_TRANS_DDI_EDP_INPUT_A_ONOFF:
			pipe = PIPE_A;
			break;
		case _REGBIT_TRANS_DDI_EDP_INPUT_B_ONOFF:
			pipe = PIPE_B;
			break;
		case _REGBIT_TRANS_DDI_EDP_INPUT_C_ONOFF:
			pipe = PIPE_C;
			break;
		default:
			pipe = I915_MAX_PIPES;
	}
	return pipe;
}


bool rebuild_pipe_mapping(struct vgt_device *vgt, unsigned int reg, uint32_t wr_data)
{
	vgt_reg_t hw_value;
	int i = 0;

	enum vgt_pipe virtual_pipe = PIPE_A;
	enum vgt_pipe physical_pipe = PIPE_A ;

	if((_REGBIT_TRANS_DDI_FUNC_ENABLE & wr_data) == 0)
	{
		return false;
	}

	if(reg == _REG_TRANS_DDI_FUNC_CTL_A)
	{
		virtual_pipe = PIPE_A;
	}
	else if(reg == _REG_TRANS_DDI_FUNC_CTL_B)
	{
		virtual_pipe = PIPE_B;
	}
	else if(reg == _REG_TRANS_DDI_FUNC_CTL_C)
	{
		virtual_pipe = PIPE_C;
	}


	if(reg == _REG_TRANS_DDI_FUNC_CTL_EDP)
	{
		virtual_pipe = get_edp_input(wr_data);
		hw_value= VGT_MMIO_READ(vgt->pdev, _REG_TRANS_DDI_FUNC_CTL_EDP);
		physical_pipe = get_edp_input(hw_value);
	}
	else
	{
		for(i = 0; i <= TRANSCODER_C; i++)
		{
			hw_value = VGT_MMIO_READ(vgt->pdev, _VGT_TRANS_DDI_FUNC_CTL(i));
			if(is_same_port(wr_data,hw_value) )
			{
				physical_pipe = i;
				break;
			}
		}
	}

	vgt->pipe_mapping[virtual_pipe] = physical_pipe;

	return true;

}
