/*
 * Display context switch
 *
 * This file is almost a copy from i915_suspend.c
 * So currenly marked as  GPLv2 license.  When using or
 * redistributing this file, you may do so under such license.
 *
 * Copyright 2008 (c) Intel Corporation
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
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
 */
#include <linux/slab.h>
#include <linux/delay.h>
#include <xen/vgt.h>
#include "vgt.h"

#define vgt_restore_mmio_reg(offset)							\
	do {										\
		__sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg(vgt, (offset))); \
		VGT_MMIO_WRITE(pdev, (offset), __sreg(vgt, (offset)));			\
	} while(0)

#define vgt_restore_sreg(reg)	\
	do {	\
		VGT_MMIO_WRITE(vgt->pdev, (reg), __sreg(vgt, (reg))); \
	} while (0);

#define vgt_save_mmio_reg(offset) \
	do {									\
		__sreg(vgt, (offset)) = VGT_MMIO_READ(pdev, (offset));		\
		__vreg(vgt, (offset)) = mmio_h2g_gmadr(vgt, (offset), __sreg(vgt, (offset)));\
	} while(0)

static void vgt_update_cursor(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	ASSERT(pipe < PIPE_C);
	vgt_restore_sreg(VGT_CURPOS(pipe));
	vgt_restore_sreg(VGT_CURCNTR(pipe));
	vgt_restore_sreg(VGT_CURBASE(pipe));
}

static void vgt_restore_display(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	/* Display arbitration */
	vgt_restore_mmio_reg(_REG_DSPARB);

	/* CRT state */
	vgt_restore_mmio_reg(_REG_PCH_ADPA);
	vgt_restore_mmio_reg(_REG_PCH_LVDS);

	vgt_restore_mmio_reg(_REG_PCH_PP_ON_DELAYS);
	vgt_restore_mmio_reg(_REG_PCH_PP_OFF_DELAYS);
	vgt_restore_mmio_reg(_REG_PCH_PP_DIVISOR);
	vgt_restore_mmio_reg(_REG_PCH_PP_CONTROL);
	vgt_restore_mmio_reg(_REG_RSTDBYCTL);
}

static int vgt_save_display(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	vgt_save_mmio_reg(_REG_DSPARB);

	/* CRT state */
	vgt_save_mmio_reg(_REG_PCH_ADPA);

	vgt_save_mmio_reg(_REG_PCH_PP_CONTROL);
	vgt_save_mmio_reg(_REG_PCH_LVDS);

	vgt_save_mmio_reg(_REG_PCH_PP_ON_DELAYS);
	vgt_save_mmio_reg(_REG_PCH_PP_OFF_DELAYS);
	vgt_save_mmio_reg(_REG_PCH_PP_DIVISOR);

	/* TODO: handle FBC state on the platform that supports FBC */

	/* VGA state */
	vgt_save_mmio_reg(_REG_VGA0);
	vgt_save_mmio_reg(_REG_VGA1);
	vgt_save_mmio_reg(_REG_VGA_PD);
	vgt_save_mmio_reg(_REG_CPU_VGACNTRL);

	return 0;
}

static int vgt_save_state(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;

	/* Hardware status page */
	/* FIXME: _REG_HWS_PGA is only used in i915 for dmah, this
	 * not used by any other vGT code, it seems like a legacy register
	 */
	vgt_save_mmio_reg(_REG_HWS_PGA);

	vgt_save_display(vgt);

	vgt_save_mmio_reg(_REG_FDI_RXA_IMR);
	vgt_save_mmio_reg(_REG_FDI_RXB_IMR);
	vgt_save_mmio_reg(_REG_RSTDBYCTL);
	/* FIXME: in i915 side, it is called PCH_PORT_HOTPLUG */
	vgt_save_mmio_reg(_REG_SHOTPLUG_CTL);

	/* Scratch space */
	for (i = 0; i < 16; i++) {
		vgt_save_mmio_reg(_REG_SWF00 + (i << 2));
		vgt_save_mmio_reg(_REG_SWF10 + (i << 2));
	}
	for (i = 0; i < 3; i++)
		vgt_save_mmio_reg(_REG_SWF30 + (i << 2));

	return 0;
}

/* intel_flush_display_plane */
static void vgt_flush_display_plane(struct vgt_device *vgt,
		enum vgt_plane plane)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t reg_data;

	ASSERT(plane < PLANE_C);

	vgt_dbg("flush display %s", VGT_PLANE_NAME(plane));
	reg_data = VGT_MMIO_READ(pdev, VGT_DSPLINOFF(plane));
	VGT_MMIO_WRITE(pdev, VGT_DSPLINOFF(plane), reg_data);
	reg_data = VGT_MMIO_READ(pdev, VGT_DSPSURF(plane));
	VGT_MMIO_WRITE(pdev, VGT_DSPSURF(plane), reg_data);
}

static int vgt_restore_state(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;

	vgt_restore_mmio_reg(_REG_HWS_PGA);

	vgt_restore_display(vgt);

	/* Interrupt state */
	vgt_restore_mmio_reg(_REG_FDI_RXA_IMR);
	vgt_restore_mmio_reg(_REG_FDI_RXB_IMR);

	vgt_dbg("vGT: restoring DSPAXXX ...\n");
	vgt_restore_sreg(_REG_DSPACNTR);
	vgt_restore_sreg(_REG_DSPASTRIDE);
	vgt_restore_sreg(_REG_DSPASURF);
	vgt_restore_sreg(_REG_DSPATILEOFF);
	vgt_restore_sreg(_REG_DSPALINOFF);
	VGT_POST_READ(vgt->pdev, _REG_DSPACNTR);
	vgt_dbg("vGT: restoring DSPAXXX done!\n");

	vgt_dbg("vGT: restoring DSPBXXX ...\n");
	vgt_restore_sreg(_REG_DSPBCNTR);
	vgt_restore_sreg(_REG_DSPBSTRIDE);
	vgt_restore_sreg(_REG_DSPBSURF);
	vgt_restore_sreg(_REG_DSPBTILEOFF);
	vgt_restore_sreg(_REG_DSPBLINOFF);
	VGT_POST_READ(vgt->pdev, _REG_DSPACNTR);
	vgt_dbg("vGT: restoring DSPBXXX done!\n");

	for (i = 0; i < 16; i++) {
		vgt_restore_mmio_reg(_REG_SWF00 + (i << 2));
		vgt_restore_mmio_reg(_REG_SWF10 + (i << 2));
	}
	for (i = 0; i < 3; i++) {
		vgt_restore_mmio_reg(_REG_SWF30 + (i << 2));
	}

	vgt_flush_display_plane(vgt, PIPE_A);
	vgt_flush_display_plane(vgt, PIPE_B);
	vgt_update_cursor(vgt, PIPE_A);
	vgt_update_cursor(vgt, PIPE_B);

	return 0;
}

/*
 * Do monitor owner switch.
 */
void vgt_switch_foreground_vm(struct vgt_device *prev,
	struct vgt_device *next)
{
	ASSERT(fastpath_dpy_switch);
	vgt_save_state(prev);
	vgt_restore_state(next);
}

void do_vgt_display_switch(struct vgt_device *to_vgt)
{
	struct pgt_device *pdev = to_vgt->pdev;

	vgt_dbg("vGT: doing display switch: from %p to %p\n",
			current_foreground_vm(pdev), to_vgt);

	ASSERT(spin_is_locked(&pdev->lock));
	vgt_dbg("before irq save\n");
	pdev->in_ctx_switch = 1;
	vgt_irq_save_context(current_foreground_vm(pdev),
			VGT_OT_DISPLAY);
	vgt_dbg("after irq save\n");

	vgt_switch_foreground_vm(current_foreground_vm(pdev),
			to_vgt);
	current_foreground_vm(pdev) = to_vgt;

	vgt_dbg("before irq restore\n");
	vgt_irq_restore_context(to_vgt, VGT_OT_DISPLAY);
	vgt_dbg("after irq restore\n");

	pdev->in_ctx_switch = 0;
	/*
	 * Virtual interrupts pending right after display switch
	 * Need send to both prev and next owner.
	 */
	if (test_bit(VGT_REQUEST_IRQ, (void*)&pdev->request)) {
		vgt_dbg("vGT: handle pending interrupt in the display context switch time\n");
		clear_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
		vgt_handle_virtual_interrupt(pdev, VGT_OT_DISPLAY);
	}
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
