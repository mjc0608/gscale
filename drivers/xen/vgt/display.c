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
