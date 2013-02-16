/*
 * vGT core module
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 Intel Corporation. All rights reserved.
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
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>

#include <xen/xen.h>
#include <xen/page.h>
#include <xen/events.h>
#include <xen/xen-ops.h>
#include <xen/interface/xen.h>
#include <xen/interface/memory.h>
#include <xen/interface/hvm/hvm_op.h>
#include <xen/interface/hvm/params.h>
#include <xen/interface/hvm/ioreq.h>

#include <xen/vgt.h>
#include <xen/vgt-if.h>
#include "vgt_drv.h"

static inline int ring_space(struct vgt_ring_buffer *ring)
{
	int space = (ring->head & RB_HEAD_OFF_MASK) - (ring->tail + 8);
	if (space < 0)
		space += ring->size;
	return space;
}

void vgt_ring_reset(struct vgt_ring_buffer *ring)
{
	ring->head = ring->tail = 0;
	ring->space = ring_space(ring);
}

void vgt_ring_init(struct pgt_device *pdev)
{
	struct vgt_ring_buffer *ring;

	pdev->ring_buffer = kzalloc(sizeof(struct vgt_ring_buffer), GFP_KERNEL);
	if (!pdev->ring_buffer) {
		printk(KERN_ERR "allocate vgt ring buffer failed!\n");
		return;
	}
	ring = pdev->ring_buffer;
	ring->pdev = pdev;
	ring->size = 4096;
	ring->offset = aperture_2_gm(pdev, rsvd_aperture_alloc(pdev, ring->size));
	ring->virtual_start = v_aperture(pdev, ring->offset);

	vgt_ring_reset(ring);
}

#define VGT_READ_CTL(pdev, id)	VGT_MMIO_READ(pdev, RB_CTL(pdev, id))
#define VGT_WRITE_CTL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_CTL(pdev, id), val)
#define VGT_POST_READ_CTL(pdev, id)	VGT_POST_READ(pdev, RB_CTL(pdev,id))

#define VGT_READ_HEAD(pdev, id)	VGT_MMIO_READ(pdev, RB_HEAD(pdev, id))
#define VGT_WRITE_HEAD(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, id), val)

#define VGT_READ_TAIL(pdev, id)	VGT_MMIO_READ(pdev, RB_TAIL(pdev, id))
#define VGT_WRITE_TAIL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, id), val)

#define VGT_READ_START(pdev, id) VGT_MMIO_READ(pdev, RB_START(pdev, id))
#define VGT_WRITE_START(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_START(pdev, id), val)

void vgt_ring_start(struct vgt_ring_buffer *ring)
{
	struct pgt_device *pdev = ring->pdev;
	int id = RING_BUFFER_RCS;
	u32 head;

	//ASSERT(ring->space == ring_space(ring));

	vgt_ring_reset(ring);

	/* execute our ring */
	VGT_WRITE_CTL(pdev, id, 0);
	VGT_WRITE_HEAD(pdev, id, 0);
	VGT_WRITE_TAIL(pdev, id, 0);

	head = VGT_READ_HEAD(pdev, id);
	if (head != 0) {
		VGT_WRITE_HEAD(pdev, id, 0);
	}

	VGT_WRITE_START(pdev, id, ring->offset);
	VGT_WRITE_CTL(pdev, id, ((ring->size - PAGE_SIZE) & 0x1FF000) | 1);
	VGT_POST_READ_CTL(pdev, id);

	wait_for(((VGT_READ_CTL(pdev, id) & 1) != 0 &&
			VGT_READ_START(pdev, id) == ring->offset &&
			(VGT_READ_HEAD(pdev, id) & RB_HEAD_OFF_MASK) == 0), 50);
	vgt_dbg("start vgt ring at 0x%x\n", ring->offset);
}

void vgt_ring_advance(struct vgt_ring_buffer *ring)
{
	int id = RING_BUFFER_RCS;

	ring->tail &= ring->size - 1;
	VGT_WRITE_TAIL(ring->pdev, id, ring->tail);
}
