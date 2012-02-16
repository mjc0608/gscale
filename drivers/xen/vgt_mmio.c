
/*
 * vGT core module
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
#include <linux/list.h>
#include <xen/vgt.h>
#include "vgt_reg.h"

static inline int tail_ro_ring_id(unsigned int tail_off)
{
	int i;

	for (i=0; i< MAX_ENGINES; i++) {
		if ( ring_mmio_base[i] == tail_off )
			return i;
	}
	printk("Wrong tail register %s\n", __FUNCTION__);
	ASSERT(0);
	return 0;
}

bool ring_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;

	printk("vGT:ring_mmio_read (%x)\n", off);

	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_ro_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	memcpy(p_data, (char *)vring + rel_off, bytes);
	return true;
}

bool ring_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;
	vgt_reg_t	oval;

	printk("vGT:ring_mmio_write (%x) with val (%x)\n", off, *p_data);
	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_ro_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	oval = *(vgt_reg_t *)((char *)vring + rel_off);
	memcpy((char *)vring + rel_off, p_data, bytes);

	switch (rel_off & ~3) {
	case RB_OFFSET_TAIL:
		break;
	case RB_OFFSET_HEAD:
	case RB_OFFSET_START:
		break;
	case RB_OFFSET_CTL:
		/* Do we need to wait for the completion of current slice? */
		if ( (oval & _RING_CTL_ENABLE) &&
			!(vring->ctl & _RING_CTL_ENABLE) ) {
			vgt_deactive (vgt->pdev, &vgt->list);
		}
		else if ( !(oval & _RING_CTL_ENABLE) &&
			(vring->ctl & _RING_CTL_ENABLE) ) {
			/* enabled */
			vgt_active (vgt->pdev, &vgt->list);
		}
		if (vring->ctl & _RING_CTL_ENABLE) {
			/*
			 * Command scan policy:
			 * 	1) here: Challenge if that if guest modify
			 *   head/tail register to a new buffer and come back,
			 *   we don't know if a cmd is converted or not.
			 *	2) at submission time: Easier, but not
			 *   that efficient (GPU have to wait for the completion).
			 *	Start from 2, TO-REVISIT LATER!!!
			 */
		}
		break;
	}
	return true;
}

bool vgt_emulate_cfg_read(struct vgt_device *vgt, unsigned int offset, void *p_data, int bytes)
{

	ASSERT ((offset + bytes) <= VGT_CFG_SPACE_SZ);
	memcpy(p_data, &vgt->state.cfg_space[offset], bytes);

	/* TODO: hooks */
	offset &= ~3;
	switch (offset) {
		case 0:
		case 4:
		break;
		default:
		break;
	}
	return true;
}

bool vgt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, int bytes)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint32_t *cfg_reg, old, new, size;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
		ASSERT((bytes == 4) && (off & 3) == 0);

		old = *cfg_reg & 0xf;
		new = *(uint32_t *)p_data;
		printk("Programming bar %x with %x\n", off, new);
		size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
		if ( new == 0xFFFFFFFF )
			/*
			 * Power-up software can determine how much address
			 * space the device requires by writing a value of
			 * all 1's to the register and then reading the value
			 * back. The device will return 0's in all don't-care
			 * address bits.
			 */
			new = new & ~(size-1);
		*cfg_reg = (new & ~0xf) | old;
		break;

		case VGT_REG_CFG_SPACE_MSAC:
		printk("Guest write MSAC %x, %d: Not supported yet\n",
				*(char *)p_data, bytes);
		break;

		case VGT_REG_CFG_SPACE_BAR1+4:
		case VGT_REG_CFG_SPACE_BAR0+4:
		case VGT_REG_CFG_SPACE_BAR2+4:
		default:
		memcpy (&vgt->state.cfg_space[off], p_data, bytes);
		break;
	}
	/*
	 * FIXME: assume most dmo0's cfg writes should be propogated to
	 * the real conf space. In the case where propogation is required
	 * but value needs be changed (sReg), do it here
	 */
	return true;
}

bool vgt_initialize_mmio_hooks()
{
    int i;

printk("mmio hooks initialized\n");
	/* ring registers */
	for (i=0; i < MAX_ENGINES; i++)
		if (!vgt_register_mmio_handler(ring_mmio_base[i],
			ring_mmio_base[i] + RB_REGS_SIZE - 1,
			ring_mmio_read, ring_mmio_write))
			return false;

	return true;
}
