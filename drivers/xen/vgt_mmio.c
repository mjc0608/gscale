
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
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>

#include <xen/xen.h>
#include <xen/page.h>
#include <xen/events.h>
#include <xen/xen-ops.h>
#include <xen/interface/xen.h>
#include <xen/interface/hvm/hvm_op.h>
#include <xen/interface/hvm/params.h>
#include <xen/interface/hvm/ioreq.h>

#include <xen/vgt.h>
#include <xen/vgt-if.h>
#include <xen/vgt-parser.h>
#include "vgt_reg.h"

static bool bypass_scan = true;
static int __init bypass_scan_setup(char *str)
{
	bypass_scan = true;

	return 1;
}
__setup("bypass_scan", bypass_scan_setup);

static inline int tail_to_ring_id(unsigned int tail_off)
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

#if 0
static void ring_debug(struct vgt_device *vgt, int ring_id)
{
	printk("phead (%x), ptail(%x), pstart(%x), pctl(%x)\n",
		VGT_MMIO_READ(vgt->pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_TAIL(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_START(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_CTL(ring_id)));
}
#endif

bool gmbus_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;

	ASSERT(bytes <= 8 && !(offset & (bytes - 1)));
	//printk("vGT(%d): read gmbus register with offset%x and size %d.\n", vgt->vgt_id, offset, bytes);

	vgt_i2c_handle_gmbus_read(&vgt->vgt_i2c_bus, offset, p_data);

	return rc;
}

bool gmbus_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_edid_data_t **pedid = NULL;
	//printk("vGT(%d): write gmbus register with offset %x and size %d, value 0x%x.\n",
	//		vgt->vgt_id, offset, bytes, *((int *)p_data));

	if (offset == _REG_PCH_GMBUS0) {
		/* select the port */
		unsigned port = *(int *)p_data & 0x7;
		switch (port) {
		case 0: /* disabled. Be treated as reset */
			pedid = NULL;
			break;
		case 1: /* LCTRCLK */
			/* Is there really the case? If so, need to handle.*/
			printk("vGT(%d): WARNING: Accessing LCTRCLK which is not supported!\n",
					vgt->vgt_id);
			BUG();
			break;
		case 2: /* Analog Mon */
			pedid = (vgt_edid_data_t **)
					&vgt->vgt_edids[EDID_VGA];
			break;
		case 3: /* LVDS */
			pedid = (vgt_edid_data_t **)
					&vgt->vgt_edids[EDID_LVDS];
			break;
		case 4: /* Port C*/
			pedid = (vgt_edid_data_t **)
					&vgt->vgt_edids[EDID_HDMIC];
			break;
		case 5: /* Should not happen. */
			ASSERT(0);
			break;
		case 6: /* Port D */
			pedid = (vgt_edid_data_t **)
					&vgt->vgt_edids[EDID_HDMID];
			break;
		case 7: /* Reserved */
			/*
			 * Well, it actually happened...
			 */
#if 0
			ASSERT(0);
#else
			printk("vGT(%d): WARNING: GMBUS accessing reserved port!!!!\n", vgt->vgt_id);
			return true;
#endif
			break;
		default:
			break;
		}
		vgt_init_i2c_bus(&vgt->vgt_i2c_bus);
		vgt->vgt_i2c_bus.state = VGT_I2C_SEND;
		vgt->vgt_i2c_bus.gmbus.pedid = pedid;
	} else {
		vgt_i2c_handle_gmbus_write (&vgt->vgt_i2c_bus,
						offset, p_data);
		pedid = vgt->vgt_i2c_bus.gmbus.pedid;
	}
	return rc;
}

#ifdef ENABLE_GPIO_EMULATION
bool gpio_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_edid_data_t **pedid;

	ASSERT(bytes == 4 && !(offset & (bytes - 1)));

	// printk("[I2C_EDID] gpio_mmio_read( offset:0x%x, bytes:%d).\n", offset, bytes);
	if (offset == _REG_PCH_GPIOA) {
		//printk("vGT(%d): Reading GPIO_A.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_VGA];
	} else if (offset == _REG_PCH_GPIOC) {
		//printk("vGT(%d): Reading GPIO_C.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_LVDS];
	} else if (offset == _REG_PCH_GPIOD) {
		//printk("vGT(%d): Reading GPIO_D.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMIC];
	} else if (offset == _REG_PCH_GPIOE) {
		//printk("vGT(%d): Reading GPIO_E.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMIB];
	} else if (offset == _REG_PCH_GPIOF) {
		//printk("vGT(%d): Reading GPIO_F.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMID];
	} else {
		// not supported yet GPIO access
		printk("vGT(%d): Not supported GPIO access! \n", vgt->vgt_id);
		dump_stack();
		ASSERT (0);
		return false;
	}

	vgt_i2c_handle_gpio_read(&vgt->vgt_i2c_bus, pedid, p_data);

	return rc;
}

bool gpio_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_edid_data_t **pedid;

	ASSERT(bytes == 4 && !(offset & (bytes - 1)));

	//printk("[I2C_EDID] gpio_mmio_write( offset:0x%x, bytes:%d, data:0x%x).\n", offset, bytes, *(unsigned int *)p_data);
	if (offset == _REG_PCH_GPIOA) {
		//printk("vGT(%d): Writing GPIO_A.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_VGA];
	} else if (offset == _REG_PCH_GPIOC) {
		//printk("vGT(%d): Writing GPIO_C.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_LVDS];
	} else if (offset == _REG_PCH_GPIOD) {
		//printk("vGT(%d): Writing GPIO_D.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMIC];
	} else if (offset == _REG_PCH_GPIOE) {
		//printk("vGT(%d): Writing GPIO_E.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMIB];
	} else if (offset == _REG_PCH_GPIOF) {
		//printk("vGT(%d): Writing GPIO_F.\n", vgt->vgt_id);
		pedid = (vgt_edid_data_t **)&vgt->vgt_edids[EDID_HDMID];
	} else {
		// not supported yet GPIO access
		printk("vGT(%d): Not supported GPIO access! \n", vgt->vgt_id);
		dump_stack();
		ASSERT (0);
		return false;
	}

	vgt_i2c_handle_gpio_write(&vgt->vgt_i2c_bus, pedid, p_data);

	return rc;
}
#endif /* ENABLE_GPIO_EMULATION */

bool fence_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int id;
	ASSERT(bytes <= 8 && !(off & (bytes - 1)));
	id = (off - _REG_FENCE_0_LOW) >> 3;

	if (id >= vgt->fence_sz) {
		printk("vGT(%d) , read fence register %x,"
			" %x out of assignment %x.\n", vgt->vgt_id,
			off, id, vgt->fence_sz);
	}
	memcpy (p_data, (char *)vgt->state.vReg + off, bytes);
	return true;
}

bool fence_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int id;
	ASSERT(bytes <= 8 && !(off & (bytes - 1)));
	id = (off - _REG_FENCE_0_LOW) >> 3;

	if (id >= vgt->fence_sz) {
		printk("vGT (%d) , write fence register %x,"
			" %x out of assignment %x.\n", vgt->vgt_id,
			off, id, vgt->fence_sz);
	}
	else {
		memcpy ((char *)vgt->state.vReg + off, p_data, bytes);
		memcpy ((char *)vgt->state.sReg + off, p_data, bytes);
		/* TODO: Check address space */

		/* FENCE registers are physically assigned, update! */
		if (bytes < 8)
			VGT_MMIO_WRITE(vgt->pdev, off + vgt->fence_base * 8,
				__sreg(vgt, off));
		else
			VGT_MMIO_WRITE_BYTES(vgt->pdev, off + vgt->fence_base * 8,
				__sreg64(vgt, off), 8);
	}
	return true;
}

bool ring_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;

	ASSERT(bytes <= 4 && !(off & (bytes - 1)));
	//printk("vGT:ring_mmio_read (%x)\n", off);

	if (hvm_render_owner && (vgt->vm_id != 0) ){
		unsigned long data;
		data = VGT_MMIO_READ_BYTES(vgt->pdev, off, bytes);
		memcpy(p_data, &data, bytes);
		return true;
	}

	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_to_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	memcpy(p_data, (char *)vring + rel_off, bytes);
	//ring_debug(vgt, ring_id);
	return true;
}

bool ring_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;
	vgt_ringbuffer_t	*sring;
	vgt_reg_t	oval;

	ASSERT(bytes <= 4);
	dprintk("vGT:ring_mmio_write (0x%x) with val (0x%x)\n", off, *((u32 *)p_data));
	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ASSERT(!(rel_off & (bytes - 1)));

	ring_id = tail_to_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;
	sring = &vgt->rb[ring_id].sring;

	oval = *(vgt_reg_t *)((char *)vring + rel_off);
	memcpy((char *)vring + rel_off, p_data, bytes);

	switch (rel_off) {
	case RB_OFFSET_TAIL:
		sring->tail = vring->tail;
		if ( !bypass_scan )
			vgt_scan_vring(vgt, ring_id);
		break;
	case RB_OFFSET_HEAD:
		//debug
		//vring->head |= 0x200000;
		sring->head = vring->head;
		break;
	case RB_OFFSET_START:
		sring->start = mmio_g2h_gmadr(vgt, off, vring->start);
		break;
	case RB_OFFSET_CTL:
		sring->ctl = vring->ctl;

		/* TODO: need lock with kthread */
		/* Do we need to wait for the completion of current slice? */
		if ( (oval & _RING_CTL_ENABLE) &&
			!(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: deactivate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_deactive (vgt->pdev, &vgt->list);
		}
		else if ( !(oval & _RING_CTL_ENABLE) &&
			(vring->ctl & _RING_CTL_ENABLE) ) {
			/* enabled */
			printk("vGT: activate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_active (vgt->pdev, &vgt->list);
		}
		if (!bypass_scan && (vring->ctl & _RING_CTL_ENABLE)) {
			vgt->last_scan_head[ring_id] =
				vring->head & RB_HEAD_OFF_MASK;
			vgt_scan_vring(vgt, ring_id);
		}
		break;
	default:
		ASSERT(0);
		break;
	}

	/* TODO: lock with kthread? */
	/*
	 * FIXME: Linux VM doesn't read head register directly. Instead it relies on
	 * automatic head reporting mechanism. Later with command parser, there's no
	 * problem since all commands are translated and filled by command parser. for
	 * now it's possible for dom0 to fill over than a full ring in a scheduled
	 * quantum
	 */
	if (reg_hw_access(vgt, off))
		VGT_MMIO_WRITE(vgt->pdev, off, *(vgt_reg_t*)((char *)sring + rel_off));
	//ring_debug(vgt, ring_id);
	return true;
}

static inline void set_vRC(struct vgt_device *vgt, int c)
{
	__vreg(vgt, _REG_GT_CORE_STATUS) = c;
	__vreg(vgt, _REG_GT_THREAD_STATUS) = c;
}

static void set_vRC_to_C6(struct vgt_device *vgt)
{
	dprintk("Virtual Render C state set to C6\n");
	set_vRC(vgt, 3);
}

static void set_vRC_to_C0(struct vgt_device *vgt)
{
	dprintk("Virtual Render C state set to C0\n");
	set_vRC(vgt, 0);
}

bool force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	if (bytes > 4){
		printk("invalid force wake data\n");
		return false;
	}

	data = (*(uint32_t*) p_data) & 1 ;

	dprintk("VM%d write register FORCE_WAKE with %x\n", vgt->vm_id, data);

	__vreg(vgt, _REG_FORCEWAKE_ACK) = data;
	__vreg(vgt, _REG_FORCEWAKE) = data;
	if (data == 1)
		set_vRC_to_C0(vgt);
	else
		set_vRC_to_C6(vgt);

	return true;
}

bool rc_state_ctrl_1_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	ASSERT(bytes == 4);

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_1 with 0x%x\n", vgt->vm_id, data);

	__vreg(vgt, _REG_RC_STATE_CTRL_1) = data;
	if ( (data & _REGBIT_RC_HW_CTRL_ENABLE) && (data & (_REGBIT_RC_RC6_ENABLE
					| _REGBIT_RC_DEEPEST_RC6_ENABLE	| _REGBIT_RC_DEEP_RC6_ENABLE) ) )
		set_vRC_to_C6(vgt);
	else
		set_vRC_to_C0(vgt);

	return true;
}

bool rc_state_ctrl_2_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	ASSERT(bytes == 4);

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_2 with 0x%x\n", vgt->vm_id, data);

	__vreg(vgt, _REG_RC_STATE_CTRL_2) = data;

	/* bits 16:18 */
	data = (data >> 16) & 7;

	if ( data >= 4)
		set_vRC_to_C6(vgt);
	else
		set_vRC_to_C0(vgt);

	return true;
}

bool gen6_gdrst_mmio_write(struct vgt_device *vgt, unsigned int offset,
		 void *p_data, unsigned int bytes)
{
	uint32_t data;
	int i;

	ASSERT(bytes <=4);

	data = 0;
	memcpy(&data, p_data, bytes);

	if (data & _REGBIT_GEN6_GRDOM_FULL){
		printk("VM%d request Full GPU Reset\n", vgt->vm_id);
	}

	if (data & _REGBIT_GEN6_GRDOM_RENDER){
		printk("VM%d request GPU Render Reset\n", vgt->vm_id);
	}

	if (data & _REGBIT_GEN6_GRDOM_MEDIA){
		printk("VM%d request GPU Media Reset\n", vgt->vm_id);
	}

	if (data & _REGBIT_GEN6_GRDOM_BLT){
		printk("VM%d request GPU BLT Reset\n", vgt->vm_id);
	}
	/* TODO: add appropriate action */
	/* so far, we just simply ignore it and VM treat it as success */

	for (i = 0; i < MAX_ENGINES; i++) {
		show_debug(vgt->pdev, i);
		show_ringbuffer(vgt->pdev, i, 16 * sizeof(vgt_reg_t));
	}
	return true;
}

bool pch_pp_control_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;
	uint32_t reg;
	union PCH_PP_CONTROL pp_control;
	union PCH_PP_STAUTS pp_status;

	ASSERT(bytes == 4);

	reg = offset & ~(bytes - 1);
	if (reg_hw_access(vgt, reg)){
		return default_mmio_write(vgt, offset, p_data, bytes);
	}

	data = *(uint32_t*)p_data;

	__vreg(vgt, _REG_PCH_PP_CONTROL) = data;

	pp_control.data = data;
	pp_status.data = __vreg(vgt, _REG_PCH_PP_STATUS);
	if (pp_control.power_state_target == 1){
		/* power on panel */
		pp_status.panel_powere_on_statue = 1;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	} else {
		/* power down panel */
		pp_status.panel_powere_on_statue = 0;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	}
	__vreg(vgt, _REG_PCH_PP_STATUS) = pp_status.data;

	return true;
}

bool transaconf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t reg;
	union _TRANS_CONFIG config;

	ASSERT(bytes == 4);

	reg = offset & ~(bytes - 1);
	if (reg_hw_access(vgt, reg)){
		return default_mmio_write(vgt, offset, p_data, bytes);
	}

	config.data = *(uint32_t*)p_data;
	/* transcoder state should synced with enable */
	config.transcoder_state = config.transcoder_enable;

	__vreg(vgt, reg) = config.data;

	return true;
}

/* Pipe Frame Count */
bool pipe_frmcount_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_passthrough_mmio_read(vgt, offset, p_data, bytes);
}

/* Pipe Display Scan Line*/
bool pipe_dsl_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_passthrough_mmio_read(vgt, offset, p_data, bytes);
}

bool hdcp_status_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	printk("VM%d read HDCP status register 0x%x\n", vgt->vm_id, offset);

	*(uint32_t*)p_data = _REGBIT_HDCP_CIPHER_AN_READY;

	return true;
}

bool hdcp_key_status_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	printk("VM%d read HDCP KEY status register 0x%x\n", vgt->vm_id, offset);

	*(uint32_t*)p_data = _REGBIT_HDCP_KEY_DONE;

	return true;
}

bool hdcp_pch_boot_auth_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	printk("VM%d read HDCP PCH Boot Authentication Status Register 0x%x\n", vgt->vm_id, offset);

	*(uint32_t*)p_data = _REGBIT_HDCP_PCH_BOOT_AUTH_STATUS_READY;

	return true;
}

/* XXX assume all rings use same PPGTT table, so try to initialize once
 * all bases are set.
 */
void vgt_try_setup_ppgtt(struct vgt_device *vgt)
{
	int ring;
	u32 base;

	for (ring = 0; ring < 3; ring++) {
		if (!vgt->rb[ring].has_ppgtt_base_set)
			return;
	}

	base = vgt->rb[0].vring_ppgtt_info.base;
	for (ring = 1; ring < 3; ring++) {
		if (vgt->rb[ring].vring_ppgtt_info.base != base) {
			dprintk("zhen: different PPGTT base is set!\n");
			vgt->pdev->enable_ppgtt = 0;
			return;
		}
	}

	dprintk("zhen: all rings are set PPGTT base and use single table!\n");
	vgt->need_ppgtt_setup = true;
	vgt_raise_request(vgt->pdev, VGT_REQUEST_PPGTT_INIT);
}

static int ring_pp_dir_base_write(struct vgt_device *vgt, int ring_id, u32 off, u32 base)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &vgt->rb[ring_id].sring_ppgtt_info;

	/* convert base which is in form of bit 31-16 in 64bytes cachelines,
	 * it turns out to be ((((base >> 16) * 64) >> 2) << PAGE_SHIFT), which
	 * is just base. */
	v_info->base = base;
	s_info->base = mmio_g2h_gmadr(vgt, off, v_info->base);

	if (reg_hw_access(vgt, off)) {
		dprintk("PP_DIR_BASE off 0x%x write 0x%x\n", off, s_info->base);
		VGT_MMIO_WRITE(vgt->pdev, off, s_info->base);
	}

	vgt->rb[ring_id].has_ppgtt_base_set = 1;

	vgt_try_setup_ppgtt(vgt);

	return 0;
}

bool rcs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			  void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_RCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;

	dprintk("RCS_PP_DIR_BASE read: 0x%x\n", v_info->base);

	return true;
}

bool rcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			   void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("RCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_RCS, off, base);
	return true;
}

bool bcs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			  void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_BCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;
	dprintk("BCS_PP_DIR_BASE read: 0x%x\n", v_info->base);
	return true;
}

bool bcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			   void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("BCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_BCS, off, base);
	return true;
}

bool vcs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			  void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;
	dprintk("VCS_PP_DIR_BASE read: 0x%x\n", v_info->base);
	return true;
}

bool vcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			   void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("VCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_VCS, off, base);
	return true;
}

bool rcs_gfx_mode_read(struct vgt_device *vgt, unsigned int off,
		       void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_RCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	dprintk("RCS_GFX_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool bcs_blt_mode_read(struct vgt_device *vgt, unsigned int off,
		       void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_BCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	dprintk("BCS_BLT_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool vcs_mfx_mode_read(struct vgt_device *vgt, unsigned int off,
		       void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	dprintk("VCS_MFX_MODE read: 0x%x\n", v_info->mode);
	return true;
}

int ring_ppgtt_mode(struct vgt_device *vgt, int ring_id, u32 off, u32 mode)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &vgt->rb[ring_id].sring_ppgtt_info;

	v_info->mode = mode;
	s_info->mode = mode;

	if (reg_hw_access(vgt, off)) {
		dprintk("RING mode: off 0x%x write 0x%x\n", off, s_info->mode);
		VGT_MMIO_WRITE(vgt->pdev, off, s_info->mode);
	}

	/* sanity check */
	if ((mode & _REGBIT_PPGTT_ENABLE) && (mode & (_REGBIT_PPGTT_ENABLE << 16))) {
		printk("PPGTT enabling on ring %d\n", ring_id);
		/* XXX the order of mode enable for PPGTT and PPGTT dir base
		 * setting is not strictly defined, e.g linux driver first
		 * enables PPGTT bit in mode reg, then write PP dir base...
		 */
		vgt->rb[ring_id].has_ppgtt_mode_enabled = 1;
	}

	return 0;
}

bool rcs_gfx_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("RCS_GFX_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_RCS, off, mode);

	return true;
}

bool bcs_blt_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("BCS_BLT_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_BCS, off, mode);

	return true;
}

bool vcs_mfx_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	dprintk("VCS_MFX_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_VCS, off, mode);

	return true;
}

/* FIXME: add EDID virtualization in the future
 */
bool dp_aux_ch_ctl_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_edid_data_t **pedid = NULL;
	VGT_DP_PORTS_IDX port_idx;

	ASSERT(bytes == 4);
	ASSERT((offset & (bytes - 1)) == 0);

#ifdef AUX_CH_WORKAROUND
	/*TODO
	 *
	 * Old logic to simply do the default_mmio access. This is kept
	 * to maintain the system behavior unchanged. It will be chagned
	 * after aux_ch virtualization has been completely supported.
	 */
	rc = default_mmio_read(vgt, offset, p_data, bytes);
#endif /* AUX_CH_WORKAROUND */

	port_idx = vgt_get_dp_port_idx(offset);
	switch (port_idx) {
	case VGT_DPB_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPB];
		break;
	case VGT_DPC_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPC];
		break;
	case VGT_DPD_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPD];
		break;
	default:
		printk("vGT(%d): WARNING: Unsupported DP port [0x%x]access!\n",
				vgt->vgt_id, offset);
		BUG();
		break;
	}

	vgt_i2c_handle_aux_ch_read(&vgt->vgt_i2c_bus, pedid,
				offset, port_idx, p_data);

	return rc;
}

bool pipe_conf_mmio_write(struct vgt_device *vgt, unsigned int offset,
        void *p_data, unsigned int bytes)
{
    unsigned int reg;
    uint32_t wr_data;

    ASSERT(bytes == 4);

    reg = offset & ~(bytes - 1);

    wr_data = *((uint32_t *)p_data);
    /* vreg status will be updated when when read hardware status */
    if (!reg_hw_access(vgt, reg)) {
        if (wr_data & _REGBIT_PIPE_ENABLE)
            wr_data |= _REGBIT_PIPE_STAT_ENABLED;
        else if (!(wr_data & _REGBIT_PIPE_ENABLE))
            wr_data &= ~_REGBIT_PIPE_STAT_ENABLED;
    }

    /* FIXME: this will cause writing to bit _REGBIT_PIPE_STAT_ENABLED,
     * this bit indicate actual pipe state, but in prm, not marked
     * readonly bit
     */
    return default_mmio_write(vgt, offset, &wr_data, bytes);
}

bool fdi_rx_iir_mmio_write(struct vgt_device *vgt, unsigned int offset,
    void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t wr_data, old_iir;
	bool rc;

    ASSERT(bytes == 4 && !(offset & (bytes - 1)));
    reg = offset & ~(bytes -1);

    wr_data = *(vgt_reg_t *)p_data;
    old_iir = __vreg(vgt, reg);

    rc = default_mmio_write(vgt, offset, p_data, bytes);

    /* FIXME: sreg will be updated only when reading hardware status happened,
     * so when dumping sreg space, the "hardware status" related bits may not
     * be trusted */
    if (!reg_hw_access(vgt, reg))
        __vreg(vgt, reg) = old_iir ^ wr_data;

    return rc;
}



#define FDI_LINK_TRAIN_PATTERN_1    0
#define FDI_LINK_TRAIN_PATTERN_2    1
/* FIXME: this function is highly platform-dependent (SNB + CPT) */
static bool check_fdi_rx_train_status(struct vgt_device *vgt, enum vgt_pipe pipe, unsigned int train_pattern)
{
    unsigned int fdi_rx_imr, fdi_tx_ctl, fdi_rx_ctl;
    unsigned int fdi_rx_check_bits, fdi_tx_check_bits, fdi_rx_train_bits, fdi_tx_train_bits, fdi_iir_check_bits;
    switch (pipe) {
        case PIPE_A:
            fdi_rx_imr = _REG_FDI_RXA_IMR;
            fdi_tx_ctl = _REG_FDI_TXA_CTL;
            fdi_rx_ctl = _REG_FDI_RXA_CTL;
            break;
        case PIPE_B:
            fdi_rx_imr = _REG_FDI_RXB_IMR;
            fdi_tx_ctl = _REG_FDI_TXB_CTL;
            fdi_rx_ctl = _REG_FDI_RXB_CTL;
            break;
        default: BUG();
    };

    switch (train_pattern) {
        case FDI_LINK_TRAIN_PATTERN_1:
            fdi_rx_train_bits =_REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT;
            fdi_tx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_1;
            fdi_iir_check_bits = _REGBIT_FDI_RX_BIT_LOCK;
            break;
        case FDI_LINK_TRAIN_PATTERN_2:
            fdi_rx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_2_CPT;
            fdi_tx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_2;
            fdi_iir_check_bits = _REGBIT_FDI_RX_SYMBOL_LOCK;
            break;
        default: BUG();
    }

    fdi_rx_check_bits = _REGBIT_FDI_RX_ENABLE
        | fdi_rx_train_bits;
    fdi_tx_check_bits = _REGBIT_FDI_TX_ENABLE
        | fdi_tx_train_bits;

    /* If imr bit not been masked */
    if (((__vreg(vgt, fdi_rx_imr) & fdi_iir_check_bits) == 0 )
            && ((__vreg(vgt, fdi_tx_ctl) & fdi_tx_check_bits) == fdi_tx_check_bits)
            && ((__vreg(vgt, fdi_rx_ctl) & fdi_rx_check_bits) == fdi_rx_check_bits))
        return true;
    else
        return false;
}

bool update_fdi_rx_iir_status(struct vgt_device *vgt, unsigned int offset,
    void *p_data, unsigned int bytes)
{
    enum vgt_pipe pipe;
    unsigned int reg, fdi_rx_iir;
    bool rc;

    ASSERT(bytes == 4 && (offset & 0x3) == 0);

    reg = offset & ~(bytes - 1);

    switch (offset) {
        case _REG_FDI_RXA_CTL:
        case _REG_FDI_TXA_CTL:
        case _REG_FDI_RXA_IMR:
            pipe = PIPE_A;
            break;
        case _REG_FDI_RXB_CTL:
        case _REG_FDI_TXB_CTL:
        case _REG_FDI_RXB_IMR:
            pipe = PIPE_B;
            break;
        default:
            BUG();
    }

    switch (pipe) {
        case PIPE_A:
            fdi_rx_iir = _REG_FDI_RXA_IIR;
            break;
        case PIPE_B:
            fdi_rx_iir = _REG_FDI_RXB_IIR;
            break;
        default:
            BUG();
    }

    rc = default_mmio_write(vgt, offset, p_data, bytes);
    if (!reg_hw_access(vgt, reg)) {
        if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN_1))
            __vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_BIT_LOCK;
        if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN_2))
            __vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_SYMBOL_LOCK;
    }
    return rc;
}

bool pch_adpa_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);
	reg = offset & ~(bytes - 1);

	rc = default_mmio_read(vgt, offset, p_data, bytes);
	if (reg_hw_access(vgt, reg)) {
		reg_data = *(vgt_reg_t *)p_data;
		if (reg_data & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK)
			set_bit(VGT_CRT, pdev->port_detect_status);
		else
			clear_bit(VGT_CRT, pdev->port_detect_status);
	}

	return rc;
}

bool pch_adpa_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t wr_data, vreg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);

	reg = offset & ~(bytes - 1);
	wr_data = *(vgt_reg_t *)p_data;

	/* FIXME: suppose the CRT has been plugged in
	 * Actually we need dom0 to tell hvm such info
	 */
	rc = default_mmio_write(vgt, offset, p_data, bytes);
	vreg_data = wr_data;
	/* Tell VM if CRT monitor physically plugged in */
	if (test_bit(VGT_CRT, pdev->port_detect_status)) {
		/* Emulation: trap hotplug force trigger */
		if ((wr_data & _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER) && !(wr_data & _REGBIT_ADPA_DAC_ENABLE)) {
			/* clear the force trigger and set read only bits */
			vreg_data &= ~_REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER;
		}
		/* FIXME: sometimes only one channel is OK ??? (blue and green channel) */
		vreg_data |= _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;

	} else
		vreg_data &= ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;

	__vreg(vgt, reg) = vreg_data;

	return rc;
}


bool dp_ctl_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);
	reg = offset & ~(bytes - 1);

	rc = default_mmio_read(vgt, offset, p_data, bytes);
	if (reg_hw_access(vgt, reg)) {
		reg_data = *(vgt_reg_t *)p_data;
		if (reg_data & _REGBIT_DP_PORT_DETECTED) {
			switch (reg) {
				case _REG_DP_A_CTL:
					set_bit(VGT_DP_A, pdev->port_detect_status);
					break;
				case _REG_DP_B_CTL:
					set_bit(VGT_DP_B, pdev->port_detect_status);
					break;
				case _REG_DP_C_CTL:
					set_bit(VGT_DP_C, pdev->port_detect_status);
					break;
				case _REG_DP_D_CTL:
					set_bit(VGT_DP_D, pdev->port_detect_status);
					break;
				default:
					BUG();
			}
			vgt_set_all_vreg_bit(pdev, _REGBIT_DP_PORT_DETECTED, reg);
		} else {
			switch (reg) {
				case _REG_DP_A_CTL:
					clear_bit(VGT_DP_A, pdev->port_detect_status);
					break;
				case _REG_DP_B_CTL:
					clear_bit(VGT_DP_B, pdev->port_detect_status);
					break;
				case _REG_DP_C_CTL:
					clear_bit(VGT_DP_C, pdev->port_detect_status);
					break;
				case _REG_DP_D_CTL:
					clear_bit(VGT_DP_D, pdev->port_detect_status);
					break;
				default:
					BUG();
			}
			vgt_clear_all_vreg_bit(pdev, _REGBIT_DP_PORT_DETECTED, reg);
		}
	}

	return rc;
}

bool dp_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t vreg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);

	reg = offset & ~(bytes - 1);
	vreg_data = *(vgt_reg_t *)p_data;

	rc = default_mmio_write(vgt, offset, p_data, bytes);
	/* Read only bit should be keeped coherent with intended hardware status */
	switch (reg) {
		case _REG_DP_A_CTL:
			if (test_bit(VGT_DP_A, pdev->port_detect_status))
				vreg_data |= _REGBIT_DP_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_DP_PORT_DETECTED;
			break;
		case _REG_DP_B_CTL:
			if (test_bit(VGT_DP_B, pdev->port_detect_status))
				vreg_data |= _REGBIT_DP_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_DP_PORT_DETECTED;
			break;
		case _REG_DP_C_CTL:
			if (test_bit(VGT_DP_C, pdev->port_detect_status))
				vreg_data |= _REGBIT_DP_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_DP_PORT_DETECTED;
			break;
		case _REG_DP_D_CTL:
			if (test_bit(VGT_DP_D, pdev->port_detect_status))
				vreg_data |= _REGBIT_DP_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_DP_PORT_DETECTED;
			break;
		default:
			BUG();
	}

	__vreg(vgt, reg) = vreg_data;

	return rc;
}

bool hdmi_ctl_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);
	reg = offset & ~(bytes - 1);

	rc = default_mmio_read(vgt, offset, p_data, bytes);
	if (reg_hw_access(vgt, reg)) {
		reg_data = *(vgt_reg_t *)p_data;
		if (reg_data & _REGBIT_HDMI_PORT_DETECTED) {
			switch (reg) {
				case _REG_HDMI_B_CTL:
					set_bit(VGT_HDMI_B, pdev->port_detect_status);
					break;
				case _REG_HDMI_C_CTL:
					set_bit(VGT_HDMI_C, pdev->port_detect_status);
					break;
				case _REG_HDMI_D_CTL:
					set_bit(VGT_HDMI_D, pdev->port_detect_status);
					break;
				default:
					BUG();
			}
			vgt_set_all_vreg_bit(pdev, _REGBIT_HDMI_PORT_DETECTED, reg);
		} else {
			switch (reg) {
				case _REG_HDMI_B_CTL:
					clear_bit(VGT_HDMI_B, pdev->port_detect_status);
					break;
				case _REG_HDMI_C_CTL:
					clear_bit(VGT_HDMI_C, pdev->port_detect_status);
					break;
				case _REG_HDMI_D_CTL:
					clear_bit(VGT_HDMI_D, pdev->port_detect_status);
					break;
				default:
					BUG();
			}
			vgt_clear_all_vreg_bit(pdev, _REGBIT_HDMI_PORT_DETECTED, reg);
		}
	}

	return rc;
}

bool hdmi_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t vreg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);

	reg = offset & ~(bytes - 1);
	vreg_data = *(vgt_reg_t *)p_data;

	rc = default_mmio_write(vgt, offset, p_data, bytes);
	/* Read only bit should be keeped coherent with intended hardware status */
	switch (reg) {
		case _REG_HDMI_B_CTL:
			if (test_bit(VGT_HDMI_B, pdev->port_detect_status))
				vreg_data |= _REGBIT_HDMI_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_HDMI_PORT_DETECTED;
			break;
		case _REG_HDMI_C_CTL:
			if (test_bit(VGT_HDMI_C, pdev->port_detect_status))
				vreg_data |= _REGBIT_HDMI_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_HDMI_PORT_DETECTED;
			break;
		case _REG_HDMI_D_CTL:
			if (test_bit(VGT_HDMI_D, pdev->port_detect_status))
				vreg_data |= _REGBIT_HDMI_PORT_DETECTED;
			else
				vreg_data &= ~_REGBIT_HDMI_PORT_DETECTED;
			break;
		default:
			BUG();
	}

	__vreg(vgt, reg) = vreg_data;

	return rc;
}

#ifdef VGT_DEBUGFS_DUMP_FB
bool dspsurf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc;
	struct pgt_device *pdev = vgt->pdev;
	struct work_struct *work = &vgt->fb_debugfs_work;

	ASSERT(bytes == 4 && (offset & 0x3) == 0)

	rc = default_mmio_write(vgt, offset, p_data, bytes);
	if (rc == false)
		return rc;

	ASSERT(work);
	if (offset == _REG_DSPASURF)
		surf_used_pipe = PIPE_A;
	else if (offset == _REG_DSPBSURF)
		surf_used_pipe = PIPE_B;
	else /* PIPE_C ? */
		BUG();

	if (0 == queue_work(pdev->pgt_wq, &vgt->fb_debugfs_work))
		dprintk("vGT: failed to submit aready attached work!\n");

	return true;
}
#endif

bool dp_aux_ch_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg = 0;
	uint32_t data;
	bool rc;
	vgt_edid_data_t **pedid = NULL;
	VGT_DP_PORTS_IDX port_idx = vgt_get_dp_port_idx(offset);

	ASSERT(bytes == 4);
	ASSERT((offset & (bytes - 1)) == 0);

#ifdef AUX_CH_WORKAROUND
	/* TODO
	 * It is the old logic just to make the i915 driver init not being
	 * blocked by aux_ch. More decent handling is needed.
	 *
	 * Currently the code is kept for incremental code change. When adding
	 * EDID support, we do not want other things through AUX_CH is
	 * destroyed. AUX_CH supports two modes. One is native mode and
	 * another is I2C-over-AUX_CH. We do full virtualization for the latter
	 * case and seems that the EDID access is the only user of this.
	 *
	 * In future, the native mode of AUX_CH access will be handled. Then
	 * the trick below is not needed.
	 */
	reg = offset & ~(bytes -1);

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if ( !reg_hw_access(vgt, reg) &&
             ((reg == _REG_PCH_DPB_AUX_CH_CTL) ||
	      (reg == _REG_PCH_DPC_AUX_CH_CTL) ||
	      (reg == _REG_PCH_DPD_AUX_CH_CTL))) {
		data = __vreg(vgt, reg);
		if (data & _REGBIT_DP_AUX_CH_CTL_DONE)
			data &= ~_REGBIT_DP_AUX_CH_CTL_DONE;
		if (data & _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR)
			data &= ~_REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR;
		if (data & _REGBIT_DP_AUX_CH_CTL_RECV_ERR)
			data &= ~_REGBIT_DP_AUX_CH_CTL_RECV_ERR;
		if (data & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY){
			data |= _REGBIT_DP_AUX_CH_CTL_DONE;
			data &= ~_REGBIT_DP_AUX_CH_CTL_SEND_BUSY;
		}
		__vreg(vgt, reg) = data;
	}
#endif /* AUX_CH_WORKAROUND */
	switch (port_idx) {
	case VGT_DPB_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPB];
		break;
	case VGT_DPC_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPC];
		break;
	case VGT_DPD_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[EDID_DPD];
		break;
	default:
		printk("vGT(%d): WARNING: Unsupported DP port access!\n",
				vgt->vgt_id);
		BUG();
		break;
	}
	vgt_i2c_handle_aux_ch_write(&vgt->vgt_i2c_bus, pedid,
				offset, port_idx, p_data);
	return true;
}

int vgt_hvm_enable (struct vgt_device *vgt)
{
	struct xen_hvm_vgt_enable vgt_enable;
	int rc;

	vgt_enable.domid = vgt->vm_id;

	rc = HYPERVISOR_hvm_op(HVMOP_vgt_enable, &vgt_enable);
	if (rc != 0)
		printk(KERN_ERR "Enable HVM vgt fail with %d!\n", rc);

	return rc;
}

static int vgt_hvm_map_rom (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t gfn_s, num;
	struct xen_hvm_vgt_map_mmio memmap;
	int r, i;

	/* guarantee the sequence of map -> unmap -> map -> unmap */
	if (map == vgt->state.bar_mapped[3])
		return 0;

	cfg_space += VGT_REG_CFG_SPACE_BAR_ROM;
	gfn_s = (* (uint32_t*) cfg_space) >> PAGE_SHIFT;
	num = vgt->state.bar_size[3] >> PAGE_SHIFT;

	if (gfn_s == 0) {
		printk("vGT: map ROM bar to GFN ZERO!!!! exit!\n");
		return 0;
	}

	num = 1;
	for (i = 0; i < num; i++) {
		memmap.first_gfn = gfn_s + i;
		memmap.first_mfn = pfn_to_mfn(page_to_pfn(vgt->pdev->vbios + i));
		memmap.nr_mfns = 1;
		memmap.map = map;
		memmap.domid = vgt->vm_id;

		printk("%s(rombar): domid=%d gfn_s=0x%llx mfn_s=0x%llx nr_mfns=0x%x\n", map==0? "remove_map":"add_map",
				vgt->vm_id, memmap.first_gfn, memmap.first_mfn, memmap.nr_mfns);

		r = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);

		if (r != 0)
			printk(KERN_ERR "vgt_hvm_map_rom fail with %d!\n", r);
	}

	vgt->state.bar_mapped[3] = map;
	return r;
}

static int vgt_hvm_map_opregion (struct vgt_device *vgt, int map)
{
	uint32_t opregion;
	struct xen_hvm_vgt_map_mmio memmap;
	int rc;

	opregion = vgt->opregion_pa;

	printk("Direct map OpRegion 0x%x\n", opregion);

	memmap.first_gfn = opregion >> PAGE_SHIFT;
	memmap.first_mfn = opregion >> PAGE_SHIFT;
	memmap.nr_mfns =  VGT_OPREGION_PAGES;
	memmap.map = map;
	memmap.domid = vgt->vm_id;
	rc = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);
	if (rc != 0)
		printk(KERN_ERR "vgt_hvm_map_opregion fail with %d!\n", rc);

	return rc;
}

/*
 * Map the apperture space (BAR1) of vGT device for direct access.
 */
static int vgt_hvm_map_apperture (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s;
	struct xen_hvm_vgt_map_mmio memmap;
	int r;

	/* guarantee the sequence of map -> unmap -> map -> unmap */
	if (map == vgt->state.bar_mapped[1])
		return 0;

	cfg_space += VGT_REG_CFG_SPACE_BAR1;	/* APERTUR */
	if (VGT_GET_BITS(*cfg_space, 2, 1) == 2){
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	memmap.first_gfn = (bar_s + vgt_aperture_offset(vgt)) >> PAGE_SHIFT;
	memmap.first_mfn = vgt_aperture_base(vgt) >> PAGE_SHIFT;
	if (!vgt->ballooning)
		memmap.nr_mfns = vgt->state.bar_size[1] >> PAGE_SHIFT;
	else
		memmap.nr_mfns = vgt_aperture_sz(vgt) >> PAGE_SHIFT;

	memmap.map = map;
	memmap.domid = vgt->vm_id;

	printk("%s: domid=%d gfn_s=0x%llx mfn_s=0x%llx nr_mfns=0x%x\n", map==0? "remove_map":"add_map",
			vgt->vm_id, memmap.first_gfn, memmap.first_mfn, memmap.nr_mfns);

	r = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);

	if (r != 0)
		printk(KERN_ERR "vgt_hvm_map_apperture fail with %d!\n", r);
	else
		vgt->state.bar_mapped[1] = map;

	return r;
}

/*
 * Zap the GTTMMIO bar area for vGT trap and emulation.
 */
static int vgt_hvm_set_trap_area(struct vgt_device *vgt)
{
	char *cfg_space = &vgt->state.cfg_space[0];
        struct xen_hvm_vgt_set_trap_io trap;
        uint64_t bar_s, bar_e;
        int r;

		trap.domid = vgt->vm_id;
        trap.nr_pio_frags = 0;
        trap.nr_mmio_frags = 1;
        cfg_space += VGT_REG_CFG_SPACE_BAR0;
		if (VGT_GET_BITS(*cfg_space, 2, 1) == 2){
			/* 64 bits MMIO bar */
			bar_s = * (uint64_t *) cfg_space;
		} else {
			/* 32 bits MMIO bar */
			bar_s = * (uint32_t*) cfg_space;
		}
		bar_s &= ~0xF; /* clear the LSB 4 bits */
        bar_e = bar_s + vgt->state.bar_size[0] - 1;
        trap.mmio_frags[0].s = bar_s;
        trap.mmio_frags[0].e = bar_e;

	r = HYPERVISOR_hvm_op(HVMOP_vgt_set_trap_io, &trap);
	if (r < 0) {
		printk(KERN_ERR "HVMOP_vgt_set_trap_io %d!\n",
			r);
		return r;
	}
	return r;
}

typedef union _SCI_REG_DATA{
	uint16_t data;
	struct {
		uint16_t trigger:1; /* bit 0: trigger SCI */
		uint16_t reserve:14;
		uint16_t method:1; /* bit 15: 1 - SCI, 0 - SMI */
	};
} SCI_REG_DATA;

static bool vgt_cfg_sci_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	printk("VM%d Read SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint16_t*)p_data);

	return true;
}

static bool vgt_cfg_sci_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	SCI_REG_DATA sci_reg;

	printk("VM%d Write SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint32_t*)p_data);

	if( (bytes == 2) || (bytes == 4)){
		memcpy (&vgt->state.cfg_space[offset], p_data, bytes);
	} else {
		printk("Warning: VM%d vgt_cfg_sci_write invalid bytes=%d, ignore it\n", vgt->vm_id, bytes);
		return false;
	}

	sci_reg.data = *(uint16_t*)(vgt->state.cfg_space + offset);
	sci_reg.method = 1; /* set method to SCI */
	if (sci_reg.trigger == 1){
		printk("SW SCI Triggered by VM%d\n", vgt->vm_id);
		/* TODO: add SCI emulation */
		sci_reg.trigger = 0; /* SCI completion indicator */
	}

	memcpy (&vgt->state.cfg_space[offset], &sci_reg.data , 2);

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
		case VGT_REG_CFG_SWSCI_TRIGGER:
			vgt_cfg_sci_read(vgt, offset, p_data, bytes);
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
	uint32_t *cfg_reg, new, size;
	bool rc = true;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
		case VGT_REG_CFG_SPACE_BAR_ROM:	/* ROM */
			ASSERT((bytes == 4) && (off & 3) == 0);

			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			if ((off & ~3) != VGT_REG_CFG_SPACE_BAR_ROM)
				size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
			else
				size = vgt->state.bar_size[3];
			if ( new == 0xFFFFFFFF || new == 0xFFFFF800 ) {
				/*
				 * Power-up software can determine how much address
				 * space the device requires by writing a value of
				 * all 1's to the register and then reading the value
				 * back. The device will return 0's in all don't-care
				 * address bits.
				 */
				new = new & ~(size-1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
			} else {
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 0);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR_ROM)
					vgt_hvm_map_rom(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR_ROM)
					vgt_hvm_map_rom(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt);
			}
			break;

		case VGT_REG_CFG_SPACE_MSAC:
			printk("Guest write MSAC %x, %d: Not supported yet\n",
					*(char *)p_data, bytes);
			break;

		case VGT_REG_CFG_SWSCI_TRIGGER:
			rc = vgt_cfg_sci_write(vgt, off, p_data, bytes);
			break;

		case VGT_REG_CFG_SPACE_BAR1+4:
		case VGT_REG_CFG_SPACE_BAR0+4:
		case VGT_REG_CFG_SPACE_BAR2+4:
			ASSERT((bytes == 4) && (off & 3) == 0);
			if (*(uint32_t *)p_data == 0xFFFFFFFF)
				/* BAR size is not beyond 4G, so return all-0 in uppper 32 bit */
				*cfg_reg = 0;
			else
				*cfg_reg = *(uint32_t*)p_data;
			break;
		case 0x90:
		case 0x94:
		case 0x98:
			printk("vGT: write to MSI capa(%x) with val (%x)\n", off, *(uint32_t *)p_data);
		default:
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			break;
	}
	/*
	 * FIXME: assume most dmo0's cfg writes should be propogated to
	 * the real conf space. In the case where propogation is required
	 * but value needs be changed (sReg), do it here
	 */
	return rc;
}

bool vgt_initialize_mmio_hooks(struct pgt_device *pdev)
{
	int i;

	printk("mmio hooks initialized\n");
	/* ring registers */
	for (i=0; i < MAX_ENGINES; i++)
		if (!vgt_register_mmio_handler(ring_mmio_base[i],
			 RB_REGS_SIZE,
			ring_mmio_read, ring_mmio_write))
			return false;

	vgt_register_mmio_handler( _REG_PCH_DPB_AUX_CH_CTL, 6*4,
			dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write);

	vgt_register_mmio_handler( _REG_PCH_DPC_AUX_CH_CTL, 6*4,
			dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write);

	vgt_register_mmio_handler( _REG_PCH_DPD_AUX_CH_CTL, 6*4,
			dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write);

	vgt_register_mmio_handler( _REG_FENCE_0_LOW, 0x80,
			fence_mmio_read, fence_mmio_write);

	vgt_register_mmio_handler(_REG_PCH_ADPA, 4,
			pch_adpa_mmio_read, pch_adpa_mmio_write);

	vgt_register_mmio_handler( _REG_PCH_GMBUS0, 4*4,
			gmbus_mmio_read, gmbus_mmio_write);

#ifdef ENABLE_GPIO_EMULATION
	vgt_register_mmio_handler( _REG_PCH_GPIOA, 6*4,
			gpio_mmio_read, gpio_mmio_write);
#endif /* ENABLE_GPIO_EMULATION */

	vgt_register_mmio_handler(_REG_DP_A_CTL, 4,
			dp_ctl_mmio_read, dp_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_DP_B_CTL, 4,
			dp_ctl_mmio_read, dp_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_DP_C_CTL, 4,
			dp_ctl_mmio_read, dp_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_DP_D_CTL, 4,
			dp_ctl_mmio_read, dp_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_HDMI_B_CTL, 4,
			hdmi_ctl_mmio_read, hdmi_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_HDMI_C_CTL, 4,
			hdmi_ctl_mmio_read, hdmi_ctl_mmio_write);

	vgt_register_mmio_handler(_REG_HDMI_D_CTL, 4,
			hdmi_ctl_mmio_read, hdmi_ctl_mmio_write);

	vgt_register_mmio_write( _REG_FORCEWAKE, force_wake_write);

	vgt_register_mmio_write( _REG_RC_STATE_CTRL_1, rc_state_ctrl_1_mmio_write);

	vgt_register_mmio_write( _REG_RC_STATE_CTRL_2, rc_state_ctrl_2_mmio_write);

	vgt_register_mmio_write( _REG_GEN6_GDRST, gen6_gdrst_mmio_write);

	vgt_register_mmio_write( _REG_PCH_PP_CONTROL, pch_pp_control_mmio_write);
	vgt_register_mmio_write( _REG_TRANSACONF, transaconf_mmio_write);
	vgt_register_mmio_write( _REG_TRANSBCONF, transaconf_mmio_write);

	vgt_register_mmio_read( _REG_PIPEA_FRMCOUNT, pipe_frmcount_mmio_read);
	vgt_register_mmio_read( _REG_PIPEB_FRMCOUNT, pipe_frmcount_mmio_read);
	vgt_register_mmio_read( _REG_PIPEADSL, pipe_dsl_mmio_read);
	vgt_register_mmio_read( _REG_PIPEBDSL, pipe_dsl_mmio_read);

	vgt_register_mmio_read( _REG_HDCP_STATUS_REG_1, hdcp_status_mmio_read);
	vgt_register_mmio_read( _REG_HDCP_STATUS_REG_2, hdcp_status_mmio_read);
	vgt_register_mmio_read( _REG_HDCP_STATUS_REG_3, hdcp_status_mmio_read);
	vgt_register_mmio_read( _REG_HDCP_STATUS_REG_4, hdcp_status_mmio_read);
	vgt_register_mmio_read( _REG_HDCP_KEY_STATUS_REG, hdcp_key_status_mmio_read);
	vgt_register_mmio_read( _REG_HDCP_PCH_BOOT_AUTH_STATUS_REG ,
			hdcp_pch_boot_auth_mmio_read);

#ifdef VGT_DEBUGFS_DUMP_FB
	vgt_register_mmio_write(_REG_DSPASURF, dspsurf_mmio_write);
	vgt_register_mmio_write(_REG_DSPBSURF, dspsurf_mmio_write);
#endif
	vgt_register_mmio_write(_REG_PIPEACONF, pipe_conf_mmio_write);
	vgt_register_mmio_write(_REG_PIPEBCONF, pipe_conf_mmio_write);
	vgt_register_mmio_write(_REG_FDI_RXA_IIR, fdi_rx_iir_mmio_write);
	vgt_register_mmio_write(_REG_FDI_RXB_IIR, fdi_rx_iir_mmio_write);
	/* TODO: vgt_register_mmio_write(_REG_FDI_RX_IIR_C,...)*/
	vgt_register_mmio_write(_REG_FDI_RXA_CTL, update_fdi_rx_iir_status);
	vgt_register_mmio_write(_REG_FDI_RXB_CTL, update_fdi_rx_iir_status);
	vgt_register_mmio_write(_REG_FDI_TXA_CTL, update_fdi_rx_iir_status);
	vgt_register_mmio_write(_REG_FDI_TXB_CTL, update_fdi_rx_iir_status);
	vgt_register_mmio_write(_REG_FDI_RXA_IMR, update_fdi_rx_iir_status);
	vgt_register_mmio_write(_REG_FDI_RXB_IMR, update_fdi_rx_iir_status);

	if (pdev->enable_ppgtt) {
		dprintk("Hook up PPGTT register handlers\n");
		/* trap PPGTT base register */
		vgt_register_mmio_handler(_REG_RCS_PP_DIR_BASE_IVB, 4,
				rcs_pp_dir_base_read, rcs_pp_dir_base_write);
		vgt_register_mmio_handler(_REG_BCS_PP_DIR_BASE, 4,
				bcs_pp_dir_base_read, bcs_pp_dir_base_write);
		vgt_register_mmio_handler(_REG_VCS_PP_DIR_BASE, 4,
				vcs_pp_dir_base_read, vcs_pp_dir_base_write);
		/* XXX cache register? */
		/* PPGTT enable register */
		vgt_register_mmio_handler(_REG_RCS_GFX_MODE_IVB, 4,
				rcs_gfx_mode_read, rcs_gfx_mode_write);
		vgt_register_mmio_handler(_REG_BCS_BLT_MODE_IVB, 4,
				bcs_blt_mode_read, bcs_blt_mode_write);
		vgt_register_mmio_handler(_REG_VCS_MFX_MODE_IVB, 4,
				vcs_mfx_mode_read, vcs_mfx_mode_write);
	}

	return true;
}

static int xen_get_nr_vcpu(int vm_id)
{
	/* get number of the VCPUs */
	/* TODO: add hypervisor specific implementation */
	return 1;
}

static int hvm_get_parameter_by_dom(domid_t domid, int idx, uint64_t *value)
{
	struct xen_hvm_param xhv;
	int r;

	xhv.domid = domid;
	xhv.index = idx;
	r = HYPERVISOR_hvm_op(HVMOP_get_param, &xhv);
	if (r < 0) {
		printk(KERN_ERR "Cannot get hvm parameter %d: %d!\n",
			idx, r);
		return r;
	}
	*value = xhv.value;
	return r;
}

static shared_iopage_t *map_hvm_iopage(struct vgt_device *vgt)
{
	uint64_t ioreq_pfn;
	int rc;

	rc =hvm_get_parameter_by_dom(vgt->vm_id, HVM_PARAM_IOREQ_PFN, &ioreq_pfn);
	if (rc < 0)
		return NULL;

	return xen_remap_domain_mfn_range_in_kernel(ioreq_pfn, 1, vgt->vm_id);
}

void vgt_hvm_write_cf8_cfc(struct vgt_device *vgt,
     unsigned int port, unsigned int bytes, unsigned long val)
{
    dprintk("vgt_hvm_write_cf8_cfc %x %d %lx\n", port, bytes, val);
    if ( (port & ~3) == 0xcf8 ) {
        ASSERT (bytes == 4);
        ASSERT ((port & 3) == 0);
        vgt->last_cf8 = (uint32_t) val;
    }
    else {
        ASSERT ( (vgt->last_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
             ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
        vgt_emulate_cfg_write (vgt,
             (vgt->last_cf8 & 0xfc) + (port & 3),
             &val, bytes);
    }
}

void vgt_hvm_read_cf8_cfc(struct vgt_device *vgt,
       unsigned int port, unsigned int bytes, unsigned long *val)
{
    unsigned long data;

    if ((port & ~3)== 0xcf8) {
        memcpy(val, (uint8_t*)&vgt->last_cf8 + (port & 3), bytes);
    }
    else {
//        ASSERT ( (vgt->last_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
            ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
        vgt_emulate_cfg_read(vgt, (vgt->last_cf8 & 0xfc) + (port & 3),
                     &data, bytes);
        memcpy(val, &data, bytes);
    }
    dprintk("VGT: vgt_cfg_read_emul port %x bytes %x got %lx\n",
               port, bytes, *val);
}

void _hvm_mmio_emulation(struct vgt_device *vgt, struct ioreq *req)
{
    int i, sign;
    char *cfg_space = &vgt->state.cfg_space[0];
    uint64_t  base = * (uint64_t *) (cfg_space + VGT_REG_CFG_SPACE_BAR0);
    uint64_t  tmp;

    sign = req->df ? -1 : 1;

    if (req->dir == IOREQ_READ) {
        /* MMIO READ */
        if (!req->data_is_ptr) {
            ASSERT (req->count == 1);

            dprintk("HVM_MMIO_read: target register (%lx).\n", (unsigned long)req->addr);
            vgt_emulate_read(vgt,
                        req->addr,
                        &req->data,
                        req->size);
        }
        else {
	    ASSERT (req->addr + sign * req->count * req->size >= base);
	    ASSERT (req->addr + sign * req->count * req->size <
                            base + vgt->state.bar_size[0]);
            dprintk("HVM_MMIO_read: rep %d target memory %lx, slow!\n",
                         req->count, (unsigned long)req->addr);
            for (i=0; i<req->count; i++) {
                tmp = 0;
                vgt_emulate_read(vgt,
                        req->addr + sign * i * req->size,
                        &tmp,
                        req->size);
                /*
                 *  TODO: Hypercall to write data (tmp) to
                 *      req->data + sign * i * req->size
                 *  We can use a hypercall or entire/cache foreign map.
                 *  Refer to IOCTL_PRIVCMD_MMAPBATCH_V2.
                 */
            }
        }
    }
    else {   /* MMIO Write */
        if (!req->data_is_ptr) {
            ASSERT (req->count == 1);

            dprintk("HVM_MMIO_write: target register (%lx).\n", (unsigned long)req->addr);
            vgt_emulate_write(vgt,
                        req->addr,
                        &req->data,
                        req->size);
        }
        else {
	    ASSERT (req->addr + sign * req->count * req->size >= base);
	    ASSERT (req->addr + sign * req->count * req->size <
                            base + vgt->state.bar_size[0]);
            dprintk("HVM_MMIO_write: rep %d target memory %lx, slow!\n",
                         req->count, (unsigned long)req->addr);

            for (i=0; i<req->count; i++) {
                tmp = 0;
                /*
                 *  TODO: Hypercall to read data (tmp) from
                 *      req->data + sign * i * req->size
                 *  We can use a hypercall or entire/cache foreign map.
                 *  Refer to IOCTL_PRIVCMD_MMAPBATCH_V2.
                 */
                vgt_emulate_write(vgt,
                        req->addr + sign * i * req->size,
                        &tmp,
                        req->size);
            }

        }
    }
}

void _hvm_pio_emulation(struct vgt_device *vgt, struct ioreq *ioreq)
{
    int sign;
    //char *pdata;

    sign = ioreq->df ? -1 : 1;

    if (ioreq->dir == IOREQ_READ) {
        /* PIO READ */
        if (!ioreq->data_is_ptr) {
            vgt_hvm_read_cf8_cfc(vgt,
                  ioreq->addr,
                  ioreq->size,
                  (unsigned long*) &ioreq->data);
        }
        else {
            dprintk("VGT: _hvm_pio_emulation read data_ptr %lx\n",
			(long)ioreq->data);
            /*
             * The data pointer of emulation is guest physical address
             * so far, which is godo to Qemu emulation, but hard for
             * vGT driver which doesn't know gpn_2_mfn translation.
             * We may ask hypervisor to use mfn for vGT driver.
             * We keep assert here to see if guest really use it.
             */
            ASSERT(0);
#if 0
            pdata = (char *)ioreq->data;
            for (i=0; i < ioreq->count; i++) {
                vgt_hvm_read_cf8_cfc(vgt,
                     ioreq->addr,
                     ioreq->size,
                     (unsigned long *)pdata);
                pdata += ioreq->size * sign;
            }
#endif
        }
    }
    else {
        /* PIO WRITE */
        if (!ioreq->data_is_ptr) {
            vgt_hvm_write_cf8_cfc(vgt,
                  ioreq->addr,
                  ioreq->size,
                  (unsigned long) ioreq->data);
        }
        else {
            dprintk("VGT: _hvm_pio_emulation write data_ptr %lx\n",
			(long)ioreq->data);
            /*
             * The data pointer of emulation is guest physical address
             * so far, which is godo to Qemu emulation, but hard for
             * vGT driver which doesn't know gpn_2_mfn translation.
             * We may ask hypervisor to use mfn for vGT driver.
             * We keep assert here to see if guest really use it.
             */
            ASSERT(0);
#if 0
            pdata = (char *)ioreq->data;

            for (i=0; i < ioreq->count; i++) {
                vgt_hvm_write_cf8_cfc(vgt,
                     ioreq->addr,
                     ioreq->size, *(unsigned long *)pdata);
                pdata += ioreq->size * sign;
            }
#endif
        }
    }
}

static int vgt_hvm_do_ioreq(struct vgt_device *vgt, struct ioreq *ioreq)
{
        switch (ioreq->type) {
            case IOREQ_TYPE_PIO:	/* PIO */
                if ((ioreq->addr & ~7) != 0xcf8)
                    printk(KERN_ERR "vGT: Unexpected PIO %lx emulation\n",
                           (long) ioreq->addr);
                else
                    _hvm_pio_emulation(vgt, ioreq);
                break;
            case IOREQ_TYPE_COPY:	/* MMIO */
                _hvm_mmio_emulation(vgt, ioreq);
                break;
            default:
                printk(KERN_ERR "vGT: Unknown ioreq type %x\n", ioreq->type);
                break;
        }
	return 0;
}

static irqreturn_t vgt_hvm_io_req_handler(int irq, void* dev)
{
	struct vgt_device *vgt;
	struct vgt_hvm_info *info;
	int vcpu;
	struct ioreq *ioreq;

	vgt = (struct vgt_device *)dev;
	info = vgt->hvm_info;

	for(vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if(info->evtchn_irq[vcpu] == irq)
			break;
	}
	if (vcpu == info->nr_vcpu){
		/*opps, irq is not the registered one*/
		return IRQ_NONE;
	}

	ioreq = vgt_get_hvm_ioreq(vgt, vcpu);

	vgt_hvm_do_ioreq(vgt, ioreq);

	ioreq->state = STATE_IORESP_READY;

	notify_remote_via_irq(irq);

	return IRQ_HANDLED;
}

static void vgt_hvm_opregion_init(struct vgt_device *vgt)
{
	uint8_t* buf;
	vgt->opregion_pa = *(uint32_t*)(vgt->state.cfg_space + VGT_REG_CFG_OPREGION );
	vgt->opregion_va = __va(vgt->opregion_pa);

	/* for unknown reason, the value in LID field is incorrect
	   which block the windows guest, so workaround it by force
	   setting it to "OPEN"
	 **/

	buf = (uint8_t*)vgt->opregion_va;
	buf[VGT_OPREGION_REG_CLID] = 0x3;
}

int vgt_hvm_io_init(struct vgt_device *vgt)
{
	vgt_hvm_opregion_init(vgt);
	return vgt_hvm_map_opregion(vgt, 1);
}

int vgt_hvm_info_init(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu, rc;

	info = kzalloc(sizeof(struct vgt_hvm_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	vgt->hvm_info = info;

	info->iopage = map_hvm_iopage(vgt);
	if (info->iopage == NULL){
		printk(KERN_ERR "Failed to map HVM I/O page for VM%d\n", vgt->vm_id);
		rc = -EFAULT;
		goto err;
	}

	info->nr_vcpu = xen_get_nr_vcpu(vgt->vm_id);
	ASSERT(info->nr_vcpu > 0);

	info->evtchn_irq = kmalloc(info->nr_vcpu * sizeof(int), GFP_KERNEL);
	if (info->evtchn_irq == NULL){
		rc = -ENOMEM;
		goto err;
	}
	for( vcpu = 0; vcpu < info->nr_vcpu; vcpu++ )
		info->evtchn_irq[vcpu] = -1;

	for( vcpu = 0; vcpu < info->nr_vcpu; vcpu++ ){
		rc = bind_interdomain_evtchn_to_irqhandler( vgt->vm_id,
				info->iopage->vcpu_ioreq[vcpu].vgt_eport,
				vgt_hvm_io_req_handler, 0,
				"vgt", vgt );
		if ( rc < 0 ){
			printk(KERN_ERR "Failed to bind event channle for vgt HVM IO handler, rc=%d\n", rc);
			goto err;
		}
		info->evtchn_irq[vcpu] = rc;
	}

	return 0;

err:
	vgt_hvm_info_deinit(vgt);
	return rc;
}

void vgt_hvm_info_deinit(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu;

	info = vgt->hvm_info;

	if (info == NULL)
		return;

	/*TODO: unmap io page */

	if (!info->nr_vcpu || info->evtchn_irq == NULL)
		goto out1;

	for (vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if( info->evtchn_irq[vcpu] >= 0)
			unbind_from_irqhandler(info->evtchn_irq[vcpu], vgt);
	}

	kfree(info->evtchn_irq);

out1:
	kfree(info);

	return;
}

#ifdef AUX_CH_WORKAROUND
void vgt_init_aux_ch_vregs(vgt_i2c_bus_t *i2c_bus, vgt_reg_t *vregs)
{
	int i, j;
	for (i = 0; i < VGT_DP_NUM; ++ i) {
		for (j = 0; j < AUX_REGISTER_NUM; ++ j) {
			i2c_bus->aux_ch.aux_registers[i][j] =
				&(i2c_bus->aux_ch.aux_shadow_reg[i][j]);
		}
	}
}
#else /* AUX_CH_WORKAROUND */
static inline void vgt_aux_register_assign(aux_reg_t *dst[AUX_REGISTER_NUM],
					vgt_reg_t *reg_base)
{
	int i;
	AUX_CH_REGISTERS reg_idx = AUX_CH_CTL;

	for (i = 0; i < AUX_REGISTER_NUM; i ++) {
		dst[reg_idx] = (aux_reg_t *)((char *)reg_base + (i << 2));
		reg_idx ++;
	}
}

void vgt_init_aux_ch_vregs(vgt_i2c_bus_t *i2c_bus, vgt_reg_t *vregs)
{
	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DP_B],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPB_AUX_CH_CTL));

	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DP_C],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPC_AUX_CH_CTL));

	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DP_D],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPD_AUX_CH_CTL));
}
#endif /* AUX_CH_WORKAROUND */

/* vgt_aux_ch_transaction()
 *
 * msg_size will not be larger than 4.
 */
static unsigned int vgt_aux_ch_transaction(struct pgt_device *pdev,
				unsigned int aux_ctrl_addr,
				unsigned int msg, int msg_size)
{
	/* TODO: DATA from the i915 driver. Need more handling.
	 */
	int aux_clock_divider = 62;
	int precharge = 3;		//GEN6
	unsigned int status;

	while (VGT_MMIO_READ(pdev, aux_ctrl_addr) &
				_REGBIT_DP_AUX_CH_CTL_SEND_BUSY);

	VGT_MMIO_WRITE(pdev, aux_ctrl_addr + 4, msg);
	VGT_MMIO_WRITE(pdev, aux_ctrl_addr,
				_REGBIT_DP_AUX_CH_CTL_SEND_BUSY	|
				_REGBIT_DP_AUX_CH_CTL_TIME_OUT_400us |
				msg_size << _DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT |
				precharge << _DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT	|
				aux_clock_divider << _DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT |
				_REGBIT_DP_AUX_CH_CTL_DONE |
				_REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR |
				_REGBIT_DP_AUX_CH_CTL_RECV_ERR);

	while((status = VGT_MMIO_READ(pdev, aux_ctrl_addr)) &
		_REGBIT_DP_AUX_CH_CTL_SEND_BUSY);

	VGT_MMIO_WRITE(pdev, aux_ctrl_addr,
				status |
				_REGBIT_DP_AUX_CH_CTL_DONE |
				_REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR |
				_REGBIT_DP_AUX_CH_CTL_RECV_ERR);

	return VGT_MMIO_READ(pdev, aux_ctrl_addr + 4);
}

#define EDID_REPEAT_UNTIL(cond, repeat_num, interval, time_out)		\
do {									\
	int i;								\
	time_out = 1;							\
	for(i = 0; i < (repeat_num); ++ i) {				\
		if(cond) {						\
			time_out = 0;					\
			break;						\
		} else {						\
			msleep(interval);				\
		}							\
	}								\
} while(0);

void vgt_probe_edid(struct pgt_device *pdev, int index)
{
	int i;

	VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS0, 0);

	for (i = 0; i < EDID_NUM; ++ i) {
		int gmbus_port = 0;
		unsigned int aux_ch_addr = 0;
		vgt_edid_data_t **pedid = &(pdev->pdev_edids[i]);

		if ((i != index) && (index != -1)) {
			continue;
		}
		switch (i) {
		case EDID_VGA:
			printk("EDID_PROBE: VGA.\n");
			gmbus_port = 2;
			break;
		case EDID_LVDS:
			printk("EDID_PROBE: LVDS.\n");
			gmbus_port = 3;
			break;
		case EDID_HDMIC:
			printk("EDID_PROBE: HDMI C.\n");
			gmbus_port = 4;
			break;
		case EDID_HDMIB:
			printk("EDID_PROBE: HDMI B.\n");
			// no gmbus corresponding interface. Do not handle it.
			break;
		case EDID_HDMID:
			printk("EDID_PROBE: HDMI D.\n");
			gmbus_port = 6;
			break;
		case EDID_DPB:
			if (VGT_MMIO_READ(pdev, _REG_PCH_DPB_AUX_CH_CTL) | _DP_DETECTED) {
				printk("EDID_PROBE: DP B Detected.\n");
				aux_ch_addr = _REG_PCH_DPB_AUX_CH_CTL;
			} else {
				printk("EDID_PROBE: DP B is not detected.\n");
			}
			break;
		case EDID_DPC:
			if (VGT_MMIO_READ(pdev, _REG_PCH_DPC_AUX_CH_CTL) | _DP_DETECTED) {
				printk("EDID_PROBE: DP C Detected.\n");
				aux_ch_addr = _REG_PCH_DPC_AUX_CH_CTL;
			} else {
				printk("EDID_PROBE: DP C is not detected.\n");
			}
			break;
		case EDID_DPD:
			if (VGT_MMIO_READ(pdev, _REG_PCH_DPD_AUX_CH_CTL) | _DP_DETECTED) {
				printk("EDID_PROBE: DP D Detected.\n");
				aux_ch_addr = _REG_PCH_DPD_AUX_CH_CTL;
			} else {
				printk("EDID_PROBE: DP D is not detected.\n");
			}
			break;
		default:
			printk("EDID_PROBE: Others?\n");
			break;
		}

		if (gmbus_port || aux_ch_addr) {
			if (!*pedid) {
				*pedid = kmalloc(sizeof(vgt_edid_data_t),
							GFP_KERNEL);
				if (*pedid == NULL) {
					printk("ERROR: Insufficient memory in %s\n",
							__FUNCTION__);
					BUG();
				}
			}
		} else {
			if (*pedid) {
				printk("EDID_PROBE: Free edid memory.\n");
				kfree(*pedid);
				*pedid = NULL;
			}
		}

		if (gmbus_port) {
			int length;
			int val;
			int timeout;
			VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS0, gmbus_port);
			// write addr and offset
			VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS3, 0);
			VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1,
					_GMBUS_SW_RDY |
					_GMBUS_CYCLE_WAIT |
					(1 << _GMBUS_BYTE_COUNT_SHIFT) |
					(EDID_ADDR << _GMBUS_SLAVE_ADDR_SHIFT) |
					_GMBUS_SLAVE_WRITE);
			(void)VGT_MMIO_READ(pdev, _REG_PCH_GMBUS2);

			EDID_REPEAT_UNTIL(((val = VGT_MMIO_READ(pdev, _REG_PCH_GMBUS2))
				 & (_GMBUS_SATOER | _GMBUS_HW_WAIT_PHASE)), 5, 10, timeout);

			if (timeout || (val & _GMBUS_SATOER)) {
				VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1, _GMBUS_SW_CLR_INT);
				VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1, 0);
				kfree(*pedid);
				*pedid = NULL;
				continue;
			}

			// start read.
			VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1,
					_GMBUS_SW_RDY |
					_GMBUS_CYCLE_STOP | _GMBUS_CYCLE_WAIT |
					(EDID_SIZE << _GMBUS_BYTE_COUNT_SHIFT) |
					(EDID_ADDR << _GMBUS_SLAVE_ADDR_SHIFT) |
					_GMBUS_SLAVE_READ);
			(void)VGT_MMIO_READ(pdev, _REG_PCH_GMBUS2);

			length = 0;
			do {
				int j = 0;
				EDID_REPEAT_UNTIL(((val = VGT_MMIO_READ(pdev, _REG_PCH_GMBUS2))
					 & (_GMBUS_SATOER | _GMBUS_HW_RDY)), 5, 10, timeout);
				if (timeout || (val & _GMBUS_SATOER)) {
					VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1, _GMBUS_SW_CLR_INT);
					VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS1, 0);
					kfree(*pedid);
					*pedid = NULL;
					break;
				}

				val = VGT_MMIO_READ(pdev, _REG_PCH_GMBUS3);
				for (j = 0; j < 4; ++ j) {
					(*pedid)->edid_block[length] = (val) & 0xff;
					length ++;
					val >>= 8;
				}
			} while (length < EDID_SIZE);

			/* finish reading. Check the hw state and disable gmbus. */
			EDID_REPEAT_UNTIL((((val = VGT_MMIO_READ(pdev, _REG_PCH_GMBUS2))
						& _GMBUS_ACTIVE) == 0), 5, 10, timeout);
			if (timeout) {
				printk("vGT: timeout while waiting for gmbus to be inactive. Will force close.\n");
			}
			VGT_MMIO_WRITE(pdev, _REG_PCH_GMBUS0, 0);
		}

		if (aux_ch_addr) {
			unsigned int msg;
			unsigned int value;
			int length;

			msg = ((VGT_AUX_I2C_MOT << 4) << 24) |
				(0 << 16) |
				(EDID_ADDR << 8) |
				0;
			/* start */
			vgt_aux_ch_transaction(pdev, aux_ch_addr, msg, 3);

			/* read */
			msg = (((VGT_AUX_I2C_MOT | VGT_AUX_I2C_READ) << 4) << 24) |
				(0 << 16) |
				(EDID_ADDR << 8) |
				0;

			for (length = 0; length < EDID_SIZE; length ++) {
				value = vgt_aux_ch_transaction(pdev, aux_ch_addr, msg, 4);
				(*pedid)->edid_block[length] = ((value) & 0xff0000) >> 16;
			}
		}

		if (*pedid) {
			int i;
			unsigned char *block = (*pedid)->edid_block;
			printk("EDID_PROBE: EDID is:\n");
			for (i = 0; i < EDID_SIZE; ++ i) {
				if ((block[i] >= 'a' && block[i] <= 'z') ||
				(block[i] >= 'A' && block[i] <= 'Z')) {
					printk ("%c ", block[i]);
				} else {
					printk ("0x%x ", block[i]);
				}
				if (((i + 1) & 0xf) == 0) {
					printk ("\n");
				}
			}
		}
	}
}

/* vgt_propagate_edid
 *
 * Propagate the EDID information stored in pdev to vdev device.
 * Right now the vgt uses the same EDID. In future, there could be
 * policy to change the EDID that is used by vdev instances.
 */
void vgt_propagate_edid(struct vgt_device *vgt, int index)
{
	int i;

	for (i = 0; i < EDID_NUM; ++ i) {
		vgt_edid_data_t	*edid = vgt->pdev->pdev_edids[i];

		if ((i != index) && (index != -1)) {
			continue;
		}

		if (!edid) {
			printk ("EDID_PROPAGATE: Clear EDID %d\n", i);
			if (vgt->vgt_edids[i]) {
				kfree(vgt->vgt_edids[i]);
				vgt->vgt_edids[i] = NULL;
			}
		} else {
			printk ("EDID_PROPAGATE: Propagate EDID %d\n", i);
			if (!vgt->vgt_edids[i]) {
				vgt->vgt_edids[i] = kmalloc(
						sizeof(vgt_edid_data_t),
						GFP_KERNEL);
				if (vgt->vgt_edids[i] == NULL) {
					printk("ERROR: Insufficient memory in %s\n",
							__FUNCTION__);
					BUG();
					return;
				}
			}
			memcpy(vgt->vgt_edids[i], edid,
				sizeof(vgt_edid_data_t));

			{
			int j;
			unsigned char *block = vgt->vgt_edids[i]->edid_block;
			printk("EDID_PROPAGATE: EDID[%d] is:\n", i);
			for (j = 0; j < EDID_SIZE; ++ j) {
				if ((block[j] >= 'a' && block[j] <= 'z') ||
					(block[j] >= 'A' && block[j] <= 'Z')) {
					printk ("%c ", block[j]);
				} else {
					printk ("0x%x ", block[j]);
				}
				if (((j + 1) & 0xf) == 0) {
					printk ("\n");
				}
			}
			}
		}
	}
}

void vgt_clear_edid(struct vgt_device *vgt, int index)
{
	int i;

	for (i = 0; i < EDID_NUM; ++ i) {
		if ((i == index) || (index == -1)) {
			if (vgt->vgt_edids[i]) {
				printk("EDID_CLEAR: Clear EDID[0x%x] of vgt %d\n",
					i, vgt->vm_id);
				kfree(vgt->vgt_edids[i]);
				vgt->vgt_edids[i] = NULL;
			}
		}
	}
}
