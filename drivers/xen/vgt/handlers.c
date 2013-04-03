
/*
 * MMIO virtualization handlers
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

#include <linux/delay.h>
#include <linux/acpi.h>

#include <xen/interface/hvm/hvm_op.h>
#include <xen/vgt.h>

#include "vgt.h"

static bool vgt_error_handler(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	printk("vGT: reg (%x) needs special handler\n", offset);
	ASSERT(0);
	return true;
}

bool gmbus_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return vgt_i2c_handle_gmbus_read(vgt, offset, p_data, bytes);
}

bool gmbus_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return vgt_i2c_handle_gmbus_write(vgt, offset, p_data, bytes);
}

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

static inline void set_vRC(struct vgt_device *vgt, int c)
{
	__vreg(vgt, _REG_GT_CORE_STATUS) = c;
	__vreg(vgt, _REG_GT_THREAD_STATUS) = c;
}

static void set_vRC_to_C6(struct vgt_device *vgt)
{
	vgt_dbg("Virtual Render C state set to C6\n");
	set_vRC(vgt, 3);
}

static void set_vRC_to_C0(struct vgt_device *vgt)
{
	vgt_dbg("Virtual Render C state set to C0\n");
	set_vRC(vgt, 0);
}

static void v_force_wake_get(struct vgt_device *vgt)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&vgt->pdev->v_force_wake_lock, flags);

	if (bitmap_empty(vgt->pdev->v_force_wake_bitmap, VGT_MAX_VMS)){
		rc = hcall_vgt_ctrl(VGT_CTRL_FORCEWAKE_GET);
		if (rc < 0){
			printk("incompatible hypervisor, consider to update your hypervisor\n");
			BUG();
		}
	}

	bitmap_set(vgt->pdev->v_force_wake_bitmap, vgt->vgt_id, VGT_MAX_VMS);

	spin_unlock_irqrestore(&vgt->pdev->v_force_wake_lock, flags);
}

static void v_force_wake_put(struct vgt_device *vgt)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&vgt->pdev->v_force_wake_lock, flags);

	if (test_and_clear_bit(vgt->vgt_id, vgt->pdev->v_force_wake_bitmap)){
		if (bitmap_empty(vgt->pdev->v_force_wake_bitmap, VGT_MAX_VMS)){
			rc = hcall_vgt_ctrl(VGT_CTRL_FORCEWAKE_PUT);
			if (rc < 0){
				printk("incompatible hypervisor, consider to update your hypervisor\n");
				BUG();
			}
		}
	}

	spin_unlock_irqrestore(&vgt->pdev->v_force_wake_lock, flags);
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

	vgt_dbg("VM%d write register FORCE_WAKE with %x\n", vgt->vm_id, data);

	if (vgt->pdev->is_haswell) {
		__vreg(vgt, _REG_FORCEWAKE_ACK_HSW) = data;
	} else {
		__vreg(vgt, _REG_FORCEWAKE_ACK) = data;
	}

	__vreg(vgt, _REG_FORCEWAKE) = data;
	if (data == 1){
		set_vRC_to_C0(vgt);
		v_force_wake_get(vgt);
	}
	else{
		set_vRC_to_C6(vgt);
		v_force_wake_put(vgt);
	}

	return true;
}

bool mul_force_wake_ack_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(u32 *)p_data = __vreg(vgt, offset);
	return true;
}

bool mul_force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data, mask, wake, old_wake, new_wake;


	if (bytes > 4){
		printk("invalid force wake data\n");
		return false;
	}

	data = *(uint32_t*) p_data;

	vgt_dbg("VM%d write register FORCE_WAKE_MT with %x\n", vgt->vm_id, data);

	if (!(__vreg(vgt, _REG_ECOBUS) & ECOBUS_FORCEWAKE_MT_ENABLE)){
		__vreg(vgt, _REG_MUL_FORCEWAKE) = data;
		return true;
	}

	/* bit 16-31: mask
	   bit 0-15: force wake
	   forcewake bit apply only if its mask bit is 1
	 */
	mask = data >> 16;
	wake = data & 0xFFFF;
	old_wake = __vreg(vgt, _REG_MUL_FORCEWAKE) & 0xFFFF;

	new_wake = (old_wake & ~mask) + (wake & mask);
	__vreg(vgt, _REG_MUL_FORCEWAKE) = (data & 0xFFFF0000) + new_wake;

	if (vgt->pdev->is_haswell) {
		__vreg(vgt, _REG_FORCEWAKE_ACK_HSW) = new_wake;
	} else {
		/* IVB */
		__vreg(vgt, _REG_MUL_FORCEWAKE_ACK) = new_wake;
	}

	if (new_wake){
		v_force_wake_get(vgt);
		set_vRC_to_C0(vgt);
	}else{
		v_force_wake_put(vgt);
		set_vRC_to_C6(vgt);
	}

	return true;
}

bool rc_state_ctrl_1_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	ASSERT(bytes == 4);

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_1 with 0x%x\n", vgt->vm_id, data);

	if ( (data & _REGBIT_RC_HW_CTRL_ENABLE) && (data & (_REGBIT_RC_RC6_ENABLE
					| _REGBIT_RC_DEEPEST_RC6_ENABLE	| _REGBIT_RC_DEEP_RC6_ENABLE) ) )
		set_vRC_to_C6(vgt);
	else
		set_vRC_to_C0(vgt);

	return default_mmio_write(vgt, offset, p_data, bytes);

}

bool rc_state_ctrl_2_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	ASSERT(bytes == 4);

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_2 with 0x%x\n", vgt->vm_id, data);

	__vreg(vgt, offset) = data;

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

	for (i = 0; i < vgt->pdev->max_engines; i++) {
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

bool dpy_reg_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	*(uint32_t*)p_data = (1<<17);

	return true;
}

bool dpy_reg_mmio_read_2(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	*(uint32_t*)p_data = 3;

	return true;
}

bool dpy_reg_mmio_read_3(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	*(uint32_t*)p_data = (0x2F << 16);

	return true;
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
	__vreg(vgt, off) = base;
	__sreg(vgt, off) = s_info->base;

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

	vgt_dbg("RCS_PP_DIR_BASE read: 0x%x\n", v_info->base);

	return true;
}

bool rcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("RCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_RCS, off, base);
	return true;
}

bool bcs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_BCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;
	vgt_dbg("BCS_PP_DIR_BASE read: 0x%x\n", v_info->base);
	return true;
}

bool bcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("BCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_BCS, off, base);
	return true;
}

bool vcs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;
	vgt_dbg("VCS_PP_DIR_BASE read: 0x%x\n", v_info->base);
	return true;
}

bool vcs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("VCS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_VCS, off, base);
	return true;
}

bool vecs_pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VECS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->base;
	vgt_dbg("VECS_PP_DIR_BASE read: 0x%x\n", v_info->base);
	return true;
}

bool vecs_pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt->vebox_support = true;

	vgt_dbg("VECS_PP_DIR_BASE write: 0x%x\n", base);
	ring_pp_dir_base_write(vgt, RING_BUFFER_VECS, off, base);
	return true;
}

bool pp_dclv_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	ASSERT(bytes == 4);

	*(u32 *)p_data = 0xFFFFFFFF;
	return true;
}

bool pp_dclv_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 dclv = *(u32 *)p_data;

	ASSERT(bytes == 4);
	__vreg(vgt, off) = dclv;
	__sreg(vgt, off) = dclv;

	vgt_dbg("PP_DCLV write: 0x%x\n", dclv);
	return true;
}

bool rcs_gfx_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_RCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	vgt_dbg("RCS_GFX_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool bcs_blt_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_BCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	vgt_dbg("BCS_BLT_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool vcs_mfx_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VCS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	vgt_dbg("VCS_MFX_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool vecs_mfx_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[RING_BUFFER_VECS].vring_ppgtt_info;

	ASSERT(bytes == 4);

	*(u32 *)p_data = v_info->mode;
	vgt_dbg("VECS_MFX_MODE read: 0x%x\n", v_info->mode);
	return true;
}

bool rcs_gfx_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("RCS_GFX_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_RCS, off, mode);

	return true;
}

bool bcs_blt_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("BCS_BLT_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_BCS, off, mode);

	return true;
}

bool vcs_mfx_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt_dbg("VCS_MFX_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_VCS, off, mode);

	return true;
}

bool vecs_mfx_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	ASSERT(bytes == 4);

	vgt->vebox_support = true;

	vgt_dbg("VECS_MFX_MODE write: 0x%x\n", mode);
	ring_ppgtt_mode(vgt, RING_BUFFER_VECS, off, mode);

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



#define FDI_LINK_TRAIN_PATTERN_1	0
#define FDI_LINK_TRAIN_PATTERN_2	1
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

/*
 * TODO:
 * DAC_CTL is special regarding to that its bits containing multiple
 * policies:
 *	- CRT control bits like enabling, transcoder selection belong
 *	  to display owner
 *	- hotplug status bits are fully virtualized like other interrupt
 *	  status bits
 *	- force hotplug trigger bit needs be emulated
 *
 * Let's take this as one example how this category may be abstracted
 * in the future
 */
bool pch_adpa_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int reg;
	bool rc = true;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);
	reg = offset & ~(bytes - 1);

	/*
	 * the channel status bit is updated by interrupt handler,
	 * or the write handler. so in most time we simply return
	 * vreg value back. The only exception is when force hotplug
	 * trigger bit is set at driver init time when hotplug irq
	 * is disabled. At that time we need to read back hw status
	 * if vgt is the display owner.
	 */
	if (__vreg(vgt, reg) & _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER) {
		if ((__vreg(vgt, reg) & _REGBIT_ADPA_DAC_ENABLE)) {
			vgt_warn("HOTPLUG_FORCE_TRIGGER is set while VGA is enabled!\n");
		}

		if (reg_hw_access(vgt, reg)) {
			rc = default_mmio_read(vgt, offset, p_data, bytes);

			/*
			 * update port detect status accordingly. since hotplug
			 * irq is disabled, no virtual event is triggered.
			 */
			if (__vreg(vgt, reg) & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK)
				set_bit(VGT_CRT, vgt->pdev->port_detect_status);
			else
				clear_bit(VGT_CRT, vgt->pdev->port_detect_status);
		}

		/* clear trigger bit to indicate end of emulation */
		__vreg(vgt, reg) &= ~_REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER;
	}

	memcpy(p_data, (char *)vgt->state.vReg + offset, bytes);

	return true;
}

bool pch_adpa_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wr_data;
	vgt_reg_t old, new, vreg_data;
	bool rc;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(bytes == 4 && (offset & 0x3) == 0);
	reg = offset & ~(bytes - 1);

	new = wr_data = *(vgt_reg_t *)p_data;
	old = __vreg(vgt, reg);

	/* This can verify that bspec was wrong that: channel status
	 * can be cleared by writing back the status bits
	 * */
#if 0
	if ((wr_data & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK)) {
		VGT_MMIO_WRITE(pdev, _REG_PCH_ADPA, wr_data);
		pdata = VGT_MMIO_READ(pdev, _REG_PCH_ADPA);
		if ((pdata & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK))
			printk("vGT: xuanhua failed to clear channel status\n");
	}
#endif

	/* go to general write handler leaving channel status handling aside */
	wr_data &= ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
	rc = default_mmio_write(vgt, offset, &wr_data, bytes);
	vreg_data = __vreg(vgt, reg);

	/* keep channel status bits read-only, it can only be updated by hw */
	vreg_data = (vreg_data & ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK) |
		(old & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK);

	/*
	 * emulate 'force hotplug trigger' which detect the channel status
	 * even when hotplug enable bit is not set.
	 */
	if (new & _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER) {
		/*
		 * TODO: need consider whether to trigger virtual hotplug event
		 * when hotplug is already enabled. Not sure how HW will behave.
		 */
		if ((new & _REGBIT_ADPA_DAC_ENABLE)) {
			vgt_warn("HOTPLUG_FORCE_TRIGGER is set while VGA is enabled!\n");
		}

		/*
		 * if vgt is the owner, the force trigger bit is forwarded
		 * to hw so that channel status will be detected in the read handler.
		 * Or else we emulate channel detection based on port_detect_status.
		 *
		 * TODO: this trick can be eliminated if XenGT detects ports directly
		 */
		if (!reg_hw_access(vgt, reg)) {
			/* FIXME: sometimes only one channel is OK ??? */
			if (test_bit(VGT_CRT, pdev->port_detect_status))
				vreg_data |= _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
			else
				vreg_data &= ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
		}

		/* delay clear of this bit to the 1st read after this write */
		//vreg_data &= ~_REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER;
	}

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
		vgt_dbg("vGT: failed to submit aready attached work!\n");

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

	if (!reg_hw_access(vgt, reg) &&
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

	for (i = 0; i < EDID_MAX; ++ i) {
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
				*pedid = kmalloc(sizeof(vgt_edid_data_t), GFP_ATOMIC);
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
				& (_GMBUS_NAK | _GMBUS_HW_WAIT)), 5, 10, timeout);

			if (timeout || (val & _GMBUS_NAK)) {
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
					& (_GMBUS_NAK | _GMBUS_HW_RDY)), 5, 10, timeout);
				if (timeout || (val & _GMBUS_NAK)) {
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
 * Propagate the EDID information stored in pdev to vgt device.
 * Right now the vgt uses the same EDID. In future, there could be
 * policy to change the EDID that is used by vgt instances.
 */
void vgt_propagate_edid(struct vgt_device *vgt, int index)
{
	int i;

	for (i = 0; i < EDID_MAX; ++ i) {
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
						sizeof(vgt_edid_data_t), GFP_ATOMIC);
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

	for (i = 0; i < EDID_MAX; ++ i) {
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

static bool vga_control_r(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT (bytes == 4 && offset == _REG_CPU_VGACNTRL);

	return default_mmio_read(vgt, offset, p_data, bytes);
}

static bool vga_control_w (struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	ASSERT (bytes == 4 && offset == _REG_CPU_VGACNTRL);

	default_mmio_write(vgt, offset, p_data, bytes);

	if ( __vreg(vgt, offset) & _REGBIT_VGA_DISPLAY_DISABLE ) {
		/* Disable VGA */
		printk("VGT(%d): Disable VGA mode %x\n", vgt->vgt_id,
			(unsigned int) __vreg(vgt, offset));
		vgt_set_uevent(vgt, VGT_DISABLE_VGA);
		vgt_raise_request(pdev, VGT_REQUEST_UEVENT);
	}
	else {
		/* Enable VGA */
		printk("VGT(%d): Enable VGA mode %x\n", vgt->vgt_id,
			(unsigned int) __vreg(vgt, offset));
		vgt_set_uevent(vgt, VGT_ENABLE_VGA);
		vgt_raise_request(pdev, VGT_REQUEST_UEVENT);
	}
	return true;
}

static bool err_int_r(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_read(vgt, offset, p_data, bytes);
	return rc;
}

static bool err_int_w(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_write(vgt, offset, p_data, bytes);
	return rc;
}

/*
 * Base reg information which is common on all platforms
 */
reg_attr_t vgt_base_reg_info[] = {

	/* -------render regs---------- */
{_REG_GTIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_GTIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_GTIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_GTISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_RCS_IMR, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_IMR, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_IMR, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_BCS_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_GEN7PLUS, NULL, NULL},
{_REG_VEBOX_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_GEN7PLUS, NULL, NULL},
/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
{0x14080, 4, F_VIRT, 0, D_SNB, NULL, NULL},
{_REG_RCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
{_REG_VCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
{_REG_BCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
{_REG_RCS_BB_PREEMPT_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_CCID, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{0x12198, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

{_REG_CXT_SIZE, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_GEN7_CXT_SIZE, 4, F_WA, 0, D_ALL, NULL, NULL},

{_REG_RCS_FBC_RT_BASE_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_RCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_RCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_RCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_RCS_CTL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_VCS_CTL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_BCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_BCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_BCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_BCS_CTL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_RCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_GFX_MODE, 4, F_RDR_MODE, 0, D_SNB, NULL, NULL},
{_REG_RCS_GFX_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_VCS_MFX_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_BCS_BLT_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_VEBOX_MODE, 4, F_RDR_MODE, 0, D_HSW, NULL, NULL},
{_REG_ARB_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_RCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_RCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_GT_MODE, 4, F_RDR_MODE, 0, D_SNB, NULL, NULL},
{_REG_GT_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_CACHE_MODE_0, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_0_IVB, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_1_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_RCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
/* TODO: need a handler */
{_REG_RCS_PP_DIR_BASE_READ, 4, F_RDR_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_RCS_PP_DIR_BASE_WRITE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_RCS_PP_DIR_BASE_IVB, 4, F_RDR_ADRFIX, 0xFFFFF000, D_GEN7PLUS, NULL, NULL},
{_REG_VCS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VECS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_RCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VECS_PP_DCLV, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_RBSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RVSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BVSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VBSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},

	/* -------display regs---------- */
{_REG_DEIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_DEIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_DEIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_DEISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SDEIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_SDEIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_SDEIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_SDEISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DE_RRMR, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEACONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEADSL, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPEA_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},
{_REG_PIPEASTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEBCONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEBDSL, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPEB_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},
{_REG_PIPEBSTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CURABASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{0x700AC, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_CURACNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_3, 4, F_DPY, 0, D_ALL, NULL, NULL},
/* FIXME: it contains physical address */
{0x7008C, 4, F_DPY, 0, D_ALL, NULL, vgt_error_handler},
{_REG_CURBBASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{0x700EC, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_CURBCNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURBPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x700D0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x700D4, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x700D8, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x700DC, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_DSPACNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DSPASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DSPALINOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPASTRIDE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPAPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPASIZE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPATILEOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBCNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DSPBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DSPBLINOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBSTRIDE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBSIZE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPBTILEOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_DVSACNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DVSASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DVSALINOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSAPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSASIZE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSATILEOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSAKEYVAL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSAKEYMSK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSAKEYMAXVAL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSASCALE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBCNTR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DVSBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DVSBLINOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBPOS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBSIZE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBTILEOFF, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBKEYVAL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBKEYMSK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBKEYMAXVAL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DVSBSCALE, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_LGC_PALETTE_A, 4*256, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_LGC_PALETTE_B, 4*256, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEASRC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_BCLRPAT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEBSRC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_BCLRPAT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_DATA_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_DATA_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_DATA_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_DATA_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PFA_CTL_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PFB_CTL_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PFA_WIN_SZ, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PFB_WIN_SZ, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PFA_WIN_POS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PFB_WIN_POS, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_WM0_PIPEA_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM0_PIPEB_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM0_PIPEC_IVB, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_WM1_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM2_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM3_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM1S_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_WM2S_LP_IVB, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_WM3S_LP_IVB, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},

{_REG_HISTOGRAM_THRSH, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_BLC_PWM_CPU_CTL2, 4, F_MGMT, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_CPU_CTL, 4, F_MGMT, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_PCH_CTL1, 4, F_MGMT, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_PCH_CTL2, 4, F_MGMT, 0, D_ALL, NULL, NULL},

{_REG_PCH_GMBUS0, 4*4, F_VIRT, 0, D_ALL, gmbus_mmio_read, gmbus_mmio_write},
{_REG_PCH_GPIOA, 6*6, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DP_BUFTRANS, 0x28, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_DPB_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},
{_REG_PCH_DPC_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},
{_REG_PCH_DPD_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},
{_REG_PCH_ADPA, 4, F_DPY, 0, D_ALL, pch_adpa_mmio_read, pch_adpa_mmio_write},
{_REG_DP_A_CTL, 4, F_DPY, 0, D_ALL, dp_ctl_mmio_read, dp_ctl_mmio_write},
{_REG_DP_B_CTL, 4, F_DPY, 0, D_ALL, dp_ctl_mmio_read, dp_ctl_mmio_write},
{_REG_DP_C_CTL, 4, F_DPY, 0, D_ALL, dp_ctl_mmio_read, dp_ctl_mmio_write},
{_REG_DP_D_CTL, 4, F_DPY, 0, D_ALL, dp_ctl_mmio_read, dp_ctl_mmio_write},
{_REG_HDMI_B_CTL, 4, F_DPY, 0, D_ALL, hdmi_ctl_mmio_read, hdmi_ctl_mmio_write},
{_REG_HDMI_C_CTL, 4, F_DPY, 0, D_ALL, hdmi_ctl_mmio_read, hdmi_ctl_mmio_write},
{_REG_HDMI_D_CTL, 4, F_DPY, 0, D_ALL, hdmi_ctl_mmio_read, hdmi_ctl_mmio_write},
{_REG_TRANSACONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_REG_TRANSBCONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_REG_FDI_RXA_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXB_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXA_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXB_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},

{_REG_TRANS_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_TRANSA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DATA_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DATA_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DP_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DP_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DP_LINK_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DP_LINK_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_TRANSA_VIDEO_DIP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_VIDEO_DIP_DATA, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_VIDEO_DIP_GCP, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_DP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_VIDEO_DIP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_VIDEO_DIP_DATA, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_VIDEO_DIP_GCP, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_DP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_CTL, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_DATA, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_GCP, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_TRANSC_DP_CTL, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},

{_REG_FDI_RXA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_FDI_RXB_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_FDI_RXA_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_FDI_RXA_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_FDI_RXB_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_FDI_RXB_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PCH_PP_CONTROL, 4, F_DPY, 0, D_ALL, NULL, pch_pp_control_mmio_write},
{_REG_PCH_PP_DIVISOR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_PP_STATUS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_LVDS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_DPLL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_DPLL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_FPA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_FPA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_FPB0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_FPB1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_DREF_CONTROL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_RAWCLK_FREQ, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_DPLL_SEL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	/* Linux defines as PP_ON_DEPLAY/PP_OFF_DELAY. Not in spec */
{0x61208, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6120c, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_PP_ON_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_PP_OFF_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0xE651C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE661C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE671C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE681C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE6C04, 4, F_DPY, 0, D_ALL,
	dpy_reg_mmio_read_2, NULL},
{0xE6E1C, 4, F_DPY, 0, D_ALL,
	dpy_reg_mmio_read_3, NULL},
{_REG_SHOTPLUG_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_LCPLL_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_FUSE_STRAP, 4, F_DPY, 0, D_HSW, NULL, NULL},

	/* -------pm regs---------- */
{_REG_PMIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_PMIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_PMIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_PMISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_FORCEWAKE, 4, F_VIRT, 0, D_ALL, NULL, force_wake_write},
{_REG_FORCEWAKE_ACK, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_GT_CORE_STATUS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_GT_THREAD_STATUS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_GTFIFODBG, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_GTFIFO_FREE_ENTRIES, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_MUL_FORCEWAKE, 4, F_VIRT, 0, D_ALL, NULL, mul_force_wake_write},
{_REG_MUL_FORCEWAKE_ACK, 4, F_VIRT, 0, D_SNB|D_IVB, mul_force_wake_ack_read, NULL},
{_REG_FORCEWAKE_ACK_HSW, 4, F_VIRT, 0, D_HSW, mul_force_wake_ack_read, NULL},
{_REG_ECOBUS, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC_CONTROL, 4, F_PM, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{_REG_RC_STATE, 4, F_PM, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{_REG_RPNSWREQ, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC_VIDEO_FREQ, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_TIMEOUT, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_INTERRUPT_LIMITS, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RPSTAT1, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_CONTROL, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_UP_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_UP_EI, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_UP, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_PREV_UP, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_DOWN_EI, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_DOWN, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_PREV_DOWN, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_UP_EI, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_EI, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RP_IDLE_HYSTERSIS, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC1_WAKE_RATE_LIMIT, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC6_WAKE_RATE_LIMIT, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC6pp_WAKE_RATE_LIMIT, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC_EVALUATION_INTERVAL, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC_IDLE_HYSTERSIS, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC_SLEEP, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC1e_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC6_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC6p_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_RC6pp_THRESHOLD, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_PMINTRMSK, 4, F_PM, 0, D_ALL, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL1, 4, F_PM, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL2, 4, F_PM, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL3, 4, F_PM, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL4, 4, F_PM, 0, D_HSW, NULL, NULL},

	/* -------miscellaneous regs-------- */
{_REG_GEN6_GDRST, 4, F_VIRT, 0, D_ALL, NULL, gen6_gdrst_mmio_write},
{_REG_FENCE_0_LOW, 0x80, F_VIRT, 0, D_ALL, fence_mmio_read, fence_mmio_write},
{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CPU_VGACNTRL, 4, F_BOOTTIME, 0, D_ALL, vga_control_r, vga_control_w},
	/* MCHBAR, suppose read-only */
{_REG_MCHBAR_MIRROR, 0x40000, F_VIRT, 0, D_ALL, NULL, NULL},

	/* -------non-audited regs--------- */
	/*
	 * below registers require next step audit:
	 *   - some are workaround registers which we allow
	 *	 for pReg access from any VM
	 *   - some are marked as virtualized for now, if they
	 *	 don't fall into coarse-grained ranges, to avoid
	 *	 regression.
	 *   - there are also boottime regs in this list. In the
	 *	 end, all boottime and workaround types should be
	 *	 removed and then placed under mgmt category, meaning
	 *	 controlled by dom0 only
	 */
{_REG_TILECTL, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_DISP_ARB_CTL, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_DISP_ARB_CTL2, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x2050, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x12050, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x22050, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_DISPLAY_CHICKEN_BITS_1, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},
{_REG_DISPLAY_CHICKEN_BITS_2, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},
{_REG_DSPCLK_GATE_D, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},
{_REG_SOUTH_CHICKEN1, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_SOUTH_CHICKEN2, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_CHICKEN1, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_CHICKEN1, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_SOUTH_DSPCLK_GATE_D, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_CHICKEN2, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_CHICKEN2, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x3c, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},
{_REG_UCG_CTL1, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},
{_REG_UCG_CTL2, 4, F_BOOTTIME, 0, D_ALL, NULL, NULL},

{0x860, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_RC_PWRCTX_MAXCNT, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN1, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN2, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN3, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x20d4, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x2088, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x20e4, 4, F_WA, 0, D_GEN7PLUS, NULL, NULL},
/* no definition on this. from Linux */
{_REG_GEN3_MI_ARB_STATE, 4, F_WA, 0, D_SNB, NULL, NULL},
{_REG_RCS_ECOSKPD, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x121d0, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_BCS_ECOSKPD, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x41d0, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x22ac, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_VFSKPD, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x2700, 4, F_WA, 0, D_SNB, NULL, NULL},
{_REG_ECOCHK, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_GAC_ECOCHK, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_2D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_3D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_3D_CG_DIS2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7004, 4, F_VIRT, 0, D_SNB, NULL, NULL},
{0x7118, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7180, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7c00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SNPCR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_MBCTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x911c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x9120, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_GAB_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
/*
 * framebuffer compression is disabled for now
 * until it's handled at display context switch
 * and we figure out how stolen memory should be virtualized (FBC needs use
 * stolen memory).
 */
{_REG_DPFC_CB_BASE, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CONTROL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_RECOMP_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CPU_FENCE_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CONTROL_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CPU_FENCE_OFFSET_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{0x48800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_1, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_4, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_5, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_COEFFICIENTS_6, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_MODE, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_HIGH_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_LOW_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_1, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_4, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_5, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_COEFFICIENTS_6, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_MODE, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_HIGH_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_LOW_COLOR_CHANNEL_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SWF, 0x110, F_VIRT, 0, D_SNB, NULL, NULL},
{_REG_SWF, 0x90, F_VIRT, 0, D_GEN7PLUS, NULL, NULL},
{0x60110, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x61110, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x70400, 4, F_WA, 0, D_ALL, NULL, NULL},
{0xce044, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6500, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6504, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6600, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6604, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6700, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6704, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6804, 4, F_VIRT, 0, D_ALL, NULL, NULL},
/* now looks gmbus handler can't cover 4/5 ports */
{_REG_PCH_GMBUS4, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_PCH_GMBUS5, 4, F_WA, 0, D_ALL, NULL, NULL},

{_REG_SUPER_QUEUE_CONFIG, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_MISC_CLOCK_GATING, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc810, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc81c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc828, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc834, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc0c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc24, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd000, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd018, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd024, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd034, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_GTDRIVER_MAILBOX_INTERFACE, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_GTDRIVER_MAILBOX_DATA0, 4, F_WA, 0, D_ALL, NULL, NULL},
{0x13812c, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_GTT_FAULT_STATUS, 4, F_WA, 0, D_ALL, err_int_r, err_int_w},
/* HSW */
{0x120010, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x9008, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_DP_A_HOTPLUG_CNTL, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_VECS_TAIL, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_VECS_HEAD, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_VECS_START, 4, F_WA, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_VECS_CTL, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_GFX_FLSH_CNT, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_GEN7_COMMON_SLICE_CHICKEN1, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_GEN7_L3CNTLREG1, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_GEN7_L3_CHICKEN_MODE_REGISTER, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_GEN7_SQ_CHICKEN_MBCUNIT_CONFIG, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_WM_DBG, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x2020, 4, F_WA, 0, D_ALL, NULL, NULL},
{_REG_IER, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x20e8, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x2214, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x2358, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x8000, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x8008, 4, F_WA, 0, D_HSW, NULL, NULL},
{0xb008, 4, F_WA, 0, D_HSW, NULL, NULL},
{0xb208, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a028, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a048, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a09c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a0a8, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a0c0, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1a134, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x320f0, 8, F_WA, 0, D_HSW, NULL, NULL},
{0x320fc, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x32230, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x44084, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x4408c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x45260, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x45270, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_SPLL_CTL, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_WRPLL_CTL1, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_WRPLL_CTL2, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIC, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIE, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_TRANS_CLK_SEL_A, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x46408, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x46508, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x49040, 0xc, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{0x49080, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{0x49090, 0x14, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{0x49180, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x49190, 0x14, F_WA, 0, D_HSW, NULL, NULL},
{0x49280, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x49290, 0x14, F_WA, 0, D_HSW, NULL, NULL},
{0x4a400, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x4a480, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x6002c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x60200, 4, F_WA, 0, D_HSW, NULL, NULL},
{0xc2014, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_SBI_ADDR, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_SBI_DATA, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_SBI_CTL_STAT, 4, F_WA, 0, D_HSW, NULL, NULL},
{0xc6020, 4, F_WA, 0, D_HSW, NULL, NULL},
{0xf200c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x1082c0, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x13005c, 4, F_WA, 0, D_HSW, NULL, NULL},
{_REG_TRANS_DDI_FUNC_CTL_A, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{0x64010, 8, F_WA, 0, D_HSW, NULL, NULL},
{_REG_DP_TP_CTL_A, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_B, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_TP_CTL_B, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_C, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_TP_CTL_C, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_D, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{_REG_DP_TP_CTL_D, 4, F_BOOTTIME, 0, D_HSW, NULL, NULL},
{0x64400, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x64440, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x64444, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x64e00, 0x50, F_WA, 0, D_HSW, NULL, NULL},
{0x64e60, 0x50, F_WA, 0, D_HSW, NULL, NULL},
{0x64ec0, 0x50, F_WA, 0, D_HSW, NULL, NULL},
{0x64f20, 0x50, F_WA, 0, D_HSW, NULL, NULL},
{0x64f80, 0x50, F_WA, 0, D_HSW, NULL, NULL},
{0x65000, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x650c0, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x6661c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x66c00, 8, F_WA, 0, D_HSW, NULL, NULL},
{0x6f400, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x70280, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x7029c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x70304, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x71080, 8, F_WA, 0, D_HSW, NULL, NULL},
{0x71280, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x7129c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x71304, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x72008, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x72080, 8, F_WA, 0, D_HSW, NULL, NULL},
{0x72280, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x7229c, 4, F_WA, 0, D_HSW, NULL, NULL},
{0x72304, 4, F_WA, 0, D_HSW, NULL, NULL},
};

bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev)
{
	printk("post mmio hooks initialized\n");

	if (pdev->enable_ppgtt) {
		vgt_dbg("Hook up PPGTT register handlers\n");
		/* trap PPGTT base register */
		reg_update_handlers(pdev, _REG_RCS_PP_DIR_BASE_IVB, 4,
				rcs_pp_dir_base_read, rcs_pp_dir_base_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DIR_BASE, 4,
				bcs_pp_dir_base_read, bcs_pp_dir_base_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DIR_BASE, 4,
				vcs_pp_dir_base_read, vcs_pp_dir_base_write);

		reg_update_handlers(pdev, _REG_RCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);

		/* XXX cache register? */
		/* PPGTT enable register */
		reg_update_handlers(pdev, _REG_RCS_GFX_MODE_IVB, 4,
				rcs_gfx_mode_read, rcs_gfx_mode_write);
		reg_update_handlers(pdev, _REG_BCS_BLT_MODE_IVB, 4,
				bcs_blt_mode_read, bcs_blt_mode_write);
		reg_update_handlers(pdev, _REG_VCS_MFX_MODE_IVB, 4,
				vcs_mfx_mode_read, vcs_mfx_mode_write);

		if (pdev->is_haswell) {
			reg_update_handlers(pdev, _REG_VECS_PP_DIR_BASE, 4,
					vecs_pp_dir_base_read,
					vecs_pp_dir_base_write);
			reg_update_handlers(pdev, _REG_VECS_PP_DCLV, 4,
					pp_dclv_read, pp_dclv_write);
			reg_update_handlers(pdev, _REG_VEBOX_MODE, 4,
					vecs_mfx_mode_read,
					vecs_mfx_mode_write);
		}
	}

#ifdef VGT_DEBUGFS_DUMP_FB
	reg_update_handlers(pdev, _REG_DSPASURF, 4, NULL, dspsurf_mmio_write);
	reg_update_handlers(pdev, _REG_DSPBSURF, 4, NULL, dspsurf_mmio_write);
#endif
	return true;
}

int vgt_get_base_reg_num()
{
	return ARRAY_NUM(vgt_base_reg_info);
}
