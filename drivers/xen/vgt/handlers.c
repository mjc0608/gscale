/*
 * MMIO virtualization handlers
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

#include <linux/delay.h>
#include <linux/acpi.h>

#include <xen/interface/vcpu.h>
#include <xen/interface/hvm/hvm_op.h>
#include <xen/fb_decoder.h>

#include "vgt.h"

static bool vgt_error_handler(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	printk("vGT: reg (%x) needs special handler\n", offset);
	ASSERT(0);
	return true;
}

static bool gmbus_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return vgt_i2c_handle_gmbus_read(vgt, offset, p_data, bytes);
}

static bool gmbus_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return vgt_i2c_handle_gmbus_write(vgt, offset, p_data, bytes);
}

static bool fence_mmio_read(struct vgt_device *vgt, unsigned int off,
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

static bool fence_mmio_write(struct vgt_device *vgt, unsigned int off,
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

static bool force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	data = (*(uint32_t*) p_data) & 1 ;

	vgt_dbg("VM%d write register FORCE_WAKE with %x\n", vgt->vm_id, data);

	if (IS_HSW(vgt->pdev)) {
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

static bool mul_force_wake_ack_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(u32 *)p_data = __vreg(vgt, offset);
	return true;
}

static bool mul_force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data, mask, wake, old_wake, new_wake;

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

	if (IS_HSW(vgt->pdev)) {
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

static bool rc_state_ctrl_1_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_1 with 0x%x\n", vgt->vm_id, data);

	if ( (data & _REGBIT_RC_HW_CTRL_ENABLE) && (data & (_REGBIT_RC_RC6_ENABLE
					| _REGBIT_RC_DEEPEST_RC6_ENABLE	| _REGBIT_RC_DEEP_RC6_ENABLE) ) )
		set_vRC_to_C6(vgt);
	else
		set_vRC_to_C0(vgt);

	return default_mmio_write(vgt, offset, p_data, bytes);

}

static bool gen6_gdrst_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	uint32_t data;

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

	show_debug(vgt->pdev);

	/* after this point, driver should re-initialize the device */
	vgt->warn_untrack = 1;
	return true;
}

static bool rrmr_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t old_rrmr, new_rrmr, new_physical_rrmr;
	struct pgt_device *pdev = vgt->pdev;

	old_rrmr = __vreg(vgt, offset);
	new_physical_rrmr = new_rrmr = *(u32 *)p_data;

	__vreg(vgt, offset) = new_rrmr;

	if (old_rrmr != new_rrmr) {
		new_physical_rrmr = vgt_recalculate_mask_bits(pdev, offset);
		VGT_MMIO_WRITE(pdev, offset, new_physical_rrmr);
	}

	vgt_info("RRMR: VM%d: old (%x), new (%x), new_physical (%x)\n",
		vgt->vm_id, old_rrmr, new_rrmr, new_physical_rrmr);
	return true;
}

static bool pch_pp_control_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;
	uint32_t reg;
	union PCH_PP_CONTROL pp_control;
	union PCH_PP_STAUTS pp_status;

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

static bool transaconf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t reg;
	union _TRANS_CONFIG config;

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

static bool shotplug_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t val = *(vgt_reg_t *)p_data;
	vgt_reg_t sticky_mask = _REGBIT_DP_B_STATUS |
				_REGBIT_DP_C_STATUS |
				_REGBIT_DP_D_STATUS;

	__vreg(vgt, offset) = (val & ~sticky_mask) |
				(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(val & sticky_mask);

	__sreg(vgt, offset) = val;

	if (reg_hw_access(vgt, offset)) {
		vgt_reg_t enable_mask = _REGBIT_DP_B_ENABLE |
					_REGBIT_DP_C_ENABLE |
					_REGBIT_DP_D_ENABLE;

		if (~(val & enable_mask) & enable_mask) {
			vgt_warn("vGT(%d): Is trying to disable HOTPLUG"
			" with writing 0x%x to SHOTPLUG_CTL!\n",
			vgt->vgt_id, val);
		}
		/* do not let display owner clear the status bits.
		 * vgt driver will do so in interrupt handling.
		 */
		val &= ~sticky_mask;
		VGT_MMIO_WRITE(vgt->pdev, offset, val);
	}

	return true;
}

/* Pipe Frame Count */
static bool pipe_frmcount_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum vgt_pipe pipe;
	bool rc = default_passthrough_mmio_read(vgt, offset, p_data, bytes);

	if (rc && !is_current_display_owner(vgt)) {
		vgt_reg_t count = *(vgt_reg_t *)p_data;
		pipe = VGT_FRMCOUNTPIPE(offset);
		ASSERT(pipe >= PIPE_A && pipe < I915_MAX_PIPES);
		if ((vgt->pipe_last_vblank[pipe] != 0) && (count < vgt->pipe_last_vblank[pipe])) {
			*(vgt_reg_t *)p_data = vgt->pipe_last_vblank[pipe] + 1;
		}
		vgt->pipe_last_vblank[pipe] = *(vgt_reg_t *)p_data;
	}
	return rc;
}

/* Pipe Display Scan Line*/
static bool pipe_dsl_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_passthrough_mmio_read(vgt, offset, p_data, bytes);
}

static bool dpy_reg_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = (1<<17);

	return true;
}

static bool dpy_reg_mmio_read_2(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = 3;

	return true;
}

static bool dpy_reg_mmio_read_3(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = (0x2F << 16);

	return true;
}

static int pp_mmio_to_ring_id(unsigned int reg)
{
	int ring_id;

	switch (reg) {
	case _REG_RCS_PP_DIR_BASE_IVB:
	case _REG_RCS_GFX_MODE_IVB:
		ring_id = RING_BUFFER_RCS;
		break;
	case _REG_BCS_PP_DIR_BASE:
	case _REG_BCS_BLT_MODE_IVB:
		ring_id = RING_BUFFER_BCS;
		break;
	case _REG_VCS_PP_DIR_BASE:
	case _REG_VCS_MFX_MODE_IVB:
		ring_id = RING_BUFFER_VCS;
		break;
	case _REG_VECS_PP_DIR_BASE:
	case _REG_VEBOX_MODE:
		ring_id = RING_BUFFER_VECS;
		break;
	default:
		ring_id = -1;
		break;
	}

	ASSERT(ring_id != -1);
	return ring_id;
}

static bool pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	int ring_id = pp_mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;

	*(u32 *)p_data = v_info->base;

	vgt_dbg("<ring-%d>PP_DIR_BASE read: 0x%x\n", ring_id, v_info->base);
	return true;
}

static bool pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;
	int ring_id = pp_mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &vgt->rb[ring_id].sring_ppgtt_info;

	vgt_dbg("<ring-%d> PP_DIR_BASE write: 0x%x\n", ring_id, base);

	/* convert base which is in form of bit 31-16 in 64bytes cachelines,
	 * it turns out to be ((((base >> 16) * 64) >> 2) << PAGE_SHIFT), which
	 * is just base. */
	v_info->base = base;
	s_info->base = mmio_g2h_gmadr(vgt, off, v_info->base);
	__vreg(vgt, off) = base;
	__sreg(vgt, off) = s_info->base;

	vgt->rb[ring_id].has_ppgtt_base_set = 1;

	vgt_try_setup_ppgtt(vgt);
	return true;
}

static bool pp_dclv_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	*(u32 *)p_data = 0xFFFFFFFF;
	return true;
}

static bool pp_dclv_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 dclv = *(u32 *)p_data;
	__vreg(vgt, off) = dclv;
	__sreg(vgt, off) = dclv;

	/* TODO: forward to pReg? */
	vgt_dbg("PP_DCLV write: 0x%x\n", dclv);
	return true;
}

/* TODO: there are other mode control bits in the registers */
static bool ring_pp_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	int ring_id = pp_mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;

	*(u32 *)p_data = v_info->mode;
	vgt_dbg("<ring-%d>GFX_MODE read: 0x%x\n", ring_id, v_info->mode);
	return true;
}

static bool ring_pp_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;
	int ring_id = pp_mmio_to_ring_id(off);

	vgt_dbg("<ring-%d>GFX_MODE write: 0x%x\n", ring_id, mode);

	if (ring_id == RING_BUFFER_VECS)
		vgt->vebox_support = 1;

	ring_ppgtt_mode(vgt, ring_id, off, mode);
	return true;
}

/* FIXME: add EDID virtualization in the future
 */
static bool dp_aux_ch_ctl_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_edid_data_t **pedid = NULL;
	VGT_DP_PORTS_IDX port_idx;

	ASSERT(bytes == 4);
	ASSERT((offset & (bytes - 1)) == 0);

	rc = default_mmio_read(vgt, offset, p_data, bytes);

	port_idx = vgt_get_dp_port_idx(offset);
	switch (port_idx) {
	case VGT_DPA_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[VGT_DP_A];
		break;
	case VGT_DPB_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[VGT_DP_B];
		break;
	case VGT_DPC_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[VGT_DP_C];
		break;
	case VGT_DPD_IDX:
		pedid = (vgt_edid_data_t **) &vgt->vgt_edids[VGT_DP_D];
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

static bool dpy_trans_ddi_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t new_data;
	uint32_t old_data;
	int i;

	old_data = __vreg(vgt, offset);
	default_mmio_write(vgt, offset, p_data, bytes);

	new_data = *((uint32_t *)p_data);

	/* if it is to enable this pipe, then rebuild the mapping for this pipe*/
	if (vgt->vm_id == 0) {
		/*when dom0 change the physical pipe/port connection,
		we need to rebuild pipe mapping for the vgt device.*/
		for (i = 0; i < VGT_MAX_VMS; ++ i) {
			struct vgt_device *vgt_virtual = vgt->pdev->device[i];
			if (!vgt_virtual || vgt_virtual->vm_id == 0)
				continue;
			update_pipe_mapping(vgt_virtual, offset, new_data);
		}

	} else {
		rebuild_pipe_mapping(vgt,  offset, new_data, old_data);
	}


	return true;
}

static bool pipe_conf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc;
	unsigned int reg;
	uint32_t wr_data;

	reg = offset & ~(bytes - 1);

	wr_data = *((uint32_t *)p_data);
	/* vreg status will be updated when when read hardware status */
	if (!reg_hw_access(vgt, reg)) {
		if (wr_data & _REGBIT_PIPE_ENABLE)
			wr_data |= _REGBIT_PIPE_STAT_ENABLED;
		else if (!(wr_data & _REGBIT_PIPE_ENABLE))
			wr_data &= ~_REGBIT_PIPE_STAT_ENABLED;
	}

	rc = default_mmio_write(vgt, offset, &wr_data, bytes);
	vgt_manage_emul_dpy_events(vgt->pdev);
	return rc;
}

static bool ddi_buf_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc;
	vgt_reg_t reg_val;

	reg_val = *(vgt_reg_t *)p_data;

	// set the fully virtualized RO bit with its original value
	reg_val = (reg_val & ~_DDI_BUFCTL_DETECT_MASK)
		| (__vreg(vgt, offset) & _DDI_BUFCTL_DETECT_MASK);

	rc = default_mmio_write(vgt, offset, &reg_val, bytes);

	// clear the auto_training done bit
	if ((offset == _REG_DDI_BUF_CTL_E) &&
		(!(reg_val & _REGBIT_DDI_BUF_ENABLE))) {
		if (!reg_hw_access(vgt, offset)) {
			__vreg(vgt, _REG_DP_TP_STATUS_E) &=
				~_REGBIT_DP_TP_STATUS_AUTOTRAIN_DONE;
		}
	}

	return rc;
}

static bool fdi_rx_iir_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t wr_data, old_iir;
	bool rc;

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

static bool fdi_auto_training_started(struct vgt_device *vgt)
{
	bool rc = false;
	vgt_reg_t ddi_buf_ctl = __vreg(vgt, _REG_DDI_BUF_CTL_E);
	vgt_reg_t rx_ctl = __vreg(vgt, _REG_FDI_RXA_CTL);
	vgt_reg_t tx_ctl = __vreg(vgt, _REG_DP_TP_CTL_E);

	if ((ddi_buf_ctl & _REGBIT_DDI_BUF_ENABLE) &&
		(rx_ctl & _REGBIT_FDI_RX_ENABLE) &&
		(rx_ctl & _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE) &&
		(tx_ctl & _REGBIT_DP_TP_ENABLE) &&
		(tx_ctl & _REGBIT_DP_TP_FDI_AUTO_TRAIN_ENABLE)) {
			rc = true;
	}

	return rc;
}

/* FIXME: this function is highly platform-dependent (SNB + CPT) */
static bool check_fdi_rx_train_status(struct vgt_device *vgt,
		enum vgt_pipe pipe, unsigned int train_pattern)
{
	unsigned int fdi_rx_imr, fdi_tx_ctl, fdi_rx_ctl;
	unsigned int fdi_rx_check_bits, fdi_tx_check_bits;
	unsigned int fdi_rx_train_bits, fdi_tx_train_bits;
	unsigned int fdi_iir_check_bits;

	fdi_rx_imr = VGT_FDI_RX_IMR(pipe);
	fdi_tx_ctl = VGT_FDI_TX_CTL(pipe);
	fdi_rx_ctl = VGT_FDI_RX_CTL(pipe);

	if (train_pattern == FDI_LINK_TRAIN_PATTERN_1) {
		fdi_rx_train_bits =_REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT;
		fdi_tx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_1;
		fdi_iir_check_bits = _REGBIT_FDI_RX_BIT_LOCK;
	} else if (train_pattern == FDI_LINK_TRAIN_PATTERN_2) {
		fdi_rx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_2_CPT;
		fdi_tx_train_bits = _REGBIT_FDI_LINK_TRAIN_PATTERN_2;
		fdi_iir_check_bits = _REGBIT_FDI_RX_SYMBOL_LOCK;
	} else {
		BUG();
	}

	fdi_rx_check_bits = _REGBIT_FDI_RX_ENABLE | fdi_rx_train_bits;
	fdi_tx_check_bits = _REGBIT_FDI_TX_ENABLE | fdi_tx_train_bits;

	/* If imr bit not been masked */
	if (((__vreg(vgt, fdi_rx_imr) & fdi_iir_check_bits) == 0)
		&& ((__vreg(vgt, fdi_tx_ctl)
			& fdi_tx_check_bits) == fdi_tx_check_bits)
		&& ((__vreg(vgt, fdi_rx_ctl)
			& fdi_rx_check_bits) == fdi_rx_check_bits))
		return true;
	else
		return false;
}

static bool update_fdi_rx_iir_status(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum vgt_pipe pipe;
	unsigned int reg, fdi_rx_iir;
	bool rc;

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

		case _REG_FDI_RXC_CTL:
		case _REG_FDI_TXC_CTL:
		case _REG_FDI_RXC_IMR:
			pipe = PIPE_C;
			break;

		default:
			BUG();
	}

	fdi_rx_iir = VGT_FDI_RX_IIR(pipe);

	rc = default_mmio_write(vgt, offset, p_data, bytes);
	if (!reg_hw_access(vgt, reg)) {
		if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN_1))
			__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_BIT_LOCK;
		if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN_2))
			__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_SYMBOL_LOCK;
		if (offset == _REG_FDI_RXA_CTL) {
			if (fdi_auto_training_started(vgt))
				__vreg(vgt, _REG_DP_TP_STATUS_E) |=
					_REGBIT_DP_TP_STATUS_AUTOTRAIN_DONE;
		}
	}
	return rc;
}

#define DP_TP_CTL_10_8_MASK	0x00000700
#define DP_TP_CTL_8_SHIFT	0x8
#define DP_TP_STATUS_25_SHIFT	25

static bool dp_tp_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	enum vgt_port port;
	unsigned int dp_tp_status_reg, val;
	vgt_reg_t ctl_val;
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		port = VGT_DP_TP_CTL_PORT(offset);
		ctl_val = __vreg(vgt, offset);
		val = (ctl_val & DP_TP_CTL_10_8_MASK) >> DP_TP_CTL_8_SHIFT;

		if (val == 0x2) {
			dp_tp_status_reg = VGT_DP_TP_STATUS(port);
			__vreg(vgt, dp_tp_status_reg) |= (1 << DP_TP_STATUS_25_SHIFT);
			__sreg(vgt, dp_tp_status_reg) = __vreg(vgt, dp_tp_status_reg);
		}
	}

	return rc;
}

#define BIT_27		27
#define BIT_26		26
#define BIT_24		24

static bool dp_tp_status_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_reg_t reg_val;
	vgt_reg_t sticky_mask;

	reg_val = *((vgt_reg_t *)p_data);
	sticky_mask = (1 << BIT_27) | (1 << BIT_26) | (1 << BIT_24);

	__vreg(vgt, offset) = (reg_val & ~sticky_mask) |
				(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(reg_val & sticky_mask);

	__sreg(vgt, offset) = reg_val;

	if (reg_hw_access(vgt, offset)) {
		VGT_MMIO_WRITE(vgt->pdev, offset, reg_val);
	}

	return rc;
}

/*
 * DAC_CTL is special regarding to that its bits containing multiple
 * policies:
 *	- CRT control bits like enabling, transcoder selection belong
 *	  to display owner
 *	- hotplug status bits are fully virtualized like other interrupt
 *	  status bits
 *	- force hotplug trigger bit is also fully virtualized in the MMIO
 *	  write. XenGT will determin what status bits will look like based
 *	  on the monitor connection information. From driver's point of
 *	  view, the force hotplug will always succeed immediately(bit is
 *	  cleared immediately) and status bits contains the expected
 *	  information.
 *
 * Let's take this as one example how this category may be abstracted
 * in the future
 */
static bool pch_adpa_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t adpa_value;

	reg = offset & ~(bytes - 1);

	/*
	 * the channel status bit is updated by interrupt handler,
	 * or the write handler.
	 */

	if (reg_hw_access(vgt, reg)) {

		adpa_value = VGT_MMIO_READ(vgt->pdev, _REG_PCH_ADPA);

		/* force trigger bit was fully virtualized. Should always be zero */
		ASSERT (!(adpa_value & _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER));

		__vreg(vgt, reg) &= _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER |
					_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
		__vreg(vgt, reg) |= adpa_value &
					~(_REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER |
					_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK);
	}

	memcpy(p_data, (char *)vgt->state.vReg + offset, bytes);

	return true;
}

static bool pch_adpa_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_reg_t old, new;
	struct pgt_device *pdev = vgt->pdev;

	new = *(vgt_reg_t *)p_data;
	old = __vreg(vgt, offset);

	/* Clear the bits of 'force hotplug trigger' and status because they
	 * will be fully virtualized. Other bits will be written to hardware.
	 */
	if (reg_hw_access(vgt, offset)) {
		VGT_MMIO_WRITE(pdev, _REG_PCH_ADPA, new &
				~(_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK |
				 _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER));
	}

	if (new & _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER) {

		if ((new & _REGBIT_ADPA_DAC_ENABLE)) {
			vgt_warn("HOTPLUG_FORCE_TRIGGER is set while VGA is enabled!\n");
		}

		/* emulate the status based on monitor connection information */
		new &= ~_REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER;

		if (test_bit(VGT_CRT, vgt->presented_ports))
			new |= _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
		else
			new &= ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK;
	} else {
		/* ignore the status bits in new value
		 * since they are read only actually
		 */
		new = (new & ~_REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK) |
			(old & _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK);
	}

	__vreg(vgt, offset) = __sreg(vgt, offset) = new;

	return true;
}

static bool dp_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t vreg_data;
	bool rc;

	vreg_data = *(vgt_reg_t *)p_data;

	// Keep the fully virtualized RO bit with its original value
	vreg_data = (vreg_data & ~_REGBIT_DP_PORT_DETECTED)
			| (__vreg(vgt, offset) & _REGBIT_DP_PORT_DETECTED);

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	return rc;
}

static bool hdmi_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t vreg_data;
	bool rc;

	vreg_data = *(vgt_reg_t *)p_data;

	// Keep the fully virtualized RO bit with its original value
	vreg_data = (vreg_data & ~_REGBIT_HDMI_PORT_DETECTED)
			| (__vreg(vgt, offset) & _REGBIT_HDMI_PORT_DETECTED);

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	return rc;
}

bool vgt_map_plane_reg(struct vgt_device *vgt, unsigned int reg, unsigned int *p_real_reg)
{
	enum vgt_pipe virtual_pipe;
	enum vgt_pipe real_pipe ;

	switch (reg)
	{
	case _REG_CURABASE:
	case _REG_CURACNTR:
	case _REG_CURAPOS:
	case _REG_DSPACNTR:
	case _REG_DSPASURF:
	case _REG_DSPASURFLIVE:
	case _REG_DSPALINOFF:
	case _REG_DSPASTRIDE:
	case _REG_DSPAPOS:
	case _REG_DSPASIZE:
	case _REG_DSPATILEOFF:
	case _REG_SPRASURF:
		real_pipe = vgt->pipe_mapping[0];
		virtual_pipe = PIPE_A;
		break;

	case _REG_CURBBASE_SNB:
	case _REG_CURBCNTR_SNB:
	case _REG_CURBPOS_SNB:
	case _REG_CURBBASE:
	case _REG_CURBCNTR:
	case _REG_CURBPOS:
	case _REG_DSPBCNTR:
	case _REG_DSPBSURF:
	case _REG_DSPBSURFLIVE:
	case _REG_DSPBLINOFF:
	case _REG_DSPBSTRIDE:
	case _REG_DSPBPOS:
	case _REG_DSPBSIZE:
	case _REG_DSPBTILEOFF:
	case _REG_SPRBSURF:
		real_pipe = vgt->pipe_mapping[1];
		virtual_pipe = PIPE_B;
		break;

	case _REG_CURCBASE:
	case _REG_CURCCNTR:
	case _REG_CURCPOS:
	case _REG_DSPCCNTR:
	case _REG_DSPCSURF:
	case _REG_DSPCSURFLIVE:
	case _REG_DSPCLINOFF:
	case _REG_DSPCSTRIDE:
	case _REG_DSPCPOS:
	case _REG_DSPCSIZE:
	case _REG_DSPCTILEOFF:
	case _REG_SPRCSURF:
		real_pipe = vgt->pipe_mapping[2];
		virtual_pipe = PIPE_C;
		break;

	default:
		vgt_warn("try to map mmio that is not plane related! reg = %x\n", reg);
		ASSERT(0);
	}

	if(real_pipe == I915_MAX_PIPES)
	{
		vgt_dbg("the mapping for pipe %d is not ready or created!\n", virtual_pipe);
		return false;
	}

	*p_real_reg = reg + 0x1000 * real_pipe - 0x1000 * virtual_pipe;

	return true;

}

static bool dpy_plane_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{

	*(vgt_reg_t *)p_data = __vreg(vgt, offset);

	return true;
}

static bool dpy_plane_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int real_offset;

	memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);
	memcpy ((char *)vgt->state.sReg + offset, p_data, bytes);
	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}
	return true;
}

static bool dpy_plane_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum vgt_pipe pipe = PIPE_A;
	vgt_reg_t new_plane_ctl;
	bool enable_plane = false;

	new_plane_ctl = *(vgt_reg_t *)p_data;
	pipe = VGT_DSPCNTRPIPE(offset);
	if ( (_PRI_PLANE_ENABLE & new_plane_ctl) &&  (_PRI_PLANE_ENABLE & __vreg(vgt, offset)) == 0) {
		enable_plane = true;
	}

	if (enable_plane && current_foreground_vm(vgt->pdev) == vgt) {
		if ( !(_REGBIT_PIPE_ENABLE & VGT_MMIO_READ(vgt->pdev, VGT_PIPECONF(vgt->pipe_mapping[pipe])))) {
			vgt_warn("enable panel fitting before pipe is enabled\n");
		}
		set_panel_fitting(vgt, pipe);
	}

	dpy_plane_mmio_write(vgt,offset, p_data,bytes);

	return true;
}


static bool pri_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct fb_notify_msg msg;

	dpy_plane_mmio_write(vgt, offset, p_data, bytes);

	msg.vm_id = vgt->vm_id;
	msg.pipe_id = VGT_DSPSURFPIPE(offset);
	vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	return true;
}

static bool spr_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct fb_notify_msg msg;

	dpy_plane_mmio_write(vgt, offset, p_data, bytes);

	msg.vm_id = vgt->vm_id;
	msg.pipe_id = VGT_SPRSURFPIPE(offset);
	vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	return true;
}

static bool dpy_modeset_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset) &&
		(*(vgt_reg_t *)p_data != __vreg(vgt, offset))) {

		vgt_warn("modeset mmio[0x%x] change value from 0x%x to 0x%x\n"
			 "\twhich is not supported. MMIO write is ignored!\n",
						offset,
						__vreg(vgt, offset),
						*(vgt_reg_t *)p_data);
	}

	return true;
}

static bool surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes, enum vgt_plane_type plane)
{
	vgt_reg_t surflive_val;
	unsigned int surf_reg = 0;
	enum vgt_pipe pipe;

	if (plane == PRIMARY_PLANE) {
		pipe = VGT_DSPSURFLIVEPIPE(offset);
		surf_reg = VGT_DSPSURF(pipe);
	} else if (plane == CURSOR_PLANE) {
		if (offset == _REG_CURBSURFLIVE_SNB) {
			surf_reg = _REG_CURBBASE_SNB;
		} else {
			pipe = VGT_CURSURFPIPE(offset);
			surf_reg = VGT_CURSURF(pipe);
		}
	} else if (plane == SPRITE_PLANE) {
		pipe = VGT_SPRSURFPIPE(offset);
		surf_reg = VGT_SPRSURF(pipe);
	} else {
		BUG();
	}

	surflive_val = __vreg(vgt, surf_reg);
	__vreg(vgt, offset) = __sreg(vgt, offset) = surflive_val;
	*(vgt_reg_t *)p_data = surflive_val;

	return true;
}

static bool pri_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, PRIMARY_PLANE);
}

static bool cur_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, CURSOR_PLANE);
}

static bool spr_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, SPRITE_PLANE);
}

static bool surflive_mmio_write (struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	/* surflive is readonly registers. ignore the write from driver*/
	return true;
}

static void dp_aux_ch_ctl_trans_done(struct vgt_device *vgt, vgt_reg_t value,
	 unsigned int reg, int len)
{
	/* mark transaction done */
	value |= _REGBIT_DP_AUX_CH_CTL_DONE;
	value &= ~_REGBIT_DP_AUX_CH_CTL_SEND_BUSY;
	value &= ~_REGBIT_DP_AUX_CH_CTL_RECV_ERR;
	value &= ~_REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR;
	/* message size */
	value &= ~(0xf << 20);
	value |= (len << 20);
	__vreg(vgt, reg) = value;
}

static void dp_aux_ch_ctl_link_training(struct vgt_dpcd_data *dpcd, uint8_t t)
{
	if ((t & DPCD_TRAINING_PATTERN_SET_MASK) == DPCD_TRAINING_PATTERN_1) {

		/* training pattern 1 for CR */
		/* set LANE0_CR_DONE, LANE1_CR_DONE */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_CR_DONE;
		/* set LANE2_CR_DONE, LANE3_CR_DONE */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_CR_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
		DPCD_TRAINING_PATTERN_2) {

		/* training pattern 2 for EQ */

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane0_1 */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_SYMBOL_LOCKED;

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane2_3 */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_SYMBOL_LOCKED;
		/* set INTERLANE_ALIGN_DONE */
		dpcd->data[DPCD_LANE_ALIGN_STATUS_UPDATED] |=
			DPCD_INTERLANE_ALIGN_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
		DPCD_LINK_TRAINING_DISABLED) {

		/* finish link training */
		/* set sink status as synchronized */
		dpcd->data[DPCD_SINK_STATUS] = DPCD_SINK_IN_SYNC;
	}

}

static bool dp_aux_ch_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg = 0;
	vgt_reg_t value = *(vgt_reg_t *)p_data;
	int msg, addr, ctrl, op, len;
	vgt_edid_data_t *edid = NULL;
	struct vgt_dpcd_data *dpcd = NULL;
	VGT_DP_PORTS_IDX port_idx = vgt_get_dp_port_idx(offset);

	ASSERT(bytes == 4);
	ASSERT((offset & (bytes - 1)) == 0);

	reg = offset & ~(bytes - 1);

	default_mmio_write(vgt, offset, p_data, bytes);

	/* HW access had been handled by default_mmio_write() */
	if (reg_hw_access(vgt, reg))
		return true;

	if (reg != _REG_DPA_AUX_CH_CTL &&
	    reg != _REG_PCH_DPB_AUX_CH_CTL &&
	    reg != _REG_PCH_DPC_AUX_CH_CTL &&
	    reg != _REG_PCH_DPD_AUX_CH_CTL) {
		/* write to the data registers */
		return true;
	}

	if (!(value & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY)) {
		/* just want to clear the sticky bits */
		__vreg(vgt, reg) = 0;
		return true;
	}

	switch (port_idx) {
	case VGT_DPA_IDX:
		edid = vgt->vgt_edids[VGT_DP_A];
		dpcd = vgt->vgt_dpcds[DPCD_DPA];
		break;
	case VGT_DPB_IDX:
		edid = vgt->vgt_edids[VGT_DP_B];
		dpcd = vgt->vgt_dpcds[DPCD_DPB];
		break;
	case VGT_DPC_IDX:
		edid = vgt->vgt_edids[VGT_DP_C];
		dpcd = vgt->vgt_dpcds[DPCD_DPC];
		break;
	case VGT_DPD_IDX:
		edid = vgt->vgt_edids[VGT_DP_D];
		dpcd = vgt->vgt_dpcds[DPCD_DPD];
		break;
	default:
		vgt_warn("vGT(%d): Unsupported DP port access!\n",
				vgt->vgt_id);
		BUG();
		break;
	}

	/* read out message from DATA1 register */
	msg = __vreg(vgt, reg + 4);
	addr = (msg >> 8) & 0xffff;
	ctrl = (msg >> 24) & 0xff;
	len = msg & 0xff;
	op = ctrl >> 4;

	if (op == VGT_AUX_NATIVE_WRITE) {
		int t;
		uint8_t buf[16];

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * Write request exceeds what we supported,
			 * DCPD spec: When a Source Device is writing a DPCD
			 * address not supported by the Sink Device, the Sink
			 * Device shall reply with AUX NACK and “M” equal to zero.
			 */

			/* NAK the write */
			__vreg(vgt, reg + 4) = AUX_NATIVE_REPLY_NAK;

			dp_aux_ch_ctl_trans_done(vgt, value, reg, 2);

			return true;
		}

		/*
		 * Write request format: (command + address) occupies
		 * 3 bytes, followed by (len + 1) bytes of data.
		 */
		ASSERT((len + 4) <= AUX_BURST_SIZE);

		/* unpack data from vreg to buf */
		for (t = 0; t < 4; t ++) {
			vgt_reg_t r = __vreg(vgt, reg + 8 + t*4);

			buf[t*4] = (r >> 24) & 0xff;
			buf[t*4 + 1] = (r >> 16) & 0xff;
			buf[t*4 + 2] = (r >> 8) & 0xff;
			buf[t*4 + 3] = r & 0xff;
		}

		/* write to virtual DPCD */
		if (dpcd) {
			for (t = 0; t <= len; t ++) {
				int p = addr + t;

				dpcd->data[p] = buf[t];

				/* check for link training */
				if (p == DPCD_TRAINING_PATTERN_SET)
					dp_aux_ch_ctl_link_training(dpcd, buf[t]);
			}
		}

		/* ACK the write */
		__vreg(vgt, reg + 4) = 0;

		dp_aux_ch_ctl_trans_done(vgt, value, reg, 1);

		return true;
	}

	if (op == VGT_AUX_NATIVE_READ) {
		int idx, i, ret = 0;

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * read request exceeds what we supported
			 * DPCD spec: A Sink Device receiving a Native AUX CH
			 * read request for an unsupported DPCD address must
			 * reply with an AUX ACK and read data set equal to
			 * zero instead of replying with AUX NACK.
			 */

			/* ACK the READ*/
			__vreg(vgt, reg + 4) = 0;
			__vreg(vgt, reg + 8) = 0;
			__vreg(vgt, reg + 12) = 0;
			__vreg(vgt, reg + 16) = 0;
			__vreg(vgt, reg + 20) = 0;

			dp_aux_ch_ctl_trans_done(vgt ,value, reg, len + 2);

			return true;
		}

		for (idx = 1; idx <= 5; idx ++) {
			/* clear the data registers */
			__vreg(vgt, reg + 4 * idx) = 0;
		}

		/*
		 * Read reply format: ACK (1 byte) plus (len + 1) bytes of data.
		 */
		ASSERT((len + 2) <= AUX_BURST_SIZE);

		/* read from virtual DPCD to vreg */
		/* first 4 bytes: [ACK][addr][addr+1][addr+2] */
		if (dpcd) {
			for (i = 1; i <= (len + 1); i ++) {
				int t;

				t = dpcd->data[addr + i - 1];
				t <<= (24 - 8*(i%4));
				ret |= t;

				if ((i%4 == 3) || (i == (len + 1))) {
					__vreg(vgt, reg + (i/4 + 1)*4) = ret;
					ret = 0;
				}
			}
		}

		dp_aux_ch_ctl_trans_done(vgt, value, reg, len + 2);

		return true;
	}

	/* i2c transaction starts */

	vgt_i2c_handle_aux_ch_write(&vgt->vgt_i2c_bus, edid,
				offset, port_idx, p_data);
	return true;
}

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
	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DPA_IDX],
		(vgt_reg_t *)((char *)vregs + _REG_DPA_AUX_CH_CTL));

	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DPB_IDX],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPB_AUX_CH_CTL));

	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DPC_IDX],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPC_AUX_CH_CTL));

	vgt_aux_register_assign(i2c_bus->aux_ch.aux_registers[VGT_DPD_IDX],
		(vgt_reg_t *)((char *)vregs + _REG_PCH_DPD_AUX_CH_CTL));
}

static bool vga_control_r(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_mmio_read(vgt, offset, p_data, bytes);
}

static bool vga_control_w (struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;

	default_mmio_write(vgt, offset, p_data, bytes);

	/* no notification at dom0 boot time */
	if (vgt_ops->boot_time)
		return true;

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

static vgt_reg_t get_sbi_reg_cached_value(struct vgt_device *vgt,
	unsigned int sbi_offset)
{
	int i;
	int num = vgt->sbi_regs.number;
	vgt_reg_t value = 0;

	for (i = 0; i < num; ++ i) {
		if (vgt->sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i < num) {
		value = vgt->sbi_regs.registers[i].value;
	} else {
		vgt_warn("vGT(%d): SBI reading did not find the cached value"
			" for offset 0x%x. 0 will be returned!\n",
			vgt->vgt_id, sbi_offset);
	}

	return value;
}

static void cache_sbi_reg_value(struct vgt_device *vgt, unsigned int sbi_offset,
	vgt_reg_t value)
{
	int i;
	int num = vgt->sbi_regs.number;

	for (i = 0; i < num; ++ i) {
		if (vgt->sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i == num) {
		if (num < SBI_REG_MAX) {
			vgt->sbi_regs.number ++;
		} else {
			vgt_warn("vGT(%d): SBI caching meets maximum limits!\n",
				vgt->vgt_id);
			return;
		}
	}

	vgt->sbi_regs.registers[i].offset = sbi_offset;
	vgt->sbi_regs.registers[i].value = value;
}

static bool sbi_mmio_data_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_read(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		if (((__vreg(vgt, _REG_SBI_CTL_STAT) & _SBI_OPCODE_MASK) >>
			_SBI_OPCODE_SHIFT) == _SBI_CMD_CRRD) {
			unsigned int sbi_offset = (__vreg(vgt, _REG_SBI_ADDR) &
				_SBI_ADDR_OFFSET_MASK) >> _SBI_ADDR_OFFSET_SHIFT;
			vgt_reg_t val = get_sbi_reg_cached_value(vgt, sbi_offset);
			*(vgt_reg_t *)p_data = val;
		}
	}

	return rc;
}

static bool sbi_mmio_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		vgt_reg_t data = __vreg(vgt, offset);

		data &= ~(_SBI_STAT_MASK << _SBI_STAT_SHIFT);
		data |= _SBI_READY;

		data &= ~(_SBI_RESPONSE_MASK << _SBI_RESPONSE_SHIFT);
		data |= _SBI_RESPONSE_SUCCESS;

		__vreg(vgt, offset) = data;

		if (((__vreg(vgt, _REG_SBI_CTL_STAT) & _SBI_OPCODE_MASK) >>
			_SBI_OPCODE_SHIFT) == _SBI_CMD_CRWR) {
			unsigned int sbi_offset = (__vreg(vgt, _REG_SBI_ADDR) &
				_SBI_ADDR_OFFSET_MASK) >> _SBI_ADDR_OFFSET_SHIFT;
			vgt_reg_t val = __vreg(vgt, _REG_SBI_DATA);
			cache_sbi_reg_value(vgt, sbi_offset, val);
		}
	}

	return rc;
}

static bool pvinfo_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_read(vgt, offset, p_data, bytes);
	bool invalid_read = false;

	switch (offset) {
		case vgt_info_off(magic) ... vgt_info_off(vgt_id):
			if (offset + bytes > vgt_info_off(vgt_id) + 4)
				invalid_read = true;
			break;

		case vgt_info_off(avail_rs.low_gmadr.my_base) ...
			vgt_info_off(avail_rs.fence_num):
			if (offset + bytes >
				vgt_info_off(avail_rs.fence_num) + 4)
				invalid_read = true;
			break;

		case vgt_info_off(drv_version_major) ...
			vgt_info_off(min_fence_num):
			if (offset + bytes > vgt_info_off(min_fence_num) + 4)
				invalid_read = true;
			break;
		case vgt_info_off(v2g_notify):
			/* set cursor setting here.  For example:
			 *   *((unsigned int *)p_data)) = VGT_V2G_SET_SW_CURSOR;
			 */
			break;
		default:
			invalid_read = true;
			break;
	}

	if (invalid_read)
		vgt_warn("invalid pvinfo read: [%x:%x] = %x!!!\n",
			offset, bytes, *(vgt_reg_t *)p_data);

	return rc;
}

static bool pvinfo_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t val = *(vgt_reg_t *)p_data;
	vgt_reg_t min;
	bool rc = true;

	switch (offset) {
		case vgt_info_off(min_low_gmadr):
			min = val;
			if (vgt->aperture_sz < min) {
				vgt_err("VM(%d): aperture size(%llx) is less than"
					"its driver's minimum requirement(%x)!\n",
					vgt->vm_id, vgt->aperture_sz, min);
				rc = false;
			}
			break;
		case vgt_info_off(min_high_gmadr):
			min = val;
			if (vgt->gm_sz - vgt->aperture_sz < min) {
				vgt_err("VM(%d): hiden gm size(%llx) is less than"
					"its driver's minimum requirement(%x)!\n",
					vgt->vm_id, vgt->gm_sz - vgt->aperture_sz,
				        min);
				rc = false;
			}
			break;
		case vgt_info_off(min_fence_num):
			min = val;
			if (vgt->fence_sz < min) {
				vgt_err("VM(%d): fence size(%x) is less than"
					"its drivers minimum requirement(%x)!\n",
					vgt->vm_id, vgt->fence_sz, min);
				rc = false;
			}
			break;
		case vgt_info_off(display_ready):
			if (vgt->vm_id
				&& hvm_boot_foreground == true
				&& !vgt->hvm_boot_foreground_visible) {
				/*
				 * Guest had a vaild surface to show.
				 */
				vgt->hvm_boot_foreground_visible = 1;
				do_vgt_fast_display_switch(vgt);
			}
			break;
		case vgt_info_off(g2v_notify):
			if (val == VGT_G2V_DISPLAY_REFRESH) {
				/* put handler here
				 * printk("VGT_G2V_DISPLAY_REFRESH\n");
				 */
			} else if (val == VGT_G2V_SET_POINTER_SHAPE) {
				/* put handler here
				 * printk("VGT_G2V_SET_POINTER_SHAPE\n");
				 */
			} else {
				vgt_warn("INVALID_WRITE_NOTIFICATION %x\n", val);
			}
			break;
		case vgt_info_off(xhot):
				/* vgt_info("xhot %x\n", val); */
			break;
		case vgt_info_off(yhot):
				/* vgt_info("yhot %x\n", val); */
			break;
		default:
			/* keep rc's default value: true.
			 * NOTE: returning false will crash the VM.
			 */
			vgt_warn("invalid pvinfo write: [%x:%x] = %x!!!\n",
				offset, bytes, val);
			break;
	}

	if (rc == true)
		 rc = default_mmio_write(vgt, offset, p_data, bytes);

	return rc;
}

static bool pf_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	if (enable_panel_fitting) {
		*(vgt_reg_t *)p_data = __vreg(vgt, offset);
	} else {
		default_mmio_read(vgt, offset, p_data, bytes);
	}

	return true;
}

static bool pf_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{

	if (enable_panel_fitting) {
		memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);
	} else {
		default_mmio_write(vgt, offset, p_data, bytes);
	}

	return true;
}

/*
 * Track policies of all captured registers
 *
 * The registers are organized in blocks according to their locations
 * on the spec:
 *	- render
 *	- display
 *	- others (pm, workaround, etc.)
 *      - un-categorized
 *
 * The poclies within the same block can vary:
 *      - [F_VIRT]: default virtualization policy
 *          * all VMs access vReg
 *      - [F_RDR]/[F_DPY]: ownership based virtualization
 *          * owner accesses pReg
 *          * non-owner accesses vReg
 *          * vReg<->pReg at ownership switch time
 *      - [F_DOM0]: uniquely owned by Dom0
 *          * dom0 accesses pReg
 *          * other VMs accesses vReg
 *      - [F_PT]: passthrough policy with HIGH RISK
 *          * all VMs access pReg!!!
 *          * temp solution. must be removed in the end
 *
 * There are some ancillary attributes, which can be linked together
 *      - [ADRFIX]: require address check
 *      - [HWSTS]: need sync with pReg for status bit change
 *      - [MODE]: higher 16bits are mask bits
 *
 * When there are handlers registered, handlers can supersede all
 * above policies.
 */
reg_attr_t vgt_base_reg_info[] = {

	/* -------render regs---------- */
{_REG_GTIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_GTIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_GTIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_GTISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_RCS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_BCS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_VCS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_VECS_IMR, 4, F_RDR, 0, D_HSW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_RCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_BCS_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_GEN7PLUS, NULL, NULL},
{_REG_VEBOX_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_GEN7PLUS, NULL, NULL},
{_REG_VECS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},

/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
{0x14080, 4, F_VIRT, 0, D_SNB, NULL, NULL},
{_REG_RCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VECS_EXCC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_RCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_VCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_BCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_VECS_UHPTR, 4, F_RDR_HWSTS, 0, D_HSW_PLUS, NULL, ring_uhptr_write},
{_REG_RCS_BB_PREEMPT_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_CCID, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{0x12198, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

{_REG_CXT_SIZE, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_GEN7_CXT_SIZE, 4, F_PT, 0, D_ALL, NULL, NULL},

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

{_REG_VECS_TAIL, 4, F_RDR, 0, D_HSW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VECS_HEAD, 4, F_RDR, 0, D_HSW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VECS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VECS_CTL, 4, F_RDR, 0, D_HSW_PLUS, ring_mmio_read, ring_mmio_write},//for TLB

{_REG_RCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VECS_ACTHD, 4, F_RDR, 0, D_HSW, NULL, NULL},

{_REG_GFX_MODE, 4, F_RDR_MODE, 0, D_SNB, NULL, NULL},
{_REG_RCS_GFX_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_VCS_MFX_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_BCS_BLT_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_VEBOX_MODE, 4, F_RDR_MODE, 0, D_HSW, NULL, NULL},
{_REG_ARB_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

{_REG_RCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VECS_MI_MODE, 4, F_RDR_MODE, 0, D_HSW_PLUS, NULL, NULL},

{_REG_RCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VECS_INSTPM, 4, F_RDR_MODE, 0, D_HSW_PLUS, NULL, NULL},

{_REG_GT_MODE, 4, F_RDR_MODE, 0, D_SNB, NULL, NULL},
{_REG_GT_MODE_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_CACHE_MODE_0, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_0_IVB, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_1_IVB, 4, F_RDR_MODE, 0, D_GEN7PLUS, NULL, NULL},
{_REG_RCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VECS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW_PLUS, NULL, NULL},
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
{_REG_RVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_BRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BVSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VBSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VEBSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VERSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VEVSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},

{0x23bc, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x2448, 8, F_RDR, 0, D_HSW, NULL, NULL},
{0x2450, 8, F_RDR, 0, D_HSW, NULL, NULL},
{0x7034, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x2b00, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x91b8, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x91bc, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x91c0, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x91c4, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x9150, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x9154, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x9160, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x9164, 4, F_RDR, 0, D_HSW, NULL, NULL},

{0x4040, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb010, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb020, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb024, 4, F_RDR, 0, D_HSW, NULL, NULL},

{0x2050, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x12050, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x22050, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x1A050, 4, F_PT, 0, D_HSW_PLUS, NULL, NULL},

{0x20dc, 4, F_PT, 0, D_ALL, NULL, NULL},

	/* -------display regs---------- */
{_REG_VGA_CR_INDEX_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA_ST01_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA_AR_INDEX, 1, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA_DACMASK, 1, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA_MSR_READ, 1, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VGA_PD, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0x42080, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{_REG_DEIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_DEIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_DEIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_DEISR, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SDEIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_SDEIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{_REG_SDEIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{_REG_SDEISR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_isr_handler},

{_REG_DE_RRMR, 4, F_VIRT, 0, D_ALL, NULL, rrmr_mmio_write},

{_REG_PIPEADSL, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPEACONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEASTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPARB, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},

{_REG_PIPEBDSL, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPEBCONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEBSTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},

{_REG_PIPECDSL, 4, F_DPY, 0, D_HSW, pipe_dsl_mmio_read, NULL},
{_REG_PIPECCONF, 4, F_DPY, 0, D_HSW, NULL, pipe_conf_mmio_write},
{_REG_PIPECSTAT, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PIPEC_FRMCOUNT, 4, F_DPY, 0, D_HSW, pipe_frmcount_mmio_read, NULL},

{_REG_PIPE_EDP_CONF, 4, F_DPY, 0, D_HSW, NULL, pipe_conf_mmio_write},

{_REG_CURABASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURACNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_REG_CURAPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_REG_CURASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, cur_surflive_mmio_read,
					surflive_mmio_write},

{_REG_CURAPALET_0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_3, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CURBBASE_SNB, 4, F_DPY_ADRFIX, 0xFFFFF000, D_SNB, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBCNTR_SNB, 4, F_DPY, 0, D_SNB, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBPOS_SNB, 4, F_DPY, 0, D_SNB, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBSURFLIVE_SNB, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_SNB, cur_surflive_mmio_read,
					surflive_mmio_write},

{_REG_CURBBASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBCNTR, 4, F_DPY, 0, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBPOS, 4, F_DPY, 0, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_GEN7PLUS, cur_surflive_mmio_read,
					surflive_mmio_write},

{_REG_CURCBASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURCCNTR, 4, F_DPY, 0, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURCPOS, 4, F_DPY, 0, D_GEN7PLUS, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURCSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_GEN7PLUS, cur_surflive_mmio_read,
					surflive_mmio_write},

{0x7008C, 4, F_DPY, 0, D_ALL, NULL, vgt_error_handler},

{0x700D0, 4, F_DPY, 0, D_SNB, NULL, NULL},
{0x700D4, 4, F_DPY, 0, D_SNB, NULL, NULL},
{0x700D8, 4, F_DPY, 0, D_SNB, NULL, NULL},
{0x700DC, 4, F_DPY, 0, D_SNB, NULL, NULL},

{0x701b0, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DSPACNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_REG_DSPASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_REG_DSPASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, pri_surflive_mmio_read,
							surflive_mmio_write},
{_REG_DSPALINOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPASTRIDE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPAPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPASIZE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPATILEOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_REG_DSPBCNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_REG_DSPBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_REG_DSPBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, pri_surflive_mmio_read,
							surflive_mmio_write},
{_REG_DSPBLINOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBSTRIDE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBSIZE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBTILEOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_REG_DSPCCNTR, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_REG_DSPCSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_HSW, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_REG_DSPCSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_HSW, pri_surflive_mmio_read,
							surflive_mmio_write},
{_REG_DSPCLINOFF, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPCSTRIDE, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPCPOS, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPCSIZE, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPCTILEOFF, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_REG_DVSACNTR, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_DVSASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_DVSALINOFF, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSAPOS, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSASIZE, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSATILEOFF, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSAKEYVAL, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSAKEYMSK, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSAKEYMAXVAL, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSASCALE, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBCNTR, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_DVSBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_SNB, NULL, NULL},
{_REG_DVSBLINOFF, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBPOS, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBSIZE, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBTILEOFF, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBKEYVAL, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBKEYMSK, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBKEYMAXVAL, 4, F_DPY, 0, D_SNB, NULL, NULL},
{_REG_DVSBSCALE, 4, F_DPY, 0, D_SNB, NULL, NULL},

{_REG_SPRASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_HSW,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_REG_SPRASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_HSW,
			spr_surflive_mmio_read, surflive_mmio_write},

{_REG_SPRBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_HSW,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_REG_SPRBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_HSW,
			spr_surflive_mmio_read, surflive_mmio_write},

{_REG_SPRCSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_HSW,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_REG_SPRCSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_HSW,
			spr_surflive_mmio_read, surflive_mmio_write},

{_REG_SPRA_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SPRA_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_SPRB_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SPRB_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_SPRC_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SPRC_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},


{_REG_LGC_PALETTE_A, 4*256, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_LGC_PALETTE_B, 4*256, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_LGC_PALETTE_C, 4*256, F_DPY, 0, D_GEN7PLUS, NULL, NULL},

{_REG_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_PIPEASRC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_BCLRPAT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_REG_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_PIPEBSRC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_BCLRPAT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_REG_HTOTAL_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HBLANK_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HSYNC_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VTOTAL_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VBLANK_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VSYNC_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_PIPECSRC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_BCLRPAT_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{0x6F000, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F004, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F008, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F00C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F010, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F014, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F028, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F030, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F034, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F040, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F044, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEA_DATA_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEA_DATA_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEA_LINK_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEA_LINK_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},

{_REG_PIPEB_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEB_DATA_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEB_DATA_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEB_LINK_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEB_LINK_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},

{_REG_PIPEC_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEC_DATA_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEC_DATA_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEC_LINK_M2, 4, F_DPY, 0, D_IVB, NULL, NULL},
{_REG_PIPEC_LINK_N2, 4, F_DPY, 0, D_IVB, NULL, NULL},

{_REG_PF_CTL_0, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_SZ_0, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_POS_0, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_CTL_1, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_SZ_1, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_POS_1, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_CTL_2, 4, F_DPY, 0, D_GEN7PLUS, pf_read, pf_write},
{_REG_PF_WIN_SZ_2, 4, F_DPY, 0, D_GEN7PLUS, pf_read, pf_write},
{_REG_PF_WIN_POS_2, 4, F_DPY, 0, D_GEN7PLUS, pf_read, pf_write},

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

{_REG_BLC_PWM_CPU_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_CPU_CTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_PCH_CTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_BLC_PWM_PCH_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_PCH_GMBUS0, 4*4, F_VIRT, 0, D_ALL, gmbus_mmio_read, gmbus_mmio_write},
{_REG_PCH_GPIOA, 6*4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DP_BUFTRANS, 0x28, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PCH_DPB_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},
{_REG_PCH_DPC_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},
{_REG_PCH_DPD_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL,
	dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},

{_REG_PCH_ADPA, 4, F_DPY, 0, D_ALL, pch_adpa_mmio_read, pch_adpa_mmio_write},
{_REG_DP_B_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, dp_ctl_mmio_write},
{_REG_DP_C_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, dp_ctl_mmio_write},
{_REG_DP_D_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, dp_ctl_mmio_write},
{_REG_HDMI_B_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, hdmi_ctl_mmio_write},
{_REG_HDMI_C_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, hdmi_ctl_mmio_write},
{_REG_HDMI_D_CTL, 4, F_DPY, 0, D_SNB|D_IVB, NULL, hdmi_ctl_mmio_write},
{_REG_TRANSACONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_REG_TRANSBCONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_REG_FDI_RXA_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXB_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXC_IIR, 4, F_DPY, 0, D_GEN7PLUS, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXC_CTL, 4, F_DPY, 0, D_GEN7PLUS, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXC_CTL, 4, F_DPY, 0, D_GEN7PLUS, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXA_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXB_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXC_IMR, 4, F_DPY, 0, D_GEN7PLUS, NULL, update_fdi_rx_iir_status},

{_REG_TRANS_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_REG_TRANS_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_TRANS_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

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
{_REG_SHOTPLUG_CTL, 4, F_DPY, 0, D_ALL, NULL, shotplug_ctl_mmio_write},
{_REG_LCPLL_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_FUSE_STRAP, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_DP_A_HOTPLUG_CNTL, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_DISP_ARB_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DISP_ARB_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_DISPLAY_CHICKEN_BITS_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DISPLAY_CHICKEN_BITS_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_SOUTH_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_SOUTH_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_SOUTH_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSA_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSB_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},

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

{_REG_CSC_A_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CSC_B_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CSC_C_COEFFICIENTS, 4*6, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_CSC_C_MODE, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_PRECSC_C_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_PRECSC_C_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},
{_REG_PRECSC_C_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_GEN7PLUS, NULL, NULL},

{0x60110, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x61110, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x70400, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_WM_DBG, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PIPE_WM_LINETIME_A, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PIPE_WM_LINETIME_B, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PIPE_WM_LINETIME_C, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SPLL_CTL, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_WRPLL_CTL1, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_WRPLL_CTL2, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIA, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIB, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIC, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDID, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIE, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_TRANS_CLK_SEL_A, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_TRANS_CLK_SEL_B, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_TRANS_CLK_SEL_C, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x46408, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x46508, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x49040, 0xc, F_DPY, 0, D_HSW, NULL, NULL},
{0x49140, 0xc, F_DPY, 0, D_HSW, NULL, NULL},
{0x49240, 0xc, F_DPY, 0, D_HSW, NULL, NULL},
{0x49080, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x49090, 0x14, F_DPY, 0, D_HSW, NULL, NULL},
{0x49180, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x49190, 0x14, F_DPY, 0, D_HSW, NULL, NULL},
{0x49280, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x49290, 0x14, F_DPY, 0, D_HSW, NULL, NULL},
{0x4A400, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x4A480, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x4AC00, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x4AC80, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x4B400, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x4B480, 4, F_DPY, 0, D_HSW, NULL, NULL},

{0x6002C, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_HSW_VIDEO_DIP_CTL_A, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_VIDEO_DIP_CTL_B, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_VIDEO_DIP_CTL_C, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_VIDEO_DIP_CTL_EDP, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_SFUSE_STRAP, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SBI_ADDR, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_SBI_DATA, 4, F_DPY, 0, D_HSW, sbi_mmio_data_read, NULL},
{_REG_SBI_CTL_STAT, 4, F_DPY, 0, D_HSW, NULL, sbi_mmio_ctl_write},
{_REG_PIXCLK_GATE, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0xF200C, 4, F_DPY, 0, D_SNB, NULL, NULL},

{_REG_DPA_AUX_CH_CTL, 6*4, F_DPY, 0, D_HSW, dp_aux_ch_ctl_mmio_read, dp_aux_ch_ctl_mmio_write},

{_REG_DDI_BUF_CTL_A, 4, F_DPY, 0, D_HSW, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_B, 4, F_DPY, 0, D_HSW, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_C, 4, F_DPY, 0, D_HSW, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_D, 4, F_DPY, 0, D_HSW, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_E, 4, F_DPY, 0, D_HSW, NULL, ddi_buf_ctl_mmio_write},

{_REG_DP_TP_CTL_A, 4, F_DPY, 0, D_HSW, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_B, 4, F_DPY, 0, D_HSW, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_C, 4, F_DPY, 0, D_HSW, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_D, 4, F_DPY, 0, D_HSW, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_E, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_DP_TP_STATUS_A, 4, F_DPY, 0, D_HSW, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_B, 4, F_DPY, 0, D_HSW, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_C, 4, F_DPY, 0, D_HSW, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_D, 4, F_DPY, 0, D_HSW, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_E, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_DDI_BUF_TRANS_A, 0x50, F_DPY, 0, D_HSW, NULL, NULL},
{0x64E60, 0x50, F_DPY, 0, D_HSW, NULL, NULL},
{0x64Ec0, 0x50, F_DPY, 0, D_HSW, NULL, NULL},
{0x64F20, 0x50, F_DPY, 0, D_HSW, NULL, NULL},
{0x64F80, 0x50, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_HSW_AUD_CONFIG_A, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x650C0, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x6661c, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x66C00, 8, F_DPY, 0, D_HSW, NULL, NULL},

{_REG_TRANS_DDI_FUNC_CTL_A, 4, F_DPY, 0, D_HSW, NULL, dpy_trans_ddi_ctl_write},
{_REG_TRANS_DDI_FUNC_CTL_B, 4, F_DPY, 0, D_HSW, NULL, dpy_trans_ddi_ctl_write},
{_REG_TRANS_DDI_FUNC_CTL_C, 4, F_DPY, 0, D_HSW, NULL, dpy_trans_ddi_ctl_write},
{_REG_TRANS_DDI_FUNC_CTL_EDP, 4, F_DPY, 0, D_HSW, NULL, dpy_trans_ddi_ctl_write},

{_REG_TRANS_MSA_MISC_A, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_TRANS_MSA_MISC_B, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_REG_TRANS_MSA_MISC_C, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x6F410, 4, F_DPY, 0, D_HSW, NULL, NULL},

	/* -------others---------- */
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
{_REG_ECOBUS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{_REG_RC_STATE, 4, F_DOM0, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{_REG_RPNSWREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC_VIDEO_FREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_TIMEOUT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_INTERRUPT_LIMITS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RPSTAT1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_UP_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_PREV_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_CUR_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_PREV_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RP_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC1_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC6_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC6pp_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC_EVALUATION_INTERVAL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC_SLEEP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC1e_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC6_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC6p_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_RC6pp_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_PMINTRMSK, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL1, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL2, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL3, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{_REG_HSW_PWR_WELL_CTL4, 4, F_DOM0, 0, D_HSW, NULL, NULL},

{_REG_RSTDBYCTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_GEN6_GDRST, 4, F_VIRT, 0, D_ALL, NULL, gen6_gdrst_mmio_write},
{_REG_FENCE_0_LOW, 0x80, F_VIRT, 0, D_ALL, fence_mmio_read, fence_mmio_write},
{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE, F_VIRT, 0, D_ALL, pvinfo_read, pvinfo_write},
{_REG_CPU_VGACNTRL, 4, F_DOM0, 0, D_ALL, vga_control_r, vga_control_w},

/* TODO: MCHBAR, suppose read-only */
{_REG_MCHBAR_MIRROR, 0x40000, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_TILECTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_UCG_CTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{_REG_UCG_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_SWF, 0x110, F_VIRT, 0, D_SNB, NULL, NULL},
{_REG_SWF, 0x90, F_VIRT, 0, D_GEN7PLUS, NULL, NULL},
{_REG_SWF00, 0x40, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SWF10, 4*16, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_SWF30, 4*3, F_VIRT, 0, D_ALL, NULL, NULL},

	/* -------un-categorized regs--------- */

{0x3c, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x860, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN1, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN2, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_3D_CHICKEN3, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x20d4, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x2088, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x20e4, 4, F_PT, 0, D_GEN7PLUS, NULL, NULL},
/* no definition on this. from Linux */
{_REG_GEN3_MI_ARB_STATE, 4, F_PT, 0, D_SNB, NULL, NULL},
{_REG_RCS_ECOSKPD, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x121d0, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_BCS_ECOSKPD, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x41d0, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x22ac, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_VFSKPD, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x2700, 4, F_PT, 0, D_SNB, NULL, NULL},
{_REG_ECOCHK, 4, F_PT, 0, D_ALL, NULL, NULL},
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
{0x48800, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{0xce044, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6500, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6504, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6600, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6604, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6700, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6704, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6804, 4, F_VIRT, 0, D_ALL, NULL, NULL},
/* FIXME: now looks gmbus handler can't cover 4/5 ports */
{_REG_PCH_GMBUS4, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PCH_GMBUS5, 4, F_DPY, 0, D_ALL, NULL, NULL},

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
{_REG_GTDRIVER_MAILBOX_INTERFACE, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_GTDRIVER_MAILBOX_DATA0, 4, F_PT, 0, D_ALL, NULL, NULL},
{0x13812c, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_GTT_FAULT_STATUS, 4, F_PT, 0, D_ALL, err_int_r, err_int_w},
/* HSW */
{0x120010, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x9008, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_GFX_FLSH_CNT, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_GEN7_COMMON_SLICE_CHICKEN1, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_GEN7_L3CNTLREG1, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_GEN7_L3_CHICKEN_MODE_REGISTER, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_GEN7_SQ_CHICKEN_MBCUNIT_CONFIG, 4, F_PT, 0, D_HSW, NULL, NULL},

{0x2020, 4, F_PT, 0, D_ALL, NULL, NULL},
{_REG_IER, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x20e8, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x2214, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_RCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
{_REG_VCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
{0x1a358, 8, F_PT, 0, D_ALL, NULL, NULL},
{_REG_BCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
{0x8000, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x8008, 4, F_PT, 0, D_HSW, NULL, NULL},
{0xb008, 4, F_PT, 0, D_HSW, NULL, NULL},
{0xb208, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x320f0, 8, F_PT, 0, D_HSW, NULL, NULL},
{0x320fc, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x32230, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x44084, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x4408c, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x45260, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x1082c0, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x13005c, 4, F_PT, 0, D_HSW, NULL, NULL},
{_REG_FPGA_DBG, 4, F_DOM0, 0, D_HSW, NULL, NULL},

/* DOM0 PM owns these registers. */
{_REG_SCRATCH1, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{_REG_ROW_CHICKEN3, 4, F_DOM0, 0, D_HSW, NULL, NULL},
/* MAXCNT means max idle count */

{_REG_RC_PWRCTX_MAXCNT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x12054, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x22054, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x1A054, 4, F_DOM0, 0, D_HSW, NULL, NULL},
};

bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev)
{
	printk("post mmio hooks initialized\n");

	if (pdev->enable_ppgtt) {
		vgt_dbg("Hook up PPGTT register handlers\n");
		/* trap PPGTT base register */
		reg_update_handlers(pdev, _REG_RCS_PP_DIR_BASE_IVB, 4,
				pp_dir_base_read, pp_dir_base_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DIR_BASE, 4,
				pp_dir_base_read, pp_dir_base_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DIR_BASE, 4,
				pp_dir_base_read, pp_dir_base_write);

		reg_update_handlers(pdev, _REG_RCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);

		/* XXX cache register? */
		/* PPGTT enable register */
		reg_update_handlers(pdev, _REG_RCS_GFX_MODE_IVB, 4,
				ring_pp_mode_read, ring_pp_mode_write);
		reg_update_handlers(pdev, _REG_BCS_BLT_MODE_IVB, 4,
				ring_pp_mode_read, ring_pp_mode_write);
		reg_update_handlers(pdev, _REG_VCS_MFX_MODE_IVB, 4,
				ring_pp_mode_read, ring_pp_mode_write);

		if (IS_HSW(pdev)) {
			reg_update_handlers(pdev, _REG_VECS_PP_DIR_BASE, 4,
					pp_dir_base_read,
					pp_dir_base_write);
			reg_update_handlers(pdev, _REG_VECS_PP_DCLV, 4,
					pp_dclv_read, pp_dclv_write);
			reg_update_handlers(pdev, _REG_VEBOX_MODE, 4,
					ring_pp_mode_read,
					ring_pp_mode_write);
		}
	}

	return true;
}

int vgt_get_base_reg_num()
{
	return ARRAY_NUM(vgt_base_reg_info);
}

/*
 * This array lists registers which stick to original policy, as
 * specified in vgt_base_reg_info, and not impacted by the super
 * owner mode (which has most registers owned by HVM instead of
 * dom0).
 *
 * Currently the registers in this list are those, which must be
 * virtualized, with XenGT driver itself as the exclusive owner.
 * Some features like monitor hotplug may be broken, due to the
 * whole handling flow already fixed (first to dom0). But that
 * should be fine, since super owner mode is used for analyze
 * basic stability issues.
 */
reg_list_t vgt_sticky_regs[] = {
	/* interrupt control registers */
	{_REG_GTIMR, 4},
	{_REG_GTIER, 4},
	{_REG_GTIIR, 4},
	{_REG_GTISR, 4},
	{_REG_RCS_IMR, 4},
	{_REG_BCS_IMR, 4},
	{_REG_VCS_IMR, 4},
	{_REG_VECS_IMR, 4},
	{_REG_DEIMR, 4},
	{_REG_DEIER, 4},
	{_REG_DEIIR, 4},
	{_REG_DEISR, 4},
	{_REG_SDEIMR, 4},
	{_REG_SDEIER, 4},
	{_REG_SDEIIR, 4},
	{_REG_SDEISR, 4},
	{_REG_PMIMR, 4},
	{_REG_PMIER, 4},
	{_REG_PMIIR, 4},
	{_REG_PMISR, 4},

	/* PPGTT related registers */
	{_REG_RCS_GFX_MODE_IVB, 4},
	{_REG_VCS_MFX_MODE_IVB, 4},
	{_REG_BCS_BLT_MODE_IVB, 4},
	{_REG_VEBOX_MODE, 4},
	{_REG_RCS_PP_DIR_BASE_IVB, 4},
	{_REG_VCS_PP_DIR_BASE, 4},
	{_REG_BCS_PP_DIR_BASE, 4},
	{_REG_VECS_PP_DIR_BASE, 4},
	{_REG_RCS_PP_DCLV, 4},
	{_REG_VCS_PP_DCLV, 4},
	{_REG_BCS_PP_DCLV, 4},
	{_REG_VECS_PP_DCLV, 4},

	/* forcewake */
	{_REG_FORCEWAKE, 4},
	{_REG_FORCEWAKE_ACK, 4},
	{_REG_GT_CORE_STATUS, 4},
	{_REG_GT_THREAD_STATUS, 4},
	{_REG_GTFIFODBG, 4},
	{_REG_GTFIFO_FREE_ENTRIES, 4},
	{_REG_MUL_FORCEWAKE, 4},
	{_REG_MUL_FORCEWAKE_ACK, 4},
	{_REG_FORCEWAKE_ACK_HSW, 4},

	/* misc */
	{_REG_GEN6_GDRST, 4},
	{_REG_FENCE_0_LOW, 0x80},
	{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE},
	{_REG_CPU_VGACNTRL, 4},
};

int vgt_get_sticky_reg_num()
{
	return ARRAY_NUM(vgt_sticky_regs);
}
