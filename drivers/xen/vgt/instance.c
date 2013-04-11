/*
 * Instance life-cycle management
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
 */

#include <linux/pci.h>

#include <xen/vgt.h>

#include "vgt.h"

/*
 * bitmap of allocated vgt_ids.
 * bit = 0 means free ID, =1 means allocated ID.
 */
static unsigned long vgt_id_alloc_bitmap;

struct vgt_device *vmid_2_vgt_device(int vmid)
{
	unsigned int bit;
	struct vgt_device *vgt;

	ASSERT(vgt_id_alloc_bitmap != ~0UL)
	for_each_set_bit(bit, &vgt_id_alloc_bitmap, (8 * sizeof(unsigned long))) {
		vgt = default_device.device[bit];
		if (vgt->vm_id == vmid)
			return vgt;
	}
	return NULL;
}

static int allocate_vgt_id(void)
{
	unsigned long bit_index;

	ASSERT(vgt_id_alloc_bitmap != ~0UL)
	do {
		bit_index = ffz (vgt_id_alloc_bitmap);
		if (bit_index >= VGT_MAX_VMS) {
			vgt_err("vGT: allocate_vgt_id() failed\n");
			return -ENOSPC;
		}
	} while (test_and_set_bit(bit_index, &vgt_id_alloc_bitmap) != 0);

	return bit_index;
}

static void free_vgt_id(int vgt_id)
{
	ASSERT(vgt_id >= 0 && vgt_id < VGT_MAX_VMS);
	ASSERT(vgt_id_alloc_bitmap & (1UL << vgt_id));
	clear_bit(vgt_id, &vgt_id_alloc_bitmap);
}

/*
 * Initialize the vgt state instance.
 * Return:	0: failed
 *		1: success
 *
 */
static int create_state_instance(struct vgt_device *vgt)
{
	vgt_state_t	*state;
	int i;

	vgt_dbg("create_state_instance\n");
	state = &vgt->state;
	state->vReg = vmalloc(vgt->pdev->mmio_size);
	state->sReg = vmalloc(vgt->pdev->mmio_size);
	if ( state->vReg == NULL || state->sReg == NULL )
	{
		printk("VGT: insufficient memory allocation at %s\n", __FUNCTION__);
		if ( state->vReg )
			vfree (state->vReg);
		if ( state->sReg )
			vfree (state->sReg);
		state->sReg = state->vReg = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < VGT_BAR_NUM; i++)
		state->bar_mapped[i] = 0;
	return 0;
}

/*
 * priv: VCPU ?
 */
int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp)
{
	struct vgt_device *vgt;
	char *cfg_space;
	int rc = -ENOMEM;

	vgt_info("vm_id=%d, aperture_sz=%dMB, gm_sz=%dMB, fence_sz=%d, vgt_primary=%d\n",
		vp.vm_id, vp.aperture_sz, vp.gm_sz, vp.fence_sz, vp.vgt_primary);

	vgt = kzalloc (sizeof(*vgt), GFP_KERNEL);
	if (vgt == NULL) {
		printk("Insufficient memory for vgt_device in %s\n", __FUNCTION__);
		return rc;
	}

	if ((rc = vgt->vgt_id = allocate_vgt_id()) < 0 )
		goto err;

	vgt->vm_id = vp.vm_id;
	vgt->pdev = pdev;

	vgt->force_removal = false;

	INIT_LIST_HEAD(&vgt->list);

	if ((rc = create_state_instance(vgt)) < 0)
		goto err;

	/* Hard code ballooning now. We can support non-ballooning too in the future */
	vgt->ballooning = true;

	/* present aperture to the guest at the same host address */
	vgt->state.aperture_base = phys_aperture_base(pdev);

	/* init aperture/gm ranges allocated to this vgt */
	if ((rc = allocate_vm_aperture_gm_and_fence(vgt, vp)) < 0) {
		printk("vGT: %s: no enough available aperture/gm/fence!\n", __func__);
		goto err;
	}

	vgt->aperture_offset = aperture_2_gm(pdev, vgt->aperture_base);
	vgt->aperture_base_va = phys_aperture_vbase(pdev) +
		vgt->aperture_offset;

	if (vgt->ballooning)
		vgt->vgtt_sz = (gm_sz(pdev) >> GTT_PAGE_SHIFT) * GTT_ENTRY_SIZE;
	else
		vgt->vgtt_sz = (vgt->gm_sz >> GTT_PAGE_SHIFT) * GTT_ENTRY_SIZE;
	vgt_info("Virtual GTT size: 0x%lx\n", (long)vgt->vgtt_sz);
	vgt->vgtt = vzalloc(vgt->vgtt_sz);
	if (!vgt->vgtt) {
		printk("vGT: failed to allocate virtual GTT table\n");
		goto err;
	}

	alloc_vm_rsvd_aperture(vgt);

	vgt_vmem_init(vgt);

	vgt->rb[RING_BUFFER_RCS].stateless = 0;	/* RCS */
	vgt->rb[RING_BUFFER_VCS].stateless = 1;	/* BCS */
	vgt->rb[RING_BUFFER_BCS].stateless = 1;	/* VCS */

	vgt->state.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	vgt->state.bar_size[1] =			/* Aperture */
		vgt->ballooning ? pdev->bar_size[1] : vgt_aperture_sz(vgt);
	vgt->state.bar_size[2] = pdev->bar_size[2];	/* PIO */
	vgt->state.bar_size[3] = pdev->bar_size[3];	/* ROM */

	/* Set initial configuration space and MMIO space registers. */
	cfg_space = &vgt->state.cfg_space[0];
	memcpy (cfg_space, pdev->initial_cfg_space, VGT_CFG_SPACE_SZ);
	cfg_space[VGT_REG_CFG_SPACE_MSAC] = vgt->state.bar_size[1];
	cfg_space[_REG_GMCH_CONTRL] &= ~(_REGBIT_GMCH_GMS_MASK << _REGBIT_GMCH_GMS_SHIFT);
	vgt_pci_bar_write_32(vgt, VGT_REG_CFG_SPACE_BAR1, phys_aperture_base(pdev) );

	vgt_info("aperture: [0x%llx, 0x%llx] guest [0x%llx, 0x%llx] "
		"va(0x%llx)\n",
		vgt_aperture_base(vgt),
		vgt_aperture_end(vgt),
		vgt_guest_aperture_base(vgt),
		vgt_guest_aperture_end(vgt),
		(uint64_t)vgt->aperture_base_va);

	vgt_info("GM: [0x%llx, 0x%llx], [0x%llx, 0x%llx], "
		"guest[0x%llx, 0x%llx], [0x%llx, 0x%llx]\n",
		vgt_visible_gm_base(vgt),
		vgt_visible_gm_end(vgt),
		vgt_hidden_gm_base(vgt),
		vgt_hidden_gm_end(vgt),
		vgt_guest_visible_gm_base(vgt),
		vgt_guest_visible_gm_end(vgt),
		vgt_guest_hidden_gm_base(vgt),
		vgt_guest_hidden_gm_end(vgt));

	/* If the user explicitly specified a value, use it; or, use the
	 * global vgt_primary.
	 */
	ASSERT(vgt->vm_id == 0 || (vp.vgt_primary >= -1 && vp.vgt_primary <= 1));
	if (vgt->vm_id != 0 &&
		(vp.vgt_primary == 0 || (vp.vgt_primary == -1 && !vgt_primary))) {
		/* Mark vgt device as non primary VGA */
		cfg_space[VGT_REG_CFG_CLASS_CODE] = VGT_PCI_CLASS_VGA;
		cfg_space[VGT_REG_CFG_SUB_CLASS_CODE] = VGT_PCI_CLASS_VGA_OTHER;
		cfg_space[VGT_REG_CFG_CLASS_PROG_IF] = VGT_PCI_CLASS_VGA_OTHER;
	}

	state_sreg_init (vgt);
	state_vreg_init(vgt);

	if ((rc = vgt_vstate_irq_init(vgt)) < 0)
		goto err;

	/* setup the ballooning information */
	if (vgt->ballooning) {
		__vreg64(vgt, vgt_info_off(magic)) = VGT_MAGIC;
		__vreg(vgt, vgt_info_off(version_major)) = 1;
		__vreg(vgt, vgt_info_off(version_minor)) = 0;
		__vreg(vgt, vgt_info_off(display_ready)) = 0;
		__vreg(vgt, vgt_info_off(vgt_id)) = vgt->vgt_id;
		__vreg(vgt, vgt_info_off(avail_rs.low_gmadr.my_base)) = vgt_visible_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.low_gmadr.my_size)) = vgt_aperture_sz(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.high_gmadr.my_base)) = vgt_hidden_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.high_gmadr.my_size)) = vgt_hidden_gm_sz(vgt);

		__vreg(vgt, vgt_info_off(avail_rs.fence_num)) = vgt->fence_sz;
		vgt_info("filling VGT_PVINFO_PAGE for dom%d:\n"
			"   visable_gm_base=0x%llx, size=0x%llx\n"
			"   hidden_gm_base=0x%llx, size=0x%llx\n"
			"   fence_base=%d, num=%d\n",
			vgt->vm_id,
			vgt_visible_gm_base(vgt), vgt_aperture_sz(vgt),
			vgt_hidden_gm_base(vgt), vgt_hidden_gm_sz(vgt),
			vgt->fence_base, vgt->fence_sz);
	}

	pdev->device[vgt->vgt_id] = vgt;
	list_add(&vgt->list, &pdev->rendering_idleq_head);

	if (vgt->vm_id != 0){
		/* HVM specific init */
		if ((rc = vgt_hvm_info_init(vgt)) < 0 ||
			(rc = vgt_hvm_enable(vgt)) < 0)
			goto err;
		if (pdev->enable_ppgtt) {
			hash_init((vgt->wp_table));
			vgt_init_shadow_ppgtt(vgt);
		}
	}

	if (vgt->vm_id) {
		vgt_ops->boot_time = 0;

		/* a special debug mode to give full access to hvm guest */
		if (hvm_render_owner)
			current_render_owner(pdev) = vgt;

		if (hvm_dpy_owner)
			current_foreground_vm(pdev) = vgt;
	}
	bitmap_zero(vgt->enabled_rings, MAX_ENGINES);
	bitmap_zero(vgt->started_rings, MAX_ENGINES);

	/* create debugfs per vgt */
	if (vgt_create_debugfs(vgt)) {
		vgt_err("failed to create debugfs for vgt-%d\n",
			vgt->vgt_id);
		goto err;
	}

	if (vgt_create_mmio_dev(vgt)) {
		vgt_err("failed to create mmio devnode for vgt-%d\n",
				vgt->vgt_id);
		goto err;
	}

	/* initialize i2c states */
	vgt_init_i2c_bus(&vgt->vgt_i2c_bus);
	/* assign aux_ch vregs for aux_ch virtualization */
	vgt_init_aux_ch_vregs(&vgt->vgt_i2c_bus, vgt->state.vReg);
	vgt_propagate_edid(vgt, -1);

	*ptr_vgt = vgt;

	/* initialize context scheduler infor */
	if (event_based_qos)
		vgt_init_sched_info(vgt);

	if (shadow_tail_based_qos)
		vgt_init_rb_tailq(vgt);

	return 0;
err:
	kfree(vgt->irq_vstate);
	if ( vgt->aperture_base > 0)
		free_vm_aperture_gm_and_fence(vgt);
	vfree(vgt->vgtt);
	vfree(vgt->state.vReg);
	vfree(vgt->state.sReg);
	if (vgt->vgt_id >= 0)
		free_vgt_id(vgt->vgt_id);
	kfree(vgt);
	return rc;
}

void vgt_release_instance(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;
	struct list_head *pos;
	struct vgt_device *v = NULL;

	printk("prepare to destroy vgt (%d)\n", vgt->vgt_id);

	/* destroy vgt_mmio_device */
	vgt_destroy_mmio_dev(vgt);

	vgt_destroy_debugfs(vgt);

	spin_lock_irq(&pdev->lock);
	printk("check display ownership...\n");

	if (current_display_owner(pdev) == vgt) {
		vgt_dbg("switch display ownership back to dom0\n");
		current_display_owner(pdev) = vgt_dom0;
	}

	if (current_foreground_vm(pdev) == vgt) {
		vgt_dbg("switch foreground vm back to dom0\n");
		do_vgt_fast_display_switch(vgt_dom0);
	}

	printk("check render ownership...\n");
	list_for_each (pos, &pdev->rendering_runq_head) {
		v = list_entry (pos, struct vgt_device, list);
		if (v == vgt)
			break;
	}

	if (v != vgt)
		printk("vgt instance has been removed from run queue\n");
	else if (hvm_render_owner || current_render_owner(pdev) != vgt) {
		printk("remove vgt(%d) from runqueue safely\n",
			vgt->vgt_id);
		vgt_disable_render(vgt);
	} else {
		printk("vgt(%d) is current owner, request reschedule\n",
			vgt->vgt_id);
		vgt->force_removal = true;
		next_sched_vgt = vgt_dom0;
		vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
		wmb();
	}
	if (previous_render_owner(pdev) == vgt)
		previous_render_owner(pdev) = NULL;

	spin_unlock_irq(&pdev->lock);
	if (vgt->force_removal)
		/* wait for removal completion */
		wait_event(pdev->destroy_wq, !vgt->force_removal);

	printk("release display/render ownership... done\n");

	/* FIXME: any conflicts between destroy_wq ? */
	if (shadow_tail_based_qos)
		vgt_destroy_rb_tailq(vgt);

	vgt_hvm_info_deinit(vgt);
	vgt->pdev->device[vgt->vgt_id] = NULL;

	vgt_vstate_irq_exit(vgt);
	/* already idle */
	list_del(&vgt->list);

	for (i = 0; i < EDID_MAX; ++ i) {
		if (vgt->vgt_edids[i]) {
			kfree(vgt->vgt_edids[i]);
			vgt->vgt_edids[i] = NULL;
		}
	}

	if (vgt->pdev->enable_ppgtt)
		vgt_destroy_shadow_ppgtt(vgt);

	free_vm_aperture_gm_and_fence(vgt);
	free_vm_rsvd_aperture(vgt);
	vgt_vmem_destroy(vgt);
	vfree(vgt->vgtt);
	vfree(vgt->state.vReg);
	vfree(vgt->state.sReg);
	free_vgt_id(vgt->vgt_id);
	kfree(vgt);
	printk("vGT: vgt_release_instance done\n");
}
