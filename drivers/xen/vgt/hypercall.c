/*
 * Interfaces coupled to Xen
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

#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>

#include <xen/xen-ops.h>
#include <xen/interface/memory.h>
#include <xen/interface/hvm/params.h>

#include "vgt.h"

/* Translate from VM's guest pfn to machine pfn */
unsigned long g2m_pfn(int vm_id, unsigned long g_pfn)
{
	struct xen_get_mfn_from_pfn pfn_arg;
	int rc;
	unsigned long pfn_list[1];

	pfn_list[0] = g_pfn;

	set_xen_guest_handle(pfn_arg.pfn_list, pfn_list);
	pfn_arg.nr_pfns = 1;
	pfn_arg.domid = vm_id;

	rc = HYPERVISOR_memory_op(XENMEM_get_mfn_from_pfn, &pfn_arg);
	if(rc < 0){
		vgt_err("failed to get mfn for gpfn(0x%lx)\n, errno=%d\n", g_pfn,rc);
		return INVALID_MFN;
	}

	return pfn_list[0];
}

int vgt_get_hvm_max_gpfn(int vm_id)
{
	domid_t dom_id = vm_id;
	int max_gpfn = HYPERVISOR_memory_op(XENMEM_maximum_gpfn, &dom_id);
	BUG_ON(max_gpfn < 0);
	return max_gpfn;
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

int vgt_pause_domain(struct vgt_device *vgt)
{
	int rc;
	struct xen_domctl domctl;

	domctl.domain = (domid_t)vgt->vm_id;
	domctl.cmd = XEN_DOMCTL_pausedomain;
	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&domctl);
	if (rc != 0)
		vgt_err("HYPERVISOR_domctl pausedomain fail with %d!\n", rc);

	return rc;
}

void vgt_crash_domain(struct vgt_device *vgt)
{
	int rc;
	struct sched_remote_shutdown r;

	r.reason = SHUTDOWN_crash;
	r.domain_id = vgt->vm_id;
	rc = HYPERVISOR_sched_op(SCHEDOP_remote_shutdown, &r);
	if (rc != 0)
		vgt_err("failed to HYPERVISOR_sched_op\n");
}

int vgt_hvm_opregion_map(struct vgt_device *vgt, int map)
{
	void *opregion;
	struct xen_hvm_vgt_map_mmio memmap;
	int rc;
	int i;

	opregion = vgt->state.opregion_va;

	memset(&memmap, 0, sizeof(memmap));
	for (i = 0; i < VGT_OPREGION_PAGES; i++) {

		memmap.first_gfn = vgt->state.opregion_gfn[i];
		memmap.first_mfn = virt_to_mfn(opregion + i*PAGE_SIZE);
		memmap.nr_mfns = 1;
		memmap.map = map;
		memmap.domid = vgt->vm_id;
		rc = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);
		if (rc != 0)
			vgt_err("vgt_hvm_map_opregion fail with %d!\n", rc);
	}

	return rc;
}

/*
 * Map the apperture space (BAR1) of vGT device for direct access.
 */
int vgt_hvm_map_apperture (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s;
	struct xen_hvm_vgt_map_mmio memmap;
	int r;

	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

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
int vgt_hvm_set_trap_area(struct vgt_device *vgt)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	struct xen_hvm_vgt_set_trap_io trap;
	uint64_t bar_s, bar_e;
	int r;

	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

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

int xen_get_nr_vcpu(int vm_id)
{
	struct xen_domctl arg;
	int rc;

	arg.domain = vm_id;
	arg.cmd = XEN_DOMCTL_getdomaininfo;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&arg);
	if (rc<0){
		printk(KERN_ERR "HYPERVISOR_domctl fail ret=%d\n",rc);
		/* assume it is UP */
		return 1;
	}

	return arg.u.getdomaininfo.max_vcpu_id + 1;
}

int hvm_get_parameter_by_dom(domid_t domid, int idx, uint64_t *value)
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

struct vm_struct *map_hvm_iopage(struct vgt_device *vgt)
{
	uint64_t ioreq_pfn;
	int rc;

	rc =hvm_get_parameter_by_dom(vgt->vm_id, HVM_PARAM_IOREQ_PFN, &ioreq_pfn);
	if (rc < 0)
		return NULL;

	return xen_remap_domain_mfn_range_in_kernel(ioreq_pfn, 1, vgt->vm_id);
}

int vgt_hvm_vmem_init(struct vgt_device *vgt)
{
	unsigned long i;

	/* Dom0 already has mapping for itself */
	ASSERT(vgt->vm_id != 0)

	ASSERT(vgt->vmem_vma == NULL);

	vgt->vmem_sz = vgt_get_hvm_max_gpfn(vgt->vm_id) + 1;
	vgt->vmem_sz <<= PAGE_SHIFT;

	vgt->vmem_vma = kmalloc(sizeof(*vgt->vmem_vma)*(vgt->vmem_sz>>VMEM_BUCK_SHIFT),GFP_KERNEL);
	if (vgt->vmem_vma == NULL){
		vgt_err("Insufficient memory for vmem_vma, vmem_sz=0x%llx\n",
				vgt->vmem_sz );
		return -ENOMEM;
	}

	for (i = 0; i < vgt->vmem_sz >> VMEM_BUCK_SHIFT; i++){
		vgt->vmem_vma[i]= xen_remap_domain_mfn_range_in_kernel(
				i << (VMEM_BUCK_SHIFT - PAGE_SHIFT),
				VMEM_BUCK_SIZE >> PAGE_SHIFT,
				vgt->vm_id);

		/* To reduce the number of err messages, we only print the
		 * message at every 64MB boundary.
		 * NOTE: normally, only the MFNs of 0MB and the MMIO hole
		 * below 4GB get mapping failure.
		 */
		if (vgt->vmem_vma[i] == NULL && (i % 64 == 0))
			vgt_dbg("vGT: VM%d: can't map %ldMB\n",
				vgt->vm_id, i<<(VMEM_BUCK_SHIFT-20));
	}

	return 0;
}

void vgt_vmem_destroy(struct vgt_device *vgt)
{
	int i;

	if(vgt->vm_id == 0)
		return;

	if (vgt->vmem_vma == NULL)
		return;

	for (i=0; i <vgt->vmem_sz >> VMEM_BUCK_SHIFT; i++){
		if (vgt->vmem_vma[i] != NULL){
			xen_unmap_domain_mfn_range_in_kernel(vgt->vmem_vma[i],
					VMEM_BUCK_SIZE >> PAGE_SHIFT, vgt->vm_id);
		}
	}
	kfree(vgt->vmem_vma);
}

void* vgt_vmem_gpa_2_va(struct vgt_device *vgt, unsigned long gpa)
{
	unsigned long buck_index;

	if (vgt->vm_id == 0)
		return (char*)mfn_to_virt(gpa>>PAGE_SHIFT) + (gpa & (PAGE_SIZE-1));

	/*
	 * At the beginning of _hvm_mmio_emulation(), we already initialize
	 * vgt->vmem_vma.
	 */
	ASSERT(vgt->vmem_vma != NULL);

	buck_index = gpa >> VMEM_BUCK_SHIFT;
	if (!vgt->vmem_vma[buck_index])
		return NULL;

	return (char*)(vgt->vmem_vma[buck_index]->addr) + (gpa & (VMEM_BUCK_SIZE -1));
}
