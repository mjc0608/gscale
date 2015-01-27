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

/*
 * NOTE:
 * This file contains hypervisor specific interactions to
 * implement the concept of mediated pass-through framework.
 * What this file provides is actually a general abstraction
 * of in-kernel device model, which is not vgt specific.
 *
 * Now temporarily in vgt code. long-term this should be
 * in hypervisor (xen/kvm) specific directory
 */
#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>
#include <xen/xen-ops.h>
#include <xen/interface/memory.h>
#include <xen/interface/hvm/params.h>

#include "vgt.h"

/* Translate from VM's guest pfn to machine pfn */
static unsigned long xen_g2m_pfn(int vm_id, unsigned long g_pfn)
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
		printk("failed to get mfn for gpfn(0x%lx)\n, errno=%d\n", g_pfn, rc);
		return INVALID_MFN;
	}

	return pfn_list[0];
}

static int xen_get_max_gpfn(int vm_id)
{
	domid_t dom_id = vm_id;
	int max_gpfn = HYPERVISOR_memory_op(XENMEM_maximum_gpfn, &dom_id);
	BUG_ON(max_gpfn < 0);
	return max_gpfn;
}

static int xen_pause_domain(int vm_id)
{
	int rc;
	struct xen_domctl domctl;

	domctl.domain = vm_id;
	domctl.cmd = XEN_DOMCTL_pausedomain;
	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&domctl);
	if (rc != 0)
		printk("HYPERVISOR_domctl pausedomain fail with %d!\n", rc);

	return rc;
}

static int xen_shutdown_domain(int vm_id)
{
	int rc;
	struct sched_remote_shutdown r;

	r.reason = SHUTDOWN_crash;
	r.domain_id = vm_id;
	rc = HYPERVISOR_sched_op(SCHEDOP_remote_shutdown, &r);
	if (rc != 0)
		printk("HYPERVISOR_sched_op failed: %d\n", rc);
	return rc;
}

static int xen_domain_iomem_perm(uint32_t domain_id, uint64_t first_mfn,
                               uint64_t nr_mfns, uint8_t allow_access)
{
	struct xen_domctl arg;
	int rc;

	arg.domain = domain_id;
	arg.cmd = XEN_DOMCTL_iomem_permission;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	arg.u.iomem_perm.first_mfn = first_mfn;
	arg.u.iomem_perm.nr_mfns = nr_mfns;
	arg.u.iomem_perm.allow_access = allow_access;
	rc = HYPERVISOR_domctl(&arg);

	return rc;
}

static int xen_hvm_memory_mapping(int vm_id, uint64_t first_gfn, uint64_t first_mfn,
				  uint32_t nr_mfns, uint32_t add_mapping)
{
	struct xen_domctl arg;
	int rc;

	if (add_mapping) {
		rc = xen_domain_iomem_perm(vm_id, first_mfn, nr_mfns, 1);
	        if (rc < 0) {
			printk(KERN_ERR "xen_domain_iomem_perm failed: %d\n", rc);
	        	return rc;
		}
	}

	arg.domain = vm_id;
	arg.cmd = XEN_DOMCTL_memory_mapping;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	arg.u.memory_mapping.first_gfn = first_gfn;
	arg.u.memory_mapping.first_mfn = first_mfn;
	arg.u.memory_mapping.nr_mfns = nr_mfns;
	arg.u.memory_mapping.add_mapping = add_mapping;

	rc = HYPERVISOR_domctl(&arg);
	if (rc < 0) {
		printk(KERN_ERR "HYPERVISOR_domctl failed: %d\n", rc);
		return rc;
	}

	if (!add_mapping) {
		rc = xen_domain_iomem_perm(vm_id, first_mfn, nr_mfns, 0);
	        if (rc < 0) {
			printk(KERN_ERR "xen_domain_iomem_perm failed: %d\n", rc);
			return rc;
		}
	}

	return rc;
}

static int xen_map_mfn_to_gpfn(int vm_id, unsigned long gpfn,
	unsigned long mfn, int nr, int map)
{
	int rc;
	rc = xen_hvm_memory_mapping(vm_id, gpfn, mfn, nr,
			map ? DPCI_ADD_MAPPING : DPCI_REMOVE_MAPPING);
	if (rc != 0)
		printk("xen_hvm_memory_mapping failed: %d\n", rc);
	return rc;
}

static int xen_set_trap_area(struct vgt_device *vgt, uint64_t start, uint64_t end, bool map)
{
	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

	return hvm_map_io_range_to_ioreq_server(vgt, 1, start, end, map);
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

static int hvm_create_iorequest_server(struct vgt_device *vgt)
{
	struct xen_hvm_create_ioreq_server arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.handle_bufioreq = 0;
	r = HYPERVISOR_hvm_op(HVMOP_create_ioreq_server, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot create io-requset server: %d!\n", r);
		return r;
	}
	vgt->iosrv_id = arg.id;

	return r;
}

int hvm_toggle_iorequest_server(struct vgt_device *vgt, bool enable)
{
	struct xen_hvm_set_ioreq_server_state arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = vgt->iosrv_id;
	arg.enabled = enable;
	r = HYPERVISOR_hvm_op(HVMOP_set_ioreq_server_state, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot %s io-request server: %d!\n",
			enable ? "enable" : "disbale",  r);
		return r;
	}

       return r;
}

static int hvm_get_ioreq_pfn(struct vgt_device *vgt, uint64_t *value)
{
	struct xen_hvm_get_ioreq_server_info arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = vgt->iosrv_id;
	r = HYPERVISOR_hvm_op(HVMOP_get_ioreq_server_info, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot get ioreq pfn: %d!\n", r);
		return r;
	}
	*value = arg.ioreq_pfn;
	return r;
}

int hvm_destroy_iorequest_server(struct vgt_device *vgt)
{
	struct xen_hvm_destroy_ioreq_server arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = vgt->iosrv_id;
	r = HYPERVISOR_hvm_op(HVMOP_destroy_ioreq_server, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot destroy io-request server(%d): %d!\n",
			vgt->iosrv_id, r);
		return r;
	}
	vgt->iosrv_id = 0;
	return r;
}

int hvm_map_io_range_to_ioreq_server(struct vgt_device *vgt,
	int is_mmio, uint64_t start, uint64_t end, int map)
{
	xen_hvm_io_range_t arg;
	int rc;

	arg.domid = vgt->vm_id;
	arg.id = vgt->iosrv_id;
	arg.type = is_mmio ? HVMOP_IO_RANGE_MEMORY : HVMOP_IO_RANGE_PORT;
	arg.start = start;
	arg.end = end;

	if (map)
		rc = HYPERVISOR_hvm_op(HVMOP_map_io_range_to_ioreq_server, &arg);
	else
		rc = HYPERVISOR_hvm_op(HVMOP_unmap_io_range_from_ioreq_server, &arg);

	return rc;
}

int hvm_map_pcidev_to_ioreq_server(struct vgt_device *vgt, uint64_t sbdf)
{
	xen_hvm_io_range_t arg;
	int rc;

	arg.domid = vgt->vm_id;
	arg.id = vgt->iosrv_id;
	arg.type = HVMOP_IO_RANGE_PCI;
	arg.start = arg.end = sbdf;
	rc = HYPERVISOR_hvm_op(HVMOP_map_io_range_to_ioreq_server, &arg);
	if (rc < 0) {
		printk(KERN_ERR "Cannot map pci_dev to ioreq_server: %d!\n", rc);
		return rc;
	}

	return rc;
}

static int hvm_set_mem_type(struct vgt_device *vgt,
	uint16_t mem_type, uint64_t first_pfn, uint64_t nr)
{
	xen_hvm_set_mem_type_t args;
	int rc;

	args.domid = vgt->vm_id;
	args.hvmmem_type = mem_type;
	args.first_pfn = first_pfn;
	args.nr = 1;
	rc = HYPERVISOR_hvm_op(HVMOP_set_mem_type, &args);

	return rc;
}

static int hvm_wp_page_to_ioreq_server(struct vgt_device *vgt, unsigned long page, int set)
{
	int rc = 0;
	uint64_t start, end;
	uint16_t mem_type;

	start = page << PAGE_SHIFT;
	end = ((page + 1) << PAGE_SHIFT) - 1;

	rc = hvm_map_io_range_to_ioreq_server(vgt, 1, start, end, set);
	if (rc < 0) {
		printk(KERN_ERR "Failed to %s page 0x%lx to ioreq_server: %d!\n",
			set ? "map":"unmap", page , rc);
		return rc;
	}

	mem_type = set ? HVMMEM_mmio_write_dm : HVMMEM_ram_rw;
	rc = hvm_set_mem_type(vgt, mem_type, page, 1);
	if (rc < 0) {
		printk(KERN_ERR "Failed to set mem type of page 0x%lx to %s!\n", page,
			set ? "HVMMEM_mmio_write_dm":"HVMMEM_ram_rw");
		return rc;
	}
	return rc;
}

struct vm_struct *xen_map_iopage(struct vgt_device *vgt)
{
	uint64_t ioreq_pfn;
	int rc;

	rc = hvm_create_iorequest_server(vgt);
	if (rc < 0)
		return NULL;
	rc = hvm_get_ioreq_pfn(vgt, &ioreq_pfn);
	if (rc < 0) {
		hvm_destroy_iorequest_server(vgt);
		return NULL;
	}

	return xen_remap_domain_mfn_range_in_kernel(ioreq_pfn, 1, vgt->vm_id);
}

static bool xen_set_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	int r;

	if (guest_page->writeprotection)
		return true;

	r = hvm_wp_page_to_ioreq_server(vgt, guest_page->gfn, 1);
	if (r) {
		vgt_err("fail to set write protection.\n");
		return false;
	}

	guest_page->writeprotection = true;

	atomic_inc(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

static bool xen_clear_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	int r;

	if (!guest_page->writeprotection)
		return true;

	r = hvm_wp_page_to_ioreq_server(vgt, guest_page->gfn, 0);
	if (r) {
		vgt_err("fail to clear write protection.\n");
		return false;
	}

	guest_page->writeprotection = false;

	atomic_dec(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

static int xen_check_host(void)
{
	return xen_initial_domain();
}

static int xen_virt_to_mfn(void *addr)
{
	return virt_to_mfn(addr);
}

static void *xen_mfn_to_virt(int mfn)
{
	return mfn_to_virt(mfn);
}

static int xen_inject_msi(int vm_id, u32 addr_lo, u16 data)
{
	struct xen_hvm_inject_msi info = {
		.domid	= vm_id,
		.addr	= addr_lo, /* only low addr used */
		.data	= data,
	};

	return HYPERVISOR_hvm_op(HVMOP_inject_msi, &info);
}

struct kernel_dm xen_kdm = {
	.g2m_pfn = xen_g2m_pfn,
	.get_max_gpfn = xen_get_max_gpfn,
	.pause_domain = xen_pause_domain,
	.shutdown_domain = xen_shutdown_domain,
	.map_mfn_to_gpfn = xen_map_mfn_to_gpfn,
	.set_trap_area = xen_set_trap_area,
	.map_iopage = xen_map_iopage,
	.remap_mfn_range_in_kernel = xen_remap_domain_mfn_range_in_kernel,
	.unmap_mfn_range_in_kernel = xen_unmap_domain_mfn_range_in_kernel,
	.set_wp_pages = xen_set_guest_page_writeprotection,
	.unset_wp_pages = xen_clear_guest_page_writeprotection,
	.check_host = xen_check_host,
	.from_virt_to_mfn = xen_virt_to_mfn,
	.from_mfn_to_virt = xen_mfn_to_virt,
	.inject_msi = xen_inject_msi,
};
