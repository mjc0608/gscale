/*
 * KVM implementation of mediated pass-through framework of VGT.
 *
 * Copyright(c) 2014-2015 Intel Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/kvm.h>
#include <linux/kvm_host.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <asm/page.h>
#include <asm/vmx.h>

#include "vgt.h"
#include "iodev.h"

struct kvmgt_trap_info {
	u64 base_addr;
	int len;
	struct kvm_io_device iodev;
	bool set;
};

struct kvmgt_hvm_info {
	struct kvm *kvm;
	struct vgt_device *vgt;
	struct kvmgt_trap_info trap_mmio;
};

static struct kvm *kvmgt_find_by_domid(int domid)
{
	struct kvm *kvm = NULL;

	if (unlikely(domid <= 0)) {
		vgt_err("FIXME! domid=%d\n", domid);
		return NULL;
	}

	spin_lock(&kvm_lock);
	list_for_each_entry(kvm,  &vm_list, vm_list) {
		if (kvm->domid == domid) {
			spin_unlock(&kvm_lock);
			goto found;
		}
	}
	spin_unlock(&kvm_lock);
	return NULL;

found:
	return kvm;
}

static int kvmgt_vm_getdomid(void)
{
	/* 0 is reserved for host */
	static int domid = 1;

	return domid++;
}

void kvmgt_kvm_init(struct kvm *kvm)
{
	kvm->domid = kvmgt_vm_getdomid();
	kvm->vgt_enabled = false;
	kvm->vgt = NULL;
}

void kvmgt_kvm_exit(struct kvm *kvm)
{
	vgt_params_t vp;

	if (!kvm->vgt_enabled || !kvm->vgt)
		return;

	vgt_info("release vgt resrouce for KVM!\n");
	vp.vm_id = -kvm->domid;
	vgt_ops->del_state_sysfs(vp);
	kvm->vgt_enabled = false;
}

void kvmgt_record_cf8(struct kvm_vcpu *vcpu, unsigned port, unsigned long rax)
{
	if (port == 0xcf8)
		vcpu->arch.last_cfg_addr = (u32)rax;
}

bool kvmgt_pio_is_igd_cfg(struct kvm_vcpu *vcpu)
{
	unsigned int b, d, f;
	u32 addr = vcpu->arch.last_cfg_addr;

	switch (vcpu->arch.pio.port) {
	case 0xcfc ... 0xcff:
		break;
	default:
		return false;
	}

	b = (addr >> 16) & 0xff;
	d = (addr >> 11) & 0x1f;
	f = (addr >> 8) & 0x7;

	return (b == 0 && d == 2 && f == 0);
}

bool kvmgt_pio_igd_cfg(struct kvm_vcpu *vcpu)
{
	bool ret = false;

	if (vcpu->arch.pio.in) {
		ret = vgt_ops->emulate_cfg_read(vcpu->kvm->vgt,
					(vcpu->arch.last_cfg_addr & 0xfc) + (vcpu->arch.pio.port & 3),
					vcpu->arch.pio_data,
					vcpu->arch.pio.size);
	} else {
		ret = vgt_ops->emulate_cfg_write(vcpu->kvm->vgt,
					(vcpu->arch.last_cfg_addr & 0xfc) + (vcpu->arch.pio.port & 3),
					vcpu->arch.pio_data,
					vcpu->arch.pio.size);
	}

	return ret;
}

/* Tanslate from VM's guest pfn to host pfn */
static unsigned long kvmgt_gfn_2_pfn(int vm_id, unsigned long g_pfn)
{
	pfn_t pfn;
	struct kvm *kvm;

	if (!vm_id) {
		pfn = g_pfn;
		goto out;
	}

	kvm = kvmgt_find_by_domid(vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vm_id);
		pfn = INVALID_MFN;
		return pfn;
	}

	pfn = gfn_to_pfn_atomic(kvm, g_pfn);
	if (is_error_pfn(pfn)) {
		vgt_err("gfn_to_pfn failed for VM%d, gfn: 0x%lx\n", vm_id, g_pfn);
		pfn = INVALID_MFN;
	}

out:
	return pfn;
}

static int kvmgt_pause_domain(int vm_id)
{
	/*TODO*/
	return 0;
}

static int kvmgt_shutdown_domain(int vm_id)
{
	/*TODO*/
	return 0;
}

static int kvmgt_guest_mmio_in_range(struct kvmgt_trap_info *info, gpa_t addr)
{
	return ((addr >= info->base_addr) &&
		(addr < info->base_addr + info->len));
}

static int kvmgt_guest_mmio_read(struct kvm_io_device *this, gpa_t addr,
			int len, void *val)
{
	struct kvmgt_trap_info *info = container_of(this, struct kvmgt_trap_info,
				iodev);
	struct kvmgt_hvm_info *hvm = container_of(info, struct kvmgt_hvm_info,
				trap_mmio);
	struct vgt_device *vgt = hvm->vgt;
	u64 result = 0;

	if (!kvmgt_guest_mmio_in_range(info, addr))
		return -EOPNOTSUPP;

	if (!vgt_ops->emulate_read(vgt, addr, &result, len)) {
		vgt_err("vgt_emulate_read failed!\n");
		return -EFAULT;
	}

	switch (len) {
	case 8:
		*(u64 *)val = result;
		break;
	case 1:
	case 2:
	case 4:
		memcpy(val, (char *)&result, len);
		break;
	default:
		vgt_err("FIXME! len is %d\n", len);
		return -EFAULT;
	}

	return 0;
}

static int kvmgt_guest_mmio_write(struct kvm_io_device *this, gpa_t addr,
			int len, const void *val)
{
	struct kvmgt_trap_info *info = container_of(this, struct kvmgt_trap_info,
				iodev);
	struct kvmgt_hvm_info *hvm = container_of(info, struct kvmgt_hvm_info,
				trap_mmio);
	struct vgt_device *vgt = hvm->vgt;

	if (!kvmgt_guest_mmio_in_range(info, addr))
		return -EOPNOTSUPP;

	if (!vgt_ops->emulate_write(vgt, addr, (void *)val, len)) {
		vgt_err("vgt_emulate_write failed\n");
		return 0;
	}

	return 0;
}

const struct kvm_io_device_ops trap_mmio_ops = {
	.read	= kvmgt_guest_mmio_read,
	.write	= kvmgt_guest_mmio_write,
};

static int kvmgt_set_trap_area(struct vgt_device *vgt, uint64_t start,
			uint64_t end, bool map)
{
	int r;
	struct kvm *kvm;
	bool unlock = false;
	struct kvmgt_hvm_info *info = vgt->hvm_info;

	if (info->trap_mmio.set)
		return 0;

	kvm = kvmgt_find_by_domid(vgt->vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vgt->vm_id);
		return 0;
	}

	info->trap_mmio.base_addr = start;
	info->trap_mmio.len = end - start;

	kvm_iodevice_init(&info->trap_mmio.iodev, &trap_mmio_ops);
	if (!mutex_is_locked(&kvm->slots_lock)) {
		unlock = true;
		mutex_lock(&kvm->slots_lock);
	}
	r = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS,
				info->trap_mmio.base_addr,
				info->trap_mmio.len, &info->trap_mmio.iodev);
	if (unlock)
		mutex_unlock(&kvm->slots_lock);
	if (r < 0) {
		vgt_err("kvm_io_bus_register_dev failed: %d\n", r);
		return r;
	}

	info->trap_mmio.set = true;

	return r;
}

static bool kvmgt_set_guest_page_writeprotection(struct vgt_device *vgt,
			guest_page_t *guest_page)
{
	/*TODO*/
	return 0;
}

static bool kvmgt_clear_guest_page_writeprotection(struct vgt_device *vgt,
			guest_page_t *guest_page)
{
	/*TODO*/
	return 0;
}

static int kvmgt_check_guest(void)
{
	unsigned int eax, ebx, ecx, edx;
	char s[12];
	unsigned int *i;

	/* KVM_CPUID_SIGNATURE */
	eax = 0x40000000;
	ebx = ecx = edx = 0;

	asm volatile ("cpuid"
		      : "+a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
		      :
		      : "cc", "memory");
	i = (unsigned int *)s;
	i[0] = ebx;
	i[1] = ecx;
	i[2] = edx;

	return !strncmp(s, "KVMKVMKVM", strlen("KVMKVMKVM"));
}

/* NOTE:
 * It's actually impossible to check if we are running in KVM host,
 * since the "KVM host" is simply native. So we only dectect guest here.
 */
static int kvmgt_check_host(void)
{
	return !kvmgt_check_guest();
}

static int kvmgt_virt_to_pfn(void *addr)
{
	return PFN_DOWN(__pa(addr));
}

static void *kvmgt_pfn_to_virt(unsigned long pfn)
{
	return pfn_to_kaddr(pfn);
}

static int kvmgt_hvm_init(struct vgt_device *vgt)
{
	struct kvm *kvm;
	struct kvmgt_hvm_info *info;

	kvm = kvmgt_find_by_domid(vgt->vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vgt->vm_id);
		return -EFAULT;
	}

	kvm->vgt_enabled = true;
	kvm->vgt = vgt;

	info = kzalloc(sizeof(struct kvmgt_hvm_info), GFP_KERNEL);
	if (!info) {
		vgt_err("cannot alloc hvm info\n");
		return -ENOMEM;
	}

	vgt->hvm_info = info;
	info->vgt = vgt;
	info->kvm = kvm;

	return 0;
}

static void *kvmgt_gpa_to_va(struct vgt_device *vgt, unsigned long gpa)
{
	unsigned long hva;
	gfn_t gfn = gpa_to_gfn(gpa);
	struct kvmgt_hvm_info *info = vgt->hvm_info;

	ASSERT(vgt->vm_id);
	hva = gfn_to_hva(info->kvm, gfn) + offset_in_page(gpa);

	return (void *)hva;
}

static int kvmgt_inject_msi(int vm_id, u32 addr_lo, u16 data)
{
	struct kvm_msi info = {
		.address_lo = addr_lo,
		.address_hi = 0,
		.data = data,
		.flags = 0,
	};
	struct kvm *kvm = kvmgt_find_by_domid(vm_id);

	memset(info.pad, 0, sizeof(info.pad));
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vm_id);
		return -EFAULT;
	}
	kvm_send_userspace_msi(kvm, &info);

	return 0;
}

static void kvmgt_hvm_exit(struct vgt_device *vgt)
{
	kfree(vgt->hvm_info);
}

static inline bool kvmgt_read_hva(struct vgt_device *vgt, void *hva,
			void *data, int len, int atomic)
{
	int rc;

	pagefault_disable();
	rc = atomic ? __copy_from_user_inatomic(data, hva, len) :
			__copy_from_user(data, hva, len);
	pagefault_enable();

	if (rc != 0)
		vgt_err("copy_from_user failed: rc == %d, len == %d\n", rc, len);

	return true;
}

static bool kvmgt_write_hva(struct vgt_device *vgt, void *hva, void *data,
			int len, int atomic)
{
	int r;

	pagefault_disable();
	if (atomic)
		r = __copy_to_user_inatomic((void __user *)hva, data, len);
	else
		r = __copy_to_user((void __user *)hva, data, len);
	pagefault_enable();

	if (r) {
		vgt_err("__copy_to_user failed: %d\n", r);
		return false;
	}

	return true;
}

static bool kvmgt_add_apt_slot(struct vgt_device *vgt, pfn_t p1, gfn_t g1,
			int nr_mfns, u64 hva)
{
	struct kvm_userspace_memory_region kvm_userspace_mem;
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	int r = 0;
	struct kvm *kvm = info->kvm;
	bool unlock = false;

	vgt_info("vgt-%d, p1: 0x%lx, g1: 0x%lx, nr_mfns: %d, hva: 0x%lx\n",
				vgt->vm_id, (unsigned long)p1, (unsigned long)g1,
				nr_mfns, (unsigned long)hva);
	vgt_info("vgt-%d, aperture_offset: 0x%lx\n", vgt->vm_id,
				(unsigned long)vgt->aperture_offset);

	kvm_userspace_mem.slot = VGT_APERTURE_PRIVATE_MEMSLOT;
	kvm_userspace_mem.flags = 0;
	kvm_userspace_mem.guest_phys_addr = g1 << PAGE_SHIFT;
	kvm_userspace_mem.memory_size = nr_mfns * PAGE_SIZE;

	kvm->aperture_hpa = p1 << PAGE_SHIFT;

	if (!mutex_is_locked(&kvm->slots_lock)) {
		mutex_lock(&kvm->slots_lock);
		unlock = true;
	}
	r = __kvm_set_memory_region(kvm, &kvm_userspace_mem);
	if (r) {
		vgt_err("__kvm_set_memory_region failed: %d\n", r);
		if (unlock)
			mutex_unlock(&kvm->slots_lock);
			return false;
		}
	if (unlock)
		mutex_unlock(&kvm->slots_lock);

	return true;
}

static bool kvmgt_add_opreg_slot(struct vgt_device *vgt, int nr_pages)
{
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;
	struct kvm_userspace_memory_region kvm_userspace_mem;
	bool unlock = false;
	int r = 0;

	kvm_userspace_mem.slot = VGT_OPREGION_PRIVATE_MEMSLOT;
	kvm_userspace_mem.flags = 0;
	kvm_userspace_mem.guest_phys_addr = kvm->opregion_gpa & PAGE_MASK;
	kvm_userspace_mem.memory_size = nr_pages * PAGE_SIZE;

	if (!mutex_is_locked(&kvm->slots_lock)) {
		mutex_lock(&kvm->slots_lock);
		unlock = true;
	}
	r = __kvm_set_memory_region(kvm, &kvm_userspace_mem);
	if (r) {
		vgt_err("__kvm_set_memory_region failed: %d\n", r);
	if (unlock)
		mutex_unlock(&kvm->slots_lock);
		return false;
	}
	if (unlock)
		mutex_unlock(&kvm->slots_lock);

	return true;
}

static bool kvmgt_opregion_init(struct vgt_device *vgt, uint32_t gpa)
{
	struct kvm *kvm = kvmgt_find_by_domid(vgt->vm_id);
	int rc;
	int i;

	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vgt->vm_id);
		return false;
	}
	rc = kvmgt_add_opreg_slot(vgt, VGT_OPREGION_PAGES);
	if (!rc) {
		vgt_err("VM%d: kvmgt_add_opreg_slot failed\n", vgt->vm_id);
		return false;
	}
	down_read(&(kvm->mm->mmap_sem));
	rc = get_user_pages(NULL, kvm->mm, kvm->opregion_hva,
				VGT_OPREGION_PAGES, 1, 1, vgt->state.opregion_pages, NULL);
	up_read(&kvm->mm->mmap_sem);
	if (rc != VGT_OPREGION_PAGES) {
		vgt_err("get_user_pages failed, rc is %d\n", rc);
		return false;
	}
	vgt->state.opregion_va = vmap(vgt->state.opregion_pages,
				VGT_OPREGION_PAGES, 0, PAGE_KERNEL);
	if (vgt->state.opregion_va == NULL) {
		vgt_err("VM%d: failed to allocate memory for opregion\n", vgt->vm_id);
		goto kvm_fail;
	}
	vgt->state.opregion_offset = offset_in_page(kvm->opregion_gpa);
	vgt->state.opregion_va += vgt->state.opregion_offset;
	memcpy_fromio(vgt->state.opregion_va, vgt->pdev->opregion_va,
				VGT_OPREGION_SIZE - vgt->state.opregion_offset);

	return true;

kvm_fail:
	for (i = 0; i < VGT_OPREGION_PAGES; i++)
		put_page(vgt->state.opregion_pages[i]);

	return false;
}

static int kvmgt_map_mfn_to_gpfn(int vm_id, unsigned long gpfn,
			unsigned long mfn, int nr, int map, enum map_type type)
{
	struct kvm *kvm = NULL;
	struct vgt_device *vgt = NULL;
	int r = 0;

	kvm = kvmgt_find_by_domid(vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vm_id);
		return -EFAULT;
	}
	if (!map)
		return r;

	vgt = kvm->vgt;
	switch (type) {
	case VGT_MAP_APERTURE:
		if (kvm->aperture_hpa == 0) {
			if (kvmgt_add_apt_slot(vgt, gpfn, mfn, nr,
							(u64)vgt_aperture_vbase(vgt))) {
				r = 0;
				vgt->state.bar_mapped[1] = 1;
			} else
				r = -EFAULT;
		}
		break;
	case VGT_MAP_OPREGION:
		if (vgt->state.opregion_va == NULL) {
			if (kvmgt_opregion_init(vgt, 0))
				r = 0;
			else
				r = -EFAULT;
		}
		break;
	default:
		vgt_err("type:%d not supported!\n", type);
		r = -EOPNOTSUPP;
	}

	return r;
}

struct kernel_dm kvmgt_kdm = {
	.name = "kvmgt_kdm",
	.g2m_pfn = kvmgt_gfn_2_pfn,
	.pause_domain = kvmgt_pause_domain,
	.shutdown_domain = kvmgt_shutdown_domain,
	.map_mfn_to_gpfn = kvmgt_map_mfn_to_gpfn,
	.set_trap_area = kvmgt_set_trap_area,
	.set_wp_pages = kvmgt_set_guest_page_writeprotection,
	.unset_wp_pages = kvmgt_clear_guest_page_writeprotection,
	.check_host = kvmgt_check_host,
	.from_virt_to_mfn = kvmgt_virt_to_pfn,
	.from_mfn_to_virt = kvmgt_pfn_to_virt,
	.inject_msi = kvmgt_inject_msi,
	.hvm_init = kvmgt_hvm_init,
	.hvm_exit = kvmgt_hvm_exit,
	.gpa_to_va = kvmgt_gpa_to_va,
	.read_va = kvmgt_read_hva,
	.write_va = kvmgt_write_hva,
};
EXPORT_SYMBOL(kvmgt_kdm);
