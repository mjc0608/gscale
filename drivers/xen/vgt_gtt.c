/*
 * vGT core module
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 Intel Corporation. All rights reserved.
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
u64 gtt_mmio_rcnt=0;
u64 gtt_mmio_wcnt=0;
u64 gtt_mmio_wcycles=0;
u64 gtt_mmio_rcycles=0;

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
		printk(KERN_ERR "failed to get mfn for gpfn(0x%lx)\n, errno=%d\n", g_pfn,rc);
		return INVALID_MFN;
	}

	return pfn_list[0];
}

/*
 * IN:  p_gtt_val - guest GTT entry
 * OUT: m_gtt_val - translated machine GTT entry from guest GTT entry
 *					on success, it will be written with correct value
 *					otherwise, it will not be written
 */
int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val)
{
	gtt_pte_t pte, *p_pte;
	unsigned long g_pfn, mfn;

	p_pte = &pte;
	gtt_pte_make(p_pte, p_gtt_val);

	if (!gtt_pte_valid(p_pte)){
		*m_gtt_val = p_gtt_val;
		return 0;
	}

	g_pfn = gtt_pte_get_pfn(p_pte);
	mfn = g2m_pfn(vgt->vm_id, g_pfn);
	if (mfn == INVALID_MFN){
		printk(KERN_ERR "Invalid gtt entry 0x%x\n", p_gtt_val);
		return -EINVAL;
	}
	gtt_pte_set_pfn(p_pte, mfn);

	*m_gtt_val = gtt_pte_get_val(p_pte);

	return 0;
}

/*  translate gma (graphics memory address) to guest phyiscal address
 *  by walking guest GTT table
 */
unsigned long vgt_gma_2_gpa(struct vgt_device *vgt, unsigned long gma, bool ppgtt)
{
   gtt_pte_t pte;
   uint32_t gtt_index;
   unsigned long pfn, pa;

   if (ppgtt){
       /*TODO: add PPGTT support */
       BUG();
   } else {
       /* Global GTT */
       gtt_index = gma >> GTT_PAGE_SHIFT;
       if (gtt_index * GTT_ENTRY_SIZE >= vgt->vgtt_sz){
           printk(KERN_ERR "invalid gma %lx\n", gma);
		   return INVALID_ADDR;
       }

       gtt_pte_make(&pte, vgt->vgtt[gtt_index]);
       pfn = gtt_pte_get_pfn(&pte);
       pa = (pfn << PAGE_SHIFT) + (gma & ~PAGE_MASK);
   }
   return pa;
}

void* vgt_gma_to_va(struct vgt_device *vgt, unsigned long gma, bool ppgtt)
{
	unsigned long gpa;

	gpa = vgt_gma_2_gpa(vgt, gma, ppgtt);
	if (gpa == INVALID_ADDR){
		return NULL;
	}

	return vgt_vmem_gpa_2_va(vgt, gpa);
}

bool gtt_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	uint32_t g_gtt_index;
        cycles_t t0, t1;

	ASSERT(bytes == 4);

	t0 = get_cycles();
	gtt_mmio_rcnt++;
	off -= vgt->pdev->mmio_size;
	if (off >= vgt->vgtt_sz) {
		dprintk("vGT(%d): captured out of range GTT read on off %x\n", vgt->vgt_id, off);
		return false;
	}

	g_gtt_index = GTT_OFFSET_TO_INDEX(off);
	*(uint32_t*)p_data = vgt->vgtt[g_gtt_index];
	t1 = get_cycles();
	t1 -= t0;
	gtt_mmio_rcycles += (u64) t1;
	return true;
}

#define GTT_INDEX_MB(x) ((SIZE_1MB*(x)) >> GTT_PAGE_SHIFT)

bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	uint32_t g_gtt_val, h_gtt_val, g_gtt_index, h_gtt_index;
	int rc;
	uint64_t g_addr;
        cycles_t t0, t1;

	ASSERT(bytes == 4);

	t0 = get_cycles();
	gtt_mmio_wcnt++;
	off -= vgt->pdev->mmio_size;

	g_gtt_index = GTT_OFFSET_TO_INDEX(off);
	g_gtt_val = *(uint32_t*)p_data;
	vgt->vgtt[g_gtt_index] = g_gtt_val;

	g_addr = g_gtt_index << GTT_PAGE_SHIFT;
	/* the VM may configure the whole GM space when ballooning is used */
	if (!g_gm_is_visible(vgt, g_addr) && !g_gm_is_hidden(vgt, g_addr)) {
		static int count = 0;

		/* print info every 32MB */
		if (!(count % 8192))
			dprintk("vGT(%d): capture ballooned write for %d times (%x)\n",
				vgt->vgt_id, count, off);

		count++;
		/* in this case still return true since the impact is on vgtt only */
		goto out;
	}

	rc = gtt_p2m(vgt, g_gtt_val, &h_gtt_val);
	if (rc < 0){
		printk("vGT(%d): failed to translate g_gtt_val(%x)\n", vgt->vgt_id, g_gtt_val);
		return false;
	}

	h_gtt_index = g2h_gtt_index(vgt, g_gtt_index);
	vgt_write_gtt( vgt->pdev, h_gtt_index, h_gtt_val );
#ifdef DOM0_DUAL_MAP
	if ( (h_gtt_index >= GTT_INDEX_MB(128)) && (h_gtt_index < GTT_INDEX_MB(192)) ){
		vgt_write_gtt( vgt->pdev, h_gtt_index - GTT_INDEX_MB(128), h_gtt_val );
	}
#endif
out:
	t1 = get_cycles();
	t1 -= t0;
	gtt_mmio_wcycles += (u64) t1;

	return true;
}

/* So idea is that for PPGTT base in GGTT, real PDE entry will point to shadow
 * PTE, then shadow PTE entry will point to final page. So have to fix shadow
 * PTE address in PDE, and final page address in PTE. That's two-phrase address
 * fixing.
 */

/* Handle write protect fault on virtual PTE page */
bool vgt_ppgtt_handle_pte_wp(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	int index, i;
	u32 *pte;
	unsigned long g_val = 0, g_addr = 0, h_addr = 0;
	u32 addr_mask = 0, ctl_mask = 0;

	dprintk("PTE WP handler: offset 0x%x data 0x%lx bytes %d\n", offset, *(unsigned long *)p_data, bytes);

	/* need to know: fault pfn, write fault address, write fault data */

	/* find shadow PTE */
	/* XXX search PDE table for PTE table index */
	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		if ((vgt->shadow_pde_table[i].virtual_phyaddr & PAGE_MASK) == (offset & PAGE_MASK)) {
			dprintk(KERN_INFO "zhen: Found PTE page at 0x%lx (%d)\n", offset & PAGE_MASK, i);
			break;
		}
	}
	if (i == VGT_PPGTT_PDE_ENTRIES) {
		printk(KERN_ERR "Failed to find PTE page at 0x%x\n", offset);
		return false;
	}

	g_val = *(unsigned long*)p_data;

	/* find entry index, fill in shadow PTE */

	index = (offset & (PAGE_SIZE - 1)) >> 2;

	if (pdev->is_ivybridge) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (pdev->is_haswell) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	g_addr = (unsigned long)(g_val & addr_mask) << 28 | (unsigned long)(g_val & 0xfffff000);

	h_addr = g2m_pfn(vgt->vm_id, (g_addr >> PAGE_SHIFT));
	if (h_addr == INVALID_MFN) {
		printk(KERN_ERR "Failed to convert WP page at 0x%lx\n", g_addr);
		return false;
	}
	h_addr <<= PAGE_SHIFT;

	pte = kmap_atomic(vgt->shadow_pte_table[i].pte_page);
	pte[index] = (h_addr & ~0xfff) | ((h_addr >> 28) & addr_mask);
	pte[index] |= g_val & ctl_mask;
	pte[index] |= _REGBIT_PTE_VALID;
	clflush((u8 *)pte + index * 4);
	kunmap_atomic(pte);

	dprintk("WP: PDE[%d], PTE[%d], entry 0x%x, g_addr 0x%lx, h_addr 0x%lx\n", i, index, pte[index], g_addr, h_addr);

	return true;
}

/* handler to set page wp */

int vgt_set_wp_pages(struct vgt_device *vgt, int nr, unsigned long *pages)
{
	xen_hvm_vgt_wp_pages_t req;
	int i, rc = 0;
	unsigned long *p = pages;

	if (nr > MAX_WP_BATCH_PAGES)
		return -1;

	memset(&req, 0, sizeof(xen_hvm_vgt_wp_pages_t));
	req.domid = vgt->vm_id;
	req.set = 1;
	req.nr_pages = nr;

	for (i = 0; i < nr; i++)
		req.wp_pages[i] = *pages++;

	rc = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (rc)
		printk(KERN_ERR "Set WP pages failed!\n");
	else {
		/* Add pages in hash table */
		struct mmio_hash_table *mht;

		for (i = 0; i < nr; i++) {
			mht = kmalloc(sizeof(struct mmio_hash_table), GFP_KERNEL);
			if (!mht)
				break; /* XXX */
			mht->mmio_base = *p++;
			mht->write = vgt_ppgtt_handle_pte_wp;
			vgt_hash_register_entry(vgt, VGT_HASH_WP_PAGE, mht);
		}
	}

	return rc;
}

int vgt_set_wp_page(struct vgt_device *vgt, unsigned long pfn)
{
	return vgt_set_wp_pages(vgt, 1, &pfn);
}

int vgt_unset_wp_pages(struct vgt_device *vgt, int nr, unsigned long *pages)
{
	xen_hvm_vgt_wp_pages_t req;
	int i, rc = 0;
	unsigned long *p = pages;

	if (nr > MAX_WP_BATCH_PAGES)
		return -1;

	memset(&req, 0, sizeof(xen_hvm_vgt_wp_pages_t));
	req.domid = vgt->vm_id;
	req.set = 0;
	req.nr_pages = nr;

	for (i = 0; i < nr; i++)
		req.wp_pages[i] = *pages++;

	rc = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (rc)
		printk(KERN_ERR "Unset WP pages failed!\n");
	else {
		for (i = 0; i < nr; i++)
			vgt_hash_remove_entry(vgt, VGT_HASH_WP_PAGE, *p++);
	}

	return rc;
}

int vgt_unset_wp_page(struct vgt_device *vgt, unsigned long pfn)
{
	return vgt_unset_wp_pages(vgt, 1, &pfn);
}

/* handler to map guest page in dom0 kernel space.
 * XXX no unmap for now, assume current PTE pages are always allocated.
 */
struct vm_struct *vgt_ppgtt_map_guest_pte_page(struct vgt_device *vgt, unsigned long gaddr)
{
	struct vm_struct *area;

	area = xen_remap_domain_mfn_range_in_kernel((gaddr >> PAGE_SHIFT), 1, vgt->vm_id);
	return (area == NULL) ? NULL : area;
}


int vgt_ppgtt_shadow_pte_init(struct vgt_device *vgt, int idx, dma_addr_t virt_pte)
{
	int i;
	vgt_ppgtt_pte_t *p = &vgt->shadow_pte_table[idx];
	u32 *ent;
	u32 *shadow_ent;
	dma_addr_t addr, s_addr;
	u32 addr_mask = 0, ctl_mask = 0;
	struct pgt_device *pdev = vgt->pdev;

	if (!p->pte_page) {
		printk(KERN_ERR "Uninitialized shadow PTE page at index %d?\n", idx);
		return -1;
	}
	vgt->shadow_pde_table[idx].shadow_pte_maddr = p->shadow_addr;

	/* access VM's pte page */
	p->guest_pte_vm = vgt_ppgtt_map_guest_pte_page(vgt, virt_pte);
	if (p->guest_pte_vm == NULL) {
		printk(KERN_ERR "Failed to map guest PTE page!\n");
		return -1;
	}
	ent = p->guest_pte_vm->addr;

	if (pdev->is_ivybridge) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (pdev->is_haswell) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	shadow_ent = kmap_atomic(p->pte_page);

	/* for each PTE entry */
	for (i = 0; i < 1024; i++) {
		/* check valid */
		if ((ent[i] & _REGBIT_PTE_VALID) == 0)
			continue;
		/* get page physical address */
		addr = (u64)(ent[i] & addr_mask) << 28 | (u64)(ent[i] & 0xfffff000);

		/* get real physical address for that page */
		s_addr = g2m_pfn(vgt->vm_id, (addr >> PAGE_SHIFT));
		if (s_addr == INVALID_MFN) {
			printk("vGT[%d]: Failed to get machine address for 0x%lx\n", vgt->vm_id, (unsigned long)addr);
			return -1;
		}
		s_addr <<= PAGE_SHIFT;

		/* update shadow PTE entry with targe page address */
		shadow_ent[i] = (s_addr & ~0xfff) | ((s_addr >> 28) & addr_mask);
		shadow_ent[i] |= ent[i] & ctl_mask;
		shadow_ent[i] |= _REGBIT_PTE_VALID;
	}
	kunmap_atomic(shadow_ent);
	/* XXX unmap guest VM page? */
	return 0;
}

bool vgt_setup_ppgtt(struct vgt_device *vgt)
{
	u32 base = vgt->rb[0].sring_ppgtt_info.base;
	int i;
	u32 pde, shadow_pde;
	dma_addr_t pte_phy;
	u32 gtt_base;
	unsigned int index, h_index;

	printk(KERN_INFO "vgt_setup_ppgtt on vm %d: PDE base 0x%x\n", vgt->vm_id, base);

	gtt_base = base >> PAGE_SHIFT;

	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		index = gtt_base + i;

		/* Just use guest virtual value instead of real machine address */
		pde = vgt->vgtt[index];

		if (!(pde & _REGBIT_PDE_VALID))
			continue;

		if ((pde & _REGBIT_PDE_PAGE_32K)) {
			printk("zhen: 32K page in PDE!\n");
			continue;
		}
		pde &= ~0xf;
		pte_phy = (u64)(pde & 0xff0) << 28 | (u64)(pde & 0xfffff000);

		vgt->shadow_pde_table[i].virtual_phyaddr = pte_phy;

		/* allocate shadow PTE page, and fix it up */
		vgt_ppgtt_shadow_pte_init(vgt, i, pte_phy);

		/* WP original PTE page */
		vgt_set_wp_page(vgt, pte_phy >> PAGE_SHIFT);

		shadow_pde = vgt->shadow_pde_table[i].shadow_pte_maddr & ~0xfff;
		shadow_pde |= (vgt->shadow_pde_table[i].shadow_pte_maddr >> 28) & 0xff0;
		shadow_pde |= _REGBIT_PDE_VALID;

		h_index = g2h_gtt_index(vgt, index);

		/* write_gtt with new shadow PTE page address */
		vgt_write_gtt(vgt->pdev, h_index, shadow_pde);
	}
	return true;
}

bool vgt_init_shadow_ppgtt(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_ppgtt_pte_t *p;
	dma_addr_t dma_addr;

	dprintk("vgt_init_shadow_ppgtt for vm %d\n", vgt->vm_id);

	/* each PDE entry has one shadow PTE page */
	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		p = &vgt->shadow_pte_table[i];
		p->pte_page = alloc_page(GFP_KERNEL);
		if (!p->pte_page) {
			printk(KERN_ERR "Init shadow PTE page failed!\n");
			return false;
		}

		dma_addr = pci_map_page(pdev->pdev, p->pte_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
			printk(KERN_ERR "Pci map shadow PTE page failed!\n");
			return false;
		}

		p->shadow_addr = dma_addr;
	}
	return true;
}

void vgt_destroy_shadow_ppgtt(struct vgt_device *vgt)
{
	int i;
	vgt_ppgtt_pte_t *p;

	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		vgt_unset_wp_page(vgt, vgt->shadow_pde_table[i].virtual_phyaddr >> PAGE_SHIFT);

		p = &vgt->shadow_pte_table[i];
		xen_unmap_domain_mfn_range_in_kernel(p->guest_pte_vm, 1, vgt->vm_id);
		__free_page(p->pte_page);
	}
}
