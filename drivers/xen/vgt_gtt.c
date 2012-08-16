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
 * BSD LICENSE
 *
 * Copyright(c) 2012 Intel Corporation. All rights reserved.
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
#include "vgt_reg.h"
u64 gtt_mmio_rcnt=0;
u64 gtt_mmio_wcnt=0;
u64 gtt_mmio_wcycles=0;
u64 gtt_mmio_rcycles=0;

/* Don't be confused. 'g_pfn' is actually page _address_, instead of page frame
 * number. And return value is also machine page _address_, but not frame
 * number.
 */
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

bool gtt_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	uint32_t g_gtt_index;
        cycles_t t0, t1;

	ASSERT(bytes == 4);

	t0 = get_cycles();
	gtt_mmio_rcnt++;
	off -= VGT_MMIO_SPACE_SZ;
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
	off -= VGT_MMIO_SPACE_SZ;

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
	unsigned long g_addr = 0, h_addr = 0;
	u32 addr_mask = 0, ctl_mask = 0;

	dprintk("PTE WP handler: offset 0x%x data 0x%lx bytes %d\n", offset, *(unsigned long *)p_data, bytes);

	/* need to know: fault pfn, write fault address, write fault data */

	/* find shadow PTE */
	/* XXX search PDE table for PTE table index */
	for (i = 0; i < 1024; i++) {
		if ((vgt->shadow_pde_table[i].virtual_phyaddr & PAGE_MASK) == (offset & PAGE_MASK)) {
			dprintk(KERN_INFO "zhen: Found PTE page at 0x%lx (%d)\n", offset & PAGE_MASK, i);
			break;
		}
	}
	if (i == 1024) {
		printk(KERN_ERR "Failed to find PTE page at 0x%x\n", offset);
		return false;
	}

	g_addr = *(unsigned long*)p_data;

	/* find entry index, fill in shadow PTE */
	pte = vgt->shadow_pte_table[i].virt;
	index = (offset & (PAGE_SIZE - 1)) >> 2;

	if (pdev->is_ivybridge) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (pdev->is_haswell) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	h_addr = g2m_pfn(vgt->vm_id, g_addr);
	if (h_addr == INVALID_MFN) {
		printk(KERN_ERR "Failed to convert WP page at 0x%lx\n", g_addr);
		return false;
	}

	pte[index] = h_addr | ((h_addr >> 28) & addr_mask);
	pte[index] |= g_addr & ctl_mask;
	pte[index] |= _REGBIT_PTE_VALID;

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
void *vgt_ppgtt_map_guest_pte_page(struct vgt_device *vgt, unsigned long gpfn)
{
	unsigned long mfn;
	struct vm_struct *area;

	mfn = g2m_pfn(vgt->vm_id, gpfn);
	if (mfn == INVALID_MFN) {
		printk(KERN_ERR "Try to get VM PTE page frame number failed!\n");
		return NULL;
	}

	area = xen_remap_domain_mfn_range_in_kernel(mfn >> PAGE_SHIFT, 1, vgt->vm_id);
	return (area == NULL) ? NULL : area->addr;
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
	vgt->shadow_pde_table[idx].shadow_pte_phyaddr = p->shadow_mpfn << PAGE_SHIFT;

	p->virt = shadow_ent = page_address(p->pte_page);

	/* access VM's pte page */
	ent = vgt_ppgtt_map_guest_pte_page(vgt, virt_pte);
	if (ent == NULL) {
		printk(KERN_ERR "Failed to map guest PTE page!\n");
		return -1;
	}

	if (pdev->is_ivybridge) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (pdev->is_haswell) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	/* for each PTE entry */
	for (i = 0; i < 1024; i++) {
		/* check valid */
		if ((ent[i] & _REGBIT_PTE_VALID) == 0)
			continue;
		/* get page physical address */
		addr = (u64)(ent[i] & addr_mask) << 20 | (ent[i] & 0xfffff000);

		/* get real physical address for that page */
		s_addr = g2m_pfn(vgt->vm_id, addr);

		/* update shadow PTE entry with targe page address */
		shadow_ent[i] = s_addr | ((s_addr >> 28) & addr_mask);
		shadow_ent[i] |= ent[i] & ctl_mask;
		shadow_ent[i] |= _REGBIT_PTE_VALID;
	}
	/* XXX unmap guest VM page? */
	return 0;
}

bool vgt_setup_ppgtt(struct vgt_device *vgt)
{
	u32 base = vgt->rb[0].sring_ppgtt_info.base;
	int pde_entries = 512;	/* XXX current assume 512 entries for 2G mapping */
	int i;
	u32 pde, shadow_pde;
	dma_addr_t pte_phy;
	u32 gtt_base;

	printk(KERN_INFO "vgt_setup_ppgtt on vm %d: PDE base 0x%x\n", vgt->vm_id, base);

	gtt_base = base >> PAGE_SHIFT;

	for (i = 0; i < pde_entries; i++) {
		/* Just use guest virtual value instead of real machine address */
		pde = vgt->vgtt[gtt_base + i];

		if (!(pde & _REGBIT_PDE_VALID))
			continue;

		if ((pde & _REGBIT_PDE_PAGE_32K)) {
			printk("zhen: 32K page in PDE!\n");
			continue;
		}
		pde &= ~3;
		pte_phy = (u64)(pde & 0xff0) << 20 | (u64)(pde & 0xfffff000);

		vgt->shadow_pde_table[i].virtual_phyaddr = pte_phy;

		/* allocate shadow PTE page, and fix it up */
		vgt_ppgtt_shadow_pte_init(vgt, i, pte_phy);

		/* WP original PTE page */
		vgt_set_wp_page(vgt, pte_phy >> PAGE_SHIFT);

		shadow_pde = vgt->shadow_pde_table[i].shadow_pte_phyaddr;
		shadow_pde |= (vgt->shadow_pde_table[i].shadow_pte_phyaddr >> 28) & 0xff0;
		shadow_pde |= _REGBIT_PDE_VALID;

		/* write_gtt with new shadow PTE page address */
		vgt_write_gtt(vgt->pdev, gtt_base + i, shadow_pde);
	}
	return true;
}

bool vgt_init_shadow_ppgtt(struct vgt_device *vgt)
{
	int i;
	vgt_ppgtt_pte_t *p;

	dprintk("vgt_init_shadow_ppgtt for vm %d\n", vgt->vm_id);

	for (i = 0; i < 1024; i++) {
		p = &vgt->shadow_pte_table[i];
		p->pte_page = alloc_page(GFP_KERNEL);
		if (!p->pte_page) {
			printk(KERN_ERR "Init shadow PTE page failed!\n");
			return false;
		}

		p->shadow_mpfn = g2m_pfn(vgt->vm_id, page_to_phys(p->pte_page));
		if (p->shadow_mpfn == INVALID_MFN) {
			printk(KERN_ERR "Failed to get mpfn for shadow PTE!\n");
			return false;
		}
	}
	return true;
}

void vgt_destroy_shadow_ppgtt(struct vgt_device *vgt)
{
	int i;
	vgt_ppgtt_pte_t *p;

	for (i = 0; i < 1024; i++) {
		p = &vgt->shadow_pte_table[i];
		__free_page(p->pte_page);
	}
}
