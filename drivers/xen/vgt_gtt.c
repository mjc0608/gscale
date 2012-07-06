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
#include <xen/interface/hvm/hvm_op.h>
#include <xen/interface/hvm/params.h>
#include <xen/interface/hvm/ioreq.h>

#include <xen/vgt.h>
#include <xen/vgt-if.h>
#include <xen/vgt-parser.h> /* for g2m_pfn() */
#include "vgt_reg.h"

/* So idea is that for PPGTT base in GGTT, real PDE entry will point to shadow
 * PTE, then shadow PTE entry will point to final page. So have to fix shadow
 * PTE address in PDE, and final page address in PTE. That's two-phrase address
 * fixing.
 */

/* Handle write protect fault on virtual PTE page */
bool vgt_ppgtt_handle_pte_wp(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes)
{
	int index, i;
	u32 *pte;

	/* need to know: fault pfn, write fault address, write fault data */

	/* find shadow PTE */
	/* XXX search PDE table for PTE table index */
	for (i = 0; i < 1024; i++) {
		if (vgt->shadow_pde_table[i].virtual_phyaddr == (offset & PAGE_MASK)) {
			printk(KERN_INFO "zhen: Found PTE page at 0x%lx (%d)\n", offset & PAGE_MASK, i);
			break;
		}
	}
	if (i == 1024) {
		printk(KERN_ERR "Failed to find PTE page at 0x%x\n", offset);
		return false;
	}

	/* find entry index, fill in shadow PTE */
	pte = vgt->shadow_pte_table[i].virt;
	index = (offset & (PAGE_SIZE - 1)) >> 2;
	pte[index] = g2m_pfn(vgt->vm_id, *(dma_addr_t *)p_data >> PAGE_SHIFT);

	return true;
}

/* handler to set page wp */

int vgt_set_wp_pages(struct vgt_device *vgt, int nr, unsigned long *pages)
{
	xen_hvm_vgt_wp_pages_t req;
	int i, rc = 0;

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
			mht->mmio_base = *pages++;
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
			vgt_hash_remove_entry(vgt, VGT_HASH_WP_PAGE, *pages++);
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
void *vgt_ppgtt_map_guest_pte_page(struct vgt_device *vgt, int gpfn)
{
	unsigned long mfn;

	mfn = g2m_pfn(vgt->vm_id, gpfn);
	if (mfn == INVALID_MFN)
		return NULL;

	return xen_remap_domain_mfn_range_in_kernel(mfn, 1, vgt->vm_id);
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

	p->pte_page = alloc_page(GFP_KERNEL);
	if (!p->pte_page) {
		printk(KERN_ERR "Allocate shadow PTE page failed!\n");
		return -1; /* XXX */
	}
	/* XXX */
	p->shadow_mpfn = g2m_pfn(vgt->vm_id, page_to_pfn(p->pte_page));
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
		s_addr = g2m_pfn(vgt->vm_id, addr >> PAGE_SHIFT) << PAGE_SHIFT;

		/* update shadow PTE entry with targe page address */
		shadow_ent[i] = s_addr | ((s_addr >> 28) & 0xff0);
		shadow_ent[i] |= ent[i] & ctl_mask;
		shadow_ent[i] |= _REGBIT_PTE_VALID;
	}
	/* XXX unmap guest VM page? */
	return 0;
}

bool vgt_setup_ppgtt(struct vgt_device *vgt)
{
	u32 base = vgt->rb[0].vring_ppgtt_info.base;
	int pde_entries = 512;	/* XXX current assume 512 entries for 2G mapping */
	int i;
	u32 pde, shadow_pde;
	dma_addr_t pte_phy;

	for (i = 0; i < pde_entries; i++) {
		pde = vgt_read_gtt(vgt->pdev, base + i);
		if (!(pde & _REGBIT_PDE_VALID))
			continue;
		if ((pde & _REGBIT_PDE_PAGE_32K)) {
			printk("zhen: 32K page in PDE!\n");
			continue;
		}
		pte_phy = (u64)(pde & 0xff0) << 20 | (u64)(pde & 0xfffff000);

		vgt->shadow_pde_table[i].virtual_phyaddr = pte_phy;

		/* allocate shadow PTE page, and fix it up */
		vgt_ppgtt_shadow_pte_init(vgt, i, pte_phy);

		/* WP original PTE page */
		vgt_set_wp_page(vgt, pte_phy);

		shadow_pde = vgt->shadow_pde_table[i].shadow_pte_phyaddr;
		shadow_pde |= (vgt->shadow_pde_table[i].shadow_pte_phyaddr >> 28) & 0xff0;
		shadow_pde |= _REGBIT_PDE_VALID;

		/* write_gtt with new shadow PTE page address */
		vgt_write_gtt(vgt->pdev, base + i, shadow_pde);
	}
	return true;
}
