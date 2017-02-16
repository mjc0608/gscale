/*
 * GTT virtualization
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

#include <linux/highmem.h>
#include "vgt.h"
#include "trace.h"



/*
 * Mappings between GTT_TYPE* enumerations.
 * Following informations can be found according to the given type:
 * - type of next level page table
 * - type of entry inside this level page table
 * - type of entry with PSE set
 *
 * If the given type doesn't have such a kind of information,
 * e.g. give a l4 root entry type, then request to get its PSE type,
 * give a PTE page table type, then request to get its next level page
 * table type, as we know l4 root entry doesn't have a PSE bit,
 * and a PTE page table doesn't have a next level page table type,
 * GTT_TYPE_INVALID will be returned. This is useful when traversing a
 * page table.
 */

struct gtt_type_table_entry {
	gtt_type_t entry_type;
	gtt_type_t next_pt_type;
	gtt_type_t pse_entry_type;
};

#define GTT_TYPE_TABLE_ENTRY(type, e_type, npt_type, pse_type) \
	[type] = { \
		.entry_type = e_type, \
		.next_pt_type = npt_type, \
		.pse_entry_type = pse_type, \
	}

static struct gtt_type_table_entry gtt_type_table[] = {
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_ROOT_L4_ENTRY,
			GTT_TYPE_PPGTT_ROOT_L4_ENTRY,
			GTT_TYPE_PPGTT_PML4_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PML4_PT,
			GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
			GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_2M_ENTRY,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_1G_ENTRY,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_GGTT_PTE,
			GTT_TYPE_GGTT_PTE,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
};

static inline gtt_type_t get_next_pt_type(gtt_type_t type) {
	return gtt_type_table[type].next_pt_type;
}

static inline gtt_type_t get_entry_type(gtt_type_t type) {
	return gtt_type_table[type].entry_type;
}

static inline gtt_type_t get_pse_type(gtt_type_t type) {
	return gtt_type_table[type].pse_entry_type;
}

/*
 * Per-platform GTT entry routines.
 */

/* Mochi: modify function to support shadow GTT. 
If the current owner is the VM who is reading the GTT or Dom0, we just let it read the physical GTT.
If the current owner is ont the VM who is reading the GTT, the entries read will be from shadow GTT. */
static gtt_entry_t *gtt_get_entry32(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct vgt_device_info *info = &e->pdev->device_info;
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	void *shadow_gtt = ggtt_mm->shadow_gtt;

	ASSERT(info->gtt_entry_size == 4);

	if (!pt) {
		if(vgt->vm_id == 0){
			e->val32[0] = vgt_read_gtt(e->pdev, index);
			e->val32[1] = 0;
		}else{
			/* domU gets its GM entries */
			e->val32[0] = *((u32 *)shadow_gtt + index);
			e->val32[1] = 0;
		}
	} else {
		if (!hypervisor_access) {
			e->val32[0] = *((u32 *)pt + index);
			e->val32[1] = 0;
		} else {
			hypervisor_read_va(vgt, (u32 *)pt + index, &e->val32[0], 4, 1);
			e->val32[1] = 0;
		}
	}
	return e;
}

/* Mochi: If the domU is not the owner, its GTT modifications will be on shadow GTT. */
static gtt_entry_t *gtt_set_entry32(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct vgt_device_info *info = &e->pdev->device_info;
	
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	void *shadow_gtt = ggtt_mm->shadow_gtt;
	unsigned long hidden_gm_index_start = hidden_gm_base(vgt->pdev) >> GTT_PAGE_SHIFT;
	unsigned long fence_aperture_index_start = (hidden_gm_base(vgt->pdev) - vgt->pdev->rsvd_aperture_sz - vgt->pdev->fence_aperture_sz) >> GTT_PAGE_SHIFT;
	
	unsigned long mfn, target_mfn, target_gfn;
	struct vgt_gtt_pte_ops *pte_ops = vgt->pdev->gtt.pte_ops;
	
	vgt->gtt_modify++;
	ASSERT(info->gtt_entry_size == 4);
	if (!pt){
		if(vgt->vm_id == 0)
			vgt_write_gtt(e->pdev, index, e->val32[0]);
		else if(current_render_owner(e->pdev)==vgt){
			/* domU which owns render sets rsvd_aperture and hidden gm entries */
			vgt_write_gtt(e->pdev, index, e->val32[0]);
			*((u32 *)shadow_gtt + index) = e->val32[0];	
		}else{
			/* domU sets rsvd_aperture entries */
			if(index < hidden_gm_index_start && index >= fence_aperture_index_start)
				vgt_write_gtt(e->pdev, index, e->val32[0]);

			if(index > hidden_gm_index_start && vgt->vm_id == vgt->pdev->gtt_owner)
                                vgt_write_gtt(e->pdev, index, e->val32[0]);

			*((u32 *)shadow_gtt + index) = e->val32[0];
		}

		
		if(vgt->vm_id!=0 && index < fence_aperture_index_start){
			if(vgt->ept_umap==0){
				hypervisor_map_mfn_to_gpfn(vgt, vgt->first_gfn, vgt->first_mfn, vgt_aperture_sz(vgt) >> PAGE_SHIFT, 0, VGT_MAP_APERTURE);
				vgt->ept_umap=1;
			}
				
			/* get target mfn through the GTT entry */
			target_mfn = pte_ops->get_pfn(e);

			/* this my naive way to jump over invalid value and reduce hypercall. */
			if(vgt->invalid==target_mfn){
				vgt->invalid_count++;
			}else{
				vgt->invalid = target_mfn;
				vgt->invalid_count=0;
			}

			if(vgt->invalid_count>10)
				return e;
			
			vgt->ladder_mapping ++;
			/* domU sets aperture entries */
			/*    |<-     GM address     ->| + |<-     aperture offset    ->| = HPA -> MFN */
			mfn = ((index << GTT_PAGE_SHIFT) + phys_aperture_base(vgt->pdev)) >> PAGE_SHIFT;

			target_gfn = vgt->first_gfn + mfn - vgt->first_mfn;

			
			/* modify EPT to map GPA direct to HPA without using GTT. */
			hypervisor_map_mfn_to_gpfn(vgt, target_gfn, target_mfn, 1, 1, VGT_MAP_APERTURE);
		}
	}
	else {
		if (!hypervisor_access)
			*((u32 *)pt + index) = e->val32[0];
		else
			hypervisor_write_va(vgt, (u32 *)pt + index, &e->val32[0], 4, 1);
	}
	return e;
}

static inline gtt_entry_t *gtt_get_entry64(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		e->val64 = vgt_read_gtt64(e->pdev, index);
	else {
		if (!hypervisor_access)
			e->val64 = *((u64 *)pt + index);
		else
			hypervisor_read_va(vgt, (u64 *)pt + index, &e->val64, 8, 1);
	}
	return e;
}

static inline gtt_entry_t *gtt_set_entry64(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		vgt_write_gtt64(e->pdev, index, e->val64);
	else {
		if (!hypervisor_access)
			*((u64 *)pt + index) = e->val64;
		else
			hypervisor_write_va(vgt, (u64 *)pt + index, &e->val64, 8, 1);
	}
	return e;
}

static unsigned long gen7_gtt_get_pfn(gtt_entry_t *e)
{
	u32 pte = e->val32[0];
	u64 addr = 0;

	if (IS_SNB(e->pdev) || IS_IVB(e->pdev))
		addr = (((u64)pte & 0xff0) << 28) | (u64)(pte & 0xfffff000);
	else if (IS_HSW(e->pdev))
		addr = (((u64)pte & 0x7f0) << 28) | (u64)(pte & 0xfffff000);

	return (addr >> GTT_PAGE_SHIFT);
}

static void gen7_gtt_set_pfn(gtt_entry_t *e, unsigned long pfn)
{
	u64 addr = pfn << GTT_PAGE_SHIFT;
	u32 addr_mask = 0, ctl_mask = 0;
	u32 old_pte = e->val32[0];

	if (IS_SNB(e->pdev) || IS_IVB(e->pdev)) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (IS_HSW(e->pdev)) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	e->val32[0] = (addr & ~0xfff) | ((addr >> 28) & addr_mask);
	e->val32[0] |= (old_pte & ctl_mask);
	e->val32[0] |= _REGBIT_PTE_VALID;

	return;
}

static bool gen7_gtt_test_present(gtt_entry_t *e)
{
	return (e->val32[0] & _REGBIT_PTE_VALID);
}

static bool gen7_gtt_test_pse(gtt_entry_t *e)
{
	return false;
}

static unsigned long gen8_gtt_get_pfn(gtt_entry_t *e)
{
	if (e->type == GTT_TYPE_PPGTT_PTE_1G_ENTRY)
		return (e->val64 & (0x1ff << 30)) >> 12;
	else if (e->type == GTT_TYPE_PPGTT_PTE_2M_ENTRY)
		return (e->val64 & (0x3ffff << 21)) >> 12;
	else
		return (e->val64 >> 12) & 0x7ffffff;
}

static void gen8_gtt_set_pfn(gtt_entry_t *e, unsigned long pfn)
{
	if (e->type == GTT_TYPE_PPGTT_PTE_1G_ENTRY) {
		e->val64 &= ~(0x1ff << 30);
		pfn &= ((0x1ff << 30) >> 12);
	} else if (e->type == GTT_TYPE_PPGTT_PTE_2M_ENTRY) {
		e->val64 &= ~(0x3ffff << 21);
		pfn &= ((0x3ffff << 21) >> 12);
	} else {
		e->val64 &= ~(0x7ffffff << 12);
		pfn &= 0x7ffffff;
	}

	e->val64 |= (pfn << 12);
}

static bool gen8_gtt_test_pse(gtt_entry_t *e)
{
	/* Entry doesn't have PSE bit. */
	if (get_pse_type(e->type) == GTT_TYPE_INVALID)
		return false;

	e->type = get_entry_type(e->type);
	if (!(e->val64 & (1 << 7)))
		return false;

	e->type = get_pse_type(e->type);
	return true;
}

static bool gen8_gtt_test_present(gtt_entry_t *e)
{
	/*
	 * i915 writes PDP root pointer registers without present bit,
	 * it also works, so we need to treat root pointer entry
	 * specifically.
	 */
	if (e->type == GTT_TYPE_PPGTT_ROOT_L3_ENTRY
			|| e->type == GTT_TYPE_PPGTT_ROOT_L4_ENTRY)
		return (e->val64 != 0);
	else
		return (e->val32[0] & _REGBIT_PTE_VALID);
}

static void gtt_entry_clear_present(gtt_entry_t *e)
{
	e->val32[0] &= ~_REGBIT_PTE_VALID;
}

/*
 * Per-platform GMA routines.
 */
static unsigned long gma_to_ggtt_pte_index(unsigned long gma)
{
	unsigned long x = (gma >> GTT_PAGE_SHIFT);
	trace_gma_index(__func__, gma, x);
	return x;
}

#define DEFINE_PPGTT_GMA_TO_INDEX(prefix, ename, exp) \
	static unsigned long prefix##_gma_to_##ename##_index(unsigned long gma) { \
		unsigned long x = (exp); \
		trace_gma_index(__func__, gma, x); \
		return x; \
	}

DEFINE_PPGTT_GMA_TO_INDEX(gen7, pte, (gma >> 12 & 0x3ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen7, pde, (gma >> 22 & 0x1ff));

DEFINE_PPGTT_GMA_TO_INDEX(gen8, pte, (gma >> 12 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, pde, (gma >> 21 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, l3_pdp, (gma >> 30 & 0x3));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, l4_pdp, (gma >> 30 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, pml4, (gma >> 39 & 0x1ff));

struct vgt_gtt_pte_ops gen7_gtt_pte_ops = {
	.get_entry = gtt_get_entry32,
	.set_entry = gtt_set_entry32,
	.clear_present = gtt_entry_clear_present,
	.test_present = gen7_gtt_test_present,
	.test_pse = gen7_gtt_test_pse,
	.get_pfn = gen7_gtt_get_pfn,
	.set_pfn = gen7_gtt_set_pfn,
};

struct vgt_gtt_gma_ops gen7_gtt_gma_ops = {
	.gma_to_ggtt_pte_index = gma_to_ggtt_pte_index,
	.gma_to_pte_index = gen7_gma_to_pte_index,
	.gma_to_pde_index = gen7_gma_to_pde_index,
};

struct vgt_gtt_pte_ops gen8_gtt_pte_ops = {
	.get_entry = gtt_get_entry64,
	.set_entry = gtt_set_entry64,
	.clear_present = gtt_entry_clear_present,
	.test_present = gen8_gtt_test_present,
	.test_pse = gen8_gtt_test_pse,
	.get_pfn = gen8_gtt_get_pfn,
	.set_pfn = gen8_gtt_set_pfn,
};

struct vgt_gtt_gma_ops gen8_gtt_gma_ops = {
	.gma_to_ggtt_pte_index = gma_to_ggtt_pte_index,
	.gma_to_pte_index = gen8_gma_to_pte_index,
	.gma_to_pde_index = gen8_gma_to_pde_index,
	.gma_to_l3_pdp_index = gen8_gma_to_l3_pdp_index,
	.gma_to_l4_pdp_index = gen8_gma_to_l4_pdp_index,
	.gma_to_pml4_index = gen8_gma_to_pml4_index,
};

static bool gtt_entry_p2m(struct vgt_device *vgt, gtt_entry_t *p, gtt_entry_t *m)
{
        struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
        unsigned long gfn, mfn;

        *m = *p;

        if (!ops->test_present(p))
                return true;

        gfn = ops->get_pfn(p);

        mfn = hypervisor_g2m_pfn(vgt, gfn);
        if (mfn == INVALID_MFN) {
                vgt_err("fail to translate gfn: 0x%lx\n", gfn);
                return false;
        }

        ops->set_pfn(m, mfn);

        return true;
}

/*
 * MM helpers.
 */
gtt_entry_t *vgt_mm_get_entry(struct vgt_mm *mm,
		void *page_table, gtt_entry_t *e,
		unsigned long index)
{
	struct pgt_device *pdev = mm->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;
	e->type = mm->page_table_entry_type;

	/*
	 * If this read goes to HW, we translate
	 * the relative index to absolute index
	 * for pre-bdw platform.
	 */
	if (IS_PREBDW(pdev)) {
		if (mm->type == VGT_MM_PPGTT && !page_table)
			index += mm->pde_base_index;
	}

	ops->get_entry(page_table, e, index, false, mm->vgt);
	ops->test_pse(e);

	return e;
}

gtt_entry_t *vgt_mm_set_entry(struct vgt_mm *mm,
		void *page_table, gtt_entry_t *e,
		unsigned long index)
{
	struct pgt_device *pdev = mm->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;

	/*
	 * If this write goes to HW, we translate
	 * the relative index to absolute index
	 * for pre-bdw platform.
	 */
	if (IS_PREBDW(pdev)) {
		if (mm->type == VGT_MM_PPGTT && !page_table)
			index += mm->pde_base_index;
	}

	return ops->set_entry(page_table, e, index, false, mm->vgt);
}

/*
 * PPGTT shadow page table helpers.
 */
static inline gtt_entry_t *ppgtt_spt_get_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index,
		bool guest)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;
	e->type = get_entry_type(type);

	ASSERT(gtt_type_is_entry(e->type));

	ops->get_entry(page_table, e, index, guest, spt->vgt);
	ops->test_pse(e);

	return e;
}

static inline gtt_entry_t *ppgtt_spt_set_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index,
		bool guest)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;

	ASSERT(gtt_type_is_entry(e->type));

	return ops->set_entry(page_table, e, index, guest, spt->vgt);
}

#define ppgtt_get_guest_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index, true)

#define ppgtt_set_guest_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index, true)

#define ppgtt_get_shadow_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index, false)

#define ppgtt_set_shadow_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index, false)


bool vgt_init_guest_page(struct vgt_device *vgt, guest_page_t *guest_page,
		unsigned long gfn, guest_page_handler_t handler, void *data)
{
	INIT_HLIST_NODE(&guest_page->node);

	guest_page->vaddr = hypervisor_gpa_to_va(vgt, gfn << GTT_PAGE_SHIFT);
	if (!guest_page->vaddr)
		return false;

	guest_page->writeprotection = false;
	guest_page->gfn = gfn;
	guest_page->handler = handler;
	guest_page->data = data;
	guest_page->oos_page = NULL;
	guest_page->write_cnt = 0;

	hash_add(vgt->gtt.guest_page_hash_table, &guest_page->node, guest_page->gfn);

	return true;
}

static bool vgt_detach_oos_page(struct vgt_device *vgt, oos_page_t *oos_page);

void vgt_clean_guest_page(struct vgt_device *vgt, guest_page_t *guest_page)
{
	if(!hlist_unhashed(&guest_page->node))
		hash_del(&guest_page->node);

	if (guest_page->oos_page)
		vgt_detach_oos_page(vgt, guest_page->oos_page);

	if (guest_page->writeprotection)
		hypervisor_unset_wp_pages(vgt, guest_page);

	if (guest_page == vgt->gtt.last_partial_ppgtt_access_gpt)
		vgt->gtt.last_partial_ppgtt_access_index = -1;
}

guest_page_t *vgt_find_guest_page(struct vgt_device *vgt, unsigned long gfn)
{
	guest_page_t *guest_page;
	struct vgt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	t0 = get_cycles();

	hash_for_each_possible(vgt->gtt.guest_page_hash_table, guest_page, node, gfn) {
		if (guest_page->gfn == gfn) {
			t1 = get_cycles();
			stat->gpt_find_hit_cnt++;
			stat->gpt_find_hit_cycles += t1 - t0;
			return guest_page;
		}
	}

	t1 = get_cycles();
	stat->gpt_find_miss_cnt++;
	stat->gpt_find_miss_cycles += t1 - t0;

	return NULL;
}

/*
 * Shadow page manipulation routines.
 */
static inline bool vgt_init_shadow_page(struct vgt_device *vgt,
		shadow_page_t *sp, gtt_type_t type)
{
	sp->vaddr = page_address(sp->page);
	sp->type = type;
	memset(sp->vaddr, 0, PAGE_SIZE);

	INIT_HLIST_NODE(&sp->node);
	sp->mfn = hypervisor_virt_to_mfn(sp->vaddr);
	hash_add(vgt->gtt.shadow_page_hash_table, &sp->node, sp->mfn);

	return true;
}

static inline void vgt_clean_shadow_page(shadow_page_t *sp)
{
	if(!hlist_unhashed(&sp->node))
		hash_del(&sp->node);
}

static inline shadow_page_t *vgt_find_shadow_page(struct vgt_device *vgt,
		unsigned long mfn)
{
	shadow_page_t *shadow_page;
	struct vgt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	t0 = get_cycles();

	hash_for_each_possible(vgt->gtt.shadow_page_hash_table, shadow_page, node, mfn) {
		if (shadow_page->mfn == mfn) {
			t1 = get_cycles();
			stat->spt_find_hit_cnt++;
			stat->spt_find_hit_cycles += t1 - t0;
			return shadow_page;
		}
	}

	t1 = get_cycles();
	stat->spt_find_miss_cnt++;
	stat->spt_find_miss_cycles += t1 - t0;

	return NULL;
}

#define guest_page_to_ppgtt_spt(ptr) \
	container_of(ptr, ppgtt_spt_t, guest_page)

#define shadow_page_to_ppgtt_spt(ptr) \
	container_of(ptr, ppgtt_spt_t, shadow_page)

static void ppgtt_free_shadow_page(ppgtt_spt_t *spt)
{
	trace_spt_free(spt->vgt->vm_id, spt, spt->shadow_page.type);

	vgt_clean_shadow_page(&spt->shadow_page);
	vgt_clean_guest_page(spt->vgt, &spt->guest_page);

	mempool_free(spt, spt->vgt->pdev->gtt.mempool);
}

static void ppgtt_free_all_shadow_page(struct vgt_device *vgt)
{
	struct hlist_node *n;
	shadow_page_t *sp;
	int i;

	hash_for_each_safe(vgt->gtt.shadow_page_hash_table, i, n, sp, node)
		ppgtt_free_shadow_page(shadow_page_to_ppgtt_spt(sp));

	return;
}

static bool ppgtt_handle_guest_write_page_table_bytes(void *gp,
		uint64_t pa, void *p_data, int bytes);

static bool ppgtt_write_protection_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *gpt = (guest_page_t *)gp;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct vgt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	if (bytes != 4 && bytes != 8)
		return false;

	if (!gpt->writeprotection)
		return false;

	t0 = get_cycles();

	if (!ppgtt_handle_guest_write_page_table_bytes(gp,
		pa, p_data, bytes))
		return false;

	t1 = get_cycles();
	stat->ppgtt_wp_cnt++;
	stat->ppgtt_wp_cycles += t1 - t0;

	return true;
}

static ppgtt_spt_t *ppgtt_alloc_shadow_page(struct vgt_device *vgt,
		gtt_type_t type, unsigned long gpt_gfn)
{
	ppgtt_spt_t *spt = NULL;

	spt = mempool_alloc(vgt->pdev->gtt.mempool, GFP_ATOMIC);
	if (!spt) {
		vgt_err("fail to allocate ppgtt shadow page.\n");
		return NULL;
	}

	spt->vgt = vgt;
	spt->guest_page_type = type;
	atomic_set(&spt->refcount, 1);

	/*
	 * TODO: Guest page may be different with shadow page type,
	 *	 if we support PSE page in future.
	 */
	if (!vgt_init_shadow_page(vgt, &spt->shadow_page, type)) {
		vgt_err("fail to initialize shadow_page_t for spt.\n");
		goto err;
	}

	if (!vgt_init_guest_page(vgt, &spt->guest_page,
				gpt_gfn, ppgtt_write_protection_handler, NULL)) {
		vgt_err("fail to initialize shadow_page_t for spt.\n");
		goto err;
	}

	trace_spt_alloc(vgt->vm_id, spt, type, spt->shadow_page.mfn, gpt_gfn);

	return spt;
err:
	ppgtt_free_shadow_page(spt);
	return NULL;
}

static ppgtt_spt_t *ppgtt_find_shadow_page(struct vgt_device *vgt, unsigned long mfn)
{
	shadow_page_t *sp = vgt_find_shadow_page(vgt, mfn);

	if (sp)
		return shadow_page_to_ppgtt_spt(sp);

	vgt_err("VM %d fail to find ppgtt shadow page: 0x%lx.\n",
			vgt->vm_id, mfn);

	return NULL;
}

#define pt_entry_size_shift(spt) \
	((spt)->vgt->pdev->device_info.gtt_entry_size_shift)

#define pt_entries(spt) \
	(PAGE_SIZE >> pt_entry_size_shift(spt))

#define for_each_present_guest_entry(spt, e, i) \
	for (i = 0; i < pt_entries(spt); i++) \
	if (spt->vgt->pdev->gtt.pte_ops->test_present(ppgtt_get_guest_entry(spt, e, i)))

#define for_each_present_shadow_entry(spt, e, i) \
	for (i = 0; i < pt_entries(spt); i++) \
	if (spt->vgt->pdev->gtt.pte_ops->test_present(ppgtt_get_shadow_entry(spt, e, i)))

static void ppgtt_get_shadow_page(ppgtt_spt_t *spt)
{
	int v = atomic_read(&spt->refcount);

	trace_spt_refcount(spt->vgt->vm_id, "inc", spt, v, (v + 1));

	atomic_inc(&spt->refcount);
}

static bool ppgtt_invalidate_shadow_page(ppgtt_spt_t *spt);

static bool ppgtt_invalidate_shadow_page_by_shadow_entry(struct vgt_device *vgt,
		gtt_entry_t *e)
{
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s;

	if (!gtt_type_is_pt(get_next_pt_type(e->type)))
		return false;

	s = ppgtt_find_shadow_page(vgt, ops->get_pfn(e));
	if (!s) {
		vgt_err("VM %d fail to find shadow page: mfn: 0x%lx.\n",
				vgt->vm_id, ops->get_pfn(e));
		return false;
	}

	return ppgtt_invalidate_shadow_page(s);
}

static bool ppgtt_invalidate_shadow_page(ppgtt_spt_t *spt)
{
	gtt_entry_t e;
	unsigned long index;
	int v = atomic_read(&spt->refcount);

	trace_spt_change(spt->vgt->vm_id, "die", spt,
			spt->guest_page.gfn, spt->shadow_page.type);

	trace_spt_refcount(spt->vgt->vm_id, "dec", spt, v, (v - 1));

	if (atomic_dec_return(&spt->refcount) > 0)
		return true;

	if (gtt_type_is_pte_pt(spt->shadow_page.type))
		goto release;

	for_each_present_shadow_entry(spt, &e, index) {
		if (!gtt_type_is_pt(get_next_pt_type(e.type))) {
			vgt_err("VGT doesn't support pse bit now.\n");
			return false;
		}
		if (!ppgtt_invalidate_shadow_page_by_shadow_entry(spt->vgt, &e))
			goto fail;
	}

release:
	trace_spt_change(spt->vgt->vm_id, "release", spt,
			spt->guest_page.gfn, spt->shadow_page.type);
	ppgtt_free_shadow_page(spt);
	return true;
fail:
	vgt_err("fail: shadow page %p shadow entry 0x%llx type %d.\n",
			spt, e.val64, e.type);
	return false;
}

static bool ppgtt_populate_shadow_page(ppgtt_spt_t *spt);

static ppgtt_spt_t *ppgtt_populate_shadow_page_by_guest_entry(struct vgt_device *vgt,
		gtt_entry_t *we)
{
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s = NULL;
	guest_page_t *g;

	if (!gtt_type_is_pt(get_next_pt_type(we->type)))
		goto fail;

	g = vgt_find_guest_page(vgt, ops->get_pfn(we));
	if (g) {
		s = guest_page_to_ppgtt_spt(g);
		ppgtt_get_shadow_page(s);
	} else {
		gtt_type_t type = get_next_pt_type(we->type);
		s = ppgtt_alloc_shadow_page(vgt, type, ops->get_pfn(we));
		if (!s)
			goto fail;

		if (!hypervisor_set_wp_pages(vgt, &s->guest_page))
			goto fail;

		if (!ppgtt_populate_shadow_page(s))
			goto fail;

		trace_spt_change(vgt->vm_id, "new", s, s->guest_page.gfn, s->shadow_page.type);
	}
	return s;
fail:
	vgt_err("fail: shadow page %p guest entry 0x%llx type %d.\n",
			s, we->val64, we->type);
	return NULL;
}

static inline void ppgtt_generate_shadow_entry(gtt_entry_t *se,
		ppgtt_spt_t *s, gtt_entry_t *ge)
{
	struct vgt_gtt_pte_ops *ops = s->vgt->pdev->gtt.pte_ops;

	se->type = ge->type;
	se->val64 = ge->val64;
	se->pdev = ge->pdev;

	ops->set_pfn(se, s->shadow_page.mfn);
}

static bool ppgtt_populate_shadow_page(ppgtt_spt_t *spt)
{
	struct vgt_device *vgt = spt->vgt;
	ppgtt_spt_t *s;
	gtt_entry_t se, ge;
	unsigned long i;

	trace_spt_change(spt->vgt->vm_id, "born", spt,
			spt->guest_page.gfn, spt->shadow_page.type);

	if (gtt_type_is_pte_pt(spt->shadow_page.type)) {
		for_each_present_guest_entry(spt, &ge, i) {
			if (!gtt_entry_p2m(vgt, &ge, &se))
				goto fail;
			ppgtt_set_shadow_entry(spt, &se, i);
		}
		return true;
	}

	for_each_present_guest_entry(spt, &ge, i) {
		if (!gtt_type_is_pt(get_next_pt_type(ge.type))) {
			vgt_err("VGT doesn't support pse bit now.\n");
			goto fail;
		}

		s = ppgtt_populate_shadow_page_by_guest_entry(vgt, &ge);
		if (!s)
			goto fail;
		ppgtt_get_shadow_entry(spt, &se, i);
		ppgtt_generate_shadow_entry(&se, s, &ge);
		ppgtt_set_shadow_entry(spt, &se, i);
	}
	return true;
fail:
	vgt_err("fail: shadow page %p guest entry 0x%llx type %d.\n",
			spt, ge.val64, ge.type);
	return false;
}

static bool ppgtt_handle_guest_entry_removal(guest_page_t *gpt,
		gtt_entry_t *we, unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	shadow_page_t *sp = &spt->shadow_page;
	struct vgt_device *vgt = spt->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	gtt_entry_t e;

	trace_gpt_change(spt->vgt->vm_id, "remove", spt, sp->type, we->val64, index);

	ppgtt_get_shadow_entry(spt, &e, index);
	if (!ops->test_present(&e))
		return true;

	if (gtt_type_is_pt(get_next_pt_type(we->type))) {
		guest_page_t *g = vgt_find_guest_page(vgt, ops->get_pfn(we));
		if (!g) {
			vgt_err("fail to find guest page.\n");
			goto fail;
		}
		if (!ppgtt_invalidate_shadow_page(guest_page_to_ppgtt_spt(g)))
			goto fail;
	}
	e.val64 = 0;
	ppgtt_set_shadow_entry(spt, &e, index);
	return true;
fail:
	vgt_err("fail: shadow page %p guest entry 0x%llx type %d.\n",
			spt, we->val64, we->type);
	return false;
}

static bool ppgtt_handle_guest_entry_add(guest_page_t *gpt,
		gtt_entry_t *we, unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	shadow_page_t *sp = &spt->shadow_page;
	struct vgt_device *vgt = spt->vgt;
	gtt_entry_t m;
	ppgtt_spt_t *s;

	trace_gpt_change(spt->vgt->vm_id, "add", spt, sp->type, we->val64, index);

	if (gtt_type_is_pt(get_next_pt_type(we->type))) {
		s = ppgtt_populate_shadow_page_by_guest_entry(vgt, we);
		if (!s)
			goto fail;
		ppgtt_get_shadow_entry(spt, &m, index);
		ppgtt_generate_shadow_entry(&m, s, we);
		ppgtt_set_shadow_entry(spt, &m, index);
	} else {
		if (!gtt_entry_p2m(vgt, we, &m))
			goto fail;
		ppgtt_set_shadow_entry(spt, &m, index);
	}

	return true;

fail:
	vgt_err("fail: spt %p guest entry 0x%llx type %d.\n", spt, we->val64, we->type);
	return false;
}

static bool vgt_sync_oos_page(struct vgt_device *vgt, oos_page_t *oos_page)
{
	struct vgt_device_info *info = &vgt->pdev->device_info;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(oos_page->guest_page);
	gtt_entry_t old, new, m;
	int index;

	trace_oos_change(vgt->vm_id, "sync", oos_page->id,
			oos_page->guest_page, spt->guest_page_type);

	old.type = new.type = get_entry_type(spt->guest_page_type);
	old.pdev = new.pdev = pdev;
	old.val64 = new.val64 = 0;

	for (index = 0; index < (GTT_PAGE_SIZE >> info->gtt_entry_size_shift); index++) {
		ops->get_entry(oos_page->mem, &old, index, false, vgt);
		ops->get_entry(oos_page->guest_page->vaddr, &new, index, true, vgt);

		if (old.val64 == new.val64)
			continue;

		trace_oos_sync(vgt->vm_id, oos_page->id,
				oos_page->guest_page, spt->guest_page_type,
				new.val64, index);

		if (!gtt_entry_p2m(vgt, &new, &m))
			return false;

		ops->set_entry(oos_page->mem, &new, index, false, vgt);
		ppgtt_set_shadow_entry(spt, &m, index);
	}

	oos_page->guest_page->write_cnt = 0;

	return true;
}

static bool vgt_detach_oos_page(struct vgt_device *vgt, oos_page_t *oos_page)
{
	struct pgt_device *pdev = vgt->pdev;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(oos_page->guest_page);

	trace_oos_change(vgt->vm_id, "detach", oos_page->id,
			oos_page->guest_page, spt->guest_page_type);

	oos_page->guest_page->write_cnt = 0;
	oos_page->guest_page->oos_page = NULL;
	oos_page->guest_page = NULL;

	list_del_init(&oos_page->vm_list);
	list_move_tail(&oos_page->list, &pdev->gtt.oos_page_free_list_head);

	pdev->stat.oos_page_cur_avail_cnt++;
	pdev->stat.oos_page_detach_cnt++;

	return true;
}

static oos_page_t *vgt_attach_oos_page(struct vgt_device *vgt,
		oos_page_t *oos_page, guest_page_t *gpt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (!hypervisor_read_va(vgt, gpt->vaddr, oos_page->mem, GTT_PAGE_SIZE, 1))
		return NULL;

	oos_page->guest_page = gpt;
	gpt->oos_page = oos_page;

	list_move_tail(&oos_page->list, &pdev->gtt.oos_page_use_list_head);

	if (--pdev->stat.oos_page_cur_avail_cnt < pdev->stat.oos_page_min_avail_cnt)
		pdev->stat.oos_page_min_avail_cnt = pdev->stat.oos_page_cur_avail_cnt;

	trace_oos_change(vgt->vm_id, "attach", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	pdev->stat.oos_page_attach_cnt++;

	return oos_page;
}

static bool ppgtt_set_guest_page_sync(struct vgt_device *vgt, guest_page_t *gpt)
{
	if (!hypervisor_set_wp_pages(vgt, gpt))
		return false;

	trace_oos_change(vgt->vm_id, "set page sync", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	list_del_init(&gpt->oos_page->vm_list);
	return vgt_sync_oos_page(vgt, gpt->oos_page);
}

static bool ppgtt_allocate_oos_page(struct vgt_device *vgt, guest_page_t *gpt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_info *gtt = &pdev->gtt;
	oos_page_t *oos_page = gpt->oos_page;

	/* oos_page should be NULL at this point */
	ASSERT(!oos_page);

	if (list_empty(&gtt->oos_page_free_list_head)) {
		oos_page = container_of(gtt->oos_page_use_list_head.next, oos_page_t, list);
		if (!ppgtt_set_guest_page_sync(vgt, oos_page->guest_page)
			|| !vgt_detach_oos_page(vgt, oos_page))
			return false;
		ASSERT(!list_empty(&gtt->oos_page_free_list_head));
		pdev->stat.oos_page_steal_cnt++;
	} else
		oos_page = container_of(gtt->oos_page_free_list_head.next, oos_page_t, list);

	return vgt_attach_oos_page(vgt, oos_page, gpt);
}

static bool ppgtt_set_guest_page_oos(struct vgt_device *vgt, guest_page_t *gpt)
{
	oos_page_t *oos_page = gpt->oos_page;

	ASSERT(oos_page);

	trace_oos_change(vgt->vm_id, "set page out of sync", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	list_add_tail(&oos_page->vm_list, &vgt->gtt.oos_page_list_head);
	return hypervisor_unset_wp_pages(vgt, gpt);
}

bool ppgtt_sync_oos_pages(struct vgt_device *vgt)
{
	struct list_head *pos, *n;
	oos_page_t *oos_page;

	if (!spt_out_of_sync)
		return true;

	list_for_each_safe(pos, n, &vgt->gtt.oos_page_list_head) {
		oos_page = container_of(pos, oos_page_t, vm_list);
		if (!ppgtt_set_guest_page_sync(vgt, oos_page->guest_page))
			return false;
	}

	return true;
}

/*
 * The heart of PPGTT shadow page table.
 */
static bool ppgtt_handle_guest_write_page_table(guest_page_t *gpt, gtt_entry_t *we,
		unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	gtt_entry_t ge;

	int old_present, new_present;

	ppgtt_get_guest_entry(spt, &ge, index);

	old_present = ops->test_present(&ge);
	new_present = ops->test_present(we);

	ppgtt_set_guest_entry(spt, we, index);

	if (old_present && new_present) {
		if (!ppgtt_handle_guest_entry_removal(gpt, &ge, index)
		|| !ppgtt_handle_guest_entry_add(gpt, we, index))
			goto fail;
	} else if (!old_present && new_present) {
		if (!ppgtt_handle_guest_entry_add(gpt, we, index))
			goto fail;
	} else if (old_present && !new_present) {
		if (!ppgtt_handle_guest_entry_removal(gpt, &ge, index))
			goto fail;
	}
	return true;
fail:
	vgt_err("fail: shadow page %p guest entry 0x%llx type %d.\n",
			spt, we->val64, we->type);
	return false;
}

static inline bool can_do_out_of_sync(guest_page_t *gpt)
{
	return spt_out_of_sync
		&& gtt_type_is_pte_pt(guest_page_to_ppgtt_spt(gpt)->guest_page_type)
		&& gpt->write_cnt >= 2;
}

bool ppgtt_check_partial_access(struct vgt_device *vgt)
{
	struct vgt_vgtt_info *gtt = &vgt->gtt;

	if (gtt->last_partial_ppgtt_access_index == -1)
		return true;

	if (!gtt->warn_partial_ppgtt_access_once) {
		vgt_warn("Incomplete PPGTT page table access sequence.\n");
		gtt->warn_partial_ppgtt_access_once = true;
	}

	if (!ppgtt_handle_guest_write_page_table(
			gtt->last_partial_ppgtt_access_gpt,
			&gtt->last_partial_ppgtt_access_entry,
			gtt->last_partial_ppgtt_access_index))
		return false;

	gtt->last_partial_ppgtt_access_index = -1;
	return true;
}

static bool ppgtt_handle_guest_write_page_table_bytes(void *gp,
		uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *gpt = (guest_page_t *)gp;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	struct vgt_device_info *info = &vgt->pdev->device_info;
	struct vgt_vgtt_info *gtt = &vgt->gtt;
	gtt_entry_t we, se;
	unsigned long index;

	bool partial_access = (bytes != info->gtt_entry_size);
	bool hi = (partial_access && (pa & (info->gtt_entry_size - 1)));

	index = (pa & (PAGE_SIZE - 1)) >> info->gtt_entry_size_shift;

	ppgtt_get_guest_entry(spt, &we, index);
	memcpy(&we.val64 + (pa & (info->gtt_entry_size - 1)), p_data, bytes);

	if (partial_access && !hi) {
		trace_gpt_change(vgt->vm_id, "partial access - LOW",
				NULL, we.type, *(u32 *)(p_data), index);

		ppgtt_check_partial_access(vgt);

		ppgtt_set_guest_entry(spt, &we, index);
		ppgtt_get_shadow_entry(spt, &se, index);

		if (!ops->test_present(&se))
			return true;

		if (gtt_type_is_pt(get_next_pt_type(se.type)))
			if (!ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &se))
				return false;

		se.val64 = 0;
		ppgtt_set_shadow_entry(spt, &se, index);

		gtt->last_partial_ppgtt_access_index = index;
		gtt->last_partial_ppgtt_access_gpt = gpt;
		gtt->last_partial_ppgtt_access_entry = we;

		return true;
	} else
		gtt->last_partial_ppgtt_access_index = -1;

	if (hi)
		trace_gpt_change(vgt->vm_id, "partial access - HIGH",
				NULL, we.type, *(u32 *)(p_data), index);

	ops->test_pse(&we);

	gpt->write_cnt++;

	if (!ppgtt_handle_guest_write_page_table(gpt, &we, index))
		return false;

	if (spt_out_of_sync) {
		if (gpt->oos_page) {
			/* 1. only GTT_PTE type has oos_page assocaited
			 * 2. update oos_page according to wp guest page change
			 */
			ops->set_entry(gpt->oos_page->mem, &we, index, false, vgt);

		}

		if (can_do_out_of_sync(gpt)) {
			if (!gpt->oos_page)
				ppgtt_allocate_oos_page(vgt, gpt);

			if (!ppgtt_set_guest_page_oos(vgt, gpt)) {
				/* should not return false since we can handle it*/
				ppgtt_set_guest_page_sync(vgt, gpt);
				vgt_info("Mochi debug mark 5.\n");

			}
		}

	}

	return true;
}

bool ppgtt_handle_guest_write_root_pointer(struct vgt_mm *mm,
		gtt_entry_t *we, unsigned long index)
{
	struct vgt_device *vgt = mm->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *spt = NULL;
	gtt_entry_t e;

	if (mm->type != VGT_MM_PPGTT || !mm->shadowed)
		return false;

	trace_gpt_change(vgt->vm_id, __func__, NULL,
			we->type, we->val64, index);

	ppgtt_get_guest_root_entry(mm, &e, index);

	if (ops->test_present(&e)) {
		ppgtt_get_shadow_root_entry(mm, &e, index);

		trace_gpt_change(vgt->vm_id, "destroy old root pointer",
				spt, e.type, e.val64, index);

		if (gtt_type_is_pt(get_next_pt_type(e.type))) {
			if (!ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &e))
				goto fail;
		} else {
			vgt_err("VGT doesn't support pse bit now.\n");
			goto fail;
		}
		e.val64 = 0;
		ppgtt_set_shadow_root_entry(mm, &e, index);
	}

	if (ops->test_present(we)) {
		if (gtt_type_is_pt(get_next_pt_type(we->type))) {
			spt = ppgtt_populate_shadow_page_by_guest_entry(vgt, we);
			if (!spt) {
				vgt_err("fail to populate root pointer.\n");
				goto fail;
			}
			ppgtt_generate_shadow_entry(&e, spt, we);
			ppgtt_set_shadow_root_entry(mm, &e, index);
		} else {
			vgt_err("VGT doesn't support pse bit now.\n");
			goto fail;
		}
		trace_gpt_change(vgt->vm_id, "populate root pointer",
				spt, e.type, e.val64, index);
	}
	return true;
fail:
	vgt_err("fail: shadow page %p guest entry 0x%llx type %d.\n",
			spt, we->val64, we->type);
	return false;
}

/*
 * mm page table allocation policy for pre-bdw:
 *  - for ggtt, a virtual page table will be allocated.
 *  - for ppgtt, the virtual page table(root entry) will use a part of
 *	virtual page table from ggtt.
 */
bool gen7_mm_alloc_page_table(struct vgt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_vgtt_info *gtt = &vgt->gtt;
	struct vgt_device_info *info = &pdev->device_info;
	void *mem;

	if (mm->type == VGT_MM_PPGTT) {
		struct vgt_mm *ggtt_mm = gtt->ggtt_mm;
		if (!ggtt_mm) {
			vgt_err("ggtt mm hasn't been created.\n");
			return false;
		}
		mm->page_table_entry_cnt = 512;
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mm->virtual_page_table = ggtt_mm->virtual_page_table +
			(mm->pde_base_index << info->gtt_entry_size_shift);
		/* shadow page table resides in the hw mmio entries. */
	} else if (mm->type == VGT_MM_GGTT) {
		mm->page_table_entry_cnt = (gm_sz(pdev) >> GTT_PAGE_SHIFT);
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = vzalloc(mm->page_table_entry_size);
		if (!mem) {
			vgt_err("fail to allocate memory.\n");
			return false;
		}
		mm->virtual_page_table = mem;
		/* Mochi: alloc shadow GTT. */
		mem = vzalloc(mm->page_table_entry_size);
		if (!mem) {
			vgt_err("fail to allocate memory.\n");
			return false;
		}
		mm->shadow_gtt = mem;
	}
	return true;
}

void gen7_mm_free_page_table(struct vgt_mm *mm)
{
	if (mm->type == VGT_MM_GGTT) {
		if (mm->virtual_page_table)
			vfree(mm->virtual_page_table);
	}
	mm->virtual_page_table = mm->shadow_page_table = NULL;
}

/*
 * mm page table allocation policy for bdw+
 *  - for ggtt, only virtual page table will be allocated.
 *  - for ppgtt, dedicated virtual/shadow page table will be allocated.
 */
bool gen8_mm_alloc_page_table(struct vgt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_device_info *info = &pdev->device_info;
	void *mem;

	if (mm->type == VGT_MM_PPGTT) {
		mm->page_table_entry_cnt = 4;
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = kzalloc(mm->has_shadow_page_table ?
			mm->page_table_entry_size * 2 : mm->page_table_entry_size,
			GFP_ATOMIC);
		if (!mem) {
			vgt_err("fail to allocate memory.\n");
			return false;
		}
		mm->virtual_page_table = mem;
		if (!mm->has_shadow_page_table)
			return true;
		mm->shadow_page_table = mem + mm->page_table_entry_size;
	} else if (mm->type == VGT_MM_GGTT) {
		mm->page_table_entry_cnt = (gm_sz(pdev) >> GTT_PAGE_SHIFT);
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = vzalloc(mm->page_table_entry_size);
		if (!mem) {
			vgt_err("fail to allocate memory.\n");
			return false;
		}
		mm->virtual_page_table = mem;
	}
	return true;
}

void gen8_mm_free_page_table(struct vgt_mm *mm)
{
	if (mm->type == VGT_MM_PPGTT) {
		if (mm->virtual_page_table)
			kfree(mm->virtual_page_table);
	} else if (mm->type == VGT_MM_GGTT) {
		if (mm->virtual_page_table)
			vfree(mm->virtual_page_table);
	}
	mm->virtual_page_table = mm->shadow_page_table = NULL;
}

void vgt_destroy_mm(struct vgt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_info *gtt = &pdev->gtt;
	struct vgt_gtt_pte_ops *ops = gtt->pte_ops;
	gtt_entry_t se;
	int i;

	if (!mm->initialized)
		goto out;

	if (atomic_dec_return(&mm->refcount) > 0)
		return;

	list_del(&mm->list);

	if (mm->has_shadow_page_table && mm->shadowed) {
		for (i = 0; i < mm->page_table_entry_cnt; i++) {
			ppgtt_get_shadow_root_entry(mm, &se, i);
			if (!ops->test_present(&se))
				continue;
			ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &se);
			se.val64 = 0;
			ppgtt_set_shadow_root_entry(mm, &se, i);

			trace_gpt_change(vgt->vm_id, "destroy root pointer",
					NULL, se.type, se.val64, i);
		}
	}
	gtt->mm_free_page_table(mm);
out:
	kfree(mm);
}

struct vgt_mm *vgt_create_mm(struct vgt_device *vgt,
		vgt_mm_type_t mm_type, gtt_type_t page_table_entry_type,
		void *virtual_page_table, int page_table_level,
		u32 pde_base_index)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_info *gtt = &pdev->gtt;
	struct vgt_gtt_pte_ops *ops = gtt->pte_ops;
	struct vgt_mm *mm;
	ppgtt_spt_t *spt;
	gtt_entry_t ge, se;
	int i;

	mm = kzalloc(sizeof(*mm), GFP_ATOMIC);
	if (!mm) {
		vgt_err("fail to allocate memory for mm.\n");
		goto fail;
	}

	mm->type = mm_type;
	mm->page_table_entry_type = page_table_entry_type;
	mm->page_table_level = page_table_level;
	mm->pde_base_index = pde_base_index;

	mm->vgt = vgt;
	mm->has_shadow_page_table = (vgt->vm_id != 0 && mm_type == VGT_MM_PPGTT);

	atomic_set(&mm->refcount, 1);
	INIT_LIST_HEAD(&mm->list);
	list_add_tail(&mm->list, &vgt->gtt.mm_list_head);

	if (!gtt->mm_alloc_page_table(mm)) {
		vgt_err("fail to allocate page table for mm.\n");
		goto fail;
	}

	mm->initialized = true;

	if (virtual_page_table)
		memcpy(mm->virtual_page_table, virtual_page_table,
				mm->page_table_entry_size);

	if (mm->has_shadow_page_table) {
		for (i = 0; i < mm->page_table_entry_cnt; i++) {
			ppgtt_get_guest_root_entry(mm, &ge, i);
			if (!ops->test_present(&ge))
				continue;

			trace_gpt_change(vgt->vm_id, __func__, NULL,
					ge.type, ge.val64, i);

			spt = ppgtt_populate_shadow_page_by_guest_entry(vgt, &ge);
			if (!spt) {
				vgt_err("fail to populate guest root pointer.\n");
				goto fail;
			}
			ppgtt_generate_shadow_entry(&se, spt, &ge);
			ppgtt_set_shadow_root_entry(mm, &se, i);

			trace_gpt_change(vgt->vm_id, "populate root pointer",
					NULL, se.type, se.val64, i);
		}
		mm->shadowed = true;
	}
	return mm;
fail:
	vgt_err("fail to create mm.\n");
	if (mm)
		vgt_destroy_mm(mm);
	return NULL;
}

/*
 * GMA translation APIs.
 */
static inline bool ppgtt_get_next_level_entry(struct vgt_mm *mm,
		gtt_entry_t *e, unsigned long index, bool guest)
{
	struct vgt_device *vgt = mm->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s;
	void *pt;

	if (mm->has_shadow_page_table) {
		if (!(s = ppgtt_find_shadow_page(vgt, ops->get_pfn(e))))
			return false;
		if (!guest)
			ppgtt_get_shadow_entry(s, e, index);
		else
			ppgtt_get_guest_entry(s, e, index);
	} else {
		pt = hypervisor_mfn_to_virt(ops->get_pfn(e));
		ops->get_entry(pt, e, index, false, mm->vgt);
		e->type = get_entry_type(get_next_pt_type(e->type));
	}
	return true;
}

static inline unsigned long vgt_gma_to_gpa(struct vgt_mm *mm, unsigned long gma)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_pte_ops *pte_ops = pdev->gtt.pte_ops;
	struct vgt_gtt_gma_ops *gma_ops = pdev->gtt.gma_ops;

	unsigned long gpa = INVALID_ADDR;
	unsigned long gma_index[4];
	gtt_entry_t e;
	int i, index;

	if (mm->type != VGT_MM_GGTT && mm->type != VGT_MM_PPGTT)
		return INVALID_ADDR;

	if (mm->type == VGT_MM_GGTT) {
		if (!g_gm_is_valid(vgt, gma))
			goto err;

		ggtt_get_guest_entry(mm, &e, gma_ops->gma_to_ggtt_pte_index(gma));
		gpa = (pte_ops->get_pfn(&e) << GTT_PAGE_SHIFT) + (gma & ~GTT_PAGE_MASK);

		trace_gma_translate(vgt->vm_id, "ggtt", 0, 0, gma, gpa);

		return gpa;
	}

	switch (mm->page_table_level) {
		case 4:
			ppgtt_get_shadow_root_entry(mm, &e, 0);
			gma_index[0] = gma_ops->gma_to_pml4_index(gma);
			gma_index[1] = gma_ops->gma_to_l4_pdp_index(gma);
			gma_index[2] = gma_ops->gma_to_pde_index(gma);
			gma_index[3] = gma_ops->gma_to_pte_index(gma);
			index = 4;
			break;
		case 3:
			ppgtt_get_shadow_root_entry(mm, &e, gma_ops->gma_to_l3_pdp_index(gma));
			gma_index[0] = gma_ops->gma_to_pde_index(gma);
			gma_index[1] = gma_ops->gma_to_pte_index(gma);
			index = 2;
			break;
		case 2:
			ppgtt_get_shadow_root_entry(mm, &e, gma_ops->gma_to_pde_index(gma));
			gma_index[0] = gma_ops->gma_to_pte_index(gma);
			index = 1;
			break;
		default:
			BUG();
	}
	/* walk into the last level shadow page table and get gpa from guest entry */
	for (i = 0; i < index; i++)
		if (!ppgtt_get_next_level_entry(mm, &e, gma_index[i],
			(i == index - 1)))
			goto err;

	gpa = (pte_ops->get_pfn(&e) << GTT_PAGE_SHIFT) + (gma & ~GTT_PAGE_MASK);

	trace_gma_translate(vgt->vm_id, "ppgtt", 0, mm->page_table_level, gma, gpa);

	return gpa;
err:
	vgt_err("invalid mm type: %d, gma %lx\n", mm->type, gma);
	return INVALID_ADDR;
}

void *vgt_gma_to_va(struct vgt_mm *mm, unsigned long gma)
{
	struct vgt_device *vgt = mm->vgt;
	unsigned long gpa;

	gpa = vgt_gma_to_gpa(mm, gma);
	if (gpa == INVALID_ADDR) {
		vgt_warn("invalid gpa! gma 0x%lx, mm type %d\n", gma, mm->type);
		return NULL;
	}

	return hypervisor_gpa_to_va(vgt, gpa);
}

/*
 * GTT MMIO emulation.
 */
bool gtt_mmio_read(struct vgt_device *vgt,
	unsigned int off, void *p_data, unsigned int bytes)
{
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	struct vgt_device_info *info = &vgt->pdev->device_info;
	unsigned long index = off >> info->gtt_entry_size_shift;
	gtt_entry_t e;

	if (bytes != 4 && bytes != 8)
		return false;

	ggtt_get_guest_entry(ggtt_mm, &e, index);
	memcpy(p_data, &e.val64 + (off & (info->gtt_entry_size - 1)), bytes);

	return true;
}

bool gtt_emulate_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct vgt_device_info *info = &vgt->pdev->device_info;
	int ret;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;

	if (bytes != 4 && bytes != 8)
		return false;

	t0 = get_cycles();
	stat->gtt_mmio_rcnt++;

	off -= info->gtt_start_offset;

	if (IS_PREBDW(vgt->pdev)) {
		ret = gtt_mmio_read(vgt, off, p_data, 4);
		if (ret && bytes == 8)
			ret = gtt_mmio_read(vgt, off + 4, (char*)p_data + 4, 4);
	} else {
		ret = gtt_mmio_read(vgt, off, p_data, bytes);
	}

	t1 = get_cycles();
	stat->gtt_mmio_rcycles += (u64) (t1 - t0);
	return ret;
}

static bool process_ppgtt_root_pointer(struct vgt_device *vgt,
		gtt_entry_t *ge, unsigned int gtt_index)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_mm *mm = NULL;
	gtt_entry_t e;
	int index;
	struct list_head *pos;

	if (!IS_PREBDW(pdev))
		return true;

	list_for_each(pos, &vgt->gtt.mm_list_head) {
		struct vgt_mm *ppgtt_mm = container_of(pos, struct vgt_mm, list);
		if (ppgtt_mm->type != VGT_MM_PPGTT || !ppgtt_mm->initialized)
			continue;

		if (gtt_index >= ppgtt_mm->pde_base_index
			&& gtt_index < ppgtt_mm->pde_base_index +
			ppgtt_mm->page_table_entry_cnt) {
			mm = ppgtt_mm;
			break;
		}
	}

	if (!mm || !mm->has_shadow_page_table)
		return true;

	gtt_init_entry(&e, GTT_TYPE_PPGTT_PDE_ENTRY, pdev, ge->val64);

	index = gtt_index - mm->pde_base_index;

	ppgtt_set_guest_root_entry(mm, &e, index);

	if (!ppgtt_handle_guest_write_root_pointer(mm, &e, index))
		return false;

	ge->type = GTT_TYPE_PPGTT_PDE_ENTRY;

	return true;
}

bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_device_info *info = &pdev->device_info;
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	unsigned long g_gtt_index = off >> info->gtt_entry_size_shift;
	bool partial_access = (bytes != info->gtt_entry_size);
	bool hi = (partial_access && (off & (info->gtt_entry_size - 1)));
	unsigned long gma;
	gtt_entry_t e, m;
	int rc;

	if (bytes != 4 && bytes != 8)
		return false;

	gma = g_gtt_index << GTT_PAGE_SHIFT;
	/* the VM may configure the whole GM space when ballooning is used */
	if (!g_gm_is_valid(vgt, gma)) {
		static int count = 0;

		/* print info every 32MB */
		if (!(count % 8192))
			vgt_dbg(VGT_DBG_MEM, "vGT(%d): capture ballooned write for %d times (%x)\n",
				vgt->vgt_id, count, off);

		count++;
		/* in this case still return true since the impact is on vgtt only */
		return true;
	}

	ggtt_get_guest_entry(ggtt_mm, &e, g_gtt_index);

	memcpy(&e.val64 + (off & (info->gtt_entry_size - 1)), p_data, bytes);

	if (partial_access && !hi)
		goto out;

	if (!process_ppgtt_root_pointer(vgt, &e, g_gtt_index))
		return false;

	if (e.type != GTT_TYPE_GGTT_PTE)
		return true;

	rc = gtt_entry_p2m(vgt, &e, &m);
	if (!rc) {
		vgt_err("VM %d: failed to translate guest gtt entry\n", vgt->vm_id);
		return false;
	}

	ggtt_set_shadow_entry(ggtt_mm, &m, g_gtt_index);
out:
	ggtt_set_guest_entry(ggtt_mm, &e, g_gtt_index);
	return true;
}

bool gtt_emulate_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct vgt_device_info *info = &vgt->pdev->device_info;
	int ret;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;

	if (bytes != 4 && bytes != 8)
		return false;

	t0 = get_cycles();
	stat->gtt_mmio_wcnt++;

	off -= info->gtt_start_offset;

	if (IS_PREBDW(vgt->pdev)) {
		ret = gtt_mmio_write(vgt, off, p_data, 4);
		if (ret && bytes == 8)
			ret = gtt_mmio_write(vgt, off + 4, (char*)p_data + 4, 4);
	} else {
		ret = gtt_mmio_write(vgt, off, p_data, bytes);
	}

	t1 = get_cycles();
	stat->gtt_mmio_wcycles += (u64) (t1 - t0);
	return ret;
}

#define ring_id_to_pp_dclv(pdev, ring_id) \
	(RB_TAIL(pdev, ring_id) - 0x30 + 0x220)

#define ring_id_to_pp_dir_base(pdev, ring_id) \
	(RB_TAIL(pdev, ring_id) - 0x30 + 0x228)

static void gen7_ppgtt_mm_switch(struct vgt_mm *mm, int ring_id)
{
	struct pgt_device *pdev = mm->vgt->pdev;
	u32 base = mm->pde_base_index << GTT_PAGE_SHIFT;

	VGT_MMIO_WRITE(pdev, ring_id_to_pp_dclv(pdev, ring_id), 0xffffffff);
	VGT_MMIO_WRITE(pdev, ring_id_to_pp_dir_base(pdev, ring_id), base);

	return;
}

struct vgt_mm *gen7_find_ppgtt_mm(struct vgt_device *vgt,
		u32 pde_base_index)
{
	struct list_head *pos;
	struct vgt_mm *mm;

	list_for_each(pos, &vgt->gtt.mm_list_head) {
		mm = container_of(pos, struct vgt_mm, list);
		if (mm->type != VGT_MM_PPGTT)
			continue;

		if (mm->pde_base_index == pde_base_index)
			return mm;
	}

	return NULL;
}

bool gen7_ppgtt_mm_setup(struct vgt_device *vgt, int ring_id)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[ring_id];
	struct vgt_mm *mm = rb->active_ppgtt_mm;
	u32 pde_base_index = rb->sring_ppgtt_info.base >> GTT_PAGE_SHIFT;

	if (!IS_PREBDW(pdev))
		return false;

	if (!rb->has_ppgtt_base_set
		|| !rb->has_ppgtt_mode_enabled)
		return true;

	if (mm)
		vgt_destroy_mm(mm);

	mm = gen7_find_ppgtt_mm(vgt, pde_base_index);
	if (mm) {
		atomic_inc(&mm->refcount);
	} else {
		mm = vgt_create_mm(vgt, VGT_MM_PPGTT, rb->ppgtt_root_pointer_type,
				NULL, rb->ppgtt_page_table_level, pde_base_index);
		if (!mm)
			return false;
	}

	rb->active_ppgtt_mm = mm;

	set_bit(ring_id, &vgt->gtt.active_ppgtt_mm_bitmap);

	if (current_render_owner(pdev) == vgt)
		gen7_ppgtt_mm_switch(mm, ring_id);

	return true;
}

void vgt_ppgtt_switch(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_vgtt_info *gtt = &vgt->gtt;
	struct vgt_mm *mm;
	int bit;

	if (!IS_PREBDW(pdev))
		return;

	if (current_render_owner(pdev) != vgt)
		return;

	for_each_set_bit(bit, &gtt->active_ppgtt_mm_bitmap, vgt->pdev->max_engines) {
		mm = vgt->rb[bit].active_ppgtt_mm;
		gen7_ppgtt_mm_switch(mm, bit);
	}
}

bool vgt_expand_shadow_page_mempool(struct pgt_device *pdev)
{
	mempool_t *mempool = pdev->gtt.mempool;
	bool rc = true;
	int new_min_nr;

	mutex_lock(&pdev->gtt.mempool_lock);

	if (mempool->curr_nr >= preallocated_shadow_pages / 3)
		goto out;

	/*
	 * Have to do this to let the pool expand directly.
	 */
	new_min_nr = preallocated_shadow_pages - 1;
	if (mempool_resize(mempool, new_min_nr, GFP_KERNEL)) {
		vgt_err("fail to resize the mempool.\n");
		rc = false;
		goto out;
	}

	new_min_nr = preallocated_shadow_pages;
	if (mempool_resize(mempool, new_min_nr, GFP_KERNEL)) {
		vgt_err("fail to resize the mempool.\n");
		rc = false;
		goto out;
	}

out:
	mutex_unlock(&pdev->gtt.mempool_lock);
	return rc;
}

static void *mempool_alloc_spt(gfp_t gfp_mask, void *pool_data)
{
	ppgtt_spt_t *spt;

	spt = kzalloc(sizeof(*spt), gfp_mask);
	if (!spt)
		return NULL;

	spt->shadow_page.page = alloc_page(gfp_mask);
	if (!spt->shadow_page.page) {
		kfree(spt);
		return NULL;
	}
	return spt;
}

static void mempool_free_spt(void *element, void *pool_data)
{
	ppgtt_spt_t *spt = element;

	__free_page(spt->shadow_page.page);
	kfree(spt);
}

bool vgt_init_vgtt(struct vgt_device *vgt)
{
	struct vgt_vgtt_info *gtt = &vgt->gtt;
	struct vgt_mm *ggtt_mm;

	hash_init(gtt->guest_page_hash_table);
	hash_init(gtt->shadow_page_hash_table);
	hash_init(gtt->el_ctx_hash_table);

	INIT_LIST_HEAD(&gtt->mm_list_head);
	INIT_LIST_HEAD(&gtt->oos_page_list_head);

	gtt->last_partial_ppgtt_access_index = -1;

	if (!vgt_expand_shadow_page_mempool(vgt->pdev)) {
		vgt_err("fail to expand the shadow page mempool.");
		return false;
	}

	ggtt_mm = vgt_create_mm(vgt, VGT_MM_GGTT,
			GTT_TYPE_GGTT_PTE, NULL, 1, 0);
	if (!ggtt_mm) {
		vgt_err("fail to create mm for ggtt.\n");
		return false;
	}

	gtt->ggtt_mm = ggtt_mm;
	return true;
}

void vgt_clean_vgtt(struct vgt_device *vgt)
{
	struct list_head *pos, *n;
	struct vgt_mm *mm;

	ppgtt_free_all_shadow_page(vgt);

	list_for_each_safe(pos, n, &vgt->gtt.mm_list_head) {
		mm = container_of(pos, struct vgt_mm, list);
		vgt->pdev->gtt.mm_free_page_table(mm);
		kfree(mm);
	}

	execlist_ctx_table_destroy(vgt);

	return;
}

static void vgt_clean_spt_oos(struct pgt_device *pdev)
{
	struct vgt_gtt_info *gtt = &pdev->gtt;
	struct list_head *pos, *n;
	oos_page_t *oos_page;

	ASSERT(list_empty(&gtt->oos_page_use_list_head));

	list_for_each_safe(pos, n, &gtt->oos_page_free_list_head) {
		oos_page = container_of(pos, oos_page_t, list);
		list_del(&oos_page->list);
		kfree(oos_page);
	}
}

static bool vgt_setup_spt_oos(struct pgt_device *pdev)
{
	struct vgt_gtt_info *gtt = &pdev->gtt;
	oos_page_t *oos_page;
	int i;

	INIT_LIST_HEAD(&gtt->oos_page_free_list_head);
	INIT_LIST_HEAD(&gtt->oos_page_use_list_head);

	for (i = 0; i < preallocated_oos_pages; i++) {
		oos_page = kzalloc(sizeof(*oos_page), GFP_KERNEL);
		if (!oos_page) {
			vgt_err("fail to pre-allocate oos page.\n");
			goto fail;
		}

		INIT_LIST_HEAD(&oos_page->list);
		INIT_LIST_HEAD(&oos_page->vm_list);
		oos_page->id = i;
		list_add_tail(&oos_page->list, &gtt->oos_page_free_list_head);
	}

	pdev->stat.oos_page_cur_avail_cnt = preallocated_oos_pages;
	pdev->stat.oos_page_min_avail_cnt = preallocated_oos_pages;
	pdev->stat.oos_page_steal_cnt = 0;
	pdev->stat.oos_page_attach_cnt = 0;
	pdev->stat.oos_page_detach_cnt = 0;

	vgt_info("%d oos pages preallocated\n", preallocated_oos_pages);

	return true;
fail:
	vgt_clean_spt_oos(pdev);
	return false;
}

bool vgt_gtt_init(struct pgt_device *pdev)
{
	if (IS_PREBDW(pdev)) {
		pdev->gtt.pte_ops = &gen7_gtt_pte_ops;
		pdev->gtt.gma_ops = &gen7_gtt_gma_ops;
		pdev->gtt.mm_alloc_page_table = gen7_mm_alloc_page_table;
		pdev->gtt.mm_free_page_table = gen7_mm_free_page_table;

		if (preallocated_shadow_pages == -1)
			preallocated_shadow_pages = 512;
		if (preallocated_oos_pages == -1)
			preallocated_oos_pages = 2048;
	} else if (IS_BDW(pdev)) {
		pdev->gtt.pte_ops = &gen8_gtt_pte_ops;
		pdev->gtt.gma_ops = &gen8_gtt_gma_ops;
		pdev->gtt.mm_alloc_page_table = gen8_mm_alloc_page_table;
		pdev->gtt.mm_free_page_table = gen8_mm_free_page_table;

		if (preallocated_shadow_pages == -1)
			preallocated_shadow_pages = 8192;
		if (preallocated_oos_pages == -1)
			preallocated_oos_pages = 4096;
	} else {
		vgt_err("Unsupported platform.\n");
		return false;
	}

	if (spt_out_of_sync) {
		if (!vgt_setup_spt_oos(pdev)) {
			vgt_err("fail to initialize SPT oos.\n");
			return false;
		}
	}

	mutex_init(&pdev->gtt.mempool_lock);

	pdev->gtt.mempool = mempool_create(preallocated_shadow_pages,
		mempool_alloc_spt, mempool_free_spt, pdev);
	if (!pdev->gtt.mempool) {
		vgt_err("fail to create mempool.\n");
		vgt_clean_spt_oos(pdev);
		return false;
	}

	return true;
}

void vgt_gtt_clean(struct pgt_device *pdev)
{
	if (spt_out_of_sync)
		vgt_clean_spt_oos(pdev);

	mempool_destroy(pdev->gtt.mempool);
}

int ring_ppgtt_mode(struct vgt_device *vgt, int ring_id, u32 off, u32 mode)
{
	vgt_state_ring_t *rb = &vgt->rb[ring_id];
	vgt_ring_ppgtt_t *v_info = &rb->vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &rb->sring_ppgtt_info;

	v_info->mode = mode;
	s_info->mode = mode;

	__sreg(vgt, off) = mode;
	__vreg(vgt, off) = mode;

	if (reg_hw_access(vgt, off)) {
		vgt_dbg(VGT_DBG_MEM, "RING mode: offset 0x%x write 0x%x\n", off, s_info->mode);
		VGT_MMIO_WRITE(vgt->pdev, off, s_info->mode);
	}

	/* sanity check */
	if ((mode & _REGBIT_PPGTT_ENABLE) && (mode & (_REGBIT_PPGTT_ENABLE << 16))) {
		/* XXX the order of mode enable for PPGTT and PPGTT dir base
		 * setting is not strictly defined, e.g linux driver first
		 * enables PPGTT bit in mode reg, then write PP dir base...
		 */
		vgt->rb[ring_id].has_ppgtt_mode_enabled = 1;
		if (IS_PREBDW(vgt->pdev)) {
			rb->ppgtt_page_table_level = 2;
			rb->ppgtt_root_pointer_type = GTT_TYPE_PPGTT_PDE_ENTRY;
		} else {
			rb->ppgtt_page_table_level = 3;
			rb->ppgtt_root_pointer_type = GTT_TYPE_PPGTT_ROOT_L3_ENTRY;

			if ((mode & _REGBIT_PPGTT64_ENABLE)
					&& (mode & (_REGBIT_PPGTT64_ENABLE << 16))) {
				printk("PPGTT 64 bit VA enabling on ring %d\n", ring_id);
				rb->ppgtt_page_table_level = 4;
				rb->ppgtt_root_pointer_type = GTT_TYPE_PPGTT_ROOT_L4_ENTRY;
			}
		}

		printk("PPGTT enabling on ring %d page table level %d type %d\n",
				ring_id, rb->ppgtt_page_table_level,
				rb->ppgtt_root_pointer_type);
	}

	return 0;
}

struct vgt_mm *gen8_find_ppgtt_mm(struct vgt_device *vgt,
		int page_table_level, void *root_entry)
{
	struct list_head *pos;
	struct vgt_mm *mm;
	u64 *src, *dst;

	list_for_each(pos, &vgt->gtt.mm_list_head) {
		mm = container_of(pos, struct vgt_mm, list);
		if (mm->type != VGT_MM_PPGTT)
			continue;

		if (mm->page_table_level != page_table_level)
			continue;

		src = root_entry;
		dst = mm->virtual_page_table;

		if (page_table_level == 3) {
			if (src[0] == dst[0]
					&& src[1] == dst[1]
					&& src[2] == dst[2]
					&& src[3] == dst[3])
				return mm;
		} else {
			if (src[0] == dst[0])
				return mm;
		}
	}

	return NULL;
}

bool vgt_g2v_create_ppgtt_mm(struct vgt_device *vgt, int page_table_level)
{
	u64 *pdp = (u64 *)&__vreg64(vgt, vgt_info_off(pdp0_lo));
	gtt_type_t root_entry_type = page_table_level == 4 ?
		GTT_TYPE_PPGTT_ROOT_L4_ENTRY : GTT_TYPE_PPGTT_ROOT_L3_ENTRY;

	struct vgt_mm *mm;

	ASSERT(page_table_level == 4 || page_table_level == 3);

	mm = gen8_find_ppgtt_mm(vgt, page_table_level, pdp);
	if (mm) {
		atomic_inc(&mm->refcount);
	} else {
		mm = vgt_create_mm(vgt, VGT_MM_PPGTT, root_entry_type,
				pdp, page_table_level, 0);
		if (!mm)
			return false;
	}

	return true;
}

bool vgt_g2v_destroy_ppgtt_mm(struct vgt_device *vgt, int page_table_level)
{
	u64 *pdp = (u64 *)&__vreg64(vgt, vgt_info_off(pdp0_lo));
	struct vgt_mm *mm;

	ASSERT(page_table_level == 4 || page_table_level == 3);

	mm = gen8_find_ppgtt_mm(vgt, page_table_level, pdp);
	if (!mm) {
		vgt_err("fail to find ppgtt instance.\n");
		return false;
	}

	vgt_destroy_mm(mm);

	return true;
}

#define get_pdp_from_context(status, pdp_udw, pdp_ldw, idx)	\
do{								\
	switch(idx) {						\
		case 0:						\
			pdp_udw = &status->pdp0_UDW;		\
			pdp_ldw = &status->pdp0_LDW;		\
			break;					\
		case 1:						\
			pdp_udw = &status->pdp1_UDW;		\
			pdp_ldw = &status->pdp1_LDW;		\
			break;					\
		case 2:						\
			pdp_udw = &status->pdp2_UDW;		\
			pdp_ldw = &status->pdp2_LDW;		\
			break;					\
		case 3:						\
			pdp_udw = &status->pdp3_UDW;		\
			pdp_ldw = &status->pdp3_LDW;		\
			break;					\
		default:					\
			BUG();					\
	}							\
}while(0);

static inline bool ppgtt_get_rootp_from_ctx(
			struct reg_state_ctx_header *state,
			gtt_entry_t *e, int idx)
{
	struct mmio_pair *pdp_udw;
	struct mmio_pair *pdp_ldw;

	get_pdp_from_context(state, pdp_udw, pdp_ldw, idx);

	e->val32[0] = pdp_ldw->val;
	e->val32[1] = pdp_udw->val;

	e->type= GTT_TYPE_INVALID;
	e->pdev = NULL;
	return true;
}

static bool ppgtt_set_rootp_to_ctx(
			struct reg_state_ctx_header *state,
			gtt_entry_t *e, int idx)
{
	struct mmio_pair *pdp_udw;
	struct mmio_pair *pdp_ldw;

	get_pdp_from_context(state, pdp_udw, pdp_ldw, idx);

	pdp_udw->val = e->val32[1];
	pdp_ldw->val = e->val32[0];

	return true;
}

bool vgt_handle_guest_write_rootp_in_context(struct execlist_context *el_ctx, int idx)
{
	struct reg_state_ctx_header *guest_ctx_state;
	struct reg_state_ctx_header *shadow_ctx_state;
	gtt_entry_t guest_rootp;
	gtt_entry_t shadow_rootp;
	gtt_entry_t ctx_g_rootp;
	struct vgt_mm *mm = el_ctx->ppgtt_mm;
	bool rc = true;

	guest_ctx_state = (struct reg_state_ctx_header *)
					el_ctx->ctx_pages[1].guest_page.vaddr;
	shadow_ctx_state = (struct reg_state_ctx_header *)
					el_ctx->ctx_pages[1].shadow_page.vaddr;

	ppgtt_get_guest_root_entry(mm, &guest_rootp, idx);
	ppgtt_get_rootp_from_ctx(guest_ctx_state, &ctx_g_rootp, idx);

	vgt_dbg(VGT_DBG_EXECLIST, "Guest root pointer in context is: 0x%llx\n",
					ctx_g_rootp.val64);
	vgt_dbg(VGT_DBG_EXECLIST, "Guest root pointer in root table is: 0x%llx\n",
					guest_rootp.val64);

	if (ctx_g_rootp.val64 == guest_rootp.val64)
		return rc;

	ctx_g_rootp.type = guest_rootp.type;
	ctx_g_rootp.pdev = guest_rootp.pdev;

	rc = ppgtt_handle_guest_write_root_pointer(mm, &ctx_g_rootp, idx);
	if (!rc)
		return rc;

	ppgtt_set_guest_root_entry(mm, &ctx_g_rootp, idx);

	ppgtt_get_shadow_root_entry(mm, &shadow_rootp, idx);
	vgt_dbg(VGT_DBG_EXECLIST, "Shadow root pointer for guest rootp is: 0x%llx\n",
					shadow_rootp.val64);
	ppgtt_set_rootp_to_ctx(shadow_ctx_state, &shadow_rootp, idx);

	return rc;
}

int switch_gtt_aperture(struct pgt_device *pdev, struct vgt_device *vgt)
{
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	unsigned long i;
	unsigned long aperture_gm_index_start;
	unsigned long aperture_gm_pages;
	uint32_t *entry = ggtt_mm->shadow_gtt;
	
	aperture_gm_index_start = vgt->aperture_offset >> GTT_PAGE_SHIFT;
	aperture_gm_pages = vgt->aperture_sz >> GTT_PAGE_SHIFT;
	for(i = aperture_gm_index_start; i < aperture_gm_pages + aperture_gm_index_start; i++){
		vgt_write_gtt(pdev, i , *(entry+i));
	}

	return 0;	
}

int switch_gtt_hidden(struct pgt_device *pdev, struct vgt_device *vgt)
{
	struct vgt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	unsigned long i;
	unsigned long high_gm_index_start;
	unsigned long high_gm_pages;
	uint32_t *entry = ggtt_mm->shadow_gtt;
	
	high_gm_index_start = vgt->hidden_gm_offset >> GTT_PAGE_SHIFT;
	high_gm_pages = (vgt->gm_sz - vgt->aperture_sz) >> GTT_PAGE_SHIFT;
	for(i = high_gm_index_start; i < high_gm_pages + high_gm_index_start; i++){
		vgt_write_gtt(pdev, i , *(entry+i));
	}

	return 0;
}

static uint32_t vgt_slot_sched_weight(uint32_t prev, uint32_t curr) {
    return (prev+curr)/2;
}

unsigned long get_hidden_gm_start(struct pgt_device *pdev, struct vgt_device *vgt)
{
	struct vgt_device_info *info = &pdev->device_info;
	int category_load = ~(1<<31);
	int i = 0;
	int category_id = 0;
	unsigned long hidden_gm_start;
	unsigned long category_sz, visible_gm_sz, total_gm_sz;
#if 1
	for(i=1; i<4; i++){
		if(pdev->category_load[i]<category_load){
			category_load = pdev->category_load[i];
			category_id = i;
		}
	}
#else
    category_load = pdev->slot_sched_info.slot_sched_cnt[1];
    category_load = vgt_slot_sched_weight(pdev->slot_sched_info.slot_prev_weighted_cnt[1],
                                        pdev->slot_sched_info.slot_sched_cnt[1]);
    category_id = 1;
    for (i=1; i<4; i++) {
        uint32_t curr_load = vgt_slot_sched_weight(pdev->slot_sched_info.slot_prev_weighted_cnt[i],
                                                pdev->slot_sched_info.slot_sched_cnt[i]);
        if (curr_load<category_load) {
            category_load = curr_load;
            category_id = i;
        }
    }
#endif
	printk("jachin: vm=%u, slot=%u\n", vgt->vm_id, category_id);

	pdev->category_load[category_id]++;
	vgt->category = category_id;

	total_gm_sz = info->max_gtt_gm_sz/SIZE_1MB;
	visible_gm_sz = hidden_gm_base(pdev)/SIZE_1MB;
	category_sz = (total_gm_sz - visible_gm_sz)/4;
	hidden_gm_start = visible_gm_sz + vgt->category * category_sz;
	return hidden_gm_start;
}

int category_sched(struct pgt_device *pdev, struct vgt_device *vgt)	/* Here could add some scheduling policies. Currently we just switch the category owner. */
{
	switch_gtt_aperture(pdev, vgt);
	//spin_lock(&pdev->pre_copy_info.slot_locks[vgt->category]);
    //printk("fish: in category_sched\n");
	if(vgt->vm_id != pdev->category_owner[vgt->category]){
		/* Jachin: add lock */
		printk("Jachin: gtt copy: %d\n", vgt->vm_id);
		//spin_lock(&pdev->pre_copy_info.slot_locks[vgt->category]);
		if(pdev->gtt_owner!=vgt->vm_id)
			switch_gtt_hidden(pdev, vgt);
		pdev->category_owner[vgt->category] = vgt->vm_id;
		//spin_unlock(&pdev->pre_copy_info.slot_locks[vgt->category]);
	}
	pdev->gtt_owner = vgt->vm_id;
	//spin_unlock(&pdev->pre_copy_info.slot_locks[vgt->category]);
	return 0;
}

/* Jachin: the pre-copy thread */
int pre_copy_thread(void *args) {
	struct vgt_pre_copy_info *info = (struct vgt_pre_copy_info*)args;
	struct vgt_device *curr, *next, *pre_copy_vgt, *render_owner;
	struct pgt_device *pdev;
	int i;
	//set_current_state(TASK_INTERRUPTIBLE);
	while (1) {
		//set_current_state(TASK_INTERRUPTIBLE);
        //printk("fish: pre_copy_thread: schedule out\n");
		//schedule();
        printk("fish: pre_copy_thread: acquire pre lock\n");
        //down(&info->pre_copy_sem);
        while (info->flag == 1 || spin_trylock(&info->main_lock) == 0) {
            msleep(5);
        }
        //while (info->flag == 1 || test_and_set_bit(0, &info->lock)) {
            //printk("fish: pre_copy_thread: failed to acquire lock\n");
        //    msleep(15);
            //printk("fish: pre_copy_thread: after sleep 15ms\n");
        //}
        printk("fish: pre_copy_thread: acquire pre lock success\n");
	//	printk("jachin: vgt = %lu\n", info->pre_copy_vgt->vm_id);
	//	info->wake_up=false;

		// get info
		curr = info->curr_vgt;
		next = info->next_vgt;
		pre_copy_vgt = NULL;
		pdev = info->pdev;
		render_owner = NULL;
		
		// store predicitive map
		info->possible_next[curr->vgt_id] = next;
		pre_copy_vgt = info->possible_next[next->vgt_id];
		
//		printk("Jachin: pre-copy mapping: %d -> %d\n", curr->vm_id, next->vm_id);

		//for (i=0; i<3; i++) {
		//	printk("jachin: slot %d, vm %d\n", i, pdev->category_owner[i]);
		//}
        //if (pre_copy_vgt != NULL)
		//    spin_lock(&info->slot_locks[pre_copy_vgt->category]); // lock the slot	

		// try to do pre-copy
		if (pre_copy_vgt == NULL|| pre_copy_vgt->vm_id==0) goto pre_copy_out; // not initialized
		render_owner = current_render_owner(pdev);	
		if (render_owner == NULL) goto pre_copy_out; // may not happen
//		printk("jachin: runder_owner: %d, slot: %d\n", render_owner->vm_id, render_owner->category);
		if (render_owner->category==pre_copy_vgt->category) goto pre_copy_out; // when the current owner and the pre-copy vgt has the same category
		
		if(pre_copy_vgt->vm_id == pdev->category_owner[pre_copy_vgt->category]) goto pre_copy_out;

		// do not copy when the pre-copy category conflict with two previous ones
//		if(pre_copy_vgt->category == curr->category || pre_copy_vgt->category == next->category)
//			continue;

		printk("Jachin: pre-copy start: %d on slot %d\n", pre_copy_vgt->vm_id, pre_copy_vgt->category);
	
		/* if the vgt to pre-copy is not the category owner, do the pre-copy */	
		//spin_lock(&info->slot_locks[pre_copy_vgt->category]); // lock the slot	
		switch_gtt_hidden(pdev, pre_copy_vgt);
		pdev->category_owner[pre_copy_vgt->category] = pre_copy_vgt->vm_id;
		//spin_unlock(&info->slot_locks[pre_copy_vgt->category]); // unlock the slot

pre_copy_out:
		//printk("Jachin: pre-copy end\n");
        //if (pre_copy_vgt != NULL)
		//    spin_unlock(&info->slot_locks[pre_copy_vgt->category]); // unlock the slot
        //printk("fish: pre_copy_thread: release main lock\n");
		//set_current_state(TASK_INTERRUPTIBLE);
        info->flag = 1;
        spin_unlock_irq(&info->main_lock);
        //clear_bit(0, &info->lock);
        //printk("fish: pre_copy_thread: release main lock success\n");
        //up(&info->pre_copy_sem);
			
	}
}
