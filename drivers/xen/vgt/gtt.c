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

#include <xen/page.h>
#include <xen/events.h>
#include <xen/xen-ops.h>
#include <xen/interface/hvm/hvm_op.h>

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
static gtt_entry_t *gtt_get_entry32(void *pt, gtt_entry_t *e,
		unsigned long index)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 4);

	if (!pt) {
		e->val32[0] = vgt_read_gtt(e->pdev, index);
		e->val32[1] = 0;
	} else {
		e->val32[0] = *((u32 *)pt + index);
		e->val32[1] = 0;
	}
	return e;
}

static gtt_entry_t *gtt_set_entry32(void *pt, gtt_entry_t *e,
		unsigned long index)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 4);

	if (!pt)
		vgt_write_gtt(e->pdev, index, e->val32[0]);
	else
		*((u32 *)pt + index) = e->val32[0];
	/* Non-LLC machine vlv needs clflush here. */
	return e;
}

static inline gtt_entry_t *gtt_get_entry64(void *pt, gtt_entry_t *e,
		unsigned long index)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		e->val64 = vgt_read_gtt64(e->pdev, index);
	else
		e->val64 = *((u64 *)pt + index);

	return e;
}

static inline gtt_entry_t *gtt_set_entry64(void *pt, gtt_entry_t *e,
		unsigned long index)
{
	struct vgt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		vgt_write_gtt64(e->pdev, index, e->val64);
	else
		*((u64 *)pt + index) = e->val64;
	/* Non-LLC machine chv needs clflush here. */
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

        mfn = g2m_pfn(vgt->vm_id, gfn);
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
static inline gtt_entry_t *mm_get_entry(struct vgt_mm *mm,
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

	ops->get_entry(page_table, e, index);
	ops->test_pse(e);

	return e;
}

static inline gtt_entry_t *mm_set_entry(struct vgt_mm *mm,
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

	return ops->set_entry(page_table, e, index);
}

#define ggtt_get_guest_entry(mm, e, index) \
	(mm->vgt->vm_id == 0) ? \
	mm_get_entry(mm, NULL, e, index) : \
	mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_set_guest_entry(mm, e, index) \
	mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_get_shadow_entry(mm, e, index) \
	mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ggtt_set_shadow_entry(mm, e, index) \
	mm_set_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_get_guest_root_entry(mm, e, index) \
	mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_set_guest_root_entry(mm, e, index) \
	mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_get_shadow_root_entry(mm, e, index) \
	mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_set_shadow_root_entry(mm, e, index) \
	mm_set_entry(mm, mm->shadow_page_table, e, index)

/*
 * PPGTT shadow page table helpers.
 */
static inline gtt_entry_t *ppgtt_spt_get_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;
	e->type = get_entry_type(type);

	ASSERT(gtt_type_is_entry(e->type));

	ops->get_entry(page_table, e, index);
	ops->test_pse(e);

	return e;
}

static inline gtt_entry_t *ppgtt_spt_set_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;

	ASSERT(gtt_type_is_entry(e->type));

	return ops->set_entry(page_table, e, index);
}

#define ppgtt_get_guest_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index)

#define ppgtt_set_guest_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index)

#define ppgtt_get_shadow_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index)

#define ppgtt_set_shadow_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index)

/*
 * Guest page mainpulation APIs.
 */
bool vgt_set_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	xen_hvm_vgt_wp_pages_t req;
	int r;

	if (guest_page->writeprotection)
		return true;

	memset(&req, 0, sizeof(xen_hvm_vgt_wp_pages_t));
	req.domid = vgt->vm_id;
	req.set = 1;
	req.nr_pages = 1;
	req.wp_pages[0] = guest_page->gfn;

	r = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (r) {
		vgt_err("fail to set write protection.\n");
		return false;
	}

	guest_page->writeprotection = true;

	atomic_inc(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

bool vgt_clear_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	xen_hvm_vgt_wp_pages_t req;
	int r;

	if (!guest_page->writeprotection)
		return true;

	memset(&req, 0, sizeof(xen_hvm_vgt_wp_pages_t));
	req.domid = vgt->vm_id;
	req.set = 0;
	req.nr_pages = 1;
	req.wp_pages[0] = guest_page->gfn;

	r = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (r) {
		vgt_err("fail to clear write protection.\n");
		return false;
	}

	guest_page->writeprotection = false;

	atomic_dec(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

bool vgt_init_guest_page(struct vgt_device *vgt, guest_page_t *guest_page,
		unsigned long gfn, guest_page_handler_t handler, void *data)
{
	INIT_HLIST_NODE(&guest_page->node);

	guest_page->vaddr = vgt_vmem_gpa_2_va(vgt, gfn << GTT_PAGE_SHIFT);
	if (!guest_page->vaddr)
		return false;

	guest_page->writeprotection = false;
	guest_page->gfn = gfn;
	guest_page->handler = handler;
	guest_page->data = data;

	hash_add(vgt->gtt.guest_page_hash_table, &guest_page->node, guest_page->gfn);

	return true;
}

void vgt_clean_guest_page(struct vgt_device *vgt, guest_page_t *guest_page)
{
	if(!hlist_unhashed(&guest_page->node))
		hash_del(&guest_page->node);

	if (guest_page->writeprotection)
		vgt_clear_guest_page_writeprotection(vgt, guest_page);
}

guest_page_t *vgt_find_guest_page(struct vgt_device *vgt, unsigned long gfn)
{
	guest_page_t *guest_page;

	hash_for_each_possible(vgt->gtt.guest_page_hash_table, guest_page, node, gfn)
		if (guest_page->gfn == gfn)
			return guest_page;

	return NULL;
}

/*
 * Shadow page manipulation routines.
 */
static inline bool vgt_init_shadow_page(struct vgt_device *vgt,
		shadow_page_t *sp, gtt_type_t type)
{
	sp->page = alloc_page(GFP_ATOMIC);
	if (!sp->page) {
		vgt_err("fail to allocate page for shadow_page_t.\n");
		return false;
	}

	sp->vaddr = page_address(sp->page);
	sp->type = type;
	memset(sp->vaddr, 0, PAGE_SIZE);

	INIT_HLIST_NODE(&sp->node);
	sp->mfn = pfn_to_mfn(page_to_pfn(sp->page));
	hash_add(vgt->gtt.shadow_page_hash_table, &sp->node, sp->mfn);

	return true;
}

static inline void vgt_clean_shadow_page(shadow_page_t *sp)
{
	if(!hlist_unhashed(&sp->node))
		hash_del(&sp->node);

	if (sp->page)
		__free_page(sp->page);
}

static inline shadow_page_t *vgt_find_shadow_page(struct vgt_device *vgt,
		unsigned long mfn)
{
	shadow_page_t *shadow_page;

	hash_for_each_possible(vgt->gtt.shadow_page_hash_table, shadow_page, node, mfn) {
		if (shadow_page->mfn == mfn)
			return shadow_page;
	}

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

	kfree(spt);
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

static bool ppgtt_handle_guest_write_page_table(guest_page_t *gpt, gtt_entry_t *we,
		unsigned long index);

static bool ppgtt_write_protection_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *gpt = (guest_page_t *)gp;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct vgt_device_info *info = &vgt->pdev->device_info;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	gtt_type_t type = get_entry_type(spt->guest_page_type);
	unsigned long index;
	gtt_entry_t e;

	if (bytes != 4 && bytes != 8)
		return false;

	if (!gpt->writeprotection)
		return false;

	e.val64 = 0;

	if (info->gtt_entry_size == 4) {
		gtt_init_entry(&e, type, vgt->pdev, *(u32 *)p_data);
	} else if (info->gtt_entry_size == 8) {
		ASSERT_VM(bytes == 8, vgt);
		gtt_init_entry(&e, type, vgt->pdev, *(u64 *)p_data);
	}

	ops->test_pse(&e);

	index = (pa & (PAGE_SIZE - 1)) >> info->gtt_entry_size_shift;

	return ppgtt_handle_guest_write_page_table(gpt, &e, index);
}

static ppgtt_spt_t *ppgtt_alloc_shadow_page(struct vgt_device *vgt,
		gtt_type_t type, unsigned long gpt_gfn)
{
	ppgtt_spt_t *spt = NULL;

	spt = kzalloc(sizeof(*spt), GFP_ATOMIC);
	if (!spt) {
		vgt_err("fail to allocate spt_t.\n");
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

		if (!vgt_set_guest_page_writeprotection(vgt, &s->guest_page))
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

	trace_guest_pt_change(spt->vgt->vm_id, "remove", spt, sp->type, we->val64, index);

	if (gtt_type_is_pt(get_next_pt_type(we->type))) {
		guest_page_t *g = vgt_find_guest_page(vgt, ops->get_pfn(we));
		if (!g) {
			vgt_err("fail to find guest page.\n");
			goto fail;
		}
		if (!ppgtt_invalidate_shadow_page(guest_page_to_ppgtt_spt(g)))
			goto fail;
	}
	ppgtt_get_shadow_entry(spt, &e, index);
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

	trace_guest_pt_change(spt->vgt->vm_id, "add", spt, sp->type, we->val64, index);

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

bool ppgtt_handle_guest_write_root_pointer(struct vgt_mm *mm,
		gtt_entry_t *we, unsigned long index)
{
	struct vgt_device *vgt = mm->vgt;
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *spt = NULL;
	gtt_entry_t e;

	if (mm->type != VGT_MM_PPGTT || !mm->shadowed)
		return false;

	trace_guest_pt_change(vgt->vm_id, __func__, NULL,
			we->type, we->val64, index);

	ppgtt_get_guest_root_entry(mm, &e, index);

	if (ops->test_present(&e)) {
		ppgtt_get_shadow_root_entry(mm, &e, index);

		trace_guest_pt_change(vgt->vm_id, "destroy old root pointer",
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
		trace_guest_pt_change(vgt->vm_id, "populate root pointer",
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

			trace_guest_pt_change(vgt->vm_id, "destroy root pointer",
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

	if (mm->has_shadow_page_table) {
		if (virtual_page_table)
			memcpy(mm->virtual_page_table, virtual_page_table,
					mm->page_table_entry_size);
		for (i = 0; i < mm->page_table_entry_cnt; i++) {
			ppgtt_get_guest_root_entry(mm, &ge, i);
			if (!ops->test_present(&ge))
				continue;

			trace_guest_pt_change(vgt->vm_id, __func__, NULL,
					ge.type, ge.val64, i);

			spt = ppgtt_populate_shadow_page_by_guest_entry(vgt, &ge);
			if (!spt) {
				vgt_err("fail to populate guest root pointer.\n");
				goto fail;
			}
			ppgtt_generate_shadow_entry(&se, spt, &ge);
			ppgtt_set_shadow_root_entry(mm, &se, i);

			trace_guest_pt_change(vgt->vm_id, "populate root pointer",
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

unsigned long gtt_pte_get_pfn(struct pgt_device *pdev, u32 pte)
{
	u64 addr = 0;

	if (IS_SNB(pdev) || IS_IVB(pdev))
		addr = (((u64)pte & 0xff0) << 28) | (u64)(pte & 0xfffff000);
	else if (IS_HSW(pdev))
		addr = (((u64)pte & 0x7f0) << 28) | (u64)(pte & 0xfffff000);

	return (addr >> GTT_PAGE_SHIFT);
}

static u32 gtt_pte_update(struct pgt_device *pdev, unsigned long pfn, u32 old_pte)
{
	u64 addr = pfn << GTT_PAGE_SHIFT;
	u32 pte, addr_mask = 0, ctl_mask = 0;

	if (IS_SNB(pdev) || IS_IVB(pdev)) {
		addr_mask = 0xff0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7;
	} else if (IS_HSW(pdev)) {
		addr_mask = 0x7f0;
		ctl_mask = _REGBIT_PTE_CTL_MASK_GEN7_5;
	}

	pte = (addr & ~0xfff) | ((addr >> 28) & addr_mask);
	pte |= (old_pte & ctl_mask);
	pte |= _REGBIT_PTE_VALID;

	return pte;
}

/*
 * IN:  p_gtt_val - guest GTT entry
 * OUT: m_gtt_val - translated machine GTT entry from guest GTT entry
 *					on success, it will be written with correct value
 *					otherwise, it will not be written
 */
int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val)
{
	unsigned long g_pfn, mfn;

	if (!(p_gtt_val & _REGBIT_PTE_VALID)) {
		*m_gtt_val = p_gtt_val;
		return 0;
	}

	g_pfn = gtt_pte_get_pfn(vgt->pdev, p_gtt_val);

	mfn = g2m_pfn(vgt->vm_id, g_pfn);
	if (mfn == INVALID_MFN){
		vgt_err("Invalid gtt entry 0x%x\n", p_gtt_val);
		return -EINVAL;
	}

	*m_gtt_val = gtt_pte_update(vgt->pdev, mfn, p_gtt_val);

	return 0;
}

/*  translate gma (graphics memory address) to guest phyiscal address
 *  by walking guest GTT table
 */
unsigned long vgt_gma_2_gpa(struct vgt_device *vgt, unsigned long gma)
{
	uint32_t gtt_index;
	unsigned long pfn, pa;

	/* Global GTT */
	if (!g_gm_is_valid(vgt, gma)) {
		vgt_err("invalid gma %lx\n", gma);
		return INVALID_ADDR;
	}
	gtt_index = gma >> GTT_PAGE_SHIFT;
	pfn = gtt_pte_get_pfn(vgt->pdev, vgt->vgtt[gtt_index]);
	pa = (pfn << PAGE_SHIFT) + (gma & ~PAGE_MASK);
	return pa;
}

static unsigned long vgt_gma_2_shadow_gpa(struct vgt_device *vgt, unsigned long gma)
{
	unsigned long gpa;
	vgt_ppgtt_pte_t *p;
	u32 *e, pte;

	ASSERT(vgt->vm_id != 0);

	if (unlikely(gma >= (1 << 31))) {
		vgt_warn("invalid gma value 0x%lx\n", gma);
		return INVALID_ADDR;
	}

	p = &vgt->shadow_pte_table[((gma >> 22) & 0x1ff)];

	/* gpa is physical pfn from shadow page table, we need VM's
	 * pte page entry */
	if (!p->guest_pte_va) {
		vgt_warn("No guest pte mapping? index %lu\n",(gma >> 22) & 0x3ff);
		return INVALID_ADDR;
	}

	e = (u32 *)p->guest_pte_va;
	pte = *((u32*)(e + ((gma >> 12) & 0x3ff)));
	gpa = (gtt_pte_get_pfn(vgt->pdev, pte) << PAGE_SHIFT) + (gma & ~PAGE_MASK);
	return gpa;
}

static unsigned long vgt_gma_2_dom0_ppgtt_gpa(struct vgt_device *vgt, unsigned long gma)
{
	/* dom0 has no shadow PTE */
	uint32_t gtt_index;
	unsigned long pfn, gpa;
	u32 *ent, pte;

	if (unlikely(gma >= (1 << 31))) {
		vgt_warn("invalid gma value 0x%lx\n", gma);
		return INVALID_ADDR;
	}

	gtt_index = vgt->ppgtt_base + ((gma >> 22) & 0x1ff);
	pfn = gtt_pte_get_pfn(vgt->pdev, vgt->vgtt[gtt_index]);

	/* dom0 PTE page */
	ent = (u32*)mfn_to_virt(pfn);
	pte = *((u32*)(ent + ((gma >> 12) & 0x3ff)));
	gpa = (gtt_pte_get_pfn(vgt->pdev, pte) << PAGE_SHIFT) + (gma & ~PAGE_MASK);
	return gpa;
}

void* vgt_gma_to_va(struct vgt_device *vgt, unsigned long gma, bool ppgtt)
{
	unsigned long gpa;

	if (!ppgtt) {
		gpa = vgt_gma_2_gpa(vgt, gma);
	} else {
		if (vgt->vm_id != 0)
			gpa = vgt_gma_2_shadow_gpa(vgt, gma);
		else
			gpa = vgt_gma_2_dom0_ppgtt_gpa(vgt, gma);
	}

	if (gpa == INVALID_ADDR) {
		vgt_warn("invalid gpa! gma 0x%lx, ppgtt %s\n", gma, ppgtt ? "yes":"no");
		return NULL;
	}

	return vgt_vmem_gpa_2_va(vgt, gpa);
}

/* handler to set page wp */

int vgt_set_wp_pages(struct vgt_device *vgt, int nr, unsigned long *pages,
			int *idx)
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
		req.wp_pages[i] = pages[i];

	rc = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (rc)
		vgt_err("Set WP pages failed!\n");
	else {
		/* Add pages in hash table */
		struct vgt_wp_page_entry *mht;

		for (i = 0; i < nr; i++) {
			mht = kmalloc(sizeof(*mht), GFP_ATOMIC);
			if (!mht) {
				vgt_err("out of memory!\n");
				vgt_unset_wp_pages(vgt, nr, pages);
				return -ENOMEM;
			}
			mht->pfn = pages[i];
			mht->idx = idx[i];
			vgt_add_wp_page_entry(vgt, mht);
		}
	}

	return rc;
}


int vgt_set_wp_page(struct vgt_device *vgt, unsigned long pfn, int idx)
{
	return vgt_set_wp_pages(vgt, 1, &pfn, &idx);
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
		req.wp_pages[i] = pages[i];

	rc = HYPERVISOR_hvm_op(HVMOP_vgt_wp_pages, &req);
	if (rc)
		vgt_err("Unset WP pages failed!\n");
	else {
		for (i = 0; i < nr; i++)
			vgt_del_wp_page_entry(vgt, pages[i]);
	}

	return rc;
}

int vgt_unset_wp_page(struct vgt_device *vgt, unsigned long pfn)
{
	return vgt_unset_wp_pages(vgt, 1, &pfn);
}

int vgt_ppgtt_shadow_pte_init(struct vgt_device *vgt, int idx, dma_addr_t virt_pte)
{
	int i;
	vgt_ppgtt_pte_t *p = &vgt->shadow_pte_table[idx];
	u32 *ent;
	u32 *shadow_ent;
	dma_addr_t addr, s_addr;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(vgt->vm_id != 0);

	if (!p->pte_page) {
		vgt_err("Uninitialized shadow PTE page at index %d?\n", idx);
		return -1;
	}

	p->guest_pte_va = vgt_vmem_gpa_2_va(vgt, virt_pte);
	if (!p->guest_pte_va) {
		vgt_err("Failed to get guest PTE page memory access!\n");
		return -1;
	}
	ent = p->guest_pte_va;

	shadow_ent = kmap_atomic(p->pte_page);

	/* for each PTE entry */
	for (i = 0; i < 1024; i++) {
		/* check valid */
		if ((ent[i] & _REGBIT_PTE_VALID) == 0)
			continue;
		/* get page physical address */
		addr = gtt_pte_get_pfn(pdev, ent[i]);

		/* get real physical address for that page */
		s_addr = g2m_pfn(vgt->vm_id, addr);
		if (s_addr == INVALID_MFN) {
			vgt_err("vGT: VM[%d]: Failed to get machine address for 0x%lx\n",
				vgt->vm_id, (unsigned long)addr);
			return -1;
		}

		/* update shadow PTE entry with targe page address */
		shadow_ent[i] = gtt_pte_update(pdev, s_addr, ent[i]);
	}
	kunmap_atomic(shadow_ent);
	/* XXX unmap guest VM page? */
	return 0;
}

/* Process needed shadow setup for one PDE entry.
 * i: index from PDE base
 * pde: guest GTT PDE entry value
 */
static void
vgt_ppgtt_pde_handle(struct vgt_device *vgt, unsigned int i, u32 pde)
{
	struct pgt_device *pdev = vgt->pdev;
	u32 shadow_pde;
	unsigned int index, h_index;
	dma_addr_t pte_phy;

	if (!(pde & _REGBIT_PDE_VALID)) {
		printk("vGT(%d): PDE %d not valid!\n", vgt->vgt_id, i);
		return;
	}

	if ((pde & _REGBIT_PDE_PAGE_32K)) {
		printk("vGT(%d): 32K page in PDE!\n", vgt->vgt_id);
		vgt->shadow_pde_table[i].big_page = true;
	} else
		vgt->shadow_pde_table[i].big_page = false;

	vgt->shadow_pde_table[i].entry = pde;

	pte_phy = gtt_pte_get_pfn(pdev, pde);
	pte_phy <<= PAGE_SHIFT;

	vgt->shadow_pde_table[i].virtual_phyaddr = pte_phy;

	/* allocate shadow PTE page, and fix it up */
	vgt_ppgtt_shadow_pte_init(vgt, i, pte_phy);

	/* WP original PTE page */
	vgt_set_wp_page(vgt, pte_phy >> PAGE_SHIFT, i);

	shadow_pde = gtt_pte_update(pdev,
					vgt->shadow_pde_table[i].shadow_pte_maddr >> GTT_PAGE_SHIFT,
					pde);

	if (vgt->shadow_pde_table[i].big_page) {
		/* For 32K page, even HVM thinks it's continual, it's
		 * really not on physical pages. But fallback to 4K
		 * addressing can still provide correct page reference.
		 */
		shadow_pde &= ~_REGBIT_PDE_PAGE_32K;
	}

	index = vgt->ppgtt_base + i;
	h_index = g2h_gtt_index(vgt, index);

	/* write_gtt with new shadow PTE page address */
	vgt_write_gtt(vgt->pdev, h_index, shadow_pde);
}


static void
vgt_ppgtt_pde_write(struct vgt_device *vgt, unsigned int g_gtt_index, u32 g_gtt_val)
{
	int i = g_gtt_index - vgt->ppgtt_base;
	u32 h_gtt_index;

	if (vgt->shadow_pde_table[i].entry == g_gtt_val) {
		vgt_dbg(VGT_DBG_MEM, "write same PDE value?\n");
		return;
	}

	vgt_dbg(VGT_DBG_MEM, "write PDE[%d] old: 0x%x new: 0x%x\n", i, vgt->shadow_pde_table[i].entry, g_gtt_val);

	if (vgt->shadow_pde_table[i].entry & _REGBIT_PDE_VALID)
		vgt_unset_wp_page(vgt, vgt->shadow_pde_table[i].virtual_phyaddr >> PAGE_SHIFT);

	if (!(g_gtt_val & _REGBIT_PDE_VALID)) {
		h_gtt_index = g2h_gtt_index(vgt, g_gtt_index);
		vgt_write_gtt(vgt->pdev, h_gtt_index, 0);
	} else {
		vgt_ppgtt_pde_handle(vgt, i, g_gtt_val);
	}
}

static bool gtt_mmio_read32(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct vgt_device_info *info = &vgt->pdev->device_info;
	uint32_t g_gtt_index;

	ASSERT(bytes == 4);

	off -= info->gtt_start_offset;
	/*
	if (off >= vgt->vgtt_sz) {
		vgt_dbg(VGT_DBG_MEM, "vGT(%d): captured out of range GTT read on off %x\n", vgt->vgt_id, off);
		return false;
	}
	*/

	g_gtt_index = off >> info->gtt_entry_size_shift;
	*(uint32_t*)p_data = vgt->vgtt[g_gtt_index];
	if (vgt->vm_id == 0) {
		*(uint32_t*)p_data = vgt_read_gtt(vgt->pdev,
						  g_gtt_index);
	} else if (off < vgt->vgtt_sz) {
		*(uint32_t*)p_data = vgt->vgtt[g_gtt_index];
	} else {
		printk("vGT(%d): captured out of range GTT read on "
		       "off %x\n", vgt->vgt_id, off);
		return false;
	}
	
	return true;
}

bool gtt_emulate_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ret;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;

	t0 = get_cycles();
	stat->gtt_mmio_rcnt++;

	ASSERT(bytes == 4 || bytes == 8);

	ret = gtt_mmio_read32(vgt, off, p_data, 4);
	if (ret && bytes == 8)
		ret = gtt_mmio_read32(vgt, off + 4, (char*)p_data + 4, 4);

	t1 = get_cycles();
	stat->gtt_mmio_rcycles += (u64) (t1 - t0);
	return ret;
}

#define GTT_INDEX_MB(x) ((SIZE_1MB*(x)) >> GTT_PAGE_SHIFT)

static bool gtt_mmio_write32(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct vgt_device_info *info = &vgt->pdev->device_info;
	uint32_t g_gtt_val, h_gtt_val, g_gtt_index, h_gtt_index;
	int rc;
	uint64_t g_addr;

	ASSERT(bytes == 4);

	off -= info->gtt_start_offset;

	g_gtt_index = off >> info->gtt_entry_size_shift;
	g_gtt_val = *(uint32_t*)p_data;
	vgt->vgtt[g_gtt_index] = g_gtt_val;

	g_addr = g_gtt_index << GTT_PAGE_SHIFT;
	/* the VM may configure the whole GM space when ballooning is used */
	if (!g_gm_is_valid(vgt, g_addr)) {
		static int count = 0;

		/* print info every 32MB */
		if (!(count % 8192))
			vgt_dbg(VGT_DBG_MEM, "vGT(%d): capture ballooned write for %d times (%x)\n",
				vgt->vgt_id, count, off);

		count++;
		/* in this case still return true since the impact is on vgtt only */
		goto out;
	}

	if (vgt->ppgtt_initialized && vgt->vm_id &&
			g_gtt_index >= vgt->ppgtt_base &&
			g_gtt_index < vgt->ppgtt_base + VGT_PPGTT_PDE_ENTRIES) {
		vgt_dbg(VGT_DBG_MEM, "vGT(%d): Change PPGTT PDE %d!\n", vgt->vgt_id, g_gtt_index);
		vgt_ppgtt_pde_write(vgt, g_gtt_index, g_gtt_val);
		goto out;
	}

	rc = gtt_p2m(vgt, g_gtt_val, &h_gtt_val);
	if (rc < 0){
		vgt_err("vGT(%d): failed to translate g_gtt_val(%x)\n", vgt->vgt_id, g_gtt_val);
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
	return true;
}

bool gtt_emulate_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ret;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;

	t0 = get_cycles();
	stat->gtt_mmio_wcnt++;

	ASSERT(bytes == 4 || bytes == 8);

	ret = gtt_mmio_write32(vgt, off, p_data, 4);
	if (ret && bytes == 8)
		ret = gtt_mmio_write32(vgt, off + 4, (char*)p_data + 4, 4);

	t1 = get_cycles();
	stat->gtt_mmio_wcycles += (u64) (t1 - t0);
	return ret;
}

/* So idea is that for PPGTT base in GGTT, real PDE entry will point to shadow
 * PTE, then shadow PTE entry will point to final page. So have to fix shadow
 * PTE address in PDE, and final page address in PTE. That's two-phrase address
 * fixing.
 */

/* Handle write protect fault on virtual PTE page */
bool vgt_ppgtt_handle_pte_wp(struct vgt_device *vgt, struct vgt_wp_page_entry *e,
				unsigned int offset, void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	int index, i;
	u32 *pte;
	unsigned long g_val = 0, g_addr = 0, h_addr = 0;
	struct vgt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	ASSERT(vgt->vm_id != 0);

	t0 = get_cycles();

	vgt_dbg(VGT_DBG_MEM, "PTE WP handler: offset 0x%x data 0x%lx bytes %d\n", offset, *(unsigned long *)p_data, bytes);

	i = e->idx;

	g_val = *(unsigned long*)p_data;

	/* find entry index, fill in shadow PTE */

	index = (offset & (PAGE_SIZE - 1)) >> 2;

	g_addr = gtt_pte_get_pfn(pdev, g_val);

	h_addr = g2m_pfn(vgt->vm_id, g_addr);
	if (h_addr == INVALID_MFN) {
		vgt_err("Failed to convert WP page at 0x%lx\n", g_addr);
		return false;
	}

	if (vgt->shadow_pte_table[i].guest_pte_va) {
		u32 *guest_pte;
		guest_pte = (u32*)vgt->shadow_pte_table[i].guest_pte_va;
		guest_pte[index] = gtt_pte_update(pdev, g_addr, g_val);
	}

	pte = kmap_atomic(vgt->shadow_pte_table[i].pte_page);
	pte[index] = gtt_pte_update(pdev, h_addr, g_val);
	clflush((u8 *)pte + index * 4);
	kunmap_atomic(pte);

	vgt_dbg(VGT_DBG_MEM, "WP: PDE[%d], PTE[%d], entry 0x%x, g_addr 0x%lx, h_addr 0x%lx\n", i, index, pte[index], g_addr, h_addr);

	t1 = get_cycles();
	stat->ppgtt_wp_cnt++;
	stat->ppgtt_wp_cycles += t1 - t0;

	return true;
}

static void vgt_init_ppgtt_hw(struct vgt_device *vgt, u32 base)
{
	/* only change HW setting if vgt is current render owner.*/
	if (current_render_owner(vgt->pdev) != vgt)
		return;

	/* Rewrite PP_DIR_BASE to let HW reload PDs in internal cache */
	VGT_MMIO_WRITE(vgt->pdev, _REG_RCS_PP_DCLV, 0xffffffff);
	VGT_MMIO_WRITE(vgt->pdev, _REG_RCS_PP_DIR_BASE_IVB, base);

	VGT_MMIO_WRITE(vgt->pdev, _REG_BCS_PP_DCLV, 0xffffffff);
	VGT_MMIO_WRITE(vgt->pdev, _REG_BCS_PP_DIR_BASE, base);

	VGT_MMIO_WRITE(vgt->pdev, _REG_VCS_PP_DCLV, 0xffffffff);
	VGT_MMIO_WRITE(vgt->pdev, _REG_VCS_PP_DIR_BASE, base);

	if (IS_HSW(vgt->pdev) && vgt->vebox_support) {
		VGT_MMIO_WRITE(vgt->pdev, _REG_VECS_PP_DCLV, 0xffffffff);
		VGT_MMIO_WRITE(vgt->pdev, _REG_VECS_PP_DIR_BASE, base);
	}
}

void vgt_ppgtt_switch(struct vgt_device *vgt)
{
	u32 base = vgt->rb[0].sring_ppgtt_info.base;
	vgt_dbg(VGT_DBG_MEM, "vGT: VM(%d): switch to ppgtt base 0x%x\n", vgt->vm_id, base);
	vgt_init_ppgtt_hw(vgt, base);
}

bool vgt_setup_ppgtt(struct vgt_device *vgt)
{
	u32 base = vgt->rb[0].sring_ppgtt_info.base;
	int i;
	u32 pde, gtt_base;
	unsigned int index;

	vgt_info("vgt_setup_ppgtt on vm %d: PDE base 0x%x\n", vgt->vm_id, base);

	gtt_base = base >> PAGE_SHIFT;

	vgt->ppgtt_base = gtt_base;

	/* dom0 already does mapping for PTE page itself and PTE entry target
	 * page. So we're just ready to go.
	 */
	if (vgt->vm_id == 0)
		goto finish;

	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		index = gtt_base + i;

		/* Just use guest virtual value instead of real machine address */
		pde = vgt->vgtt[index];

		vgt_ppgtt_pde_handle(vgt, i, pde);
	}

finish:
	vgt_init_ppgtt_hw(vgt, base);

	vgt->ppgtt_initialized = true;

	return true;
}

bool vgt_init_shadow_ppgtt(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	vgt_ppgtt_pte_t *p;
	dma_addr_t dma_addr;

	/* only hvm guest needs shadowed PT pages */
	ASSERT(vgt->vm_id != 0);

	vgt_dbg(VGT_DBG_MEM, "vgt_init_shadow_ppgtt for vm %d\n", vgt->vm_id);

	/* each PDE entry has one shadow PTE page */
	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		p = &vgt->shadow_pte_table[i];
		p->pte_page = alloc_page(GFP_ATOMIC | __GFP_ZERO);
		if (!p->pte_page) {
			vgt_err("Init shadow PTE page failed!\n");
			return false;
		}

		dma_addr = pci_map_page(pdev->pdev, p->pte_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
			vgt_err("Pci map shadow PTE page failed!\n");
			return false;
		}

		p->shadow_addr = dma_addr;
		vgt->shadow_pde_table[i].shadow_pte_maddr = p->shadow_addr;
	}
	return true;
}

void vgt_destroy_shadow_ppgtt(struct vgt_device *vgt)
{
	int i;
	vgt_ppgtt_pte_t *p;

	/* only hvm guest needs shadowed PT pages */
	ASSERT(vgt->vm_id != 0);

	for (i = 0; i < VGT_PPGTT_PDE_ENTRIES; i++) {
		p = &vgt->shadow_pte_table[i];

		if (vgt->ppgtt_initialized) {
			vgt_unset_wp_page(vgt, vgt->shadow_pde_table[i].virtual_phyaddr >> PAGE_SHIFT);
		}
		__free_page(p->pte_page);
	}
}

bool vgt_init_vgtt(struct vgt_device *vgt)
{
	struct vgt_vgtt_info *gtt = &vgt->gtt;

	hash_init(gtt->guest_page_hash_table);
	hash_init(gtt->shadow_page_hash_table);

	INIT_LIST_HEAD(&gtt->mm_list_head);

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

	return;
}

void vgt_reset_dom0_ppgtt_state(void)
{
	int i;
	struct vgt_device *vgt = vgt_dom0;

	vgt->ppgtt_initialized = false;

	for (i = 0; i < MAX_ENGINES; i++) {
		vgt->rb[i].has_ppgtt_mode_enabled = 0;
		vgt->rb[i].has_ppgtt_base_set = 0;
	}
}

/* XXX assume all rings use same PPGTT table, so try to initialize once
 * all bases are set.
 */
void vgt_try_setup_ppgtt(struct vgt_device *vgt)
{
	int ring, i, num;
	u32 base;

	if (vgt->vebox_support)
		num = 4;
	else
		num = 3;

	for (ring = 0; ring < num; ring++) {
		if (!vgt->rb[ring].has_ppgtt_base_set)
			return;
	}

	base = vgt->rb[0].vring_ppgtt_info.base;
	for (i = 1; i < num; i++) {
		if (vgt->rb[i].vring_ppgtt_info.base != base) {
			printk(KERN_WARNING "zhen: different PPGTT base set is not supported now!\n");
			vgt->pdev->enable_ppgtt = 0;
			return;
		}
	}
	vgt_dbg(VGT_DBG_MEM, "zhen: all rings are set PPGTT base and use single table!\n");
	vgt_setup_ppgtt(vgt);
}

int ring_ppgtt_mode(struct vgt_device *vgt, int ring_id, u32 off, u32 mode)
{
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &vgt->rb[ring_id].sring_ppgtt_info;

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
		printk("PPGTT enabling on ring %d\n", ring_id);
		/* XXX the order of mode enable for PPGTT and PPGTT dir base
		 * setting is not strictly defined, e.g linux driver first
		 * enables PPGTT bit in mode reg, then write PP dir base...
		 */
		vgt->rb[ring_id].has_ppgtt_mode_enabled = 1;
	}

	return 0;
}
