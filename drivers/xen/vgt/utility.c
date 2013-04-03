/*
 * Various utility helpers, and interfaces coupled to Xen/i915
 *
 * This file is provided under a GPLv2 license.
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
 *
 */

#include <linux/pci.h>
#include <linux/delay.h>

#include <drm/intel-gtt.h>

#include <asm/cacheflush.h>
#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>

#include <xen/vgt.h>
#include <xen/xen-ops.h>
#include <xen/interface/memory.h>
#include <xen/interface/hvm/params.h>

#include "vgt.h"

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
void show_debug(struct pgt_device *pdev, int ring_id)
{
	vgt_reg_t reg;
	struct vgt_device *vgt_dom1 = default_device.device[1];

	if (vgt_dom1) {
		vgt_show_irq_state(vgt_dom0);
		vgt_show_irq_state(vgt_dom1);
		printk("DERRMR: %x\n", VGT_MMIO_READ(pdev, 0x44050));
		printk("VBLANK_A: %x\n", VGT_MMIO_READ(pdev, _REG_VBLANK_A));
		printk("VBLANK_B: %x\n", VGT_MMIO_READ(pdev, _REG_VBLANK_B));
	}

	printk("debug registers(ring-%d),reg maked with <*> may not apply to every ring):\n", ring_id);
	printk("....EIR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_RCS_EIR));
	printk("....EMR: %x\n", VGT_MMIO_READ(pdev, _REG_RCS_EMR));
	printk("....ESR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_RCS_ESR));
	printk("....blit EIR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_BCS_EIR));
	printk("....blit ESR: 0x%x\n", VGT_MMIO_READ(pdev, _REG_BCS_ESR));
	printk("....%x: %x\n", 0x2064 + 0x10000*ring_id,
		VGT_MMIO_READ(pdev, 0x2064 + 0x10000*ring_id));
	printk("....%x: 0x%x\n", 0x2068 + 0x10000*ring_id,
		VGT_MMIO_READ(pdev, 0x2068 + 0x10000*ring_id));
	reg = VGT_MMIO_READ(pdev, 0x2070 + 0x10000*ring_id);
	printk("....INSTPS* (parser state): 0x%x :\n", reg);
	printk("....ACTHD(active header): 0x%x\n", VGT_MMIO_READ(pdev, 0x2074 + 0x10000*ring_id));
	printk("....UHPTR(pending header): %x\n", VGT_MMIO_READ(pdev, _REG_RCS_UHPTR));
	printk("....%x: 0x%x\n", 0x2078 + 0x10000*ring_id,
		VGT_MMIO_READ(pdev, 0x2078 + 0x10000*ring_id));
	printk("....CSCMDOP* (instruction DWORD): 0x%x\n", VGT_MMIO_READ(pdev, 0x220C + 0x10000*ring_id));
	printk("....CSCMDVLD* (command buffer valid): 0x%x\n", VGT_MMIO_READ(pdev, 0x2210 + 0x10000*ring_id));
	printk("(informative)\n");
	printk("....INSTDONE_1(FYI)*: 0x%x\n", VGT_MMIO_READ(pdev, 0x206C + 0x10000*ring_id));
	printk("....INSTDONE_2*: 0x%x\n", VGT_MMIO_READ(pdev, 0x207C + 0x10000*ring_id));
}

/*
 * Show some global register settings, if we care about bits
 * in those registers.
 *
 * normally invoked from initialization phase, and mmio emulation
 * logic
 */
void show_mode_settings(struct pgt_device *pdev)
{
	vgt_reg_t val;
	struct vgt_device *vgt1 = default_device.device[1];

	if (current_render_owner(pdev))
		printk("Current render owner: %d\n", current_render_owner(pdev)->vgt_id);

#define SHOW_MODE(reg)		\
	do{				\
		val = VGT_MMIO_READ(pdev, reg);	\
		printk("vGT: "#reg"(%x): p(%x), 0(%x), 1(%x)\n",	\
			reg, val, __sreg(vgt_dom0, reg), vgt1 ? __sreg(vgt1, reg) : 0);	\
	} while (0);
	SHOW_MODE(_REG_RCS_MI_MODE);
	SHOW_MODE(_REG_VCS_MI_MODE);
	SHOW_MODE(_REG_BCS_MI_MODE);

	if (pdev->is_ivybridge || pdev->is_haswell) {
		SHOW_MODE(_REG_RCS_GFX_MODE_IVB);
		SHOW_MODE(_REG_BCS_BLT_MODE_IVB);
		SHOW_MODE(_REG_VCS_MFX_MODE_IVB);
		SHOW_MODE(_REG_CACHE_MODE_0_IVB);
		SHOW_MODE(_REG_CACHE_MODE_1_IVB);
		SHOW_MODE(_REG_GT_MODE_IVB);
	} else if (pdev->is_sandybridge) {
		SHOW_MODE(_REG_GFX_MODE);
		SHOW_MODE(_REG_ARB_MODE);
		SHOW_MODE(_REG_GT_MODE);
		SHOW_MODE(_REG_CACHE_MODE_0);
		SHOW_MODE(_REG_CACHE_MODE_1);
	}

	SHOW_MODE(_REG_RCS_INSTPM);
	SHOW_MODE(_REG_VCS_INSTPM);
	SHOW_MODE(_REG_BCS_INSTPM);

	SHOW_MODE(_REG_TILECTL);
}

void show_batchbuffer(struct pgt_device *pdev, u32 addr)
{
	int i, index1, index2, pte_val;
	char *p_contents;

	index1 = GTT_INDEX(pdev, pdev->batch_buffer_page);
	index2 = GTT_INDEX(pdev, addr);
	pte_val = vgt_read_gtt(pdev, index2);
	vgt_write_gtt(pdev, index1, pte_val);

	p_contents = phys_aperture_vbase(pdev) +
		pdev->batch_buffer_page +
		(addr & ~GTT_PAGE_MASK);
	printk("Batch buffer remaps to %x (p_contents: %llx)\n",
		pte_val, (u64)p_contents);
	printk("[%08x]:", (u32)(addr & ~GTT_PAGE_MASK));
	for (i = 0; i < 32; i += 4)
		printk(" %08x", *((u32 *)p_contents + i));
	printk("\n");
}

/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
void show_ringbuffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	vgt_reg_t p_tail, p_head, p_start, p_ctl;
	char *p_contents;
	int i;
	struct vgt_device *vgt = current_render_owner(pdev);
	u32* cur;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	p_start = VGT_MMIO_READ(pdev, RB_START(pdev, ring_id));
	p_ctl = VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id));
	printk("ring buffer(%d): head (0x%x) tail(0x%x), start(0x%x), ctl(0x%x)\n", ring_id,
		p_head, p_tail, p_start, p_ctl);
	printk("psmi idle:(%d), mi_mode idle:(%d)\n",
		VGT_MMIO_READ(pdev, pdev->ring_psmi[ring_id]) & _REGBIT_PSMI_IDLE_INDICATOR,
		VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_RINGS_IDLE);

	p_head &= RB_HEAD_OFF_MASK;
	p_contents = phys_aperture_vbase(pdev) + p_start + p_head;
	printk("p_contents(%lx)\n", (unsigned long)p_contents);
	/* FIXME: consider wrap */
	for (i = -(bytes/4); i < bytes/4; i++) {
		if (!(i % 8))
			printk("\n[%08x]:", p_head + i * 4);
		printk(" %8x", *((u32*)p_contents + i));
		if (!i)
			printk("(*)");
	}
	printk("\n");

	cur = (u32*)p_contents - 2;
	if ((*cur & 0xfffff000) == 0x18800000 && vgt) {
		u32 val, h_val;
		u64 mfn;
		int rc;
		extern int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val);

		cur++;
		printk("Hang in batch buffer (%x)\n", *cur);
		val = vgt->vgtt[GTT_INDEX(pdev, *cur)];
		printk("vGTT: %x\n", val);
		rc = gtt_p2m(vgt, val, &h_val);
		if (rc < 0) {
			printk("failed to translate\n");
		} else {
			mfn = gtt_pte_get_pfn(pdev, h_val);
			printk("MACH: %x %llx\n", h_val, mfn);
		}
		printk("Actual pGTT: %x\n",
			vgt_read_gtt(pdev, GTT_INDEX(pdev, *cur)));
		show_batchbuffer(pdev, VGT_MMIO_READ(pdev,
			_REG_RCS_ACTHD + 0x10000 * ring_id));
	}
}

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

/* get VM's total memory size */
unsigned long xen_get_vm_mem_sz(int vm_id)
{
	struct xen_domctl arg;
	int rc;

	arg.domain = vm_id;
	arg.cmd = XEN_DOMCTL_getdomaininfo;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&arg);
	if (rc<0){
		vgt_err("HYPERVISOR_domctl fail ret=%d\n",rc);
		return 0;
	}

	return arg.u.getdomaininfo.tot_pages << PAGE_SHIFT;
}

uint32_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off)
{
	unsigned long bar_s, bar_size=0;
	struct pci_dev *dev = pdev->pdev;

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_s);
	pci_write_config_dword(dev, bar_off, 0xFFFFFFFF);

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_size);
	vgt_dbg("read back bar_size %lx\n", bar_size);
	bar_size &= ~0xf; /* bit 4-31 */
	vgt_dbg("read back bar_size1 %lx\n", bar_size);
	bar_size = 1 << find_first_bit(&bar_size, BITS_PER_LONG);
	vgt_dbg("read back bar_size2 %lx\n", bar_size);

	pci_write_config_dword(dev, bar_off, bar_s);

#if 0
	bar_s = pci_conf_read32( 0, vgt_bus, vgt_dev, vgt_fun, bar_off);
	pci_conf_write32(0, vgt_bus, vgt_dev, vgt_fun, bar_off, 0xFFFFFFFF);

	bar_size = pci_conf_read32(0, vgt_bus, vgt_dev, vgt_fun, bar_off);
	bar_size &= ~0xf; /* bit 4-31 */
	bar_size = 1 << find_first_bit(&bar_size, sizeof(bar_size));

	pci_conf_write32(0, vgt_bus, vgt_dev, vgt_fun, bar_offset, bar_s);
#endif
	return bar_size;
}

uint64_t vgt_get_gtt_size(struct pci_bus *bus)
{
	uint16_t gmch_ctrl;

	ASSERT(!bus->number);
	/* GTT size is within GMCH. */
	pci_bus_read_config_word(bus, 0, _REG_GMCH_CONTRL, &gmch_ctrl);
	switch ( (gmch_ctrl >> 8) & 3 ) {
	case	1:
		return 1 * SIZE_1MB;
	case	2:
		return 2 * SIZE_1MB;
	default:
		printk("Wrong GTT memory size\n");
		break;
	}
	return 0;
}

/*
 * random GTT entry check
 */
void check_gtt(struct pgt_device *pdev)
{
	static unsigned int addr[] = {
	0x00000000, 0x02000000, 0x04000000, 0x08000000,
	0x0C000000, 0x0FFFF000, 0x10000000, 0x20000000,
	0x40000000, 0x60000000, 0x7FFFF000 };

	int i;

	for (i = 0; i < ARRAY_SIZE(addr); i++)
		printk("GMADR: 0x08%x, GTT INDEX: %x, GTT VALUE: %x\n",
			addr[i], GTT_INDEX(pdev, addr[i]),
			vgt_read_gtt(pdev, GTT_INDEX(pdev, addr[i])));
}

u32 __inline dma_addr_to_pte_uc(struct pgt_device *pdev, dma_addr_t addr)
{
	u32 pte;

	if (pdev->is_haswell) {
		/* Haswell has new cache control bits */
		pte = addr & ~0xfff;
		pte |= (addr >> 28) & 0x7f0;
		pte |= 1; /* valid */
	} else {
		pte = addr & ~0xfff;
		pte |= (addr >> 28) & 0xff0;
		pte |= (1 << 1); /* UC */
		pte |= 1; /* valid */
	}
	return pte;
}

/* FIXME: allocate instead of static */
#define VGT_APERTURE_PAGES	(VGT_RSVD_APERTURE_SZ >> GTT_PAGE_SHIFT)
struct page *pages[VGT_APERTURE_PAGES];
struct page *dummy_page;
dma_addr_t dummy_addr;
/* TODO: check license. May move to another file */
int setup_gtt(struct pgt_device *pdev)
{
	struct page *page;
	int i, ret, index;
	dma_addr_t dma_addr;
	u32 pte;

	check_gtt(pdev);
	printk("vGT: clear all GTT entries.\n");
	dummy_page = alloc_page(GFP_KERNEL | __GFP_ZERO | GFP_DMA32);
	if (!dummy_page)
		return -ENOMEM;

	get_page(dummy_page);
	set_pages_uc(dummy_page, 1);
	dma_addr = pci_map_page(pdev->pdev, dummy_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	if (pci_dma_mapping_error(pdev->pdev, dma_addr))
		return -EINVAL;

	pte = dma_addr_to_pte_uc(pdev, dma_addr);
	printk("....dummy page (0x%llx, 0x%llx)\n", page_to_phys(dummy_page), dma_addr);
	dummy_addr = dma_addr;

	/* for debug purpose */
	memset(pfn_to_kaddr(page_to_pfn(dummy_page)), 0x77, PAGE_SIZE);

	/* clear all GM space, instead of only aperture */
	for (i = 0; i < gm_pages(pdev); i++)
		vgt_write_gtt(pdev, i, pte);

	vgt_dbg("content at 0x0: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x0));
	vgt_dbg("content at 0x64000: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x64000));
	vgt_dbg("content at 0x8064000: %lx\n", *(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x8064000));

	check_gtt(pdev);
	printk("vGT: allocate vGT aperture\n");
	/* Fill GTT range owned by vGT driver */
	index = GTT_INDEX(pdev, aperture_2_gm(pdev, pdev->rsvd_aperture_base));
	for (i = 0; i < VGT_APERTURE_PAGES; i++) {
		/* need a DMA flag? */
		page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!page) {
			vgt_dbg("vGT: Failed to create page for setup_gtt!\n");
			ret = -ENOMEM;
			goto err_out;
		}

		get_page(page);
		/* use wc instead! */
		set_pages_uc(page, 1);

		pages[i] = page;

		/* dom0 needs DMAR anyway */
		dma_addr = pci_map_page(pdev->pdev, page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
			printk(KERN_ERR "vGT: Failed to do pci_dma_mapping while handling %d 0x%llx\n", i, dma_addr);
			ret = -EINVAL;
			goto err_out;
		}

		pte = dma_addr_to_pte_uc(pdev, dma_addr);
		vgt_write_gtt(pdev, index + i, pte);

		if (!(i % 1024))
			printk("vGT: write GTT-%x phys: %llx, dma: %llx\n",
				index + i, page_to_phys(page), dma_addr);
	}

	check_gtt(pdev);
	/* any cache flush required here? */
	return 0;
err_out:
	printk("vGT: error in GTT initialization\n");
	for (i = 0; i < VGT_APERTURE_PAGES; i++)
		if (pages[i]) {
			put_page(pages[i]);
			__free_page(pages[i]);
		}

	return ret;
}

void free_gtt(struct pgt_device *pdev)
{
	int i;
	intel_gtt_clear_range(0,
		(phys_aperture_sz(pdev) - GTT_PAGE_SIZE)/PAGE_SIZE);
	for (i = 0; i < phys_aperture_pages(pdev); i++)
		if (pages[i]) {
			put_page(pages[i]);
			__free_page(pages[i]);
		}
}

static inline int ring_space(struct vgt_ring_buffer *ring)
{
	int space = (ring->head & RB_HEAD_OFF_MASK) - (ring->tail + 8);
	if (space < 0)
		space += ring->size;
	return space;
}

void vgt_ring_reset(struct vgt_ring_buffer *ring)
{
	ring->head = ring->tail = 0;
	ring->space = ring_space(ring);
}

void vgt_ring_init(struct pgt_device *pdev)
{
	struct vgt_ring_buffer *ring;

	pdev->ring_buffer = kzalloc(sizeof(struct vgt_ring_buffer), GFP_KERNEL);
	if (!pdev->ring_buffer) {
		printk(KERN_ERR "allocate vgt ring buffer failed!\n");
		return;
	}
	ring = pdev->ring_buffer;
	ring->pdev = pdev;
	ring->size = 4096;
	ring->offset = aperture_2_gm(pdev, rsvd_aperture_alloc(pdev, ring->size));
	ring->virtual_start = v_aperture(pdev, ring->offset);

	vgt_ring_reset(ring);
}

#define VGT_READ_CTL(pdev, id)	VGT_MMIO_READ(pdev, RB_CTL(pdev, id))
#define VGT_WRITE_CTL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_CTL(pdev, id), val)
#define VGT_POST_READ_CTL(pdev, id)	VGT_POST_READ(pdev, RB_CTL(pdev,id))

#define VGT_READ_HEAD(pdev, id)	VGT_MMIO_READ(pdev, RB_HEAD(pdev, id))
#define VGT_WRITE_HEAD(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, id), val)

#define VGT_READ_TAIL(pdev, id)	VGT_MMIO_READ(pdev, RB_TAIL(pdev, id))
#define VGT_WRITE_TAIL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, id), val)

#define VGT_READ_START(pdev, id) VGT_MMIO_READ(pdev, RB_START(pdev, id))
#define VGT_WRITE_START(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_START(pdev, id), val)

void vgt_ring_start(struct vgt_ring_buffer *ring)
{
	struct pgt_device *pdev = ring->pdev;
	int id = RING_BUFFER_RCS;
	u32 head;

	//ASSERT(ring->space == ring_space(ring));

	vgt_ring_reset(ring);

	/* execute our ring */
	VGT_WRITE_CTL(pdev, id, 0);
	VGT_WRITE_HEAD(pdev, id, 0);
	VGT_WRITE_TAIL(pdev, id, 0);

	head = VGT_READ_HEAD(pdev, id);
	if (head != 0) {
		VGT_WRITE_HEAD(pdev, id, 0);
	}

	VGT_WRITE_START(pdev, id, ring->offset);
	VGT_WRITE_CTL(pdev, id, ((ring->size - PAGE_SIZE) & 0x1FF000) | 1);
	VGT_POST_READ_CTL(pdev, id);

	wait_for(((VGT_READ_CTL(pdev, id) & 1) != 0 &&
			VGT_READ_START(pdev, id) == ring->offset &&
			(VGT_READ_HEAD(pdev, id) & RB_HEAD_OFF_MASK) == 0), 50);
	vgt_dbg("start vgt ring at 0x%x\n", ring->offset);
}

void vgt_ring_advance(struct vgt_ring_buffer *ring)
{
	int id = RING_BUFFER_RCS;

	ring->tail &= ring->size - 1;
	VGT_WRITE_TAIL(ring->pdev, id, ring->tail);
}

#if 0
static void ring_debug(struct vgt_device *vgt, int ring_id)
{
	printk("phead (%x), ptail(%x), pstart(%x), pctl(%x)\n",
		VGT_MMIO_READ(vgt->pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_TAIL(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_START(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_CTL(ring_id)));
}
#endif

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

int vgt_hvm_map_opregion (struct vgt_device *vgt, int map)
{
	uint32_t opregion;
	struct xen_hvm_vgt_map_mmio memmap;
	int rc;

	opregion = vgt->opregion_pa;

	printk("Direct map OpRegion 0x%x\n", opregion);

	memmap.first_gfn = opregion >> PAGE_SHIFT;
	memmap.first_mfn = opregion >> PAGE_SHIFT;
	memmap.nr_mfns = VGT_OPREGION_PAGES;
	memmap.map = map;
	memmap.domid = vgt->vm_id;
	rc = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);
	if (rc != 0)
		printk(KERN_ERR "vgt_hvm_map_opregion fail with %d!\n", rc);

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

int vgt_vmem_init(struct vgt_device *vgt)
{
	unsigned long i;

	/* Dom0 already has mapping for itself */
	if(vgt->vm_id == 0)
		return 0;

	vgt->vmem_sz = xen_get_vm_mem_sz(vgt->vm_id);
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
		if (vgt->vmem_vma[i] == NULL)
			vgt_warn("no mapping for vmem buck starting @ %ldMB\n", i<<(VMEM_BUCK_SHIFT-20));
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

	buck_index = gpa >> VMEM_BUCK_SHIFT;
	if (!vgt->vmem_vma || !vgt->vmem_vma[buck_index])
		return NULL;

	return (char*)(vgt->vmem_vma[buck_index]->addr) + (gpa & (VMEM_BUCK_SIZE -1));
}
