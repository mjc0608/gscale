/*
 * Various utility helpers.
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

#include <linux/module.h>

#include <linux/delay.h>

#include "vgt.h"
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
void show_debug(struct pgt_device *pdev, int ring_id)
{
	vgt_reg_t reg;

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
	printk("....ACTHD(active header): 0x%x\n", VGT_MMIO_READ(pdev, VGT_ACTHD(ring_id)));
	printk("....UHPTR(pending header): %x\n",
			VGT_MMIO_READ(pdev, VGT_UHPTR(ring_id)));
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

	if (IS_IVB(pdev) || IS_HSW(pdev)) {
		SHOW_MODE(_REG_RCS_GFX_MODE_IVB);
		SHOW_MODE(_REG_BCS_BLT_MODE_IVB);
		SHOW_MODE(_REG_VCS_MFX_MODE_IVB);
		SHOW_MODE(_REG_CACHE_MODE_0_IVB);
		SHOW_MODE(_REG_CACHE_MODE_1_IVB);
		SHOW_MODE(_REG_GT_MODE_IVB);
	} else if (IS_SNB(pdev)) {
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
		printk(" %08x", *((u32 *)(p_contents + i)));
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

		/*TODO: we do not handle batch buffer in PPGTT yet */
		if (*(cur - 1) & 0x100) {
			printk("Dumping batch buffer in PPGTT"
					" is not supported yet!\n");
			return;
		}

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
		vgt_dbg("GMADR: 0x08%x, GTT INDEX: %x, GTT VALUE: %x\n",
			addr[i], GTT_INDEX(pdev, addr[i]),
			vgt_read_gtt(pdev, GTT_INDEX(pdev, addr[i])));
}

u32 __inline dma_addr_to_pte_uc(struct pgt_device *pdev, dma_addr_t addr)
{
	u32 pte;

	if (IS_HSW(pdev)) {
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

static void vgt_free_gtt_pages(struct pgt_device *pdev)
{
	int i;
	struct page *dummy_page = pdev->dummy_page;
	struct page *(*pages)[VGT_APERTURE_PAGES] =
		pdev->rsvd_aperture_pages;

	if (pages != NULL) {
		for (i = 0; i < VGT_APERTURE_PAGES; i++) {
			if ((*pages)[i] == NULL)
				continue;
			put_page((*pages)[i]);
			__free_page((*pages)[i]);
		}
		kfree(pages);
	}

	if (dummy_page != NULL) {
		put_page(dummy_page);
		__free_page(dummy_page);
	}
}

int setup_gtt(struct pgt_device *pdev)
{
	struct page *dummy_page;
	struct page *(*pages)[VGT_APERTURE_PAGES];
	struct page *page;

	int i, ret, index;
	dma_addr_t dma_addr;
	u32 pte;

	check_gtt(pdev);

	printk("vGT: clear all GTT entries.\n");

	dummy_page = alloc_page(GFP_KERNEL | __GFP_ZERO | GFP_DMA32);
	if (!dummy_page)
		return -ENOMEM;
	pdev->dummy_page = dummy_page;

	get_page(dummy_page);
	set_pages_uc(dummy_page, 1);
	dma_addr = pci_map_page(pdev->pdev, dummy_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
		ret = -EINVAL;
		goto err_out;
	}

	pte = dma_addr_to_pte_uc(pdev, dma_addr);
	printk("....dummy page (0x%llx, 0x%llx)\n", page_to_phys(dummy_page), dma_addr);

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

	ASSERT(sizeof(*pages) == VGT_APERTURE_PAGES * sizeof(struct page*));
	if ((pages = kzalloc(sizeof(*pages), GFP_KERNEL)) == NULL) {
		ret = -ENOMEM;
		goto err_out;
	}
	pdev->rsvd_aperture_pages = pages;

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

		(*pages)[i] = page;

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
			vgt_dbg("vGT: write GTT-%x phys: %llx, dma: %llx\n",
				index + i, page_to_phys(page), dma_addr);
	}

	check_gtt(pdev);
	/* any cache flush required here? */
	return 0;
err_out:
	printk("vGT: error in GTT initialization\n");
	vgt_free_gtt_pages(pdev);

	return ret;
}

void free_gtt(struct pgt_device *pdev)
{
	intel_gtt_clear_range(0,
		(phys_aperture_sz(pdev) - GTT_PAGE_SIZE)/PAGE_SIZE);

	vgt_free_gtt_pages(pdev);
}

void vgt_print_dpcd(struct vgt_dpcd_data *dpcd)
{
	int idx;
	uint8_t *data = dpcd->data;

	for (idx = 0; idx < DPCD_SIZE; ++idx) {
		printk("0x%0x ", data[idx]);
		if (((idx + 1) & 0xf) == 0)
			printk("\n");
	}
}
