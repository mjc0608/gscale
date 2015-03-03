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

bool inline is_execlist_mode(struct pgt_device *pdev, int ring_id)
{
	unsigned long ring_mode = RB_TAIL(pdev, ring_id) - 0x30 + 0x29c;

	return VGT_MMIO_READ(pdev, ring_mode) & _REGBIT_EXECLIST_ENABLE;
}

void show_debug(struct pgt_device *pdev)
{
	int i, cpu;

	printk("========vGT DEBUG INFO==========\n");
	for_each_online_cpu(cpu)
		printk("CPU[%d]: %s\n", cpu,
			per_cpu(in_vgt, cpu) ? "in vgt" : "out of vgt");
	printk("DE_RRMR: %x\n", VGT_MMIO_READ(pdev, _REG_DE_RRMR));

	for (i = 0; i < pdev->max_engines; i++) {
		printk("-----------ring-%d info-------------\n", i);
		show_ring_debug(pdev, i);
		show_ring_buffer(pdev, i, 16 * sizeof(vgt_reg_t));
	}
}

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
void common_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	printk("debug registers,reg maked with <*>"
		" may not apply to every ring):\n");
	printk("....RING_EIR: %08x\n", VGT_MMIO_READ(pdev, RING_EIR(ring_id)));
	printk("....RING_EMR: %08x\n", VGT_MMIO_READ(pdev, RING_EMR(ring_id)));
	printk("....RING_ESR: %08x\n", VGT_MMIO_READ(pdev, RING_ESR(ring_id)));

	if (ring_id)
		printk("....%08x*: %08x\n", RING_REG_2064(ring_id),
				VGT_MMIO_READ(pdev, RING_REG_2064(ring_id)));

	printk("....%08x: %08x\n", RING_REG_2068(ring_id),
		VGT_MMIO_READ(pdev, RING_REG_2068(ring_id)));
	printk("....ACTHD(active header): %08x\n",
			VGT_MMIO_READ(pdev, VGT_ACTHD(ring_id)));
	printk("....UHPTR(pending header): %08x\n",
			VGT_MMIO_READ(pdev, VGT_UHPTR(ring_id)));
	printk("....%08x: %08x\n", RING_REG_2078(ring_id),
		VGT_MMIO_READ(pdev, RING_REG_2078(ring_id)));

	if (!ring_id) {
		printk("....INSTPS* (parser state): %08x :\n",
				VGT_MMIO_READ(pdev, 0x2070));
		printk("....CSCMDOP* (instruction DWORD): %08x\n",
				VGT_MMIO_READ(pdev, 0x220C));
		printk("....CSCMDVLD* (command buffer valid): %08x\n",
				VGT_MMIO_READ(pdev, 0x2210));
	}

	printk("(informative)\n");
	printk("....INSTDONE_1(FYI): %08x\n",
			VGT_MMIO_READ(pdev, RING_REG_206C(ring_id)));
	if (!ring_id)
		printk("....INSTDONE_2*: %08x\n",
				VGT_MMIO_READ(pdev, 0x207C));
}

void legacy_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	int i;

	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt;
		if (pdev->device[i]) {
			vgt = pdev->device[i];
			if (vgt == current_render_owner(pdev))
				printk("VM%d(*):", vgt->vm_id);
			else
				printk("VM%d   :", vgt->vm_id);

			printk("head(%x), tail(%x), start(%x), ctl(%x), uhptr(%x)\n",
				vgt->rb[ring_id].sring.head,
				vgt->rb[ring_id].sring.tail,
				vgt->rb[ring_id].sring.start,
				vgt->rb[ring_id].sring.ctl,
				__vreg(vgt, VGT_UHPTR(ring_id)));
		}
	}

	common_show_ring_debug(pdev, ring_id);
}

void execlist_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	int i;

	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt;

		if (!pdev->device[i])
			continue;

		vgt = pdev->device[i];

		if (vgt == current_render_owner(pdev))
			printk("VM%d(*):", vgt->vm_id);
		else
			printk("VM%d   :", vgt->vm_id);
	}

	common_show_ring_debug(pdev, ring_id);
}

void show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	is_execlist_mode(pdev, ring_id) ?
		execlist_show_ring_debug(pdev, ring_id) :
		legacy_show_ring_debug(pdev, ring_id);
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
	} else if (IS_BDWGT3(pdev)) {
		SHOW_MODE(_REG_VCS2_MI_MODE);
		SHOW_MODE(_REG_VCS2_MFX_MODE_BDW);
		SHOW_MODE(_REG_VCS2_INSTPM);
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

static void show_batchbuffer(struct pgt_device *pdev, int ring_id, u64 addr,
	int bytes, int ppgtt)
{
	struct vgt_device_info *info = &pdev->device_info;
	int i;
	char *ip_va;
	u64 start;
	struct vgt_device *vgt = current_render_owner(pdev);
	uint32_t val;

	if (!vgt) {
		vgt_err("no render owner at hanging point\n");
		return;
	}

	if (addr < bytes) {
		bytes *= 2;
		start = 0;
	} else if (!ppgtt && (addr + bytes) >= info->max_gtt_gm_sz) {
		bytes *= 2;
		start = info->max_gtt_gm_sz - bytes;
	} else {
		start = addr - bytes;
		bytes *= 2;
	}

	printk("Batch buffer contents: \n");
	for (i = 0; i < bytes; i += 4) {
		struct vgt_mm *mm = ppgtt ? vgt->rb[ring_id].active_ppgtt_mm :
			vgt->gtt.ggtt_mm;

		ip_va = vgt_gma_to_va(mm, start + i);

		if (!(i % 32))
			printk("\n[%08llx]:", start + i);

		if (ip_va == NULL)
			printk(" %8s", "N/A");
		else {
			hypervisor_read_va(vgt, ip_va, &val, sizeof(val), 0);
			printk(" %08x", val);
		}
		if (start + i == addr)
			printk("(*)");
	}
	printk("\n");
}

/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
void common_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes,
	vgt_reg_t p_tail, vgt_reg_t p_head, vgt_reg_t p_start, vgt_reg_t p_ctl,
	unsigned long batch_head)
{
	char *p_contents;
	int i;
	struct vgt_device *vgt = current_render_owner(pdev);
	u32 *cur;
	u64 ring_len, off;

	printk("ring buffer(%d): head (0x%x) tail(0x%x), start(0x%x), "
			"ctl(0x%x)\n", ring_id, p_head, p_tail, p_start, p_ctl);
	printk("ring xxx:(%d), mi_mode idle:(%d)\n",
		VGT_MMIO_READ(pdev, pdev->ring_xxx[ring_id]) & (1 << pdev->ring_xxx_bit[ring_id]),
		VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & _REGBIT_MI_RINGS_IDLE);

	if (!(p_ctl & _RING_CTL_ENABLE)) {
		printk("<NO CONTENT>\n");
		return;
	}

	p_head &= RB_HEAD_OFF_MASK;
	ring_len = _RING_CTL_BUF_SIZE(p_ctl);
	p_contents = vgt_gma_to_va(vgt->gtt.ggtt_mm, p_start);

#define WRAP_OFF(off, size)			\
	({					\
		u64 val = off;			\
		if ((int64_t)val < 0)		\
			val += size;	\
		if (val >= size)		\
			val -= size;	\
		(val);				\
	})
	printk("p_contents(%lx)\n", (unsigned long)p_contents);
	/* length should be 4 bytes aligned */
	bytes &= ~0x3;
	for (i = -bytes; i < bytes; i += 4) {
		off = (p_head + i) % ring_len;
		off = WRAP_OFF(off, ring_len);
		/* print offset within the ring every 8 Dword */
		if (!((i + bytes) % 32))
			printk("\n[%08llx]:", off);
		printk(" %08x", *((u32*)(p_contents + off)));
		if (!i)
			printk("(*)");
	}
	printk("\n");

	if (IS_PREBDW(pdev))
		off = WRAP_OFF(((int32_t)p_head) - 8, ring_len);
	else
		off = WRAP_OFF(((int32_t)p_head) - 12, ring_len);

	cur = (u32*)(p_contents + off);
	if ((*cur & 0xfff00000) == 0x18800000 && vgt) {
		int ppgtt = (*cur & _CMDBIT_BB_START_IN_PPGTT);

		if (ppgtt && !test_bit(ring_id, &vgt->gtt.active_ppgtt_mm_bitmap)) {
			printk("Batch buffer in PPGTT with PPGTT disabled?\n");
			return;
		}

		printk("Hang in (%s) batch buffer (%x)\n",
			ppgtt ? "PPGTT" : "GTT",
			*(cur + 1));

		show_batchbuffer(pdev, ring_id,
			batch_head,
			bytes,
			ppgtt);
	}
}

void legacy_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	vgt_reg_t p_tail, p_head, p_start, p_ctl;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	p_start = VGT_MMIO_READ(pdev, RB_START(pdev, ring_id));
	p_ctl = VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id));

	common_show_ring_buffer(pdev, ring_id, bytes,
			p_tail, p_head, p_start, p_ctl,
			VGT_MMIO_READ(pdev, VGT_ACTHD(ring_id)));
}

unsigned long ring_id_2_current_desc_reg [] = {
	[RING_BUFFER_RCS] = 0x4400,
	[RING_BUFFER_VCS] = 0x4440,
	[RING_BUFFER_VCS2] = 0x4480,
	[RING_BUFFER_VECS] = 0x44c0,
	[RING_BUFFER_BCS] = 0x4500,
};

void execlist_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	struct vgt_device *vgt = current_render_owner(pdev);
	vgt_reg_t p_tail, p_head, p_start, p_ctl;
	unsigned long reg, val;
	u64 bb_head;
	u32 *p;

	printk("Execlist:\n");

	reg = RB_TAIL(pdev, ring_id) - 0x30 + _EL_OFFSET_STATUS;
	val = VGT_MMIO_READ(pdev, reg);

	printk("....Current execlist status: %lx.\n", val);

	val = VGT_MMIO_READ(pdev, ring_id_2_current_desc_reg[ring_id]);

	printk("....Current element descriptor(low): %lx.\n", val);

	val &= ~0xfff;

	printk("....LRCA: %lx.\n", val);

	if (!val)
		return;

	p = vgt_gma_to_va(vgt->gtt.ggtt_mm, val + 4096);
	if (!p)
		return;

	if ((ring_id == RING_BUFFER_RCS && p[1] != 0x1100101b)
		|| (ring_id != RING_BUFFER_RCS && p[1] != 0x11000015)) {
		printk("Invalid signature: %x.\n", p[1]);
		return;
	}

	p_head = *(p + 0x4 + 1);
	p_tail = *(p + 0x6 + 1);
	p_start = *(p + 0x8 + 1);
	p_ctl = *(p + 0xa + 1);

	bb_head = *(p + 0xc + 1) & 0xFFFF;
	bb_head <<= 32;
	bb_head |= *(p + 0xe + 1);
	reg = RB_TAIL(pdev, ring_id) - 0x30 + 0x140;

	common_show_ring_buffer(pdev, ring_id, bytes,
			p_tail, p_head, p_start, p_ctl,
			bb_head);
}

void show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	is_execlist_mode(pdev, ring_id) ?
		execlist_show_ring_buffer(pdev, ring_id, bytes) :
		legacy_show_ring_buffer(pdev, ring_id, bytes);
}

void show_interrupt_regs(struct pgt_device *pdev,
		struct seq_file *seq)
{
#define P(fmt, args...) \
	do { \
		if (!seq) \
			vgt_info(fmt, ##args); \
		else \
			seq_printf(seq, fmt, ##args); \
	}while(0)

	if (IS_PREBDW(pdev)) {
		P("vGT: DEISR is %x, DEIIR is %x, DEIMR is %x, DEIER is %x\n",
				VGT_MMIO_READ(pdev, _REG_DEISR),
				VGT_MMIO_READ(pdev, _REG_DEIIR),
				VGT_MMIO_READ(pdev, _REG_DEIMR),
				VGT_MMIO_READ(pdev, _REG_DEIER));
		P("vGT: GTISR is %x, GTIIR is %x, GTIMR is %x, GTIER is %x\n",
				VGT_MMIO_READ(pdev, _REG_GTISR),
				VGT_MMIO_READ(pdev, _REG_GTIIR),
				VGT_MMIO_READ(pdev, _REG_GTIMR),
				VGT_MMIO_READ(pdev, _REG_GTIER));
		P("vGT: PMISR is %x, PMIIR is %x, PMIMR is %x, PMIER is %x\n",
				VGT_MMIO_READ(pdev, _REG_PMISR),
				VGT_MMIO_READ(pdev, _REG_PMIIR),
				VGT_MMIO_READ(pdev, _REG_PMIMR),
				VGT_MMIO_READ(pdev, _REG_PMIER));
	} else {
		P("vGT: MASTER_IRQ: %x\n",
			VGT_MMIO_READ(pdev, _REG_MASTER_IRQ));

#define P_GROUP_WHICH(group, w) do {\
		P("vGT: "#group"|"#w" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			VGT_MMIO_READ(pdev, _REG_##group##_ISR(w)), \
			VGT_MMIO_READ(pdev, _REG_##group##_IIR(w)), \
			VGT_MMIO_READ(pdev, _REG_##group##_IMR(w)), \
			VGT_MMIO_READ(pdev, _REG_##group##_IER(w))); \
	}while(0)

#define P_GROUP(group) do {\
		P("vGT: "#group" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			VGT_MMIO_READ(pdev, _REG_##group##_ISR), \
			VGT_MMIO_READ(pdev, _REG_##group##_IIR), \
			VGT_MMIO_READ(pdev, _REG_##group##_IMR), \
			VGT_MMIO_READ(pdev, _REG_##group##_IER)); \
	}while(0)

		P_GROUP_WHICH(DE_PIPE, PIPE_A);
		P_GROUP_WHICH(DE_PIPE, PIPE_B);
		P_GROUP_WHICH(DE_PIPE, PIPE_C);

		P_GROUP_WHICH(GT, 0);
		P_GROUP_WHICH(GT, 1);
		P_GROUP_WHICH(GT, 2);
		P_GROUP_WHICH(GT, 3);

		P_GROUP(DE_PORT);
		P_GROUP(DE_MISC);
		P_GROUP(PCU);
	}

	P("vGT: SDEISR is %x, SDEIIR is %x, SDEIMR is %x, SDEIER is %x\n",
			VGT_MMIO_READ(pdev, _REG_SDEISR),
			VGT_MMIO_READ(pdev, _REG_SDEIIR),
			VGT_MMIO_READ(pdev, _REG_SDEIMR),
			VGT_MMIO_READ(pdev, _REG_SDEIER));

	P("vGT: RCS_IMR is %x, VCS_IMR is %x, BCS_IMR is %x\n",
			VGT_MMIO_READ(pdev, _REG_RCS_IMR),
			VGT_MMIO_READ(pdev, _REG_VCS_IMR),
			VGT_MMIO_READ(pdev, _REG_BCS_IMR));
	return;
#undef P
#undef P_GROUP
#undef P_GROUP_WHICH
}

void show_virtual_interrupt_regs(struct vgt_device *vgt,
		struct seq_file *seq)
{
#define P(fmt, args...) \
	do { \
		if (!seq) \
			vgt_info(fmt, ##args); \
		else \
			seq_printf(seq, fmt, ##args); \
	}while(0)

	if (IS_PREBDW(vgt->pdev)) {
		P("....vreg (deier: %x, deiir: %x, deimr: %x, deisr: %x)\n",
				__vreg(vgt, _REG_DEIER),
				__vreg(vgt, _REG_DEIIR),
				__vreg(vgt, _REG_DEIMR),
				__vreg(vgt, _REG_DEISR));
		P("....vreg (gtier: %x, gtiir: %x, gtimr: %x, gtisr: %x)\n",
				__vreg(vgt, _REG_GTIER),
				__vreg(vgt, _REG_GTIIR),
				__vreg(vgt, _REG_GTIMR),
				__vreg(vgt, _REG_GTISR));
		P("....vreg (pmier: %x, pmiir: %x, pmimr: %x, pmisr: %x)\n",
				__vreg(vgt, _REG_PMIER),
				__vreg(vgt, _REG_PMIIR),
				__vreg(vgt, _REG_PMIMR),
				__vreg(vgt, _REG_PMISR));
	} else {
		P("....vreg: MASTER_IRQ: %x\n",
				__vreg(vgt, _REG_MASTER_IRQ));

#define P_GROUP_WHICH(group, w) do {\
		P("....vreg "#group"|"#w" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			__vreg(vgt, _REG_##group##_ISR(w)), \
			__vreg(vgt, _REG_##group##_IIR(w)), \
			__vreg(vgt, _REG_##group##_IMR(w)), \
			__vreg(vgt, _REG_##group##_IER(w))); \
	}while(0)

#define P_GROUP(group) do {\
		P("....vreg "#group" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			__vreg(vgt, _REG_##group##_ISR), \
			__vreg(vgt, _REG_##group##_IIR), \
			__vreg(vgt, _REG_##group##_IMR), \
			__vreg(vgt, _REG_##group##_IER)); \
	}while(0)

		P_GROUP_WHICH(DE_PIPE, PIPE_A);
		P_GROUP_WHICH(DE_PIPE, PIPE_B);
		P_GROUP_WHICH(DE_PIPE, PIPE_C);

		P_GROUP_WHICH(GT, 0);
		P_GROUP_WHICH(GT, 1);
		P_GROUP_WHICH(GT, 2);
		P_GROUP_WHICH(GT, 3);

		P_GROUP(DE_PORT);
		P_GROUP(DE_MISC);
		P_GROUP(PCU);
	}

	P("....vreg (sdeier: %x, sdeiir: %x, sdeimr: %x, sdeisr: %x)\n",
			__vreg(vgt, _REG_SDEIER),
			__vreg(vgt, _REG_SDEIIR),
			__vreg(vgt, _REG_SDEIMR),
			__vreg(vgt, _REG_SDEISR));

	P("....vreg (rcs_imr: %x, vcs_imr: %x, bcs_imr: %x\n",
			__vreg(vgt, _REG_RCS_IMR),
			__vreg(vgt, _REG_VCS_IMR),
			__vreg(vgt, _REG_BCS_IMR));

	return;
#undef P
#undef P_GROUP
#undef P_GROUP_WHICH
}

uint32_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off)
{
	unsigned long bar_s, bar_size=0;
	struct pci_dev *dev = pdev->pdev;

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_s);
	pci_write_config_dword(dev, bar_off, 0xFFFFFFFF);

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_size);
	vgt_dbg(VGT_DBG_GENERIC, "read back bar_size %lx\n", bar_size);
	bar_size &= ~0xf; /* bit 4-31 */
	vgt_dbg(VGT_DBG_GENERIC, "read back bar_size1 %lx\n", bar_size);
	bar_size = 1 << find_first_bit(&bar_size, BITS_PER_LONG);
	vgt_dbg(VGT_DBG_GENERIC, "read back bar_size2 %lx\n", bar_size);

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

uint64_t vgt_get_gtt_size(struct pgt_device *pdev)
{
	struct pci_bus *bus = pdev->pbus;
	uint16_t gmch_ctrl;

	ASSERT(!bus->number);

	/* GTT size is within GMCH. */
	pci_bus_read_config_word(bus, 0, _REG_GMCH_CONTRL, &gmch_ctrl);

	if (IS_PREBDW(pdev)) {
		gmch_ctrl = (gmch_ctrl >> 8) & 3;
		switch (gmch_ctrl) {
			case 1:
			case 2:
				return gmch_ctrl << 20;
			default:
				vgt_err("Invalid GTT memory size: %d\n", gmch_ctrl);
				break;
		}
	} else {
		gmch_ctrl = (gmch_ctrl >> 6) & 3;
		if (gmch_ctrl)
			gmch_ctrl = 1 << gmch_ctrl;
		switch (gmch_ctrl) {
			case 2:
			case 4:
			case 8:
				return gmch_ctrl << 20;
			default:
				vgt_err("Invalid GTT memory size: %d\n", gmch_ctrl);
				break;
		}
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
		vgt_dbg(VGT_DBG_MEM, "GMADR: 0x08%x, GTT INDEX: %x, GTT VALUE: %x\n",
			addr[i], GTT_INDEX(pdev, addr[i]),
			vgt_read_gtt(pdev, GTT_INDEX(pdev, addr[i])));
}

static inline u64 dma_addr_to_pte_uc(struct pgt_device *pdev, dma_addr_t addr)
{
	u64 v;

	if (IS_BDWPLUS(pdev)) {
		v = addr & (0x7ffffff << 12);
	} else {
		if (IS_HSW(pdev)) {
			/* Haswell has new cache control bits */
			v = addr & ~0xfff;
			v |= (addr >> 28) & 0x7f0;
		} else {
			v = addr & ~0xfff;
			v |= (addr >> 28) & 0xff0;
			v |= (1 << 1); /* UC */
		}
	}
	v |= 1;
	return v;
}

void init_gm_space(struct pgt_device *pdev)
{
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	unsigned long i;

	/* clear all GM space, instead of only aperture */
	for (i = 0; i < gm_pages(pdev); i++)
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, i, false, NULL);

	vgt_dbg(VGT_DBG_MEM, "content at 0x0: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x0));
	vgt_dbg(VGT_DBG_MEM, "content at 0x64000: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x64000));
	vgt_dbg(VGT_DBG_MEM, "content at 0x8064000: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x8064000));
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

void vgt_clear_gtt(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	uint32_t index;
	uint32_t offset;
	uint32_t num_entries;

	index = vgt_visible_gm_base(vgt) >> PAGE_SHIFT;
	num_entries = vgt_aperture_sz(vgt) >> PAGE_SHIFT;
	for (offset = 0; offset < num_entries; offset++){
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, index+offset, false, NULL);
	}

	index = vgt_hidden_gm_base(vgt) >> PAGE_SHIFT;
	num_entries = vgt_hidden_gm_sz(vgt) >> PAGE_SHIFT;
	for (offset = 0; offset < num_entries; offset++){
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, index+offset, false, NULL);
	}
}

int setup_gtt(struct pgt_device *pdev)
{
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	struct page *dummy_page;
	struct page *(*pages)[VGT_APERTURE_PAGES];
	struct page *page;

	int i, ret, index;
	dma_addr_t dma_addr;
	gtt_entry_t e;
	u64 v;

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

	printk("....dummy page (0x%llx, 0x%llx)\n", page_to_phys(dummy_page), dma_addr);

	/* for debug purpose */
	memset(pfn_to_kaddr(page_to_pfn(dummy_page)), 0x77, PAGE_SIZE);

	v = dma_addr_to_pte_uc(pdev, dma_addr);
	gtt_init_entry(&e, GTT_TYPE_GGTT_PTE, pdev, v);
	pdev->dummy_gtt_entry = e;

	init_gm_space(pdev);

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
			vgt_dbg(VGT_DBG_MEM, "vGT: Failed to create page for setup_gtt!\n");
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

		ops->set_pfn(&e, dma_addr >> GTT_PAGE_SHIFT);
		ops->set_entry(NULL, &e, index + i, false, NULL);

		if (!(i % 1024))
			vgt_dbg(VGT_DBG_MEM, "vGT: write GTT-%x phys: %llx, dma: %llx\n",
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

void vgt_save_gtt_and_fence(struct pgt_device *pdev)
{
	int i;
	uint32_t *entry = pdev->saved_gtt;

	ASSERT(pdev->saved_gtt);
	vgt_info("Save GTT table...\n");
	for (i = 0; i < gm_pages(pdev); i++)
		*(entry + i) = vgt_read_gtt(pdev, i);

	for (i = 0; i < VGT_MAX_NUM_FENCES; i++)
		pdev->saved_fences[i] = VGT_MMIO_READ_BYTES(pdev,
			_REG_FENCE_0_LOW + 8 * i, 8);
}

void vgt_restore_gtt_and_fence(struct pgt_device *pdev)
{
	int i;
	uint32_t *entry = pdev->saved_gtt;

	ASSERT(pdev->saved_gtt);
	vgt_info("Restore GTT table...\n");
	for (i = 0; i < gm_pages(pdev); i++)
		vgt_write_gtt(pdev, i, *(entry + i));

	for (i = 0; i < VGT_MAX_NUM_FENCES; i++)
		VGT_MMIO_WRITE_BYTES(pdev,
			_REG_FENCE_0_LOW + 8 * i,
			pdev->saved_fences[i], 8);
}

static void _hex_dump(const char *data, size_t size)
{
	char buf[74];
	size_t offset;
	int line;

	for (line = 0; line < ((size + 0xF) / 0x10); line++) {
		int byte;

		memset(buf, ' ', sizeof(buf));
		buf[73] = '\0';
		offset = 0;

		offset += snprintf(buf + offset, 74 - offset, "%07x: ", line * 0x10);

		for (byte = 0; byte < 0x10; byte++) {
			if (!(byte & 0x1)) {
				offset += snprintf(buf + offset, 74 - offset, " ");
			}

			if (((line * 0x10) + byte) >= size) {
				offset += snprintf(buf + offset, 74 - offset, "  ");
			} else {
				offset += snprintf(buf + offset, 74 - offset, "%02x",
					       data[byte + (line * 0x10)] & 0xFF);
			}
		}
		
		offset += snprintf(buf + offset, 74 - offset, "  ");

		for (byte = 0; byte < 0x10; byte++) {
			if (data[byte + (line * 0x10)] >= 0x20 &&
			    data[byte + (line * 0x10)] <= 0x7E) {
				offset += snprintf(buf + offset, 74 - offset, "%c",
				    data[byte + (line * 0x10)] & 0xFF);
			} else {
				offset += snprintf(buf + offset, 74 - offset, ".");
			}
		}

		offset += snprintf(buf + offset, 74 - offset, "\n");
		printk(buf);
	}
}

void vgt_print_edid(struct vgt_edid_data_t *edid)
{
	if (edid && edid->data_valid) {
		_hex_dump(edid->edid_block, EDID_SIZE);
	} else {
		printk("EDID is not available!\n");
	}

	return;
}

void vgt_print_dpcd(struct vgt_dpcd_data *dpcd)
{
	if (dpcd && dpcd->data_valid) {
		_hex_dump(dpcd->data, DPCD_SIZE);
	} else {
		printk("DPCD is not available!\n");
	}
}

int vgt_hvm_map_aperture (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s;
	int r, nr_mfns;
	unsigned long first_gfn, first_mfn;

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

	first_gfn = (bar_s + vgt_aperture_offset(vgt)) >> PAGE_SHIFT;
	first_mfn = vgt_aperture_base(vgt) >> PAGE_SHIFT;
	if (!vgt->ballooning)
		nr_mfns = vgt->state.bar_size[1] >> PAGE_SHIFT;
	else
		nr_mfns = vgt_aperture_sz(vgt) >> PAGE_SHIFT;

	printk("%s: domid=%d gfn_s=0x%lx mfn_s=0x%lx nr_mfns=0x%x\n", map==0? "remove_map":"add_map",
		vgt->vm_id, first_gfn, first_mfn, nr_mfns);

	r = hypervisor_map_mfn_to_gpfn(vgt, first_gfn, first_mfn,
		nr_mfns, map);

	if (r != 0)
		printk(KERN_ERR "vgt_hvm_map_aperture fail with %d!\n", r);
	else
		vgt->state.bar_mapped[1] = map;

	return r;
}

/*
 * Zap the GTTMMIO bar area for vGT trap and emulation.
 */
int vgt_hvm_set_trap_area(struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s, bar_e;

	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

	cfg_space += VGT_REG_CFG_SPACE_BAR0;
	if (VGT_GET_BITS(*cfg_space, 2, 1) == 2) {
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	bar_s &= ~0xF; /* clear the LSB 4 bits */
	bar_e = bar_s + vgt->state.bar_size[0] - 1;

	return hypervisor_set_trap_area(vgt, bar_s, bar_e, 1);
}
