
/*
 * MMIO virtualization framework
 *
 * This file is provided under GPLv2 license.
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

#include <linux/acpi.h>

#include <xen/events.h>
#include <xen/xen-ops.h>
#include <xen/vgt.h>

#include "vgt.h"

DEFINE_HASHTABLE(vgt_mmio_table, VGT_HASH_BITS);

void vgt_add_mmio_entry(struct vgt_mmio_entry *e)
{
	hash_add(vgt_mmio_table, &e->hlist, e->base);
}

struct vgt_mmio_entry * vgt_find_mmio_entry(unsigned int base)
{
	struct vgt_mmio_entry *e;

	hash_for_each_possible(vgt_mmio_table, e, hlist, base) {
		if (base == e->base)
			return e;
	}
	return NULL;
}

void vgt_del_mmio_entry(unsigned int base)
{
	struct vgt_mmio_entry *e;

	if ((e = vgt_find_mmio_entry(base))) {
		hash_del(&e->hlist);
	}
}

void vgt_clear_mmio_table(void)
{
	int i;
	struct hlist_node *tmp;
	struct vgt_mmio_entry *e;

	hash_for_each_safe(vgt_mmio_table, i, tmp, e, hlist)
		kfree(e);

	hash_init(vgt_mmio_table);
}

void vgt_add_wp_page_entry(struct vgt_device *vgt, struct vgt_wp_page_entry *e)
{
	hash_add((vgt->wp_table), &e->hlist, e->pfn);
}

struct vgt_wp_page_entry * vgt_find_wp_page_entry(struct vgt_device *vgt, unsigned int pfn)
{
	struct vgt_wp_page_entry *e;

	hash_for_each_possible((vgt->wp_table), e, hlist, pfn) {
		if (pfn == e->pfn)
			return e;
	}
	return NULL;
}

void vgt_del_wp_page_entry(struct vgt_device *vgt, unsigned int pfn)
{
	struct vgt_wp_page_entry *e;

	if ((e = vgt_find_wp_page_entry(vgt, pfn))) {
		hash_del(&e->hlist);
		kfree(e);
	}
}

void vgt_clear_wp_table(struct vgt_device *vgt)
{
	int i;
	struct hlist_node *tmp;
	struct vgt_wp_page_entry *e;

	hash_for_each_safe((vgt->wp_table), i, tmp, e, hlist)
		kfree(e);

	hash_init((vgt->wp_table));
}

bool vgt_register_mmio_handler(unsigned int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write)
{
	int i, end;
	struct vgt_mmio_entry *mht;

	end = start + bytes -1;

	vgt_dbg("start=0x%x end=0x%x\n", start, end);

	ASSERT((start & 3) == 0);
	ASSERT(((end+1) & 3) == 0);

	for ( i = start; i < end; i += 4 ) {
		mht = kmalloc(sizeof(*mht), GFP_KERNEL);
		if (mht == NULL) {
			printk("Insufficient memory in %s\n", __FUNCTION__);
			return false;
		}
		mht->base = i;
		mht->read = read;
		mht->write = write;
		INIT_HLIST_NODE(&mht->hlist);
		vgt_add_mmio_entry(mht);
	}
	return true;
}

static inline unsigned long vgt_get_passthrough_reg(struct vgt_device *vgt,
		unsigned int reg)
{
	__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	return __vreg(vgt, reg);
}

static unsigned long vgt_get_reg(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_status(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg(vgt, reg);
	else
		return __vreg(vgt, reg);
}

static inline unsigned long vgt_get_passthrough_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	__sreg64(vgt, reg) = VGT_MMIO_READ_BYTES(vgt->pdev, reg, 8);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	__vreg(vgt, reg + 4) = mmio_h2g_gmadr(vgt, reg + 4, __sreg(vgt, reg + 4));
	return __vreg64(vgt, reg);
}
/*
 * for 64bit reg access, we split into two 32bit accesses since each part may
 * require address fix
 *
 * TODO: any side effect with the split? or instead install specific handler
 * for 64bit regs like fence?
 */
static unsigned long vgt_get_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_status(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg_64(vgt, reg);
	else
		return __vreg64(vgt, reg);
}

static void vgt_update_reg(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	if (reg == _REG_DSPASURF)
		vgt_dbg("%s: =======: write vReg(%x), sReg(%x)\n", __func__, __vreg(vgt, reg), __sreg(vgt, reg));
	if (reg_hw_access(vgt, reg))
		VGT_MMIO_WRITE(pdev, reg, __sreg(vgt, reg));
}

static void vgt_update_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	__sreg(vgt, reg + 4) = mmio_g2h_gmadr(vgt, reg + 4, __vreg(vgt, reg + 4));
	if (reg_hw_access(vgt, reg))
			VGT_MMIO_WRITE_BYTES(pdev, reg, __sreg64(vgt, reg), 8);
}

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_reg(vgt, reg);
	} else {
		wvalue = vgt_get_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

bool default_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	memcpy((char *)vgt->state.vReg + offset,
			p_data, bytes);

	offset &= ~(bytes - 1);
	if (bytes <= 4)
		vgt_update_reg(vgt, offset);
	else
		vgt_update_reg_64(vgt, offset);

	return true;
}

bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_passthrough_reg(vgt, reg);
	} else {
		wvalue = vgt_get_passthrough_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

#define PCI_BAR_ADDR_MASK (~0xFUL)  /* 4 LSB bits are not address */

static inline unsigned int vgt_pa_to_mmio_offset(struct vgt_device *vgt, unsigned long pa)
{
	return (vgt->vm_id == 0)?
		pa - vgt->pdev->gttmmio_base :
		pa - ( (*(uint64_t*)(vgt->state.cfg_space + VGT_REG_CFG_SPACE_BAR0))
				& PCI_BAR_ADDR_MASK );
}

/*
 * Emulate the VGT MMIO register read ops.
 * Return : true/false
 * */
bool vgt_emulate_read(struct vgt_device *vgt, unsigned int pa, void *p_data,int bytes)
{
	struct vgt_mmio_entry *mht;
	struct pgt_device *pdev = vgt->pdev;
	unsigned int offset;
	unsigned long flags;

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	ASSERT (bytes <= 8);
//	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);
	if (!VGT_REG_IS_ALIGNED(offset, bytes)){
		printk("unaligned reg %x, bytes=%d\n", offset, bytes);
		offset = VGT_REG_ALIGN(offset, bytes);
	}

	if (bytes > 4)
		vgt_dbg("vGT: capture >4 bytes read to %x\n", offset);

	spin_lock_irqsave(&pdev->lock, flags);

	raise_ctx_sched(vgt);

	if (reg_is_gtt(pdev, offset)) {
		gtt_mmio_read(vgt, offset, p_data, bytes);
		spin_unlock_irqrestore(&pdev->lock, flags);
		return true;
	}

	ASSERT (reg_is_mmio(pdev, offset + bytes));

	mht = vgt_find_mmio_entry(offset);
	if ( mht && mht->read )
		mht->read(vgt, offset, p_data, bytes);
	else {
		default_mmio_read(vgt, offset, p_data, bytes);
	}

	if (!reg_is_tracked(pdev, offset)) {
		vgt_warn("vGT: untracked MMIO: vm_id(%d), offset=0x%x,"
			"len=%d, val=0x%x!!!\n",
			vgt->vm_id,	offset, bytes, *(u32 *)p_data);

		WARN_ON(vgt->vm_id == 0); /* The call stack is meaningless for HVM */
	}

	reg_set_accessed(pdev, offset);

	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

/*
 * Emulate the VGT MMIO register write ops.
 * Return : true/false
 * */
bool vgt_emulate_write(struct vgt_device *vgt, unsigned int pa,
	void *p_data, int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_mmio_entry *mht;
	unsigned int offset;
	unsigned long flags;
	vgt_reg_t old_vreg=0, old_sreg=0;

	/* XXX PPGTT PTE WP comes here too. */
	if (pdev->enable_ppgtt && vgt->vm_id != 0 && vgt->ppgtt_initialized) {
		struct vgt_wp_page_entry *wp;
		wp = vgt_find_wp_page_entry(vgt, pa >> PAGE_SHIFT);
		if (wp) {
			/* XXX lock? */
			vgt_ppgtt_handle_pte_wp(vgt, wp, pa, p_data, bytes);
			return true;
		}
	}

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	/* at least FENCE registers are accessed in 8 bytes */
	ASSERT (bytes <= 8);
	ASSERT ((offset & (bytes - 1)) + bytes <= bytes);

	if (bytes > 4)
		vgt_dbg("vGT: capture >4 bytes write to %x with val (%lx)\n", offset, *(unsigned long*)p_data);
/*
	if (reg_rdonly(pdev, offset & (~(bytes - 1)))) {
		printk("vGT: captured write to read-only reg (%x)\n", offset);
		return true;
	}
*/

	spin_lock_irqsave(&pdev->lock, flags);

	raise_ctx_sched(vgt);

	if (reg_is_gtt(pdev, offset)) {
		gtt_mmio_write(vgt, offset, p_data, bytes);
		spin_unlock_irqrestore(&pdev->lock, flags);
		return true;
	}

	ASSERT (reg_is_mmio(pdev, offset + bytes));

	if (reg_mode_ctl(pdev, offset)) {
		old_vreg = __vreg(vgt, offset);
		old_sreg = __sreg(vgt, offset);
	}

	if (!reg_is_tracked(pdev, offset)) {
		vgt_warn("vGT: untracked MMIO : vm_id(%d), offset=0x%x,"
			"len=%d, val=0x%x!!!\n",
			vgt->vm_id,	offset, bytes, *(u32 *)p_data);

		WARN_ON(vgt->vm_id == 0); /* The call stack is meaningless for HVM */
	}

	mht = vgt_find_mmio_entry(offset);
	if ( mht && mht->write )
		mht->write(vgt, offset, p_data, bytes);
	else {
		default_mmio_write(vgt, offset, p_data, bytes);
	}


	if (offset == _REG_DSPASURF || offset == _REG_DSPBSURF) {
		vgt_dbg("vGT(%d): write to surface base (%x) with (%x), pReg(%x)\n",
			vgt->vgt_id, offset, __vreg(vgt, offset),
			VGT_MMIO_READ(pdev, offset));
		/* update live reg as vm may wait on the update */
		if (!reg_hw_access(vgt, offset)) {
			__vreg(vgt, offset + 0x10) = __vreg(vgt, offset);
			__sreg(vgt, offset + 0x10) = __sreg(vgt, offset);
		}
	}

	/* higher 16bits of mode ctl regs are mask bits for change */
	if (reg_mode_ctl(pdev, offset)) {
		u32 mask = __vreg(vgt, offset) >> 16;

		vgt_dbg("old mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset),
			reg_aux_mode_mask(pdev, offset));
		/*
		 * share the global mask among VMs, since having one VM touch a bit
		 * not changed by another VM should be still saved/restored later
		 */
		reg_aux_mode_mask(pdev, offset) |= mask << 16;
		__vreg(vgt, offset) = (old_vreg & ~mask) | (__vreg(vgt, offset) & mask);
		__sreg(vgt, offset) = (old_sreg & ~mask) | (__sreg(vgt, offset) & mask);
		vgt_dbg("new mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset),
			reg_aux_mode_mask(pdev, offset));
		//show_mode_settings(vgt->pdev);
	}

	if (offset == _REG_RCS_UHPTR)
		printk("vGT: write to UHPTR (%x,%x)\n", __vreg(vgt, offset), __sreg(vgt, offset));

	reg_set_accessed(pdev, offset);
	spin_unlock_irqrestore(&pdev->lock, flags);
	return true;
}

u64 mmio_rcnt=0;
u64 mmio_wcnt=0;
u64 mmio_rcycles=0;
u64 mmio_wcycles=0;

void _hvm_mmio_emulation(struct vgt_device *vgt, struct ioreq *req)
{
	int i, sign;
	void *gva;
	unsigned long gpa;
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t base = * (uint64_t *) (cfg_space + VGT_REG_CFG_SPACE_BAR0);
	uint64_t tmp;
	cycles_t t0, t1;

	sign = req->df ? -1 : 1;

	if (req->dir == IOREQ_READ) {
		t0 = get_cycles();
		mmio_rcnt++;
		/* MMIO READ */
		if (!req->data_is_ptr) {
			ASSERT (req->count == 1);

			//vgt_dbg("HVM_MMIO_read: target register (%lx).\n",
			//	(unsigned long)req->addr);
			vgt_emulate_read(vgt, req->addr, &req->data, req->size);
		}
		else {
			ASSERT (req->addr + sign * req->count * req->size >= base);
			ASSERT (req->addr + sign * req->count * req->size <
				base + vgt->state.bar_size[0]);
			//vgt_dbg("HVM_MMIO_read: rep %d target memory %lx, slow!\n",
			//	req->count, (unsigned long)req->addr);

			for (i = 0; i < req->count; i++) {
				vgt_emulate_read(vgt, req->addr + sign * i * req->size,
					&tmp, req->size);
				gpa = req->data + sign * i * req->size;
				gva = vgt_vmem_gpa_2_va(vgt, gpa);
				// XXX: FIXME: on the SNB laptop, writing tmp to gva can
				//cause bug 119. So let's do the writing only on HSW for now.
				if (gva != NULL && vgt->pdev->is_haswell)
					memcpy(gva, &tmp, req->size);
				else
					vgt_dbg("vGT: can not write gpa = 0x%lx!!!\n", gpa);
			}
		}
		t1 = get_cycles();
		t1 -= t0;
		mmio_rcycles += (u64) t1;
	}
	else { /* MMIO Write */
		t0 = get_cycles();
		mmio_wcnt++;
		if (!req->data_is_ptr) {
			ASSERT (req->count == 1);
			//vgt_dbg("HVM_MMIO_write: target register (%lx).\n", (unsigned long)req->addr);
			vgt_emulate_write(vgt, req->addr, &req->data, req->size);
		}
		else {
			ASSERT (req->addr + sign * req->count * req->size >= base);
			ASSERT (req->addr + sign * req->count * req->size <
				base + vgt->state.bar_size[0]);
			//vgt_dbg("HVM_MMIO_write: rep %d target memory %lx, slow!\n",
			//	req->count, (unsigned long)req->addr);

			for (i = 0; i < req->count; i++) {
				gpa = req->data + sign * i * req->size;
				gva = vgt_vmem_gpa_2_va(vgt, gpa);
				if (gva != NULL)
					memcpy(&tmp, gva, req->size);
				else {
					tmp = 0;
					vgt_dbg("vGT: can not read gpa = 0x%lx!!!\n", gpa);
				}
				vgt_emulate_write(vgt, req->addr + sign * i * req->size, &tmp, req->size);
			}
		}
		t1 = get_cycles();
		t1 -= t0;
		mmio_wcycles += (u64) t1;
	}
}

void _hvm_pio_emulation(struct vgt_device *vgt, struct ioreq *ioreq)
{
	int sign;
	//char *pdata;

	sign = ioreq->df ? -1 : 1;

	if (ioreq->dir == IOREQ_READ) {
		/* PIO READ */
		if (!ioreq->data_is_ptr) {
			vgt_hvm_read_cf8_cfc(vgt,
				ioreq->addr,
				ioreq->size,
				(unsigned long*) &ioreq->data);
		}
		else {
			vgt_dbg("VGT: _hvm_pio_emulation read data_ptr %lx\n",
			(long)ioreq->data);
			/*
			 * The data pointer of emulation is guest physical address
			 * so far, which is godo to Qemu emulation, but hard for
			 * vGT driver which doesn't know gpn_2_mfn translation.
			 * We may ask hypervisor to use mfn for vGT driver.
			 * We keep assert here to see if guest really use it.
			 */
			ASSERT(0);
#if 0
			pdata = (char *)ioreq->data;
			for (i=0; i < ioreq->count; i++) {
				vgt_hvm_read_cf8_cfc(vgt,
					ioreq->addr,
					ioreq->size,
					(unsigned long *)pdata);
				pdata += ioreq->size * sign;
			}
#endif
		}
	}
	else {
		/* PIO WRITE */
		if (!ioreq->data_is_ptr) {
			vgt_hvm_write_cf8_cfc(vgt,
				ioreq->addr,
				ioreq->size,
				(unsigned long) ioreq->data);
		}
		else {
			vgt_dbg("VGT: _hvm_pio_emulation write data_ptr %lx\n",
			(long)ioreq->data);
			/*
			 * The data pointer of emulation is guest physical address
			 * so far, which is godo to Qemu emulation, but hard for
			 * vGT driver which doesn't know gpn_2_mfn translation.
			 * We may ask hypervisor to use mfn for vGT driver.
			 * We keep assert here to see if guest really use it.
			 */
			ASSERT(0);
#if 0
			pdata = (char *)ioreq->data;

			for (i=0; i < ioreq->count; i++) {
				vgt_hvm_write_cf8_cfc(vgt,
					ioreq->addr,
					ioreq->size, *(unsigned long *)pdata);
				pdata += ioreq->size * sign;
			}
#endif
		}
	}
}

static int vgt_hvm_do_ioreq(struct vgt_device *vgt, struct ioreq *ioreq)
{
	switch (ioreq->type) {
		case IOREQ_TYPE_PIO:	/* PIO */
			if ((ioreq->addr & ~7) != 0xcf8)
				printk(KERN_ERR "vGT: Unexpected PIO %lx emulation\n",
					(long) ioreq->addr);
			else
				_hvm_pio_emulation(vgt, ioreq);
			break;
		case IOREQ_TYPE_COPY:	/* MMIO */
			_hvm_mmio_emulation(vgt, ioreq);
			break;
		default:
			printk(KERN_ERR "vGT: Unknown ioreq type %x\n", ioreq->type);
			break;
	}
	return 0;
}

static irqreturn_t vgt_hvm_io_req_handler(int irq, void* dev)
{
	struct vgt_device *vgt;
	struct vgt_hvm_info *info;
	int vcpu;
	struct ioreq *ioreq;

	vgt = (struct vgt_device *)dev;
	info = vgt->hvm_info;

	for(vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if(info->evtchn_irq[vcpu] == irq)
			break;
	}
	if (vcpu == info->nr_vcpu){
		/*opps, irq is not the registered one*/
		return IRQ_NONE;
	}

	ioreq = vgt_get_hvm_ioreq(vgt, vcpu);

	vgt_hvm_do_ioreq(vgt, ioreq);

	ioreq->state = STATE_IORESP_READY;

	notify_remote_via_irq(irq);

	return IRQ_HANDLED;
}

static void vgt_hvm_opregion_init(struct vgt_device *vgt)
{
	uint8_t* buf;
	vgt->opregion_pa = *(uint32_t*)(vgt->state.cfg_space + VGT_REG_CFG_OPREGION );
	vgt->opregion_va = acpi_os_ioremap(vgt->opregion_pa,
				VGT_OPREGION_PAGES);
	ASSERT(vgt->opregion_va);

	/* for unknown reason, the value in LID field is incorrect
	 * which block the windows guest, so workaround it by force
	 * setting it to "OPEN"
	 */

	buf = (uint8_t*)vgt->opregion_va;
	buf[VGT_OPREGION_REG_CLID] = 0x3;
}

int vgt_hvm_io_init(struct vgt_device *vgt)
{
	vgt_hvm_opregion_init(vgt);
	return vgt_hvm_map_opregion(vgt, 1);
}

int vgt_hvm_info_init(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu, rc;

	info = kzalloc(sizeof(struct vgt_hvm_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	vgt->hvm_info = info;

	info->iopage_vma = map_hvm_iopage(vgt);
	if (info->iopage_vma == NULL) {
		printk(KERN_ERR "Failed to map HVM I/O page for VM%d\n", vgt->vm_id);
		rc = -EFAULT;
		goto err;
	}
	info->iopage = info->iopage_vma->addr;

	info->nr_vcpu = xen_get_nr_vcpu(vgt->vm_id);
	ASSERT(info->nr_vcpu > 0);

	info->evtchn_irq = kmalloc(info->nr_vcpu * sizeof(int), GFP_KERNEL);
	if (info->evtchn_irq == NULL){
		rc = -ENOMEM;
		goto err;
	}
	for( vcpu = 0; vcpu < info->nr_vcpu; vcpu++ )
		info->evtchn_irq[vcpu] = -1;

	for( vcpu = 0; vcpu < info->nr_vcpu; vcpu++ ){
		rc = bind_interdomain_evtchn_to_irqhandler( vgt->vm_id,
				info->iopage->vcpu_ioreq[vcpu].vgt_eport,
				vgt_hvm_io_req_handler, 0,
				"vgt", vgt );
		if ( rc < 0 ){
			printk(KERN_ERR "Failed to bind event channle for vgt HVM IO handler, rc=%d\n", rc);
			goto err;
		}
		info->evtchn_irq[vcpu] = rc;
	}

	return 0;

err:
	vgt_hvm_info_deinit(vgt);
	return rc;
}

void vgt_hvm_info_deinit(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu;

	info = vgt->hvm_info;

	if (info == NULL)
		return;

	if (vgt->opregion_va)
		iounmap(vgt->opregion_va);

	if (!info->nr_vcpu || info->evtchn_irq == NULL)
		goto out1;

	for (vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if( info->evtchn_irq[vcpu] >= 0)
			unbind_from_irqhandler(info->evtchn_irq[vcpu], vgt);
	}

	if (info->iopage_vma != NULL)
		xen_unmap_domain_mfn_range_in_kernel(info->iopage_vma, 1, vgt->vm_id);

	kfree(info->evtchn_irq);

out1:
	kfree(info);

	return;
}


static void vgt_set_reg_attr(struct pgt_device *pdev,
	u32 reg, reg_attr_t *attr, bool track)
{
	/* ensure one entry per reg */
	ASSERT_NUM(!reg_is_tracked(pdev, reg) || !track, reg);

	if (reg_is_tracked(pdev, reg)) {
		if (track)
			printk("vGT: init a tracked reg (%x)!!!\n", reg);

		return;
	}

	reg_set_owner(pdev, reg, attr->flags & VGT_REG_OWNER);
	if (attr->flags & VGT_REG_WORKAROUND)
		reg_set_workaround(pdev, reg);
	if (attr->flags & VGT_REG_ADDR_FIX ) {
		if (!attr->addr_mask)
			printk("vGT: ZERO addr fix mask for %x\n", reg);
		reg_set_addr_fix(pdev, reg, attr->addr_mask);
	}
	if (attr->flags & VGT_REG_MODE_CTL)
		reg_set_mode_ctl(pdev, reg);
	if (attr->flags & VGT_REG_VIRT)
		reg_set_virt(pdev, reg);
	if (attr->flags & VGT_REG_HW_STATUS)
		reg_set_hw_status(pdev, reg);
	if (attr->flags & VGT_REG_BOOTTIME)
		reg_set_boottime(pdev, reg);

	/* last mark the reg as tracked */
	if (track)
		reg_set_tracked(pdev, reg);
}

static void vgt_initialize_reg_attr(struct pgt_device *pdev,
	reg_attr_t *info, int num, bool track)
{
	int i, cnt = 0, tot = 0;
	u32 reg;
	reg_attr_t *attr;

	attr = info;
	for (i = 0; i < num; i++, attr++) {
		if (!vgt_match_device_attr(pdev, attr))
			continue;

		cnt++;
		if (track)
			printk("reg(%x): size(%x), device(%d), flags(%x), mask(%x), read(%llx), write(%llx)\n",
				attr->reg, attr->size, attr->device,
				attr->flags,
				attr->addr_mask,
				(u64)attr->read, (u64)attr->write);
		for (reg = attr->reg;
			reg < attr->reg + attr->size;
			reg += REG_SIZE) {
			vgt_set_reg_attr(pdev, reg, attr, track);
			tot++;
		}

		if (attr->read || attr->write)
			vgt_register_mmio_handler(attr->reg, attr->size,
				attr->read, attr->write);
	}
	printk("%d listed, %d used\n", num, cnt);
	printk("total %d registers tracked\n", tot);
}

void vgt_setup_reg_info(struct pgt_device *pdev)
{
	printk("vGT: setup tracked reg info\n");
	vgt_initialize_reg_attr(pdev, vgt_base_reg_info,
		vgt_get_base_reg_num(), true);
}

static void __vgt_initial_mmio_space (struct pgt_device *pdev,
					reg_attr_t *info, int num)
{
	int i, j;
	reg_attr_t *attr;

	attr = info;

	for (i = 0; i < num; i++, attr++) {
		if (!vgt_match_device_attr(pdev, attr))
			continue;

		for (j = 0; j < attr->size; j += 4) {
			pdev->initial_mmio_state[REG_INDEX(attr->reg + j)] =
				readl(pdev->gttmmio_base_va + attr->reg + j);
		}
	}

}

bool vgt_initial_mmio_setup (struct pgt_device *pdev)
{
	pdev->initial_mmio_state = vzalloc(pdev->mmio_size);
	if (!pdev->initial_mmio_state) {
		printk("vGT: failed to allocate initial_mmio_state\n");
		return false;
	}

	__vgt_initial_mmio_space(pdev, vgt_base_reg_info, vgt_get_base_reg_num());

	return true;
}

void state_vreg_init(struct vgt_device *vgt)
{
	memcpy (vgt->state.vReg, vgt->pdev->initial_mmio_state,
		vgt->pdev->mmio_size);

	/* set the bit 0:2 (Thread C-State) to C0
	 * TODO: consider other bit 3:31
	 */
	__vreg(vgt, _REG_GT_THREAD_STATUS) = 0;

	/* set the bit 0:2(Core C-State ) to C0 */
	__vreg(vgt, _REG_GT_CORE_STATUS) = 0;

	/*TODO: init other regs that need different value from pdev */
}

/* TODO: figure out any security holes by giving the whole initial state */
void state_sreg_init(struct vgt_device *vgt)
{
	vgt_reg_t *sreg;

	sreg = vgt->state.sReg;
	memcpy (sreg, vgt->pdev->initial_mmio_state, vgt->pdev->mmio_size);

	/*
	 * Do we really need address fix for initial state? Any address information
	 * there is meaningless to a VM, unless that address is related to allocated
	 * GM space to the VM. Translate a host address '0' to a guest GM address
	 * is just a joke.
	 */
#if 0
	/* FIXME: add off in addr table to avoid checking all regs */
	for (i = 0; i < vgt->pdev->reg_num; i++) {
		if (reg_addr_fix(vgt->pdev, i * REG_SIZE)) {
			__sreg(vgt, i) = mmio_g2h_gmadr(vgt, i, __vreg(vgt, i));
			vgt_dbg("vGT: address fix for reg (%x): (%x->%x)\n",
				i, __vreg(vgt, i), __sreg(vgt, i));
		}
	}
#endif
}
