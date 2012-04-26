
/*
 * vGT core module
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
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
 * Copyright(c) 2011 Intel Corporation. All rights reserved.
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
#include <xen/vgt-parser.h>
#include "vgt_reg.h"

static inline int tail_to_ring_id(unsigned int tail_off)
{
	int i;

	for (i=0; i< MAX_ENGINES; i++) {
		if ( ring_mmio_base[i] == tail_off )
			return i;
	}
	printk("Wrong tail register %s\n", __FUNCTION__);
	ASSERT(0);
	return 0;
}

static void ring_debug(struct vgt_device *vgt, int ring_id)
{
	printk("phead (%x), ptail(%x), pstart(%x), pctl(%x)\n",
		VGT_MMIO_READ(vgt->pdev, RB_HEAD(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_TAIL(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_START(ring_id)),
		VGT_MMIO_READ(vgt->pdev, RB_CTL(ring_id)));
}

bool ring_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;

	ASSERT(bytes <= 4 && !(off & (bytes - 1)));
	//printk("vGT:ring_mmio_read (%x)\n", off);

	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_to_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	memcpy(p_data, (char *)vring + rel_off, bytes);
	//ring_debug(vgt, ring_id);
	return true;
}

bool ring_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;
	vgt_ringbuffer_t	*sring;
	vgt_reg_t	oval;

	ASSERT(bytes <= 4);
//	printk("vGT:ring_mmio_write (%x) with val (%x)\n", off, *((u32 *)p_data));
	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ASSERT(!(rel_off & (bytes - 1)));

	ring_id = tail_to_ring_id ( _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;
	sring = &vgt->rb[ring_id].sring;

	oval = *(vgt_reg_t *)((char *)vring + rel_off);
	memcpy((char *)vring + rel_off, p_data, bytes);

	switch (rel_off) {
	case RB_OFFSET_TAIL:
		sring->tail = vring->tail;
//		FIXME: temporarily not enable command parser for debuging purpose
//		vgt_scan_ring_buffer(vgt, ring_id);
		break;
	case RB_OFFSET_HEAD:
		//debug
		//vring->head |= 0x200000;
		sring->head = vring->head;
		break;
	case RB_OFFSET_START:
		sring->start = mmio_g2h_gmadr(vgt, off, vring->start);
		break;
	case RB_OFFSET_CTL:
		sring->ctl = vring->ctl;

		/* TODO: need lock with kthread */
		/* Do we need to wait for the completion of current slice? */
		if ( (oval & _RING_CTL_ENABLE) &&
			!(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: deactivate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_deactive (vgt->pdev, &vgt->list);
		}
		else if ( !(oval & _RING_CTL_ENABLE) &&
			(vring->ctl & _RING_CTL_ENABLE) ) {
			/* enabled */
			printk("vGT: activate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_active (vgt->pdev, &vgt->list);
		}
		if (vring->ctl & _RING_CTL_ENABLE) {
			/*
			 * Command scan policy:
			 * 	1) here: Challenge if that if guest modify
			 *   head/tail register to a new buffer and come back,
			 *   we don't know if a cmd is converted or not.
			 *	2) at submission time: Easier, but not
			 *   that efficient (GPU have to wait for the completion).
			 *	Start from 2, TO-REVISIT LATER!!!
			 */
//			vgt_scan_ring_buffer(vgt, ring_id);
		}
		break;
	default:
		ASSERT(0);
		break;
	}

	/* TODO: lock with kthread? */
	/*
	 * FIXME: Linux VM doesn't read head register directly. Instead it relies on
	 * automatic head reporting mechanism. Later with command parser, there's no
	 * problem since all commands are translated and filled by command parser. for
	 * now it's possible for dom0 to fill over than a full ring in a scheduled
	 * quantum
	 */
	if (vgt_ops->boot_time || is_current_render_owner(vgt))
		VGT_MMIO_WRITE(vgt->pdev, off, *(vgt_reg_t*)((char *)sring + rel_off));
	//ring_debug(vgt, ring_id);
	return true;
}

/*
 * Map the apperture space (BAR1) of vGT device for direct access.
 */
static int vgt_hvm_map_apperture (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0], *pcfg_space;
        uint64_t bar_s, bar_e;
        struct xen_hvm_vgt_map_mmio memmap;
        int r;

        cfg_space += VGT_REG_CFG_SPACE_BAR1;	/* APERTUR */
		if (VGT_GET_BITS(*cfg_space, 2, 1) == 2){
			/* 64 bits MMIO bar */
			bar_s = * (uint64_t *) cfg_space;
		} else {
			/* 32 bits MMIO bar */
			bar_s = * (uint32_t*) cfg_space;
		}

        bar_e = bar_s + vgt->state.bar_size[0] - 1;

        memmap.first_gfn = bar_s >> PAGE_SHIFT;
        pcfg_space = &vgt->pdev->initial_cfg_space[0];
        pcfg_space += VGT_REG_CFG_SPACE_BAR1;

        memmap.first_mfn = vgt_aperture_base(vgt) >> PAGE_SHIFT;
        memmap.nr_mfns = vgt->state.bar_size[0] >> PAGE_SHIFT ;

		memmap.map = map;

	r = HYPERVISOR_hvm_op(HVMOP_vgt_map_mmio, &memmap);

	if (r < 0)
	    printk(KERN_ERR "vgt_hvm_map_apperture %d!\n", r);
        return r;
}

/*
 * Zap the GTTMMIO bar area for vGT trap and emulation.
 */
static int vgt_hvm_set_trap_area(struct vgt_device *vgt)
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

bool vgt_emulate_cfg_read(struct vgt_device *vgt, unsigned int offset, void *p_data, int bytes)
{

	ASSERT ((offset + bytes) <= VGT_CFG_SPACE_SZ);
	memcpy(p_data, &vgt->state.cfg_space[offset], bytes);

	/* TODO: hooks */
	offset &= ~3;
	switch (offset) {
		case 0:
		case 4:
		break;
		default:
		break;
	}
	return true;
}

bool vgt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, int bytes)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint32_t *cfg_reg, old, new, size;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
			ASSERT((bytes == 4) && (off & 3) == 0);

			old = *cfg_reg & 0xf;
			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
			if ( new == 0xFFFFFFFF ) {
				/*
				 * Power-up software can determine how much address
				 * space the device requires by writing a value of
				 * all 1's to the register and then reading the value
				 * back. The device will return 0's in all don't-care
				 * address bits.
				 */
				new = new & ~(size-1);
				*cfg_reg = (new & ~0xf) | old;
			} else {
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 0);
				*cfg_reg = (new & ~0xf) | old;
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt);
			}
			break;

		case VGT_REG_CFG_SPACE_MSAC:
			printk("Guest write MSAC %x, %d: Not supported yet\n",
					*(char *)p_data, bytes);
			break;

		case VGT_REG_CFG_SPACE_BAR1+4:
		case VGT_REG_CFG_SPACE_BAR0+4:
		case VGT_REG_CFG_SPACE_BAR2+4:
		default:
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			break;
	}
	/*
	 * FIXME: assume most dmo0's cfg writes should be propogated to
	 * the real conf space. In the case where propogation is required
	 * but value needs be changed (sReg), do it here
	 */
	return true;
}

bool vgt_initialize_mmio_hooks()
{
    int i;

printk("mmio hooks initialized\n");
	/* ring registers */
	for (i=0; i < MAX_ENGINES; i++)
		if (!vgt_register_mmio_handler(ring_mmio_base[i],
			ring_mmio_base[i] + RB_REGS_SIZE - 1,
			ring_mmio_read, ring_mmio_write))
			return false;
	return true;
}

static int xen_get_nr_vcpu(int vm_id)
{
	/* get number of the VCPUs */
	/* TODO: add hypervisor specific implementation */
	return 1;
}

static int hvm_get_parameter_by_dom(domid_t domid, int idx, uint64_t *value)
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

static shared_iopage_t *map_hvm_iopage(struct vgt_device *vgt)
{
	uint64_t ioreq_pfn;
	int rc;

	rc =hvm_get_parameter_by_dom(vgt->vm_id, HVM_PARAM_IOREQ_PFN, &ioreq_pfn);
	if (rc < 0)
		return NULL;

	return xen_remap_domain_mfn_range_in_kernel(ioreq_pfn, 1, vgt->vm_id);
}

void vgt_hvm_write_cf8_cfc(struct vgt_device *vgt,
     unsigned int port, unsigned int bytes, unsigned long val)
{
    dprintk("vgt_hvm_write_cf8_cfc %x %d %lx\n", port, bytes, val);
    if ( (port & ~3) == 0xcf8 ) {
        ASSERT (bytes == 4);
        ASSERT ((port & 3) == 0);
        vgt->last_cf8 = (uint32_t) val;
    }
    else {
        ASSERT ( (vgt->last_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
             ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
        vgt_emulate_cfg_write (vgt,
             (vgt->last_cf8 & 0xfc) + (port & 3),
             &val, bytes);
    }
}

void vgt_hvm_read_cf8_cfc(struct vgt_device *vgt,
       unsigned int port, unsigned int bytes, unsigned long *val)
{
    unsigned long data;

    if ((port & ~3)== 0xcf8) {
        memcpy(val, (uint8_t*)&vgt->last_cf8 + (port & 3), bytes);
    }
    else {
//        ASSERT ( (vgt->last_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
            ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
        vgt_emulate_cfg_read(vgt, (vgt->last_cf8 & 0xfc) + (port & 3),
                     &data, bytes);
        memcpy(val, &data, bytes);
    }
    dprintk("VGT: vgt_cfg_read_emul port %x bytes %x got %lx\n",
               port, bytes, *val);
}

void _hvm_mmio_emulation(struct vgt_device *vgt, struct ioreq *req)
{
    int i, sign;
    char *cfg_space = &vgt->state.cfg_space[0];
    uint64_t  base = * (uint64_t *) (cfg_space + VGT_REG_CFG_SPACE_BAR0);
    uint64_t  tmp;

    sign = req->df ? -1 : 1;

    if (req->dir == IOREQ_READ) {
        /* MMIO READ */
        if (!req->data_is_ptr) {
            ASSERT (req->count == 1);

            dprintk("HVM_MMIO_read: target register (%lx).\n", req->addr);
            vgt_emulate_read(vgt,
                        req->addr - base,
                        &req->data,
                        req->size);
        }
        else {
	    ASSERT (req->addr + sign * req->count * req->size >= base);
	    ASSERT (req->addr + sign * req->count * req->size <
                            base + vgt->state.bar_size[0]);
            dprintk("HVM_MMIO_read: rep %d target memory %lx, slow!\n",
                         req->count, req->addr);
            for (i=0; i<req->count; i++) {
                tmp = 0;
                vgt_emulate_read(vgt,
                        req->addr + sign * i * req->size - base,
                        &tmp,
                        req->size);
                /*
                 *  TODO: Hypercall to write data (tmp) to
                 *      req->data + sign * i * req->size
                 *  We can use a hypercall or entire/cache foreign map.
                 *  Refer to IOCTL_PRIVCMD_MMAPBATCH_V2.
                 */
            }
        }
    }
    else {   /* MMIO Write */
        if (!req->data_is_ptr) {
            ASSERT (req->count == 1);

            dprintk("HVM_MMIO_write: target register (%lx).\n", req->addr);
            vgt_emulate_write(vgt,
                        req->addr - base,
                        &req->data,
                        req->size);
        }
        else {
	    ASSERT (req->addr + sign * req->count * req->size >= base);
	    ASSERT (req->addr + sign * req->count * req->size <
                            base + vgt->state.bar_size[0]);
            dprintk("HVM_MMIO_write: rep %d target memory %lx, slow!\n",
                         req->count, req->addr);

            for (i=0; i<req->count; i++) {
                tmp = 0;
                /*
                 *  TODO: Hypercall to read data (tmp) from
                 *      req->data + sign * i * req->size
                 *  We can use a hypercall or entire/cache foreign map.
                 *  Refer to IOCTL_PRIVCMD_MMAPBATCH_V2.
                 */
                vgt_emulate_write(vgt,
                        req->addr + sign * i * req->size - base,
                        &tmp,
                        req->size);
            }

        }
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
            dprintk("VGT: _hvm_pio_emulation read data_ptr %lx\n",
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
            dprintk("VGT: _hvm_pio_emulation write data_ptr %lx\n",
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

int vgt_hvm_info_init(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu, rc;

	info = kzalloc(sizeof(struct vgt_hvm_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	vgt->hvm_info = info;

	info->iopage = map_hvm_iopage(vgt);
	if (info->iopage == NULL){
		printk(KERN_ERR "Failed to map HVM I/O page for VM%d\n", vgt->vm_id);
		rc = -EFAULT;
		goto err;
	}

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

	/*TODO: unmap io page */

	if (!info->nr_vcpu || info->evtchn_irq == NULL)
		goto out1;

	for (vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if( info->evtchn_irq[vcpu] >= 0)
			unbind_from_irqhandler(info->evtchn_irq[vcpu], vgt);
	}

	kfree(info->evtchn_irq);

out1:
	kfree(info);

	return;
}
