/*
 * vGT instruction emulator
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <asm/bitops.h>
#include <asm/ptrace.h>
#include <asm/traps.h>
#include <asm/xen/interface.h>
#include <asm/xen/x86_emulate.h>
#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>
#include <asm/desc.h>
#include <xen/interface/vcpu.h>
#include <xen/vgt.h>
#include <linux/init.h>
#include <linux/page-flags.h>

vgt_ops_t *vgt_ops = NULL;
#define SINGLE_VM_DEBUG

#define MAX_VGT_DEVICES     16
unsigned long   vgt_device_bitmap = 0;
struct {
    int dom_id;
    struct vgt_device *vgt;
} vgt_devices[MAX_VGT_DEVICES];      /* Dom0 is always in ..[0] */

int find_free_device_id(void)
{
    int index;

    do {
        index = ffz (vgt_device_bitmap);
        if (index >= MAX_VGT_DEVICES)
            return -1;
    } while (test_and_set_bit(index, &vgt_device_bitmap) != 0);
    return index;
}

static inline void free_device_id(int id)
{
    clear_bit(id, &vgt_device_bitmap);
}

/*
 * Return ID of registered vgt device.
 *  >= 0: successful
 *  -1: failed.
 */
int xen_register_vgt_device(int dom_id, struct vgt_device *vgt)
{
    int dev_id = find_free_device_id();
    bool ret = -1;

printk("Eddie: xen_register_vgt_device %d %p\n", dom_id, vgt);
    if ((dev_id < MAX_VGT_DEVICES) && (dev_id >= 0)) {
        vgt_devices[dev_id].dom_id = dom_id;
        vgt_devices[dev_id].vgt = vgt;

        if (dev_id == 1) {
            /* TODO: switch dom0 vgt from pass thru to virt. */
            vgt_ops->boot_time = 0;
        }
        ret = dev_id;
    }

    if (!vgt_ops->initialized)
	vgt_ops->initialized = 1;
    return ret;
}

void xen_deregister_vgt_device(struct vgt_device *vgt)
{
    int i;

printk("Eddie: xen_deregister_vgt_device %p\n", vgt);
    for (i=0; i < MAX_VGT_DEVICES; i++)
         if (vgt_devices[i].vgt == vgt) {
            vgt_devices[i].vgt = NULL;
            vgt_devices[i].dom_id = 0;
            free_device_id(i);
            return;
        };

    printk("xen_deregister_vgt_device failed\n");
}

#define ASSERT(x)						\
	do {							\
		if (!(x)) {					\
			printk("ASSERT in %s:%d...\n",		\
				__FUNCTION__, __LINE__);	\
			while (1); 				\
		};						\
	} while (0)
#define UNSUPPORTED(name)					\
	do {								\
		printk("VGT: Unsupported emulation of %s, gip=%lx\n",	\
			name, (unsigned long)ctxt->regs->rip); 		\
		return 0;						\
	} while (1)

//#define VGT_DEBUG
#ifdef VGT_DEBUG
#define dprintk(fmt, a...)	\
	printk("vGT:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##a)
#else
#define dprintk(fmt, a...)
#endif

struct vcpu_io_forwarding_request trap_req;

static inline uint16_t get_selector(
	enum x86_segment seg, struct cpu_user_regs *regs)
{
    uint16_t selector = 0;

    switch (seg)
    {
        case x86_seg_cs:
            selector = regs->cs;
            break;
        case x86_seg_ss:
            selector = regs->ss;
            break;
        case x86_seg_ds:
            selector = regs->ds;
            break;
        case x86_seg_es:
            selector = regs->es;
            break;
        case x86_seg_fs:
            selector = regs->fs;
            break;
        case x86_seg_gs:
            selector = regs->gs;
            break;
#if 0
        case x86_seg_tr:
            store_tr(selector);
            break;
#endif
        default:
	    printk("VGT:Unsupported segment %d in get_selector!!!\n", seg);
	    break;
    }
    return selector;
}

int _un_wbinvd(struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("wbinvd");
}

int _un_cpuid(unsigned int *eax, unsigned int *ebx,
	unsigned int *ecx, unsigned int *edx,
	struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("cpuid");
}

int _un_inject_hw_exception(uint8_t vec, int32_t error_code,
	struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("inject_hw_exception");
}

int _un_inject_sw_exception(uint8_t vec, uint8_t insn_len,
	struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("inject_sw_exception");
}

int _un_get_fpu(void (*fn)(void *, struct cpu_user_regs *),
	void *args, enum x86_emulate_fpu_type type, struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("get_fpu");
}

void _un_put_fpu(struct x86_emulate_ctxt *ctxt)
{
	dprintk("VGT: Unsupported emulation of %s, gip=%lx\n",	\
		"put_fpu", (unsigned long)ctxt->regs->rip); 		\
}

int _un_invlpg(enum x86_segment seg, unsigned long offset, struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("invlpg");
}

int _un_read_msr(
        unsigned long reg,
        uint64_t *val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("read_msr");
}

int _un_write_msr(
        unsigned long reg,
        uint64_t val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("write_msr");
}

int _un_read_dr(
        unsigned int reg,
        unsigned long *val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("read_dr");
}

int _un_write_dr(
        unsigned int reg,
        unsigned long val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("write_dr");
}

int _un_read_cr(
        unsigned int reg,
        unsigned long *val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("read_cr");
}

int _un_write_cr(
        unsigned int reg,
        unsigned long val,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("write_cr");
}

int is_vgt_trap_pio(unsigned int port)
{
	int i;

	for (i=0; i<trap_req.nr_pio_frags; i++) {
		if ( port >= trap_req.pio_frags[i].s &&
			port <= trap_req.pio_frags[i].e )
			return 1;
	}
	return 0;
}

int hcall_pio_write(
        unsigned int port,
        unsigned int bytes,
        unsigned long val)
{
    struct vcpu_emul_ioreq req;
    req.data = val;
    req.addr = port;
    req.size = bytes;
    req.dir = PV_IOREQ_WRITE;
    req.type = PV_IOREQ_TYPE_PIO;
    if (HYPERVISOR_vcpu_op(VCPUOP_request_io_emulation,
			smp_processor_id(), &req) < 0) {
	printk("vGT: failed to do hypercall for read address (%x)\n", port);
	return X86EMUL_UNHANDLEABLE;
    }
    return X86EMUL_OKAY;
}

int hcall_pio_read(
        unsigned int port,
        unsigned int bytes,
        unsigned long *val)
{
    struct vcpu_emul_ioreq req;

    req.data = 0x12345678; // a correctness check
    req.addr = port;
    req.size = bytes;
    req.dir = PV_IOREQ_READ;
    req.type = PV_IOREQ_TYPE_PIO;
    if (HYPERVISOR_vcpu_op(VCPUOP_request_io_emulation,
			smp_processor_id(), &req) < 0) {
	printk("vGT: failed to do hypercall for read address (%x)\n", port);
	return X86EMUL_UNHANDLEABLE;
    }
    *val = req.data;
    return X86EMUL_OKAY;
}

static unsigned int vgt_cf8;
int vgt_cfg_write_emul(
        unsigned int port,
        unsigned int bytes,
        unsigned long val,
        struct x86_emulate_ctxt *ctxt)
{
    int rc = X86EMUL_OKAY;
#ifdef SINGLE_VM_DEBUG
	int dom_id = 0;
#else
	int dom_id = ...;
#endif

    dprintk("VGT: vgt_cfg_write_emul %x %x %lx at %llx\n",
	    port, bytes, val, ctxt->regs->rip);

    if ((port & ~3)== 0xcf8) {
        ASSERT (bytes == 4);
        ASSERT ((port & 3) == 0);

        vgt_cf8 = val;
	dprintk("vgt_cf8 write w/ %x\n", vgt_cf8);
    }
    else {	// port 0xCFC */
	dprintk("cfg_write_emul port %x %d %lx\n",port, bytes, val);
        ASSERT ( (vgt_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
            ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));

	/*
	 * at boot time, dom0 always has write accesses to hw
	 * for initialization work
	 *
	 * FIXME: bar size check by i915 driver shouldn't go to hw!!!
	 * FIXME: S3 suspend/resume needs to reset boot_time again!!!
	 */
	if (vgt_ops && !vgt_ops->boot_time) {
		if (!vgt_ops->cfg_write(vgt_devices[dom_id].vgt,
			(vgt_cf8 & 0xfc) + (port & 3),
			&val, bytes)) {
			rc = X86EMUL_UNHANDLEABLE;
			goto out;
		}
	} else
		rc = hcall_pio_write(port, bytes, val);
    }

out:
    return rc;
}

int vgt_cfg_read_emul(
        unsigned int port,
        unsigned int bytes,
        unsigned long *val)
{
    unsigned long data;
    int rc = X86EMUL_OKAY;
#ifdef SINGLE_VM_DEBUG
	int dom_id = 0;
#else
	int dom_id = ...;
#endif

    if ((port & ~3)== 0xcf8) {
        memcpy(val, (uint8_t*)&vgt_cf8 + (port & 3), bytes);
    }
    else {
        ASSERT ( (vgt_cf8 & 3) == 0);
        ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
            ((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));

	/*
	 * FIXME: similarly, for i915 bar size check we want it in virtual
	 * bar, but if there's some real hw initialization work, we'd like
	 * it to hw!!! hard to check
	 */
	if (!vgt_ops || vgt_ops->boot_time) {
		rc = hcall_pio_read(port, bytes, &data);
		if (rc != X86EMUL_OKAY)
			goto out;
	} else {
		if (!vgt_ops->cfg_read(vgt_devices[dom_id].vgt,
			(vgt_cf8 & 0xfc) + (port & 3),
			&data, bytes)) {
			rc = X86EMUL_UNHANDLEABLE;
			goto out;
		}
	}

	memcpy(val, &data, bytes);
    }
    dprintk("VGT: vgt_cfg_read_emul port %x bytes %x got %lx\n",
			port, bytes, *val);
out:
    return rc;
}

/* PIO read */
int read_io(
        unsigned int port,
        unsigned int bytes,
        unsigned long *val,
        struct x86_emulate_ctxt *ctxt)
{
    unsigned int  aport;
    unsigned long data;

    ASSERT ( is_vgt_trap_pio(port) == is_vgt_trap_pio(port + bytes - 1) );
    ASSERT (bytes <= 4);	/* TODO */

    aport = port & ~3;
    if ( aport == 0xcf8 || aport == 0xcfc ) {
	dprintk("VGT: read pio at gip %lx port %x bytes %x\n",
		(unsigned long)ctxt->regs->rip, port,
		bytes);
	return vgt_cfg_read_emul(port, bytes, val);
    }

    if ( !is_vgt_trap_pio(port) ) {
        printk("Unknown PIO read at %lx, port %x bytes %x!!!\n",
		(unsigned long)ctxt->regs->rip, port, bytes);
        return X86EMUL_UNHANDLEABLE;
    }
    if ( hcall_pio_read(port, bytes, &data) != X86EMUL_OKAY)
	return X86EMUL_UNHANDLEABLE;

#if 0
    dprintk("read pio at gip %lx port %x bytes %x val %x\n",
	(unsigned long)ctxt->regs->rip, port, bytes, (unsigned int) req.data);
#endif
    memcpy ( val, &data, bytes);

    return X86EMUL_OKAY;
}

int write_io(
        unsigned int port,
        unsigned int bytes,
        unsigned long val,
        struct x86_emulate_ctxt *ctxt)
{
    unsigned int  aport;

    ASSERT ( is_vgt_trap_pio(port) == is_vgt_trap_pio(port + bytes - 1) );
    ASSERT (bytes <= 4);	/* TODO */

    aport = port & ~3;
    if ( aport == 0xcf8 || aport == 0xcfc ) {
        dprintk("VGT: write pio at gip %lx port %x bytes %x val %lx\n",
		(unsigned long)ctxt->regs->rip,
		port, bytes, val);
	return vgt_cfg_write_emul(port, bytes, val, ctxt);
    }

    if ( !is_vgt_trap_pio(port) ) {
        printk("Unknown PIO write at %lx, port %x bytes %x!!!\n",
		(unsigned long)ctxt->regs->rip, port, bytes);
        return X86EMUL_UNHANDLEABLE;
    }

    return hcall_pio_write(port, bytes, val);
}

static int xen_read_sys_data(void *p_data, unsigned long offset, int bytes)
{
    struct vcpu_sysdata_request req;

    req.op_type = VCPUOP_sysdata_read;
    req.bytes = bytes;
    req.src_addr = offset;

    ASSERT (bytes <= 8);
    if (HYPERVISOR_vcpu_op(VCPUOP_get_sysdata,
			smp_processor_id(), &req) < 0) {
	printk("vGT: failed to do VCPUOP_sysdata_read hypercall,src : %lx\n",
			offset);
		return X86EMUL_UNHANDLEABLE;
    }
#if 0
    dprintk("xen_read_sys_data: offset %lx got %llx bytes %d\n",
		offset, req.sys_data, bytes);
#endif
    memcpy (p_data, &req.sys_data, bytes);
    return X86EMUL_OKAY;
}

int read_segment(
        enum x86_segment seg,
        struct segment_register *reg,
        struct x86_emulate_ctxt *ctxt)
{
    struct vcpu_sysdata_request req;
    struct desc_struct desc;

    req.selector = get_selector(seg, ctxt->regs);
    req.op_type = VCPUOP_sysdata_get_segment;

    if (HYPERVISOR_vcpu_op(VCPUOP_get_sysdata,
			smp_processor_id(), &req) < 0) {
	printk("vGT: failed to do get_segment hypercall, sel: %x\n",
			req.selector);
		return X86EMUL_UNHANDLEABLE;
    }
    desc = *(struct desc_struct*)&req.xdt_desc[0];
    reg->sel = req.selector;
    reg->attr.fields.type = desc.type;
    reg->attr.fields.s = desc.s;
    reg->attr.fields.dpl = desc.dpl;
    reg->attr.fields.p = desc.p;
    reg->attr.fields.avl = desc.avl;
    reg->attr.fields.l = desc.l;
    reg->attr.fields.db = desc.d;
    reg->attr.fields.g = desc.g;

    if (desc.l)
    {	/* 64 bit mode */
	if ( (seg = x86_seg_fs) || (seg == x86_seg_gs) )
	    reg->base = desc.base0 | (desc.base1 << 16) | (desc.base2 << 24);
	else
	    reg->base = 0;
        reg->limit = 0xfffff;
    }
    else
    {
	reg->base = desc.base0 | ((uint64_t)desc.base1 << 16) |
		((uint64_t)desc.base2 << 24) | req.xdt_desc[1] << 32;
        reg->limit = desc.limit0 | (desc.limit << 16L);
    }
#if 0
    dprintk("VGT: seg %x sel %x base %lx limit %lx attr %x\n",
	seg, req.selector,
	(unsigned long)reg->base, (unsigned long)reg->limit, reg->attr.bytes);
#endif
    return X86EMUL_OKAY;
}

int _un_write_segment(
        enum x86_segment seg,
        struct segment_register *reg,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("write_segment");
}

int _un_cmpxchg(
        enum x86_segment seg,
        unsigned long offset,
        void *p_old,
        void *p_new,
        unsigned int bytes,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("cmpxchg");
}

int _un_rep_ins(
        uint16_t src_port,
        enum x86_segment dst_seg,
        unsigned long dst_offset,
        unsigned int bytes_per_rep,
        unsigned long *reps,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("rep_ins");
}

int _un_rep_outs(
        enum x86_segment src_seg,
        unsigned long src_offset,
        uint16_t dst_port,
        unsigned int bytes_per_rep,
        unsigned long *reps,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("rep_outs");
}

int _un_rep_movs(
        enum x86_segment src_seg,
        unsigned long src_offset,
        enum x86_segment dst_seg,
        unsigned long dst_offset,
        unsigned int bytes_per_rep,
        unsigned long *reps,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("rep_movs");
	printk("src: %lx, dst: %lx\n", src_offset, dst_offset);
}

//#define _PT_WALK_
static unsigned long vgt_va_to_pa(unsigned long v_addr)
{
	unsigned long addr = v_addr, p_addr=0;
	struct page *page = NULL;
	pgd_t *pgd = pgd_offset_k(addr);

#ifdef _PT_WALK_
	printk("pgd %p va %lx\n", pgd, addr);
#endif
	VIRTUAL_BUG_ON(!is_vmalloc_or_module_addr(vmalloc_addr));

	if (!pgd_none(*pgd)) {
		pud_t *pud = pud_offset(pgd, addr);

#ifdef _PT_WALK_
        printk("pud %p val %lx\n", pud, (long)(*pud).pud);
#endif
		if (!pud_none(*pud)) {
			pmd_t *pmd = pmd_offset(pud, addr);
#ifdef _PT_WALK_
printk("pmd %p val %lx\n", pmd, (long)(*pmd).pmd);
#endif
			if (!pmd_none(*pmd)) {
				pte_t *ptep, pte;

				ptep = pte_offset_map(pmd, addr);
				pte = *ptep;
#ifdef _PT_WALK_
	printk("pte %p val %lx\n", ptep, (long)pte.pte);
#endif
				if (pte_present(pte)) {
					page = pte_page(pte);
					p_addr = pte_pfn(pte);
					p_addr <<= PAGE_SHIFT;
					p_addr += (v_addr & ~PAGE_MASK);
				}
				pte_unmap(ptep);
			}
		}
	}
	return p_addr;
}

int is_vgt_trap_address(unsigned long pa)
{
	int i;

	/* Trap address is in page unit. */
	pa &= PAGE_MASK;
	for (i=0; i<trap_req.nr_mmio_frags; i++)
	{
		if ( pa >= trap_req.mmio_frags[i].s &&
			pa <= trap_req.mmio_frags[i].e )
			return 1;
	}
	return 0;
}

int emulate_read(
        enum x86_segment seg,
        unsigned long offset,
        void *p_data,
        unsigned int bytes,
        struct x86_emulate_ctxt *ctxt)
{
	unsigned long r_pa;
#ifdef SINGLE_VM_DEBUG
	int dom_id = 0;
#else
	int dom_id = ...;
#endif

#if 0
	dprintk("VGT: read seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);
#endif
	if ( seg == x86_seg_none ) {
		/* read system structure such as TSS, GDTR etc */
		return xen_read_sys_data(p_data, offset, bytes);
	}

	r_pa = vgt_va_to_pa (offset);
	if ( is_vgt_trap_address(r_pa) ) {
		if (!vgt_ops || !vgt_ops->initialized) {
			struct vcpu_emul_ioreq req;

			req.data = 0x12345678; // a correctness check
			req.addr = r_pa;
			req.size = bytes;
			req.dir = PV_IOREQ_READ;
			req.type = PV_IOREQ_TYPE_COPY;
			if (HYPERVISOR_vcpu_op(VCPUOP_request_io_emulation,
						smp_processor_id(), &req) < 0) {
				printk("vGT: failed to do hypercall for read address (%lx)\n", r_pa);
				return X86EMUL_UNHANDLEABLE;
			}

			memcpy(p_data, (void *)&(req.data), bytes);
		} else {
			uint64_t data;
			if (vgt_ops->mem_read(vgt_devices[dom_id].vgt, r_pa, &data, bytes))
				memcpy(p_data, (void *)&data, bytes);
			else
				return X86EMUL_UNHANDLEABLE;
		}
#if 0
		dprintk("VGT: read pa %08lx data %08lx (%08llx)\n", r_pa, *(unsigned long *)p_data, req.data);
#endif
	}
	else
		memcpy (p_data, (void*)offset, bytes);

	return X86EMUL_OKAY;
}

int emulate_insn_fetch (
        enum x86_segment seg,
        unsigned long offset,
        void *p_data,
        unsigned int bytes,
        struct x86_emulate_ctxt *ctxt)
{
	ASSERT (seg == x86_seg_cs ); 	// TO FIX
#if 0
	dprintk("VGT: insn_fetch seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);
#endif

	memcpy(p_data, (void *)offset, bytes);
	return X86EMUL_OKAY;
}

int emulate_write(
        enum x86_segment seg,
        unsigned long offset,
        void *p_data,
        unsigned int bytes,
        struct x86_emulate_ctxt *ctxt)
{
	unsigned long w_pa, data;
#ifdef SINGLE_VM_DEBUG
	int dom_id = 0;
#else
	int dom_id = ...;
#endif

	ASSERT (seg == x86_seg_ds ); 	// TO FIX
	dprintk("VGT: write seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);

	w_pa = vgt_va_to_pa (offset);
	data = *(long *)p_data;
	dprintk("VGT: write pa %08lx data %08lx\n", w_pa, data);

	if ( is_vgt_trap_address(w_pa) ) {
		if (!vgt_ops || !vgt_ops->initialized) {
			struct vcpu_emul_ioreq req;

			req.data = data; // a correctness check
			req.addr = w_pa;
			req.size = bytes;
			req.dir = PV_IOREQ_WRITE;
			req.type = PV_IOREQ_TYPE_COPY;
			if (HYPERVISOR_vcpu_op(VCPUOP_request_io_emulation,
						smp_processor_id(), &req) < 0) {
				printk("vGT: failed to do hypercall for write address (%lx)\n", w_pa);
				return X86EMUL_UNHANDLEABLE;
			}
			dprintk("hcall return\n");
		} else {
			if (!vgt_ops->mem_write(vgt_devices[dom_id].vgt, w_pa, &data, bytes))
				return X86EMUL_UNHANDLEABLE;
		}
	}
	else
		memcpy ((void*)offset, p_data, bytes);

	return X86EMUL_OKAY;
}

static const struct x86_emulate_ops vgt_emu_ops = {
	.read = emulate_read,
	.write = emulate_write,
	.insn_fetch = emulate_insn_fetch,
	.cmpxchg = _un_cmpxchg,
	.rep_ins = _un_rep_ins,
	.rep_outs = _un_rep_outs,
	.rep_movs = _un_rep_movs,
	.read_segment = read_segment,
	.write_segment = _un_write_segment,
	.read_io = read_io,
	.write_io = write_io,
	.read_cr = _un_read_cr,
	.write_cr = _un_write_cr,
	.read_dr = _un_read_dr,
	.write_dr = _un_write_dr,
	.read_msr = _un_read_msr,
	.write_msr = _un_write_msr,
	.wbinvd = _un_wbinvd,
	.cpuid = _un_cpuid,
	.inject_hw_exception = _un_inject_hw_exception,
	.inject_sw_interrupt = _un_inject_sw_exception,
	.get_fpu = _un_get_fpu,
	.put_fpu = _un_put_fpu,
	.invlpg = _un_invlpg,
};

static struct cpu_user_regs em_regs;
static struct x86_emulate_ctxt ctxt = {
	.force_writeback = 0,
#ifdef __x86_64__
	.addr_size = 64,
	.sp_size = 64,
#else
	.addr_size = 32,
	.sp_size = 32,
#endif
	.retire.byte = 0,
	.regs = &em_regs,
};

void em_regs_2_pt_regs(
	struct cpu_user_regs *src_regs,
	struct pt_regs *tgt_regs)
{
	tgt_regs->r15 = src_regs->r15;
	tgt_regs->r14 = src_regs->r14;
	tgt_regs->r13 = src_regs->r13;
	tgt_regs->r12 = src_regs->r12;
	tgt_regs->bp = src_regs->rbp;
	tgt_regs->bx = src_regs->rbx;
	tgt_regs->r11 = src_regs->r11;
	tgt_regs->r10 = src_regs->r10;
	tgt_regs->r9 = src_regs->r9;
	tgt_regs->r8 = src_regs->r8;
	tgt_regs->ax = src_regs->rax;
	tgt_regs->cx = src_regs->rcx;
	tgt_regs->dx = src_regs->rdx;
	tgt_regs->si = src_regs->rsi;
	tgt_regs->di = src_regs->rdi;
	// skip orig_rax
	tgt_regs->ip = src_regs->rip;
	tgt_regs->cs = src_regs->cs;
	tgt_regs->flags = src_regs->eflags;
	tgt_regs->sp = src_regs->rsp;
	tgt_regs->ss = src_regs->ss;
#if 0
	dprintk("user_regs to pt_regs eax %08lx ebx %08lx ecx %08lx edx"
		" %08lx ip %08lx\n",
		tgt_regs->ax, tgt_regs->bx, tgt_regs->cx,
		tgt_regs->dx, tgt_regs->ip);
#endif
}

void pt_regs_2_em_regs(
	struct pt_regs *src_regs,
	struct cpu_user_regs *tgt_regs)
{
	tgt_regs->r15 = src_regs->r15;
	tgt_regs->r14 = src_regs->r14;
	tgt_regs->r13 = src_regs->r13;
	tgt_regs->r12 = src_regs->r12;
	tgt_regs->rbp = src_regs->bp;
	tgt_regs->rbx = src_regs->bx;
	tgt_regs->r11 = src_regs->r11;
	tgt_regs->r10 = src_regs->r10;
	tgt_regs->r9 = src_regs->r9;
	tgt_regs->r8 = src_regs->r8;
	tgt_regs->rax = src_regs->ax;
	tgt_regs->rcx = src_regs->cx;
	tgt_regs->rdx = src_regs->dx;
	tgt_regs->rsi = src_regs->si;
	tgt_regs->rdi = src_regs->di;
	// skip orig_rax
	tgt_regs->rip = src_regs->ip;
	tgt_regs->cs = src_regs->cs;
	tgt_regs->eflags = src_regs->flags;
	tgt_regs->rsp = src_regs->sp;
	tgt_regs->ss = src_regs->ss;
#if 0
	dprintk("pt_regs to user_regs eax %08llx ebx %08llx ecx %08llx edx"
		" %08llx ip %08llx\n",
		tgt_regs->rax, tgt_regs->rbx, tgt_regs->rcx,
		tgt_regs->rdx, tgt_regs->rip);
#endif
}

static int vgt_emulate_ins(struct pt_regs *regs)
{
	int rc;

	pt_regs_2_em_regs(regs, &em_regs);
	rc = x86_emulate (&ctxt, &vgt_emu_ops);
	em_regs_2_pt_regs(&em_regs, regs);

	return rc;
}

static int vgt_io_forward = 0;
static int xen_vgt_handler(struct pt_regs *regs, long error_code)
{
	if (!vgt_io_forward || (error_code != 0xe008 && error_code != 0xe00c))
		return 0;

	return vgt_emulate_ins(regs) == X86EMUL_OKAY;
}

int xen_setup_vgt(vgt_ops_t *ops)
{
    vgt_ops = ops;
	if (!register_gp_prehandler(xen_vgt_handler)) {
		trap_req.nr_pio_frags = 1;
		trap_req.pio_frags[0].s = 0x3B0;
		trap_req.pio_frags[0].e = 0x3DF;
		trap_req.nr_mmio_frags = -1;	/* let hypervisor tell */
		printk("vGT: install GP handler successfully\n");
		if (HYPERVISOR_vcpu_op(VCPUOP_start_io_forward, 0, &trap_req) < 0) {
			printk("vGT: failed to start I/O forwarding\n");
			return -EINVAL;
		} else {
			vgt_io_forward = 1;
			printk("vGT: trap_req.nr_pio_frags: mmio %d %lx %lx\n",
				trap_req.nr_mmio_frags,
				(long)trap_req.mmio_frags[0].s,
				(long)trap_req.mmio_frags[0].e
				);
		}
	} else
		printk("vGT: fail to install GP handler\n");
	return 0;
}

int xen_start_vgt(struct pci_dev *pdev)
{
    int ret = 0;

	if (vgt_ops && vgt_ops->initialized) {
		printk("vgt_ops has been intialized\n");
		return 0;
	}

printk("eddie: xen_start_vgt vgt_ops %p \n", vgt_ops);
if (vgt_ops)
printk("Eddie: start_vgt %p\n", vgt_ops->start_vgt);
    if (vgt_ops && vgt_ops->start_vgt)
        ret = vgt_ops->start_vgt(pdev);
    return ret;
}

//core_initcall(xen_setup_vgt);
/* for GFX driver */
EXPORT_SYMBOL(xen_start_vgt);

/* for vGT driver */
EXPORT_SYMBOL(xen_setup_vgt);
EXPORT_SYMBOL(xen_register_vgt_device);
EXPORT_SYMBOL(xen_deregister_vgt_device);
