/*
 * vGT instruction emulator
 * Copyright (c) 2011, Intel Corporation.
 *
 * CCCCCCCCCCCCCCCCCCCCCCCCCC (leave copyright to be filled later)
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/ptrace.h>
#include <asm/xen/interface.h>
#include <xen/x86_emulate.h>
#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>
#include <xen/interface/vcpu.h>

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

#define VGT_DEBUG
#ifdef VGT_DEBUG
#define dprintk(fmt, a...)	\
	printk("vGT:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##a)
#else
#define dprintk(fmt, a...)
#endif

extern struct vcpu_io_forwarding_request trap_req;

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

int read_io(
        unsigned int port,
        unsigned int bytes,
        unsigned long *val,
        struct x86_emulate_ctxt *ctxt)
{
    struct vcpu_emul_ioreq req;

    ASSERT ( is_vgt_trap_pio(port) == is_vgt_trap_pio(port + bytes - 1) );
    ASSERT (bytes <= 4);	/* TODO */

    if ( !is_vgt_trap_pio(port) ) {
        printk("Unknown PIO read at %lx, port %x bytes %x!!!\n",
		(unsigned long)ctxt->regs->rip, port, bytes);
        return X86EMUL_UNHANDLEABLE;
    }
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
dprintk("read pio at gip %lx port %x bytes %x val %x\n",
	(unsigned long)ctxt->regs->rip, port, bytes, (unsigned int) req.data);
    memcpy ( val, &req.data, bytes);

    return X86EMUL_OKAY;
}

int write_io(
        unsigned int port,
        unsigned int bytes,
        unsigned long val,
        struct x86_emulate_ctxt *ctxt)
{
    struct vcpu_emul_ioreq req;

    ASSERT ( is_vgt_trap_pio(port) == is_vgt_trap_pio(port + bytes - 1) );
    ASSERT (bytes <= 4);	/* TODO */

    if ( !is_vgt_trap_pio(port) ) {
        printk("Unknown PIO write at %lx, port %x bytes %x!!!\n",
		(unsigned long)ctxt->regs->rip, port, bytes);
        return X86EMUL_UNHANDLEABLE;
    }
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
dprintk("write pio at gip %lx port %x bytes %x val %lx\n",
	(unsigned long)ctxt->regs->rip, port, bytes, val);

    return X86EMUL_OKAY;
}

int _un_read_segment(
        enum x86_segment seg,
        struct segment_register *reg,
        struct x86_emulate_ctxt *ctxt)
{
	UNSUPPORTED("read_segment");
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

	ASSERT (seg == x86_seg_ds ); 	// TO FIX
	dprintk("VGT: read seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);

	r_pa = vgt_va_to_pa (offset);
	if ( is_vgt_trap_address(r_pa) ) {
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

		dprintk("hcall return\n");
		memcpy(p_data, (void *)&(req.data), bytes);
		dprintk("VGT: read pa %08lx data %08lx (%08llx)\n", r_pa, *(unsigned long *)p_data, req.data);
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
	dprintk("VGT: insn_fetch seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);

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

	ASSERT (seg == x86_seg_ds ); 	// TO FIX
	dprintk("VGT: write seg %x off %lx data %p bytes %d gip = %llx\n",
		seg, offset, p_data, bytes, ctxt->regs->rip);

	w_pa = vgt_va_to_pa (offset);
	data = *(long *)p_data;
	dprintk("VGT: write pa %08lx data %08lx\n", w_pa, data);

	if ( is_vgt_trap_address(w_pa) ) {
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
	}
	else
		memcpy ((void*)offset, p_data, bytes);

	return X86EMUL_OKAY;
}

static const struct x86_emulate_ops vgt_ops = {
	.read = emulate_read,
	.write = emulate_write,
	.insn_fetch = emulate_insn_fetch,
	.cmpxchg = _un_cmpxchg,
	.rep_ins = _un_rep_ins,
	.rep_outs = _un_rep_outs,
	.rep_movs = _un_rep_movs,
	.read_segment = _un_read_segment,
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
	dprintk("user_regs to pt_regs eax %08lx ebx %08lx ecx %08lx edx"
		" %08lx ip %08lx\n",
		tgt_regs->ax, tgt_regs->bx, tgt_regs->cx,
		tgt_regs->dx, tgt_regs->ip);
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
	dprintk("pt_regs to user_regs eax %08llx ebx %08llx ecx %08llx edx"
		" %08llx ip %08llx\n",
		tgt_regs->rax, tgt_regs->rbx, tgt_regs->rcx,
		tgt_regs->rdx, tgt_regs->rip);
}

int vgt_emulate_ins(struct pt_regs *regs)
{
	int rc;

	pt_regs_2_em_regs(regs, &em_regs);
	rc = x86_emulate (&ctxt, &vgt_ops);
	em_regs_2_pt_regs(&em_regs, regs);

	return rc;
}

