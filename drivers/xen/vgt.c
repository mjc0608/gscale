/*
 * vGT core module
 * Copyright (c) 2011, Intel Corporation.
 *
 * CCCCCCCCCCCCCCCCCCCCCCCCCC (leave copyright to be filled later)
 */
#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <asm/sync_bitops.h>
#include <asm/traps.h>
#include <asm/xen/pci.h>
#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/xen-ops.h>
#include <xen/interface/xen.h>
#include <xen/interface/vcpu.h>
#include <xen/x86_emulate.h>
#include <linux/init.h>

#define VGT_DEBUG
#ifdef VGT_DEBUG
#define dprintk(fmt, a...)	\
	printk("vGT:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##a)
#else
#define dprintk(fmt, a...)
#endif
static int vgt_io_forward = 0;
extern int vgt_emulate_ins(struct pt_regs *regs);
static int xen_vgt_handler(struct pt_regs *regs, long error_code)
{
	if (!vgt_io_forward || error_code != 0xe008)
		return 0;

	return vgt_emulate_ins(regs) == X86EMUL_OKAY;
}

struct vcpu_io_forwarding_request trap_req;
static int __init xen_setup_vgt(void)
{
	if (!register_gp_prehandler(xen_vgt_handler)) {
		trap_req.nr_pio_frags = 1;
		trap_req.pio_frags[0].s = 0x3B0;
		trap_req.pio_frags[0].e = 0x3DF;
		trap_req.nr_mmio_frags = -1;	/* let hypervisor tell */
		printk("vGT: install GP handler successfully\n");
		if (HYPERVISOR_vcpu_op(VCPUOP_start_io_forward, 0, &trap_req) < 0)
			printk("vGT: failed to start I/O forwarding\n");
		else {
			vgt_io_forward = 1;
			printk("trap_req.nr_pio_frags: mmio %d %lx %lx\n",
				trap_req.nr_mmio_frags,
				(long)trap_req.mmio_frags[0].s,
				(long)trap_req.mmio_frags[0].e
				);
		}
	} else
		printk("vGT: fail to install GP handler\n");
	return 0;
}
core_initcall(xen_setup_vgt);
