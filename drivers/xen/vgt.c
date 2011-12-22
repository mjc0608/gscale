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
	sf_ioreq_t *req = (sf_ioreq_t *)&HYPERVISOR_shared_info->arch.sf_ioreq;

	if (!vgt_io_forward || req->state != STATE_PV_IOREQ_READY)
		return 0;

	dprintk("vgt_handler error_code %lx req->addr %llx \n", error_code, req->addr);
	if (error_code != 0xe008)
		return 0;

	return vgt_emulate_ins(regs) == X86EMUL_OKAY;
}

static void __init xen_setup_vgt(void)
{
	if (!register_gp_prehandler(xen_vgt_handler)) {
		printk("vGT: install GP handler successfully\n");
		vgt_io_forward = 1;
	} else
		printk("vGT: fail to install GP handler\n");
}
core_initcall(xen_setup_vgt);
