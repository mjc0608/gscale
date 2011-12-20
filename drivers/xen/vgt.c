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
#include <linux/init.h>

static int vgt_io_forward = 0;
static int xen_vgt_handler(struct pt_regs *regs, long error_code)
{
	sf_ioreq_t *req = (sf_ioreq_t *)&HYPERVISOR_shared_info->arch.sf_ioreq;

	if (!vgt_io_forward || req->state != STATE_PV_IOREQ_READY)
		return -EINVAL;

	/* a short circuit to Xen with a default pass-through policy */
	req->state = STATE_PV_IOREQ_EMUL;
	if (HYPERVISOR_vcpu_op(VCPUOP_request_io_emulation, 0, NULL) < 0 ||
	    req->state != STATE_PV_IOREQ_EMUL_DONE)
		return -EINVAL;

	req->state = STATE_PV_IORESP_READY; /* if relying on Xen to complete */
	//req->state = STATE_PV_IOREQ_NONE; /* if completing the emulation by vGT itself */
	return 0;
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
