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
 */

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <xen/vgt.h>
#include "vgt_drv.h"
#include <xen/vgt-parser.h>

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("vGT mediated graphics passthrough driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.1");

static vgt_ops_t vgt_xops = {
    .mem_read = vgt_emulate_read,
    .mem_write = vgt_emulate_write,
    .cfg_read = vgt_emulate_cfg_read,
    .cfg_write = vgt_emulate_cfg_write,
    .boot_time = 1,
};

/* for GFX driver */
int xen_start_vgt(struct pci_dev *pdev)
{
	if (!xen_initial_domain())
		return 0;

	if (vgt_xops.initialized) {
		printk("vgt_ops has been intialized\n");
		return 0;
	}

	return vgt_initialize(pdev) == 0;
}
EXPORT_SYMBOL(xen_start_vgt);

static int __init vgt_init_module(void)
{
	int rc;

	if (!xen_initial_domain())
		return 0;

	rc  = vgt_cmd_parser_init();
	if(rc)
		return rc;

	vgt_klog_init();

	return xen_register_vgt_driver(&vgt_xops);
}
module_init(vgt_init_module);

static void __exit vgt_exit_module(void)
{
	int rc = 0;

	if (!xen_initial_domain())
		return;
	// Need cancel the i/o forwarding

	// fill other exit works here
	vgt_destroy();
	vgt_cmd_parser_exit();
	vgt_klog_cleanup();
	printk("VGT module exit %d\n", rc);
	return;
}
module_exit(vgt_exit_module);
