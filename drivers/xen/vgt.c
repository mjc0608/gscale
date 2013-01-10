/*
 * vGT core module
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
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static vgt_ops_t vgt_xops = {
    .mem_read = vgt_emulate_read,
    .mem_write = vgt_emulate_write,
    .cfg_read = vgt_emulate_cfg_read,
    .cfg_write = vgt_emulate_cfg_write,
    .boot_time = 1,
};

bool hvm_render_owner = false;
module_param_named(hvm_render_owner, hvm_render_owner, bool, 0600);
MODULE_PARM_DESC(hvm_render_owner, "Make HVM to be render owner after create (default: false)");

bool hvm_dpy_owner = false;
module_param_named(hvm_dpy_owner, hvm_dpy_owner, bool, 0600);
MODULE_PARM_DESC(hvm_dpy_owner, "Make HVM to be display owner after create (default: false)");

bool hvm_owner = false;
module_param_named(hvm_owner, hvm_owner, bool, 0600);
MODULE_PARM_DESC(hvm_owner, "Make HVM to be GPU owner after create (default: false)");

bool hvm_super_owner = false;
module_param_named(hvm_super_owner, hvm_super_owner, bool, 0600);

bool vgt_primary = false;
module_param_named(vgt_primary, vgt_primary, bool, 0600);

bool vgt_debug = false;
module_param_named(vgt_debug, vgt_debug, bool, 0600);

bool novgt = false;
module_param_named(novgt, novgt, bool, 0400);

bool fastpath_dpy_switch = false;
module_param_named(fastpath_dpy_switch, fastpath_dpy_switch, bool, 0600);

int fastmode = 1;
module_param_named(fastmode, fastmode, int, 0600);

int disable_ppgtt = 0;
module_param_named(disable_ppgtt, disable_ppgtt, int, 0600);

/*
 * FIXME: now video ring switch has weird issue. The cmd
 * parser may enter endless loop even when head/tail is
 * zero. earlier posting read doesn't solve the issue.
 * so disable it for now.
 *
 * Dexuan: let's enable VCS switch, because on HSW, win7 gfx drver's PAVP
 * initialization uses VCS. Without enabling this option, win7 guest's gfx
 * driver's initializtion will hang when we create the guest for the 2nd
 * time(VCS.TAIL is 0x70, but VCS.HEAD is always 0x30).
 */
int enable_video_switch = 1;
module_param_named(enable_video_switch, enable_video_switch, int, 0600);

/* enable this to use the old style switch context */
bool use_old_ctx_switch = false;
module_param_named(use_old_ctx_switch, use_old_ctx_switch , bool, 0600);

int dom0_aperture_sz = 64;	//in MB.
module_param_named(dom0_aperture_sz, dom0_aperture_sz, int, 0600);

int dom0_gm_sz = 64;			//in MB. Dom0 has no hidden gm.
module_param_named(dom0_gm_sz, dom0_gm_sz, int, 0600);

int dom0_fence_sz = 4;
module_param_named(dom0_fence_sz, dom0_fence_sz, int, 0600);

bool bypass_scan = true;
module_param_named(bypass_scan, bypass_scan, bool, 0600);

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

static void vgt_param_check(void)
{
	if (hvm_owner || hvm_super_owner) {
		hvm_dpy_owner = true;
		hvm_render_owner = true;
	}

	if (dom0_aperture_sz > 256)
		dom0_aperture_sz = 256;

	if (dom0_gm_sz > 2048)
		dom0_gm_sz = 2048;

	if (dom0_gm_sz < dom0_aperture_sz)
		dom0_gm_sz = dom0_aperture_sz;

	if (dom0_fence_sz > 16)
		dom0_fence_sz = 16;
}

static int __init vgt_init_module(void)
{
	int rc;

	if (!xen_initial_domain())
		return 0;

	vgt_param_check();

	rc  = vgt_cmd_parser_init();
	if(rc)
		return rc;

	vgt_klog_init();

	/* register call back function for i915 driver*/
	cb_xen_start_vgt = xen_start_vgt;
	cb_vgt_install_irq = vgt_install_irq;

	return xen_register_vgt_driver(&vgt_xops);
}
arch_initcall(vgt_init_module);

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
