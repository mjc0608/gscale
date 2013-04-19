/*
 * vGT module interface
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
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
 */
/*
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
#include <linux/kthread.h>
#include <linux/pci.h>
#include <asm/xen/hypercall.h>
#include <xen/interface/vcpu.h>
#include <xen/vgt.h>

#include "vgt.h"
#include "devtable.h"

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("vGT mediated graphics passthrough driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

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
module_param_named(debug, vgt_debug, bool, 0600);

bool vgt_enabled = true;
module_param_named(vgt, vgt_enabled, bool, 0400);

bool fastpath_dpy_switch = true;
module_param_named(fastpath_dpy_switch, fastpath_dpy_switch, bool, 0600);

int fastmode = 1;
module_param_named(fastmode, fastmode, int, 0600);

bool event_based_qos = false;
module_param_named(event_based_qos, event_based_qos, bool, 0600);
MODULE_PARM_DESC(event_based_qos, "Use event based QoS scheduler (default: false)");

bool shadow_tail_based_qos = false;
module_param_named(shadow_tail_based_qos, shadow_tail_based_qos, bool, 0600);
MODULE_PARM_DESC(shadow_tail_based_qos, "Use Shadow tail based QoS scheduler (default: false)");
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

int dom0_aperture_sz = 64;	//in MB.
module_param_named(dom0_aperture_sz, dom0_aperture_sz, int, 0600);

int dom0_gm_sz = 64;			//in MB. Dom0 has no hidden gm.
module_param_named(dom0_gm_sz, dom0_gm_sz, int, 0600);

int dom0_fence_sz = 4;
module_param_named(dom0_fence_sz, dom0_fence_sz, int, 0600);

bool bypass_scan = false;
module_param_named(bypass_scan, bypass_scan, bool, 0600);

static vgt_ops_t vgt_xops = {
	.mem_read = vgt_emulate_read,
	.mem_write = vgt_emulate_write,
	.cfg_read = vgt_emulate_cfg_read,
	.cfg_write = vgt_emulate_cfg_write,
	.boot_time = 1,
};

LIST_HEAD(pgt_devices);
struct pgt_device default_device = {
	.bus = 0,
	.devfn = 0x10,		/* BDF: 0:2:0 */
};

struct vgt_device *vgt_dom0;

static bool vgt_start_io_forwarding(struct pgt_device *pdev)
{
	struct vcpu_io_forwarding_request trap_req;
	uint64_t bar0; /* the MMIO BAR for regs(2MB) and GTT */

	struct xen_platform_op xpop;


	bar0 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR0];
	bar0 &= ~0xf;	/* bit0~3 of the bar is the attribution info */

	trap_req.nr_pio_frags = 1;
	trap_req.pio_frags[0].s = 0x3B0;
	trap_req.pio_frags[0].e = 0x3DF;
	trap_req.nr_mmio_frags = 1;
	trap_req.mmio_frags[0].s = bar0;
	trap_req.mmio_frags[0].e = (bar0 + pdev->bar_size[0] - 1) & PAGE_MASK;

	if (HYPERVISOR_vcpu_op(VCPUOP_start_io_forward, 0, &trap_req) < 0) {
		vgt_err("vGT: failed to start I/O forwarding\n");
		return false;
	}

	if (xen_register_vgt_driver(&vgt_xops) != 0)
		return false;

	/*
	 * Pass the GEN device's BDF and the type(SNB/IVB/HSW?) to
	 * the xen hypervisor: xen needs the info to decide which device's
	 * PCI CFG R/W access should be forwarded to the vgt driver, and
	 * to decice the proper forcewake logic.
	 */
	xpop.cmd = XENPF_set_vgt_info;
	xpop.u.vgt_info.gen_dev_bdf = PCI_BDF2(pdev->pbus->number, pdev->devfn);
	xpop.u.vgt_info.gen_dev_type = pdev->gen_dev_type;
	if (HYPERVISOR_dom0_op(&xpop) != 0)
		return false;

	return true;
}

bool initial_phys_states(struct pgt_device *pdev)
{
	int i;
	uint64_t	bar0, bar1;
	struct pci_dev *dev = pdev->pdev;

	vgt_dbg("VGT: Initial_phys_states\n");

	pdev->gtt_size = vgt_get_gtt_size(pdev->pbus);
	gm_sz(pdev) = vgt_get_gtt_size(pdev->pbus) * 1024;

	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4)
		pci_read_config_dword(dev, i,
				(uint32_t *)&pdev->initial_cfg_space[i]);

	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4) {
		if (!(i % 16))
			vgt_dbg("\n[%2x]: ", i);

		vgt_dbg("%02x %02x %02x %02x ",
			*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff00) >> 8,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff0000) >> 16,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff000000) >> 24);
	}
	for (i=0; i < 3; i++) {
		pdev->bar_size[i] = pci_bar_size(pdev, VGT_REG_CFG_SPACE_BAR0 + 8*i);
		printk("bar-%d size: %x\n", i, pdev->bar_size[i]);
	}

	bar0 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR0];
	bar1 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR1];
	printk("bar0: 0x%llx, Bar1: 0x%llx\n", bar0, bar1);

	ASSERT ((bar0 & 7) == 4);
	/* memory, 64 bits bar0 */
	pdev->gttmmio_base = bar0 & ~0xf;
	pdev->mmio_size = VGT_MMIO_SPACE_SZ;
	pdev->reg_num = pdev->mmio_size/REG_SIZE;
	printk("mmio size: %x, gtt size: %x\n", pdev->mmio_size,
		pdev->gtt_size);
	ASSERT(pdev->mmio_size + pdev->gtt_size <= pdev->bar_size[0]);

	ASSERT ((bar1 & 7) == 4);
	/* memory, 64 bits bar */
	pdev->gmadr_base = bar1 & ~0xf;
	printk("gttmmio: 0x%llx, gmadr: 0x%llx\n", pdev->gttmmio_base, pdev->gmadr_base);

	/* start the io forwarding! */
	if (!vgt_start_io_forwarding(pdev))
		return false;;

	/*
	 * From now on, the vgt driver can invoke the
	 * VGT_MMIO_READ()/VGT_MMIO_WRITE()hypercalls, and any access to the
	 * 4MB MMIO of the GEN device is trapped into the vgt driver.
	 */

#if 1		// TODO: runtime sanity check warning...
	pdev->gmadr_va = ioremap (pdev->gmadr_base, pdev->bar_size[1]);
	if ( pdev->gmadr_va == NULL ) {
		printk("Insufficient memory for ioremap2\n");
		return false;
	}
	printk("gmadr_va: 0x%llx\n", (uint64_t)pdev->gmadr_va);
#endif

	vgt_initial_mmio_setup(pdev);
	vgt_initial_opregion_setup(pdev);

	/* FIXME: GMBUS2 has an in-use bit as the hw semaphore, and we should recover
	 * it after the snapshot. Remove this workaround after GMBUS virtualization
	 */
	{
		u32 val = VGT_MMIO_READ(pdev, 0xc5108);
		printk("vGT: GMBUS2 init value: %x, %x\n", pdev->initial_mmio_state[REG_INDEX(0xc5100)], val);
		VGT_MMIO_WRITE(pdev, 0xc5108, val | 0x8000);
	}

	return true;
}

static bool vgt_set_device_type(struct pgt_device *pdev)
{
	if (_is_sandybridge(pdev->pdev->device)) {
		pdev->gen_dev_type = XEN_IGD_SNB;
		vgt_info("Detected Sandybridge\n");
		return true;
	}

	if (_is_ivybridge(pdev->pdev->device)) {
		pdev->gen_dev_type = XEN_IGD_IVB;
		vgt_info("Detected Ivybridge\n");
		return true;
	}

	if (_is_haswell(pdev->pdev->device)) {
		pdev->gen_dev_type = XEN_IGD_HSW;
		vgt_info("Detected Haswell\n");
		return true;
	}

	vgt_err("Unknown chip 0x%x\n", pdev->pdev->device);
	return false;
}

static bool vgt_initialize_pgt_device(struct pci_dev *dev, struct pgt_device *pdev)
{
	pdev->pdev = dev;
	pdev->pbus = dev->bus;

	if (!vgt_set_device_type(pdev))
		return false;

	/* check PPGTT enabling. */
	if (IS_IVB(pdev) || IS_HSW(pdev))
		pdev->enable_ppgtt = 1;

	INIT_LIST_HEAD(&pdev->rendering_runq_head);
	INIT_LIST_HEAD(&pdev->rendering_idleq_head);

	/* TODO: add ivb/hsw difference later */
	pdev->max_engines = 3;
	pdev->ring_mmio_base[RING_BUFFER_RCS] = _REG_RCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_VCS] = _REG_VCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_BCS] = _REG_BCS_TAIL;

	pdev->ring_psmi[RING_BUFFER_RCS] = 0x2050;
	pdev->ring_psmi[RING_BUFFER_VCS] = 0x12050;
	pdev->ring_psmi[RING_BUFFER_BCS] = 0x22050;

	pdev->ring_mi_mode[RING_BUFFER_RCS] = _REG_RCS_MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_VCS] = _REG_VCS_MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_BCS] = _REG_BCS_MI_MODE;

	/* clean port status, 0 means not plugged in */
	memset(pdev->port_detect_status, 0, sizeof(pdev->port_detect_status));
	bitmap_zero(pdev->dpy_emul_request, VGT_MAX_VMS);

	if (!initial_phys_states(pdev)) {
		printk("vGT: failed to initialize physical state\n");
		return false;
	}

	pdev->reg_info = vzalloc (pdev->reg_num * sizeof(reg_info_t));
	if (!pdev->reg_info) {
		printk("vGT: failed to allocate reg_info\n");
		return false;
	}

	initialize_gm_fence_allocation_bitmaps(pdev);

	vgt_setup_reg_info(pdev);
	vgt_post_setup_mmio_hooks(pdev);
	if (vgt_irq_init(pdev) != 0) {
		printk("vGT: failed to initialize irq\n");
		return false;
	}

	bitmap_zero(pdev->v_force_wake_bitmap, VGT_MAX_VMS);
	spin_lock_init(&pdev->v_force_wake_lock);

	vgt_init_reserved_aperture(pdev);

	vgt_ring_init(pdev);

	perf_pgt = pdev;
	return true;
}

/*
 * Initialize the vgt driver.
 *  return 0: success
 *	-1: error
 */
int vgt_initialize(struct pci_dev *dev)
{
	struct pgt_device *pdev = &default_device;
	struct task_struct *p_thread;
	vgt_params_t vp;

	if (!vgt_enabled)
		return 0;

	spin_lock_init(&pdev->lock);

	if (!vgt_initialize_pgt_device(dev, pdev))
		goto err;

	if (vgt_cmd_parser_init(pdev) < 0)
		goto err;

	/* initialize EDID data */
	vgt_probe_edid(pdev, -1);
	pdev->probe_ports = true;

	/* create debugfs interface */
	if (!vgt_init_debugfs(pdev)) {
		printk("vGT:failed to create debugfs\n");
		goto err;
	}

	/* init all mmio_device */
	vgt_init_mmio_device(pdev);

	/* create domain 0 instance */
	vp.vm_id = 0;
	vp.aperture_sz = dom0_aperture_sz;
	vp.gm_sz = dom0_gm_sz;
	vp.fence_sz = dom0_fence_sz;
	vp.vgt_primary = 1; /* this isn't actually used for dom0 */
	if (create_vgt_instance(pdev, &vgt_dom0, vp) < 0)
		goto err;

	pdev->owner[VGT_OT_DISPLAY] = vgt_dom0;
	vgt_dbg("create dom0 instance succeeds\n");

	show_mode_settings(pdev);

	if (setup_gtt(pdev))
		goto err;

	xen_vgt_dom0_ready(vgt_dom0);

	/* "hvm_owner" is a special mode where we give all the ownerships to the hvm guest */
	if (!hvm_render_owner)
		current_render_owner(pdev) = vgt_dom0;
	else
		vgt_ctx_switch = 0;
	current_display_owner(pdev) = vgt_dom0;
	current_foreground_vm(pdev) = vgt_dom0;
	current_pm_owner(pdev) = vgt_dom0;
	current_mgmt_owner(pdev) = vgt_dom0;
	pdev->ctx_check = 0;
	pdev->ctx_switch = 0;
	pdev->magic = 0;

	init_waitqueue_head(&pdev->event_wq);
	init_waitqueue_head(&pdev->destroy_wq);

	p_thread = kthread_run(vgt_thread, vgt_dom0, "vgt_thread");
	if (!p_thread) {
		goto err;
	}
	pdev->p_thread = p_thread;
	show_debug(pdev, 0);

	vgt_initialize_ctx_scheduler(pdev);

	list_add(&pdev->list, &pgt_devices);

	vgt_init_sysfs(pdev);

#ifdef VGT_DEBUGFS_DUMP_FB
	/* There is anytime only one instance of the workqueue,
	 * and NON_REENTRANT
	 */
	pdev->pgt_wq = alloc_workqueue("vgt_workqueue",
			WQ_UNBOUND | WQ_NON_REENTRANT,
			1);
	if (!pdev->pgt_wq) {
		printk("vGT: failed to create kthread: vgt_workqueue.\n");
		goto err;
	}
#endif

	printk("vgt_initialize succeeds.\n");
	return 0;
err:
	printk("vgt_initialize failed.\n");
	vgt_destroy();
	return -1;
}

void vgt_destroy(void)
{
	struct list_head *pos, *next;
	struct vgt_device *vgt;
	struct pgt_device *pdev = &default_device;
	int i;

	vgt_cleanup_mmio_dev(pdev);

	perf_pgt = NULL;
	list_del(&pdev->list);

	vgt_cleanup_ctx_scheduler(pdev);

	/* do we need the thread actually stopped? */
	kthread_stop(pdev->p_thread);

	vgt_irq_exit(pdev);

	/* Deactive all VGTs */
	while ( !list_empty(&pdev->rendering_runq_head) ) {
		list_for_each (pos, &pdev->rendering_runq_head) {
			vgt = list_entry (pos, struct vgt_device, list);
			vgt_disable_render(vgt);
		}
	};

#ifdef VGT_DEBUGFS_DUMP_FB
	/* Destruct pgt_wq */
	destroy_workqueue(pdev->pgt_wq);
#endif

	/* Destruct all vgt_debugfs */
	vgt_release_debugfs();

	free_gtt(pdev);

	if (pdev->gmadr_va)
		iounmap(pdev->gmadr_va);
	if (pdev->opregion_va)
		iounmap(pdev->opregion_va);

	while ( !list_empty(&pdev->rendering_idleq_head)) {
		for (pos = pdev->rendering_idleq_head.next;
			pos != &pdev->rendering_idleq_head; pos = next) {
			next = pos->next;
			vgt = list_entry (pos, struct vgt_device, list);
			vgt_release_instance(vgt);
		}
	}
	vgt_clear_mmio_table();
	vfree(pdev->reg_info);
	vfree(pdev->initial_mmio_state);

	for (i = 0; i < EDID_MAX; ++ i) {
		if (pdev->pdev_edids[i]) {
			kfree(pdev->pdev_edids[i]);
			pdev->pdev_edids[i] = NULL;
		}
	}
	vgt_cmd_parser_exit();
}

/* for GFX driver */
int xen_start_vgt(struct pci_dev *pdev)
{
	if (!xen_initial_domain())
		return 0;

	if (vgt_xops.initialized) {
		vgt_info("vgt_ops has been intialized\n");
		return 0;
	}

	return vgt_initialize(pdev);
}

EXPORT_SYMBOL(xen_start_vgt);

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
	if (!xen_initial_domain())
		return 0;

	vgt_param_check();

	vgt_klog_init();

	return 0;
}
module_init(vgt_init_module);

static void __exit vgt_exit_module(void)
{
	if (!xen_initial_domain())
		return;
	// Need cancel the i/o forwarding

	// fill other exit works here
	vgt_destroy();
	vgt_klog_cleanup();
	return;
}
module_exit(vgt_exit_module);
