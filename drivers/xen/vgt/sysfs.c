/*
 * vGT sysfs interface (the original code comes from samples/kobject-example.c)
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

#include <linux/slab.h>
#include <asm/xen/x86_emulate.h> /* only for X86EMUL_OKAY */
#include "vgt.h"

struct kobject *vgt_ctrl_kobj;
static struct kset *vgt_kset;
static DEFINE_MUTEX(vgt_sysfs_lock);

static void vgt_kobj_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
	/* NOTE: we do not deallocate our kobject */
	/* see the comment before vgt_init_sysfs() */
	//kfree(kobj);
}

static int vgt_add_state_sysfs(vgt_params_t vp);
static int vgt_del_state_sysfs(vgt_params_t vp);
static ssize_t vgt_create_instance_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	vgt_params_t vp;
	int param_cnt;
	char param_str[64];
	int rc;
	int high_gm_sz;
	int low_gm_sz;
	int cpu;

	/* We expect the param_str should be vmid,a,b,c (where the guest
	* wants a MB aperture and b MB gm, and c fence registers) or -vmid
	* (where we want to release the vgt instance).
	*/
	(void)sscanf(buf, "%63s", param_str);
	param_cnt = sscanf(param_str, "%d,%d,%d,%d,%d", &vp.vm_id, &low_gm_sz,
		&high_gm_sz, &vp.fence_sz, &vp.vgt_primary);
	vp.aperture_sz = low_gm_sz;
	vp.gm_sz = high_gm_sz + low_gm_sz;

	if (param_cnt == 1) {
		if (vp.vm_id >= 0)
			return -EINVAL;
	} else if (param_cnt == 4 || param_cnt == 5) {
		if (!(vp.vm_id > 0 && vp.aperture_sz > 0 &&
			vp.aperture_sz <= vp.gm_sz && vp.fence_sz > 0))
			return -EINVAL;

		if (param_cnt == 5) {
			/* -1/0/1 means: not-specified, non-primary, primary */
			if (vp.vgt_primary < -1 && vp.vgt_primary > 1)
				return -EINVAL;
		} else {
			vp.vgt_primary = -1; /* no valid value specified. */
		}
	} else
		return -EINVAL;

	cpu = vgt_enter();
	mutex_lock(&vgt_sysfs_lock);
	rc = (vp.vm_id > 0) ? vgt_add_state_sysfs(vp) : vgt_del_state_sysfs(vp);
	mutex_unlock(&vgt_sysfs_lock);
	vgt_exit(cpu);

	return rc < 0 ? rc : count;
}

static ssize_t vgt_display_owner_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct pgt_device *pdev = &default_device;
	return sprintf(buf,"%d\n", current_display_owner(pdev)->vm_id);
}

static ssize_t vgt_display_owner_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	int vmid;
	if (sscanf(buf, "%d", &vmid) != 1)
		return -EINVAL;

	if (vmid != 0) {
		vgt_warn("Cannot change display_owner to vms other than domain0!\n");
	}

	return count;
}

static ssize_t vgt_foreground_vm_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf,"%d\n", current_foreground_vm((&default_device))->vm_id);
}

static ssize_t vgt_foreground_vm_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long flags;
	int ret = count;
	int vmid;
	struct vgt_device *next_vgt;
	struct pgt_device *pdev;
	int cpu;

	if (sscanf(buf, "%d", &vmid) != 1)
		return -EINVAL;

	cpu = vgt_enter();
	mutex_lock(&vgt_sysfs_lock);

	spin_lock_irqsave(&default_device.lock, flags);

	next_vgt = vmid_2_vgt_device(vmid);
	if (next_vgt == NULL) {
		printk("vGT: can not find the vgt instance of dom%d!\n", vmid);
		ret = -ENODEV;
		goto out;
	}

	pdev = next_vgt->pdev;
	if (current_foreground_vm(pdev) == next_vgt) {
		goto out;
	}

	if (!__vreg(next_vgt, vgt_info_off(display_ready))) {
		printk("VGT %d: Display is not ready.\n", vmid);
		ret = -EAGAIN;
		goto out;
	}

	pdev->next_foreground_vm = next_vgt;
	vgt_raise_request(pdev, VGT_REQUEST_DPY_SWITCH);
out:
	spin_unlock_irqrestore(&default_device.lock, flags);

	mutex_unlock(&vgt_sysfs_lock);

	vgt_exit(cpu);
	return ret;
}

static ssize_t vgt_display_pointer_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int vmid;

	if (sscanf(buf, "%du", &vmid) != 1)
		return -EINVAL;

	mutex_lock(&vgt_sysfs_lock);
	vgt_set_display_pointer(vmid);
	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t vgt_display_pointer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	ssize_t ret;

	mutex_lock(&vgt_sysfs_lock);
	ret = vgt_get_display_pointer(buf);
	mutex_unlock(&vgt_sysfs_lock);

	return ret;
}

static ssize_t vgt_ctx_switch_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int val;
	bool enabled;

	if (sscanf(buf, "%du", &val) != 1)
		return -EINVAL;
	enabled = !!val;
	vgt_toggle_ctx_switch(enabled);
	return count;
}

static ssize_t vgt_validate_ctx_switch_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%du", &val) != 1)
		return -EINVAL;
	vgt_validate_ctx_switch = !!val;
	return count;
}

static ssize_t vgt_ctx_switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "VGT context switch: %s\n",
			vgt_ctx_switch ? "enabled" : "disabled");
}

static ssize_t vgt_validate_ctx_switch_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "VGT check mmio restore: %s\n",
			vgt_validate_ctx_switch ? "enabled" : "disabled");
}

static ssize_t vgt_dpy_switch_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

//	fastpath_dpy_switch = !!val;
	fastpath_dpy_switch = true;
	return count;
}

static ssize_t vgt_dpy_switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "VGT display_owner switch: using the %s\n",
				fastpath_dpy_switch ?
				"fast-path method. (write 0 to use the slow-path method)"
				: "slow-path method. (write 1 to use the fast-path method)");
}

 static ssize_t vgt_available_res_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct pgt_device *pdev = &default_device;
	ssize_t buf_len;
	int cpu;

	cpu = vgt_enter();

	mutex_lock(&vgt_sysfs_lock);
	spin_lock_irq(&pdev->lock);
	buf_len = get_avl_vm_aperture_gm_and_fence(pdev, buf,
			PAGE_SIZE);
	spin_unlock_irq(&pdev->lock);
	mutex_unlock(&vgt_sysfs_lock);

	vgt_exit(cpu);
	return buf_len;
}


static ssize_t vgt_hot_plug_reader(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	/* read the hot-plug node, do nothing */
	return 0;
}

/* bit field definition of the hot_plug trigger value:
 *
 * bit 31 - bit 16	: Reserved;
 * bit 15 - bit 8	: vmid;
 * bit 7 - bit 4	: Reserved;
 * bit 3 - bit 1	: port/monitor selection:
 *		0	-	CRT
 *		1	-	PORT_A
 *		2	-	PORT_B
 *		3	-	PORT_C
 *		4	-	PORT_D
 * bit 0 - bit 0	: Direction.
 *		0: pull out;
 *		1: plug in;
 */
static ssize_t vgt_hot_plug_trigger(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned hotplug_cmd = 0;
	int cpu;

	if (sscanf(buf, "%i", &hotplug_cmd) != 1)
		return -EINVAL;
	cpu = vgt_enter();
	vgt_trigger_display_hot_plug(&default_device, (vgt_hotplug_cmd_t)hotplug_cmd);
	vgt_exit(cpu);
	return count;
}

static struct kobj_attribute create_vgt_instance_attrs =
	__ATTR(create_vgt_instance, 0220, NULL, vgt_create_instance_store);
static struct kobj_attribute display_owner_ctrl_attrs =
	__ATTR(display_owner, 0660, vgt_display_owner_show, vgt_display_owner_store);
static struct kobj_attribute foreground_vm_ctrl_attrs =
	__ATTR(foreground_vm, 0660, vgt_foreground_vm_show, vgt_foreground_vm_store);
static struct kobj_attribute display_pointer_attrs =
	__ATTR(display_pointer, 0660, vgt_display_pointer_show, vgt_display_pointer_store);

static struct kobj_attribute hot_plug_event_attrs =
	__ATTR(virtual_event, 0660, vgt_hot_plug_reader, vgt_hot_plug_trigger);

static struct kobj_attribute ctx_switch_attrs =
	__ATTR(ctx_switch, 0660, vgt_ctx_switch_show, vgt_ctx_switch_store);

static struct kobj_attribute validate_ctx_switch_attrs =
	__ATTR(validate_ctx_switch, 0660, vgt_validate_ctx_switch_show, vgt_validate_ctx_switch_store);

static struct kobj_attribute dpy_switch_attrs =
	__ATTR(display_switch_method, 0660, vgt_dpy_switch_show, vgt_dpy_switch_store);

static struct kobj_attribute available_res_attrs =
	__ATTR(available_resource, 0440, vgt_available_res_show, NULL);

static struct attribute *vgt_ctrl_attrs[] = {
	&create_vgt_instance_attrs.attr,
	&display_owner_ctrl_attrs.attr,
	&foreground_vm_ctrl_attrs.attr,
	&display_pointer_attrs.attr,
	&hot_plug_event_attrs.attr,
	&ctx_switch_attrs.attr,
	&validate_ctx_switch_attrs.attr,
	&dpy_switch_attrs.attr,
	&available_res_attrs.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/* copied code from here */
static ssize_t kobj_attr_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);
	return ret;
}

static ssize_t kobj_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->store)
		ret = kattr->store(kobj, kattr, buf, count);
	return ret;
}

const struct sysfs_ops vgt_kobj_sysfs_ops = {
	.show	= kobj_attr_show,
	.store	= kobj_attr_store,
};


/* copied code end */

#define kobj_to_vgt(kobj) container_of((kobj), struct vgt_device, kobj)

static ssize_t vgt_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%x\n", vgt->vgt_id);
}

static ssize_t gm_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->gm_sz);
}

static ssize_t aperture_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->aperture_sz);
}

static ssize_t aperture_base_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->aperture_base);
}

static ssize_t aperture_base_va_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%p\n", vgt->aperture_base_va);
}

static struct kobj_attribute vgt_id_attribute =
	__ATTR_RO(vgt_id);

static struct kobj_attribute gm_sz_attribute =
	__ATTR_RO(gm_sz);

static struct kobj_attribute aperture_sz_attribute =
	__ATTR_RO(aperture_sz);

static struct kobj_attribute aperture_base_attribute =
	__ATTR_RO(aperture_base);

static struct kobj_attribute aperture_base_va_attribute =
	__ATTR_RO(aperture_base_va);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *vgt_instance_attrs[] = {
	&vgt_id_attribute.attr,
	&gm_sz_attribute.attr,
	&aperture_sz_attribute.attr,
	&aperture_base_attribute.attr,
	&aperture_base_va_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
#if 0
static struct attribute_group attr_group = {
	.attrs = attrs,
};
#endif

static struct kobj_type vgt_instance_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops	= &vgt_kobj_sysfs_ops,
	.default_attrs = vgt_instance_attrs,
};

static struct kobj_type vgt_ctrl_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops  = &vgt_kobj_sysfs_ops,
	.default_attrs = vgt_ctrl_attrs,
};

static ssize_t
igd_mmio_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct pgt_device *pdev = &default_device;
	size_t init_count = count, len;
	unsigned long data;
	int cpu;

	if (!count || off < 0 || off + count > bin_attr->size || (off & 0x3))
		return -EINVAL;

	cpu = vgt_enter();
	spin_lock_irq(&pdev->lock);

	while (count > 0) {
		len = (count > sizeof(unsigned long)) ? sizeof(unsigned long) :
				count;

		if (hcall_mmio_read(_vgt_mmio_pa(pdev, off), len, &data) !=
				X86EMUL_OKAY) {
			spin_unlock_irq(&pdev->lock);
			vgt_exit(cpu);
			return -EIO;
		}

		memcpy(buf, &data, len);
		buf += len;
		count -= len;
	}

	spin_unlock_irq(&pdev->lock);
	vgt_exit(cpu);

	return init_count;
}

static ssize_t
igd_mmio_write(struct file* filp, struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct pgt_device *pdev = &default_device;
	size_t init_count = count, len;
	unsigned long data;
	int cpu;

	if (!count || off < 0 || off + count > bin_attr->size || (off & 0x3))
		return -EINVAL;

	cpu = vgt_enter();
	spin_lock_irq(&pdev->lock);

	while (count > 0) {
		len = (count > sizeof(unsigned long)) ? sizeof(unsigned long) :
				count;

		memcpy(&data, buf, len);
		if (hcall_mmio_write(_vgt_mmio_pa(pdev, off), len, data) !=
				X86EMUL_OKAY) {
			spin_unlock_irq(&pdev->lock);
			vgt_exit(cpu);
			return -EIO;
		}

		buf += len;
		count -= len;
	}

	spin_unlock_irq(&pdev->lock);
	vgt_exit(cpu);
	return init_count;
}

static struct bin_attribute igd_mmio_attr = {
	.attr =	{
		.name = "igd_mmio",
		.mode = 0660
	},
	.size = VGT_MMIO_SPACE_SZ,
	.read = igd_mmio_read,
	.write = igd_mmio_write,
};


static int vgt_add_state_sysfs(vgt_params_t vp)
{
	int retval;
	struct vgt_device *vgt;
	/*
	* Create a simple kobject located under /sys/kernel/
	* As this is a simple directory, no uevent will be sent to
	* userspace.  That is why this function should not be used for
	* any type of dynamic kobjects, where the name and number are
	* not known ahead of time.
	*/

	ASSERT(vgt_ctrl_kobj);

	/* check if such vmid has been used */
	if (vmid_2_vgt_device(vp.vm_id))
		return -EINVAL;

	retval = create_vgt_instance(&default_device, &vgt, vp);

	if (retval < 0)
		return retval;

	/* init kobject */
	kobject_init(&vgt->kobj, &vgt_instance_ktype);

	/* set it before calling the kobject core */
	vgt->kobj.kset = vgt_kset;

	/* add kobject, NULL parent indicates using kset as parent */
	retval = kobject_add(&vgt->kobj, NULL, "vm%u", vgt->vm_id);
	if (retval) {
		printk(KERN_WARNING "%s: vgt kobject add error: %d\n",
					__func__, retval);
		kobject_put(&vgt->kobj);
	}

	return retval;
}

static int vgt_del_state_sysfs(vgt_params_t vp)
{
	struct vgt_device *vgt;

	vp.vm_id = -vp.vm_id;
	vgt = vmid_2_vgt_device(vp.vm_id);
	if (!vgt)
		return -ENODEV;

	kobject_put(&vgt->kobj);

	vgt_release_instance(vgt);

	return 0;
}

int vgt_init_sysfs(struct pgt_device *pdev)
{
	int ret;

	vgt_kset = kset_create_and_add("vgt", NULL, kernel_kobj);
	if (!vgt_kset) {
		ret = -ENOMEM;
		goto kset_fail;
	}

	vgt_ctrl_kobj = kzalloc(sizeof(struct kobject), GFP_KERNEL);
	if (!vgt_ctrl_kobj) {
		ret = -ENOMEM;
		goto ctrl_fail;
	}

	vgt_ctrl_kobj->kset = vgt_kset;

	ret = kobject_init_and_add(vgt_ctrl_kobj, &vgt_ctrl_ktype, NULL, "control");
	if (ret) {
		ret = -EINVAL;
		goto kobj_fail;
	}

	ret = sysfs_create_bin_file(vgt_ctrl_kobj, &igd_mmio_attr);
	if (ret < 0) {
		ret = -EINVAL;
		goto kobj_fail;
	}

	return 0;

kobj_fail:
	kobject_put(vgt_ctrl_kobj);
ctrl_fail:
	kset_unregister(vgt_kset);
kset_fail:
	return ret;
}

void vgt_destroy_sysfs(void)
{
	sysfs_remove_bin_file(vgt_ctrl_kobj, &igd_mmio_attr);
	kobject_put(vgt_ctrl_kobj);
	kset_unregister(vgt_kset);
}
