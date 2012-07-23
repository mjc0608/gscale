/*
 * vGT sysfs interface (the original code comes from samples/kobject-example.c)
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
/*TODO: clean up unnecessary head files*/
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
//#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/hash.h>
#include <linux/delay.h>
#include <asm/bitops.h>
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>
#include <xen/vgt.h>
#include "vgt_reg.h"

struct kobject *vgt_ctrl_kobj;
struct kset *vgt_kset;
static struct pgt_device *vgt_kobj_priv;
struct vgt_device *vmid_2_vgt_device(int vmid);
static unsigned int query_reg;

static void vgt_kobj_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
	/* FIXME: we do not deallocate our kobject */
	//kfree(kobj);
}

#if 0
static ssize_t vgt_create_instance_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    /* TODO: show some global statistics ? */
	return sprintf(buf, "To be done...\n");
}
#endif

int vgt_add_state_sysfs(int vm_id);
void vgt_del_state_sysfs(int vmid);
static ssize_t vgt_create_instance_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    int vmid;

    /* TODO: scanned value not checked */
    sscanf(buf, "%du", &vmid);
    if (vmid == 0)
        return count;

    if (vmid > 0)
        vgt_add_state_sysfs(vmid);
    else
        vgt_del_state_sysfs(-vmid);
    return count;
}

static ssize_t vgt_display_owner_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    /* TODO: show the current owner ???  */
	return sprintf(buf,"%d\n", current_display_owner(vgt_kobj_priv)->vm_id);
}

struct vgt_device *vmid_2_vgt_device(int vmid);
extern struct vgt_device *next_display_owner;
void do_vgt_display_switch(struct pgt_device *pdev);
static ssize_t vgt_display_owner_store(struct kobject *kobj, struct kobj_attribute *attr,
            const char *buf, size_t count)
{
    int vmid;
    struct vgt_device *next_vgt;
    /* TODO: scanned value not checked */
    sscanf(buf, "%du", &vmid);

    /* FIXME: to avoid nested spin_lock_irq issue, use spin_lock_irqsave instead of spin_lock_irq*/
    //spin_lock_irqsave(&vgt_kobj_priv->lock, flags);
    next_vgt = vmid_2_vgt_device(vmid);
    if (next_vgt) {
        next_display_owner = next_vgt;
		do_vgt_display_switch(vgt_kobj_priv);
	}
    //spin_unlock_irqrestore(&vgt_kobj_priv->lock, flags);

    return count;
}

static ssize_t vgt_display_pointer_store(struct kobject *kobj, struct kobj_attribute *attr,
            const char *buf, size_t count)
{
	int vmid;

	/* TODO: scanned value not checked */
	sscanf(buf, "%du", &vmid);
	vgt_set_display_pointer(vmid);
	return count;
}

static ssize_t vgt_display_pointer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return vgt_get_display_pointer(buf);
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
 * bit 3 - bit 1	: Monitor selection:
 * 		0	-	CRT
 * 		1	-	DP_A
 * 		2	-	DP_B
 * 		3	-	DP_C
 * 		4	-	DP_D
 *		5	-	HDMIB
 *		6	-	HDMIC
 *		7	-	HDMID
 * bit 0 - bit 0	: Direction.
 *		0: pull out;
 *		1: plug in;
 */
static ssize_t vgt_hot_plug_trigger(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned hotplug_cmd = 0;
	sscanf(buf, "%du", &hotplug_cmd);
	vgt_trigger_display_hot_plug(vgt_kobj_priv, hotplug_cmd);
	return count;
}

static ssize_t vgt_reg_owner_store(struct kobject *kobj, struct kobj_attribute *attr,
            const char *buf, size_t count)
{
	unsigned int reg;

	sscanf(buf, "%xu", &reg);
	if ((reg & 0x3) || reg > (VGT_MMIO_SPACE_SZ - 4))
		return 0;

	query_reg = reg;

	return count;
}

static ssize_t vgt_reg_owner_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	unsigned int bit, reg;
	bool found_owner = false;
	enum vgt_owner_type type;
	struct pgt_device *pdev = vgt_kobj_priv;
	struct vgt_device *vgt;

	reg = query_reg;
	type = pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER;

	for_each_set_bit(bit, &vgt_id_alloc_bitmap, (8 * sizeof(unsigned long))) {
        vgt = pdev->device[bit];
        if (vgt == vgt_get_owner(pdev, type)) {
			found_owner = true;
			break;
		}
	}

	return sprintf(buf,"Reg(%08x) ownership info: \n"
			"Type #:              [%03d]\n"
			"INVALID:             [%-3s]\n"
			"GLOBAL:              [%-3s]\n"
			"RCS:                 [%-3s]\n"
			"BCS:                 [%-3s]\n"
			"VCS:                 [%-3s]\n"
			"RENDER:              [%-3s]\n"
			"DISPLAY:             [%-3s]\n"
			"PM:                  [%-3s]\n"
			"MGMT:                [%-3s]\n\n"
			"Owner Domain         [%08x]\n",
			reg,
			type,
			(type == VGT_OT_INVALID) ? "yes" : "no",
			(type == VGT_OT_GLOBAL) ? "yes" : "no",
			(type == VGT_OT_RCS) ? "yes" : "no",
			(type == VGT_OT_BCS) ? "yes" : "no",
			(type == VGT_OT_VCS) ? "yes" : "no",
			(type == VGT_OT_RENDER) ? "yes" : "no",
			(type == VGT_OT_DISPLAY) ? "yes" : "no",
			(type == VGT_OT_PM) ? "yes" : "no",
			(type == VGT_OT_MGMT) ? "yes" : "no",
			found_owner ? vgt->vgt_id : 0xdeadbeef
			);
}

static struct kobj_attribute create_vgt_instance_attrs =
	__ATTR(create_vgt_instance, 0220, NULL, vgt_create_instance_store);
static struct kobj_attribute display_owner_ctrl_attrs =
	__ATTR(display_owner, 0660, vgt_display_owner_show, vgt_display_owner_store);

static struct kobj_attribute display_pointer_attrs =
	__ATTR(display_pointer, 0660, vgt_display_pointer_show, vgt_display_pointer_store);

static struct kobj_attribute hot_plug_event_attrs =
	__ATTR(hot_plug_event, 0660, vgt_hot_plug_reader, vgt_hot_plug_trigger);

static struct kobj_attribute reg_owner_attrs =
	__ATTR(reg_owner, 0660, vgt_reg_owner_show, vgt_reg_owner_store);

static struct attribute *vgt_ctrl_attrs[] = {
	&create_vgt_instance_attrs.attr,
	&display_owner_ctrl_attrs.attr,
	&display_pointer_attrs.attr,
	&hot_plug_event_attrs.attr,
	&reg_owner_attrs.attr,
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
	&gm_sz_attribute.attr,
	&aperture_sz_attribute.attr,
	&aperture_base_attribute.attr,
	&aperture_base_va_attribute.attr,
	//&reg_owner_attribute.attr,
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
    .release    = vgt_kobj_release,
    .sysfs_ops  = &vgt_kobj_sysfs_ops,
    .default_attrs = vgt_ctrl_attrs,
};


int vgt_add_state_sysfs(int vm_id)
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
	if (vmid_2_vgt_device(vm_id))
		return -EINVAL;

	vgt = create_vgt_instance(vgt_kobj_priv, vm_id);
	if (vgt == NULL)
		return -1;

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

void vgt_del_state_sysfs(int vmid)
{
	struct vgt_device *vgt;
	vgt = vmid_2_vgt_device(vmid);
	if (!vgt)
		return;

	kobject_put(&vgt->kobj);
	vgt_release_instance(vgt);
}


int vgt_init_sysfs(struct pgt_device *pdev)
{
    int retval;

    vgt_kset = kset_create_and_add("vgt", NULL, kernel_kobj);
    if (!vgt_kset)
        return -ENOMEM;

    vgt_ctrl_kobj = kzalloc(sizeof(struct kobject), GFP_KERNEL);
    if (!vgt_ctrl_kobj)
        return -ENOMEM;

    vgt_ctrl_kobj->kset = vgt_kset;

    retval = kobject_init_and_add(vgt_ctrl_kobj, &vgt_ctrl_ktype, NULL, "control");
    if (retval) {
        kobject_put(vgt_ctrl_kobj);
        return -EINVAL;
    }

    vgt_kobj_priv = pdev;
    return 0;
}
