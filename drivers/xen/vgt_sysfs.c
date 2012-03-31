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
/*
 * This module shows how to create a simple subdirectory in sysfs called
 * /sys/kernel/kobject-example  In that directory, 3 files are created:
 * "foo", "baz", and "bar".  If an integer is written to these files, it can be
 * later read out of it.
 */
static struct kobject *vgt_kobj;
static int vgt_instance_cnt = 1;

static int vgt_create_topdir_kobject(void)
{
    /* Place vgt directory under /sys/kernel */
    vgt_kobj = kobject_create_and_add("vgt", kernel_kobj);
    if (!vgt_kobj)
        return -ENOMEM;

    return 0;
}

int vgt_add_state_sysfs(struct vgt_device *vgt);
static struct vgt_device *dummy_create_vgt_instance(int vgt_id)
{
    /* FIXME: dummy_vgt will be lost tracking, so there are memory leak
     * here
     */
    struct vgt_device *dummy_vgt = kmalloc(sizeof(*dummy_vgt), GFP_KERNEL);
    if (dummy_vgt == NULL) {
        printk("Insufficient memory for vgt_device dummy_vgt\n");
        return NULL;
    } else {
        dummy_vgt->vgt_id = vgt_id;
        dummy_vgt->gm_sz = 0xdeadbeef;
        dummy_vgt->aperture_sz = 0xdeadbeef;
        dummy_vgt->aperture_base = 0xbeefdead;
        dummy_vgt->aperture_base_va = (void*)0xbeefdead;
        vgt_add_state_sysfs(dummy_vgt);
    }

    return dummy_vgt;
}

static ssize_t vgt_create_instance_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    /* TODO: show some global statistics ? */
	return sprintf(buf, "%d\n", vgt_instance_cnt);
}

static ssize_t vgt_create_instance_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    int val;
    /* TODO: scanned value not checked */
	sscanf(buf, "%du", &val);
    if (val > 0) {
        dummy_create_vgt_instance(vgt_instance_cnt);
        vgt_instance_cnt++;
    } else {
        printk(KERN_WARNING"vGT sysfs node create error: value should be an posivive integer\n" );
    }
	return count;
}

static struct kobj_attribute create_vgt_instance_attrs =
	__ATTR(create_vgt_instance, 0666, vgt_create_instance_show, vgt_create_instance_store);

static struct attribute *ctl_attrs[] = {
	&create_vgt_instance_attrs.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group ctl_attr_group = {
	.attrs = ctl_attrs,
};

int vgt_init_sysfs(void)
{
    int retval;

    vgt_kobj = kobject_create_and_add("vgt", kernel_kobj);
    if (!vgt_kobj)
        return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(vgt_kobj, &ctl_attr_group);
	if (retval) {
		kobject_put(vgt_kobj);
        return retval;
    }

    return 0;
}

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

static void vgt_kobj_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
    /* FIXME: we do not deallocate our kobject */
	//kfree(kobj);
}

static struct kobj_type vgt_kobj_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops	= &vgt_kobj_sysfs_ops,
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
static struct attribute *attrs[] = {
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
static struct attribute_group attr_group = {
	.attrs = attrs,
};

int vgt_add_state_sysfs(struct vgt_device *vgt)
{
	int retval;
	/*
	 * Create a simple kobject located under /sys/kernel/
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */

    if (!vgt_kobj) {
        retval = vgt_create_topdir_kobject();
        if (retval < 0)
            return retval;
    }

    /* init kobject */
	kobject_init(&vgt->kobj, &vgt_kobj_ktype);

    /* add kobject */
    retval = kobject_add(&vgt->kobj, vgt_kobj, "vgt%d", vgt->vgt_id);
    if (retval) {
        printk(KERN_WARNING "%s: vgt kobject add error: %d\n",
                __func__, retval);
        kobject_put(&vgt->kobj);
    }

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(&vgt->kobj, &attr_group);
	if (retval)
		kobject_put(&vgt->kobj);

	return retval;
}

void vgt_del_state_sysfs(struct vgt_device *vgt)
{
    kobject_put(&vgt->kobj);
}
