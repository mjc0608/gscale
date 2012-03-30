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
static struct vgt_device *current_vgt;

static ssize_t gm_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "%016llx\n", current_vgt->gm_sz);
}

static ssize_t aperture_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "%016llx\n", current_vgt->aperture_sz);
}

static ssize_t aperture_base_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "%016llx\n", current_vgt->aperture_base);
}

static ssize_t aperture_base_va_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "%p\n", current_vgt->aperture_base_va);
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
#define MAX_VGT_NAME_LENGTH 7
    /* FIXME: current legal name vgt_0 ~ vgt_99 */
    char vgt_name[MAX_VGT_NAME_LENGTH];
	/*
	 * Create a simple kobject located under /sys/kernel/
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
    retval = snprintf(vgt_name, MAX_VGT_NAME_LENGTH, "vgt_%d", vgt->vgt_id);
    /* The vgt_name is truncated */
    if (retval >= MAX_VGT_NAME_LENGTH)
        return -EINVAL;

	vgt->kobj = kobject_create_and_add(vgt_name, kernel_kobj);
	if (!vgt->kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(vgt->kobj, &attr_group);
	if (retval)
		kobject_put(vgt->kobj);

    current_vgt = vgt;

	return retval;
}

void vgt_del_state_sysfs(struct vgt_device *vgt)
{
    kobject_put(vgt->kobj);
}
