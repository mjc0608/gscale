/*
 * vGT debugfs
 *
 * WARNING: This file is provided under a GPL license since some codes
 * copied from some drivers.
 *
 * When using or
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
/* TODO: clean up redundant head files */
/* TODO: this file's code copied from arch/x86/xen/debugfs.c */
#include <linux/linkage.h>
#include <linux/module.h>
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
#include <asm/system.h>
#include <xen/vgt.h>
#include "vgt_reg.h"
#include <linux/debugfs.h>
#include <linux/module.h>

/* Maximum lenth of stringlized integer is 10 */
#define MAX_VM_NAME_LEN (3 + 10)
static struct dentry *d_vgt_debug;
static struct dentry *d_per_vgt[VGT_MAX_VMS];
static char vm_dir_name[VGT_MAX_VMS][MAX_VM_NAME_LEN];

struct array_data
{
	void *array;
	unsigned elements;
};

static int u32_array_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return nonseekable_open(inode, file);
}

/* TODO: this is generic function, can used to format ring_buffer and etc. */
static size_t format_array(char *buf, size_t bufsize, const char *fmt,
			   u32 *array, unsigned array_size)
{
	size_t ret = 0;
	unsigned i;

	for(i = 0; i < array_size; i++) {
		size_t len;

		if (i % 16 == 0) {
			len = snprintf(buf, bufsize, "0x%x:",i*4);
			ret += len;

			if (buf) {
				buf += len;
				bufsize -= len;
			}
		}

		len = snprintf(buf, bufsize, fmt, array[i]);
		len++;	/* ' ' or '\n' */
		ret += len;

		if (buf) {
			buf += len;
			bufsize -= len;
			buf[-1] = ((i + 1) % 16 == 0) ? '\n' : ' ';
		}
	}

	ret++;		/* \0 */
	if (buf)
		*buf = '\0';

	return ret;
}

/*TODO: what is this function used for ??? */
static char *format_array_alloc(const char *fmt, u32 *array, unsigned array_size)
{
	/* very tricky way */
	size_t len = format_array(NULL, 0, fmt, array, array_size);
	char *ret;

	ret = kmalloc(len, GFP_KERNEL);
	if (ret == NULL)
		return NULL;

	format_array(ret, len, fmt, array, array_size);
	return ret;
}

/* TODO: data copied from kernel space to user space */
static ssize_t u32_array_read(struct file *file, char __user *buf, size_t len,
			      loff_t *ppos)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct array_data *data = inode->i_private;
	size_t size;

	if (*ppos == 0) {
		if (file->private_data) {
			kfree(file->private_data);
			file->private_data = NULL;
		}

		file->private_data = format_array_alloc("%x", data->array, data->elements);
	}

	size = 0;
	if (file->private_data)
		size = strlen(file->private_data);

	return simple_read_from_buffer(buf, len, ppos, file->private_data, size);
}

static int vgt_array_release(struct inode *inode, struct file *file)
{
    kfree(file->private_data);

    return 0;
}

static const struct file_operations u32_array_fops = {
	.owner	= THIS_MODULE,
	.open	= u32_array_open,
	.release= vgt_array_release,
	.read	= u32_array_read,
	.llseek = no_llseek,
};

static struct dentry *vgt_debugfs_create_u32_array(const char *name, mode_t mode,
					    struct dentry *parent,
					    u32 *array, unsigned elements)
{
	struct array_data *data = kmalloc(sizeof(*data), GFP_KERNEL);

	if (data == NULL)
		return NULL;

	data->array = array;
	data->elements = elements;

	return debugfs_create_file(name, mode, parent, data, &u32_array_fops);
}

/* TODO: dummy function, ringbuffer specific read
 * operation should involved in file operations */
static struct dentry *vgt_debugfs_create_blob(const char *name, mode_t mode,
					    struct dentry *parent,
					    u32 *array, unsigned elements)
{
	struct array_data *data = kmalloc(sizeof(*data), GFP_KERNEL);

	if (data == NULL)
		return NULL;

	data->array = array;
	data->elements = elements;

	return debugfs_create_file(name, mode, parent, data, &u32_array_fops);
}

/* TODO: initialize vGT debufs top directory */
/* FIXME: how about the second graphics card */
struct dentry *vgt_init_debugfs(void)
{
    if (!d_vgt_debug) {
        d_vgt_debug = debugfs_create_dir("vgt", NULL);

        if (!d_vgt_debug)
            pr_warning("Could not create 'vgt' debugfs directory\n");
    }

    return d_vgt_debug;
}

int vgt_create_debugfs(struct vgt_device *vgt)
{
	int vgt_id = vgt->vgt_id;
	int retval;
	struct dentry *d_vmmio, *d_smmio,
				  *d_vfb_a, *d_sfb_a,
				  *d_vfb_b, *d_sfb_b;

    struct pgt_device *pdev = vgt->pdev;

	unsigned int dspa_surf_size = VGT_MMIO_READ(pdev, _REG_DSPASIZE);
	unsigned int dspb_surf_size = VGT_MMIO_READ(pdev, _REG_DSPBSIZE);

	printk("vGT(%d): Display surface A size is %d\n", dspa_surf_size);
	printk("vGT(%d): Display surface B size is %d\n", dspb_surf_size);

	ASSERT(vgt);
	ASSERT(d_vgt_debug);

	retval = sprintf(vm_dir_name[vgt_id], "vm%d", vgt->vm_id);
	if (retval <= 0) {
		printk(KERN_ERR "vGT: failed to generating dirname:  vm%d\n", vgt->vm_id);
		return -EINVAL;
	}
	/* create vm directory */
	d_per_vgt[vgt_id] = debugfs_create_dir(vm_dir_name[vgt_id], d_vgt_debug);
	if (d_per_vgt[vgt_id] == NULL) {
		printk(KERN_ERR "vGT: creation faiure for debugfs directory: vm%d\n", vgt->vm_id);
		return -EINVAL;
	}

	/* TODO: create debugfs file per vgt as you like */

	/* virtual mmio space dump */
	d_vmmio = vgt_debugfs_create_blob("virtual_mmio_space",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)(vgt->state.vReg),
			VGT_MMIO_SPACE_SZ/(sizeof(u32)));

	if (!d_vmmio)
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: virtual_mmio_space\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: virtual_mmio_space\n", vgt_id);


	d_smmio = vgt_debugfs_create_blob("shadow_mmio_space",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)(vgt->state.sReg),
			VGT_MMIO_SPACE_SZ/(sizeof(u32)));

	if (!d_smmio)
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: shadow_mmio_space\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: shadow_mmio_space\n", vgt_id);

	d_sfb_b = vgt_debugfs_create_blob("shadow_surfB_fb",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)((void *)(vgt->state.sReg) + _REG_DSPBSURF),
			1024*1024/4);

	if (!d_sfb_b)
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: shadow_surfB_fb\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: shadow_surfB_fb\n", vgt_id);

	d_sfb_a = vgt_debugfs_create_blob("shadow_surfA_fb",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)((void *)(vgt->state.sReg) + _REG_DSPASURF),
			1024*1024/4);

	if (!d_sfb_a)
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: shadow_surfA_fb\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: shadow_surfA_fb\n", vgt_id);


	return 0;
}

