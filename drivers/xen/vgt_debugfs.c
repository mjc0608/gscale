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
#include <xen/vgt.h>
#include "vgt_reg.h"
#include <linux/debugfs.h>
#include <linux/module.h>

/* Maximum lenth of stringlized integer is 10 */
#define MAX_VM_NAME_LEN (3 + 10)
enum vgt_debugfs_entry_t
{
	VGT_DEBUGFS_SURFA_FB = 0,
	VGT_DEBUGFS_SURFB_FB,
	VGT_DEBUGFS_SURFC_FB,
	VGT_DEBUGFS_SURFA_BASE,
	VGT_DEBUGFS_SURFB_BASE,
	VGT_DEBUGFS_SURFC_BASE,
	VGT_DEBUGFS_VIRTUAL_MMIO,
	VGT_DEBUGFS_SHADOW_MMIO,
	VGT_DEBUGFS_ENTRY_MAX
};

static debug_statistics_t  stat_info [] = {
	{ "gtt_mmio_rcnt", &gtt_mmio_rcnt },
	{ "gtt_mmio_wcnt", &gtt_mmio_wcnt },
	{ "gtt_mmio_wcycles", &gtt_mmio_wcycles },
	{ "gtt_mmio_rcycles", &gtt_mmio_rcycles },
	{ "mmio_rcnt", &mmio_rcnt },
	{ "mmio_wcnt", &mmio_wcnt },
	{ "mmio_wcycles", &mmio_wcycles },
	{ "mmio_rcycles", &mmio_rcycles },
	{ "ring_mmio_rcnt", &ring_mmio_rcnt },
	{ "ring_mmio_wcnt", &ring_mmio_wcnt },
	{ "ring_tail_mmio_wcnt", &ring_tail_mmio_wcnt },
	{ "ring_tail_mmio_wcycles", &ring_tail_mmio_wcycles },
	{ "", NULL}
};

static struct dentry *d_vgt_debug;
static struct dentry *d_per_vgt[VGT_MAX_VMS];
static struct dentry *d_debugfs_entry[VGT_MAX_VMS][VGT_DEBUGFS_ENTRY_MAX];
static char vm_dir_name[VGT_MAX_VMS][MAX_VM_NAME_LEN];

/* TODO: sometimes domain will change their framebuffer like from fbconsole to X mode,
 * and at this time, */
void *dsp_surf_base[VGT_MAX_VMS][I915_MAX_PIPES];
unsigned int dsp_surf_size[VGT_MAX_VMS][I915_MAX_PIPES];
enum vgt_pipe surf_used_pipe;

struct array_data
{
	void *array;
	unsigned elements;
};
struct array_data vgt_debugfs_data[VGT_MAX_VMS][VGT_DEBUGFS_ENTRY_MAX];

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

#if 0
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
#endif

static struct dentry *vgt_debugfs_create_blob(const char *name, mode_t mode,
					    struct dentry *parent,
					    struct array_data *p)
{
	ASSERT(p);
	ASSERT(p->array);

	return debugfs_create_file(name, mode, parent, p, &u32_array_fops);
}

/* TODO: initialize vGT debufs top directory */
/* FIXME: how about the second graphics card */
struct dentry *vgt_init_debugfs(void)
{
	struct dentry *temp_d;
	int   i;

    if (!d_vgt_debug) {
        d_vgt_debug = debugfs_create_dir("vgt", NULL);

        if (!d_vgt_debug)
            pr_warning("Could not create 'vgt' debugfs directory\n");
    }

    if ( d_vgt_debug )
	for ( i = 0; stat_info[i].stat != NULL; i++ ) {
		temp_d = debugfs_create_u64(stat_info[i].node_name,
			0444,
			d_vgt_debug,
			stat_info[i].stat);
		if (!temp_d)
			printk(KERN_ERR "Failed to create debugfs node %s\n",
				stat_info[i].node_name);
	}

    return d_vgt_debug;
}

#ifdef VGT_DEBUGFS_DUMP_FB
/* When surface A/B base address or size changed use this function
 * to update fb debugfs, and since the update for surface base
 * indicate the completion of fb update so only reconstruct debugfs
 * when detect such changes. FIXME: only support surface A right now
 */
#define _REG_SURF_BASE(p)	((p) == PIPE_A ? _REG_DSPASURF : _REG_DSPBSURF)
#define _REG_SURF_SZ(p)	((p) == PIPE_A ? _REG_DSPASIZE : _REG_DSPBSIZE)
#define _VGT_DEBUGFS_FB(p) ((p) == PIPE_A ? VGT_DEBUGFS_SURFA_FB : VGT_DEBUGFS_SURFB_FB)
#define vgt_fb_name(p)	((p) == PIPE_A ? "surfA_fb" : "surfB_fb")
static void fb_debugfs_work_func(struct work_struct *work)
{
	vgt_reg_t surf_sz;
	void *surf_base;
	struct array_data *p;
	struct vgt_device *vgt = container_of(work, struct vgt_device, fb_debugfs_work);
	struct pgt_device *pdev = vgt->pdev;
	int vgt_id = vgt->vgt_id;
	enum vgt_pipe pipe = surf_used_pipe;

	ASSERT(pipe != PIPE_C);

	surf_base = phys_aperture_vbase(pdev) + ((__sreg(vgt, _REG_SURF_BASE(pipe))) & PAGE_MASK);
	surf_sz = __sreg(vgt, _REG_SURF_SZ(pipe));

	if (!surf_base)
		return;

	/* FIXME: Not sure why i915 just does not use _REG_DSPASIZE or
	 * _REG_DSPBSIZE to store the fb size, defualt in fb console
	 * mode, the size is 5763072 (4096 * 1407) */
	if (surf_sz == 0) {
		surf_sz = 4 * 1024 * 1024;
		dprintk("vGT(%d): debugfs read 0 from _REG_DSPASIZE, use default surface size(0x%x)\n", vgt_id, surf_sz);
	}

	if (surf_base != dsp_surf_base[vgt_id][pipe]) {

		/* destroy current debugfs node */
		debugfs_remove(d_debugfs_entry[vgt_id][_VGT_DEBUGFS_FB(pipe)]);

		dsp_surf_base[vgt_id][pipe] = surf_base;
		/* The urgly side of debugfs is, each debugfs file
		 * node need a exclusive struct array_data, vgt_debugfs_
		 * data is used for this purpose; When destroy & recreate debugfs
		 * file, this global array can be reused and no memory leak.
		 */
		p = &vgt_debugfs_data[vgt_id][_VGT_DEBUGFS_FB(pipe)];
		p->array = (u32*)dsp_surf_base[vgt_id][pipe];
		p->elements = surf_sz/sizeof(u32);
		d_debugfs_entry[vgt_id][_VGT_DEBUGFS_FB(pipe)] = vgt_debugfs_create_blob(vgt_fb_name(pipe),
				0444,
				d_per_vgt[vgt_id],
				p);
		dprintk("vGT(%d): create debugfs file: %s [base(%p), size(%d)]\n",
				vgt_id,
				vgt_fb_name(pipe),
				surf_base,
				surf_sz);
	}
}
#endif

int vgt_create_debugfs(struct vgt_device *vgt)
{
	int retval;
	struct array_data *p;
	int vgt_id = vgt->vgt_id;
	struct pgt_device *pdev = vgt->pdev;

#ifdef VGT_DEBUGFS_DUMP_FB
	INIT_WORK(&vgt->fb_debugfs_work, fb_debugfs_work_func);
#endif

	dsp_surf_size[vgt_id][PIPE_A] = __sreg(vgt, _REG_DSPASIZE);
	dsp_surf_size[vgt_id][PIPE_B] = __sreg(vgt, _REG_DSPBSIZE);

	dsp_surf_base[vgt_id][PIPE_A] = phys_aperture_vbase(pdev) + ((__sreg(vgt, _REG_DSPASURF)) & PAGE_MASK);
	dsp_surf_base[vgt_id][PIPE_A] = phys_aperture_vbase(pdev) + ((__sreg(vgt, _REG_DSPBSURF)) & PAGE_MASK);

	printk("vGT(%d): Display surface A va(%p) size(%d)\n", vgt_id, dsp_surf_base[vgt_id][PIPE_A], dsp_surf_size[vgt_id][PIPE_A]);
	printk("vGT(%d): Display surface B va(%p) size(%d)\n", vgt_id, dsp_surf_base[vgt_id][PIPE_B], dsp_surf_size[vgt_id][PIPE_B]);

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
	p = &vgt_debugfs_data[vgt_id][VGT_DEBUGFS_VIRTUAL_MMIO];
	p->array = (u32 *)(vgt->state.vReg);
	p->elements = VGT_MMIO_SPACE_SZ/(sizeof(u32));
	d_debugfs_entry[vgt_id][VGT_DEBUGFS_VIRTUAL_MMIO] = vgt_debugfs_create_blob("virtual_mmio_space",
			0444,
			d_per_vgt[vgt_id],
			p);

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_VIRTUAL_MMIO])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: virtual_mmio_space\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: virtual_mmio_space\n", vgt_id);


	p = &vgt_debugfs_data[vgt_id][VGT_DEBUGFS_SHADOW_MMIO];
	p->array = (u32 *)(vgt->state.sReg);
	p->elements = VGT_MMIO_SPACE_SZ/(sizeof(u32));
	d_debugfs_entry[vgt_id][VGT_DEBUGFS_SHADOW_MMIO] = vgt_debugfs_create_blob("shadow_mmio_space",
			0444,
			d_per_vgt[vgt_id],
			p);

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_SHADOW_MMIO])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: shadow_mmio_space\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: shadow_mmio_space\n", vgt_id);

	/* surface B is not used for boot, empty framebuffer cannot be used for debugfs */
#if 0
	p = &vgt_debugfs_data[vgt_id][VGT_DEBUGFS_SURFB_FB];
	p->array = (u32*)dsp_surf_base[vgt_id][PIPE_B];
	p->elements = 1024*1024/4;
	d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFB_FB] = vgt_debugfs_create_blob("surfB_fb",
			0444,
			d_per_vgt[vgt_id],
			p);

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFB_FB])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: fb of surface B\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: fb of surface B\n", vgt_id);
#endif

#ifdef VGT_DEBUGFS_DUMP_FB
	p = &vgt_debugfs_data[vgt_id][VGT_DEBUGFS_SURFA_FB];
	p->array = (u32*)dsp_surf_base[vgt_id][PIPE_A];
	p->elements = 1024*1024/4;
	d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFA_FB] = vgt_debugfs_create_blob("surfA_fb",
			0444,
			d_per_vgt[vgt_id],
			p);
#endif

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFA_FB])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: fb of surface A\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: fb of surface A\n", vgt_id);

	d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFA_BASE] = debugfs_create_x32("surfA_base",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)(vgt_sreg(vgt, _REG_DSPASURF)));

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFA_BASE])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: surfA_base\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: surfA_base\n", vgt_id);

	d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFB_BASE] = debugfs_create_x32("surfB_base",
			0444,
			d_per_vgt[vgt_id],
			(u32 *)(vgt_sreg(vgt, _REG_DSPBSURF)));

	if (!d_debugfs_entry[vgt_id][VGT_DEBUGFS_SURFB_BASE])
		printk(KERN_ERR "vGT(%d): failed to create debugfs node: surfB_base\n", vgt_id);
	else
		printk("vGT(%d): create debugfs node: surfB_base\n", vgt_id);



	return 0;
}

/* debugfs_remove_recursive has no return value, this fuction
 * also return nothing */
void vgt_destroy_debugfs(struct vgt_device *vgt)
{
	int vgt_id = vgt->vgt_id;

	ASSERT(d_per_vgt[vgt_id]);

	debugfs_remove_recursive(d_per_vgt[vgt_id]);
	d_per_vgt[vgt_id] = NULL;
}

void vgt_release_debugfs(void)
{
	if (!d_vgt_debug)
		return;

	debugfs_remove_recursive(d_vgt_debug);
}
