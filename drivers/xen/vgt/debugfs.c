/*
 * Debugfs interfaces
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

/* TODO: this file's code copied from arch/x86/xen/debugfs.c and
 * fs/debugfs/file.c. Can we clean up and/or minimize this file???
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/debugfs.h>

#include <xen/vgt.h>
#include "vgt.h"

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
	{ "context_switch_cycles", &context_switch_cost },
	{ "context_switch_num", &context_switch_num },
	{ "ring_0_busy", &ring_0_busy },
	{ "ring_0_idle", &ring_0_idle },
	{ "", NULL}
};

#define debugfs_create_u64_node(name, perm, parent, u64_ptr) \
	do { \
		struct dentry *__dentry = debugfs_create_u64( \
		(name),\
		(perm), \
		(parent), \
		(u64_ptr) \
		); \
		if (!__dentry) \
			printk(KERN_ERR "Failed to create debugfs node: %s\n", (name)); \
	} while (0)

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

/* This is generic function, used to format ring_buffer and etc. */
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

static char *format_array_alloc(const char *fmt, u32 *array, unsigned array_size)
{
	/* very tricky way */
	size_t len = format_array(NULL, 0, fmt, array, array_size);
	char *ret;

	ret = vmalloc(len);
	if (ret == NULL) {
		vgt_err("failed to alloc memory!");
		return NULL;
	}

	format_array(ret, len, fmt, array, array_size);
	return ret;
}

/* data copied from kernel space to user space */
static ssize_t u32_array_read(struct file *file, char __user *buf, size_t len,
				loff_t *ppos)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct array_data *data = inode->i_private;
	size_t size;

	if (*ppos == 0) {
		if (file->private_data) {
			vfree(file->private_data);
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
	vfree(file->private_data);
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

static inline char *reg_show_reg_owner(struct pgt_device *pdev, int i)
{
	char *str;
	switch (reg_get_owner(pdev, i)) {
		case VGT_OT_NONE:
			str = "NONE";
			break;
		case VGT_OT_RENDER:
			str = "Render";
			break;
		case VGT_OT_DISPLAY:
			str = "Display";
			break;
		case VGT_OT_CONFIG:
			str = "Config";
			break;
		default:
			str = "";
			break;
	}
	return str;
}

static inline char *reg_show_reg_type(struct pgt_device *pdev, int i)
{
	if (reg_get_owner(pdev, i) != VGT_OT_NONE)
		return "MPT";
	else if (reg_workaround(pdev, i))
		return "WA";
	else if (reg_virt(pdev, i))
		return "Virt";
	else
		return "";
}

static int vgt_show_regs(struct seq_file *m, void *data)
{
	int i, tot;
	struct pgt_device *pdev = (struct pgt_device *)m->private;

	tot = 0;
	seq_printf(m, "------------------------------------------\n");
	seq_printf(m, "MGMT - Management context\n");
	seq_printf(m, "MPT - Mediated Pass-Through based on owner type\n");
	seq_printf(m, "WA - workaround regs with special risk\n");
	seq_printf(m, "%8s: %8s (%-8s %-4s)\n",
			"Reg", "Flags", "Owner", "Type");
	for (i = 0; i < pdev->mmio_size; i +=  REG_SIZE) {
		if (!reg_is_accessed(pdev, i))
			continue;

		tot++;
		seq_printf(m, "%8x: %8x (%-8s %-4s)\n",
			i, pdev->reg_info[REG_INDEX(i)],
			reg_show_reg_owner(pdev, i),
			reg_show_reg_type(pdev, i));
	}
	seq_printf(m, "------------------------------------------\n");
	seq_printf(m, "Total %d accessed registers are shown\n", tot);
	return 0;
}

static int vgt_reginfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, vgt_show_regs, inode->i_private);
}

static const struct file_operations reginfo_fops = {
	.open = vgt_reginfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * It's always dangerious to read from pReg directly, since some
 * read has side effect e.g. read-to-clear bit.
 *
 * So use it with caution only when debugging hard GPU hang problem
 */
static int vgt_show_pregs(struct seq_file *m, void *data)
{
	u64 i;
	struct pgt_device *pdev = (struct pgt_device *)m->private;

	seq_printf(m, "Use this interface with caution b/c side effect may be caused by reading hw status\n");
	for(i = 0; i < pdev->reg_num; i++) {
		if (!(i % 16))
			seq_printf(m, "\n%8llx:", i * REG_SIZE);
		seq_printf(m, " %x", VGT_MMIO_READ(pdev, i * REG_SIZE));
	}

	seq_printf(m, "\n");
	return 0;
}

static int vgt_preg_open(struct inode *inode, struct file *file)
{
	return single_open(file, vgt_show_pregs, inode->i_private);
}

static const struct file_operations preg_fops = {
	.open = vgt_preg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int vgt_show_irqinfo(struct seq_file *m, void *data)
{
	struct pgt_device *pdev = (struct pgt_device *)m->private;
	struct vgt_device *vgt;
	struct pgt_statistics *pstat = &pdev->stat;
	struct vgt_statistics *vstat;
	int i, j;

	if (!pstat->irq_num) {
		seq_printf(m, "No irq logged\n");
		return 0;
	}
	seq_printf(m, "--------------------------\n");
	seq_printf(m, "Interrupt control status:\n");
	seq_printf(m, "vGT: DEISR is %x, DEIIR is %x, DEIMR is %x, DEIER is %x\n",
		VGT_MMIO_READ(pdev, _REG_DEISR),
		VGT_MMIO_READ(pdev, _REG_DEIIR),
		VGT_MMIO_READ(pdev, _REG_DEIMR),
		VGT_MMIO_READ(pdev, _REG_DEIER));
	seq_printf(m, "vGT: SDEISR is %x, SDEIIR is %x, SDEIMR is %x, SDEIER is %x\n",
		VGT_MMIO_READ(pdev, _REG_SDEISR),
		VGT_MMIO_READ(pdev, _REG_SDEIIR),
		VGT_MMIO_READ(pdev, _REG_SDEIMR),
		VGT_MMIO_READ(pdev, _REG_SDEIER));
	seq_printf(m, "vGT: GTISR is %x, GTIIR is %x, GTIMR is %x, GTIER is %x\n",
		VGT_MMIO_READ(pdev, _REG_GTISR),
		VGT_MMIO_READ(pdev, _REG_GTIIR),
		VGT_MMIO_READ(pdev, _REG_GTIMR),
		VGT_MMIO_READ(pdev, _REG_GTIER));
	seq_printf(m, "vGT: PMISR is %x, PMIIR is %x, PMIMR is %x, PMIER is %x\n",
		VGT_MMIO_READ(pdev, _REG_PMISR),
		VGT_MMIO_READ(pdev, _REG_PMIIR),
		VGT_MMIO_READ(pdev, _REG_PMIMR),
		VGT_MMIO_READ(pdev, _REG_PMIER));
	seq_printf(m, "vGT: RCS_IMR is %x, VCS_IMR is %x, BCS_IMR is %x\n",
		VGT_MMIO_READ(pdev, _REG_RCS_IMR),
		VGT_MMIO_READ(pdev, _REG_VCS_IMR),
		VGT_MMIO_READ(pdev, _REG_BCS_IMR));
	seq_printf(m, "Total %lld interrupts logged:\n", pstat->irq_num);
	seq_printf(m, "#	WARNING: precisely this is the number of vGT \n"
			"#	physical interrupt handler be called,\n"
			"#	each calling several events can be\n"
			"#	been handled, so usually this number\n"
			"#	is less than the total events number.\n");
	for (i = 0; i < EVENT_MAX; i++) {
		if (!pstat->events[i])
			continue;
		seq_printf(m, "\t%16lld: %s\n", pstat->events[i],
				vgt_irq_name[i]);
	}

	seq_printf(m, "%16lld: Last pirq\n", pstat->last_pirq);
	seq_printf(m, "%16lld: Last virq\n", pstat->last_virq);
	seq_printf(m, "%16lld: Average pirq cycles\n",
		pstat->pirq_cycles / pstat->irq_num);
	seq_printf(m, "%16lld: Average virq cycles\n",
		pstat->virq_cycles / pstat->irq_num);
	seq_printf(m, "%16lld: Average delay between pirq/virq handling\n",
		pstat->irq_delay_cycles / pstat->irq_num);
	/* TODO: hold lock */
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (!pdev->device[i])
			continue;

		seq_printf(m, "\n-->vgt-%d:\n", pdev->device[i]->vgt_id);
		vgt = pdev->device[i];
		vstat = &vgt->stat;

		seq_printf(m, "....vreg (deier: %x, deiir: %x, deimr: %x, deisr: %x)\n",
				__vreg(vgt, _REG_DEIER),
				__vreg(vgt, _REG_DEIIR),
				__vreg(vgt, _REG_DEIMR),
				__vreg(vgt, _REG_DEISR));
		seq_printf(m, "....vreg (gtier: %x, gtiir: %x, gtimr: %x, gtisr: %x)\n",
				__vreg(vgt, _REG_GTIER),
				__vreg(vgt, _REG_GTIIR),
				__vreg(vgt, _REG_GTIMR),
				__vreg(vgt, _REG_GTISR));
		seq_printf(m, "....vreg (sdeier: %x, sdeiir: %x, sdeimr: %x, sdeisr: %x)\n",
				__vreg(vgt, _REG_SDEIER),
				__vreg(vgt, _REG_SDEIIR),
				__vreg(vgt, _REG_SDEIMR),
				__vreg(vgt, _REG_SDEISR));
		seq_printf(m, "....vreg (pmier: %x, pmiir: %x, pmimr: %x, pmisr: %x)\n",
				__vreg(vgt, _REG_PMIER),
				__vreg(vgt, _REG_PMIIR),
				__vreg(vgt, _REG_PMIMR),
				__vreg(vgt, _REG_PMISR));
		seq_printf(m, "....vreg (rcs_imr: %x, vcs_imr: %x, bcs_imr: %x\n",
				__vreg(vgt, _REG_RCS_IMR),
				__vreg(vgt, _REG_VCS_IMR),
				__vreg(vgt, _REG_BCS_IMR));
		seq_printf(m, "%16lld: Last injection\n",
			vstat->last_injection);

		if (!vstat->irq_num)
			continue;

		seq_printf(m, "Total %lld virtual irq injection:\n",
			vstat->irq_num);
		for (j = 0; j < EVENT_MAX; j++) {
			if (!vstat->events[j])
				continue;
			seq_printf(m, "\t%16lld: %s\n", vstat->events[j],
					vgt_irq_name[j]);
		}

		if (vstat->pending_events)
			seq_printf(m, "\t%16lld: %s\n", vstat->pending_events,
					"pending virt events");
	}
	return 0;
}

static int vgt_irqinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, vgt_show_irqinfo, inode->i_private);
}

static const struct file_operations irqinfo_fops = {
	.open = vgt_irqinfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* initialize vGT debufs top directory */
struct dentry *vgt_init_debugfs(struct pgt_device *pdev)
{
	struct dentry *temp_d;
	int   i;

	if (!d_vgt_debug) {
		d_vgt_debug = debugfs_create_dir("vgt", NULL);

		if (!d_vgt_debug) {
			pr_warning("Could not create 'vgt' debugfs directory\n");
			return NULL;
		}
	}

	for ( i = 0; stat_info[i].stat != NULL; i++ ) {
		temp_d = debugfs_create_u64(stat_info[i].node_name,
			0444,
			d_vgt_debug,
			stat_info[i].stat);
		if (!temp_d)
			printk(KERN_ERR "Failed to create debugfs node %s\n",
				stat_info[i].node_name);
	}

	temp_d = debugfs_create_file("reginfo", 0444, d_vgt_debug,
		pdev, &reginfo_fops);
	if (!temp_d)
		return NULL;

	temp_d = debugfs_create_file("preg", 0444, d_vgt_debug,
		pdev, &preg_fops);
	if (!temp_d)
		return NULL;

	temp_d = debugfs_create_file("irqinfo", 0444, d_vgt_debug,
		pdev, &irqinfo_fops);
	if (!temp_d)
		return NULL;

	return d_vgt_debug;
}

static void vgt_create_cmdstat_per_ring(struct vgt_device *vgt, int ring_id, struct dentry *parent)
{
	char *ring_name;
	struct dentry *ring_dir_entry;
	switch (ring_id) {
		case RING_BUFFER_RCS:
			ring_name = "render";
			break;
		case RING_BUFFER_VCS:
			ring_name = "video";
			break;
		case RING_BUFFER_BCS:
			ring_name = "blitter";
			break;
		case RING_BUFFER_VECS:
			ring_name = "ve";
			break;
		default:
			return;
	}
	ring_dir_entry = debugfs_create_dir(ring_name, parent);
	if (!ring_dir_entry)
		printk(KERN_ERR "vGT(%d): failed to create debugfs directory: %s\n", vgt->vgt_id, ring_name);
	else {
		debugfs_create_u64_node("ring_cmd_nr", 0444, ring_dir_entry, &(vgt->rb[ring_id].nr_cmd_ring));
		debugfs_create_u64_node("batch_cmd_nr", 0444, ring_dir_entry, &(vgt->rb[ring_id].nr_cmd_batch));
	}
}

int vgt_create_debugfs(struct vgt_device *vgt)
{
	int retval,i;
	struct array_data *p;
	int vgt_id = vgt->vgt_id;
	struct pgt_device *pdev = vgt->pdev;
	struct dentry *perf_dir_entry, *cmdstat_dir_entry;

	dsp_surf_size[vgt_id][PIPE_A] = __sreg(vgt, _REG_DSPASIZE);
	dsp_surf_size[vgt_id][PIPE_B] = __sreg(vgt, _REG_DSPBSIZE);

	dsp_surf_base[vgt_id][PIPE_A] = phys_aperture_vbase(pdev) + ((__sreg(vgt, _REG_DSPASURF)) & PAGE_MASK);
	dsp_surf_base[vgt_id][PIPE_B] = phys_aperture_vbase(pdev) + ((__sreg(vgt, _REG_DSPBSURF)) & PAGE_MASK);

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

	/* virtual mmio space dump */
	p = &vgt_debugfs_data[vgt_id][VGT_DEBUGFS_VIRTUAL_MMIO];
	p->array = (u32 *)(vgt->state.vReg);
	p->elements = pdev->reg_num;
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
	p->elements = pdev->reg_num;
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

	/* perf vm perfermance statistics */
	perf_dir_entry = debugfs_create_dir("perf", d_per_vgt[vgt_id]);
	if (!perf_dir_entry)
		printk(KERN_ERR "vGT(%d): failed to create debugfs directory: perf\n", vgt_id);
	else {
		debugfs_create_u64_node ("schedule_in_time", 0444, perf_dir_entry, &(vgt->stat.schedule_in_time));
		debugfs_create_u64_node ("allocated_cycles", 0444, perf_dir_entry, &(vgt->stat.allocated_cycles));
		//debugfs_create_u64_node ("used_cycles", 0444, perf_dir_entry, &(vgt->stat.used_cycles));

		/* cmd statistics for ring/batch buffers */
		cmdstat_dir_entry = debugfs_create_dir("ring", perf_dir_entry);
		if (!cmdstat_dir_entry)
			printk(KERN_ERR "vGT(%d): failed to create debugfs directory: ringbuffer\n", vgt_id);
		else
			/* for each ring */
			for (i = 0; i < pdev->max_engines; i++)
				vgt_create_cmdstat_per_ring(vgt, i, cmdstat_dir_entry);
	}

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
