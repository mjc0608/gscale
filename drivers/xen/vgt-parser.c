/*
 * vGT command parser
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
 */

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_drv.h"

/*
 * parse and handle the gfx command
 * which usually in ring buffer
 *
 * IN: instruction - mapped instruction address
 *
 * Return: the instruction bytes that have been handled
 *         -1 if error, e.g. command not recognized
 */

#define BATCH_BUFFER_ADDR_MASK ((1UL << 32) - (1U <<2))
#define BATCH_BUFFER_ADR_SPACE_BIT(x)	(((x)>>8) & 1U)

#ifdef VGT_ENABLE_ADDRESS_FIX_SAVE_RESTORE

static struct vgt_addr_fix_list addr_list;

void vgt_addr_fix_save(uint32_t* addr, uint32_t data)
{
	ASSERT(addr_list.pos < addr_list.len);

	spin_lock(&addr_list.lock);

	addr_list.entrys[addr_list.pos].addr = addr;
	addr_list.entrys[addr_list.pos].data = data;

	addr_list.pos++;

	spin_unlock(&addr_list.lock);
}

void vgt_addr_fix_restore(void)
{
	int i;

	spin_lock(&addr_list.lock);

	for (i=0; i<addr_list.pos; i++){
		*(addr_list.entrys[i].addr) = addr_list.entrys[i].data;
	}
	addr_list.pos = 0;

	spin_unlock(&addr_list.lock);
}

#define ADDR_FIX_MAX_ENTRY    102400

static int vgt_addr_fix_list_init(void)
{
	/*FIXME: implement more dynamic memory allocation */
	addr_list.len = ADDR_FIX_MAX_ENTRY;
	addr_list.pos = 0;
	addr_list.entrys = vmalloc( sizeof(struct vgt_addr_fix_entry) * addr_list.len );
	ASSERT(addr_list.entrys);
	spin_lock_init(&addr_list.lock);
	return 0;
}
#else

void vgt_addr_fix_save(uint32_t* addr, uint32_t data) { }

void vgt_addr_fix_restore(void) { }

static int vgt_addr_fix_list_init(void) {}

#endif /* VGT_ENABLE_ADDRESS_FIX_SAVE_RESTORE */

static inline int cmd_length(struct vgt_cmd_data *data, int nr_bits)
{
	/*  DWord Length is bits (nr_bits-1):0 */
	return (data->instruction[0] & ( (1U << nr_bits) - 1)) + 2;
}

static bool vgt_cmd_debug = false;
static int __init vgt_cmd_debug_setup(char *str)
{
	vgt_cmd_debug = true;
	return 1;
}
__setup("vgt_cmd_debug", vgt_cmd_debug_setup);

#define CMD_LOG_BUF_LEN 128
/* must be power of 2 */
#define CMD_LOG_BUFS	(1<<5)
static struct {
	spinlock_t lock;
	int head;
	int tail;
	int size;
	char bufs[CMD_LOG_BUFS][CMD_LOG_BUF_LEN];
} *logs = NULL;

#define INDEX_INC(index, size) (((index)+1) & ((size)-1))

static void vgt_cmd_logs_init(void)
{
	if (!vgt_cmd_debug)
		return;

	logs = kmalloc(sizeof(*logs), GFP_KERNEL);
	if (logs == NULL){
		printk(KERN_ERR "insufficient memory, command parser debug disabled\n");
		vgt_cmd_debug = false;
		return;
	}

	logs->head = logs->tail = 0;
	logs->size = CMD_LOG_BUFS;
	spin_lock_init(&logs->lock);
}

static void vgt_cmd_printk(const char *fmt, ...)
{
	va_list args;
	unsigned long flags;

	if (!vgt_cmd_debug)
		return;

	va_start(args, fmt);
	spin_lock_irqsave(&logs->lock, flags);

	if (CIRC_SPACE(logs->head, logs->tail, logs->size) == 1) {
		/* free old logs entry */
		logs->tail = INDEX_INC(logs->tail, logs->size);
	}
	vscnprintf(logs->bufs[logs->head], CMD_LOG_BUF_LEN, fmt, args);
	logs->head = INDEX_INC(logs->head, logs->size);

	spin_unlock_irqrestore(&logs->lock, flags);
	va_end(args);
}

static void vgt_cmd_show_logs(void)
{
	int head, tail;
	unsigned long flags;

	if (!vgt_cmd_debug)
		return;

	spin_lock_irqsave(&logs->lock, flags);
	head = logs->head;
	tail = logs->tail;
	while(head != tail){
		printk("CMD_LOG: %s", logs->bufs[tail]);
		tail = INDEX_INC(tail, logs->size);
	}
	spin_unlock_irqrestore(&logs->lock, flags);
}

static void* batch_buffer_va(struct vgt_cmd_data *d)
{
	unsigned long gpa;

	gpa = vgt_gma_2_gpa(d->vgt, d->instruction[1] & BATCH_BUFFER_ADDR_MASK,
			d->buf_addr_type == PPGTT_BUFFER);
	if (gpa == INVALID_ADDR){
		return NULL;
	}

	return vgt_vmem_gpa_2_va(d->vgt, gpa);
}

static void show_instruction_info(struct vgt_cmd_data *d);

static unsigned int constant_buffer_address_offset_disable(struct vgt_cmd_data *d)
{
	/* return the "CONSTANT_BUFFER Address Offset Disable" bit
	  in "INSTPM—Instruction Parser Mode Register"
	  0 - use as offset
	  1 - use as graphics address
	 */

	return VGT_MMIO_READ(d->vgt->pdev,_REG_RCS_INSTPM) & INSTPM_CONS_BUF_ADDR_OFFSET_DIS;
}

static void inline address_fixup(struct vgt_cmd_data *d, uint32_t *addr)
{
#if 0

	uint32_t val = *addr;

	if (h_gm_is_visible(d->vgt,val) || h_gm_is_hidden(d->vgt, val)){
		/* address already translated before, do nothing but return */
		VGT_CMD_PRINTK(KERN_WARNING "vgt: address 0x%x in %p already translated\n",
				val, addr);
		return 0;
	}

	if (g_gm_is_visible(d->vgt, val) || g_gm_is_hidden(d->vgt, val)){
		/* valid guest gm address */
		vgt_addr_fix_save(addr, val);
		*addr = g2h_gm(d->vgt, val);
		return 0;
	}

	/* invalid guest address */
	ASSERT(0);

	/* TODO: the address is out of range, raise fault? */
	printk(KERN_WARNING "vgt: address 0x%x in %p out of bound\n", val, addr);

	return -VGT_UNHANDLEABLE;
#endif
}
static int vgt_cmd_handler_mi_batch_buffer_end(struct vgt_cmd_data *data);


static inline bool batch_buffer_cross_page(struct vgt_cmd_data *d, int len)
{
	unsigned long start, end;

	if (d->buffer_type != BATCH_BUFFER_INSTRUCTION)
		return false;

	start = (unsigned long)d->instruction;
	end = (unsigned long)(d->instruction + len);

	return (start & PAGE_MASK) != (end & PAGE_MASK);
}

static inline void advance_ip(struct vgt_cmd_data *d, int len_in_qword)
{
	if (batch_buffer_cross_page(d, len_in_qword)){
		printk(KERN_WARNING "Batch buffer (0x%p:0x%x) cross page, so skip scaning it\n",
				d->instruction, len_in_qword*4);
		vgt_cmd_handler_mi_batch_buffer_end(d);
	}
	else
		d->instruction += len_in_qword;
	vgt_cmd_printk("cmd length are 0x%x bytes\n", len_in_qword*4);
}

static int vgt_cmd_handler_noop(struct vgt_cmd_data *data)
{
	advance_ip(data, 1);
	return 0;
}

static inline void length_fixup(struct vgt_cmd_data *data, int nr_bits)
{
	advance_ip(data, cmd_length(data, nr_bits));
}

static int vgt_cmd_handler_length_fixup_8(struct vgt_cmd_data *data)
{
	length_fixup(data, 8);
	return 0;
}

static int vgt_cmd_handler_length_fixup_16(struct vgt_cmd_data *data)
{
	length_fixup(data, 16);
	return 0;
}

static int vgt_cmd_handler_mi_batch_buffer_end(struct vgt_cmd_data *data)
{
	data->instruction = data->ret_instruction;
	data->buffer_type = RING_BUFFER_INSTRUCTION;
	data->buf_addr_type = GTT_BUFFER;
	return 0;
}

static int vgt_cmd_handler_mi_conditional_batch_buffer_end(struct vgt_cmd_data *data)
{
	/* FIXME: currently ignore the "Use Global GTT" bit, and assume always using GGTT.
	   need add PPGTT support later */

	/* TODO: handle the DWord Length */
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);

	return 0;
}

static int vgt_cmd_handler_mi_display_flip(struct vgt_cmd_data *data)
{
	/* TODO: handle the DWord Length */
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);

	return 0;
}
static int vgt_cmd_handler_mi_semaphore_mbox(struct vgt_cmd_data *data)
{
	/* TODO: handle the DWord Length */

	if (!(data->data & (1U<<18))){
		/* memory address, not MMIO register */
		address_fixup(data,data->instruction + 2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_set_context(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 1);
	length_fixup(data,8);

	return 0;
}

static int vgt_cmd_handler_mi_math(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_data_imm(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_data_index(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_load_register_imm(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

#define USE_GLOBAL_GTT_MASK (1U << 22)

static int vgt_cmd_handler_mi_update_gtt(struct vgt_cmd_data *data)
{
	uint32_t entry_num, *entry;
	int rc, i;
	gtt_pte_t pte, *p_pte;

	/*TODO: remove this assert when PPGTT support is added */
	ASSERT(data->instruction[0] & USE_GLOBAL_GTT_MASK);
	dprintk("mi_update_gtt\n");

	address_fixup(data,data->instruction + 1);

	entry_num = data->instruction[0] & ((1U<<8) - 1); /* bit 7:0 */
	entry = v_aperture(data->vgt->pdev, data->instruction[1]);
	p_pte = &pte;
	for (i=0; i<entry_num; i++){
		dprintk("vgt: update GTT entry %d\n", i);
		/*TODO: optimize by batch g2m translation*/
		rc = gtt_p2m(data->vgt, entry[i], &entry[i] );
		if (rc < 0){
			/* TODO: how to handle the invalide guest value */
		}
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_register_mem(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_flush_dw(struct vgt_cmd_data *data)
{
	int i, len;
	/* Check post-sync bit */
	if ( (data->data >> 14) & 0x3){
		address_fixup(data,data->instruction + 1);
	}

	len = cmd_length(data, 6);

	for (i=2; i<len; i++){
		address_fixup(data, data->instruction+i);
	}

	length_fixup(data,6);
	return 0;
}

static int vgt_cmd_handler_mi_clflush(struct vgt_cmd_data *data)
{
	/* TODO: may need to check if field "DW Representing 1/2 Cache Line" is out of bound */
	address_fixup(data,data->instruction + 1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_report_perf_count(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 1);
	length_fixup(data,6);
	return 0;
}

static void addr_type_update_snb(struct vgt_cmd_data *d)
{
	if ( (d->buffer_type == RING_BUFFER_INSTRUCTION) &&
			(d->vgt->rb[d->ring_id].has_ppgtt_mode_enabled) &&
			(BATCH_BUFFER_ADR_SPACE_BIT(d->instruction[0]) == 1)
	   )
	{
		d->buf_addr_type = PPGTT_BUFFER;
	}
}

static int vgt_cmd_handler_mi_batch_buffer_start(struct vgt_cmd_data *data)
{
	/* FIXME: add IVB/HSW code */
	addr_type_update_snb(data);

	if (data->buffer_type == RING_BUFFER_INSTRUCTION){
		data->ret_instruction = data->instruction + 2;
	}

	vgt_cmd_printk("MI_BATCH_BUFFER_START: buffer GraphicsAddress=%x Clear Command Buffer Enable=%d\n",
			data->instruction[1], (data->instruction[0]>>11) & 1);

	address_fixup(data, data->instruction + 1);

	data->instruction = (uint32_t*)batch_buffer_va(data);
	if (data->instruction == NULL){
		printk(KERN_WARNING"invalid batch buffer addr, so skip scanning it\n");
		vgt_cmd_handler_mi_batch_buffer_end(data);
		return 0;
	}

	data->buffer_type = BATCH_BUFFER_INSTRUCTION;
	return 0;
}

static int vgt_cmd_handler_xy_setup_blt(struct vgt_cmd_data *data)
{
	/* Destination Base Address */
	address_fixup(data,data->instruction + 4);
	/* Pattern Base Address for Color Pattern */
	address_fixup(data,data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_setup_clip_blt(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_setup_mono_pattern_sl_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pixel_blt(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_scanlines_blt(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_text_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 3);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_color_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_xy_color_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_blt(struct vgt_cmd_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_src_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	address_fixup(data,data->instruction + 8);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_mono_src_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_fixed_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_immediate_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt_immediate(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_chroma_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_immediate_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 7);
	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_xy_full_mono_src_immediate_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt_immediate(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_vertex_buffers(struct vgt_cmd_data *data)
{
	int length, i;

	length = cmd_length(data, 8);

	for (i=1; i < length; i = i+4){
		address_fixup(data,data->instruction + i + 1);
		address_fixup(data,data->instruction + i + 2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_index_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 1);

	if (*(data->instruction + 2) != 0)
		address_fixup(data,data->instruction + 2);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_gs(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,data->instruction + 1);
	}
	address_fixup(data,data->instruction + 2);
	address_fixup(data,data->instruction + 3);
	address_fixup(data,data->instruction + 4);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_ps(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,data->instruction + 1);
	}
	address_fixup(data,data->instruction + 2);
	address_fixup(data,data->instruction + 3);
	address_fixup(data,data->instruction + 4);

	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_3dstate_depth_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_stencil_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_hier_depth_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_so_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	address_fixup(data,data->instruction + 3);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_so_decl_list(struct vgt_cmd_data *data)
{
	length_fixup(data,9);
	return 0;
}

static int vgt_cmd_handler_pipe_control(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_prefetch(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_base_address(struct vgt_cmd_data *data)
{
	address_fixup(data,data->instruction + 1);
	address_fixup(data,data->instruction + 2);
	address_fixup(data,data->instruction + 3);
	address_fixup(data,data->instruction + 4);
	address_fixup(data,data->instruction + 5);
	/* Zero Bound is ignore */
	if (data->instruction[6] >> 12)
		address_fixup(data,data->instruction + 6);
	if (data->instruction[7] >> 12)
		address_fixup(data,data->instruction + 7);
	if (data->instruction[8] >> 12)
		address_fixup(data,data->instruction + 8);
	if (data->instruction[9] >> 12)
		address_fixup(data,data->instruction + 9);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_vs(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,data->instruction + 1);
	}
	address_fixup(data,data->instruction + 2);
	address_fixup(data,data->instruction + 3);
	address_fixup(data,data->instruction + 4);

	length_fixup(data,8);
	return 0;
}

/* FIXME: use hashtable to optimize the space */
static struct vgt_cmd_handlers cmd_handlers[GEN_GFX_CMD_TYPE_MAX + 1];

static int vgt_cmd_register_default(void)
{

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_NOOP,
			"MI_NOOP", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_USER_INTERRUPT,
			"MI_USER_INTERRUPT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_WAIT_FOR_EVENT,
			"MI_WAIT_FOR_EVENT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_FLUSH,
			"MI_FLUSH", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_ARB_CHECK,
			"MI_ARB_CHECK", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_REPORT_HEAD,
			"MI_REPORT_HEAD", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_ARB_ON_OFF,
			"MI_ARB_ON_OFF", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_BATCH_BUFFER_END,
			"MI_BATCH_BUFFER_END",	vgt_cmd_handler_mi_batch_buffer_end);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_SUSPEND_FLUSH,
			"MI_SUSPEND_FLUSH", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_DISPLAY_FLIP,
			"MI_DISPLAY_FLIP",	vgt_cmd_handler_mi_display_flip);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_SEMAPHORE_MBOX,
			"MI_SEMAPHORE_MBOX", vgt_cmd_handler_mi_semaphore_mbox);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_SET_CONTEXT,
			"MI_SET_CONTEXT", vgt_cmd_handler_mi_set_context);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_MATH,
			"MI_MATH", vgt_cmd_handler_mi_math);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_STORE_DATA_IMM,
			"MI_STORE_DATA_IMM", vgt_cmd_handler_mi_store_data_imm);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_STORE_DATA_INDEX,
	"MI_STORE_DATA_INDEX", vgt_cmd_handler_mi_store_data_index);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_LOAD_REGISTER_IMM,
	"MI_LOAD_REGISTER_IMM", vgt_cmd_handler_mi_load_register_imm);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_UPDATE_GTT,
	"MI_UPDATE_GTT", vgt_cmd_handler_mi_update_gtt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_STORE_REGISTER_MEM,
	"MI_STORE_REGISTER_MEM", vgt_cmd_handler_mi_store_register_mem);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_FLUSH_DW,
	"MI_FLUSH_DW", vgt_cmd_handler_mi_flush_dw);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_CLFLUSH,
	"MI_CLFLUSH", vgt_cmd_handler_mi_clflush);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_REPORT_PERF_COUNT,
	"MI_REPORT_PERF_COUNT", vgt_cmd_handler_mi_report_perf_count);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_BATCH_BUFFER_START,
	"MI_BATCH_BUFFER_START", vgt_cmd_handler_mi_batch_buffer_start);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, OP_MI_CONDITIONAL_BATCH_BUFFER_END,
			"MI_CONDITIONAL_BATCH_BUFFER_END", vgt_cmd_handler_mi_conditional_batch_buffer_end);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SETUP_BLT,
	"XY_SETUP_BLT", vgt_cmd_handler_xy_setup_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SETUP_CLIP_BLT,
	"XY_SETUP_CLIP_BLT", vgt_cmd_handler_xy_setup_clip_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SETUP_MONO_PATTERN_SL_BLT,
	"XY_SETUP_MONO_PATTERN_SL_BLT", vgt_cmd_handler_xy_setup_mono_pattern_sl_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_PIXEL_BLT,
	"XY_PIXEL_BLT", vgt_cmd_handler_xy_pixel_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SCANLINES_BLT,
	"XY_SCANLINES_BLT", vgt_cmd_handler_xy_scanlines_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_TEXT_BLT,
	"XY_TEXT_BLT", vgt_cmd_handler_xy_text_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_TEXT_IMMEDIATE_BLT,
	"XY_TEXT_IMMEDIATE_BLT", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_COLOR_BLT,
	"COLOR_BLT", vgt_cmd_handler_color_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_SRC_COPY_BLT,
	"SRC_COPY_BLT", vgt_cmd_handler_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_COLOR_BLT,
	"XY_COLOR_BLT", vgt_cmd_handler_xy_color_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_PAT_BLT,
	"XY_PAT_BLT", vgt_cmd_handler_xy_pat_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_MONO_PAT_BLT,
	"XY_MONO_PAT_BLT", vgt_cmd_handler_xy_mono_pat_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SRC_COPY_BLT,
	"XY_SRC_COPY_BLT", vgt_cmd_handler_xy_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_MONO_SRC_COPY_BLT,
	"XY_MONO_SRC_COPY_BLT", vgt_cmd_handler_xy_mono_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_BLT,
	"XY_FULL_BLT", vgt_cmd_handler_xy_full_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_MONO_SRC_BLT,
	"XY_FULL_MONO_SRC_BLT", vgt_cmd_handler_xy_full_mono_src_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_MONO_PATTERN_BLT,
	"XY_FULL_MONO_PATTERN_BLT", vgt_cmd_handler_xy_full_mono_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_MONO_PATTERN_MONO_SRC_BLT,
	"XY_FULL_MONO_PATTERN_MONO_SRC_BLT", vgt_cmd_handler_xy_full_mono_pattern_mono_src_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_MONO_PAT_FIXED_BLT,
	"XY_MONO_PAT_FIXED_BLT", vgt_cmd_handler_xy_mono_pat_fixed_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_MONO_SRC_COPY_IMMEDIATE_BLT,
	"XY_MONO_SRC_COPY_IMMEDIATE_BLT", vgt_cmd_handler_xy_mono_src_copy_immediate_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_PAT_BLT_IMMEDIATE,
	"XY_PAT_BLT_IMMEDIATE", vgt_cmd_handler_xy_pat_blt_immediate);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_SRC_COPY_CHROMA_BLT,
	"XY_SRC_COPY_CHROMA_BLT", vgt_cmd_handler_xy_src_copy_chroma_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_IMMEDIATE_PATTERN_BLT,
	"XY_FULL_IMMEDIATE_PATTERN_BLT", vgt_cmd_handler_xy_full_immediate_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT,
	"XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT", vgt_cmd_handler_xy_full_mono_src_immediate_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_PAT_CHROMA_BLT,
	"XY_PAT_CHROMA_BLT", vgt_cmd_handler_xy_pat_chroma_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, OP_XY_PAT_CHROMA_BLT_IMMEDIATE,
	"XY_PAT_CHROMA_BLT_IMMEDIATE", vgt_cmd_handler_xy_pat_chroma_blt_immediate);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_BINDING_TABLE_POINTERS,
	"3DSTATE_BINDING_TABLE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SAMPLER_STATE_POINTERS,
	"3DSTATE_SAMPLER_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_URB,
	"3DSTATE_URB", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VERTEX_BUFFERS,
	"3DSTATE_VERTEX_BUFFERS", vgt_cmd_handler_3dstate_vertex_buffers);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VERTEX_ELEMENTS,
	"3DSTATE_VERTEX_ELEMENTS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_INDEX_BUFFER,
	"3DSTATE_INDEX_BUFFER", vgt_cmd_handler_3dstate_index_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VF_STATISTICS,
	"3DSTATE_VF_STATISTICS", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VIEWPORT_STATE_POINTERS,
	"3DSTATE_VIEWPORT_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CC_STATE_POINTERS,
	"3DSTATE_CC_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SCISSOR_STATE_POINTERS,
	"3DSTATE_SCISSOR_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_GS,
	"3DSTATE_GS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CLIP,
	"3DSTATE_CLIP", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_WM,
	"3DSTATE_WM", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CONSTANT_GS,
	"3DSTATE_CONSTANT_GS", vgt_cmd_handler_3dstate_constant_gs);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CONSTANT_PS,
	"3DSTATE_CONSTANT_PS", vgt_cmd_handler_3dstate_constant_ps);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SAMPLE_MASK,
	"3DSTATE_SAMPLE_MASK", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_DRAWING_RECTANGLE,
	"3DSTATE_DRAWING_RECTANGLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SAMPLER_PALETTE_LOAD0,
	"3DSTATE_SAMPLER_PALETTE_LOAD0", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CHROMA_KEY,
	"3DSTATE_CHROMA_KEY", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_DEPTH_BUFFER,
	"3DSTATE_DEPTH_BUFFER", vgt_cmd_handler_3dstate_depth_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_POLY_STIPPLE_OFFSET,
	"3DSTATE_POLY_STIPPLE_OFFSET", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_POLY_STIPPLE_PATTERN,
	"3DSTATE_POLY_STIPPLE_PATTERN", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_LINE_STIPPLE,
	"3DSTATE_LINE_STIPPLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_AA_LINE_PARAMS,
	"3DSTATE_AA_LINE_PARAMS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_GS_SVB_INDEX,
	"3DSTATE_GS_SVB_INDEX", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SAMPLER_PALETTE_LOAD1,
	"3DSTATE_SAMPLER_PALETTE_LOAD1", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_MULTISAMPLE,
	"3DSTATE_MULTISAMPLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_STENCIL_BUFFER,
	"3DSTATE_STENCIL_BUFFER", vgt_cmd_handler_3dstate_stencil_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_HIER_DEPTH_BUFFER,
	"3DSTATE_HIER_DEPTH_BUFFER", vgt_cmd_handler_3dstate_hier_depth_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CLEAR_PARAMS,
	"3DSTATE_CLEAR_PARAMS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_MONOFILTER_SIZE,
	"3DSTATE_MONOFILTER_SIZE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SO_DECL_LIST,
	"3DSTATE_SO_DECL_LIST", vgt_cmd_handler_3dstate_so_decl_list);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SO_BUFFER,
	"3DSTATE_SO_BUFFER", vgt_cmd_handler_3dstate_so_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_PIPE_CONTROL,
	"PIPE_CONTROL", vgt_cmd_handler_pipe_control);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DPRIMITIVE,
	"3DPRIMITIVE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_PIPELINE_SELECT,
	"PIPELINE_SELECT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_STATE_PREFETCH,
	"STATE_PREFETCH", vgt_cmd_handler_state_prefetch);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_STATE_SIP,
	"STATE_SIP", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_STATE_BASE_ADDRESS,
	"STATE_BASE_ADDRESS", vgt_cmd_handler_state_base_address);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VS,
	"3DSTATE_VS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_SF,
	"3DSTATE_SF", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_CONSTANT_VS,
	"3DSTATE_CONSTANT_VS", vgt_cmd_handler_3dstate_constant_vs);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_INTERFACE_DESCRIPTOR_LOAD,
	"MEDIA_INTERFACE_DESCRIPTOR_LOAD", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_GATEWAY_STATE,
	"MEDIA_GATEWAY_STATE", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_STATE_FLUSH,
	"MEDIA_STATE_FLUSH", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_OBJECT,
	"MEDIA_OBJECT", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_CURBE_LOAD,
	"MEDIA_CURBE_LOAD", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_OBJECT_PRT,
	"MEDIA_OBJECT_PRT", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_OBJECT_WALKER,
	"MEDIA_OBJECT_WALKER", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_MEDIA_VFE_STATE,
	"MEDIA_VFE_STATE", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, OP_3DSTATE_VF_STATISTICS_GM45,
	"3DSTATE_VF_STATISTICS_GM45", vgt_cmd_handler_noop);

	return 0;
}

int vgt_cmd_parser_init(void)
{
	int type, size;

	cmd_handlers[GEN_GFX_CMD_TYPE_RENDER_MI].len = VGT_RENDER_MI_OPCODE_MAX + 1;
	cmd_handlers[GEN_GFX_CMD_TYPE_RENDER_2D].len = VGT_RENDER_2D_OPCODE_MAX + 1;
	cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].len = VGT_RENDER_3D_MEDIA_OPCODE_MAX + 1;
	cmd_handlers[GEN_GFX_CMD_TYPE_RENDER_MISC].len = 0;

	for (type=0; type<=GEN_GFX_CMD_TYPE_MAX; type++){
		size = sizeof(struct vgt_cmd_handler) * cmd_handlers[type].len;
		if (size == 0){
			cmd_handlers[type].handlers = NULL;
			continue;
		}
		cmd_handlers[type].handlers = vmalloc(size);
		if (cmd_handlers[type].handlers == NULL)
			return -ENOMEM;
		memset(cmd_handlers[type].handlers, 0, size);
	}

	vgt_cmd_register_default();

	vgt_addr_fix_list_init();

	vgt_cmd_logs_init();

	return 0;
}

void vgt_cmd_parser_exit(void)
{
	kfree(logs);
}

int vgt_cmd_handler_register(unsigned int type, unsigned int index,
		char* name, int (*handler)(struct vgt_cmd_data *data))
{
	if (type > GEN_GFX_CMD_TYPE_MAX || cmd_handlers[type].handlers == NULL)
	{
		printk(KERN_ERR "unsupported command handler type %d", type);
		return -ERANGE;
	}

	if (index >= cmd_handlers[type].len)
	{
		printk(KERN_ERR "unsupported command handler opcode 0x%x", index);
		return -ERANGE;
	}

	cmd_handlers[type].handlers[index].name = name;
	cmd_handlers[type].handlers[index].handler = handler;

	return 0;
}

static int vgt_cmd_handler_exec(struct vgt_cmd_data *decode_data)
{
	unsigned int type, opcode;

	type = decode_data->type;
	opcode = decode_data->opcode;

	if ( cmd_handlers[type].handlers == NULL ||
			opcode >= cmd_handlers[type].len ||
			cmd_handlers[type].handlers[opcode].handler == NULL){
		printk(KERN_ERR "vgt_cmd_handler_exec: unsupported command\n");
		show_instruction_info(decode_data);
		return  -VGT_UNHANDLEABLE;
	}

	decode_data->name = cmd_handlers[type].handlers[opcode].name;
	vgt_cmd_printk("cmd %s\n", decode_data->name);

	return cmd_handlers[type].handlers[opcode].handler(decode_data);
}

int vgt_cmd_parser_render(struct vgt_cmd_data* decode_data)
{
	union gen_gfx_render_cmd  command;
	unsigned int index;
	int ret = -VGT_UNHANDLEABLE;

	command.raw = *decode_data->instruction;
	decode_data->type = command.common.type;

	switch (decode_data->type){
		case GEN_GFX_CMD_TYPE_RENDER_MI:
			decode_data->opcode = command.mi.opcode;
			decode_data->data = command.mi.data;

			ret = vgt_cmd_handler_exec(decode_data);
			break;

		case GEN_GFX_CMD_TYPE_RENDER_MISC:
			printk(KERN_ERR "vgt: unsupported command GEN_GFX_CMD_TYPE_RENDER_MISC\n");
			show_instruction_info(decode_data);
			ret = -VGT_UNHANDLEABLE;
			break;

		case GEN_GFX_CMD_TYPE_RENDER_2D:
			decode_data->opcode = command.cmd_2d.opcode;
			decode_data->data = command.cmd_2d.data;

			ret = vgt_cmd_handler_exec(decode_data);
			break;

		case GEN_GFX_CMD_TYPE_GFXPIPE:

			decode_data->sub_type= command.cmd_3d_media.sub_type;
			decode_data->opcode = command.cmd_3d_media.opcode;
			decode_data->sub_opcode = command.cmd_3d_media.sub_opcode;
			decode_data->data = command.cmd_3d_media.data;
			decode_data->count = command.cmd_3d_media.count;
			index = INDEX_3D_MEDIA(decode_data->sub_type, \
					decode_data->opcode, decode_data->sub_opcode);

			if (index > VGT_RENDER_3D_MEDIA_OPCODE_MAX ||
				cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].handler == NULL){
				printk(KERN_ERR "vgt: unsupported 3D_MEDIA instruction "
						"sub_type=%xh opcode=%xh sub_opcode=%xh\n",
						decode_data->sub_type, decode_data->opcode,
						decode_data->sub_opcode);
				show_instruction_info(decode_data);
				ret = -VGT_UNHANDLEABLE;
				goto out;
			}

			decode_data->name = cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].name;
			vgt_cmd_printk("cmd %s\n", decode_data->name );

			ret = cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].handler(decode_data);

			break;

		default:
			printk(KERN_ERR "vgt: unsupported command type %xh\n",decode_data->type);
			show_instruction_info(decode_data);
			ret = -VGT_UNHANDLEABLE;
			break;
	}
out:
	return ret;
}

static void show_instruction_info(struct vgt_cmd_data *d)
{
	/* FIXME: for unknown command, the following info may be incorrect*/
	printk(KERN_ERR "buffer_type=%d instruction=%08x type=0x%x sub_type=0x%x opcode=0x%x sub_opcode=0x%x\n",
			d->buffer_type, d->instruction[0], d->type, d->sub_type, d->opcode, d->sub_opcode);
}

static inline void stat_nr_cmd_inc(struct vgt_cmd_data* d)
{
	vgt_state_ring_t* rs;

	rs = &d->vgt->rb[d->ring_id];

	if (d->buffer_type == RING_BUFFER_INSTRUCTION)
		rs->nr_cmd_ring++;
	else
		rs->nr_cmd_batch++;
	return;
}

static int __vgt_scan_vring(struct vgt_device *vgt, int ring_id, vgt_reg_t head, vgt_reg_t tail, vgt_reg_t base, vgt_reg_t size)
{
	static int error_count=0;
	char* instr, *instr_end, *va_bottom, *va_base;
	struct vgt_cmd_data decode_data;
	struct pgt_device  *pdev;
	int ret=0;

	if (error_count > 10)
		return 0;

	pdev = vgt->pdev;

	va_base = v_aperture(pdev, base);
	va_bottom = va_base + size;

	instr = va_base + head;
	instr_end = va_base + tail;

	decode_data.buffer_type = RING_BUFFER_INSTRUCTION;
	decode_data.buf_addr_type = GTT_BUFFER;
	decode_data.vgt = vgt;
	decode_data.ring_id = ring_id;
	vgt_cmd_printk("ring buffer scan start\n");
	while(instr != instr_end){
		decode_data.instruction = (uint32_t*)instr;
		vgt_cmd_printk("scan %s ip(%p) cmd %08x %08x %08x %08x\n",
				decode_data.buffer_type == RING_BUFFER_INSTRUCTION ? "RING_BUFFER": "BATCH_BUFFER",
				instr, decode_data.instruction[0], decode_data.instruction[1],
				decode_data.instruction[2],	decode_data.instruction[3]);

		stat_nr_cmd_inc(&decode_data);

		ret = vgt_cmd_parser_render(&decode_data);
		if (ret < 0){
			error_count++;
			printk("error_count=%d\n", error_count);
			vgt_cmd_show_logs();
			break;
		}

		/* next instruction*/
		instr = (char*)decode_data.instruction;

		if (decode_data.buffer_type == RING_BUFFER_INSTRUCTION){
			/* handle the ring buffer wrap case */
			ASSERT(instr <= va_bottom);

			if (instr == va_bottom){
				instr = va_base;
			}
		}
	}
	return ret;
}

/*
 * Scan the guest ring.
 *   Return 0: success
 *         <0: Address violation.
 */
int vgt_scan_vring(struct vgt_device *vgt, int ring_id)
{
	vgt_ringbuffer_t *vring;
	int ret;

	vring = &vgt->rb[ring_id].vring;

	if ( !(vring->ctl & _RING_CTL_ENABLE) ) {
		/* Ring is enabled */
		printk("VGT-Parser.c vring head %x tail %x ctl %x\n",
			vring->head, vring->tail, vring->ctl);
		return 0;
	}

	ret = __vgt_scan_vring (vgt, ring_id, vgt->last_scan_head[ring_id],
		vring->tail & RB_TAIL_OFF_MASK,
		vring->start, _RING_CTL_BUF_SIZE(vring->ctl));

	vgt->last_scan_head[ring_id] = vring->tail;
	return ret;
}

