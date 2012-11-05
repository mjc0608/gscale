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

#define BATCH_BUFFER_ADDR_MASK ((1UL << 32) - (1U <<2))
#define BATCH_BUFFER_ADR_SPACE_BIT(x)	(((x)>>8) & 1U)
#define BATCH_BUFFER_2ND_LEVEL_BIT(x)   ((x)>>22 & 1U)

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

static inline uint32_t* instr_ptr(struct vgt_cmd_data *d, int instr_index)
{
	if (instr_index < d->instr_buf_len)
		return d->instr_va + instr_index;
	else
		return d->instr_va_next_page + (instr_index - d->instr_buf_len);
}

static inline uint32_t instr_val(struct vgt_cmd_data *d, int instr_index)
{
	return *instr_ptr(d, instr_index);
}

static int instr_gma_set(struct vgt_cmd_data *d, unsigned long instr_gma)
{
	unsigned long gma_next_page;
	ASSERT(VGT_REG_IS_ALIGNED(instr_gma, 4));

	d->instr_gma = instr_gma;
	d->instr_va = vgt_gma_to_va(d->vgt, instr_gma,
			d->buf_addr_type == PPGTT_BUFFER);
	ASSERT(d->instr_va);

	gma_next_page = ((instr_gma >> PAGE_SHIFT) + 1) << PAGE_SHIFT;
	d->instr_va_next_page = vgt_gma_to_va(d->vgt, gma_next_page,
			d->buf_addr_type == PPGTT_BUFFER);
	ASSERT(d->instr_va_next_page);

	d->instr_buf_len = (gma_next_page - instr_gma) / sizeof(uint32_t);

	return 0;
}

static int instr_gma_advance(struct vgt_cmd_data *d, unsigned long len)
{
	if (d->instr_buf_len > len){
		/* not cross page, advance ip inside page */
		d->instr_gma += len*sizeof(uint32_t);
		d->instr_va += len;
		d->instr_buf_len -= len;
	} else{
		/* cross page, reset instr_gma */
		instr_gma_set(d, d->instr_gma + len*sizeof(uint32_t));
	}
	return 0;
}

static inline int cmd_length(struct vgt_cmd_data *data, int nr_bits)
{
	/*  DWord Length is bits (nr_bits-1):0 */
	return (instr_val(data,0) & ( (1U << nr_bits) - 1)) + 2;
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

/* index: the index to the instruction  i.e. the address to be fixed is d->instruction[index] */
static void inline address_fixup(struct vgt_cmd_data *d, int index)
{
#if 0

	uint32_t val = *addr;

	if (h_gm_is_valid(d->vgt,val)) {
		/* address already translated before, do nothing but return */
		klog_printk("vgt: address 0x%x in %p already translated\n",
				val, addr);
		return 0;
	}

	if (g_gm_is_valid(d->vgt, val)) {
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

static inline void advance_ip(struct vgt_cmd_data *d, int len_in_qword)
{
	instr_gma_advance(d, len_in_qword);
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
	instr_gma_set(data, data->ret_instr_gma);
	data->buffer_type = RING_BUFFER_INSTRUCTION;
	data->buf_addr_type = GTT_BUFFER;
	return 0;
}

static int vgt_cmd_handler_mi_conditional_batch_buffer_end(struct vgt_cmd_data *data)
{
	/* FIXME: currently ignore the "Use Global GTT" bit, and assume always using GGTT.
	   need add PPGTT support later */

	/* TODO: handle the DWord Length */
	address_fixup(data,2);
	length_fixup(data,8);

	return 0;
}

static int vgt_cmd_handler_mi_display_flip(struct vgt_cmd_data *data)
{
	address_fixup(data,2);
	length_fixup(data,8);

	return 0;
}
static int vgt_cmd_handler_mi_semaphore_mbox(struct vgt_cmd_data *data)
{
	/* TODO: handle the DWord Length */

	if (!(data->data & (1U<<18))){
		/* memory address, not MMIO register */
		address_fixup(data,2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_set_context(struct vgt_cmd_data *data)
{
	address_fixup(data,1);
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
	address_fixup(data,2);
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
	int i;

	klog_printk("mi_load_register_imm:");
	for (i=1; i < cmd_length(data, 8); i=i+2){
		klog_printk(" mmio(0x%lx) data(0x%lx)", instr_val(data, i), instr_val(data, i+1));
	}
	klog_printk("\n");

	length_fixup(data,8);
	return 0;
}

#define USE_GLOBAL_GTT_MASK (1U << 22)

static int vgt_cmd_handler_mi_update_gtt(struct vgt_cmd_data *data)
{
	uint32_t entry_num, *entry;
	int rc, i;

	/*TODO: remove this assert when PPGTT support is added */
	ASSERT(instr_val(data,0) & USE_GLOBAL_GTT_MASK);
	dprintk("mi_update_gtt\n");

	address_fixup(data, 1);

	entry_num = instr_val(data,0) & ((1U<<8) - 1); /* bit 7:0 */
	entry = v_aperture(data->vgt->pdev, instr_val(data,1));
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
	int i;

	klog_printk("mi_store_register_mem: ");
	for (i=1; i < cmd_length(data, 8); i=i+2){
		klog_printk("mmio(%lx) mem(%lx) ", instr_val(data, i), instr_val(data, i+1));
	}
	klog_printk("\n");

	address_fixup(data,2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_flush_dw(struct vgt_cmd_data *data)
{
	int i, len;
	/* Check post-sync bit */
	if ( (data->data >> 14) & 0x3){
		address_fixup(data, 1);
	}

	len = cmd_length(data, 6);

	for (i=2; i<len; i++){
		address_fixup(data, i);
	}

	length_fixup(data,6);
	return 0;
}

static int vgt_cmd_handler_mi_clflush(struct vgt_cmd_data *data)
{
	/* TODO: may need to check if field "DW Representing 1/2 Cache Line" is out of bound */
	address_fixup(data,1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_report_perf_count(struct vgt_cmd_data *data)
{
	address_fixup(data, 1);
	length_fixup(data,6);
	return 0;
}

static void addr_type_update_snb(struct vgt_cmd_data *d)
{
	if ( (d->buffer_type == RING_BUFFER_INSTRUCTION) &&
			(d->vgt->rb[d->ring_id].has_ppgtt_mode_enabled) &&
			(BATCH_BUFFER_ADR_SPACE_BIT(instr_val(d,0)) == 1)
	   )
	{
		d->buf_addr_type = PPGTT_BUFFER;
	}
}

static int vgt_cmd_handler_mi_batch_buffer_start(struct vgt_cmd_data *data)
{
	int rc;

	/* FIXME: add 2nd level batch buffer support */
	ASSERT(BATCH_BUFFER_2ND_LEVEL_BIT(instr_val(data,0)) == 0);

	/* FIXME: add IVB/HSW code */
	addr_type_update_snb(data);

	if (data->buffer_type == RING_BUFFER_INSTRUCTION){
		data->ret_instr_gma = data->instr_gma + 2*sizeof(uint32_t);
	}

	klog_printk("MI_BATCH_BUFFER_START: Addr=%x ClearCommandBufferEnable=%d\n",
			instr_val(data,1),  (instr_val(data,0)>>11) & 1);

	address_fixup(data, 1);

	rc = instr_gma_set(data, instr_val(data,1) & BATCH_BUFFER_ADDR_MASK);

	if (rc < 0){
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
	address_fixup(data, 4);
	/* Pattern Base Address for Color Pattern */
	address_fixup(data, 7);
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
	address_fixup(data, 4);
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
	address_fixup(data, 3);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_color_blt(struct vgt_cmd_data *data)
{
	address_fixup(data, 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data, 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_xy_color_blt(struct vgt_cmd_data *data)
{
	address_fixup(data, 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt(struct vgt_cmd_data *data)
{
	address_fixup(data, 4);
	address_fixup(data, 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_blt(struct vgt_cmd_data *data)
{
	address_fixup(data, 4);
	address_fixup(data, 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,5);
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
	address_fixup(data,4);
	address_fixup(data,5);
	address_fixup(data,8);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_mono_src_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_fixed_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_immediate_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt_immediate(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_chroma_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_immediate_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,7);
	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_xy_full_mono_src_immediate_pattern_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	address_fixup(data,5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt_immediate(struct vgt_cmd_data *data)
{
	address_fixup(data,4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_vertex_buffers(struct vgt_cmd_data *data)
{
	int length, i;

	length = cmd_length(data, 8);

	for (i=1; i < length; i = i+4){
		address_fixup(data,i + 1);
		address_fixup(data,i + 2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_index_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,1);

	if (instr_val(data,2) != 0)
		address_fixup(data,2);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_gs(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,1);
	}
	address_fixup(data,2);
	address_fixup(data,3);
	address_fixup(data,4);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_ps(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,1);
	}
	address_fixup(data,2);
	address_fixup(data,3);
	address_fixup(data,4);

	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_3dstate_depth_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_stencil_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_hier_depth_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_so_buffer(struct vgt_cmd_data *data)
{
	address_fixup(data,2);
	address_fixup(data,3);
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
	address_fixup(data,2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_prefetch(struct vgt_cmd_data *data)
{
	address_fixup(data,1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_base_address(struct vgt_cmd_data *data)
{
	address_fixup(data,1);
	address_fixup(data,2);
	address_fixup(data,3);
	address_fixup(data,4);
	address_fixup(data,5);
	/* Zero Bound is ignore */
	if (instr_val(data,6) >> 12)
		address_fixup(data,6);
	if (instr_val(data,7) >> 12)
		address_fixup(data,7);
	if (instr_val(data,8) >> 12)
		address_fixup(data,8);
	if (instr_val(data,9) >> 12)
		address_fixup(data,9);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_vs(struct vgt_cmd_data *data)
{
	if (constant_buffer_address_offset_disable(data) == 1){
		address_fixup(data,1);
	}
	address_fixup(data,2);
	address_fixup(data,3);
	address_fixup(data,4);

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

	return 0;
}

void vgt_cmd_parser_exit(void)
{
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
	klog_printk("%s\n", decode_data->name);

	return cmd_handlers[type].handlers[opcode].handler(decode_data);
}

int vgt_cmd_parser_render(struct vgt_cmd_data* decode_data)
{
	union gen_gfx_render_cmd  command;
	unsigned int index;
	int ret = -VGT_UNHANDLEABLE;

	command.raw = instr_val(decode_data,0);
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
			klog_printk("%s\n", decode_data->name );

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
			d->buffer_type, instr_val(d,0), d->type, d->sub_type, d->opcode, d->sub_opcode);
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
	unsigned long instr_gma, instr_gma_end, instr_gma_bottom;
	struct vgt_cmd_data decode_data;
	int ret=0;

	if (error_count > 10)
		return 0;

	instr_gma = base + head;
	instr_gma_end = base + tail;
	instr_gma_bottom = base + size;

	decode_data.buffer_type = RING_BUFFER_INSTRUCTION;
	decode_data.buf_addr_type = GTT_BUFFER;
	decode_data.vgt = vgt;
	decode_data.ring_id = ring_id;
	instr_gma_set(&decode_data, instr_gma);

	klog_printk("ring buffer scan start\n");
	while(instr_gma != instr_gma_end){
		klog_printk("%s ip(%08lx): %08x %08x %08x %08x ",
				decode_data.buffer_type == RING_BUFFER_INSTRUCTION ? "RB": "BB",
				instr_gma, instr_val(&decode_data,0), instr_val(&decode_data,1),
				instr_val(&decode_data,2), instr_val(&decode_data,3));

		stat_nr_cmd_inc(&decode_data);

		ret = vgt_cmd_parser_render(&decode_data);
		if (ret < 0){
			error_count++;
			printk("error_count=%d\n", error_count);
			break;
		}

		/* next instruction*/
		instr_gma = decode_data.instr_gma;

		if (decode_data.buffer_type == RING_BUFFER_INSTRUCTION){
			/* handle the ring buffer wrap case */
			ASSERT(instr_gma <= instr_gma_bottom);

			if (instr_gma == instr_gma_bottom){
				instr_gma_set(&decode_data, base);
			}
		}
	}
	klog_printk("ring buffer scan end\n");
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

