/*
 * vGT command parser
 *
 * This file is provided under a GPLv2 license.
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
#include <xen/vgt_cmd_parser.h>
#include "vgt_drv.h"

#ifndef VGT_PARSER_OLD
/*
 * new cmd parser
 */

DEFINE_HASHTABLE(vgt_cmd_table, VGT_CMD_HASH_BITS);

static void vgt_add_cmd_entry(struct vgt_cmd_entry *e)
{
	hash_add(vgt_cmd_table, &e->hlist, e->info->opcode);
}

static struct cmd_info* vgt_find_cmd_entry(unsigned int opcode, int ring_id)
{
	struct vgt_cmd_entry *e;
	struct hlist_node *node;

	hash_for_each_possible(vgt_cmd_table, e, node, hlist, opcode) {
		if ( (opcode == e->info->opcode) && (e->info->rings & (1<<ring_id)) )
			return e->info;
	}
	return NULL;
}

static struct cmd_info* vgt_find_cmd_entry_any_ring(unsigned int opcode, int rings)
{
	struct cmd_info* info = NULL;
	unsigned int ring;
	for_each_set_bit(ring, (unsigned long*)&rings, MAX_ENGINES){
		info = vgt_find_cmd_entry(opcode, ring);
		if(info)
			break;
	}
	return info;
}

void vgt_clear_cmd_table(void)
{
	int i;
	struct hlist_node *node, *tmp;
	struct vgt_cmd_entry *e;

	hash_for_each_safe(vgt_cmd_table, i, node, tmp, e, hlist)
		kfree(e);

	hash_init(vgt_cmd_table);
}
#ifdef VGT_ENABLE_ADDRESS_FIX
static int address_fixup(struct parser_exec_state *s, int index){
	/*TODO: add address fix up implementation */
	return 0;
}
#else

#define address_fixup(s,index)	do{}while(0)

#endif

/* Render Command Parser */
static unsigned int rcp_op_len[] = {
	RCP_OP_LEN_MI,     /* type = 000 */
	RCP_OP_LEN_MISC,   /* type = 001 */
	RCP_OP_LEN_2D,     /* type = 010 */
	RCP_OP_LEN_3D_MEDIA, /* type = 011 */
	0,
	0,
	0,
	0,
};

/* Video Codec Command Parser */
static unsigned int vccp_op_len[] = {
	VCCS_OP_LEN_MI,     /* type = 000 */
	0,
	0,
	VCCS_OP_LEN_MFX_VC, /* type = 011 */
	0,
	0,
	0,
	0,
};

static unsigned int* ring_opcode_len[MAX_ENGINES] = {
	[RING_BUFFER_RCS] = rcp_op_len,
	[RING_BUFFER_VCS] = vccp_op_len,
	[RING_BUFFER_BCS] = rcp_op_len,
	[RING_BUFFER_VECS] = vccp_op_len, /* FIXME: double check VECS decode */
	[RING_BUFFER_VCS2] = vccp_op_len, /* FIXME: double check VCS2 decode */
};

uint32_t vgt_get_opcode(uint32_t cmd, int ring_id)
{

	unsigned int op_len;
	unsigned int *len_table;

	if (ring_id >= MAX_ENGINES)
		return INVALID_OP;

	len_table = ring_opcode_len[ring_id];

	/* type - bits 31:29*/
	op_len = len_table[cmd >> 29];
	if (op_len == 0)
		return INVALID_OP;

	return cmd >> (32-op_len);
}

static inline struct cmd_info* vgt_get_cmd_info(uint32_t cmd, int ring_id)
{
	uint32_t opcode;

	opcode = vgt_get_opcode(cmd, ring_id);
	if (opcode == INVALID_OP){
		return NULL;
	}


	return vgt_find_cmd_entry(opcode, ring_id);
}

static inline uint32_t* cmd_ptr(struct parser_exec_state *s, int index)
{
	if (index < s->ip_buf_len)
		return s->ip_va + index;
	else
		return s->ip_va_next_page+ (index - s->ip_buf_len);
}

static inline uint32_t cmd_val(struct parser_exec_state *s, int index)
{
	return *cmd_ptr(s, index);
}

#define RING_BUF_WRAP(s, ip)	(((s)->buf_type == RING_BUFFER_INSTRUCTION) && \
		((ip_gma) >= (s)->ring_start + (s)->ring_size))

static int ip_gma_set(struct parser_exec_state *s, unsigned long ip_gma)
{
	unsigned long gma_next_page;

	ASSERT(VGT_REG_IS_ALIGNED(ip_gma, 4));

	/* set ip_gma */

	if (RING_BUF_WRAP(s, ip_gma)){
		ip_gma = ip_gma - s->ring_size;
	}

	s->ip_gma = ip_gma;
	s->ip_va = vgt_gma_to_va(s->vgt, ip_gma,
			s->buf_addr_type == PPGTT_BUFFER);
	ASSERT(s->ip_va);

	s->ip_buf_len = (PAGE_SIZE - (ip_gma & (PAGE_SIZE-1)))
		/ sizeof(uint32_t);

	/* set ip of next page */

	if (RING_BUF_WRAP(s, ip_gma + PAGE_SIZE)){
		gma_next_page = s->ring_start;
	}else{
		gma_next_page = ((ip_gma >> PAGE_SHIFT) + 1) << PAGE_SHIFT;
	}
	s->ip_va_next_page = vgt_gma_to_va(s->vgt, gma_next_page,
			s->buf_addr_type == PPGTT_BUFFER);
	ASSERT(s->ip_va_next_page);

	return 0;
}

static inline void ip_gma_advance(struct parser_exec_state *s, unsigned int len)
{
	if (s->ip_buf_len > len){
		/* not cross page, advance ip inside page */
		s->ip_gma += len*sizeof(uint32_t);
		s->ip_va += len;
		s->ip_buf_len -= len;
	} else{
		/* cross page, reset ip_va */
		ip_gma_set(s, s->ip_gma + len*sizeof(uint32_t));
	}
}

static inline int cmd_length(struct parser_exec_state *s)
{
	struct cmd_info *info = s->info;

	if ((info->flag & F_LEN_MASK) == F_LEN_CONST)
	{
		return info->len;
	}
	else /* F_LEN_VAR */{
		return (cmd_val(s,0) & ( (1U << s->info->len) - 1)) + 2;
	}
}

static int vgt_cmd_advance_default(struct parser_exec_state *s)
{
	ip_gma_advance(s, cmd_length(s));
	return 0;
}


static int vgt_cmd_handler_mi_batch_buffer_end(struct parser_exec_state *s)
{
	s->buf_type = RING_BUFFER_INSTRUCTION;
	s->buf_addr_type = GTT_BUFFER;
	ip_gma_set(s, s->ret_instr_gma);
	return 0;
}

#define USE_GLOBAL_GTT_MASK (1U << 22)
static int vgt_cmd_handler_mi_update_gtt(struct parser_exec_state *s)
{
	uint32_t entry_num, *entry;
	int rc, i;

	/*TODO: remove this assert when PPGTT support is added */
	ASSERT(cmd_val(s,0) & USE_GLOBAL_GTT_MASK);

	address_fixup(s, 1);

	entry_num = cmd_val(s,0) & ((1U<<8) - 1); /* bit 7:0 */
	entry = v_aperture(s->vgt->pdev, cmd_val(s,1));
	for (i=0; i<entry_num; i++){
		dprintk("vgt: update GTT entry %d\n", i);
		/*TODO: optimize by batch g2m translation*/
		rc = gtt_p2m(s->vgt, entry[i], &entry[i] );
		if (rc < 0){
			/* TODO: how to handle the invalide guest value */
		}
	}

	return 0;
}

static int vgt_cmd_handler_mi_flush_dw(struct parser_exec_state* s)
{
	int i, len;

	/* Check post-sync bit */
	if ( (cmd_val(s,0) >> 14) & 0x3)
		address_fixup(s, 1);

	len = cmd_length(s);
	for (i=2; i<len; i++)
		address_fixup(s, i);

	return 0;
}

#define BATCH_BUFFER_ADDR_MASK ((1UL << 32) - (1U <<2))
#define BATCH_BUFFER_ADR_SPACE_BIT(x)	(((x)>>8) & 1U)
#define BATCH_BUFFER_2ND_LEVEL_BIT(x)   ((x)>>22 & 1U)

static void addr_type_update_snb(struct parser_exec_state* s)
{
	if ( (s->buf_type == RING_BUFFER_INSTRUCTION) &&
			(s->vgt->rb[s->ring_id].has_ppgtt_mode_enabled) &&
			(BATCH_BUFFER_ADR_SPACE_BIT(cmd_val(s,0)) == 1)
	   )
	{
		s->buf_addr_type = PPGTT_BUFFER;
	}
}

static int vgt_cmd_handler_mi_batch_buffer_start(struct parser_exec_state *s)
{
	int rc;

	/* FIXME: add 2nd level batch buffer support */
	ASSERT(BATCH_BUFFER_2ND_LEVEL_BIT(cmd_val(s,0)) == 0);

	/* FIXME: add IVB/HSW code */
	addr_type_update_snb(s);

	if (s->buf_type == RING_BUFFER_INSTRUCTION){
		s->ret_instr_gma = s->ip_gma + 2*sizeof(uint32_t);
	}

	klog_printk("MI_BATCH_BUFFER_START: Addr=%x ClearCommandBufferEnable=%d\n",
			cmd_val(s,1),  (cmd_val(s,0)>>11) & 1);

	address_fixup(s, 1);

	s->buf_type = BATCH_BUFFER_INSTRUCTION;
	rc = ip_gma_set(s, cmd_val(s,1) & BATCH_BUFFER_ADDR_MASK);

	if (rc < 0){
		printk(KERN_WARNING"invalid batch buffer addr, so skip scanning it\n");
		vgt_cmd_handler_mi_batch_buffer_end(s);
		return 0;
	}

	return 0;
}

static int vgt_cmd_handler_3dstate_vertex_buffers(struct parser_exec_state *s)
{
	int length, i;

	length = cmd_length(s);

	for (i=1; i < length; i = i+4){
		address_fixup(s,i + 1);
		address_fixup(s,i + 2);
	}

	return 0;
}

static int vgt_cmd_handler_3dstate_index_buffer(struct parser_exec_state *s)
{
	address_fixup(s,1);

	if (cmd_val(s,2) != 0)
		address_fixup(s,2);

	return 0;
}

static unsigned int constant_buffer_address_offset_disable(struct parser_exec_state *s)
{
	/* return the "CONSTANT_BUFFER Address Offset Disable" bit
	  in "INSTPM—Instruction Parser Mode Register"
	  0 - use as offset
	  1 - use as graphics address
	 */

	return VGT_MMIO_READ(s->vgt->pdev,_REG_RCS_INSTPM) & INSTPM_CONS_BUF_ADDR_OFFSET_DIS;
}

static int vgt_cmd_handler_3dstate_constant_gs(struct parser_exec_state *s)
{
	if (constant_buffer_address_offset_disable(s) == 1){
		address_fixup(s,1);
	}
	address_fixup(s,2);
	address_fixup(s,3);
	address_fixup(s,4);

	return 0;
}

static int vgt_cmd_handler_3dstate_constant_ps(struct parser_exec_state *s)
{
	if (constant_buffer_address_offset_disable(s) == 1){
		address_fixup(s,1);
	}
	address_fixup(s,2);
	address_fixup(s,3);
	address_fixup(s,4);

	return 0;
}

static int vgt_cmd_handler_3dstate_constant_vs(struct parser_exec_state *s)
{
	if (constant_buffer_address_offset_disable(s) == 1){
		address_fixup(s,1);
	}
	address_fixup(s,2);
	address_fixup(s,3);
	address_fixup(s,4);

	return 0;
}

static int vgt_cmd_handler_state_base_address(struct parser_exec_state *s)
{
	address_fixup(s,1);
	address_fixup(s,2);
	address_fixup(s,3);
	address_fixup(s,4);
	address_fixup(s,5);
	/* Zero Bound is ignore */
	if (cmd_val(s,6) >> 12)
		address_fixup(s,6);
	if (cmd_val(s,7) >> 12)
		address_fixup(s,7);
	if (cmd_val(s,8) >> 12)
		address_fixup(s,8);
	if (cmd_val(s,9) >> 12)
		address_fixup(s,9);
	return 0;
}

#if 0
	{"", OP_, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"", OP_, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"", OP_, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"", OP_, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

#endif
static struct cmd_info cmd_info[] = {
	{"MI_NOOP", OP_MI_NOOP, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_USER_INTERRUPT", OP_MI_USER_INTERRUPT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_WAIT_FOR_EVENT", OP_MI_WAIT_FOR_EVENT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_FLUSH", OP_MI_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_CHECK", OP_MI_ARB_CHECK, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_REPORT_HEAD", OP_MI_REPORT_HEAD, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_ON_OFF", OP_MI_ARB_ON_OFF, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_BATCH_BUFFER_END", OP_MI_BATCH_BUFFER_END, F_IP_ADVANCE_CUSTOM|F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, vgt_cmd_handler_mi_batch_buffer_end},

	{"MI_SUSPEND_FLUSH", OP_MI_SUSPEND_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_DISPLAY_FLIP", OP_MI_DISPLAY_FLIP, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_SEMAPHORE_MBOX", OP_MI_SEMAPHORE_MBOX, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL },

	{"MI_SET_CONTEXT", OP_MI_SET_CONTEXT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"MI_MATH", OP_MI_MATH, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_STORE_DATA_IMM", OP_MI_STORE_DATA_IMM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_DATA_INDEX", OP_MI_STORE_DATA_INDEX, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, NULL},

	{"MI_LOAD_REGISTER_IMM", OP_MI_LOAD_REGISTER_IMM, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, vgt_cmd_handler_mi_update_gtt},

	{"MI_STORE_REGISTER_MEM", OP_MI_STORE_REGISTER_MEM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_FLUSH_DW", OP_MI_FLUSH_DW, F_LEN_VAR, R_ALL, D_ALL,
		0, 6, vgt_cmd_handler_mi_flush_dw},

	{"MI_CLFLUSH", OP_MI_CLFLUSH, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"MI_REPORT_PERF_COUNT", OP_MI_REPORT_PERF_COUNT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 6, NULL},

	{"MI_BATCH_BUFFER_START", OP_MI_BATCH_BUFFER_START, F_IP_ADVANCE_CUSTOM|F_LEN_CONST,
		R_ALL, D_ALL, 0, 2, vgt_cmd_handler_mi_batch_buffer_start},

	{"MI_CONDITIONAL_BATCH_BUFFER_END", OP_MI_CONDITIONAL_BATCH_BUFFER_END,
		F_LEN_VAR, R_ALL, D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"XY_SETUP_BLT", OP_XY_SETUP_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4,7), 8, NULL},

	{"XY_SETUP_CLIP_BLT", OP_XY_SETUP_CLIP_BLT, F_LEN_VAR, R_BCS, D_ALL,
		0, 8, NULL},

	{"XY_SETUP_MONO_PATTERN_SL_BLT", OP_XY_SETUP_MONO_PATTERN_SL_BLT, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_PIXEL_BLT", OP_XY_PIXEL_BLT, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"XY_SCANLINES_BLT", OP_XY_SCANLINES_BLT, F_LEN_VAR, R_BCS, D_ALL,
		0, 8, NULL},

	{"XY_TEXT_BLT", OP_XY_TEXT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(3), 8, NULL},

	{"XY_TEXT_IMMEDIATE_BLT", OP_XY_TEXT_IMMEDIATE_BLT, F_LEN_VAR, R_BCS,
		D_ALL, 0, 8, NULL},

	{"COLOR_BLT", OP_COLOR_BLT, F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_1(3), 5, NULL},

	{"SRC_COPY_BLT", OP_SRC_COPY_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(3), 5, NULL},

	{"XY_COLOR_BLT", OP_XY_COLOR_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(4), 8, NULL},

	{"XY_PAT_BLT", OP_XY_PAT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4,5), 8, NULL},

	{"XY_MONO_PAT_BLT", OP_XY_MONO_PAT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4,5), 8, NULL},

	{"XY_SRC_COPY_BLT", OP_XY_SRC_COPY_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4,7), 8, NULL},

	{"XY_MONO_SRC_COPY_BLT", OP_XY_MONO_SRC_COPY_BLT, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_2(4,5), 8, NULL},

	{"XY_FULL_BLT", OP_XY_FULL_BLT, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"XY_FULL_MONO_SRC_BLT", OP_XY_FULL_MONO_SRC_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_3(4,5,8), 8, NULL},

	{"XY_FULL_MONO_PATTERN_BLT", OP_XY_FULL_MONO_PATTERN_BLT, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_2(4,7), 8, NULL},

	{"XY_FULL_MONO_PATTERN_MONO_SRC_BLT", OP_XY_FULL_MONO_PATTERN_MONO_SRC_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4,5), 8, NULL},

	{"XY_MONO_PAT_FIXED_BLT", OP_XY_MONO_PAT_FIXED_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(4), 8, NULL},

	{"XY_MONO_SRC_COPY_IMMEDIATE_BLT", OP_XY_MONO_SRC_COPY_IMMEDIATE_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_PAT_BLT_IMMEDIATE", OP_XY_PAT_BLT_IMMEDIATE, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_SRC_COPY_CHROMA_BLT", OP_XY_SRC_COPY_CHROMA_BLT, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_2(4,7), 8, NULL},

	{"XY_FULL_IMMEDIATE_PATTERN_BLT", OP_XY_FULL_IMMEDIATE_PATTERN_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4,7), 8, NULL},

	{"XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT", OP_XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4,5), 8, NULL},

	{"XY_PAT_CHROMA_BLT", OP_XY_PAT_CHROMA_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4,5), 8, NULL},

	{"XY_PAT_CHROMA_BLT_IMMEDIATE", OP_XY_PAT_CHROMA_BLT_IMMEDIATE, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS", OP_3DSTATE_BINDING_TABLE_POINTERS,
		F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_VS", OP_3DSTATE_BINDING_TABLE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_HS", OP_3DSTATE_BINDING_TABLE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_DS", OP_3DSTATE_BINDING_TABLE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_GS", OP_3DSTATE_BINDING_TABLE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_PS", OP_3DSTATE_BINDING_TABLE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS", OP_3DSTATE_SAMPLER_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_URB", OP_3DSTATE_URB, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_VERTEX_BUFFERS", OP_3DSTATE_VERTEX_BUFFERS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_vertex_buffers},

	{"3DSTATE_VERTEX_ELEMENTS", OP_3DSTATE_VERTEX_ELEMENTS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_INDEX_BUFFER", OP_3DSTATE_INDEX_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_index_buffer},

	{"3DSTATE_VF_STATISTICS", OP_3DSTATE_VF_STATISTICS, F_LEN_CONST,
		R_RCS, D_ALL, 0, 1, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS", OP_3DSTATE_VIEWPORT_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CC_STATE_POINTERS", OP_3DSTATE_CC_STATE_POINTERS, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SCISSOR_STATE_POINTERS", OP_3DSTATE_SCISSOR_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GS", OP_3DSTATE_GS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CLIP", OP_3DSTATE_CLIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_WM", OP_3DSTATE_WM, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_GS", OP_3DSTATE_CONSTANT_GS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_constant_gs},

	{"3DSTATE_CONSTANT_PS", OP_3DSTATE_CONSTANT_PS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_constant_ps},

	{"3DSTATE_SAMPLE_MASK", OP_3DSTATE_SAMPLE_MASK, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_DRAWING_RECTANGLE", OP_3DSTATE_DRAWING_RECTANGLE, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD0", OP_3DSTATE_SAMPLER_PALETTE_LOAD0,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CHROMA_KEY", OP_3DSTATE_CHROMA_KEY, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_DEPTH_BUFFER", OP_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_POLY_STIPPLE_OFFSET", OP_3DSTATE_POLY_STIPPLE_OFFSET,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_POLY_STIPPLE_PATTERN", OP_3DSTATE_POLY_STIPPLE_PATTERN,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_LINE_STIPPLE", OP_3DSTATE_LINE_STIPPLE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_AA_LINE_PARAMS", OP_3DSTATE_AA_LINE_PARAMS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_GS_SVB_INDEX", OP_3DSTATE_GS_SVB_INDEX, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD1", OP_3DSTATE_SAMPLER_PALETTE_LOAD1,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_MULTISAMPLE", OP_3DSTATE_MULTISAMPLE, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_STENCIL_BUFFER", OP_3DSTATE_STENCIL_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_HIER_DEPTH_BUFFER", OP_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_CLEAR_PARAMS", OP_3DSTATE_CLEAR_PARAMS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_MONOFILTER_SIZE", OP_3DSTATE_MONOFILTER_SIZE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_SO_DECL_LIST", OP_3DSTATE_SO_DECL_LIST, F_LEN_VAR, R_RCS, D_ALL,
		0, 9, NULL},

	{"3DSTATE_SO_BUFFER", OP_3DSTATE_SO_BUFFER, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_2(2,3), 8, NULL},

	{"PIPE_CONTROL", OP_PIPE_CONTROL, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"3DPRIMITIVE", OP_3DPRIMITIVE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"PIPELINE_SELECT", OP_PIPELINE_SELECT, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"STATE_PREFETCH", OP_STATE_PREFETCH, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"STATE_SIP", OP_STATE_SIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"STATE_BASE_ADDRESS", OP_STATE_BASE_ADDRESS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, vgt_cmd_handler_state_base_address},

	{"3DSTATE_VS", OP_3DSTATE_VS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SF", OP_3DSTATE_SF, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_VS", OP_3DSTATE_CONSTANT_VS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, vgt_cmd_handler_3dstate_constant_vs},

	{"MEDIA_INTERFACE_DESCRIPTOR_LOAD", OP_MEDIA_INTERFACE_DESCRIPTOR_LOAD,
		F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"MEDIA_GATEWAY_STATE", OP_MEDIA_GATEWAY_STATE, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_STATE_FLUSH", OP_MEDIA_STATE_FLUSH, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT", OP_MEDIA_OBJECT, F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"MEDIA_CURBE_LOAD", OP_MEDIA_CURBE_LOAD, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT_PRT", OP_MEDIA_OBJECT_PRT, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT_WALKER", OP_MEDIA_OBJECT_WALKER, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_VFE_STATE", OP_MEDIA_VFE_STATE, F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"3DSTATE_VF_STATISTICS_GM45", OP_3DSTATE_VF_STATISTICS_GM45, F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, NULL},
};

static int cmd_hash_init(void)
{
	int i;
	struct vgt_cmd_entry *e;
	struct cmd_info	*info;

	for (i=0; i< ARRAY_SIZE(cmd_info); i++){
		/* TODO: only register current GEN cmd */
		e = kmalloc(sizeof(*e), GFP_KERNEL);
		if (e == NULL) {
			printk("Insufficient memory in %s\n", __FUNCTION__);
			return -ENOMEM;
		}
		e->info = &cmd_info[i];
		printk("register %s opcode=%x flag=%x\n", e->info->name, e->info->opcode, e->info->flag);
		info = vgt_find_cmd_entry_any_ring(e->info->opcode, e->info->rings);
		if (info){
			printk("%s %s duplicated\n", e->info->name, info->name);
			BUG();
		}
		INIT_HLIST_NODE(&e->hlist);
		vgt_add_cmd_entry(e);
	}
	return 0;
}

/* call the cmd handler, and advance ip */
static int vgt_cmd_parser_exec(struct parser_exec_state *s)
{
	struct cmd_info *info;

	int rc = 0;

	info = vgt_get_cmd_info(*s->ip_va, s->ring_id);
	if(info == NULL){
		printk(KERN_ERR"ERROR: unknown cmd %x, ring%d[%lx,%lx] gma[%lx] va[%p]\n",
				*s->ip_va, s->ring_id, s->ring_start,
				s->ring_start + s->ring_size, s->ip_gma, s->ip_va);
		klog_printk("ERROR: unknown cmd %x, ring%d[%lx,%lx] gma[%lx] va[%p]\n",
				*s->ip_va, s->ring_id, s->ring_start,
				s->ring_start + s->ring_size, s->ip_gma, s->ip_va);
		printk("opcode=%x\n", vgt_get_opcode(*s->ip_va, s->ring_id));

		return -EINVAL;
	}

	s->info = info;

#ifdef VGT_ENABLE_ADDRESS_FIX
	{
		unsigned int bit;
		for_each_set_bit(bit, (unsigned long*)&info->addr_bitmap, 8*sizeof(info->addr_bitmap))
			address_fixup(s, bit);
	}
#endif

	if (info->handler){
		rc = info->handler(s);
		if (rc < 0){
			printk("%s: %s handler error", __func__, info->name);
			return rc;
		}
	}

	if (!(info->flag & F_IP_ADVANCE_CUSTOM)){
		vgt_cmd_advance_default(s);
	}

	return rc;
}

static inline void stat_nr_cmd_inc(struct parser_exec_state *s)
{
	vgt_state_ring_t* rs;

	rs = &s->vgt->rb[s->ring_id];

	if (s->buf_type == RING_BUFFER_INSTRUCTION)
		rs->nr_cmd_ring++;
	else
		rs->nr_cmd_batch++;
	return;
}

static int __vgt_scan_vring(struct vgt_device *vgt, int ring_id, vgt_reg_t head, vgt_reg_t tail, vgt_reg_t base, vgt_reg_t size)
{
	static int error_count=0;
	unsigned long ip_gma_end;
	struct parser_exec_state s;
	int ret=0;

	if (error_count > 10)
		return 0;

	/* ring base is page aligned */
	ASSERT((base & (PAGE_SIZE-1)) == 0);

	ip_gma_end = base + tail;

	s.buf_type = RING_BUFFER_INSTRUCTION;
	s.buf_addr_type = GTT_BUFFER;
	s.vgt = vgt;
	s.ring_id = ring_id;
	s.ring_start = base;
	s.ring_size = size;
	ip_gma_set(&s, base + head);

	klog_printk("ring buffer scan start on ring %d\n", ring_id);
	dprintk("scan_start: start=%x end=%x\n", base+head, base+tail);
	while(s.ip_gma != ip_gma_end){
		klog_printk("%s ip(%08lx): %08x %08x %08x %08x\n ",
				s.buf_type == RING_BUFFER_INSTRUCTION ? "RB": "BB",
				s.ip_gma, cmd_val(&s,0), cmd_val(&s,1),
				cmd_val(&s,2), cmd_val(&s,3));
#if 0
		printk("%s ip(%08lx): %08x %08x %08x %08x \n",
				s.buf_type == RING_BUFFER_INSTRUCTION ? "RB": "BB",
				s.ip_gma, cmd_val(&s,0), cmd_val(&s,1),
				cmd_val(&s,2), cmd_val(&s,3));
#endif

		stat_nr_cmd_inc(&s);

		ret = vgt_cmd_parser_exec(&s);
		if (ret < 0){
			error_count++;
			printk("error_count=%d\n", error_count);
			klog_printk("error_count=%d\n", error_count);
			break;
		}
	}
	klog_printk("ring buffer scan end on ring %d\n", ring_id);
	dprintk("scan_end\n");
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

int vgt_cmd_parser_init(void)
{
	return cmd_hash_init();
}

void vgt_cmd_parser_exit(void)
{
	vgt_clear_cmd_table();
}


#endif /* VGT_PARSER_OLD */
