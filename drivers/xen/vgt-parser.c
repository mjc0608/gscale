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

#include <linux/linkage.h>
#include <linux/module.h>
#include <xen/vgt-parser.h>

/*
 * parse and handle the gfx command
 * which usually in ring buffer
 *
 * IN: instruction - mapped instruction address
 *
 * Return: the instruction bytes that have been handled
 *         -1 if error, e.g. command not recognized
 */

static unsigned int vgt_address_get_vmbase(void)
{
	/* FIXME: get the aperture VM BASE */
	return 0;
}

static unsigned int constant_buffer_address_offset_disable(void)
{
	/* return the "CONSTANT_BUFFER Address Offset Disable" bit
	  in "INSTPM—Instruction Parser Mode Register"
	  0 - use as offset
	  1 - use as graphics address
	 */

	/*FIXME: add implementation*/

	return 0;
}

static unsigned int vgt_address_get_vmlength(void)
{
	/* FIXME: get the aperture VM LENGTH*/
	return (256<<20); /* 256M */
}

static int address_fixup(unsigned int *addr)
{
	if (*addr >= vgt_address_get_vmlength())
	{
		/* FIXME: raise out of bound violation */
		return -1;
	}

	if(*addr == 0)
		return 0;

	*addr = *addr + vgt_address_get_vmbase();
	return 0;
}

static inline void length_fixup(struct gen_gfx_cmd_decode_data *data, int nr_bits)
{
	/*  DWord Length is bits (nr_bits-1):0 */
	int dword_length = data->data & ( (1U << nr_bits) - 1);
	data->instruction += dword_length + 1;
}

static int vgt_cmd_handler_noop(struct gen_gfx_cmd_decode_data *data)
{
	data->instruction += 1;
	return 0;
}

static int vgt_cmd_handler_length_fixup_8(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data, 8);
	return 0;
}

static int vgt_cmd_handler_length_fixup_16(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data, 16);
	return 0;
}

static void batch_buffer_addr_push(unsigned int *addr)
{
	/* TODO: push the addr to stack */
	/* the stack should be per-VM, also need add lock */
}

static unsigned int* batch_buffer_addr_pop(void)
{
	/* TODO: pop the addr from stack */
	/* the stack should be per-VM, also need add lock */
	return NULL;
}

static int vgt_cmd_handler_mi_batch_buffer_end(struct gen_gfx_cmd_decode_data *data)
{
	/* FIXME: pop up the stack for ring buffer */
	data->instruction = batch_buffer_addr_pop();
	return 0;
}

static int vgt_cmd_handler_mi_conditional_batch_buffer_end(struct gen_gfx_cmd_decode_data *data)
{
	/* FIXME: currently ignore the "Use Global GTT" bit, and assume always using GGTT.
	   need add PPGTT support later */

	/* TODO: handle the DWord Length */
	address_fixup(data->instruction + 2);
	length_fixup(data,8);

	return 0;
}

static int vgt_cmd_handler_mi_display_flip(struct gen_gfx_cmd_decode_data *data)
{
	/* TODO: handle the DWord Length */
	address_fixup(data->instruction + 2);
	length_fixup(data,8);

	return 0;
}
static int vgt_cmd_handler_mi_semaphore_mbox(struct gen_gfx_cmd_decode_data *data)
{
	/* TODO: handle the DWord Length */

	if (!(data->data & (1U<<18))){
		/* memory address, not MMIO register */
		address_fixup(data->instruction + 2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_set_context(struct gen_gfx_cmd_decode_data *data)
{
	/* TODO: handle the DWord Length */
	address_fixup(data->instruction + 1);
	length_fixup(data,8);

	return 0;
}

static int vgt_cmd_handler_mi_math(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_data_imm(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_data_index(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_load_register_imm(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_update_gtt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_store_register_mem(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_flush_dw(struct gen_gfx_cmd_decode_data *data)
{
	/* Check post-sync bit */
	if ( (data->data >> 14) & 0x3){
		address_fixup(data->instruction + 1);
	}

	length_fixup(data,6);
	return 0;
}

static int vgt_cmd_handler_mi_clflush(struct gen_gfx_cmd_decode_data *data)
{
	/* TODO: may need to check if field "DW Representing 1/2 Cache Line" is out of bound */
	address_fixup(data->instruction + 1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_mi_report_perf_count(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 1);
	length_fixup(data,6);
	return 0;
}

static unsigned int* gtt_address_map(unsigned int addr)
{
	/* todo: return the mapping of gtt address: addr */
	return 0;
}

static int vgt_cmd_handler_mi_batch_buffer_start(struct gen_gfx_cmd_decode_data *data)
{
	batch_buffer_addr_push(data->instruction + (data->data & 0xff) +2);
	data->instruction = gtt_address_map(*(data->instruction + 1)) - 1;
	return 0;
}

static int vgt_cmd_handler_xy_setup_blt(struct gen_gfx_cmd_decode_data *data)
{
	/* Destination Base Address */
	address_fixup(data->instruction + 4);
	/* Pattern Base Address for Color Pattern */
	address_fixup(data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_setup_clip_blt(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_setup_mono_pattern_sl_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pixel_blt(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_scanlines_blt(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_text_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 3);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_color_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_src_copy_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 3);
	length_fixup(data,5);
	return 0;
}

static int vgt_cmd_handler_xy_color_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_blt(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_src_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	address_fixup(data->instruction + 8);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_mono_pattern_mono_src_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_pat_fixed_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_mono_src_copy_immediate_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_blt_immediate(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_src_copy_chroma_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 7);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_full_immediate_pattern_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 7);
	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_xy_full_mono_src_immediate_pattern_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_xy_pat_chroma_blt_immediate(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 4);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_vertex_buffers(struct gen_gfx_cmd_decode_data *data)
{
	int dword_length, i;

	dword_length = data->data & ( (1U << 8) - 1);

	for (i=1; i <= dword_length - 2 ; i = i+4){
		address_fixup(data->instruction + i + 1);
		address_fixup(data->instruction + i + 2);
	}

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_index_buffer(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 1);

	if (*(data->instruction + 2) != 0)
		address_fixup(data->instruction + 2);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_gs(struct gen_gfx_cmd_decode_data *data)
{
	if (constant_buffer_address_offset_disable() == 1){
		address_fixup(data->instruction + 1);
	}
	address_fixup(data->instruction + 2);
	address_fixup(data->instruction + 3);
	address_fixup(data->instruction + 4);

	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_ps(struct gen_gfx_cmd_decode_data *data)
{
	if (constant_buffer_address_offset_disable() == 1){
		address_fixup(data->instruction + 1);
	}
	address_fixup(data->instruction + 2);
	address_fixup(data->instruction + 3);
	address_fixup(data->instruction + 4);

	length_fixup(data,8);
	return 0;
}


static int vgt_cmd_handler_3dstate_depth_buffer(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_stencil_buffer(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_hier_depth_buffer(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_so_buffer(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	address_fixup(data->instruction + 3);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_so_decl_list(struct gen_gfx_cmd_decode_data *data)
{
	length_fixup(data,9);
	return 0;
}

static int vgt_cmd_handler_pipe_control(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 2);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_prefetch(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 1);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_state_base_address(struct gen_gfx_cmd_decode_data *data)
{
	address_fixup(data->instruction + 1);
	address_fixup(data->instruction + 2);
	address_fixup(data->instruction + 3);
	address_fixup(data->instruction + 4);
	address_fixup(data->instruction + 5);
	address_fixup(data->instruction + 6);
	address_fixup(data->instruction + 7);
	address_fixup(data->instruction + 8);
	address_fixup(data->instruction + 9);
	length_fixup(data,8);
	return 0;
}

static int vgt_cmd_handler_3dstate_constant_vs(struct gen_gfx_cmd_decode_data *data)
{
	if (constant_buffer_address_offset_disable() == 1){
		address_fixup(data->instruction + 1);
	}
	address_fixup(data->instruction + 2);
	address_fixup(data->instruction + 3);
	address_fixup(data->instruction + 4);

	length_fixup(data,8);
	return 0;
}

/* FIXME: use hashtable to optimize the space */
static struct vgt_cmd_handlers cmd_handlers[GEN_GFX_CMD_TYPE_MAX + 1];

static int vgt_cmd_register_default(void)
{

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_NOOP,
			"MI_NOOP", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_USER_INTERRUPT,
			"MI_USER_INTERRUPT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_WAIT_FOR_EVENT,
			"MI_WAIT_FOR_EVENT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_FLUSH,
			"MI_FLUSH", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_ARB_CHECK,
			"MI_ARB_CHECK", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_REPORT_HEAD,
			"MI_REPORT_HEAD", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_ARB_ON_OFF,
			"MI_ARB_ON_OFF", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_BATCH_BUFFER_END,
			"MI_BATCH_BUFFER_END",	vgt_cmd_handler_mi_batch_buffer_end);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_SUSPEND_FLUSH,
			"MI_SUSPEND_FLUSH", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_DISPLAY_FLIP,
			"MI_DISPLAY_FLIP",	vgt_cmd_handler_mi_display_flip);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_SEMAPHORE_MBOX,
			"MI_SEMAPHORE_MBOX", vgt_cmd_handler_mi_semaphore_mbox);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_SET_CONTEXT,
			"MI_SET_CONTEXT", vgt_cmd_handler_mi_set_context);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_MATH,
			"MI_MATH", vgt_cmd_handler_mi_math);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_STORE_DATA_IMM,
			"MI_STORE_DATA_IMM", vgt_cmd_handler_mi_store_data_imm);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_STORE_DATA_INDEX,
	"MI_STORE_DATA_INDEX", vgt_cmd_handler_mi_store_data_index);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_LOAD_REGISTER_IMM,
	"MI_LOAD_REGISTER_IMM", vgt_cmd_handler_mi_load_register_imm);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_UPDATE_GTT,
	"MI_UPDATE_GTT", vgt_cmd_handler_mi_update_gtt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_STORE_REGISTER_MEM,
	"MI_STORE_REGISTER_MEM", vgt_cmd_handler_mi_store_register_mem);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_FLUSH_DW,
	"MI_FLUSH_DW", vgt_cmd_handler_mi_flush_dw);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_CLFLUSH,
	"MI_CLFLUSH", vgt_cmd_handler_mi_clflush);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_REPORT_PERF_COUNT,
	"MI_REPORT_PERF_COUNT", vgt_cmd_handler_mi_report_perf_count);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_BATCH_BUFFER_START,
	"MI_BATCH_BUFFER_START", vgt_cmd_handler_mi_batch_buffer_start);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_MI, MI_CONDITIONAL_BATCH_BUFFER_END,
			"MI_CONDITIONAL_BATCH_BUFFER_END", vgt_cmd_handler_mi_conditional_batch_buffer_end);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SETUP_BLT,
	"XY_SETUP_BLT", vgt_cmd_handler_xy_setup_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SETUP_CLIP_BLT,
	"XY_SETUP_CLIP_BLT", vgt_cmd_handler_xy_setup_clip_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SETUP_MONO_PATTERN_SL_BLT,
	"XY_SETUP_MONO_PATTERN_SL_BLT", vgt_cmd_handler_xy_setup_mono_pattern_sl_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_PIXEL_BLT,
	"XY_PIXEL_BLT", vgt_cmd_handler_xy_pixel_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SCANLINES_BLT,
	"XY_SCANLINES_BLT", vgt_cmd_handler_xy_scanlines_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_TEXT_BLT,
	"XY_TEXT_BLT", vgt_cmd_handler_xy_text_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_TEXT_IMMEDIATE_BLT,
	"XY_TEXT_IMMEDIATE_BLT", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, COLOR_BLT,
	"COLOR_BLT", vgt_cmd_handler_color_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, SRC_COPY_BLT,
	"SRC_COPY_BLT", vgt_cmd_handler_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_COLOR_BLT,
	"XY_COLOR_BLT", vgt_cmd_handler_xy_color_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_PAT_BLT,
	"XY_PAT_BLT", vgt_cmd_handler_xy_pat_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_MONO_PAT_BLT,
	"XY_MONO_PAT_BLT", vgt_cmd_handler_xy_mono_pat_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SRC_COPY_BLT,
	"XY_SRC_COPY_BLT", vgt_cmd_handler_xy_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_MONO_SRC_COPY_BLT,
	"XY_MONO_SRC_COPY_BLT", vgt_cmd_handler_xy_mono_src_copy_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_BLT,
	"XY_FULL_BLT", vgt_cmd_handler_xy_full_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_MONO_SRC_BLT,
	"XY_FULL_MONO_SRC_BLT", vgt_cmd_handler_xy_full_mono_src_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_MONO_PATTERN_BLT,
	"XY_FULL_MONO_PATTERN_BLT", vgt_cmd_handler_xy_full_mono_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_MONO_PATTERN_MONO_SRC_BLT,
	"XY_FULL_MONO_PATTERN_MONO_SRC_BLT", vgt_cmd_handler_xy_full_mono_pattern_mono_src_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_MONO_PAT_FIXED_BLT,
	"XY_MONO_PAT_FIXED_BLT", vgt_cmd_handler_xy_mono_pat_fixed_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_MONO_SRC_COPY_IMMEDIATE_BLT,
	"XY_MONO_SRC_COPY_IMMEDIATE_BLT", vgt_cmd_handler_xy_mono_src_copy_immediate_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_PAT_BLT_IMMEDIATE,
	"XY_PAT_BLT_IMMEDIATE", vgt_cmd_handler_xy_pat_blt_immediate);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_SRC_COPY_CHROMA_BLT,
	"XY_SRC_COPY_CHROMA_BLT", vgt_cmd_handler_xy_src_copy_chroma_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_IMMEDIATE_PATTERN_BLT,
	"XY_FULL_IMMEDIATE_PATTERN_BLT", vgt_cmd_handler_xy_full_immediate_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT,
	"XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT", vgt_cmd_handler_xy_full_mono_src_immediate_pattern_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_PAT_CHROMA_BLT,
	"XY_PAT_CHROMA_BLT", vgt_cmd_handler_xy_pat_chroma_blt);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_RENDER_2D, XY_PAT_CHROMA_BLT_IMMEDIATE,
	"XY_PAT_CHROMA_BLT_IMMEDIATE", vgt_cmd_handler_xy_pat_chroma_blt_immediate);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_BINDING_TABLE_POINTERS,
	"3DSTATE_BINDING_TABLE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SAMPLER_STATE_POINTERS,
	"3DSTATE_SAMPLER_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_URB,
	"3DSTATE_URB", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_VERTEX_BUFFERS,
	"_3DSTATE_VERTEX_BUFFERS", vgt_cmd_handler_3dstate_vertex_buffers);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_VERTEX_ELEMENTS,
	"3DSTATE_VERTEX_ELEMENTS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_INDEX_BUFFER,
	"_3DSTATE_INDEX_BUFFER", vgt_cmd_handler_3dstate_index_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_VF_STATISTICS,
	"_3DSTATE_VF_STATISTICS", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_VIEWPORT_STATE_POINTERS,
	"_3DSTATE_VIEWPORT_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CC_STATE_POINTERS,
	"_3DSTATE_CC_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SCISSOR_STATE_POINTERS,
	"_3DSTATE_SCISSOR_STATE_POINTERS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_GS,
	"_3DSTATE_GS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CLIP,
	"_3DSTATE_CLIP", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_WM,
	"_3DSTATE_WM", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CONSTANT_GS,
	"_3DSTATE_CONSTANT_GS", vgt_cmd_handler_3dstate_constant_gs);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CONSTANT_PS,
	"_3DSTATE_CONSTANT_PS", vgt_cmd_handler_3dstate_constant_ps);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SAMPLE_MASK,
	"_3DSTATE_SAMPLE_MASK", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_DRAWING_RECTANGLE,
	"_3DSTATE_DRAWING_RECTANGLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SAMPLER_PALETTE_LOAD0,
	"_3DSTATE_SAMPLER_PALETTE_LOAD0", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CHROMA_KEY,
	"_3DSTATE_CHROMA_KEY", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_DEPTH_BUFFER,
	"_3DSTATE_DEPTH_BUFFER", vgt_cmd_handler_3dstate_depth_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_POLY_STIPPLE_OFFSET,
	"_3DSTATE_POLY_STIPPLE_OFFSET", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_POLY_STIPPLE_PATTERN,
	"_3DSTATE_POLY_STIPPLE_PATTERN", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_LINE_STIPPLE,
	"_3DSTATE_LINE_STIPPLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_AA_LINE_PARAMS,
	"_3DSTATE_AA_LINE_PARAMS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_GS_SVB_INDEX,
	"_3DSTATE_GS_SVB_INDEX", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SAMPLER_PALETTE_LOAD1,
	"_3DSTATE_SAMPLER_PALETTE_LOAD1", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_MULTISAMPLE,
	"_3DSTATE_MULTISAMPLE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_STENCIL_BUFFER,
	"_3DSTATE_STENCIL_BUFFER", vgt_cmd_handler_3dstate_stencil_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_HIER_DEPTH_BUFFER,
	"_3DSTATE_HIER_DEPTH_BUFFER", vgt_cmd_handler_3dstate_hier_depth_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CLEAR_PARAMS,
	"_3DSTATE_CLEAR_PARAMS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_MONOFILTER_SIZE,
	"_3DSTATE_MONOFILTER_SIZE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SO_DECL_LIST,
	"_3DSTATE_SO_DECL_LIST", vgt_cmd_handler_3dstate_so_decl_list);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SO_BUFFER,
	"_3DSTATE_SO_BUFFER", vgt_cmd_handler_3dstate_so_buffer);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, PIPE_CONTROL,
	"PIPE_CONTROL", vgt_cmd_handler_pipe_control);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DPRIMITIVE,
	"_3DPRIMITIVE", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, PIPELINE_SELECT,
	"PIPELINE_SELECT", vgt_cmd_handler_noop);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, STATE_PREFETCH,
	"STATE_PREFETCH", vgt_cmd_handler_state_prefetch);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, STATE_SIP,
	"STATE_SIP", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, STATE_BASE_ADDRESS,
	"STATE_BASE_ADDRESS", vgt_cmd_handler_state_base_address);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_VS,
	"_3DSTATE_VS", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_SF,
	"_3DSTATE_SF", vgt_cmd_handler_length_fixup_8);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, _3DSTATE_CONSTANT_VS,
	"_3DSTATE_CONSTANT_VS", vgt_cmd_handler_3dstate_constant_vs);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_INTERFACE_DESCRIPTOR_LOAD,
	"MEDIA_INTERFACE_DESCRIPTOR_LOAD", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_GATEWAY_STATE,
	"MEDIA_GATEWAY_STATE", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_STATE_FLUSH,
	"MEDIA_STATE_FLUSH", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_OBJECT,
	"MEDIA_OBJECT", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_CURBE_LOAD,
	"MEDIA_CURBE_LOAD", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_OBJECT_PRT,
	"MEDIA_OBJECT_PRT", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_OBJECT_WALKER,
	"MEDIA_OBJECT_WALKER", vgt_cmd_handler_length_fixup_16);

	vgt_cmd_handler_register(GEN_GFX_CMD_TYPE_GFXPIPE, MEDIA_VFE_STATE,
	"MEDIA_VFE_STATE", vgt_cmd_handler_length_fixup_16);

	return 0;
}

int cmd_handlers_init(void)
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

	return 0;
}

int vgt_cmd_handler_register(unsigned int type, unsigned int opcode,
		char* name, int (*handler)(struct gen_gfx_cmd_decode_data *data))
{
	if (type > GEN_GFX_CMD_TYPE_MAX || cmd_handlers[type].handlers == NULL)
	{
		printk(KERN_ERR "unsupported command handler type %d", type);
		return -ERANGE;
	}

	if (opcode >= cmd_handlers[type].len)
	{
		printk(KERN_ERR "unsupported command handler opcode 0x%x", opcode);
		return -ERANGE;
	}

	cmd_handlers[type].handlers[opcode].name = name;
	cmd_handlers[type].handlers[opcode].handler = handler;

	return 0;
}

static int vgt_cmd_handler_exec(struct gen_gfx_cmd_decode_data *decode_data)
{
	unsigned int type, opcode;

	type = decode_data->type;
	opcode = decode_data->opcode;

	if ( cmd_handlers[type].handlers == NULL ||
			opcode >= cmd_handlers[type].len ||
			cmd_handlers[type].handlers[opcode].handler == NULL){
		printk(KERN_ERR "vgt: unsupported cmd type %d opcode %xh\n", type, opcode);
		return  -VGT_UNHANDLEABLE;
	}

	return cmd_handlers[type].handlers[opcode].handler(decode_data);
}

int vgt_cmd_parser_render(unsigned int* instruction)
{
	union gen_gfx_render_cmd  command;
	struct gen_gfx_cmd_decode_data decode_data;
	unsigned int index;
	int ret = VGT_UNHANDLEABLE;

	command.raw = *instruction;
	decode_data.instruction = instruction;
	decode_data.type = command.common.type;

	switch (decode_data.type){
		case GEN_GFX_CMD_TYPE_RENDER_MI:
			decode_data.opcode = command.mi.opcode;
			decode_data.data = command.mi.data;

			ret = vgt_cmd_handler_exec(&decode_data);
			break;

		case GEN_GFX_CMD_TYPE_RENDER_MISC:
			break;

		case GEN_GFX_CMD_TYPE_RENDER_2D:
			decode_data.opcode = command.cmd_2d.opcode;
			decode_data.data = command.cmd_2d.data;

			ret = vgt_cmd_handler_exec(&decode_data);
			break;

		case GEN_GFX_CMD_TYPE_GFXPIPE:

			decode_data.sub_type= command.cmd_3d_media.sub_type;
			decode_data.opcode = command.cmd_3d_media.opcode;
			decode_data.sub_opcode = command.cmd_3d_media.sub_opcode;
			decode_data.data = command.cmd_3d_media.data;
			decode_data.count = command.cmd_3d_media.count;
			index = VGT_RENDER_3D_MEDIA_INDEX(decode_data.sub_type, \
					decode_data.opcode, decode_data.sub_opcode);

			if (index > VGT_RENDER_3D_MEDIA_OPCODE_MAX ||
				cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].handler == NULL){
				printk(KERN_ERR "vgt: unsupported 3D_MEDIA instruction"
						"sub_type=%xh opcode=%xh sub_opcode=%xh\n",
						decode_data.sub_type, decode_data.opcode,
						decode_data.sub_opcode);
				ret = -VGT_UNHANDLEABLE;
				goto out;
			}

			ret = cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].handler(&decode_data);

			break;

		default:
			printk(KERN_ERR "vgt: unsupported command type %xh\n",decode_data.type);
			ret = VGT_UNHANDLEABLE;
			break;
	}
out:
	return ret;
}
