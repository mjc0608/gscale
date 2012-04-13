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
#include <xen/interface/memory.h>
#include <asm/xen/hypercall.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_reg.h"

/*
 * parse and handle the gfx command
 * which usually in ring buffer
 *
 * IN: instruction - mapped instruction address
 *
 * Return: the instruction bytes that have been handled
 *         -1 if error, e.g. command not recognized
 */

#define VGT_CMD_PRINTK(fmt, arg...) {			\
	    if (vgt_cmd_debug) {				\
		printk(fmt, ##arg);		\
	    }					\
	}

static bool vgt_cmd_debug=0;

static void show_instruction_info(struct vgt_cmd_data *d);

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

static int address_fixup(struct vgt_cmd_data *d, uint32_t *addr)
{

	uint32_t val = *addr;

	/* address zero is usually used as NULL pointer, so do not translate */
	if(val == 0)
	{
		VGT_CMD_PRINTK(KERN_WARNING "vgt: NULL pointer in %p, no translation", addr);
		return 0;
	}

	if (gm_in_vm_range(d->vgt->pdev, d->vgt->vm_id, val)){
		/* address already translated before, do nothing but return */
		VGT_CMD_PRINTK(KERN_WARNING "vgt: address 0x%x in %p already translated\n",
				val, addr);
		return 0;
	}

	if (val < vm_gm_sz(d->vgt->pdev)){
		*addr = g2h_gm(d->vgt, val);
		return 0;
	}

	/* TODO: the address is out of range, raise fault? */
	printk(KERN_WARNING "vgt: address 0x%x in %p out of bound\n", val, addr);

	return -VGT_UNHANDLEABLE;
}

static inline void length_fixup(struct vgt_cmd_data *data, int nr_bits)
{
	/*  DWord Length is bits (nr_bits-1):0 */
	int dword_length = data->instruction[0] & ( (1U << nr_bits) - 1);
	data->instruction += dword_length + 1;
}

static int vgt_cmd_handler_noop(struct vgt_cmd_data *data)
{
	data->instruction += 1;
	return 0;
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
unsigned long g2m_pfn(int vm_id, unsigned long g_pfn)
{
	struct xen_get_mfn_from_pfn pfn_arg;
	int rc;
	unsigned long pfn_list[1];

	pfn_list[0] = g_pfn;

	set_xen_guest_handle(pfn_arg.pfn_list, pfn_list);
	pfn_arg.nr_pfns = 1;
	pfn_arg.domid = vm_id;

	rc = HYPERVISOR_memory_op(XENMEM_get_mfn_from_pfn, &pfn_arg);
	if(rc < 0){
		printk(KERN_ERR "failed to get mfn for gpfn(0x%lx)\n, errno=%d\n", g_pfn,rc);
		return INVALID_MFN;
	}

	return pfn_list[0];
}
/*
 * IN:  p_gtt_val - guest GTT entry
 * OUT: m_gtt_val - translated machine GTT entry from guest GTT entry
 *					on success, it will be written with correct value
 *					otherwise, it will not be written
 */
int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val)
{
	gtt_pte_t pte, *p_pte;
	unsigned long g_pfn, mfn;

	p_pte = &pte;
	gtt_pte_make(p_pte, p_gtt_val);

	if (!gtt_pte_valid(p_pte)){
		*m_gtt_val = p_gtt_val;
		return 0;
	}

	g_pfn = gtt_pte_get_pfn(p_pte);
	mfn = g2m_pfn(vgt->vm_id, g_pfn);
	if (mfn == INVALID_MFN){
		printk(KERN_ERR "Invalid gtt entry 0x%x\n", p_gtt_val);
		return -EINVAL;
	}
	gtt_pte_set_pfn(p_pte, mfn);

	*m_gtt_val = gtt_pte_get_val(p_pte);

	return 0;
}

static int vgt_cmd_handler_mi_update_gtt(struct vgt_cmd_data *data)
{
	uint32_t entry_num, *entry;
	int rc, i;
	gtt_pte_t pte, *p_pte;

	/*TODO: remove this assert when PPGTT support is added */
	ASSERT(data->instruction[0] & USE_GLOBAL_GTT_MASK);
	printk("mi_update_gtt\n");

	rc = address_fixup(data,data->instruction + 1);
	if (rc < 0){
		/* invalid GTT table address */
		printk(KERN_ERR "vgt: invalid GTT table address\n");
		return rc;
	}

	entry_num = data->instruction[0] & ((1U<<8) - 1); /* bit 7:0 */
	entry = v_aperture(data->vgt->pdev, data->instruction[1]);
	p_pte = &pte;
	for (i=0; i<entry_num; i++){
		printk("vgt: update GTT entry %d\n", i);
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
	/* Check post-sync bit */
	if ( (data->data >> 14) & 0x3){
		address_fixup(data,data->instruction + 1);
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

/* bit 31:2 */
#define BATCH_BUFFER_ADDR_MASK ((1UL << 32) - (1U <<2))
static int vgt_cmd_handler_mi_batch_buffer_start(struct vgt_cmd_data *data)
{
	if (data->buffer_type == RING_BUFFER_INSTRUCTION){
		data->ret_instruction = data->instruction + 2;
	}

	VGT_CMD_PRINTK("MI_BATCH_BUFFER_START: buffer GraphicsAddress=%x Clear Command Buffer Enable=%d\n",
			data->instruction[1], (data->instruction[0]>>11) & 1);

	/* FIXME: assume batch buffer also can be accessed by aperture */
	data->instruction = (uint32_t*)v_aperture(data->vgt->pdev,
				data->instruction[1] & BATCH_BUFFER_ADDR_MASK);
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
	int dword_length, i;

	dword_length = data->data & ( (1U << 8) - 1);

	for (i=1; i <= dword_length - 2 ; i = i+4){
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
	if (constant_buffer_address_offset_disable() == 1){
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
	if (constant_buffer_address_offset_disable() == 1){
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
	address_fixup(data,data->instruction + 6);
	/* Zero Bound is ignore */
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
	if (constant_buffer_address_offset_disable() == 1){
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

	return 0;
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

	VGT_CMD_PRINTK("cmd_name: %s\n",cmd_handlers[type].handlers[opcode].name);

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

			VGT_CMD_PRINTK("cmd name: %s\n",
					cmd_handlers[GEN_GFX_CMD_TYPE_GFXPIPE].handlers[index].name);

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
	printk(KERN_ERR "ring_id=%d buffer_type=%d instruction=%08x type=0x%x sub_type=0x%x opcode=0x%x sub_opcode=0x%x\n",
			d->ring_id, d->buffer_type, d->instruction[0], d->type, d->sub_type, d->opcode, d->sub_opcode);
}

bool gtt_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	uint32_t g_gtt_index;

	ASSERT(bytes == 4);

	if (off - VGT_MMIO_SPACE_SZ >= vgt->vgtt_sz)
		return false;

	g_gtt_index = GTT_OFFSET_TO_INDEX( off - VGT_MMIO_SPACE_SZ );
	*(uint32_t*)p_data = vgt->vgtt[g_gtt_index];
	return true;
}

#define GTT_INDEX_MB(x) ((SIZE_1MB*(x)) >> GTT_PAGE_SHIFT)

bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	uint32_t g_gtt_val, h_gtt_val, g_gtt_index, h_gtt_index;
	int rc;

	ASSERT(bytes == 4);

	if (off - VGT_MMIO_SPACE_SZ >= vgt->vgtt_sz)
		return false;

	g_gtt_val = *(uint32_t*)p_data;
	rc = gtt_p2m(vgt, g_gtt_val, &h_gtt_val);
	if (rc < 0){
		return false;
	}

	g_gtt_index = GTT_OFFSET_TO_INDEX( off - VGT_MMIO_SPACE_SZ );
	h_gtt_index = g2h_gtt_index(vgt, g_gtt_index);
	vgt_write_gtt( vgt->pdev, h_gtt_index, h_gtt_val );
#ifdef DOM0_DUAL_MAP
	if ( (h_gtt_index >= GTT_INDEX_MB(128)) && (h_gtt_index < GTT_INDEX_MB(192)) ){
		vgt_write_gtt( vgt->pdev, h_gtt_index - GTT_INDEX_MB(128), h_gtt_val );
	}
#endif
	vgt->vgtt[g_gtt_index] = g_gtt_val;

	return true;
}

static int error_count=0;

int vgt_scan_ring_buffer(struct vgt_device *vgt, int ring_id)
{
	vgt_ringbuffer_t	*sring;
	vgt_reg_t	rb_start, rb_head, rb_tail, ring_size;
	char* instr, *instr_end, *ring_buttom ;
	struct vgt_cmd_data decode_data;
	struct pgt_device  *pdev;
	int ret=0;

	if (error_count > 10)
		return 0;

	sring = &vgt->rb[ring_id].sring;
	pdev = vgt->pdev;

	rb_start = sring->start;
	rb_head = VGT_MMIO_READ(pdev, RB_HEAD(ring_id)) & RB_HEAD_OFF_MASK;
	rb_tail = sring->tail & RB_TAIL_OFF_MASK;
	ring_size = _RING_CTL_BUF_SIZE(sring->ctl);

	VGT_CMD_PRINTK("vgt_scan_ring_buffer: rb_start=%x rb_head=%x rb_tail=%x ring_size=%x\n",
			rb_start, rb_head, rb_tail, ring_size);

	instr = v_aperture(pdev, rb_start + rb_head);
	instr_end = v_aperture(pdev, rb_start + rb_tail);
	ring_buttom = v_aperture(pdev, rb_start + ring_size);

	decode_data.buffer_type = RING_BUFFER_INSTRUCTION;
	decode_data.vgt = vgt;
	decode_data.ring_id = ring_id;
	while(instr != instr_end){
		decode_data.instruction = (uint32_t*)instr;
		VGT_CMD_PRINTK("ring_id=%d %s instruction=%0x\n",
				decode_data.ring_id,
				decode_data.buffer_type == RING_BUFFER_INSTRUCTION ? "RING_BUFFER": "BATCH_BUFFER",
				decode_data.instruction[0]);

		ret = vgt_cmd_parser_render(&decode_data);
		if (ret < 0){
			error_count++;
			printk("error_count=%d\n", error_count);
			break;
		}

		/* next instruction*/
		instr = (char*)decode_data.instruction;

		if (decode_data.buffer_type == RING_BUFFER_INSTRUCTION){
			/* handle the ring buffer wrap case */
			ASSERT(instr <= ring_buttom);

			if (instr == ring_buttom){
				instr = v_aperture(pdev, rb_start);
				VGT_CMD_PRINTK("ring buffer wrap\n");
			}
		}
	}

	return ret;
}
