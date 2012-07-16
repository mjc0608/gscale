/*
 * vgt-parser.h: core header file for vGT command parser
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
#define GEN_GFX_CMD_TYPE_RENDER_MI 0
#define GEN_GFX_CMD_TYPE_RENDER_MISC 1
#define GEN_GFX_CMD_TYPE_RENDER_2D 2
#define GEN_GFX_CMD_TYPE_GFXPIPE 3
#define GEN_GFX_CMD_TYPE_MAX 3

#define GEN_GFX_CMD_TYPE_VIDEO_CODEC_MI 0
#define GEN_GFX_CMD_TYPE_VIDEO_CODEC_VC 3

struct vgt_cmd_data{
	struct vgt_device *vgt;
	enum {
		RING_BUFFER_INSTRUCTION,
		BATCH_BUFFER_INSTRUCTION
	}buffer_type;
	char* name;
	unsigned int *instruction;
	/* next instruction when return from  batch buffer to ring buffer */
	unsigned int *ret_instruction;
	unsigned int type;
	unsigned int sub_type;
	unsigned int opcode;
	unsigned int sub_opcode;
	unsigned int data;
	unsigned int count;
};

struct gen_gfx_cmd_common{
    unsigned int data:29;
    unsigned int type:3;
};

struct gen_gfx_cmd_mi{
    unsigned int data:23;
    unsigned int opcode:6;
    unsigned int type:3;
};

struct gen_gfx_cmd_misc{
    unsigned int data:16;
    unsigned int rsvd:3;
    unsigned int sub_opcode:5;
    unsigned int opcode:5;
    unsigned int type:3;
};

struct gen_gfx_cmd_2d{
    unsigned int data:22;
    unsigned int opcode:7;
    unsigned int type:3;
};

struct gen_gfx_cmd_3d_media{
    unsigned int count:8;
    unsigned int data:8;
    unsigned int sub_opcode:8;
    unsigned int opcode:3;
    unsigned int sub_type:2;
    unsigned int type:3;
};

union gen_gfx_render_cmd {
    unsigned int raw;
    struct gen_gfx_cmd_common common;
    struct gen_gfx_cmd_mi mi;
    struct gen_gfx_cmd_misc misc;
    struct gen_gfx_cmd_2d cmd_2d;
    struct gen_gfx_cmd_3d_media cmd_3d_media;
};

union gen_gfx_video_codec_cmd {
    unsigned int raw;
    struct gen_gfx_cmd_common common;
    struct gen_gfx_cmd_mi mi;
};

/* Bits 28:23 */
#define VGT_RENDER_MI_OPCODE_MAX 0x3F
/* Bits 28:22 */
#define VGT_RENDER_2D_OPCODE_MAX 0x7F
/* Bits 28:16 */
#define VGT_RENDER_3D_MEDIA_OPCODE_MAX ((1U<<13)-1)

struct vgt_cmd_handler{
	char* name;
	int (*handler)(struct vgt_cmd_data *data);
};

struct vgt_cmd_handlers{
	int len;
	struct vgt_cmd_handler* handlers;
};

struct vgt_addr_fix_entry{
	uint32_t* addr;
	uint32_t  data;
};

struct vgt_addr_fix_list{
	spinlock_t lock;
	int len;
	int pos;
	struct vgt_addr_fix_entry* entrys;
};


extern void vgt_parser_restore_address(void);

#define VGT_UNHANDLEABLE 1


/* Render Command Map */

/* MI_* command Opcode (28:23) */
#define OP_MI_NOOP					0x0
#define OP_MI_USER_INTERRUPT		0x2
#define OP_MI_WAIT_FOR_EVENT		0x3
#define OP_MI_FLUSH				0x4
#define OP_MI_ARB_CHECK			0x5
#define OP_MI_REPORT_HEAD			0x7
#define OP_MI_ARB_ON_OFF			0x8
#define OP_MI_BATCH_BUFFER_END		0xA
#define OP_MI_SUSPEND_FLUSH		0xB
#define OP_MI_DISPLAY_FLIP			0x14
#define OP_MI_SEMAPHORE_MBOX		0x16
#define OP_MI_SET_CONTEXT			0x18
#define OP_MI_MATH					0x1A

#define OP_MI_STORE_DATA_IMM		0x20
#define OP_MI_STORE_DATA_INDEX		0x21
#define OP_MI_LOAD_REGISTER_IMM	0x22
#define OP_MI_UPDATE_GTT			0x23
#define OP_MI_STORE_REGISTER_MEM	0x24
#define OP_MI_FLUSH_DW				0x26
#define OP_MI_CLFLUSH				0x27
#define OP_MI_REPORT_PERF_COUNT	0x28
#define OP_MI_BATCH_BUFFER_START	0x31
#define OP_MI_CONDITIONAL_BATCH_BUFFER_END	0x36

/* 2D command: Opcode (28:22) */
#define OP_XY_SETUP_BLT					0x1
#define OP_XY_SETUP_CLIP_BLT				0x3
#define OP_XY_SETUP_MONO_PATTERN_SL_BLT	0x11
#define OP_XY_PIXEL_BLT					0x24
#define OP_XY_SCANLINES_BLT				0x25
#define OP_XY_TEXT_BLT						0x26
#define OP_XY_TEXT_IMMEDIATE_BLT			0x31
#define OP_COLOR_BLT						0x40
#define OP_SRC_COPY_BLT					0x43
#define OP_XY_COLOR_BLT					0x50
#define OP_XY_PAT_BLT						0x51
#define OP_XY_MONO_PAT_BLT					0x52
#define OP_XY_SRC_COPY_BLT					0x53
#define OP_XY_MONO_SRC_COPY_BLT			0x54
#define OP_XY_FULL_BLT						0x55
#define OP_XY_FULL_MONO_SRC_BLT			0x56
#define OP_XY_FULL_MONO_PATTERN_BLT		0x57
#define OP_XY_FULL_MONO_PATTERN_MONO_SRC_BLT	0x58
#define OP_XY_MONO_PAT_FIXED_BLT			0x59
#define OP_XY_MONO_SRC_COPY_IMMEDIATE_BLT	0x71
#define OP_XY_PAT_BLT_IMMEDIATE			0x72
#define OP_XY_SRC_COPY_CHROMA_BLT			0x73
#define OP_XY_FULL_IMMEDIATE_PATTERN_BLT	0x74
#define OP_XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT	0x75
#define OP_XY_PAT_CHROMA_BLT				0x76
#define OP_XY_PAT_CHROMA_BLT_IMMEDIATE		0x77

/* 3D/Media Command: Pipeline Type(28:27) Opcode(26:24) Sub Opcode(23:16) */
/* macro can not begin with number, so add prefix _ */
#define INDEX_3D_MEDIA(sub_type, opcode, sub_opcode) ( ((sub_type)<<11) | ((opcode) <<8) | (sub_opcode))

#define OP_STATE_PREFETCH					INDEX_3D_MEDIA(0x0, 0x0, 0x03)
#define OP_STATE_BASE_ADDRESS				INDEX_3D_MEDIA(0x0, 0x1, 0x01)
#define OP_STATE_SIP						INDEX_3D_MEDIA(0x0, 0x1, 0x02)
#define OP_3DSTATE_VF_STATISTICS_GM45		INDEX_3D_MEDIA(0x1, 0x0, 0x0B)
#define OP_PIPELINE_SELECT					INDEX_3D_MEDIA(0x1, 0x1, 0x04)
#define OP_MEDIA_INTERFACE_DESCRIPTOR_LOAD	INDEX_3D_MEDIA( 0x2, 0x0, 0x2)
#define OP_MEDIA_GATEWAY_STATE				INDEX_3D_MEDIA( 0x2, 0x0, 0x3)
#define OP_MEDIA_STATE_FLUSH				INDEX_3D_MEDIA( 0x2, 0x0, 0x4)
#define OP_MEDIA_OBJECT					INDEX_3D_MEDIA( 0x2, 0x1, 0x0)
#define OP_MEDIA_CURBE_LOAD				INDEX_3D_MEDIA( 0x2, 0x1, 0x1)
#define OP_MEDIA_OBJECT_PRT				INDEX_3D_MEDIA( 0x2, 0x1, 0x2)
#define OP_MEDIA_OBJECT_WALKER				INDEX_3D_MEDIA( 0x2, 0x1, 0x3)
#define OP_MEDIA_VFE_STATE					INDEX_3D_MEDIA( 0x2, 0x2, 0x0)

// #define _3DSTATE_PIPELINED_POINTERS		INDEX_3D_MEDIA(0x3, 0x0, 0x00) /*Pre-DevSNB*/
#define OP_3DSTATE_BINDING_TABLE_POINTERS	INDEX_3D_MEDIA(0x3, 0x0, 0x01)
#define OP_3DSTATE_SAMPLER_STATE_POINTERS	INDEX_3D_MEDIA(0x3, 0x0, 0x02)
#define OP_3DSTATE_URB					INDEX_3D_MEDIA(0x3, 0x0, 0x05)
#define OP_3DSTATE_VERTEX_BUFFERS			INDEX_3D_MEDIA(0x3, 0x0, 0x08)
#define OP_3DSTATE_VERTEX_ELEMENTS		INDEX_3D_MEDIA(0x3, 0x0, 0x09)
#define OP_3DSTATE_INDEX_BUFFER			INDEX_3D_MEDIA(0x3, 0x0, 0x0A)
#define OP_3DSTATE_VF_STATISTICS			INDEX_3D_MEDIA(0x3, 0x0, 0x0B)
#define OP_3DSTATE_VIEWPORT_STATE_POINTERS	INDEX_3D_MEDIA(0x3, 0x0, 0x0D)
#define OP_3DSTATE_CC_STATE_POINTERS		INDEX_3D_MEDIA( 0x3 ,0x0, 0x0E )
#define OP_3DSTATE_SCISSOR_STATE_POINTERS	INDEX_3D_MEDIA( 0x3 ,0x0, 0x0F )
#define OP_3DSTATE_VS						INDEX_3D_MEDIA( 0x3 ,0x0, 0x10)
#define OP_3DSTATE_GS						INDEX_3D_MEDIA( 0x3 ,0x0, 0x11 )
#define OP_3DSTATE_CLIP					INDEX_3D_MEDIA( 0x3 ,0x0, 0x12 )
#define OP_3DSTATE_SF						INDEX_3D_MEDIA( 0x3 ,0x0, 0x13)
#define OP_3DSTATE_WM						INDEX_3D_MEDIA( 0x3 ,0x0, 0x14 )
#define OP_3DSTATE_CONSTANT_VS			INDEX_3D_MEDIA( 0x3 ,0x0, 0x15)
#define OP_3DSTATE_CONSTANT_GS			INDEX_3D_MEDIA( 0x3 ,0x0, 0x16 )
#define OP_3DSTATE_CONSTANT_PS			INDEX_3D_MEDIA( 0x3 ,0x0, 0x17 )
#define OP_3DSTATE_SAMPLE_MASK			INDEX_3D_MEDIA( 0x3 ,0x0, 0x18 )
#define OP_3DSTATE_DRAWING_RECTANGLE		INDEX_3D_MEDIA( 0x3 ,0x1, 0x00 )
#define OP_3DSTATE_SAMPLER_PALETTE_LOAD0	INDEX_3D_MEDIA( 0x3 ,0x1, 0x02 )
#define OP_3DSTATE_CHROMA_KEY				INDEX_3D_MEDIA( 0x3 ,0x1, 0x04 )
#define OP_3DSTATE_DEPTH_BUFFER			INDEX_3D_MEDIA( 0x3 ,0x1, 0x05 )
#define OP_3DSTATE_POLY_STIPPLE_OFFSET	INDEX_3D_MEDIA( 0x3 ,0x1, 0x06 )
#define OP_3DSTATE_POLY_STIPPLE_PATTERN	INDEX_3D_MEDIA( 0x3 ,0x1, 0x07 )
#define OP_3DSTATE_LINE_STIPPLE			INDEX_3D_MEDIA( 0x3 ,0x1, 0x08 )
#define OP_3DSTATE_AA_LINE_PARAMS			INDEX_3D_MEDIA( 0x3 ,0x1, 0x0A )
#define OP_3DSTATE_GS_SVB_INDEX			INDEX_3D_MEDIA( 0x3 ,0x1, 0x0B )
#define OP_3DSTATE_SAMPLER_PALETTE_LOAD1	INDEX_3D_MEDIA( 0x3 ,0x1, 0x0C )
#define OP_3DSTATE_MULTISAMPLE			INDEX_3D_MEDIA( 0x3 ,0x1, 0x0D )
#define OP_3DSTATE_STENCIL_BUFFER			INDEX_3D_MEDIA( 0x3 ,0x1, 0x0E )
#define OP_3DSTATE_HIER_DEPTH_BUFFER		INDEX_3D_MEDIA( 0x3 ,0x1, 0x0F )
#define OP_3DSTATE_CLEAR_PARAMS			INDEX_3D_MEDIA( 0x3 ,0x1, 0x10 )
#define OP_3DSTATE_MONOFILTER_SIZE		INDEX_3D_MEDIA( 0x3 ,0x1, 0x11 )
#define OP_3DSTATE_SO_DECL_LIST			INDEX_3D_MEDIA( 0x3 ,0x1, 0x17 )
#define OP_3DSTATE_SO_BUFFER				INDEX_3D_MEDIA( 0x3 ,0x1, 0x18 )
#define OP_PIPE_CONTROL					INDEX_3D_MEDIA( 0x3 ,0x2, 0x00 )
#define OP_3DPRIMITIVE					INDEX_3D_MEDIA( 0x3 ,0x3, 0x00 )

extern int vgt_cmd_parser_init(void);

extern int vgt_cmd_handler_register(unsigned int type, unsigned int opcode,
		char* name, int (*handler)(struct vgt_cmd_data *data));

extern int vgt_scan_vring(struct vgt_device *vgt, int ring_id);

extern bool gtt_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

extern bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val);

extern unsigned long g2m_pfn(int vm_id, unsigned long g_pfn);

#define INVALID_MFN  (~0UL)

extern void vgt_addr_fix_save(uint32_t* addr, uint32_t data);

extern void vgt_addr_fix_restore(void);
