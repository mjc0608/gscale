/*
 * vGT command parser
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/slab.h>
#include "vgt.h"
#include "reg.h"
#include <xen/vgt_cmd_parser.h>

#include "trace.h"
/* vgt uses bit 20 to detect buffer reuse
 *          bit 19 to mark MI_DISPLAY_FLIP command
 *          bit 0-2 to record some command information
 *
 * Assumption: Linux/Windows guest will not use these bits
 */
#define VGT_NOOP_ID_MASK	(1U << 20)
#define VGT_DISPLAY_FLIP_NOOP_ID_MASK	(1U << 19)
#define VGT_NOOP_ID_CMD_MASK	(0x7)

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

	hash_for_each_possible(vgt_cmd_table, e, hlist, opcode) {
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
	struct hlist_node *tmp;
	struct vgt_cmd_entry *e;

	hash_for_each_safe(vgt_cmd_table, i, tmp, e, hlist)
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

/* ring ALL, type = 0 */
static struct sub_op_bits sub_op_mi[]={
	{31, 29},
	{28, 23},
};

static struct decode_info decode_info_mi = {
	"MI",
	OP_LEN_MI,
	ARRAY_SIZE(sub_op_mi),
	sub_op_mi,
};


/* ring RCS, command type 2 */
static struct sub_op_bits sub_op_2d[]={
	{31, 29},
	{28, 22},
};

static struct decode_info decode_info_2d = {
	"2D",
	OP_LEN_2D,
	ARRAY_SIZE(sub_op_2d),
	sub_op_2d,
};

/* ring RCS, command type 3 */
static struct sub_op_bits sub_op_3d_media[]={
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 16},
};

static struct decode_info decode_info_3d_media = {
	"3D_Media",
	OP_LEN_3D_MEDIA,
	ARRAY_SIZE(sub_op_3d_media),
	sub_op_3d_media,
};

/* ring VCS, command type 3 */
static struct sub_op_bits sub_op_mfx_vc[]={
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 21},
	{20, 16},
};

static struct decode_info decode_info_mfx_vc = {
	"MFX_VC",
	OP_LEN_MFX_VC,
	ARRAY_SIZE(sub_op_mfx_vc),
	sub_op_mfx_vc,
};

/* ring VECS, command type 3 */
static struct sub_op_bits sub_op_vebox[] = {
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 21},
	{20, 16},
};

static struct decode_info decode_info_vebox = {
	"VEBOX",
	OP_LEN_VEBOX,
	ARRAY_SIZE(sub_op_vebox),
	sub_op_vebox,
};

static struct decode_info* ring_decode_info[MAX_ENGINES][8]=
{
	[RING_BUFFER_RCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_3d_media,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_VCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_mfx_vc,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_BCS] = {
		&decode_info_mi,
		NULL,
		&decode_info_2d,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_VECS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_vebox,
		NULL,
		NULL,
		NULL,
		NULL,
	},
};

uint32_t vgt_get_opcode(uint32_t cmd, int ring_id)
{
	struct decode_info * d_info;

	if (ring_id >= MAX_ENGINES)
		return INVALID_OP;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return INVALID_OP;

	return cmd >> (32 - d_info->op_len);
}

static inline uint32_t sub_op_val(uint32_t cmd, uint32_t hi, uint32_t low)
{
	return (cmd >> low) & ((1U << (hi-low+1)) - 1);
}

static void vgt_print_opcode(uint32_t cmd, int ring_id)
{
	struct decode_info * d_info;
	int i;

	if (ring_id >= MAX_ENGINES)
		return;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return;

	vgt_err("opcode=0x%x %s sub_ops:", cmd >> (32 - d_info->op_len), d_info->name);
	for (i=0; i< d_info->nr_sub_op; i++){
		vgt_err("0x%x ", sub_op_val(cmd, d_info->sub_op[i].hi,  d_info->sub_op[i].low));
	}
	vgt_err("\n");
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

static void parser_exec_state_dump(struct parser_exec_state *s)
{
	vgt_err("  vgt%d RING%d: ring_start(%08lx) ring_end(%08lx)"
			" ring_head(%08lx) ring_tail(%08lx)\n", s->vgt->vgt_id,
			s->ring_id, s->ring_start, s->ring_start + s->ring_size, s->ring_head, s->ring_tail);

	vgt_err("  %s %s ip_gma(%08lx) ",
			s->buf_type == RING_BUFFER_INSTRUCTION ? "RING_BUFFER": "BATCH_BUFFER",
			s->buf_addr_type == GTT_BUFFER ? "GTT" : "PPGTT", s->ip_gma);

	if (s->ip_va == NULL){
		vgt_err(" ip_va(NULL)\n");
	}else{
		vgt_err("  ip_va=%p: %08x %08x %08x %08x \n",
				s->ip_va, cmd_val(s,0), cmd_val(s,1),cmd_val(s,2), cmd_val(s,3));
		vgt_print_opcode(cmd_val(s,0), s->ring_id);
	}
}
#define RING_BUF_WRAP(s, ip_gma)	(((s)->buf_type == RING_BUFFER_INSTRUCTION) && \
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

	if (s->ip_va == NULL){
		vgt_err("ERROR: gma %lx is invalid, fail to set\n",s->ip_gma);
		dump_stack();
		parser_exec_state_dump(s);
		return -EFAULT;
	}

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

	if (s->ip_va_next_page == NULL){
		vgt_err("ERROR: next page gma %lx is invalid, fail to set\n",gma_next_page);
		dump_stack();
		parser_exec_state_dump(s);
		return -EFAULT;
	}

	return 0;
}

static inline int ip_gma_advance(struct parser_exec_state *s, unsigned int len)
{
	int rc = 0;
	if (s->ip_buf_len > len){
		/* not cross page, advance ip inside page */
		s->ip_gma += len*sizeof(uint32_t);
		s->ip_va += len;
		s->ip_buf_len -= len;
	} else{
		/* cross page, reset ip_va */
		rc = ip_gma_set(s, s->ip_gma + len*sizeof(uint32_t));
	}
	return rc;
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

static int vgt_cmd_handler_mi_set_context(struct parser_exec_state* s)
{
	struct vgt_device *vgt = s->vgt;

	if (!vgt->has_context) {
		printk("VM %d activate context\n", vgt->vm_id);
		vgt->has_context = true;
	}

	return 0;
}

static int vgt_cmd_advance_default(struct parser_exec_state *s)
{
	int rc;
	if (unlikely(s->ip_advance_update)) {
		rc = ip_gma_advance(s, s->ip_advance_update);
		s->ip_advance_update = 0;
		return rc;
	} else
		return ip_gma_advance(s, cmd_length(s));
}


static int vgt_cmd_handler_mi_batch_buffer_end(struct parser_exec_state *s)
{
	int rc;

	if (s->buf_type == BATCH_BUFFER_2ND_LEVEL){
		s->buf_type = BATCH_BUFFER_INSTRUCTION;
		rc = ip_gma_set(s, s->ret_ip_gma_bb);
		s->buf_addr_type = s->saved_buf_addr_type;
	}else{
		s->buf_type = RING_BUFFER_INSTRUCTION;
		s->buf_addr_type = GTT_BUFFER;
		rc = ip_gma_set(s, s->ret_ip_gma_ring);
	}

	return rc;
}

/* TODO
 *
 * The mi_display_flip handler below is just a workaround. The completed
 * handling is under discussion. The current approach is to NOOP the
 * MI_DISPLAY_FLIP command and then do pre-emulation. The pre-emulation
 * cannot exactly emulate command's behavior since it happens before
 * the command is issued. Consider the following two cases: one is that
 * right after the command-scan, display switch happens. Another case
 * is that commands inside ring buffer has some dependences.
 *
 * The interrupt is another consideration. mi_display_flip can trigger
 * interrupt for completion. VM's gfx driver may rely on that. Whether
 * we should inject virtual interrupt and when is the right time.
 *
 * The user space could resubmit ring/batch buffer with partially updated
 * MI_DISPLAY_FLIP. So special handling is needed in command parser for
 * such scenario.
 *
 * And we did not update HW state for the display flip.
 *
 */
#define PLANE_SELECT_SHIFT	19
#define PLANE_SELECT_MASK	(0x7 << PLANE_SELECT_SHIFT)
#define SURF_MASK		0xFFFFF000
#define PITCH_MASK		0x0000FFC0
#define TILE_PARA_SHIFT		0x0
#define TILE_PARA_MASK		0x1
/* Primary plane and sprite plane has the same tile shift in control reg */
#define PLANE_TILE_SHIFT	_PRI_PLANE_TILE_SHIFT
#define PLANE_TILE_MASK		(0x1 << PLANE_TILE_SHIFT)
#define FLIP_TYPE_MASK		0x3

static int vgt_cmd_handler_mi_display_flip(struct parser_exec_state *s)
{
	uint32_t surf_reg, surf_val, ctrl_reg;
	uint32_t stride_reg, stride_val, stride_mask;
	uint32_t tile_para, tile_in_ctrl;
	uint32_t opcode, plane_code;
	enum vgt_pipe pipe;
	enum vgt_plane_type plane;
	bool async_flip;
	int i, length = cmd_length(s);

	opcode = *(cmd_ptr(s, 0));
	stride_val = *(cmd_ptr(s, 1));
	surf_val = *(cmd_ptr(s, 2));

	plane_code = (opcode & PLANE_SELECT_MASK) >> PLANE_SELECT_SHIFT;

	switch (plane_code) {
		case 0:
			pipe = PIPE_A; plane = PRIMARY_PLANE; break;
		case 1:
			pipe = PIPE_B; plane = PRIMARY_PLANE; break;
		case 2:
			pipe = PIPE_A; plane = SPRITE_PLANE; break;
		case 3:
			pipe = PIPE_B; plane = SPRITE_PLANE; break;
		case 4:
			pipe = PIPE_C; plane = PRIMARY_PLANE; break;
		case 5:
			pipe = PIPE_C; plane = SPRITE_PLANE; break;
		default:
			goto wrong_command;
	}

	if (plane == PRIMARY_PLANE) {
		ctrl_reg = VGT_DSPCNTR(pipe);
		surf_reg = VGT_DSPSURF(pipe);
		stride_reg = VGT_DSPSTRIDE(pipe);
		stride_mask = _PRI_PLANE_STRIDE_MASK;
	} else {
		ASSERT (plane == SPRITE_PLANE);
		ctrl_reg = VGT_SPRCTL(pipe);
		surf_reg = VGT_SPRSURF(pipe);
		stride_reg = VGT_SPRSTRIDE(pipe);
		stride_mask = _SPRITE_STRIDE_MASK;
	}

	if (length == 4) {
		vgt_warn("Page flip of Stereo 3D is not supported!\n");
		goto wrong_command;
	} else if (length != 3) {
		vgt_warn("Flip length not equal to 3, ignore handling flipping");
		goto wrong_command;
	}

	async_flip = ((surf_val & FLIP_TYPE_MASK) == 0x1);
	tile_para = ((stride_val & TILE_PARA_MASK) >> TILE_PARA_SHIFT);
	tile_in_ctrl = (__vreg(s->vgt, ctrl_reg) & PLANE_TILE_MASK)
				>> PLANE_TILE_SHIFT;

	if ((__vreg(s->vgt, stride_reg) & stride_mask)
		!= (stride_val & PITCH_MASK)) {

		if (async_flip) {
			vgt_warn("Cannot change stride in async flip!\n");
			goto wrong_command;
		}
		__vreg(s->vgt, stride_reg) = (__vreg(s->vgt, stride_reg) &
							(~stride_mask)) |
					(stride_val & stride_mask);
		__sreg(s->vgt, stride_reg) = __vreg(s->vgt, stride_reg);
	}

	if (tile_para != tile_in_ctrl) {

		if (async_flip) {
			vgt_warn("Cannot change tiling in async flip!\n");
			goto wrong_command;
		}
		__vreg(s->vgt, ctrl_reg) =
			(__vreg(s->vgt, ctrl_reg) & (~PLANE_TILE_MASK)) |
					(tile_para << PLANE_TILE_SHIFT);
		__sreg(s->vgt, ctrl_reg) = __vreg(s->vgt, ctrl_reg);
	}

	__vreg(s->vgt, surf_reg) = (__vreg(s->vgt, surf_reg) & (~SURF_MASK)) |
					(surf_val & SURF_MASK);
	__sreg(s->vgt, surf_reg) = __vreg(s->vgt, surf_reg);

	if (s->vgt == current_foreground_vm(s->vgt->pdev)) {
		return 0;
	}

command_noop:
	vgt_dbg("VM %d: mi_display_flip to be ignored\n",
		s->vgt->vm_id);

	for (i = 0; i < length; i ++) {
		*(cmd_ptr(s, i)) = MI_NOOP | VGT_NOOP_ID_MASK |
					VGT_DISPLAY_FLIP_NOOP_ID_MASK;
	}

	*(cmd_ptr(s,0)) |= plane_code;

	s->ip_advance_update = length;

	return 0;

wrong_command:
	vgt_warn("VM %d: wrong mi_display_flip command!\n", s->vgt->vm_id);
	goto command_noop;
}

static int vgt_handle_resubmitted_flip_cmd(struct parser_exec_state* s)
{
	uint32_t plane_code, surf_reg;
	vgt_reg_t surf_val;
	enum vgt_pipe pipe;
	enum vgt_plane_type plane;

	plane_code = cmd_val(s, 0) & VGT_NOOP_ID_CMD_MASK;
	surf_val = cmd_val(s, 2);

	if (surf_val == (MI_NOOP | VGT_NOOP_ID_MASK |
				VGT_DISPLAY_FLIP_NOOP_ID_MASK)) {
		return 0;
	} else {
		vgt_dbg("cmd buffer contains partially updated command!\n");
		*(cmd_ptr(s, 2)) = MI_NOOP | VGT_NOOP_ID_MASK |
					VGT_DISPLAY_FLIP_NOOP_ID_MASK;
	}

	switch (plane_code) {
		case 0:
			pipe = PIPE_A; plane = PRIMARY_PLANE; break;
		case 1:
			pipe = PIPE_B; plane = PRIMARY_PLANE; break;
		case 2:
			pipe = PIPE_A; plane = SPRITE_PLANE; break;
		case 3:
			pipe = PIPE_B; plane = SPRITE_PLANE; break;
		case 4:
			pipe = PIPE_C; plane = PRIMARY_PLANE; break;
		case 5:
			pipe = PIPE_C; plane = SPRITE_PLANE; break;
		default:
			ASSERT(0);
	}

	if (plane == PRIMARY_PLANE) {
		surf_reg = VGT_DSPSURF(pipe);
	} else {
		ASSERT (plane == SPRITE_PLANE);
		surf_reg = VGT_SPRSURF(pipe);
	}

	__vreg(s->vgt, surf_reg) = (__vreg(s->vgt, surf_reg) & (~SURF_MASK)) |
					(surf_val & SURF_MASK);
	__sreg(s->vgt, surf_reg) = __vreg(s->vgt, surf_reg);

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

	entry_num = cmd_length(s) - 2; /* GTT items begin from the 3rd dword */
	//entry = v_aperture(s->vgt->pdev, cmd_val(s,1));
	entry = cmd_ptr(s, 2);
	for (i=0; i<entry_num; i++){
		vgt_dbg("vgt: update GTT entry %d\n", i);
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
	int rc=0;
	bool second_level;

	if (s->buf_type == BATCH_BUFFER_2ND_LEVEL){
		vgt_err("MI_BATCH_BUFFER_START not allowd in 2nd level batch buffer\n");
		return -EINVAL;
	}

	second_level = BATCH_BUFFER_2ND_LEVEL_BIT(cmd_val(s,0)) == 1;
	if (second_level && (s->buf_type != BATCH_BUFFER_INSTRUCTION)){
		vgt_err("Jumping to 2nd level batch buffer from ring buffer is not allowd\n");
		return -EINVAL;
	}

	s->saved_buf_addr_type = s->buf_addr_type;

	/* FIXME: add IVB/HSW code */
	addr_type_update_snb(s);

	if (s->buf_type == RING_BUFFER_INSTRUCTION){
		s->ret_ip_gma_ring = s->ip_gma + 2*sizeof(uint32_t);
		s->buf_type = BATCH_BUFFER_INSTRUCTION;
	} else if (second_level){
		s->buf_type = BATCH_BUFFER_2ND_LEVEL;
		s->ret_ip_gma_bb = s->ip_gma + 2*sizeof(uint32_t);
	 }

	klog_printk("MI_BATCH_BUFFER_START: Addr=%x ClearCommandBufferEnable=%d\n",
			cmd_val(s,1),  (cmd_val(s,0)>>11) & 1);

	address_fixup(s, 1);

	rc = ip_gma_set(s, cmd_val(s,1) & BATCH_BUFFER_ADDR_MASK);

	if (rc < 0){
		vgt_warn("invalid batch buffer addr, so skip scanning it\n");
	}

	return rc;
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

static inline int base_and_upper_addr_fix(struct parser_exec_state *s)
{
	address_fixup(s,1);
	/* Zero Bound is ignore */
	if (cmd_val(s,2) >> 12)
		address_fixup(s,2);
	return 0;
}

static int vgt_cmd_handler_3dstate_binding_table_pool_alloc(struct parser_exec_state *s)
{
	return base_and_upper_addr_fix(s);
}

static int vgt_cmd_handler_3dstate_gather_pool_alloc(struct parser_exec_state *s)
{
	return base_and_upper_addr_fix(s);
}

static int vgt_cmd_handler_3dstate_dx9_constant_buffer_pool_alloc(struct parser_exec_state *s)
{
	return base_and_upper_addr_fix(s);
}

static int vgt_cmd_handler_op_3dstate_constant_hs(struct parser_exec_state *s)
{
	address_fixup(s, 3); /* TODO: check INSTPM<CONSTANT_BUFFER Address Offset Disable */
	address_fixup(s, 4);
	address_fixup(s, 5);
	address_fixup(s, 6);
	return 0;
}

static int vgt_cmd_handler_op_3dstate_constant_ds(struct parser_exec_state *s)
{
	address_fixup(s, 3); /* TODO: check INSTPM<CONSTANT_BUFFER Address Offset Disable */
	address_fixup(s, 4);
	address_fixup(s, 5);
	address_fixup(s, 6);
	return 0;
}

static int vgt_cmd_handler_mfx_pipe_buf_addr_state(struct parser_exec_state *s)
{
	int i;
	for (i=1; i<=23; i++){
		address_fixup(s, i);
	}
	return 0;
}

static int vgt_cmd_handler_mfx_ind_obj_base_addr_state(struct parser_exec_state *s)
{
	int i;
	for (i=1; i<=10; i++){
		address_fixup(s, i);
	}
	return 0;
}

static int vgt_cmd_handler_mfx_2_6_0_0(struct parser_exec_state *s)
{
	base_and_upper_addr_fix(s);
	address_fixup(s,2);
	return 0;
}

static int vgt_cmd_handler_mi_noop(struct parser_exec_state* s)
{
	if (cmd_val(s,0) & VGT_NOOP_ID_MASK) {
		if (cmd_val(s,0) & VGT_DISPLAY_FLIP_NOOP_ID_MASK) {
			vgt_dbg("VM %d: Guest reuse cmd buffer!\n", s->vgt->vm_id);
			vgt_handle_resubmitted_flip_cmd(s);
		} else {
			vgt_err("VM %d: Guest reuse cmd buffer!\n", s->vgt->vm_id);
			parser_exec_state_dump(s);
		}
	}

	return 0;
}

#if 0
	{"", OP_, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"", OP_, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"", OP_, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"", OP_, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

#endif
static struct cmd_info cmd_info[] = {
	{"MI_NOOP", OP_MI_NOOP, F_LEN_CONST, R_ALL, D_ALL, 0, 1, vgt_cmd_handler_mi_noop},

	{"MI_SET_PREDICATE", OP_MI_SET_PREDICATE, F_LEN_CONST, R_ALL, D_HSW_PLUS,
		0, 1, NULL},

	{"MI_USER_INTERRUPT", OP_MI_USER_INTERRUPT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_WAIT_FOR_EVENT", OP_MI_WAIT_FOR_EVENT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_FLUSH", OP_MI_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_CHECK", OP_MI_ARB_CHECK, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_RS_CONTROL", OP_MI_RS_CONTROL, F_LEN_CONST, R_RCS, D_HSW_PLUS, 0, 1, NULL},

	{"MI_REPORT_HEAD", OP_MI_REPORT_HEAD, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_ON_OFF", OP_MI_ARB_ON_OFF, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_URB_ATOMIC_ALLOC", OP_MI_URB_ATOMIC_ALLOC, F_LEN_CONST, R_RCS,
		D_HSW_PLUS, 0, 1, NULL},

	{"MI_BATCH_BUFFER_END", OP_MI_BATCH_BUFFER_END, F_IP_ADVANCE_CUSTOM|F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, vgt_cmd_handler_mi_batch_buffer_end},

	{"MI_SUSPEND_FLUSH", OP_MI_SUSPEND_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_PREDICATE", OP_MI_PREDICATE, F_LEN_CONST, R_RCS, D_IVB_PLUS, 0, 1, NULL},

	{"MI_TOPOLOGY_FILTER", OP_MI_TOPOLOGY_FILTER, F_LEN_CONST, R_ALL,
		D_IVB_PLUS, 0, 1, NULL},

	{"MI_SET_APPID", OP_MI_SET_APPID, F_LEN_CONST, R_ALL, D_IVB_PLUS, 0, 1, NULL},

	{"MI_RS_CONTEXT", OP_MI_RS_CONTEXT, F_LEN_CONST, R_RCS, D_HSW_PLUS, 0, 1, NULL},

	{"MI_DISPLAY_FLIP", OP_MI_DISPLAY_FLIP, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, vgt_cmd_handler_mi_display_flip},

	{"MI_SEMAPHORE_MBOX", OP_MI_SEMAPHORE_MBOX, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL },

	{"MI_SET_CONTEXT", OP_MI_SET_CONTEXT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 8, vgt_cmd_handler_mi_set_context},

	{"MI_MATH", OP_MI_MATH, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_URB_CLEAR", OP_MI_URB_CLEAR, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"MI_STORE_DATA_IMM", OP_MI_STORE_DATA_IMM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_DATA_INDEX", OP_MI_STORE_DATA_INDEX, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, NULL},

	{"MI_LOAD_REGISTER_IMM", OP_MI_LOAD_REGISTER_IMM, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, vgt_cmd_handler_mi_update_gtt},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, (R_VCS | R_BCS | R_VECS), D_ALL,
		0, 6, vgt_cmd_handler_mi_update_gtt},

	{"MI_STORE_REGISTER_MEM", OP_MI_STORE_REGISTER_MEM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_FLUSH_DW", OP_MI_FLUSH_DW, F_LEN_VAR, R_ALL, D_ALL,
		0, 6, vgt_cmd_handler_mi_flush_dw},

	{"MI_CLFLUSH", OP_MI_CLFLUSH, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"MI_REPORT_PERF_COUNT", OP_MI_REPORT_PERF_COUNT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 6, NULL},

	{"MI_LOAD_REGISTER_MEM", OP_MI_LOAD_REGISTER_MEM, F_LEN_VAR, R_ALL, D_GEN7PLUS,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_LOAD_REGISTER_REG", OP_MI_LOAD_REGISTER_REG, F_LEN_VAR, R_ALL, D_HSW_PLUS,
		0, 8, NULL},

	{"MI_RS_STORE_DATA_IMM", OP_MI_RS_STORE_DATA_IMM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		0, 8, NULL},

	{"MI_LOAD_URB_MEM", OP_MI_LOAD_URB_MEM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_URM_MEM", OP_MI_STORE_URM_MEM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(2), 8, NULL},

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

	{"3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP", OP_3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS_CC", OP_3DSTATE_VIEWPORT_STATE_POINTERS_CC,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BLEND_STATE_POINTERS", OP_3DSTATE_BLEND_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DEPTH_STENCIL_STATE_POINTERS", OP_3DSTATE_DEPTH_STENCIL_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

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

	{"3DSTATE_SAMPLER_STATE_POINTERS_VS", OP_3DSTATE_SAMPLER_STATE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_HS", OP_3DSTATE_SAMPLER_STATE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_DS", OP_3DSTATE_SAMPLER_STATE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_GS", OP_3DSTATE_SAMPLER_STATE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_PS", OP_3DSTATE_SAMPLER_STATE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_URB_VS", OP_3DSTATE_URB_VS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_HS", OP_3DSTATE_URB_HS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_DS", OP_3DSTATE_URB_DS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_GS", OP_3DSTATE_URB_GS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_VS", OP_3DSTATE_GATHER_CONSTANT_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_GS", OP_3DSTATE_GATHER_CONSTANT_GS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_HS", OP_3DSTATE_GATHER_CONSTANT_HS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_DS", OP_3DSTATE_GATHER_CONSTANT_DS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_PS", OP_3DSTATE_GATHER_CONSTANT_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTF_VS", OP_3DSTATE_DX9_CONSTANTF_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTF_PS", OP_3DSTATE_DX9_CONSTANTF_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTI_VS", OP_3DSTATE_DX9_CONSTANTI_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTI_PS", OP_3DSTATE_DX9_CONSTANTI_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_VS", OP_3DSTATE_DX9_CONSTANTB_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_PS", OP_3DSTATE_DX9_CONSTANTB_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_VS", OP_3DSTATE_DX9_LOCAL_VALID_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_PS", OP_3DSTATE_DX9_LOCAL_VALID_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_VS", OP_3DSTATE_DX9_GENERATE_ACTIVE_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_PS", OP_3DSTATE_DX9_GENERATE_ACTIVE_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_VS", OP_3DSTATE_BINDING_TABLE_EDIT_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_GS", OP_3DSTATE_BINDING_TABLE_EDIT_GS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_HS", OP_3DSTATE_BINDING_TABLE_EDIT_HS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_DS", OP_3DSTATE_BINDING_TABLE_EDIT_DS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_PS", OP_3DSTATE_BINDING_TABLE_EDIT_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS", OP_3DSTATE_SAMPLER_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_URB", OP_3DSTATE_URB, F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_VERTEX_BUFFERS", OP_3DSTATE_VERTEX_BUFFERS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_vertex_buffers},

	{"3DSTATE_VERTEX_ELEMENTS", OP_3DSTATE_VERTEX_ELEMENTS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_INDEX_BUFFER", OP_3DSTATE_INDEX_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, vgt_cmd_handler_3dstate_index_buffer},

	{"3DSTATE_VF_STATISTICS", OP_3DSTATE_VF_STATISTICS, F_LEN_CONST,
		R_RCS, D_ALL, 0, 1, NULL},

	{"3DSTATE_VF", OP_3DSTATE_VF, F_LEN_VAR, R_RCS, D_GEN75PLUS, 0, 8, NULL},

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

	{"3DSTATE_CONSTANT_HS", OP_3DSTATE_CONSTANT_HS, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, 0, 8, vgt_cmd_handler_op_3dstate_constant_hs},

	{"3DSTATE_CONSTANT_DS", OP_3DSTATE_CONSTANT_DS, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, 0, 8, vgt_cmd_handler_op_3dstate_constant_ds},

	{"3DSTATE_HS", OP_3DSTATE_HS, F_LEN_VAR, R_RCS,	D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_TE", OP_3DSTATE_TE, F_LEN_VAR, R_RCS,	D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DS", OP_3DSTATE_DS, F_LEN_VAR, R_RCS,	D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_STREAMOUT", OP_3DSTATE_STREAMOUT, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SBE", OP_3DSTATE_SBE, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PS", OP_3DSTATE_PS, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DRAWING_RECTANGLE", OP_3DSTATE_DRAWING_RECTANGLE, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD0", OP_3DSTATE_SAMPLER_PALETTE_LOAD0,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CHROMA_KEY", OP_3DSTATE_CHROMA_KEY, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_DEPTH_BUFFER", OP_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_SNB, ADDR_FIX_1(2), 8, NULL},

	{"GEN7_3DSTATE_DEPTH_BUFFER", OP_GEN7_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

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
		D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"GEN7_3DSTATE_STENCIL_BUFFER", OP_GEN7_3DSTATE_STENCIL_BUFFER, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_HIER_DEPTH_BUFFER", OP_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_SNB, ADDR_FIX_1(2), 8, NULL},

	{"GEN7_3DSTATE_HIER_DEPTH_BUFFER", OP_GEN7_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_CLEAR_PARAMS", OP_3DSTATE_CLEAR_PARAMS, F_LEN_VAR, R_RCS, D_SNB,
		0, 8, NULL},

	{"GEN7_3DSTATE_CLEAR_PARAMS", OP_GEN7_3DSTATE_CLEAR_PARAMS, F_LEN_VAR,
		R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_VS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_HS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_DS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_GS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_PS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_MONOFILTER_SIZE", OP_3DSTATE_MONOFILTER_SIZE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_SO_DECL_LIST", OP_3DSTATE_SO_DECL_LIST, F_LEN_VAR, R_RCS, D_ALL,
		0, 9, NULL},

	{"3DSTATE_SO_BUFFER", OP_3DSTATE_SO_BUFFER, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_2(2,3), 8, NULL},

	{"3DSTATE_BINDING_TABLE_POOL_ALLOC", OP_3DSTATE_BINDING_TABLE_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_GEN75PLUS, 0, 8, vgt_cmd_handler_3dstate_binding_table_pool_alloc},

	{"3DSTATE_GATHER_POOL_ALLOC", OP_3DSTATE_GATHER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_GEN75PLUS, 0, 8, vgt_cmd_handler_3dstate_gather_pool_alloc},

	{"3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC", OP_3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_GEN75PLUS, 0, 8, vgt_cmd_handler_3dstate_dx9_constant_buffer_pool_alloc},

	{"PIPE_CONTROL", OP_PIPE_CONTROL, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"3DPRIMITIVE", OP_3DPRIMITIVE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"PIPELINE_SELECT", OP_PIPELINE_SELECT, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"STATE_PREFETCH", OP_STATE_PREFETCH, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"STATE_SIP", OP_STATE_SIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"STATE_BASE_ADDRESS", OP_STATE_BASE_ADDRESS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, vgt_cmd_handler_state_base_address},

	{"OP_3D_MEDIA_0_1_4", OP_3D_MEDIA_0_1_4, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(1), 8, NULL},

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

	{"OP_3D_MEDIA_2_1_5", OP_3D_MEDIA_2_1_5, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"MEDIA_VFE_STATE", OP_MEDIA_VFE_STATE, F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"3DSTATE_VF_STATISTICS_GM45", OP_3DSTATE_VF_STATISTICS_GM45, F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, NULL},

	{"MFX_PIPE_MODE_SELECT", OP_MFX_PIPE_MODE_SELECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_SURFACE_STATE", OP_MFX_SURFACE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_PIPE_BUF_ADDR_STATE", OP_MFX_PIPE_BUF_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, vgt_cmd_handler_mfx_pipe_buf_addr_state},

	{"MFX_IND_OBJ_BASE_ADDR_STATE", OP_MFX_IND_OBJ_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, vgt_cmd_handler_mfx_ind_obj_base_addr_state},

	{"MFX_BSP_BUF_BASE_ADDR_STATE", OP_MFX_BSP_BUF_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_ALL, ADDR_FIX_3(1,2,3), 12, NULL},

	{"OP_2_0_0_5", OP_2_0_0_5, F_LEN_VAR,
		R_VCS, D_ALL, ADDR_FIX_1(6), 12, NULL},

	{"MFX_STATE_POINTER", OP_MFX_STATE_POINTER, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_QM_STATE", OP_MFX_QM_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_FQM_STATE", OP_MFX_FQM_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_PAK_INSERT_OBJECT", OP_MFX_PAK_INSERT_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_STITCH_OBJECT", OP_MFX_STITCH_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_IT_OBJECT", OP_MFD_IT_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_WAIT", OP_MFX_WAIT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 6, NULL},

	{"MFX_AVC_IMG_STATE", OP_MFX_AVC_IMG_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_QM_STATE", OP_MFX_AVC_QM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	/* to check: is "Direct MV Buffer Base Address" GMA ? */
	{"MFX_AVC_DIRECTMODE_STATE", OP_MFX_AVC_DIRECTMODE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_SLICE_STATE", OP_MFX_AVC_SLICE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_REF_IDX_STATE", OP_MFX_AVC_REF_IDX_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_WEIGHTOFFSET_STATE", OP_MFX_AVC_WEIGHTOFFSET_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_AVC_PICID_STATE", OP_MFD_AVC_PICID_STATE, F_LEN_VAR,
		R_VCS, D_GEN75PLUS, 0, 12, NULL},
	{"MFD_AVC_DPB_STATE", OP_MFD_AVC_DPB_STATE, F_LEN_VAR,
		R_VCS, D_IVB_PLUS, 0, 12, NULL},

	{"MFD_AVC_BSD_OBJECT", OP_MFD_AVC_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFC_AVC_FQM_STATE", OP_MFC_AVC_FQM_STATE, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFC_AVC_PAK_INSERT_OBJECT", OP_MFC_AVC_PAK_INSERT_OBJECT, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFC_AVC_PAK_OBJECT", OP_MFC_AVC_PAK_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_PIC_STATE", OP_MFX_VC1_PIC_STATE, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFX_VC1_PRED_PIPE_STATE", OP_MFX_VC1_PRED_PIPE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_DIRECTMODE_STATE", OP_MFX_VC1_DIRECTMODE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_VC1_SHORT_PIC_STATE", OP_MFD_VC1_SHORT_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_VC1_LONG_PIC_STATE", OP_MFD_VC1_LONG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_VC1_BSD_OBJECT", OP_MFD_VC1_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_MPEG2_PIC_STATE", OP_MFX_MPEG2_PIC_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_MPEG2_QM_STATE", OP_MFX_MPEG2_QM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_MPEG2_BSD_OBJECT", OP_MFD_MPEG2_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_2_6_0_0", OP_MFX_2_6_0_0, F_LEN_VAR, R_VCS, D_ALL,
		0, 16, vgt_cmd_handler_mfx_2_6_0_0},

	{"MFX_2_6_0_9", OP_MFX_2_6_0_9, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_2_6_0_8", OP_MFX_2_6_0_8, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_JPEG_PIC_STATE", OP_MFX_JPEG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_JPEG_HUFF_TABLE_STATE", OP_MFX_JPEG_HUFF_TABLE_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_JPEG_BSD_OBJECT", OP_MFD_JPEG_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"VEBOX_STATE", OP_VEB_STATE, F_LEN_VAR, R_VECS, D_HSW, 0, 12, NULL},

	{"VEBOX_SURFACE_STATE", OP_VEB_SURFACE_STATE, F_LEN_VAR, R_VECS, D_HSW_PLUS, 0, 12, NULL},

	{"VEB_DI_IECP", OP_VEB_DNDI_IECP_STATE, F_LEN_VAR, R_VECS, D_HSW, 0, 12, NULL},
};

static int cmd_hash_init(struct pgt_device *pdev)
{
	int i;
	struct vgt_cmd_entry *e;
	struct cmd_info	*info;
	unsigned int gen_type;

	gen_type = vgt_gen_dev_type(pdev);

	for (i=0; i< ARRAY_SIZE(cmd_info); i++){
		if (!(cmd_info[i].devices & gen_type)){
			vgt_dbg("CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] not registered\n",
					cmd_info[i].name, cmd_info[i].opcode, cmd_info[i].flag,
					cmd_info[i].devices, cmd_info[i].rings);
			continue;
		}

		e = kmalloc(sizeof(*e), GFP_KERNEL);
		if (e == NULL) {
			vgt_err("Insufficient memory in %s\n", __FUNCTION__);
			return -ENOMEM;
		}
		e->info = &cmd_info[i];

		info = vgt_find_cmd_entry_any_ring(e->info->opcode, e->info->rings);
		if (info){
			vgt_err("%s %s duplicated\n", e->info->name, info->name);
			return -EINVAL;
		}

		INIT_HLIST_NODE(&e->hlist);
		vgt_add_cmd_entry(e);
		vgt_dbg("CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] registered\n",
				e->info->name,e->info->opcode, e->info->flag, e->info->devices,
				e->info->rings);
	}
	return 0;
}

/* This buffer is used by ftrace to store all commands copied from guest gma
 * space. Sometimes commands can cross pages, this should not be handled in
 * ftrace logic. So this is just used as a 'bounce buffer' */
static u32 cmd_trace_buf[VGT_MAX_CMD_LENGTH];

/* call the cmd handler, and advance ip */
static int vgt_cmd_parser_exec(struct parser_exec_state *s)
{
	struct cmd_info *info;

	int rc = 0, i, cmd_len;

	info = vgt_get_cmd_info(*s->ip_va, s->ring_id);
	if(info == NULL){
		vgt_err("ERROR: unknown cmd 0x%x, opcode=0x%x\n", *s->ip_va,
				vgt_get_opcode(*s->ip_va, s->ring_id));
		parser_exec_state_dump(s);
		klog_printk("ERROR: unknown cmd %x, ring%d[%lx,%lx] gma[%lx] va[%p]\n",
				*s->ip_va, s->ring_id, s->ring_start,
				s->ring_start + s->ring_size, s->ip_gma, s->ip_va);

		return -EFAULT;
	}

	s->info = info;

#ifdef VGT_ENABLE_ADDRESS_FIX
	{
		unsigned int bit;
		for_each_set_bit(bit, (unsigned long*)&info->addr_bitmap, 8*sizeof(info->addr_bitmap))
			address_fixup(s, bit);
	}
#endif

	/* Let's keep this logic here. Someone has special needs for dumping
	 * commands can customize this code snippet.
	 */
#if 0
	klog_printk("%s ip(%08lx): ",
			s->buf_type == RING_BUFFER_INSTRUCTION ?
			"RB" : "BB",
			s->ip_gma);
	cmd_len = cmd_length(s);
	for (i = 0; i < cmd_len; i++) {
		klog_printk("%08x ", cmd_val(s, i));
	}
	klog_printk("\n");
#endif

	cmd_len = cmd_length(s);
	/* The chosen value of VGT_MAX_CMD_LENGTH are just based on
	 * following two considerations:
	 * 1) From observation, most common ring commands is not that long.
	 *    But there are execeptions. So it indeed makes sence to observe
	 *    longer commands.
	 * 2) From the performance and debugging point of view, dumping all
	 *    contents of very commands is not necessary.
	 * We mgith shrink VGT_MAX_CMD_LENGTH or remove this trace event in
	 * future for performance considerations.
	 */
	if (unlikely(cmd_len > VGT_MAX_CMD_LENGTH)) {
		vgt_dbg("cmd length exceed tracing limitation!\n");
		cmd_len = VGT_MAX_CMD_LENGTH;
	}
	for (i = 0; i < cmd_len; i++)
		cmd_trace_buf[i] = cmd_val(s, i);
	trace_vgt_command(s->vgt->vm_id, s->ip_gma, cmd_trace_buf, cmd_len,
			s->buf_type == RING_BUFFER_INSTRUCTION);

	if (info->handler){
		rc = info->handler(s);
		if (rc < 0){
			vgt_err("%s: %s handler error", __func__, info->name);
			return rc;
		}
	}

	if (!(info->flag & F_IP_ADVANCE_CUSTOM)){
		rc = vgt_cmd_advance_default(s);
		if (rc < 0){
			vgt_err("%s: %s IP advance error", __func__, info->name);
			return rc;
		}
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

static inline bool gma_out_of_range(unsigned long gma, unsigned long gma_head, unsigned gma_tail)
{
	if ( gma_tail >= gma_head)
		return	(gma < gma_head) || (gma > gma_tail);
	else
		return (gma > gma_tail) && (gma < gma_head);

}

#define MAX_PARSER_ERROR_NUM	10

static int __vgt_scan_vring(struct vgt_device *vgt, int ring_id, vgt_reg_t head, vgt_reg_t tail, vgt_reg_t base, vgt_reg_t size)
{
	static int error_count=0;
	unsigned long gma_head, gma_tail, gma_bottom;
	struct parser_exec_state s;
	int rc=0;

#if 0
	if (error_count >= MAX_PARSER_ERROR_NUM)
		return 0;
#endif

	/* ring base is page aligned */
	ASSERT((base & (PAGE_SIZE-1)) == 0);

	gma_head = base + head;
	gma_tail = base + tail;
	gma_bottom = base + size;

	s.buf_type = RING_BUFFER_INSTRUCTION;
	s.buf_addr_type = GTT_BUFFER;
	s.vgt = vgt;
	s.ring_id = ring_id;
	s.ring_start = base;
	s.ring_size = size;
	s.ring_head = gma_head;
	s.ring_tail = gma_tail;
	s.ip_advance_update = 0;

	rc = ip_gma_set(&s, base + head);
	if (rc < 0){
		error_count++;
		return rc;
	}

	klog_printk("ring buffer scan start on ring %d\n", ring_id);
	vgt_dbg("scan_start: start=%lx end=%lx\n", gma_head, gma_tail);
	while(s.ip_gma != gma_tail){
		if (s.buf_type == RING_BUFFER_INSTRUCTION){
			ASSERT((s.ip_gma >= base) && (s.ip_gma < gma_bottom));
			if (gma_out_of_range(s.ip_gma, gma_head, gma_tail)){
				error_count++;
				vgt_err("ERROR: ip_gma %lx out of range\n", s.ip_gma);
				break;
			}
		}

		stat_nr_cmd_inc(&s);

		rc = vgt_cmd_parser_exec(&s);
		if (rc < 0){
			error_count++;
			vgt_err("error_count=%d\n", error_count);
#if 0
			if (error_count >= MAX_PARSER_ERROR_NUM){
				vgt_err("Reach max error number,stop parsing\n");
			}
			klog_printk("error_count=%d\n", error_count);
			break;
#endif
		}
	}
	klog_printk("ring buffer scan end on ring %d\n", ring_id);
	vgt_dbg("scan_end\n");
	return rc;
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
		vgt_dbg("VGT-Parser.c vring head %x tail %x ctl %x\n",
			vring->head, vring->tail, vring->ctl);
		return 0;
	}

	ret = __vgt_scan_vring (vgt, ring_id, vgt->last_scan_head[ring_id],
		vring->tail & RB_TAIL_OFF_MASK,
		vring->start, _RING_CTL_BUF_SIZE(vring->ctl));

	vgt->last_scan_head[ring_id] = vring->tail;
	return ret;
}

int vgt_cmd_parser_init(struct pgt_device *pdev)
{
	return cmd_hash_init(pdev);
}

void vgt_cmd_parser_exit(void)
{
	vgt_clear_cmd_table();
}
