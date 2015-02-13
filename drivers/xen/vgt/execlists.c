/*
 * BDW EXECLIST supports
 *
 * Copyright(c) 2011-2015 Intel Corporation. All rights reserved.
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

#include "trace.h"
#include <linux/kthread.h>
#include <xen/interface/vcpu.h>
#include "vgt.h"

//#define EL_SLOW_DEBUG

#define vgt_require_shadow_context(vgt)	(!((vgt) && (vgt->vm_id == 0)))

#define EXECLIST_CTX_PAGES(ring_id)	((ring_id) == RING_BUFFER_RCS ? 20 : 2)

static inline bool el_lrca_is_valid(struct vgt_device *vgt, uint32_t lrca)
{
	bool rc;
	uint32_t gma;

	if (lrca == 0) {
		/* Always return true for lrca as 0. Context status buffer
		 * contains valid entry with lrca/ctx_id as 0, especially
		 * for the idle_to_active events.
		 */
		return true;
	}

	gma = lrca << GTT_PAGE_SHIFT;
	rc = g_gm_is_valid(vgt, gma);
	if (!rc) {
		/* it is a shadow context */
		rc = g_gm_is_reserved(vgt, gma);
	}

	return rc;
}

static inline bool vgt_validate_elsp_descs(struct vgt_device *vgt,
			struct ctx_desc_format *ctx0,
			struct ctx_desc_format *ctx1)
{
	if (!ctx0->valid) {
		vgt_err("Context[0] is invalid! Which is not expected\n");
		return false;
	}

	if (!el_lrca_is_valid(vgt, ctx0->lrca)) {
		vgt_err("The context[0] in ELSP does not have a valid lrca(0x%x)!",
				ctx0->lrca);
		return false;
	}

	if (ctx1->valid) {
		if (!el_lrca_is_valid(vgt, ctx1->lrca)) {
			vgt_err("The context[1] in ELSP does not have a "
				"valid lrca(0x%x)!", ctx1->lrca);
			return false;
		}
	}

	return true;
}

static bool vgt_validate_elsp_submission(struct vgt_device *vgt,
			struct vgt_elsp_store *elsp_store)
{
	struct ctx_desc_format *ctx0;
	struct ctx_desc_format *ctx1;
	ctx0 = (struct ctx_desc_format *)&elsp_store->element[2];
	ctx1 = (struct ctx_desc_format *)&elsp_store->element[0];

	return vgt_validate_elsp_descs(vgt, ctx0, ctx1);
}

static inline bool vgt_hw_ELSP_write(struct vgt_device *vgt,
				unsigned int reg,
				struct ctx_desc_format *ctx0,
				struct ctx_desc_format *ctx1)
{
	int rc = true;

	ASSERT(ctx0 && ctx1);

	vgt_dbg(VGT_DBG_EXECLIST, "EXECLIST is submitted into hardware! "
			"Writing 0x%x with: 0x%x; 0x%x; 0x%x; 0x%x\n",
			reg,
			ctx1->elm_high, ctx1->elm_low,
			ctx0->elm_high, ctx0->elm_low);

	vgt_force_wake_get();

	VGT_MMIO_WRITE(vgt->pdev, reg, ctx1->elm_high);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx1->elm_low);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx0->elm_high);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx0->elm_low);

	vgt_force_wake_put();

	return rc;
}

bool vgt_batch_ELSP_write(struct vgt_device *vgt, int ring_id)
{
	struct vgt_elsp_store *elsp_store = &vgt->rb[ring_id].elsp_store;

	struct ctx_desc_format *ctx_descs[2];

	ASSERT(elsp_store->count == ELSP_BUNDLE_NUM);
	if (!vgt_validate_elsp_submission(vgt, elsp_store)) {
		vgt_err("VM(%d): Failed to submit an execution list!\n",
						vgt->vm_id);
		return false;
	}

	ctx_descs[0] = (struct ctx_desc_format *)&elsp_store->element[2];
	ctx_descs[1] = (struct ctx_desc_format *)&elsp_store->element[0];

	elsp_store->count = 0;
	vgt_enable_ring(vgt, ring_id);

	if (hvm_render_owner) {
		uint32_t elsp_reg = vgt_ring_id_to_EL_base(ring_id) +
						_EL_OFFSET_SUBMITPORT;
		if (!is_current_render_owner(vgt)) {
			vgt_warn("VM-%d: ELSP submission but VM is not "
			"render owner! But it will still be submitted.\n",
				vgt->vm_id);
		}
		vgt_hw_ELSP_write(vgt, elsp_reg, ctx_descs[0], ctx_descs[1]);
		return true;
	}

	return true;
}
