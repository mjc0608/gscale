/*
 * Decode framebuffer attributes from raw vMMIO
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
/*
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

#include <linux/module.h>
#include "vgt.h"
#include <xen/fb_decoder.h>
#include <uapi/drm/drm_fourcc.h>

#define FORMAT_NUM	16
struct pixel_format {
	int	drm_format;	/* Pixel format in DRM definition */
	int	bpp;		/* Bits per pixel, 0 indicates invalid */
};

/* non-supported format has bpp default to 0 */
static struct pixel_format hsw_pixel_formats[FORMAT_NUM] = {
	[0b0010]  = {DRM_FORMAT_C8, 8},
	[0b0101]  = {DRM_FORMAT_RGB565, 16},
	[0b0110]  = {DRM_FORMAT_XRGB8888, 32},
	[0b1000]  = {DRM_FORMAT_XBGR2101010, 32},
	[0b1010] = {DRM_FORMAT_XRGB2101010, 32},
	[0b1110] = {DRM_FORMAT_XBGR8888, 32},
};

static int vgt_decode_primary_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_primary_plane_format *plane)
{
	u32	val, fmt;

	val = __vreg(vgt, VGT_DSPCNTR(pipe));
	plane->enabled = !!(val & _PRI_PLANE_ENABLE);
	if (!plane->enabled)
		return 0;

	plane->tiled = !!(val & _PRI_PLANE_TILED);

	fmt = (val & _PRI_PLANE_FMT_MASK) >> _PRI_PLANE_FMT_SHIFT;
	if (!IS_HSW(vgt->pdev) || !hsw_pixel_formats[fmt].bpp) {
		vgt_err("Non-supported pixel format (0x%x)\n", fmt);
		return -EINVAL;
	}
	plane->bpp = hsw_pixel_formats[fmt].bpp;
	plane->drm_format = hsw_pixel_formats[fmt].drm_format;

	plane->base = __vreg(vgt, VGT_DSPSURF(pipe)) & GTT_PAGE_MASK;
	plane->width = __vreg(vgt, VGT_DSPSTRIDE(pipe)) &
				_PRI_PLANE_STRIDE_MASK;
	plane->width /= plane->bpp / 8;	/* raw width in bytes */
	plane->height = (__vreg(vgt, VGT_PIPESRC(pipe)) &
			 _PIPE_V_SRCSZ_MASK) >> _PIPE_V_SRCSZ_SHIFT;
	plane->height += 1;	/* raw height is one minus the real value */

	val = __vreg(vgt, VGT_DSPTILEOFF(pipe));
	plane->x_offset = (val & _PRI_PLANE_X_OFF_MASK) >>
			   _PRI_PLANE_X_OFF_SHIFT;
	plane->y_offset = (val & _PRI_PLANE_Y_OFF_MASK) >>
			   _PRI_PLANE_Y_OFF_SHIFT;
	return 0;
}

static int vgt_decode_cursor_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_cursor_plane_format *plane)
{
	/* detail format to be defined later. Now return disabled */
	plane->enabled = 0;
	return 0;
}

static int vgt_decode_sprite_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_sprite_plane_format *plane)
{
	/* detail format to be defined later. Now return disabled */
	plane->enabled = 0;
	return 0;
}

static void vgt_show_primary_plane_format(
	struct vgt_primary_plane_format *plane)
{
	printk("Primary Plane: [%s]\n",
		plane->enabled ? "Enabled" : "Disabled");
	if (!plane->enabled)
		return;

	printk("  tiled: %s\n", plane->tiled ? "yes" : "no");
	if (!plane->bpp) {
		printk("  BROKEN FORMAT (ZERO bpp)\n");
		return;
	}

	printk("  bpp: %d\n", plane->bpp);
	printk("  format: 0x%08x\n", plane->drm_format);
	printk("  base: 0x%x\n", plane->base);
	printk("  x-off: %d\n", plane->x_offset);
	printk("  y-off: %d\n", plane->y_offset);
	printk("  width: %d\n", plane->width);
	printk("  height: %d\n", plane->height);
}

static void vgt_show_cursor_plane_format(
	struct vgt_cursor_plane_format *plane)
{
	printk("Cursor Plane:  [NOT SUPPORTED]\n");
}

static void vgt_show_sprite_plane_format(
	struct vgt_sprite_plane_format *plane)
{
	printk("Sprite Plane:  [NOT SUPPORTED]\n");
}

/* Debug facility */
static void vgt_show_fb_format(int vmid, struct vgt_fb_format *fb)
{
	int i;

	printk("-----------FB format (VM-%d)--------\n", vmid);
	for (i = 0; i < MAX_INTEL_PIPES; i++) {
		struct vgt_pipe_format *pipe = &fb->pipes[i];
		printk("[PIPE-%d]:\n", i);
		vgt_show_primary_plane_format(&pipe->primary);
		vgt_show_cursor_plane_format(&pipe->cursor);
		vgt_show_sprite_plane_format(&pipe->sprite);
	}
	printk("\n");
}

/*
 * Decode framebuffer information from raw vMMIO
 *
 * INPUT:
 *   [domid] - specify the VM
 * OUTPUT:
 *   [format] - contain the decoded format info
 *
 * NOTE: The caller is expected to poll this interface, and reconstruct
 * previous reference to the new format information
 */

int vgt_decode_fb_format(int vmid, struct vgt_fb_format *fb)
{
	int i;
	struct vgt_device *vgt = vmid_2_vgt_device(vmid);
	int ret = 0;

	if (!fb)
		return -EINVAL;

	if (!vgt) {
		vgt_err("Invalid domain ID (%d)\n", vmid);
		return -ENODEV;
	}

	for (i = 0; i < MAX_INTEL_PIPES; i++) {
		struct vgt_pipe_format *pipe = &fb->pipes[i];

		ret |= vgt_decode_primary_plane_format(vgt, i, &pipe->primary);
		ret |= vgt_decode_sprite_plane_format(vgt, i, &pipe->sprite);
		ret |= vgt_decode_cursor_plane_format(vgt, i, &pipe->cursor);

		if (ret) {
			vgt_err("Decode format error for pipe(%d)\n", i);
			ret = -EINVAL;
			break;
		}
	}

	vgt_show_fb_format(vmid, fb);
	return ret;
}
EXPORT_SYMBOL_GPL(vgt_decode_fb_format);
