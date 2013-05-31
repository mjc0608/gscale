/*
 * Decode framebuffer attributes from raw vMMIO
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

#include <linux/module.h>
#include "vgt.h"
#include <xen/fb_decoder.h>
#include <uapi/drm/drm_fourcc.h>

#define FORMAT_NUM	16
struct pixel_format {
	int	drm_format;	/* Pixel format in DRM definition */
	int	bpp;		/* Bits per pixel, 0 indicates invalid */
	char	*desc;		/* The description */
};

/* non-supported format has bpp default to 0 */
static struct pixel_format hsw_pixel_formats[FORMAT_NUM] = {
	[0b0010]  = {DRM_FORMAT_C8, 8, "8-bit Indexed"},
	[0b0101]  = {DRM_FORMAT_RGB565, 16, "16-bit BGRX (5:6:5 MSB-R:G:B)"},
	[0b0110]  = {DRM_FORMAT_XRGB8888, 32, "32-bit BGRX (8:8:8:8 MSB-X:R:G:B)"},
	[0b1000]  = {DRM_FORMAT_XBGR2101010, 32, "32-bit RGBX (2:10:10:10 MSB-X:B:G:R)"},
	[0b1010] = {DRM_FORMAT_XRGB2101010, 32, "32-bit BGRX (2:10:10:10 MSB-X:R:G:B)"},
	[0b1110] = {DRM_FORMAT_XBGR8888, 32, "32-bit RGBX (8:8:8:8 MSB-X:B:G:R)"},
};

static int vgt_decode_primary_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_primary_plane_format *plane)
{
	u32	val, fmt;

	val = __vreg(vgt, VGT_DSPCNTR(pipe));
	plane->enabled = !!(val & _PRI_PLANE_ENABLE);
	if (!plane->enabled)
		return 0;

	plane->tiled = !!(val & _PRI_PLANE_TILE_MASK);

	fmt = (val & _PRI_PLANE_FMT_MASK) >> _PRI_PLANE_FMT_SHIFT;
	if (!hsw_pixel_formats[fmt].bpp) {
		vgt_err("Non-supported pixel format (0x%x)\n", fmt);
		return -EINVAL;
	}
	plane->hw_format = fmt;
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

#define CURSOR_MODE_NUM	(1 << 6)
struct cursor_mode_format {
	int	drm_format;	/* Pixel format in DRM definition */
	u8	bpp;		/* Bits per pixel; 0 indicates invalid */
	u32	width;		/* In pixel */
	u32	height;		/* In lines */
	char	*desc;		/* The description */
};

/* non-supported format has bpp default to 0 */
static struct cursor_mode_format hsw_cursor_mode_formats[CURSOR_MODE_NUM] = {
	[0b100010]  = {DRM_FORMAT_ARGB8888, 32, 128, 128,"128x128 32bpp ARGB"},
	[0b100011]  = {DRM_FORMAT_ARGB8888, 32, 256, 256, "256x256 32bpp ARGB"},
	[0b100111]  = {DRM_FORMAT_ARGB8888, 32, 64, 64, "64x64 32bpp ARGB"},
};
static int vgt_decode_cursor_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_cursor_plane_format *plane)
{
	u32 val, mode;
	u32 alpha_plane, alpha_force;

	val = __vreg(vgt, VGT_CURCNTR(pipe));
	mode = val & _CURSOR_MODE;
	plane->enabled = (mode != _CURSOR_MODE_DISABLE);
	if (!plane->enabled)
		return 0;

	if (!hsw_cursor_mode_formats[mode].bpp) {
		vgt_err("Non-supported cursor mode (0x%x)\n", mode);
		return -EINVAL;
	}
	plane->mode = mode;
	plane->bpp = hsw_cursor_mode_formats[mode].bpp;
	plane->drm_format = hsw_cursor_mode_formats[mode].drm_format;
	plane->width = hsw_cursor_mode_formats[mode].width;
	plane->height = hsw_cursor_mode_formats[mode].height;

	alpha_plane = (val & _CURSOR_ALPHA_PLANE_MASK) >>
				_CURSOR_ALPHA_PLANE_SHIFT;
	alpha_force = (val & _CURSOR_ALPHA_FORCE_MASK) >>
				_CURSOR_ALPHA_FORCE_SHIFT;
	if (alpha_plane || alpha_force)
		vgt_warn("alpha_plane=0x%x, alpha_force=0x%x\n",
			alpha_plane, alpha_force);

	plane->base = __vreg(vgt, VGT_CURBASE(pipe)) & GTT_PAGE_MASK;

	val = __vreg(vgt, VGT_CURPOS(pipe));
	plane->x_pos = (val & _CURSOR_POS_X_MASK) >> _CURSOR_POS_X_SHIFT;
	plane->x_sign = (val & _CURSOR_SIGN_X_MASK) >> _CURSOR_SIGN_X_SHIFT;
	plane->y_pos = (val & _CURSOR_POS_Y_MASK) >> _CURSOR_POS_Y_SHIFT;
	plane->y_sign = (val & _CURSOR_SIGN_Y_MASK) >> _CURSOR_SIGN_Y_SHIFT;

	return 0;
}

#define FORMAT_NUM_SRRITE	(1 << 3)

/* The formats described in the sprite format field are the 1st level of
 * cases RGB and YUV formats are further refined by the color_order and
 * yuv_order fields to cover the full set of possible formats.
 */

static struct pixel_format hsw_pixel_formats_sprite[FORMAT_NUM_SRRITE] = {
	[0b000]  = {DRM_FORMAT_YUV422, 16, "YUV 16-bit 4:2:2 packed"},
	[0b001]  = {DRM_FORMAT_XRGB2101010, 32, "RGB 32-bit 2:10:10:10"},
	[0b010]  = {DRM_FORMAT_XRGB8888, 32, "RGB 32-bit 8:8:8:8"},
	[0b100] = {DRM_FORMAT_AYUV, 32, "YUV 32-bit 4:4:4 packed (8:8:8:8 MSB-X:Y:U:V)"},
};

/* Non-supported format has bpp default to 0 */
static int vgt_decode_sprite_plane_format(struct vgt_device *vgt,
	int pipe, struct vgt_sprite_plane_format *plane)
{
	u32 val, fmt;
	u32 width;
	u32 color_order, yuv_order;
	int drm_format;

	val = __vreg(vgt, VGT_SPRCTL(pipe));
	plane->enabled = !!(val & _SPRITE_ENABLE);
	if (!plane->enabled)
		return 0;

	plane->tiled = !!(val & _SPRITE_TILED);
	color_order = !!(val & _SPRITE_COLOR_ORDER_MASK);
	yuv_order = (val & _SPRITE_YUV_ORDER_MASK) >>
				_SPRITE_YUV_ORDER_SHIFT;

	fmt = (val & _SPRITE_FMT_MASK) >> _SPRITE_FMT_SHIFT;
	if (!hsw_pixel_formats_sprite[fmt].bpp) {
		vgt_err("Non-supported pixel format (0x%x)\n", fmt);
		return -EINVAL;
	}
	plane->hw_format = fmt;
	plane->bpp = hsw_pixel_formats_sprite[fmt].bpp;
	drm_format = hsw_pixel_formats_sprite[fmt].drm_format;

	/* Order of RGB values in an RGBxxx buffer may be ordered RGB or
	 * BGR depending on the state of the color_order field
	 */
	if (!color_order) {
		if (drm_format == DRM_FORMAT_XRGB2101010)
			drm_format = DRM_FORMAT_XBGR2101010;
		else if (drm_format == DRM_FORMAT_XRGB8888)
			drm_format = DRM_FORMAT_XBGR8888;
	}

	if (drm_format == DRM_FORMAT_YUV422) {
		switch (yuv_order){
		case	0:
			drm_format = DRM_FORMAT_YUYV;
			break;
		case	1:
			drm_format = DRM_FORMAT_UYVY;
			break;
		case	2:
			drm_format = DRM_FORMAT_YVYU;
			break;
		case	3:
			drm_format = DRM_FORMAT_VYUY;
			break;
		default:
			/* yuv_order has only 2 bits */
			BUG();
			break;
		}
	}

	plane->drm_format = drm_format;

	plane->base = __vreg(vgt, VGT_SPRSURFPIPE(pipe)) & GTT_PAGE_MASK;
	plane->width = __vreg(vgt, VGT_SPRSTRIDE(pipe)) &
				_SPRITE_STRIDE_MASK;
	plane->width /= plane->bpp / 8;	/* raw width in bytes */

	val = __vreg(vgt, VGT_SPRSIZE(pipe));
	plane->height = (val & _SPRITE_SIZE_HEIGHT_MASK) >>
		_SPRITE_SIZE_HEIGHT_SHIFT;
	width = (val & _SPRITE_SIZE_WIDTH_MASK) >> _SPRITE_SIZE_WIDTH_SHIFT;
	plane->height += 1;	/* raw height is one minus the real value */
	width += 1;		/* raw width is one minus the real value */
	if (plane->width != width)
		vgt_warn("sprite_plane: plane->width=%d, width=%d\n",
			plane->width, width);

	val = __vreg(vgt, VGT_SPRPOS(pipe));
	plane->x_pos = (val & _SPRITE_POS_X_MASK) >> _SPRITE_POS_X_SHIFT;
	plane->y_pos = (val & _SPRITE_POS_Y_MASK) >> _SPRITE_POS_Y_SHIFT;

	val = __vreg(vgt, VGT_SPROFFSET(pipe));
	plane->x_offset = (val & _SPRITE_OFFSET_START_X_MASK) >>
			   _SPRITE_OFFSET_START_X_SHIFT;
	plane->y_offset = (val & _SPRITE_OFFSET_START_Y_MASK) >>
			   _SPRITE_OFFSET_START_Y_SHIFT;
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
	printk("  drm_format: 0x%08x: %s\n", plane->drm_format,
		hsw_pixel_formats[plane->hw_format].desc);
	printk("  base: 0x%x\n", plane->base);
	printk("  x-off: %d\n", plane->x_offset);
	printk("  y-off: %d\n", plane->y_offset);
	printk("  width: %d\n", plane->width);
	printk("  height: %d\n", plane->height);
}

static void vgt_show_cursor_plane_format(
	struct vgt_cursor_plane_format *plane)
{
	printk("Cursor Plane: [%s]\n",
		plane->enabled ? "Enabled" : "Disabled");
	if (!plane->enabled)
		return;

	if (!plane->bpp) {
		printk("  BROKEN FORMAT (ZERO bpp)\n");
		return;
	}

	printk("  bpp: %d\n", plane->bpp);
	printk("  mode: 0x%08x: %s\n", plane->mode,
		hsw_cursor_mode_formats[plane->mode].desc);
	printk("  drm_format: 0x%08x\n", plane->drm_format);
	printk("  base: 0x%x\n", plane->base);
	printk("  x-pos: %d\n", plane->x_pos);
	printk("  y-pos: %d\n", plane->y_pos);
	printk("  x-sign: %d\n", plane->x_sign);
	printk("  y-sign: %d\n", plane->y_sign);
	printk("  width: %d\n", plane->width);
	printk("  height: %d\n", plane->height);
}

static void vgt_show_sprite_plane_format(
	struct vgt_sprite_plane_format *plane)
{
	printk("Sprite Plane: [%s]\n",
		plane->enabled ? "Enabled" : "Disabled");
	if (!plane->enabled)
		return;

	printk("  tiled: %s\n", plane->tiled ? "yes" : "no");
	if (!plane->bpp) {
		printk("  BROKEN FORMAT (ZERO bpp)\n");
		return;
	}

	printk("  bpp: %d\n", plane->bpp);
	printk("  drm_format: 0x%08x: %s\n", plane->drm_format,
		hsw_pixel_formats_sprite[plane->hw_format].desc);
	printk("  base: 0x%x\n", plane->base);
	printk("  x-off: %d\n", plane->x_offset);
	printk("  y-off: %d\n", plane->y_offset);
	printk("  x-pos: %d\n", plane->x_pos);
	printk("  y-pos: %d\n", plane->y_pos);
	printk("  width: %d\n", plane->width);
	printk("  height: %d\n", plane->height);
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

	if (!IS_HSW(vgt->pdev)) {
		vgt_err("Only HSW is supported now\n");
		return -EINVAL;
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
