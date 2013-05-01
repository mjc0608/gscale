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

/* color space conversion and gamma correction are not included */
struct vgt_primary_plane_format {
	u8	enabled;	/* plane is enabled */
	u8	tiled;		/* X-tiled */
	u8	bpp;		/* bits per pixel */
	u32	drm_format;	/* format in DRM definition */
	u32	base;		/* framebuffer base in graphics memory */
	u32	x_offset;	/* in pixels */
	u32	y_offset;	/* in lines */
	u32	width;		/* in pixels */
	u32	height;		/* in lines */
};

/* to be defined */
struct vgt_sprite_plane_format{
	u8	enabled;
	u32	padding[6];
};

/* to be defined */
struct vgt_cursor_plane_format{
	u8	enabled;
	u32	padding[6];
};

struct vgt_pipe_format{
	struct vgt_primary_plane_format	primary;
	struct vgt_sprite_plane_format	sprite;
	struct vgt_cursor_plane_format	cursor;
};

#define MAX_INTEL_PIPES	3
struct vgt_fb_format{
	struct vgt_pipe_format	pipes[MAX_INTEL_PIPES];
};

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
int vgt_decode_fb_format(int vmid, struct vgt_fb_format *fb);
