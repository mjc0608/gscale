#ifndef _VGT_RINGBUFFER_H_
#define _VGT_RINGBUFFER_H_

/*
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

struct vgt_ring_buffer {
	struct pgt_device *pdev;
	void *virtual_start;
	int offset;

	u32 head;
	u32 tail;
	int space;
	int size;
};

void vgt_ring_init(struct pgt_device *pdev);

void vgt_ring_start(struct vgt_ring_buffer *ring);

int vgt_wait_ring_buffer(struct vgt_ring_buffer *ring, int n);

static inline int vgt_wait_ring_idle(struct vgt_ring_buffer *ring)
{
	return vgt_wait_ring_buffer(ring, ring->size - 8);
}

/* We just take easy implement of vGT ring, which would always be initialized
 * from 0 before use. And we don't care about wrap case.
 */
static inline void vgt_ring_begin(struct vgt_ring_buffer *ring, int num_dwords)
{
	ring->space -= num_dwords * 4;
}

static inline void vgt_ring_emit(struct vgt_ring_buffer *ring,
				u32 data)
{
	writel(data, ring->virtual_start + ring->tail);
	ring->tail += 4;
}

void vgt_ring_advance(struct vgt_ring_buffer *ring);

void vgt_ring_reset(struct vgt_ring_buffer *ring);

#endif
