#ifndef _VGT_RINGBUFFER_H_
#define _VGT_RINGBUFFER_H_

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
