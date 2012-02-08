/*
 * vGT core module
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

#ifndef _VGT_REG_H_
#define _VGT_REG_H_

#define SINGLE_VM_DEBUG
#define SANDY_BRIDGE
#define ASSERT(x)   do { if (!(x)) printk("Assert at %s line %d\n", __FILE__, __LINE__);} while (0);

typedef uint32_t vgt_reg_t;

/*
 * Define registers of a ring buffer per hardware  register layout.
 */
typedef struct {
    vgt_reg_t  tail;
    vgt_reg_t  head;
    vgt_reg_t  start;
    vgt_reg_t  ctl;
} vgt_ringbuffer_t;
#define _tail_reg_(ring_reg_off)	\
		(ring_reg_off & ~(sizeof(vgt_ringbuffer_t)-1))

#ifdef SANDY_BRIDGE
#define  MAX_ENGINES		3
#else
#define  MAX_ENGINES		5
#endif

#define _vgt_mmio_va(vgt, x)		((char*)NULL+x)	/* PA to VA */
#define sleep_ns(x)	{long y=1UL*x/2; while (y-- > 0) ;}
#define sleep_us(x)	{long y=500UL*x; while (y-- > 0) ;}

#define SIZE_1MB			(1024UL*1024UL)

#if 0
#define VGT_GUEST_APERTURE_SZ		(128*SIZE_1MB)
#define VGT_DOM0_APERTURE_SZ		(64*SIZE_1MB)	/* 64MB for dom0 */
#define VGT_APERTURE_SZ			(64*SIZE_1MB)	/* reserve 64MB */
#define VGT_MAX_VMS		4	/* the maximum # of VMs VGT can support */
#else	/* Initial Configuration */
#define VGT_GUEST_APERTURE_SZ		(128*SIZE_1MB)
#define VGT_DOM0_APERTURE_SZ		(128*SIZE_1MB)	/* 64MB for dom0 */
#define VGT_APERTURE_SZ			(128*SIZE_1MB)	/* reserve 64MB */
#define VGT_MAX_VMS		3	/* the maximum # of VMs VGT can support */
#endif

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*16)	/* use 64 KB for now */
extern unsigned long vgt_id_alloc_bitmap;
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#ifdef SINGLE_VM_DEBUG

#define VGT_DOM0_GFX_APERTURE_BASE		0
#define VGT_VM1_APERTURE_BASE	(VGT_DOM0_GFX_APERTURE_BASE + VGT_DOM0_APERTURE_SZ)
#define VGT_VM2_APERTURE_BASE	(VGT_VM1_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)
#define VGT_APERTURE_BASE	(VGT_VM2_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)

#else
/*
 * Layout of APERTURE (total 512MB):
 *	VM1: 0-128MB
 *	VM2: 128MB-256MB
 *	VM3: 256-384MB
 *	DOM0: GFX driver: 384MB-448MB
 *	VGT driver (in Dom0): 448MB-512MB (54MB)
 *		Used for context save area (128KB per VGT instance)
 *			4KB per ring context save area
 * TODO: This may require Gfx driver modification!!!
 */
#define VGT_VM1_APERTURE_BASE	0
#define VGT_VM2_APERTURE_BASE	(VGT_VM1_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)
#define VGT_VM3_APERTURE_BASE	(VGT_VM2_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)
#define VGT_DOM0_GFX_APERTURE_BASE		\
		(VGT_VM3_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)
#define VGT_MANAGE_APERTURE_BASE		\
		(VGT_DOM0_GFX_APERTURE_BASE + VGT_DOM0_APERTURE_SZ)
#endif
#define VGT_APERTURE_PER_INSTANCE_SZ		(4*SIZE_1MB)	/* 4MB per instance (?) */

#define REG_SIZE    sizeof(vgt_reg_t)        /* size of gReg/sReg[0] */
#define VGT_MMIO_SPACE_SZ	(2*SIZE_1MB)
#define VGT_MMIO_REG_NUM	(VGT_MMIO_SPACE_SZ/REG_SIZE)	/* 2MB space in totoal */
#define VGT_CFG_SPACE_SZ	256
typedef struct {
    int		regNum;		/* Total number of MMIO registers in vGT */
    uint64_t    mmio_base_gpa;	/* base guest physical address of the MMIO registers */
    vgt_reg_t	*vReg;		/* guest view of the register state */
    vgt_reg_t	*sReg;		/* Shadow (used by hardware) state of the register */
    uint8_t	cfg_space[VGT_CFG_SPACE_SZ];
    uint64_t	gt_mmio_base;	/* bar0/GTTMMIO  */
    uint64_t	aperture_base_pa;	/* bar1: guest aperture base */
//    uint64_t	gt_gmadr_base;	/* bar1/GMADR */

    uint32_t	bar_size[3];	/* 0: GTTMMIO, 1: GMADR, 2: PIO bar size */
} vgt_state_t;

#define __vreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))

#define RB_DWORDS_TO_SAVE	32
typedef	uint32_t	rb_dword;
typedef struct {
	vgt_ringbuffer_t	vring;		/* guest view ring */
	vgt_ringbuffer_t	sring;		/* shadow ring */
	vgt_reg_t	phys_tail;	/* temproray tail reg for context S/R */
	rb_dword	save_buffer[RB_DWORDS_TO_SAVE];
	/* In aperture, partitioned & 4KB aligned. */
	/* 64KB alignment requirement for walkaround. */
	uint64_t	context_save_area;
	bool	initialized;	/* whether it includes an valid context */
} vgt_state_ring_t;

struct vgt_device;
typedef bool (*vgt_mmio_read)(struct vgt_device *vgt, unsigned int offset,
	 void *p_data, unsigned int bytes);
typedef bool (*vgt_mmio_write)(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);
struct mmio_hash_table	{
	struct mmio_hash_table	*next;
	int	mmio_base;		/* 4 byte aligned */
	vgt_mmio_read	read;
	vgt_mmio_write	write;
};
#define		MHASH_SIZE_SHIFT	6
#define		MHASH_SIZE	(1<<MHASH_SIZE_SHIFT)
#define		mhash(x)	hash_32(x & ~3, MHASH_SIZE_SHIFT);
//#define mhash(x)	(((x >> 2) & ~(x>>8) & ~(x>>14) ) & (MHASH_SIZE-1))


/*
 * Ring ID definition.
 */
#define RING_BUFFER_RCS		0
#define RING_BUFFER_VCS		1
#define RING_BUFFER_BCS		2
#define RING_BUFFER_VECS	3
#define RING_BUFFER_VCS2	4
extern unsigned int ring_mmio_base[MAX_ENGINES];

/*
 * Definition of MMIO registers.
 */

/* PRB0, RCS */
#define _REG_RCS_TAIL		0x02030
#define _REG_RCS_HEAD		0x02034
#define _REG_RCS_START	 	0x02038
#define _REG_RCS_CTL    	0x0203c

/* VECS: HSW+ */
#define _REG_VECS_TAIL		0x1A030
#define _REG_VECS_HEAD		0x1A034
#define _REG_VECS_START		0x1A038
#define _REG_VECS_CTL		0x1A03c

/* VCS */
#define _REG_VCS_TAIL		0x12030
#define _REG_VCS_HEAD		0x12034
#define _REG_VCS_START		0x12038
#define _REG_VCS_CTL		0x1203c

/* VCS2: BDW */
#define _REG_VCS2_TAIL		0x1C030
#define _REG_VCS2_HEAD		0x1C034
#define _REG_VCS2_START		0x1C038
#define _REG_VCS2_CTL		0x1C03c

/* BCS */
#define _REG_BCS_TAIL		0x22030
#define _REG_BCS_HEAD		0x22034
#define _REG_BCS_START		0x22038
#define _REG_BCS_CTL		0x2203c

#define RB_OFFSET_TAIL		0
#define RB_OFFSET_HEAD		4
#define RB_OFFSET_START		8
#define RB_OFFSET_CTL		0xC
#define RB_REGS_SIZE		0x10

#define RB_TAIL_SIZE_MASK	((1UL << 21) - (1UL << 12))	/* bit 12 to 20 */
#define GPU_PAGE_SIZE		(1UL<<12)
#define GPU_PAGE_MASK		(~(GPU_PAGE_SIZE-1))
#define _RING_CTL_BUF_SIZE(ctl)	(((ctl) & RB_TAIL_SIZE_MASK) + GPU_PAGE_SIZE)
#define _RING_CTL_ENABLE	0x1	/* bit 0 */

#define _REG_CCID		0x02180
#define CCID_MBO_BITS		0x100		/* bit 8 must be one */
#define CCID_EXTENDED_STATE_SAVE_ENABLE		0x8
#define CCID_EXTENDED_STATE_RESTORE_ENABLE	0x4
#define CCID_VALID		0x1
#define CCID_TIMEOUT_LIMIT	150

#define _REG_ISR		    0x020AC


static inline bool is_ring_empty(vgt_ringbuffer_t *rb)
{
    return (rb->head == rb->tail);
}

static inline bool is_ring_enabled (vgt_ringbuffer_t *rb)
{
	return (rb->ctl & 1);	/* bit 0: enable/disable RB */
}

extern int vgt_thread(void *priv);
extern void vgt_destroy(void);
extern int vgt_initialize(struct pci_bus *bus);
extern bool vgt_register_mmio_handler(int start, int end,
	vgt_mmio_read read, vgt_mmio_write write);
extern bool vgt_initialize_mmio_hooks(void);

/* per-VM structure */
struct vgt_device {
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device  *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */
	void		*aperture_base_va;
	unsigned int	aperture_offset;	/* TODO: for aperture virtualization */
	void	*priv;
	uint64_t  vgt_aperture_base;	/* aperture used for VGT driver */
	vgt_reg_t		saved_wakeup;	/* disable PM before switching */
};

enum vgt_owner_type {
	VGT_OT_INVALID = 0,
	VGT_OT_RENDER,                  // the owner directly operating render command buffers
	VGT_OT_BLITTER,                 // the owner directly operating blitter command buffers
	VGT_OT_VIDEO,                   // the owner directly operating video command buffers
	VGT_OT_GT,                      // the owner directly operating all render buffers (render/blit/video)
	VGT_OT_DISPLAY,                 // the owner having its content directly shown on one or several displays
	VGT_OT_PM,                      // the owner handling GEN power management activities
	VGT_OT_MGMT,                    // the owner managing display/monitor resources
	VGT_OT_MAX,
};

/* per-device structure */
struct pgt_device {
	struct pci_bus *pbus;	/* parent bus of the device */
	int bus;		/* parent bus number */
	int devfn;		/* device function number */

	vgt_ringbuffer_t *ring_base_vaddr[MAX_ENGINES];	/* base vitrual address of ring buffer mmios */
	vgt_reg_t initial_mmio_state[VGT_MMIO_REG_NUM];	/* copy from physical at start */
	uint8_t initial_cfg_space[VGT_CFG_SPACE_SZ];	/* copy from physical at start */
	uint32_t bar_size[3];
	uint64_t gttmmio_base;	/* base of GTT */
	void *gttmmio_base_va;	/* virtual base of GTT */
	uint64_t gmadr_base;	/* base of GMADR */
	void *phys_gmadr_va;	/* virtual base of GMADR */

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct list_head rendering_runq_head;	/* ??? */
	struct list_head rendering_idleq_head;	/* ??? */
	bool switch_inprogress;	/* an ownership switch in progress */
	enum vgt_owner_type switch_owner;	/* the type of the owner in switch */
};

#define vgt_get_owner(d, t)             (d->owner[t])
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_GT))
#define current_display_owner(d)	((vgt_get_owner(d, VGT_OT_DISPLAY))->id)
#define vgt_switch_inprogress(d)        (d->switch_inprogress)
#define vgt_switch_owner_type(d)        (d->switch_owner)

#define _REG_WRITE_(preg, val)	{ *(volatile vgt_reg_t *)preg = val;}
#define _REG_READ_(preg)		(*(volatile vgt_reg_t *)preg)

#define VGT_MMIO_WRITE(vgt, mmio_offset, val)	\
		_REG_WRITE_(_vgt_mmio_va(vgt, mmio_offset), val);


#define VGT_MMIO_READ(vgt, mmio_offset)		\
		_REG_READ_(_vgt_mmio_va(vgt, mmio_offset))

#define ARRAY_NUM(x)		(sizeof(x) / sizeof(x[0]))

/*
 * Activate/Deactive an VGT instance.
 */
static inline void vgt_active(struct pgt_device *pdev, struct list_head *rq)
{
	list_del(rq);		/* remove from idle queue */
	list_add(rq, &pdev->rendering_runq_head);	/* add to run queue */
}

/*
 * Remove from run queue, but the vgt may be still in executing.
 */
static inline void vgt_deactive(struct pgt_device *pdev, struct list_head *rq)
{
	/* TODO: make sure it is not the current vgt */
	list_del(rq);		/* remove from run queue */
	list_add(rq, &pdev->rendering_idleq_head);	/* add to idle queue */
}


/*
 * Next MACROs for GT configuration space.
 */
#define VGT_REG_CFG_SPACE_BAR0			0x10
#define VGT_REG_CFG_SPACE_BAR1			0x18
#define VGT_REG_CFG_SPACE_BAR2			0x20
#define VGT_REG_CFG_SPACE_MSAC			0x62

//#define MSAC_APERTURE_SIZE_MASK		0x3
#define MSAC_APERTURE_SIZE_128M			(0 << 1)
#define MSAC_APERTURE_SIZE_256M			(1 << 1)
#define MSAC_APERTURE_SIZE_512M			(3 << 1)


#include "vgt_wr.h"
#define emulated_regs_t	i915emuRegs_t
extern emulated_regs_t	gpuregs[];

#endif	/* _VGT_REG_H_ */
