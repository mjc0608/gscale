/*
 * vGT core headers
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

#include <linux/hrtimer.h>
#include <linux/interrupt.h>

#define SINGLE_VM_DEBUG
#define SANDY_BRIDGE
#define ASSERT(x)   do { if (!(x)) {printk("Assert at %s line %d\n", __FILE__, __LINE__); BUG();}} while (0);

//#define VGT_DEBUG
#ifdef VGT_DEBUG
#define dprintk(fmt, a...)	\
	printk("vGT:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##a)
#else
#define dprintk(fmt, a...)
#endif


#define snb_device(dev)	1
#define ivb_device(dev)	0

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

#define _vgt_mmio_va(pdev, x)		((char*)pdev->gttmmio_base_va+x)	/* PA to VA */
#define _vgt_mmio_pa(pdev, x)		(pdev->gttmmio_base+x)	/* PA to VA */
#define sleep_ns(x)	{long y=1UL*x/2; while (y-- > 0) ;}
#define sleep_us(x)	{long y=500UL*x; while (y-- > 0) ;}

#define SIZE_1MB			(1024UL*1024UL)

#if 0
#define VGT_GUEST_APERTURE_SZ		(128*SIZE_1MB)
#define VGT_DOM0_APERTURE_SZ		(64*SIZE_1MB)	/* 64MB for dom0 */
#define VGT_APERTURE_SZ			(64*SIZE_1MB)	/* reserve 64MB */
#define VGT_MAX_VMS		4	/* the maximum # of VMs VGT can support */
#else	/* Initial Configuration */
/*
 * Only 256M gfx memory available on SNB. Need turn this static configuration
 * into model based or dynamically
 */
#define VGT_GUEST_APERTURE_SZ		(128*SIZE_1MB)
#define VGT_DOM0_APERTURE_SZ		(64*SIZE_1MB)	/* 64MB for dom0 */
#define VGT_APERTURE_SZ			(64*SIZE_1MB)	/* reserve 64MB */
#define VGT_TOTAL_APERTURE_SZ		(256*SIZE_1MB)

#define VGT_TOTAL_APERTURE_PAGES	(VGT_TOTAL_APERTURE_SZ >> GTT_PAGE_SHIFT)
#define VGT_APERTURE_PAGES		(VGT_APERTURE_SZ >> GTT_PAGE_SHIFT)
/*
 * SNB support 1G/2G graphics memory size
 * Assume dom0 and vGT itself has no extra gfx memory requirement except aperture
 */
#define VGT_GUEST_GFXMEM_SZ		(512*SIZE_1MB)
#define VGT_MAX_VMS		2	/* the maximum # of VMs VGT can support */
#endif

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*64)	/* use 64 KB for now */
extern unsigned long vgt_id_alloc_bitmap;
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#ifdef SINGLE_VM_DEBUG

/* SNB only support one VM now */
#define VGT_DOM0_GFX_APERTURE_BASE		0
#define VGT_VM1_APERTURE_BASE	(VGT_DOM0_GFX_APERTURE_BASE + VGT_DOM0_APERTURE_SZ)
#define VGT_VM2_APERTURE_BASE	(VGT_VM1_APERTURE_BASE)
#define VGT_APERTURE_BASE	(VGT_VM2_APERTURE_BASE+VGT_GUEST_APERTURE_SZ)

#else
/*
 * Layout of APERTURE (total 512MB):
 *	VM1: 0-128MB
 *	VM2: 128MB-256MB
 *	VM3: 256-384MB
 *	DOM0: GFX driver: 384MB-448MB
 *	VGT driver (in Dom0): 448MB-512MB (64MB)
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
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.sReg + off))
#define __vreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.vReg + off))
#define __sreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.sReg + off))
#define vgt_vreg(vgt, off)	((vgt_reg_t *)vgt->state.vReg + off)
#define vgt_sreg(vgt, off)	((vgt_reg_t *)vgt_>state.vReg + off)

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

typedef bool (*submit_context_command_t) (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);
bool rcs_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);
bool default_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);

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

#define RB_TAIL(id)	(ring_mmio_base[id] + RB_OFFSET_TAIL)
#define RB_HEAD(id)	(ring_mmio_base[id] + RB_OFFSET_HEAD)
#define RB_START(id)	(ring_mmio_base[id] + RB_OFFSET_START)
#define RB_CTL(id)	(ring_mmio_base[id] + RB_OFFSET_CTL)

#define RB_HEAD_OFF_MASK	((1UL << 21) - (1UL << 2))	/* bit 2 to 20 */
#define RB_HEAD_OFF_SHIFT	2
#define RB_TAIL_OFF_MASK	((1UL << 21) - (1UL << 3))	/* bit 2 to 20 */
#define RB_TAIL_OFF_SHIFT	3

#define RB_TAIL_SIZE_MASK	((1UL << 21) - (1UL << 12))	/* bit 12 to 20 */
#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_ENTRY_SIZE		4
#define GTT_INDEX(pdev, addr)		\
	((u32)((addr - pdev->gmadr_base) >> GTT_PAGE_SHIFT))
#define GTT_ADDR(pdev, index)		\
	(pdev->gtt_base + index * GTT_ENTRY_SIZE)
#define _RING_CTL_BUF_SIZE(ctl)	(((ctl) & RB_TAIL_SIZE_MASK) + GTT_PAGE_SIZE)
#define _RING_CTL_ENABLE	0x1	/* bit 0 */

#define _REG_CCID		0x02180
#define CCID_MBO_BITS		0x100		/* bit 8 must be one */
#define CCID_EXTENDED_STATE_SAVE_ENABLE		0x8
#define CCID_EXTENDED_STATE_RESTORE_ENABLE	0x4
#define CCID_VALID		0x1
#define CCID_TIMEOUT_LIMIT	150

#define _REG_ISR		    0x020AC

#define _REG_MI_MODE	0x209C
#define		_REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE	(1 << 14)
#define		_REGBIT_MI_FLUSH_PERFORMANCE_MODE	(1 << 13)
//#define		_REGBIT_MI_FLUSH			(3 << 11)
#define		_REGBIT_MI_FLUSH			(1 << 12)
#define		_REGBIT_MI_INVALIDATE_UHPTR		(1 << 11)
#define _REG_GFX_MODE	0x2520
#define		_REGBIT_FLUSH_TLB_INVALIDATION_MODE	(1 << 13)
#define		_REGBIT_REPLAY_MODE			(1 << 11)
#define		_REGBIT_PPGTT				(1 << 9)
#define _REG_GFX_MODE_IVB	0x229C
#define _REG_ARB_MODE	0x4030
#define		_REGBIT_ADDRESS_SWIZZLING		(3 << 4)

extern int vgt_thread(void *priv);
extern void vgt_destroy(void);
extern int vgt_initialize(struct pci_dev *dev);
extern bool vgt_register_mmio_handler(int start, int end,
	vgt_mmio_read read, vgt_mmio_write write);
extern bool vgt_initialize_mmio_hooks(void);

struct vgt_irq_virt_state;
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

	struct vgt_irq_virt_state *irq_vstate;
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

struct vgt_irq_host_state;
/* per-device structure */
struct pgt_device {
	struct pci_bus *pbus;	/* parent bus of the device */
	struct pci_dev *pdev;	/* the gfx device bound to */
	int bus;		/* parent bus number */
	int devfn;		/* device function number */

	struct task_struct *p_thread;

	vgt_ringbuffer_t *ring_base_vaddr[MAX_ENGINES];	/* base vitrual address of ring buffer mmios */
	vgt_reg_t initial_mmio_state[VGT_MMIO_REG_NUM];	/* copy from physical at start */
	uint8_t initial_cfg_space[VGT_CFG_SPACE_SZ];	/* copy from physical at start */
	uint32_t bar_size[3];
	uint64_t gttmmio_base;	/* base of GTT and MMIO */
	void *gttmmio_base_va;	/* virtual base of mmio */
	uint64_t gtt_base;		/* base of GTT */
	void *gtt_base_va;	/* virtual base of GTT */
	uint64_t vgt_aperture_base;	/* aperture used for vGT driver itself */
	uint64_t gmadr_base;	/* base of GMADR */
	void *gmadr_va;	/* virtual base of GMADR */

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct list_head rendering_runq_head;	/* ??? */
	struct list_head rendering_idleq_head;	/* ??? */
	bool switch_inprogress;	/* an ownership switch in progress */
	enum vgt_owner_type switch_owner;	/* the type of the owner in switch */


	struct vgt_irq_host_state *irq_hstate;
};

#define vgt_get_owner(d, t)             (d->owner[t])
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_GT))
#define is_current_render_owner(vgt)	(vgt && vgt == current_render_owner(vgt->pdev))
#define current_display_owner(d)	((vgt_get_owner(d, VGT_OT_DISPLAY))->id)
#define vgt_switch_inprogress(d)        (d->switch_inprogress)
#define vgt_switch_owner_type(d)        (d->switch_owner)

static inline void __REG_WRITE(unsigned long preg, unsigned long val, int bytes)
{
	int ret;

	/* TODO: any license issue? */
	ret = hcall_mmio_write(preg, bytes, val);
	//ASSERT(ret == X86EMUL_OKAY);
}

static inline unsigned long __REG_READ(unsigned long preg, int bytes)
{
	unsigned long data;
	int ret;

	/* TODO: any license issue? */
	ret = hcall_mmio_read(preg, bytes, &data);
	//ASSERT(ret == X86EMUL_OKAY);

	return data;
}

#define VGT_MMIO_READ_BYTES(pdev, mmio_offset, bytes)	\
		__REG_READ(_vgt_mmio_pa(pdev, mmio_offset), bytes)

#define VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, val, bytes)	\
		__REG_WRITE(_vgt_mmio_pa(pdev, mmio_offset), val,  bytes)

#define VGT_MMIO_WRITE(pdev, mmio_offset, val)	\
		VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, (unsigned long)val, REG_SIZE)

#define VGT_MMIO_READ(pdev, mmio_offset)		\
		((vgt_reg_t)VGT_MMIO_READ_BYTES(pdev, mmio_offset, REG_SIZE))

#define VGT_MMIO_WRITE64(pdev, mmio_offset, val)	\
		__REG_WRITE(_vgt_mmio_pa(pdev, mmio_offset), val, 8)

#define VGT_MMIO_READ64(pdev, mmio_offset, val)		\
		__REG_READ(_vgt_mmio_pa(pdev, mmio_offset), 8)

#define vgt_restore_vreg(vgt, off)		\
	VGT_MMIO_WRITE(vgt->pdev, off, __vreg(vgt, off))

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

vgt_reg_t g2h_gmadr(struct vgt_device *vgt, vgt_reg_t g_gm_addr);
vgt_reg_t h2g_gmadr(struct vgt_device *vgt, vgt_reg_t h_gm_addr);

static inline bool is_ring_empty(struct pgt_device *pgt, int ring_id)
{
	vgt_reg_t head = VGT_MMIO_READ(pgt, RB_HEAD(ring_id));
	vgt_reg_t tail = VGT_MMIO_READ(pgt, RB_TAIL(ring_id));

	head &= RB_HEAD_OFF_MASK;
	/*
	 * FIXME: PRM said bit2-20 for head count, but bit3-20 for tail count
	 * however doing that makes tail always head/2.
	 */
	tail &= RB_HEAD_OFF_MASK;
	return (head == tail);
}

#define VGT_POST_READ(pdev, reg)		\
	do {					\
		vgt_reg_t val;			\
		val = VGT_MMIO_READ(pdev, reg);	\
	} while (0)

static inline bool is_ring_enabled (struct pgt_device *pgt, int ring_id)
{
	return (VGT_MMIO_READ(pgt, RB_CTL(ring_id)) & 1);	/* bit 0: enable/disable RB */
}

/* FIXME: use readl/writel as Xen doesn't trap GTT access now */
static inline u32 vgt_read_gtt(struct pgt_device *pdev, u32 index)
{
	//return VGT_MMIO_READ(pdev, pdev->gtt_base + index * GTT_ENTRY_SIZE);
	return readl((u32*)pdev->gtt_base_va + index);
}

static inline void vgt_write_gtt(struct pgt_device *pdev, u32 index, u32 val)
{
	//VGT_MMIO_WRITE(pdev, pdev->gtt_base + index * GTT_ENTRY_SIZE, val);
	writel(val, (u32*)pdev->gtt_base_va + index);
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

/* interrupt related definitions */
#define _REG_INVALID	0xFFFFFFFF
// register definitions in a format as _REG_REGNAME
#define _REG_DEISR	0x44000
#define _REG_DEIMR	0x44004
#define _REG_DEIIR	0x44008
#define _REG_DEIER	0x4400C
#define		_REGBIT_MASTER_INTERRUPT	(1 << 31)
#define _REG_GTISR	0x44010
#define _REG_GTIMR	0x44014
#define _REG_GTIIR	0x44018
#define _REG_GTIER	0x4401C
#define _REG_PMISR	0x44020
#define _REG_PMIMR	0x44024
#define _REG_PMIIR	0x44028
#define _REG_PMIER	0x4402C
#define _REG_DP_A_HOTPLUG_CNTL	0x44030
#define		_REGBIT_DP_A_HOTPLUG_STATUS		(3 << 0)
#define		_REGBIT_DP_A_PULSE_DURATION		(3 << 2)
#define		_REGBIT_DP_A_HOTPLUG_ENABLE		(1 << 4)
#define _REG_GTT_FAULT_STATUS	0x44040
#define		_REGBIT_PRIMARY_A_GTT_FAULT_STATUS	(1 << 0)
#define		_REGBIT_PRIMARY_B_GTT_FAULT_STATUS	(1 << 1)
#define		_REGBIT_SPRITE_A_GTT_FAULT_STATUS	(1 << 2)
#define		_REGBIT_SPRITE_B_GTT_FAULT_STATUS	(1 << 3)
#define		_REGBIT_CURSOR_A_GTT_FAULT_STATUS	(1 << 4)
#define		_REGBIT_CURSOR_B_GTT_FAULT_STATUS	(1 << 5)
#define		_REGBIT_INVALID_PGTABLE_ENTRY_DATA	(1 << 6)
#define		_REGBIT_INVALID_GTT_PGTABLE_ENTRY	(1 << 7)

#define	_REG_SDEISR	0xC4000
#define	_REG_SDEIMR	0xC4004
#define	_REG_SDEIIR	0xC4008
#define	_REG_SDEIER	0xC400C
#define _REG_SHOTPLUG_CTL	0xC4030
#define		_REGBIT_DP_B_STATUS			(3 << 0)
#define		_REGBIT_DP_B_PULSE_DURATION		(3 << 2)
#define		_REGBIT_DP_B_ENABLE			(1 << 4)
#define		_REGBIT_DP_C_STATUS			(3 << 8)
#define		_REGBIT_DP_C_PULSE_DURATION		(3 << 10)
#define		_REGBIT_DP_C_ENABLE			(1 << 12)
#define		_REGBIT_DP_D_STATUS			(3 << 16)
#define		_REGBIT_DP_D_PULSE_DURATION		(3 << 18)
#define		_REGBIT_DP_D_ENABLE			(1 << 20)

#define vgt_isr(base)	(base)
#define vgt_imr(base)	(base + 0x4)
#define vgt_iir(base)	(base + 0x8)
#define vgt_ier(base)	(base + 0xC)

#define vgt_clear_reg_bit(pdev, reg, bit)		\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val &= ~(1 << bit);			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
	} while (0)

#define vgt_set_reg_bit(pdev, reg, bit)			\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val |= 1 << bit;			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
	} while (0)

#define _REG_RDR_HWSTAM		0x2098
#define _REG_VIDEO_HWSTAM	0x12098
#define _REG_BLIT_HWSTAM	0x22098
#define _REG_RDR_IMR		0x20A8
#define _REG_VIDEO_IMR		0x120A8
#define _REG_BLIT_IMR		0x220A8

#define _REG_RDR_WATCHDOG_CTL	0x2178
#define _REG_RDR_WATCHDOG_THRSH	0x217C
#define _REG_RDR_WATCHDOG_CTR	0x2190
#define _REG_VIDEO_WATCHDOG_CTR	0x12178
#define _REG_VIDEO_WATCHDOG_THRSH	0x1217C

#define _REG_RDR_EIR	0x20B0
#define _REG_RDR_EMR	0x20B4
#define _REG_RDR_ESR	0x20B8
#define _REG_BLIT_EIR	0x220B0
#define _REG_BLIT_EMR	0x220B4
#define _REG_BLIT_ESR	0x220B8
/* interesting no definitiont about video error information. */
//#define _REG_VIDEO_EIR	0x20B0
//#define _REG_VIDEO_EMR	0x20B4
//#define _REG_VIDEO_ESR	0x20B8

/* blacklight PWM control */
#define _REG_BLC_PWM_CTL2	0x48250
#define		_REGBIT_PHASE_IN_IRQ_ENABLE	(1 << 24)
#define		_REGBIT_PHASE_IN_IRQ_STATUS	(1 << 26)
#define _REG_HISTOGRAM_THRSH	0x48268
#define		_REGBIT_HISTOGRAM_IRQ_ENABLE	(1 << 31)
#define		_REGBIT_HISTOGRAM_IRQ_STATUS	(1 << 30)

enum vgt_event_type {

	// GT
	IRQ_RDR_MI_USER_INTERRUPT = 0,
	IRQ_RDR_DEBUG,
	IRQ_RDR_MMIO_SYNC_FLUSH,
	IRQ_RDR_CMD_STREAMER_ERR,
	IRQ_RDR_PIPE_CONTROL,
	IRQ_RDR_WATCHDOG_EXCEEDED,
	IRQ_RDR_PAGE_DIRECTORY_FAULT,
	IRQ_RDR_AS_CONTEXT_SWITCH,

	IRQ_VIDEO_MI_USER_INTERRUPT,
	IRQ_VIDEO_MMIO_SYNC_FLUSH,
	IRQ_VIDEO_CMD_STREAMER_ERR,
	IRQ_VIDEO_MI_FLUSH_DW,
	IRQ_VIDEO_WATCHDOG_EXCEEDED,
	IRQ_VIDEO_PAGE_DIRECTORY_FAULT,
	IRQ_VIDEO_AS_CONTEXT_SWITCH,

	IRQ_BLIT_MI_USER_INTERRUPT,
	IRQ_BLIT_MMIO_SYNC_FLUSH,
	IRQ_BLIT_CMD_STREAMER_ERR,
	IRQ_BLIT_MI_FLUSH_DW,
	IRQ_BLIT_PAGE_DIRECTORY_FAULT,
	IRQ_BLIT_AS_CONTEXT_SWITCH,

	// DISPLAY
	IRQ_PIPE_A_FIFO_UNDERRUN,
	IRQ_PIPE_A_CRC_ERR,
	IRQ_PIPE_A_CRC_DONE,
	IRQ_PIPE_A_VSYNC,
	IRQ_PIPE_A_LINE_COMPARE,
	IRQ_PIPE_A_ODD_FIELD,
	IRQ_PIPE_A_EVEN_FIELD,
	IRQ_PIPE_A_VBLANK,
	IRQ_PIPE_B_FIFO_UNDERRUN,	// This is an active high level for the duration of the Pipe B FIFO underrun
	IRQ_PIPE_B_CRC_ERR,	// This is an active high pulse on the Pipe B CRC error
	IRQ_PIPE_B_CRC_DONE,	// This is an active high pulse on the Pipe B CRC done
	IRQ_PIPE_B_VSYNC,	// This is an active high level for the duration of the Pipe B vertical sync
	IRQ_PIPE_B_LINE_COMPARE,	// This is an active high level for the duration of the selected Pipe B scan lines
	IRQ_PIPE_B_ODD_FIELD,	// This is an active high level for the duration of the Pipe B interlaced odd field
	IRQ_PIPE_B_EVEN_FIELD,	// This is an active high level for the duration of the Pipe B interlaced even field
	IRQ_PIPE_B_VBLANK,	// This is an active high level for the duration of the Pipe B vertical blank
	IRQ_DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	IRQ_DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	IRQ_GSE,
	IRQ_DP_A_HOTPLUG,
	IRQ_AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	IRQ_PCH_IRQ,	// Only the rising edge of the PCH Display interrupt will cause the IIR to be set here
	IRQ_PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	IRQ_POISON,		// This is an active high pulse on receiving the poison message
	IRQ_GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	IRQ_PRIMARY_A_FLIP_DONE,
	IRQ_PRIMARY_B_FLIP_DONE,	// This is an active high pulse when a primary plane B flip is done
	IRQ_SPRITE_A_FLIP_DONE,
	IRQ_SPRITE_B_FLIP_DONE,	// This is an active high pulse when a sprite plane B flip is done

	// PM
	IRQ_GV_DOWN_INTERVAL,
	IRQ_GV_UP_INTERVAL,
	IRQ_RP_DOWN_THRESHOLD,
	IRQ_RP_UP_THRESHOLD,
	IRQ_FREQ_DOWNWARD_TIMEOUT_RC6,
	IRQ_PCU_THERMAL,
	IRQ_PCU_PCODE2DRIVER_MAILBOX,

	// PCH
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_A,	// This is an active high level indicating content protection is requested by audio azalia verb programming for transcoder A
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_B,
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_B,
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_B,
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_C,
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_C,
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_C,
	IRQ_ERR_AND_DBG,
	IRQ_GMBUS,
	IRQ_SDVO_B_HOTPLUG,
	IRQ_CRT_HOTPLUG,
	IRQ_DP_B_HOTPLUG,
	IRQ_DP_C_HOTPLUG,
	IRQ_DP_D_HOTPLUG,
	IRQ_AUX_CHENNEL_B,
	IRQ_AUX_CHENNEL_C,
	IRQ_AUX_CHENNEL_D,
	IRQ_AUDIO_POWER_STATE_CHANGE_B,
	IRQ_AUDIO_POWER_STATE_CHANGE_C,
	IRQ_AUDIO_POWER_STATE_CHANGE_D,

	IRQ_RESERVED,
	IRQ_MAX,
};

#define VGT_FIRST_RDR_EVENT	IRQ_RDR_MI_USER_INTERRUPT
#define VGT_LAST_RDR_EVENT	IRQ_RDR_AS_CONTEXT_SWITCH
#define VGT_RDR_EVENT(e)	(e >= VGT_FIRST_RDR_EVENT && e <= VGT_LAST_RDR_EVENT)

#define VGT_FIRST_VIDEO_EVENT	IRQ_VIDEO_MI_USER_INTERRUPT
#define VGT_LAST_VIDEO_EVENT	IRQ_VIDEO_AS_CONTEXT_SWITCH
#define VGT_VIDEO_EVENT(e)	(e >= IRQ_VIDEO_MI_USER_INTERRUPT && e <= IRQ_VIDEO_AS_CONTEXT_SWITCH)

#define VGT_FIRST_BLIT_EVENT	IRQ_BLIT_MI_USER_INTERRUPT
#define VGT_LAST_BLIT_EVENT	IRQ_BLIT_AS_CONTEXT_SWITCH
#define VGT_BLIT_EVENT(e)	(e >= IRQ_BLIT_MI_USER_INTERRUPT && e <= IRQ_BLIT_AS_CONTEXT_SWITCH)

#define VGT_FIRST_GT_EVENT	VGT_FIRST_RDR_EVENT
#define VGT_LAST_GT_EVENT	VGT_LAST_BLIT_EVENT
#define VGT_GT_EVENT(e)		(e >= VGT_FIRST_GT_EVENT && e <= IRQ_BLIT_AS_CONTEXT_SWITCH)

#define VGT_FIRST_DPY_EVENT	IRQ_PIPE_A_FIFO_UNDERRUN
#define VGT_LAST_DPY_EVENT	IRQ_SPRITE_B_FLIP_DONE
#define VGT_DPY_EVENT(e)	(e >= VGT_FIRST_DPY_EVENT && e <= VGT_LAST_DPY_EVENT)

#define VGT_FIRST_PM_EVENT	IRQ_GV_DOWN_INTERVAL
#define VGT_LAST_PM_EVENT	IRQ_PCU_PCODE2DRIVER_MAILBOX
#define VGT_PM_EVENT(e)		(e >= VGT_FIRST_PM_EVENT && e <= VGT_LAST_PM_EVENT)

#define VGT_FIRST_PCH_EVENT	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A
#define VGT_LAST_PCH_EVENT	IRQ_AUDIO_POWER_STATE_CHANGE_D
#define VGT_PCH_EVENT(e)	(e >= VGT_FIRST_PCH_EVENT && e <= VGT_LAST_PCH_EVENT)

#define	VGT_IRQ_BITWIDTH	32

enum vgt_irq_action_type {
	VGT_IRQ_HANDLE_FULL,		/* both clear pReg and update vReg */
	VGT_IRQ_HANDLE_PHYSICAL,	/* only clear pReg, in case a context switch in progress */
	VGT_IRQ_HANDLE_VIRTUAL,		/* only update vReg, in a post action */
};

struct vgt_irq_info_entry;
struct vgt_irq_info;

typedef void (*vgt_core_handler_t)(struct pgt_device *dev, enum vgt_event_type event);

typedef void (*vgt_event_handler_t)(
	struct pgt_device *dev,
	int bit,
	struct vgt_irq_info_entry *entry,
	struct vgt_irq_info *info,
	enum vgt_irq_action_type action);

typedef void (*vgt_emulate_handler_t)(struct vgt_device *vstate, enum vgt_event_type event, bool enable);

struct vgt_irq_info_entry {
	enum vgt_event_type event;
	vgt_event_handler_t event_handler;
	vgt_emulate_handler_t emul_handler;
};

/* per-device level-1 interrupt bit definitions */
struct vgt_irq_info {
	char *name;
	int reg_base;
	int table_size;
	struct vgt_irq_info_entry table[VGT_IRQ_BITWIDTH];
};

#define VGT_DPY_EMUL_PERIOD	500000000	// 500ms for now

struct vgt_irq_ops {
	void (*init) (struct pgt_device *dev);

	void (*exit) (struct pgt_device *dev);

	irqreturn_t (*interrupt) (struct pgt_device *dev);

	void (*toggle_hw_event) (struct pgt_device *dev,
			enum vgt_event_type event, int bit, bool enable);

	void (*save) (struct vgt_device *vstate,
			enum vgt_owner_type type);

	void (*restore) (struct vgt_device *vstate,
			enum vgt_owner_type type);

	enum vgt_event_type (*get_event_type_from_bit) (
			struct pgt_device *dev, uint32_t reg, uint32_t bit);

	int (*get_bit_from_event) (struct pgt_device *dev,
			enum vgt_event_type event, struct vgt_irq_info *info);

	struct vgt_irq_info *(*get_irq_info_from_event) (
			struct pgt_device *dev, enum vgt_event_type event);
};

/* structure containing device specific IRQ state */
struct vgt_irq_host_state {
	struct pgt_device *pdev;
	/*
	 * ownership table for each IRQ event. a default table is defined
	 * but we hope it be instance specific so that it's flexible enough
	 */
	enum vgt_owner_type *event_owner_table;
	/* vGT core event handler table */
	vgt_core_handler_t *core_handlers;
	/* always emulated events */
	DECLARE_BITMAP(emulated_events, IRQ_MAX);
	/* a list of delayed events */
	DECLARE_BITMAP(delayed_events, IRQ_MAX);
	/* the one which delayed events target at */
	struct vgt_device *delayed_owner;
	spinlock_t lock;

	struct vgt_irq_ops *ops;

	int i915_irq;
	int pirq;
};

struct vgt_emul_timer {
	struct hrtimer timer;
	DECLARE_BITMAP(events, IRQ_MAX);
	u64 period;
};

struct vgt_watchdog_timer {
	struct hrtimer timer;
};

struct vgt_device;
/* structure containing instance specific IRQ state */
struct vgt_irq_virt_state {
	struct vgt_device *vgt;
	DECLARE_BITMAP(emulated_events, IRQ_MAX);
	DECLARE_BITMAP(enabled_events, IRQ_MAX);
	struct vgt_emul_timer dpy_timer;
	struct vgt_watchdog_timer watchdog_timer;
	bool irq_pending;
	bool pch_irq_pending;
};

#define vgt_event_owner_table(d)	(d->irq_hstate->event_owner_table)
#define vgt_get_event_owner_type(d, e)	(d->irq_hstate->event_owner_table[e])
#define vgt_core_event_handlers(d)	(d->irq_hstate->core_handlers)
#define vgt_always_emulated_events(d)	(d->irq_hstate->emulated_events)
#define vgt_delayed_events(d)		(d->irq_hstate->delayed_events)
#define vgt_delayed_owner(d)		(d->irq_hstate->delayed_owner)
#define vgt_get_irq_ops(d)		(d->irq_hstate->ops)
#define vgt_i915_irq(d)			(d->irq_hstate->i915_irq)
#define vgt_pirq(d)			(d->irq_hstate->pirq)

#define vgt_get_id(s)			(s->vgt_id)
#define vgt_state_emulated_events(s)		(s->irq_vstate->emulated_events)
#define vgt_state_enabled_events(s)	(s->irq_vstate->enabled_events)
#define vgt_dpy_timer(s)		(s->irq_vstate->dpy_timer)

#define vgt_event_owner_is_core(d, e)	\
	(vgt_core_event_handlers(d) && (vgt_core_event_handlers(d))[e])
#define vgt_get_event_owner(d, e)	\
	(vgt_get_owner(d, vgt_get_event_owner_type(d, e)))
#define vgt_event_owner_is_null(d, e)	\
	!vgt_event_owner_is_core(d, e) && !vgt_get_event_owner(d, e)
#define vgt_get_inject_event_owner(d, e, a)	\
	((a) == VGT_IRQ_HANDLE_VIRTUAL ? vgt_delayed_owner(d) : vgt_get_event_owner(d, e))

#define vgt_core_event_handler(d, e)	\
	((vgt_core_event_handlers(d))[e])

#ifdef VGT_DEBUG
#define vgt_trace_irq_event(i, t)	\
	printk("vGT (%s): catch event (%s) in reg (%x).",	\
		(i)->name, vgt_irq_name[(t)],			\
		vgt_iir((i)->reg_base))
#else
#define vgt_trace_irq_event(i, t)
#endif

extern uint8_t vgt_irq_warn_once[IRQ_MAX];
#define VGT_IRQ_WARN(i, t, msg)					\
	printk("!!!vGT (%s): event (%s) in reg (%x): " msg,	\
		(i)->name, vgt_irq_name[(t)],			\
		vgt_iir((i)->reg_base))
#define VGT_IRQ_WARN_ONCE(i, t, msg)			\
	do {						\
		if (!vgt_irq_warn_once[(t)]) { 		\
			vgt_irq_warn_once[(t)] = 1;	\
			VGT_IRQ_WARN(i, t, msg);	\
		}					\
	} while (0);

/* FIXME: IIR can handle 2 pending requests */
static inline void vgt_set_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->irq_pending = true;
}

static inline void vgt_clear_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->irq_pending = false;
}

static inline bool vgt_has_irq_pending(struct vgt_device *vstate)
{
	return vstate->irq_vstate->irq_pending;
}

/* propogation to DE is postponed since there may have multiple PCH events pending */
static inline void vgt_set_pch_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->pch_irq_pending = true;
}

static inline void vgt_clear_pch_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->pch_irq_pending = false;
}

static inline bool vgt_has_pch_irq_pending(struct vgt_device *vstate)
{
	return vstate->irq_vstate->pch_irq_pending;
}

/*
 * assumptions:
 *   - rising edge to trigger an event to next level
 *   - only cache one instance for IIR now
 *
 * FIXME: any race condition to consider here? e.g. guest write to IER/IMR right
 * at the propogation?
 */
static inline void vgt_propogate_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info)
{
	if (!test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base))) &&
	    !test_bit(bit, (void*)vgt_vreg(vstate, vgt_imr(info->reg_base))) &&
	    !test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))) &&
	    test_bit(bit, (void*)vgt_vreg(vstate, vgt_ier(info->reg_base))) &&
	    test_bit(_REGBIT_MASTER_INTERRUPT, (void*)vgt_vreg(vstate, _REG_DEIER)))
		vgt_set_irq_pending(vstate);
}

/*
 * propogate PCH specific event, which will be chained to level-1 ISR later
 * similarly need consider IIR which can store two pending instances
 */
static inline void vgt_propogate_pch_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info)
{
	if (!test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_isr(info->reg_base))) &&
	    !test_bit(bit, (void*)vgt_vreg(vstate, vgt_imr(info->reg_base))) &&
	    !test_and_set_bit(bit, (void*)vgt_vreg(vstate, vgt_iir(info->reg_base))) &&
	    test_bit(bit, (void*)vgt_vreg(vstate, vgt_ier(info->reg_base))))
		vgt_set_pch_irq_pending(vstate);
}

/*
 * FIXME: need to handle PCH propogation. Also it'd be good to share
 * same handler as in physical interrupt path, since this can only
 * handle IIR-only events.
 */
static inline void vgt_propogate_emulated_event(struct vgt_device *vstate,
	enum vgt_event_type event)
{
	int bit;
	struct pgt_device *dev = vstate->pdev;
	struct vgt_irq_info *info;
	struct vgt_irq_info_entry *entry;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(dev);

	info = ops->get_irq_info_from_event(dev, event);
	bit = ops->get_bit_from_event(dev, event, info);
	entry = info->table + bit;
	ASSERT(entry->event == event);
	vgt_propogate_virtual_event(vstate, bit, info);
}

/*
 * Note. clear physical pending bit, and then forward to virtual
 * register which includes more bits other than interrupt pending
 * bit
 */
#define vgt_forward_and_clear_bit(state, reg, bit)		\
	do {							\
		uint32_t val;					\
		val = vgt_read32(reg);				\
		val &= ~(bit);					\
		vgt_write32(reg, val);				\
		*vgt_vreg(state, reg) |= bit;			\
	} while (0);

/* for IIR type registers */
#define vgt_forward_and_clear_reg(state, reg)			\
	do {							\
		uint32_t val;					\
		val = vgt_read32(reg);				\
		vgt_write32(val);				\
		*vgt_vreg(state, reg) = val;			\
	} while (0)

extern struct vgt_irq_ops snb_irq_ops;
int hvm_inject_virtual_interrupt(struct vgt_device *vstate);
int initdom_inject_virtual_interrupt(struct vgt_device *vstate);
void vgt_setup_irq(int pirq);

void vgt_irq_handle_event(struct pgt_device *dev,
	void *iir, struct vgt_irq_info *info);
void vgt_default_event_handler(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_chained_pch_events(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_unexpected_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_host_only_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_weak_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_cmd_stream_error(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_phase_in(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_histogram(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_aux_channel(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);
void vgt_handle_gmbus(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	enum vgt_irq_action_type action);

void vgt_emulate_watchdog(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
void vgt_emulate_dpy_status(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
void vgt_reg_imr_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
void vgt_reg_ier_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);

#include "vgt_wr.h"
#define emulated_regs_t	i915emuRegs_t
extern emulated_regs_t	gpuregs[];

#endif	/* _VGT_REG_H_ */
