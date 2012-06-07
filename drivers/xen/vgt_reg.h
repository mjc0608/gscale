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
#include <linux/sched.h>
#include <linux/wait.h>
#include <xen/interface/hvm/ioreq.h>

//#define SINGLE_VM_DEBUG
#define SANDY_BRIDGE
#define ASSERT(x)   do { if (!(x)) {printk("Assert at %s line %d\n", __FILE__, __LINE__); BUG();}} while (0);
#define ASSERT_NUM(x,y) do { if (!(x)) {printk("Assert at %s line %d para %llx\n", __FILE__, __LINE__, (u64)y); BUG();}} while (0);

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

/* Maximum VMs supported by vGT. Actual number is device specific */
#define VGT_MAX_VMS			2
#define VGT_RSVD_APERTURE_SZ		(64*SIZE_1MB)	/* reserve 64MB for vGT itself */
#define VGT_DOM0_APERTURE_SZ		(64*SIZE_1MB)	/* 64MB for dom0 */
#define VGT_MIN_APERTURE_SZ		(128*SIZE_1MB)	/* minimum 128MB for other VMs */
/* only 1G/2G is supported on SNB. Need a way to enlighten the driver */
#define VGT_MIN_GM_SZ			(256*SIZE_1MB)	/* the power of 2 */

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*64)	/* use 64 KB for now */
#define VGT_APERTURE_PER_INSTANCE_SZ		(4*SIZE_1MB)	/* 4MB per instance (?) */
extern unsigned long vgt_id_alloc_bitmap;
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#define REG_SIZE    		sizeof(vgt_reg_t)        /* size of gReg/sReg[0] */
#define REG_INDEX(reg)		((reg) / REG_SIZE)
#define VGT_MMIO_SPACE_SZ	(2*SIZE_1MB)
#define VGT_MMIO_REG_NUM	(VGT_MMIO_SPACE_SZ/REG_SIZE)	/* 2MB space in totoal */
#define VGT_CFG_SPACE_SZ	256
#define VGT_BAR_NUM		4
typedef struct {
    int		regNum;		/* Total number of MMIO registers in vGT */
    uint64_t    mmio_base_gpa;	/* base guest physical address of the MMIO registers */
    vgt_reg_t	*vReg;		/* guest view of the register state */
    vgt_reg_t	*sReg;		/* Shadow (used by hardware) state of the register */
    uint8_t	cfg_space[VGT_CFG_SPACE_SZ];
	bool	bar_mapped[VGT_BAR_NUM];
    uint64_t	gt_mmio_base;	/* bar0/GTTMMIO  */
    uint64_t	aperture_base;	/* bar1: guest aperture base */
//    uint64_t	gt_gmadr_base;	/* bar1/GMADR */

    uint32_t	bar_size[VGT_BAR_NUM];	/* 0: GTTMMIO, 1: GMADR, 2: PIO bar size */

    /* FIXME: take them as part of vReg/sReg ??? */
    /* save indexed MMIO */
    uint8_t saveSR[8];  /* sequencer data register */
    uint8_t saveGR[25]; /* CRT controller register */
    uint8_t saveAR[21];
    uint8_t saveCR[37];
} vgt_state_t;

#define __vreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))
#define __vreg8(vgt, off) (*(char *)((char *)vgt->state.vReg + off))
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.sReg + off))
#define __sreg8(vgt, off) (*(char *)((char *)vgt->state.sReg + off))
#define __vreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.vReg + off))
#define __sreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.sReg + off))
#define vgt_vreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.vReg + off))
#define vgt_sreg(vgt, off)	((vgt_reg_t *)((char *)vgt_>state.sReg + off))

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
	bool	stateless;	/* whether the engine requires special context switch */
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

#define _REG_INVALID	0xFFFFFFFF

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
#define _RING_CTL_BUF_SIZE(ctl)	(((ctl) & RB_TAIL_SIZE_MASK) + GTT_PAGE_SIZE)
#define _RING_CTL_ENABLE	0x1	/* bit 0 */

#define _REG_CCID		0x02180
#define CCID_MBO_BITS		0x100		/* bit 8 must be one */
#define CCID_EXTENDED_STATE_SAVE_ENABLE		0x8
#define CCID_EXTENDED_STATE_RESTORE_ENABLE	0x4
#define CCID_VALID		0x1
#define CCID_TIMEOUT_LIMIT	150

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
#define _REG_GT_MODE	0x20D0

#define _REG_RCS_IMR		0x20A8
#define _REG_VCS_IMR		0x120A8
#define _REG_BCS_IMR		0x220A8

#define _REG_RCS_BB_ADDR	0x2140
#define _REG_VCS_BB_ADDR	0x12140
#define _REG_BCS_BB_ADDR	0x22140

#define _REG_RCS_HWS_PGA	0x4080
#define _REG_VCS_HWS_PGA	0x4180
#define _REG_BCS_HWS_PGA	0x24080
#define _REG_IVB_BCS_HWS_PGA	0x4280

#define _REG_RCS_INSTPM		0x20C0
#define _REG_VCS_INSTPM		0x120C0
#define _REG_BCS_INSTPM		0x220C0
#define INSTPM_CONS_BUF_ADDR_OFFSET_DIS (1<<6)

#define _REG_RCS_EXCC		0x2028
#define _REG_VCS_EXCC		0x12028
#define _REG_BCS_EXCC		0x22028

#define _REG_RCS_UHPTR		0x2134
#define _REG_VCS_UHPTR		0x12134
#define _REG_BCS_UHPTR		0x22134

#define _REG_RCS_ACTHD		0x2074
#define _REG_VCS_ACTHD		0x12074
#define _REG_BCS_ACTHD		0x22074

#define _REG_RCS_HWSTAM		0x2098
#define _REG_VCS_HWSTAM		0x12098
#define _REG_BCS_HWSTAM		0x22098

#define _REG_RCS_BB_PREEMPT_ADDR	0x2148

#define _REG_RCS_BB_ADDR_DIFF		0x2154
#define _REG_RCS_BB_OFFSET		0x2158
#define _REG_RCS_FBC_RT_BASE_ADDR	0x2128
#define _REG_IVB_RCS_FBC_RT_BASE_ADDR	0X7020

#define _REG_IVB_RCS_PP_DIR_BASE	0x2228
#define _REG_RCS_PP_DIR_BASE_READ	0x2518
#define _REG_RCS_PP_DIR_BASE_WRITE	0x2228
#define _REG_VCS_PP_DIR_BASE		0x12228
#define _REG_BCS_PP_DIR_BASE		0x22228

#define _REG_FENCE_0_LOW	0x100000
#define _REG_FENCE_0_HIGH	0x100004
#define _REG_FENCE_1_LOW	0x100008
#define _REG_FENCE_1_HIGH	0x10000C
#define _REG_FENCE_2_LOW	0x100010
#define _REG_FENCE_2_HIGH	0x100014
#define _REG_FENCE_3_LOW	0x100018
#define _REG_FENCE_3_HIGH	0x10001C
#define _REG_FENCE_4_LOW	0x100020
#define _REG_FENCE_4_HIGH	0x100024
#define _REG_FENCE_5_LOW	0x100028
#define _REG_FENCE_5_HIGH	0x10002C
#define _REG_FENCE_6_LOW	0x100030
#define _REG_FENCE_6_HIGH	0x100034
#define _REG_FENCE_7_LOW	0x100038
#define _REG_FENCE_7_HIGH	0x10003C
#define _REG_FENCE_8_LOW	0x100040
#define _REG_FENCE_8_HIGH	0x100044
#define _REG_FENCE_9_LOW	0x100048
#define _REG_FENCE_9_HIGH	0x10004C
#define _REG_FENCE_10_LOW	0x100050
#define _REG_FENCE_10_HIGH	0x100054
#define _REG_FENCE_11_LOW	0x100058
#define _REG_FENCE_11_HIGH	0x10005C
#define _REG_FENCE_12_LOW	0x100060
#define _REG_FENCE_12_HIGH	0x100064
#define _REG_FENCE_13_LOW	0x100068
#define _REG_FENCE_13_HIGH	0x10006C
#define _REG_FENCE_14_LOW	0x100070
#define _REG_FENCE_14_HIGH	0x100074
#define _REG_FENCE_15_LOW	0x100078
#define _REG_FENCE_15_HIGH	0x10007C
#define 	_REGBIT_FENCE_VALID	(1 << 0)

#define _REG_CURACNTR		0x70080
#define _REG_CURABASE		0x70084
#define _REG_CURAPOS		0x70088
#define _REG_CURAVGAPOPUPBASE	0x7008C
#define _REG_CURAPALET_0	0x70090
#define _REG_CURAPALET_1	0x70094
#define _REG_CURAPALET_2	0x70098
#define _REG_CURAPALET_3	0x7009C
#define _REG_CURASURFLIVE	0x700AC

#define _REG_CURBCNTR		0x700C0
#define _REG_CURBBASE		0x700C4
#define _REG_CURBPOS		0x700C8
#define _REG_CURBPALET_0	0x700D0
#define _REG_CURBPALET_1	0x700D4
#define _REG_CURBPALET_2	0x700D8
#define _REG_CURBPALET_3	0x700DC
#define _REG_CURBSURFLIVE	0x700EC

#define _REG_DSPACNTR		0x70180
#define _REG_DSPALINOFF		0x70184
#define _REG_DSPASTRIDE		0x70188
#define _REG_DSPAPOS		0x7018C /* reserved */
#define _REG_DSPASIZE		0x70190
#define _REG_DSPASURF		0x7019C
#define _REG_DSPATILEOFF	0x701A4
#define _REG_DSPASURFLIVE	0x701AC

#define _REG_DSPBCNTR		0x71180
#define _REG_DSPBLINOFF		0x71184
#define _REG_DSPBSTRIDE		0x71188
#define _REG_DSPBPOS		0x7118C
#define _REG_DSPBSIZE		0x71190
#define _REG_DSPBSURF		0x7119C
#define _REG_DSPBTILEOFF	0x701A4
#define _REG_DSPBSURFLIVE	0x711AC

#define _REG_DVSACNTR		0x72180
#define _REG_DVSALINOFF		0x72184
#define _REG_DVSASTRIDE		0x72188
#define _REG_DVSAPOS		0x7218C
#define _REG_DVSASIZE		0x72190
#define _REG_DVSAKEYVAL		0x72194
#define _REG_DVSAKEYMSK		0x72198
#define _REG_DVSASURF		0x7219C
#define _REG_DVSAKEYMAXVAL	0x721A0
#define _REG_DVSATILEOFF	0x721A4
#define _REG_DVSASURFLIVE	0x721AC
#define _REG_DVSASCALE		0x72204
/* DVSAGAMC: 0x72300 - 0x7234B */

#define _REG_DVSBCNTR		0x73180
#define _REG_DVSBLINOFF		0x73184
#define _REG_DVSBSTRIDE		0x73188
#define _REG_DVSBPOS		0x7318C
#define _REG_DVSBSIZE		0x73190
#define _REG_DVSBKEYVAL		0x73194
#define _REG_DVSBKEYMSK		0x73198
#define _REG_DVSBSURF		0x7319C
#define _REG_DVSBKEYMAXVAL	0x731A0
#define _REG_DVSBTILEOFF	0x731A4
#define _REG_DVSBSURFLIVE	0x731AC
#define _REG_DVSBSCALE		0x73204
/* DVSBGAMC: 0x73300 - 0x7334B */

#define _REG_PCH_DPB_AUX_CH_CTL 0xe4110
#define _REG_PCH_DPC_AUX_CH_CTL 0xe4210
#define _REG_PCH_DPD_AUX_CH_CTL 0xe4310
#define _REGBIT_DP_AUX_CH_CTL_SEND_BUSY (1 << 31)
#define _REGBIT_DP_AUX_CH_CTL_DONE (1 << 30)
#define _REGBIT_DP_AUX_CH_CTL_INTERRUPT (1 << 29)
#define _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR (1 << 28)
#define _REGBIT_DP_AUX_CH_CTL_RECV_ERR (1 << 25)

#define _REG_FORCEWAKE		0xA18C
#define _REG_FORCEWAKE_ACK	0x130090
#define _REG_GT_THREAD_STATUS  0x13805C
#define _REG_GT_CORE_STATUS  0x138060

#define MI_NOOP				0
#define MI_FLUSH			(0x4 << 23)
#define MI_SUSPEND_FLUSH		(0xb << 23)
#define 	MI_SUSPEND_FLUSH_EN		(1<<0)
#define MI_SET_CONTEXT			(0x18 << 23)
#define 	MI_MM_SPACE_GTT			(1<<8)
#define 	MI_MM_SPACE_PHYSICAL		(0<<8)
#define 	MI_SAVE_EXT_STATE_EN		(1<<3)
#define 	MI_RESTORE_EXT_STATE_EN		(1<<2)
#define  	MI_FORCE_RESTORE		(1<<1)
#define 	MI_RESTORE_INHIBIT		(1<<0)
/*
 * We use _IMM instead of _INDEX, to avoid switching hardware
 * status page
 */
#define MI_STORE_DATA_IMM		((0x20 << 23) | 2)
#define MI_STORE_DATA_IMM_QWORD		((0x20 << 23) | 3)
#define		MI_SDI_USE_GTT		(1<<22)

/* PCI config space */
#define _REG_LBB	0xf4

/* VGA stuff */
#define _REG_VGA_MSR_WRITE 0x3c2
#define _REG_VGA_MSR_READ 0x3cc
#define   VGA_MSR_CGA_MODE (1<<0)

#define _REG_VGA_CR_INDEX_MDA 0x3b4
#define _REG_VGA_CR_DATA_MDA 0x3b5
#define _REG_VGA_ST01_MDA 0x3ba

#define _REG_VGA_CR_INDEX_CGA 0x3d4
#define _REG_VGA_CR_DATA_CGA 0x3d5
#define _REG_VGA_ST01_CGA 0x3da

#define _REG_VGA_SR_INDEX 0x3c4
#define _REG_VGA_SR_DATA 0x3c5

#define _REG_VGA_GR_INDEX 0x3ce
#define _REG_VGA_GR_DATA 0x3cf

#define _REG_VGA_AR_INDEX 0x3c0
#define _REG_VGA_AR_DATA_WRITE 0x3c0
#define _REG_VGA_AR_DATA_READ 0x3c1

#define _REG_VGA_DACMASK 0x3c6
/*
 * Display engine regs
 */

/* Pipe A timing regs */
#define _REG_HTOTAL_A	0x60000
#define _REG_HBLANK_A	0x60004
#define _REG_HSYNC_A		0x60008
#define _REG_VTOTAL_A	0x6000c
#define _REG_VBLANK_A	0x60010
#define _REG_VSYNC_A		0x60014
#define _REG_PIPEASRC	0x6001c
#define _REG_BCLRPAT_A	0x60020

/* Pipe B timing regs */
#define _REG_HTOTAL_B	0x61000
#define _REG_HBLANK_B	0x61004
#define _REG_HSYNC_B		0x61008
#define _REG_VTOTAL_B	0x6100c
#define _REG_VBLANK_B	0x61010
#define _REG_VSYNC_B		0x61014
#define _REG_PIPEBSRC	0x6101c
#define _REG_BCLRPAT_B	0x61020

#define _REG_DISP_ARB_CTL	0x45000

/* PCH */
#define _REG_PCH_DREF_CONTROL        0xC6200

/*
 * digital port hotplug
 */
#define _REG_PCH_DPLL_A              0xc6014
#define _REG_PCH_DPLL_B              0xc6018

#define   DPLL_VCO_ENABLE		(1 << 31)

#define _REG_PCH_FPA0                0xc6040
#define  FP_CB_TUNE		(0x3<<22)
#define _REG_PCH_FPA1                0xc6044
#define _REG_PCH_FPB0                0xc6048
#define _REG_PCH_FPB1                0xc604c


/*
 * Clock control & power management
 */
#define _REG_VGA0	0x6000
#define _REG_VGA1	0x6004
#define _REG_VGA_PD	0x6010
/* FIXME: PIO ?? */
#define _REG_DPLL_A	0x06014
#define _REG_DPLL_B	0x06018

/* refresh rate hardware control */
#define _REG_PIPEA_DATA_M1           0x60030
#define _REG_PIPEA_DATA_N1           0x60034
#define _REG_PIPEA_LINK_M1           0x60040
#define _REG_PIPEA_LINK_N1           0x60044

/* PIPEB timing regs are same start from 0x61000 */
#define _REG_PIPEB_DATA_M1           0x61030
#define _REG_PIPEB_DATA_N1           0x61034
#define _REG_PIPEB_LINK_M1           0x61040
#define _REG_PIPEB_LINK_N1           0x61044

/* VGA port control */
#define _REG_ADPA			0x61100

/* FDI_RX, FDI_X is hard-wired to Transcoder_X */
#define _REG_FDI_RXA_CTL             0xf000c
#define _REG_FDI_RXB_CTL             0xf100c

/* CPU: FDI_TX */
#define _REG_FDI_TXA_CTL             0x60100
#define _REG_FDI_TXB_CTL             0x61100

/* CRT */
#define _REG_PCH_ADPA                0xe1100

/* PCH SDVOB multiplex with HDMIB */
#define _REG_PCH_LVDS	0xe1180
#define _REG_BLC_PWM_CPU_CTL2	0x48250
#define _REG_BLC_PWM_CPU_CTL		0x48254
#define _REG_BLC_PWM_PCH_CTL1	0xc8250
#define _REG_BLC_PWM_PCH_CTL2	0xc8254
#define _REG_PCH_PP_ON_DELAYS	0xc7208
#define _REG_PCH_PP_OFF_DELAYS	0xc720c
#define _REG_PCH_PP_DIVISOR		0xc7210
#define _REG_PCH_PP_CONTROL		0xc7204

/* Clocking configuration register */
#define _REG_RSTDBYCTL		0x111b8

/* CPU panel fitter */
/* IVB+ has 3 fitters, 0 is 7x5 capable, the other two only 3x3 */
#define _REG_PFA_CTL_1               0x68080
#define _REG_PFB_CTL_1               0x68880
#define _REG_PFA_WIN_SZ		0x68074
#define _REG_PFB_WIN_SZ		0x68874
#define _REG_PFA_WIN_POS		0x68070
#define _REG_PFB_WIN_POS		0x68870

/* Per-transcoder DIP controls */
#define _REG_TRANSACONF              0xf0008
#define _REG_TRANSBCONF              0xf1008

/* transcoder */
#define _REG_TRANS_HTOTAL_A          0xe0000
#define _REG_TRANS_HBLANK_A          0xe0004
#define _REG_TRANS_HSYNC_A           0xe0008
#define _REG_TRANS_VTOTAL_A          0xe000c
#define _REG_TRANS_VBLANK_A          0xe0010
#define _REG_TRANS_VSYNC_A           0xe0014
#define _REG_TRANS_HTOTAL_B          0xe1000
#define _REG_TRANS_HBLANK_B          0xe1004
#define _REG_TRANS_HSYNC_B           0xe1008
#define _REG_TRANS_VTOTAL_B          0xe100c
#define _REG_TRANS_VBLANK_B          0xe1010
#define _REG_TRANS_VSYNC_B           0xe1014

/* Display & cursor control */

/* Pipe A */
#define _REG_PIPEADSL		0x70000
#define _REG_PIPEACONF		0x70008
#define _REG_PIPEASTAT		0x70024
#define _REG_DSPARB			0x70030

/* Pipe B */
#define _REG_PIPEBDSL		0x71000
#define _REG_PIPEBCONF		0x71008
#define _REG_PIPEBSTAT		0x71024


/* For Gen 2 */
#define _REG_CURSIZE			0x700a0
/*
 * Palette regs
 */
#define _REG_PALETTE_A		0x0a000
#define _REG_PALETTE_B		0x0a800

/* legacy palette */
#define _REG_LGC_PALETTE_A           0x4a000
#define _REG_LGC_PALETTE_B           0x4a800

/*
 * SDVO/UDI pixel multiplier for VGA, same as DPLL_MD_UDI_MULTIPLIER_MASK.
 * This best be set to the default value (3) or the CRT won't work. No,
 * I don't entirely understand what this does...
 */
#define _REG_DPLL_A_MD 0x0601c /* 965+ only */
#define _REG_DPLL_B_MD 0x06020 /* 965+ only */
/*FIXME: this offset conflict with the definition in SNB Bspec */
//#define _REG_BLC_PWM_CTL2		0x61250 /* 965+ only */
#define _REG_FPA0	0x06040
#define _REG_FPA1	0x06044
#define _REG_FPB0	0x06048
#define _REG_FPB1	0x0604c

/* Display Port */
#define _REG_DP_B				0x64100
#define _REG_DP_C				0x64200
#define _REG_DP_D				0x64300

/* Ironlake */

#define _REG_CPU_VGACNTRL	0x41000

/* VBIOS regs */
#define _REG_VGACNTRL		0x71400

/*
 * Instruction and interrupt control regs
 */
#define _REG_HWS_PGA		0x02080
#define _REG_FDI_RXA_IMR             0xf0018
#define _REG_FDI_RXB_IMR             0xf1018
#define _REG_IER		0x020a0
#define _REG_IMR      0x6c

#define _REG_CACHE_MODE_0	0x02120 /* 915+ only */
#define _REG_MI_ARB_STATE	0x020e4 /* 915+ only */

/* VBIOS flags */
#define _REG_SWF00			0x71410
#define _REG_SWF10			0x70410
#define _REG_SWF30			0x72414

/* digital port hotplug */
#define _REG_PCH_GMBUS0		0xc5100

/*
 * GPIO regs
 */
#define _REG_GMBUS0			0x5100 /* clock/port select */
enum vgt_pipe {
	PIPE_A = 0,
	PIPE_B,
	PIPE_C,
	I915_MAX_PIPES
};

struct vgt_intel_device_info {
	u8 gen;
	u8 is_mobile:1;
	u8 is_i85x:1;
	u8 is_i915g:1;
	u8 is_i945gm:1;
	u8 is_g33:1;
	u8 need_gfx_hws:1;
	u8 is_g4x:1;
	u8 is_pineview:1;
	u8 is_broadwater:1;
	u8 is_crestline:1;
	u8 is_ivybridge:1;
	u8 has_fbc:1;
	u8 has_pipe_cxsr:1;
	u8 has_hotplug:1;
	u8 cursor_needs_physical:1;
	u8 has_overlay:1;
	u8 overlay_needs_physical:1;
	u8 supports_tv:1;
	u8 has_bsd_ring:1;
	u8 has_blt_ring:1;
};

/*
 * Next MACROs for GT configuration space.
 */
#define VGT_PCI_CLASS_VGA				0x03
#define VGT_PCI_CLASS_VGA_OTHER			0x80

#define VGT_REG_CFG_CLASS_PROG_IF		0x09
#define VGT_REG_CFG_SUB_CLASS_CODE		0x0A
#define VGT_REG_CFG_CLASS_CODE			0x0B
#define VGT_REG_CFG_SPACE_BAR0			0x10
#define VGT_REG_CFG_SPACE_BAR1			0x18
#define VGT_REG_CFG_SPACE_BAR2			0x20
#define VGT_REG_CFG_SPACE_BAR_ROM		0x30
#define VGT_REG_CFG_SPACE_MSAC			0x62

//#define MSAC_APERTURE_SIZE_MASK		0x3
#define MSAC_APERTURE_SIZE_128M			(0 << 1)
#define MSAC_APERTURE_SIZE_256M			(1 << 1)
#define MSAC_APERTURE_SIZE_512M			(3 << 1)

extern int vgt_thread(void *priv);
extern void vgt_destroy(void);
extern int vgt_initialize(struct pci_dev *dev);
extern bool vgt_register_mmio_handler(int start, int end,
	vgt_mmio_read read, vgt_mmio_write write);

static inline bool vgt_register_mmio_single(int reg,
	vgt_mmio_read read, vgt_mmio_write write)
{
	return vgt_register_mmio_handler(reg, reg + REG_SIZE - 1,
			read, write);
}

static inline bool vgt_register_mmio_write(int reg,
	vgt_mmio_write write)
{
	return vgt_register_mmio_single(reg, NULL, write);
}

static inline bool vgt_register_mmio_read(int reg,
	vgt_mmio_read read)
{
	return vgt_register_mmio_single(reg, read, NULL);
}

extern bool vgt_initialize_mmio_hooks(void);
extern int vgt_hvm_info_init(struct vgt_device *vgt);
extern void vgt_hvm_info_deinit(struct vgt_device *vgt);
extern int vgt_hvm_enable(struct vgt_device *vgt);

struct vgt_irq_virt_state;

struct vgt_hvm_info{
	shared_iopage_t *iopage;
	int nr_vcpu;
	int* evtchn_irq; /* the event channle irqs to handle HVM io request
				 index is vcpu id */
};

/* per-VM structure */
struct vgt_device {
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device  *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */
	vgt_reg_t		last_scan_head[MAX_ENGINES];
	bool			last_scan_head_valid[MAX_ENGINES];

	uint64_t	aperture_base;
	void		*aperture_base_va;
	uint64_t 	aperture_sz;
	uint64_t 	gm_sz;
	uint64_t	aperture_offset;	/* address fix for visible GM */
	uint64_t	hidden_gm_offset;	/* address fix for invisible GM */
	int		fence_base;

	uint64_t   vgtt_sz; /* virtual GTT size in byte */
	uint32_t   *vgtt; /* virtual GTT table for guest to read */

	uint64_t  	rsvd_aperture_base;	/* aperture used for VGT driver */
	vgt_reg_t	saved_wakeup;		/* disable PM before switching */

	struct vgt_irq_virt_state *irq_vstate;
	struct vgt_hvm_info  *hvm_info;
        uint32_t        last_cf8;
	struct kobject kobj;

	bool		ballooning;		/* VM supports ballooning */
};

extern struct vgt_device *vgt_dom0;
enum vgt_owner_type {
	VGT_OT_INVALID = 0,
	VGT_OT_GLOBAL,			// global registers controlled by dom0 or vGT only
	VGT_OT_RCS,                  // the owner directly operating render command buffers
	VGT_OT_BCS,                 // the owner directly operating blitter command buffers
	VGT_OT_VCS,                   // the owner directly operating video command buffers
	VGT_OT_RENDER,                      // the owner directly operating all render buffers (render/blit/video)
	VGT_OT_DISPLAY,                 // the owner having its content directly shown on one or several displays
	VGT_OT_PM,                      // the owner handling GEN power management activities
	VGT_OT_MGMT,                    // the owner managing display/monitor resources
	VGT_OT_MAX,
};

/* owner type of the reg, up to 16 owner type */
#define VGT_REG_OWNER		(0xF)
/* reg access is propogated to sReg and pReg */
#define VGT_REG_PT		(1 << 4)
/* reg contains address, requiring fix */
#define VGT_REG_ADDR_FIX	(1 << 5)
/* HW updated regs */
#define VGT_REG_HW_UPDATE	(1 << 6)
/* Always virtualized even at boot time */
#define VGT_REG_ALWAYS_VIRT	(1 << 7)
/* index into the address-fix table. Maximum 256 entries now */
#define VGT_REG_INDEX_SHIFT	8
#define VGT_REG_INDEX_MASK	(0xFF << VGT_REG_INDEX_SHIFT)
typedef u16 reg_info_t;

#define VGT_ADDR_FIX_NUM	256
typedef vgt_reg_t vgt_addr_mask_t;

struct vgt_irq_host_state;
#define VGT_VBIOS_PAGES 16
/* per-device structure */
struct pgt_device {
	struct list_head	list;

	struct pci_bus *pbus;	/* parent bus of the device */
	struct pci_dev *pdev;	/* the gfx device bound to */
	int bus;		/* parent bus number */
	int devfn;		/* device function number */

	struct task_struct *p_thread;
	wait_queue_head_t wq;
	uint32_t request;

	uint64_t ctx_check;	/* the number of checked count in vgt thread */
	uint64_t ctx_switch;	/* the number of context switch count in vgt thread */
	uint32_t magic;		/* the magic number for checking the completion of context switch */

	vgt_reg_t initial_mmio_state[VGT_MMIO_REG_NUM];	/* copy from physical at start */
	uint8_t initial_cfg_space[VGT_CFG_SPACE_SZ];	/* copy from physical at start */
	uint32_t bar_size[VGT_BAR_NUM];
	uint64_t total_gm_sz;	/* size of available GM space */

	uint64_t gttmmio_base;	/* base of GTT and MMIO */
	void *gttmmio_base_va;	/* virtual base of mmio */
	uint64_t gmadr_base;	/* base of GMADR */
	void *gmadr_va;		/* virtual base of GMADR */

	int max_vms;		/* maximum supported VMs */
	uint64_t rsvd_aperture_sz;
	uint64_t rsvd_aperture_base;
	uint64_t dom0_aperture_sz;
	uint64_t dom0_aperture_base;
	uint64_t vm_aperture_sz;
	uint64_t vm_gm_sz;
	uint64_t	rsvd_aperture_pos;	/* position of the next free reserved page */
	uint64_t  	scratch_page;		/* page used for data written from GPU */
	uint64_t	dummy_area;

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct vgt_device *prev_owner[VGT_OT_MAX];	/* previous owner list of different engines */
	struct list_head rendering_runq_head;
	struct list_head rendering_idleq_head;
	spinlock_t lock;

	reg_info_t *reg_info;	/* virtualization policy for a given reg */
	struct vgt_irq_host_state *irq_hstate;

	uint64_t vgtt_sz; /* in bytes */
	uint32_t *vgtt; /* virtual GTT table for guest to read*/
	struct page *vbios;
};

extern struct list_head pgt_devices;
/*
 * MI_STORE_DATA is used widely for synchronization between GPU and driver,
 * which suppports the destination in either a specific hardware status
 * page, or any other aperture pages mapped to main memory. We don't want
 * to switch the hardware status page from the VM, so adopt the latter form
 * with a scratch page created as the destination with layout defined as
 * below:
 */
#define VGT_DATA_CTX_MAGIC	0x0	/* the magic number used in the context switch */
#define vgt_data_ctx_magic(d)		(d->scratch_page + VGT_DATA_CTX_MAGIC)

#define vgt_get_owner(d, t)		(d->owner[t])
#define vgt_get_previous_owner(d, t)	(d->prev_owner[t])
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_RENDER))
#define current_display_owner(d)	(vgt_get_owner(d, VGT_OT_DISPLAY))
#define current_pm_owner(d)		(vgt_get_owner(d, VGT_OT_PM))
#define current_mgmt_owner(d)		(vgt_get_owner(d, VGT_OT_MGMT))
#define current_global_owner(d)		(vgt_get_owner(d, VGT_OT_GLOBAL))
#define is_current_render_owner(vgt)	(vgt && vgt == current_render_owner(vgt->pdev))
#define is_current_display_owner(vgt)	(vgt && vgt == current_display_owner(vgt->pdev))
#define is_current_pm_owner(vgt)	(vgt && vgt == current_pm_owner(vgt->pdev))
#define is_current_mgmt_owner(vgt)	(vgt && vgt == current_mgmt_owner(vgt->pdev))
#define is_current_global_owner(vgt)	(vgt && vgt == current_global_owner(vgt->pdev))
#define previous_render_owner(d)	(vgt_get_previous_owner(d, VGT_OT_RENDER))
#define previous_display_owner(d)	(vgt_get_previous_owner(d, VGT_OT_DISPLAY))
#define previous_pm_owner(d)		(vgt_get_previous_owner(d, VGT_OT_PM))
#define previous_mgmt_owner(d)		(vgt_get_previous_owner(d, VGT_OT_MGMT))
#define vgt_ctx_check(d)		(d->ctx_check)
#define vgt_ctx_switch(d)		(d->ctx_switch)

#define reg_pt(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_PT)
#define reg_virt(pdev, reg)		(!(reg_pt(pdev, reg)))
#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ADDR_FIX)
#define reg_hw_update(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_HW_UPDATE)
#define reg_always_virt(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ALWAYS_VIRT)
#define reg_addr_index(pdev, reg)	\
	((pdev->reg_info[REG_INDEX(reg)] & VGT_REG_INDEX_MASK) >> VGT_REG_INDEX_SHIFT)
static inline void reg_set_pt(struct pgt_device *pdev, vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_PT;
}

static inline void reg_set_hw_update(struct pgt_device *pdev, vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_HW_UPDATE;
}

static inline void reg_set_addr_fix(struct pgt_device *pdev,
	vgt_reg_t reg, int index)
{
	//ASSERT(reg_pt(pdev, reg));
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ADDR_FIX |
		(index << VGT_REG_INDEX_SHIFT);
}

static inline void reg_set_always_virt(struct pgt_device *pdev, vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ALWAYS_VIRT;
}

extern vgt_addr_mask_t vgt_addr_table[VGT_ADDR_FIX_NUM];
extern int ai_index;
/* mask bits for addr fix */
static inline void vgt_set_addr_mask(struct pgt_device *pdev,
	vgt_reg_t reg, vgt_addr_mask_t mask)
{
	ASSERT(ai_index < VGT_ADDR_FIX_NUM - 1);
	//ASSERT(!(pdev->reg_info[reg] & VGT_REG_OWNER));

	vgt_addr_table[ai_index] = mask;
	reg_set_addr_fix(pdev, reg, ai_index);
	ai_index++;
}

/* if the type is invalid, we assume dom0 always has the permission */
static inline bool reg_is_owner(struct vgt_device *vgt, vgt_reg_t reg)
{
	enum vgt_owner_type type;

	type = vgt->pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER;
	return vgt == vgt_get_owner(vgt->pdev, type);
}

static inline void reg_set_owner(struct pgt_device *pdev,
	vgt_reg_t reg, enum vgt_owner_type type)
{
	pdev->reg_info[REG_INDEX(reg)] |= type & VGT_REG_OWNER;
}

/* request types to wake up main thread */
#define VGT_REQUEST_IRQ		0	/* a new irq pending from device */
static inline void vgt_raise_request(struct pgt_device *pdev, uint32_t flag)
{
	set_bit(flag, (void *)&pdev->request);
	if (waitqueue_active(&pdev->wq))
		wake_up(&pdev->wq);
}

extern struct vgt_device *vgt_super_owner;
/* check whether a reg access should happen on real hw */
static inline bool reg_hw_access(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;

	/* always virtualzed regs like PVINFO */
	if (reg_always_virt(pdev, reg))
		return false;

	/* special phase of super owner like boot-time */
	if (vgt_super_owner == vgt)
		return true;

	/*
	 * if super owner is not dom0, it means that we want to
	 * give exclusive permission to the super owner for the
	 * test. In such case, always return false for other VMs
	 * including dom0
	 */
	if (vgt_super_owner && vgt_super_owner != vgt_dom0)
		return false;

	/* normal phase of passthrough registers if vgt is the owner */
	if (reg_pt(pdev, reg) && reg_is_owner(vgt, reg))
		return true;

	/* or else by default no hw access */
	return false;
}

/* definitions for physical aperture/GM space */
#define aperture_sz(pdev)		(pdev->bar_size[1])
#define aperture_pages(pdev)		(aperture_sz(pdev) >> GTT_PAGE_SHIFT)
#define aperture_base(pdev)		(pdev->gmadr_base)
#define aperture_vbase(pdev)		(pdev->gmadr_va)

#define gm_sz(pdev)			(pdev->total_gm_sz)
#define gm_base(pdev)			(0ULL)
#define gm_pages(pdev)			(gm_sz(pdev) >> GTT_PAGE_SHIFT)
#define hidden_gm_base(pdev)		(aperture_sz(pdev))

#define aperture_2_gm(pdev, addr)	(addr - aperture_base(pdev))
#define v_aperture(pdev, addr)		(aperture_vbase(pdev) + (addr))

#define rsvd_aperture_sz(pdev)		(pdev->rsvd_aperture_sz)
#define rsvd_aperture_base(pdev)	(pdev->rsvd_aperture_base)
#define rsvd_aperture_end(pdev)		\
	(rsvd_aperture_base(pdev) + rsvd_aperture_sz(pdev) - 1)
#define rsvd_aperture_pages(pdev)	(rsvd_aperture_sz(pdev) >> GTT_PAGE_SHIFT)

#define dom0_aperture_sz(pdev)		(pdev->dom0_aperture_sz)
#define dom0_aperture_base(pdev)	(pdev->dom0_aperture_base)
#define dom0_aperture_end(pdev)		\
	(dom0_aperture_base(pdev) + dom0_aperture_sz(pdev) - 1)

#define vm_aperture_sz(pdev)		(pdev->vm_aperture_sz)
#define vm_gm_sz(pdev)			(pdev->vm_gm_sz)
#define vm_gm_hidden_sz(pdev)		(vm_gm_sz(pdev) - vm_aperture_sz(pdev))

/*
 * Aperture/GM virtualization
 *
 * GM is split into two parts: the 1st part visible to CPU through an aperture
 * window mapping, and the 2nd part only accessible from GPU. The virtualization
 * policy is like below:
 *
 *                | VM1 | VM2 | DOM0| RSVD|    VM1   |    VM2   |
 *                ------------------------------------------------
 * Aperture Space |/////|\\\\\|xxxxx|ooooo|                     v
 * (Dev2_BAR)     v                       v                     v
 *                v                       v                     v
 * GM space       v   (visibale part)     v   (invisible part)  v
 * (start from 0) |/////|\\\\\|xxxxx|ooooo|//////////|\\\\\\\\\\|
 *                ^     ^                 ^          ^
 *                |     |  _______________|          |
 *                |     | /          ________________|
 * VM1 GM space   |     |/          /
 * (start from 0) |/////|//////////|
 */
static inline uint64_t get_vm_aperture_base(struct pgt_device *pdev, int i)
{
	return aperture_base(pdev) + i * vm_aperture_sz(pdev);
}

static inline uint64_t get_vm_aperture_end(struct pgt_device *pdev, int i)
{
	return get_vm_aperture_base(pdev, i) + vm_aperture_sz(pdev) - 1;
}

static inline uint64_t get_vm_visible_gm_base(struct pgt_device *pdev, int i)
{
	return gm_base(pdev) + i * vm_aperture_sz(pdev);
}

static inline uint64_t get_vm_visible_gm_end(struct pgt_device *pdev, int i)
{
	return get_vm_visible_gm_base(pdev, i) + vm_aperture_sz(pdev) - 1;
}

static inline uint64_t get_vm_hidden_gm_base(struct pgt_device *pdev, int i)
{
	return hidden_gm_base(pdev) + i * vm_gm_hidden_sz(pdev);
}

static inline uint64_t get_vm_hidden_gm_end(struct pgt_device *pdev, int i)
{
	return get_vm_hidden_gm_base(pdev, i) + vm_gm_hidden_sz(pdev) - 1;
}

/* definitions for vgt's aperture/gm space */
#define vgt_aperture_base(vgt)		(vgt->aperture_base)
#define vgt_aperture_vbase(vgt)		(vgt->aperture_base_va)
#define vgt_aperture_offset(vgt)	(vgt->aperture_offset)
#define vgt_hidden_gm_offset(vgt)	(vgt->hidden_gm_offset)
#define vgt_aperture_sz(vgt)		(vgt->aperture_sz)
#define vgt_gm_sz(vgt)			(vgt->gm_sz)
#define vgt_hidden_gm_sz(vgt)		(vgt_gm_sz(vgt) - vgt_aperture_sz(vgt))

#define vgt_aperture_end(vgt)		\
	(vgt_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_visible_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_aperture_offset(vgt))
#define vgt_visible_gm_end(vgt)		\
	(vgt_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_hidden_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_hidden_gm_offset(vgt))
#define vgt_hidden_gm_end(vgt)		\
	(vgt_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)
#define vgt_visible_fence_sz(vgt)	8/* TO revist: make it configurable */

/*
 * the view of the aperture/gm space from the VM's p.o.v
 *
 * when the VM supports ballooning, this view is the same as the
 * view of vGT driver.
 *
 * when the VM does not support ballooning, this view starts from
 * GM space ZERO
 */
#define vgt_guest_aperture_base(vgt)	\
	(vgt->ballooning ?		\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf) + vgt_aperture_offset(vgt) :	\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf))
#define vgt_guest_aperture_end(vgt)	\
	(vgt_guest_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_visible_gm_base(vgt)	\
	(vgt->ballooning ? vgt_visible_gm_base(vgt) : gm_base(vgt->pdev))
#define vgt_guest_visible_gm_end(vgt)	\
	(vgt_guest_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_hidden_gm_base(vgt)	\
	(vgt->ballooning ? 		\
		vgt_hidden_gm_base(vgt) :	\
		vgt_guest_visible_gm_end(vgt) + 1)
#define vgt_guest_hidden_gm_end(vgt)	\
	(vgt_guest_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

/* translate a guest aperture address to host aperture address */
static inline uint64_t g2h_aperture(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t offset;

	ASSERT_NUM((g_addr >= vgt_guest_aperture_base(vgt)) &&
		(g_addr <= vgt_guest_aperture_end(vgt)), g_addr);

	offset = g_addr - vgt_guest_aperture_base(vgt);
	return vgt_aperture_base(vgt) + offset;
}

/* translate a host aperture address to guest aperture address */
static inline uint64_t h2g_aperture(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t offset;

	ASSERT_NUM((h_addr >= vgt_aperture_base(vgt)) &&
		(h_addr <= vgt_aperture_end(vgt)), h_addr);

	offset = h_addr - vgt_aperture_base(vgt);
	return vgt_guest_aperture_base(vgt) + offset;
}

/* check whether a guest GM address is within the CPU visible range */
static inline bool g_gm_is_visible(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_visible_gm_base(vgt)) &&
		(g_addr <= vgt_guest_visible_gm_end(vgt));
}

/* check whether a guest GM address is out of the CPU visible range */
static inline bool g_gm_is_hidden(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_hidden_gm_base(vgt)) &&
		(g_addr <= vgt_guest_hidden_gm_end(vgt));
}

/* check whether a host GM address is within the CPU visible range */
static inline bool h_gm_is_visible(struct vgt_device *vgt, uint64_t h_addr)
{
	return (h_addr >= vgt_visible_gm_base(vgt)) &&
		(h_addr <= vgt_visible_gm_end(vgt));
}

/* check whether a host GM address is out of the CPU visible range */
static inline bool h_gm_is_hidden(struct vgt_device *vgt, uint64_t h_addr)
{
	return (h_addr >= vgt_hidden_gm_base(vgt)) &&
		(h_addr <= vgt_hidden_gm_end(vgt));
}

/* for a guest GM address, return the offset within the CPU visible range */
static inline uint64_t g_gm_visible_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_visible_gm_base(vgt);
}

/* for a guest GM address, return the offset within the hidden range */
static inline uint64_t g_gm_hidden_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_hidden_gm_base(vgt);
}

/* for a host GM address, return the offset within the CPU visible range */
static inline uint64_t h_gm_visible_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_visible_gm_base(vgt);
}

/* for a host GM address, return the offset within the hidden range */
static inline uint64_t h_gm_hidden_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_hidden_gm_base(vgt);
}

/* translate a guest gm address to host gm address */
static inline uint64_t g2h_gm(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t h_addr;

	ASSERT_NUM(g_gm_is_visible(vgt, g_addr) || g_gm_is_hidden(vgt, g_addr), g_addr);

	if (g_gm_is_visible(vgt, g_addr))	/* aperture */
		h_addr = vgt_visible_gm_base(vgt) +
			g_gm_visible_offset(vgt, g_addr);
	else	/* hidden GM space */
		h_addr = vgt_hidden_gm_base(vgt) +
			g_gm_hidden_offset(vgt, g_addr);

	return h_addr;
}

/* translate a host gm address to guest gm address */
static inline uint64_t h2g_gm(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t g_addr;

	ASSERT_NUM(h_gm_is_visible(vgt, h_addr) || h_gm_is_hidden(vgt, h_addr), h_addr);

	if (h_gm_is_visible(vgt, h_addr))
		g_addr = vgt_guest_visible_gm_base(vgt) +
			h_gm_visible_offset(vgt, h_addr);
	else
		g_addr = vgt_guest_hidden_gm_base(vgt) +
			h_gm_hidden_offset(vgt, h_addr);

	return g_addr;
}

extern dma_addr_t dummy_addr;
/*
 * check whether a structure pointed by MMIO, or an instruction filled in
 * the command buffer, may cross the visible and invisible boundary. That
 * should be avoid since physically two parts are not contiguous
 */
static inline bool check_g_gm_cross_boundary(struct vgt_device *vgt,
	uint64_t g_start, uint64_t size)
{
	if (!vgt_hidden_gm_offset(vgt))
		return false;

	return g_gm_is_visible(vgt, g_start) &&
		g_gm_is_hidden(vgt, g_start + size - 1);
}

#define GTT_MMIO_OFFSET			VGT_MMIO_SPACE_SZ
#define GTT_BASE(pdev)			(pdev->gttmmio_base + GTT_MMIO_OFFSET)
#define GTT_VBASE(pdev)			(pdev->gttmmio_base_va + GTT_MMIO_OFFSET)
#define GTT_SIZE				(2* SIZE_1MB)

#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_PAE_MASK        ((1UL <<12) - (1UL << 4)) /* bit 11:4 */
#define GTT_ENTRY_SIZE		4

#define GTT_INDEX(pdev, addr)		\
	((u32)((addr - gm_base(pdev)) >> GTT_PAGE_SHIFT))

#define GTT_OFFSET_TO_INDEX(offset)		((offset) >> 2)

#define GTT_ADDR(pdev, index)		\
	(GTT_BASE(pdev) + index * GTT_ENTRY_SIZE)

#define GTT_VADDR(pdev, index)		\
	((u32*)GTT_VBASE(pdev) + index)

static inline uint32_t g2h_gtt_index(struct vgt_device *vgt, uint32_t g_index)
{
	uint64_t g_addr = g_index << GTT_PAGE_SHIFT;

	return (uint32_t)(g2h_gm(vgt, g_addr) >> GTT_PAGE_SHIFT);
}

static inline uint32_t h2g_gtt_index(struct vgt_device *vgt, uint32_t h_index)
{
	uint64_t h_addr = h_index << GTT_PAGE_SHIFT;

	return (uint32_t)(h2g_gm(vgt, h_addr) >> GTT_PAGE_SHIFT);
}

#define GTT_MAX_PFN (1UL << 28)
#define GTT_PFN_HIGH_SHIFT 20
#define GTT_PFN_LOW_MASK ((1U << GTT_PFN_HIGH_SHIFT) - 1)

typedef union{
	uint32_t val;
	struct{
		uint32_t valid:1;		/* Valid PTE */
		uint32_t l3cc:1;		/* L3 Cacheability Control */
		uint32_t llccc:1;		/* LLC Cacheability Control */
		uint32_t gfdt:1;		/* Graphics Data Type */
		uint32_t pfn_high:8;	/* Physical Start Address Extension */
		uint32_t pfn_low:20;
	}u;
} gtt_pte_t;

static inline void gtt_pte_make(gtt_pte_t *p_pte, uint32_t val)
{
	p_pte->val = val;
}

static inline unsigned long gtt_pte_get_pfn(gtt_pte_t *p_pte)
{
	return p_pte->u.pfn_low + (p_pte->u.pfn_high << GTT_PFN_HIGH_SHIFT);
}

static inline uint32_t gtt_pte_get_val(gtt_pte_t *p_pte)
{
	return p_pte->val;
}

static inline void gtt_pte_set_pfn(gtt_pte_t *p_pte, uint32_t pfn)
{
	ASSERT(pfn < GTT_MAX_PFN);

	p_pte->u.pfn_low = pfn & GTT_PFN_LOW_MASK;
	p_pte->u.pfn_high = pfn >> GTT_PFN_HIGH_SHIFT;
}

static inline int gtt_pte_valid(gtt_pte_t *p_pte)
{
	return p_pte->u.valid;
}

static inline struct ioreq * vgt_get_hvm_ioreq(struct vgt_device *vgt, int vcpu)
{
	return &(vgt->hvm_info->iopage->vcpu_ioreq[vcpu]);
}

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

/* Save/Restore display context */
int vgt_save_state(struct vgt_device *vgt);
int vgt_restore_state(struct vgt_device *vgt);

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

vgt_reg_t mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t g_value);
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value);

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
//	printk("vgt_read_gtt: index=0x%x, gtt_addr=%lx\n", index, GTT_ADDR(pdev, index));
	return VGT_MMIO_READ(pdev, GTT_MMIO_OFFSET + index*GTT_ENTRY_SIZE);
	//return readl(GTT_VADDR(pdev, index));
}

static inline void vgt_write_gtt(struct pgt_device *pdev, u32 index, u32 val)
{
//	printk("vgt_write_gtt: index=0x%x, gtt_addr=%lx\n", index, GTT_ADDR(pdev, index));
	VGT_MMIO_WRITE(pdev, GTT_MMIO_OFFSET + index*GTT_ENTRY_SIZE , val);
	//writel(val, GTT_VADDR(pdev, index));
}

static inline bool vgt_register_mmio_write_virt(struct pgt_device *pdev,
	int reg, vgt_mmio_write write)
{
	/* add virt policy to let common read handler to emulate read */
	reg_set_always_virt(pdev, reg);
	return vgt_register_mmio_write(reg, write);
}

static inline bool vgt_register_mmio_read_virt(struct pgt_device *pdev,
	int reg, vgt_mmio_read read)
{
	/* add virt policy to let common write handler to emulate write */
	reg_set_always_virt(pdev, reg);
	return vgt_register_mmio_read(reg, read);
}

static inline void vgt_pci_bar_write_32(struct vgt_device *vgt, uint32_t bar_offset, uint32_t val)
{
	uint32_t* cfg_reg;

	/* BAR offset should be 32 bits algiend */
	cfg_reg = (uint32_t*)&vgt->state.cfg_space[bar_offset & ~3];

	/* only write the bits 31-4, leave the 3-0 bits unchanged, as they are read-only */
	*cfg_reg = (val & 0xFFFFFFF0) | (*cfg_reg & 0xF);
}

/* interrupt related definitions */
#define _REG_DEISR	0x44000
#define _REG_DEIMR	0x44004
#define _REG_DEIIR	0x44008
#define _REG_DEIER	0x4400C
#define		_REGSHIFT_MASTER_INTERRUPT	31
#define		_REGBIT_MASTER_INTERRUPT	(1 << 31)
/* FIXME: make better name for shift and bit */
#define		_REGSHIFT_PCH			21
#define		_REGBIT_PCH			(1 << 21)
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
#define vgt_imr_to_isr(vgt, reg)	(__vreg(vgt, reg - 0x4))
#define vgt_imr_to_iir(vgt, reg)	(__vreg(vgt, reg + 0x4))
#define vgt_imr_to_ier(vgt, reg)	(__vreg(vgt, reg + 0x8))
#define vgt_ier_to_isr(vgt, reg)	(__vreg(vgt, reg - 0xC))
#define vgt_ier_to_iir(vgt, reg)	(__vreg(vgt, reg - 0x4))
#define vgt_ier_to_imr(vgt, reg)	(__vreg(vgt, reg - 0x8))

#define _REG_RCS_IMR		0x20A8
#define _REG_VCS_IMR		0x120A8
#define _REG_BCS_IMR		0x220A8

#define _REG_RCS_WATCHDOG_CTL	0x2178
#define _REG_RCS_WATCHDOG_THRSH	0x217C
#define _REG_RCS_WATCHDOG_CTR	0x2190
#define _REG_VCS_WATCHDOG_CTR	0x12178
#define _REG_VCS_WATCHDOG_THRSH	0x1217C

#define _REG_RCS_EIR	0x20B0
#define _REG_RCS_EMR	0x20B4
#define _REG_RCS_ESR	0x20B8
#define _REG_BCS_EIR	0x220B0
#define _REG_BCS_EMR	0x220B4
#define _REG_BCS_ESR	0x220B8
/* interesting no definitiont about video error information. */
//#define _REG_VCS_EIR	0x20B0
//#define _REG_VCS_EMR	0x20B4
//#define _REG_VCS_ESR	0x20B8

/* blacklight PWM control */
#define _REG_BLC_PWM_CTL2	0x48250
#define		_REGBIT_PHASE_IN_IRQ_ENABLE	(1 << 24)
#define		_REGBIT_PHASE_IN_IRQ_STATUS	(1 << 26)
#define _REG_HISTOGRAM_THRSH	0x48268
#define		_REGBIT_HISTOGRAM_IRQ_ENABLE	(1 << 31)
#define		_REGBIT_HISTOGRAM_IRQ_STATUS	(1 << 30)

#define vgt_clear_reg_bit(pdev, reg, bit)		\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val &= ~(1 << bit);			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
		VGT_POST_READ(pdev, reg);		\
	} while (0)

#define vgt_set_reg_bit(pdev, reg, bit)			\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val |= 1 << bit;			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
		VGT_POST_READ(pdev, reg);		\
	} while (0)

enum vgt_event_type {

	// GT
	IRQ_RCS_MI_USER_INTERRUPT = 0,
	IRQ_RCS_DEBUG,
	IRQ_RCS_MMIO_SYNC_FLUSH,
	IRQ_RCS_CMD_STREAMER_ERR,
	IRQ_RCS_PIPE_CONTROL,
	IRQ_RCS_WATCHDOG_EXCEEDED,
	IRQ_RCS_PAGE_DIRECTORY_FAULT,
	IRQ_RCS_AS_CONTEXT_SWITCH,

	IRQ_VCS_MI_USER_INTERRUPT,
	IRQ_VCS_MMIO_SYNC_FLUSH,
	IRQ_VCS_CMD_STREAMER_ERR,
	IRQ_VCS_MI_FLUSH_DW,
	IRQ_VCS_WATCHDOG_EXCEEDED,
	IRQ_VCS_PAGE_DIRECTORY_FAULT,
	IRQ_VCS_AS_CONTEXT_SWITCH,

	IRQ_BCS_MI_USER_INTERRUPT,
	IRQ_BCS_MMIO_SYNC_FLUSH,
	IRQ_BCS_CMD_STREAMER_ERR,
	IRQ_BCS_MI_FLUSH_DW,
	IRQ_BCS_PAGE_DIRECTORY_FAULT,
	IRQ_BCS_AS_CONTEXT_SWITCH,

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

#define VGT_FIRST_RCS_EVENT	IRQ_RCS_MI_USER_INTERRUPT
#define VGT_LAST_RCS_EVENT	IRQ_RCS_AS_CONTEXT_SWITCH
#define VGT_RCS_EVENT(e)	(e >= VGT_FIRST_RCS_EVENT && e <= VGT_LAST_RCS_EVENT)

#define VGT_FIRST_VCS_EVENT	IRQ_VCS_MI_USER_INTERRUPT
#define VGT_LAST_VCS_EVENT	IRQ_VCS_AS_CONTEXT_SWITCH
#define VGT_VCS_EVENT(e)	(e >= IRQ_VCS_MI_USER_INTERRUPT && e <= IRQ_VCS_AS_CONTEXT_SWITCH)

#define VGT_FIRST_BCS_EVENT	IRQ_BCS_MI_USER_INTERRUPT
#define VGT_LAST_BCS_EVENT	IRQ_BCS_AS_CONTEXT_SWITCH
#define VGT_BCS_EVENT(e)	(e >= IRQ_BCS_MI_USER_INTERRUPT && e <= IRQ_BCS_AS_CONTEXT_SWITCH)

#define VGT_FIRST_RENDER_EVENT	VGT_FIRST_RCS_EVENT
#define VGT_LAST_RENDER_EVENT	VGT_LAST_BCS_EVENT
#define VGT_RENDER_EVENT(e)		(e >= VGT_FIRST_RENDER_EVENT && e <= IRQ_BCS_AS_CONTEXT_SWITCH)

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

struct vgt_irq_info_entry;
struct vgt_irq_info;

typedef void (*vgt_core_handler_t)(struct pgt_device *dev, enum vgt_event_type event);

typedef void (*vgt_event_handler_t)(
	struct pgt_device *dev,
	int bit,
	struct vgt_irq_info_entry *entry,
	struct vgt_irq_info *info,
	bool physical,
	struct vgt_device *vgt);

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
	void (*propogate_virtual_event)(struct vgt_device *vstate,
		int bit, struct vgt_irq_info *info);
	struct vgt_irq_info_entry table[VGT_IRQ_BITWIDTH];
};

#define VGT_DPY_EMUL_PERIOD	500000000	// 500ms for now

struct vgt_irq_ops {
	void (*init) (struct pgt_device *dev);

	void (*exit) (struct pgt_device *dev);

	irqreturn_t (*interrupt) (struct pgt_device *dev);

	void (*handle_virtual_interrupt) (struct pgt_device *dev, enum vgt_owner_type type);

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

	char *(*get_reg_name)(struct pgt_device *dev, uint32_t reg);
};

union vgt_event_state {
	/* common state for bit based status */
	struct {
		vgt_reg_t val;
	} status;

	/* command stream error */
	struct {
		int eir_reg;
		vgt_reg_t eir_val;
	} cmd_err;
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
	spinlock_t lock;

	struct vgt_irq_ops *ops;

	int i915_irq;
	int pirq;

	union vgt_event_state states[IRQ_MAX];

	/* master bit enable status from all VMs */
	u64 master_enable;
	/* pch enable status from all VMs */
	u64 pch_enable;
	u64 pch_unmask;

	/* cached pending events */
	u32 gt_iir;
	u32 de_iir;
	u32 pm_iir;
	u32 sde_iir;

	/* display/mgmt mask for DE and PCH registers */
	u32 de_dpy_mask;
	u32 de_mgmt_mask;
	u32 pch_dpy_mask;
	u32 pch_mgmt_mask;
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
#define vgt_get_irq_ops(d)		(d->irq_hstate->ops)
#define vgt_i915_irq(d)			(d->irq_hstate->i915_irq)
#define vgt_pirq(d)			(d->irq_hstate->pirq)
#define vgt_master_enable(d)		(d->irq_hstate->master_enable)
#define vgt_pch_enable(d)		(d->irq_hstate->pch_enable)
#define vgt_pch_unmask(d)		(d->irq_hstate->pch_unmask)
#define vgt_pm_iir(d)			(d->irq_hstate->pm_iir)
#define vgt_de_iir(d)			(d->irq_hstate->de_iir)
#define vgt_gt_iir(d)			(d->irq_hstate->gt_iir)
#define vgt_sde_iir(d)			(d->irq_hstate->sde_iir)
#define vgt_de_dpy_mask(d)		(d->irq_hstate->de_dpy_mask)
#define vgt_de_mgmt_mask(d)		(d->irq_hstate->de_mgmt_mask)
#define vgt_pch_dpy_mask(d)		(d->irq_hstate->pch_dpy_mask)
#define vgt_pch_mgmt_mask(d)		(d->irq_hstate->pch_mgmt_mask)
#define vgt_event_state(d, e)		(d->irq_hstate->states[e])

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

#define vgt_core_event_handler(d, e)	\
	((vgt_core_event_handlers(d))[e])

//#ifdef VGT_DEBUG
#define vgt_trace_irq_event(i, t)	\
	printk("vGT (%s): catch event (%s) in reg (%x).\n",	\
		(i)->name, vgt_irq_name[(t)],			\
		vgt_iir((i)->reg_base))
//#else
//#define vgt_trace_irq_event(i, t)
//#endif

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
void inject_hvm_virtual_interrupt(struct vgt_device *vgt);
void inject_dom0_virtual_interrupt(struct vgt_device *vgt);
int vgt_vstate_irq_init(struct vgt_device *vgt);
void vgt_vstate_irq_exit(struct vgt_device *vgt);
int vgt_irq_init(struct pgt_device *pgt);
void vgt_irq_exit(struct pgt_device *pgt);
void vgt_irq_save_context(struct vgt_device *vstate, enum vgt_owner_type owner);
void vgt_irq_restore_context(struct vgt_device *vstate, enum vgt_owner_type owner);

void vgt_propogate_pch_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info);
void vgt_propogate_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info);
void vgt_propogate_emulated_event(struct vgt_device *vstate,
	enum vgt_event_type event);
void vgt_irq_handle_event(struct pgt_device *dev, void *iir,
	struct vgt_irq_info *info, bool physical,
	enum vgt_owner_type o_type);
void vgt_handle_virtual_interrupt(struct pgt_device *pdev, enum vgt_owner_type type);
void vgt_default_event_handler(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_unexpected_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_host_only_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_weak_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_cmd_stream_error(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_phase_in(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_histogram(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);

void vgt_handle_aux_channel(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_gmbus(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);

void vgt_emulate_watchdog(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
void vgt_emulate_dpy_status(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_irr_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
extern char *vgt_irq_name[IRQ_MAX];

struct vgt_device *create_vgt_instance(struct pgt_device *pdev, int vm_id);
void vgt_release_instance(struct vgt_device *vgt);
int vgt_init_sysfs(struct pgt_device *pdev);
extern void vgt_set_display_pointer(int vm_id);
extern ssize_t vgt_get_display_pointer(char *buf);

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,	void *p_data, unsigned int bytes);
bool default_mmio_write(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes);

/*
 * Configuration register definition for BDF: 0:0:0.
 */
#define _REG_GMCH_CONTRL	0x50

#endif	/* _VGT_REG_H_ */
