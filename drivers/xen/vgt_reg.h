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

#define RB_TAIL(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_TAIL)
#define RB_HEAD(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_HEAD)
#define RB_START(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_START)
#define RB_CTL(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_CTL)

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

#define _REG_RCS_MI_MODE	0x209C
#define		_REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE	(1 << 14)
#define		_REGBIT_MI_FLUSH_PERFORMANCE_MODE	(1 << 13)
//#define		_REGBIT_MI_FLUSH			(3 << 11)
#define		_REGBIT_MI_FLUSH			(1 << 12)
#define		_REGBIT_MI_INVALIDATE_UHPTR		(1 << 11)
#define		_REGBIT_MI_RINGS_IDLE			(1 << 9)
#define		_REGBIT_MI_STOP_RINGS			(1 << 8)
#define	_REG_VCS_MI_MODE	0x1209C
#define _REG_BCS_MI_MODE	0x2209C
#define _REG_GFX_MODE	0x2520
#define		_REGBIT_FLUSH_TLB_INVALIDATION_MODE	(1 << 13)
#define		_REGBIT_REPLAY_MODE			(1 << 11)
#define		_REGBIT_PPGTT_ENABLE			(1 << 9)
#define _REG_ARB_MODE	0x4030
#define		_REGBIT_ADDRESS_SWIZZLING		(3 << 4)
#define _REG_GT_MODE	0x20D0

#define _REG_GAC_MODE		0x120A0
#define _REG_GAB_MODE		0x220A0

#define _REG_RCS_INSTPM		0x20C0
#define _REG_VCS_INSTPM		0x120C0
#define _REG_BCS_INSTPM		0x220C0
#define INSTPM_CONS_BUF_ADDR_OFFSET_DIS (1<<6)

/* IVB+ */
#define _REG_BCS_BLT_MODE_IVB	0x2229C
#define _REG_RCS_GFX_MODE_IVB	0x0229C
#define _REG_VCS_MFX_MODE_IVB	0x1229C

/* PPGTT entry */
#define _REGBIT_PDE_VALID	(1<<0)
#define _REGBIT_PDE_PAGE_32K	(1<<1)
#define _REGBIT_PTE_VALID	(1<<0)
/* control bits except address and valid bit */
#define _REGBIT_PTE_CTL_MASK_GEN7	0xe	/* SNB/IVB */
#define _REGBIT_PTE_CTL_MASK_GEN7_5	0x80e	/* HSW */

#define _REG_RCS_PSMI		0x2050
#define _REG_VCS_PSMI		0x12050
#define _REG_BCS_PSMI		0x22050
#define 	_REGBIT_PSMI_IDLE_INDICATOR	(1 << 3)

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

#define _REG_RCS_PP_DIR_BASE_READ	0x2518
#define _REG_RCS_PP_DIR_BASE_WRITE	0x2228
#define _REG_RCS_PP_DIR_BASE_IVB	0x2228
#define _REG_RCS_PP_DCLV		0x2220
#define _REG_BCS_PP_DIR_BASE		0x22228
#define _REG_BCS_PP_DCLV		0x22220
#define _REG_VCS_PP_DIR_BASE		0x12228
#define _REG_VCS_PP_DCLV		0x12220

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
#define _REG_DSPBTILEOFF	0x711A4
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

#define _REG_PCH_DPB_AUX_CH_CTL 	0xe4110
#define _REG_PCH_DPB_AUX_CH_DATA1	0xe4114
#define _REG_PCH_DPB_AUX_CH_DATA2	0xe4118
#define _REG_PCH_DPB_AUX_CH_DATA3	0xe411c
#define _REG_PCH_DPB_AUX_CH_DATA4	0xe4120
#define _REG_PCH_DPB_AUX_CH_DATA5	0xe4124

#define _REG_PCH_DPC_AUX_CH_CTL		0xe4210
#define _REG_PCH_DPC_AUX_CH_DATA1	0xe4214
#define _REG_PCH_DPC_AUX_CH_DATA2	0xe4218
#define _REG_PCH_DPC_AUX_CH_DATA3	0xe421c
#define _REG_PCH_DPC_AUX_CH_DATA4	0xe4220
#define _REG_PCH_DPC_AUX_CH_DATA5	0xe4224

#define _REG_PCH_DPD_AUX_CH_CTL 	0xe4310
#define _REG_PCH_DPD_AUX_CH_DATA1	0xe4314
#define _REG_PCH_DPD_AUX_CH_DATA2	0xe4318
#define _REG_PCH_DPD_AUX_CH_DATA3	0xe431c
#define _REG_PCH_DPD_AUX_CH_DATA4	0xe4320
#define _REG_PCH_DPD_AUX_CH_DATA5	0xe4324

#define _REGBIT_DP_AUX_CH_CTL_INTERRUPT (1 << 29)
#define _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR (1 << 28)
#define _REGBIT_DP_AUX_CH_CTL_RECV_ERR (1 << 25)
#define _REGBIT_DP_AUX_CH_CTL_TIME_OUT_400us	(0 << 26)
#define _DP_DETECTED			(1 << 2)
#define _DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT 	0
#define _DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT	16
#define _REG_FORCEWAKE		0xA18C
#define _REG_FORCEWAKE_ACK	0x130090
#define _REG_MUL_FORCEWAKE	0xA188
#define _REG_MUL_FORCEWAKE_ACK  0x130040
#define _REG_ECOBUS		0xA180
#define _REGBIT_MUL_FORCEWAKE_ENABLE	(1<<5)

#define _REG_GEN6_GDRST	0x941c
#define  _REGBIT_GEN6_GRDOM_FULL		(1 << 0)
#define  _REGBIT_GEN6_GRDOM_RENDER		(1 << 1)
#define  _REGBIT_GEN6_GRDOM_MEDIA		(1 << 2)
#define  _REGBIT_GEN6_GRDOM_BLT			(1 << 3)

#define _REG_GT_THREAD_STATUS  0x13805C
#define _REG_GT_CORE_STATUS  0x138060
#define _REG_RC_STATE_CTRL_1    0xA090
#define _REG_RC_STATE_CTRL_2    0xA094

#define _REGBIT_RC_HW_CTRL_ENABLE    (1<<31)
#define _REGBIT_RC_RC1_ENABLE    (1<<20)
#define _REGBIT_RC_RC6_ENABLE    (1<<18)
#define _REGBIT_RC_DEEP_RC6_ENABLE    (1<<17)
#define _REGBIT_RC_DEEPEST_RC6_ENABLE    (1<<16)

#define _REG_HDCP_STATUS_REG_1	0xE651C
#define _REG_HDCP_STATUS_REG_2	0xE661C
#define _REG_HDCP_STATUS_REG_3	0xE671C
#define _REG_HDCP_STATUS_REG_4	0xE681C
#define _REGBIT_HDCP_CIPHER_AN_READY	(1<<17)

#define _REG_HDCP_KEY_STATUS_REG	0xE6C04
#define _REGBIT_HDCP_KEY_DONE		(3)

#define _REG_HDCP_PCH_BOOT_AUTH_STATUS_REG     0xE6E1C
#define _REGBIT_HDCP_PCH_BOOT_AUTH_STATUS_READY (0x2F << 16)

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
#define _REG_HSYNC_A	0x60008
#define _REG_VTOTAL_A	0x6000c
#define _REG_VBLANK_A	0x60010
#define _REG_VSYNC_A	0x60014
#define _REG_PIPEASRC	0x6001c
#define _REG_BCLRPAT_A	0x60020

/* Pipe B timing regs */
#define _REG_HTOTAL_B	0x61000
#define _REG_HBLANK_B	0x61004
#define _REG_HSYNC_B	0x61008
#define _REG_VTOTAL_B	0x6100c
#define _REG_VBLANK_B	0x61010
#define _REG_VSYNC_B	0x61014
#define _REG_PIPEBSRC	0x6101c
#define _REG_BCLRPAT_B	0x61020

#define _REG_DISP_ARB_CTL	0x45000
#define _REG_DISP_ARB_CTL2	0x45004
#define _REG_TILECTL 		0x101000

/* PCH */
#define _REG_PCH_DREF_CONTROL        0xc6200

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

#define _REGBIT_FDI_RX_ENABLE       (1 << 31)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT       (0<<8)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_2_CPT       (1<<8)

#define _REG_FDI_RXA_IIR           0xf0014
#define _REG_FDI_RXB_IIR           0xf1014

#define _REGBIT_FDI_RX_BIT_LOCK     (1 << 8) /* train 1*/
#define _REGBIT_FDI_RX_SYMBOL_LOCK  (1 << 9) /* train 2*/

/* CPU: FDI_TX */
#define _REG_FDI_TXA_CTL             0x60100
#define _REG_FDI_TXB_CTL             0x61100

#define _REGBIT_FDI_TX_ENABLE       (1 << 31)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_1    (0 << 28)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_2    (1 << 28)

/* CRT */
#define _REG_PCH_ADPA                0xe1100
#define _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER (1 << 16)
#define _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK (3 << 24)
#define _REGBIT_ADPA_DAC_ENABLE (1 << 31)

/* Display port */
#define _REG_DP_B_CTL	0xe4100
#define _REG_DP_C_CTL	0xe4200
#define _REG_DP_D_CTL   0xe4300
#define _REGBIT_DP_PORT_DETECTED	(1 << 2)

/* Digital display A (DP_A, embedded) */
#define _REG_DP_A_CTL	0x64000
#define _REGBIT_DP_PORT_A_DETECTED	(1 << 2)

/* HDMI/DVI/SDVO port */
#define _REG_HDMI_B_CTL	0xe1140
#define _REG_HDMI_C_CTL	0xe1150
#define _REG_HDMI_D_CTL	0xe1160
#define _REGBIT_HDMI_PORT_DETECTED	(1 << 2)


/* PCH SDVOB multiplex with HDMIB */
#define _REG_PCH_LVDS	0xe1180
#define _REG_BLC_PWM_CPU_CTL2	0x48250
#define _REG_BLC_PWM_CPU_CTL	0x48254
#define _REG_BLC_PWM_PCH_CTL1	0xc8250
#define _REG_BLC_PWM_PCH_CTL2	0xc8254
#define _REG_PCH_PP_ON_DELAYS	0xc7208
#define _REG_PCH_PP_OFF_DELAYS	0xc720c
#define _REG_PCH_PP_DIVISOR		0xc7210
#define _REG_PCH_PP_CONTROL		0xc7204

union PCH_PP_CONTROL
{
	uint32_t data;
	struct
	{
		uint32_t power_state_target	: 1; // bit 0
		uint32_t power_down_on_reset	: 1; // bit 1
		uint32_t backlight_enable	: 1; // bit 2
		uint32_t edp_vdd_override_for_aux : 1; // bit 3
		uint32_t reserve : 12; // bits 15:4
		uint32_t write_protect_key :16; // bits 31:16 0xABCD to disable protected)
	};
};

#define _REG_PCH_PP_STATUS		0xc7200

union PCH_PP_STAUTS
{
	uint32_t data;
	struct
	{
		uint32_t reserv1	: 4;	// bit 3:0
		uint32_t reserv2	: 23;	// bit 26:4
		uint32_t power_cycle_delay_active	:1;	// bit 27
		uint32_t power_sequence_progress	:2;	// bits 29:28
		uint32_t require_asset_status		:1; // bit 30
		uint32_t panel_powere_on_statue		:1; // bit 31   (0 - Disable, 1 - Enable)
	};
};

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

union _TRANS_CONFIG
{
	uint32_t data;
	struct
	{
		uint32_t reserve1 : 10;   // bit 9:0
		uint32_t xvycc_color_range_limit : 1; // bit 10
		uint32_t reserve2 : 10; // bit 20:11
		uint32_t interlaced_mode: 3; // bit 23:21
		uint32_t reserve3 : 6; // bit 29:24
		uint32_t transcoder_state : 1; // bit 30
		uint32_t transcoder_enable : 1; // bit 31
	};
};

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
#define _REG_PIPEA_FRMCOUNT 0x70040

/* Pipe B */
#define _REG_PIPEBDSL		0x71000
#define _REG_PIPEBCONF		0x71008
#define _REG_PIPEBSTAT		0x71024
#define _REG_PIPEB_FRMCOUNT 0x71040

#define _REGBIT_PIPE_ENABLE    (1 << 31)
#define _REGBIT_PIPE_STAT_ENABLED  (1 << 30)

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
#define _REG_IMR      0x020a8

#define _REG_CACHE_MODE_0	0x02120 /* 915+ only */
#define _REG_CACHE_MODE_1	0x02124
#define _REG_MI_ARB_STATE	0x020e4 /* 915+ only */

/* VBIOS flags */
#define _REG_SWF00			0x71410
#define _REG_SWF10			0x70410
#define _REG_SWF30			0x72414

/* digital port hotplug */

#define _REG_PCH_GPIOA		0xc5010
#define _REG_PCH_GPIOB		0xc5014
#define _REG_PCH_GPIOC		0xc5018
#define _REG_PCH_GPIOD		0xc501c
#define _REG_PCH_GPIOE		0xc5020
#define _REG_PCH_GPIOF		0xc5024

#define _REG_PCH_GMBUS0		0xc5100
#define _REG_PCH_GMBUS1		0xc5104
#define _REG_PCH_GMBUS2		0xc5108
#define _REG_PCH_GMBUS3		0xc510c

#define _GMBUS_SW_CLR_INT	(1<<31)
#define _GMBUS_SW_RDY		(1<<30)
#define _GMBUS_CYCLE_WAIT	(1<<25)
#define _GMBUS_CYCLE_STOP	(4<<25)
#define _GMBUS_HW_RDY		(1<<11)
#define _GMBUS_SATOER		(1<<10)
#define _GMBUS_SLAVE_READ	(1<<0)
#define _GMBUS_SLAVE_WRITE	(0<<0)
#define _GMBUS_BYTE_COUNT_SHIFT	16
#define _GMBUS_SLAVE_ADDR_SHIFT	1

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

#define EDID_NUM	8
typedef enum {
	EDID_VGA = 0,
	EDID_LVDS,
	EDID_HDMIC,
	EDID_HDMIB,
	EDID_HDMID,
	EDID_DPB,
	EDID_DPC,
	EDID_DPD
}edid_index_t;

enum vgt_port_type {
	VGT_CRT = 0,
	VGT_DP_A,
	VGT_DP_B,   /* DP use the same physical pins as HDMI/DVI, therefore HDMI/DI and DP cannot be enabled simultaneously */
	VGT_DP_C,
	VGT_DP_D,
	VGT_SDVO_B, /* HDMI port B can be used for sDVO by setting the encoding field */
	VGT_DVI_B,
	VGT_DVI_C,
	VGT_DVI_D,
	VGT_HDMI_B,
	VGT_HDMI_C, /* HDMI port C can only be used for HDMI/DVI */
	VGT_HDMI_D,
	VGT_LVDS,
	VGT_PORT_MAX
};

/* interrupt related definitions */
#define _REG_DEISR	0x44000
#define _REG_DEIMR	0x44004
#define _REG_DEIIR	0x44008
#define _REG_DEIER	0x4400C
#define		_REGSHIFT_MASTER_INTERRUPT	31
#define		_REGBIT_MASTER_INTERRUPT	(1 << 31)
#define		_REGBIT_DP_A_HOTPLUG		(1 << 19)
/* FIXME: make better name for shift and bit */
#define		_REGSHIFT_PCH			21
#define		_REGBIT_PCH			(1 << 21)
/* GEN7 */
#define		_REGSHIFT_PCH_GEN7		28
#define		_REGBIT_PCH_GEN7		(1 << 28)
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
#define     _REGBIT_CRT_HOTPLUG         (1 << 19)
#define		_REGBIT_SDVO_B_HOTPLUG		(1 << 18)
#define		_REGBIT_DP_B_HOTPLUG		(1 << 21)
#define		_REGBIT_DP_C_HOTPLUG		(1 << 22)
#define		_REGBIT_DP_D_HOTPLUG		(1 << 23)
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
#define VGT_REG_CFG_SWSCI_TRIGGER		0xE8
#define VGT_REG_CFG_OPREGION			0xFC

#define VGT_OPREGION_PAGES				2
#define VGT_OPREGION_REG_CLID			0x1AC

//#define MSAC_APERTURE_SIZE_MASK		0x3
#define MSAC_APERTURE_SIZE_128M			(0 << 1)
#define MSAC_APERTURE_SIZE_256M			(1 << 1)
#define MSAC_APERTURE_SIZE_512M			(3 << 1)


/*
 * Configuration register definition for BDF: 0:0:0.
 */
#define _REG_GMCH_CONTRL	0x50

#endif	/* _VGT_REG_H_ */
