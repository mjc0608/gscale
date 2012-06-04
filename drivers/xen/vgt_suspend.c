/*
 * vGT display save/restore
 *
 * This file is almost a copy from i915_suspend.c
 * So currenly marked as  GPLv2 license.  When using or
 * redistributing this file, you may do so under such license.
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
 */
#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/hash.h>
#include <linux/delay.h>
#include <asm/bitops.h>
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_reg.h"

/* copied & modified from include/drm/drmP.h */
#define VGT_DRIVER_MODESET     0x2000
u32 __vgt_driver_cap = VGT_DRIVER_MODESET;
//#define vgt_driver_check_feature(feature)   (__vgt_driver_cap & (feature))

bool static vgt_driver_check_feature(struct vgt_device *vgt, int feature)
{
    vgt_reg_t sreg;
    bool retval = false;
#define _VGA_DISP_DISABLED   1 << 31
    switch(feature) {
        case VGT_DRIVER_MODESET:
            sreg = __sreg(vgt, _REG_CPU_VGACNTRL);
            if (sreg & _VGA_DISP_DISABLED)
                retval = true;
            break;
        default:
            printk(KERN_WARNING"%s:WARNING: vGT check UNKNOWN driver feature\n", __func__);
    }

    return retval;
}

/* FIXME: snb_devinfo copied from  */
/* static const struct intel_device_info intel_sandybridge_d_info = {
 */
static struct vgt_intel_device_info snb_devinfo = {
    .gen = 6,
	.need_gfx_hws = 1,
    .has_hotplug = 1,
	.has_bsd_ring = 1,
	.has_blt_ring = 1,
};

/* FIXME: should be a member with pgt device */
struct vgt_intel_device_info *vgt_devinfo = &snb_devinfo;

#define VGT_HAS_PCH_SPLIT(pdev) ( (vgt_devinfo->gen == 5) || (vgt_devinfo->gen ==6) || (vgt_devinfo->is_ivybridge == 1))
#define VGT_SUPPORT_INTEGRATED_DP(pdev) ((vgt_devinfo->is_g4x) || (vgt_devinfo->gen == 5))
#define VGT_I915_HAS_FBC(pdev) (vgt_devinfo->has_fbc)
#define VGT_GEN(pdev) (vgt_devinfo->gen)

/* FIXME: does read_mmio need to update v/s regs ??? */
#define vgt_read_mmio_reg_8(offset) VGT_MMIO_READ_BYTES(pdev, (offset), 1)

#define vgt_write_mmio_reg(offset, val)    \
    do {                                    \
        __vreg(vgt, (offset)) = val;       \
        __sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), (val));    \
        VGT_MMIO_WRITE(pdev, (offset), __sreg(vgt, (offset)));            \
    } while(0)

#define vgt_write_mmio_reg_8(offset, val)    \
    do {                                    \
        __vreg8(vgt, (offset)) = (val);           \
        __sreg8(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), (val));    \
        VGT_MMIO_WRITE_BYTES(pdev, (offset), __sreg8(vgt, (offset)), 1);            \
    } while(0)

#define vgt_restore_mmio_reg(offset)    \
    do { __sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg(vgt, (offset))); \
        VGT_MMIO_WRITE(pdev, (offset), __sreg(vgt, (offset)));                    \
    } while(0)

#define vgt_restore_mmio_reg_64(offset) \
    do {   __sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg(vgt, (offset)));  \
        __sreg(vgt, (offset) + 4) = mmio_g2h_gmadr(vgt, (offset) + 4, __vreg(vgt, (offset) + 4));  \
        VGT_MMIO_WRITE_BYTES(pdev, (offset), __sreg64(vgt, (offset)), 8);             \
    } while(0)

#define vgt_restore_mmio_reg_8(offset) \
    do {                                    \
        __sreg8(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg8(vgt, (offset))); \
        VGT_MMIO_WRITE_BYTES(pdev, (offset), __sreg8(vgt, (offset)), 1); \
    } while(0)

#define vgt_save_mmio_reg(offset)   \
    do {                            \
        __sreg(vgt, (offset)) = VGT_MMIO_READ(pdev, (offset));               \
		__vreg(vgt, (offset)) = mmio_h2g_gmadr(vgt, (offset), __sreg(vgt, (offset)));    \
    } while(0)

#define vgt_save_mmio_reg_8(offset)   \
    do {                            \
        __sreg8(vgt, (offset)) = VGT_MMIO_READ_BYTES(pdev, (offset), 1);     \
        __vreg8(vgt, (offset)) = mmio_h2g_gmadr(vgt, (offset), __sreg8(vgt, (offset)));     \
    } while(0)

#define vgt_save_mmio_reg_64(offset)    \
    do {                                \
        __sreg64(vgt, (offset)) = VGT_MMIO_READ_BYTES(pdev, (offset), 8);   \
    } while(0)

#define not_done()              \
    do {                        \
        printk(KERN_WARNING"%s: %d:  not done yet!\n", __func__, __LINE__);   \
        BUG();                  \
    } while(0)



static void vgt_restore_vga(struct vgt_device *vgt)
{
	int i;
	u16 cr_index, cr_data, st01;
    struct pgt_device *pdev = vgt->pdev;
    vgt_state_t *vgt_state = &vgt->state;

    vgt_restore_mmio_reg_8(_REG_VGA_MSR_WRITE);
    if (__vreg(vgt, _REG_VGA_MSR_WRITE) & VGA_MSR_CGA_MODE) {
        cr_index = _REG_VGA_CR_INDEX_CGA;
        cr_data = _REG_VGA_CR_DATA_CGA;
        st01 = _REG_VGA_ST01_CGA;
    } else {
        cr_index = _REG_VGA_CR_INDEX_MDA;
        cr_data = _REG_VGA_CR_DATA_MDA;
        st01 = _REG_VGA_ST01_MDA;
    }

	/* Sequencer registers, don't write SR07 */
    for (i = 0; i < 7; i++) {
        vgt_write_mmio_reg_8(_REG_VGA_SR_INDEX, i);
        vgt_write_mmio_reg_8(_REG_VGA_SR_DATA, vgt_state->saveSR[i]);
    }

	/* CRT controller regs */
	/* Enable CR group 0 writes */
    vgt_write_mmio_reg_8(cr_index, 0x11);
    vgt_write_mmio_reg_8(cr_data, vgt_state->saveCR[0x11]);
    for (i = 0; i <= 0x24; i++) {
        vgt_write_mmio_reg_8(cr_index, i);
        vgt_write_mmio_reg_8(cr_data, vgt_state->saveCR[i]);
    }

	/* Graphics controller regs */
    for (i = 0; i < 9; i++) {
        vgt_write_mmio_reg_8(_REG_VGA_GR_INDEX, i);
        vgt_write_mmio_reg_8(_REG_VGA_GR_DATA, vgt_state->saveGR[i]);
    }

    vgt_write_mmio_reg_8(_REG_VGA_GR_INDEX, 0x10);
    vgt_write_mmio_reg_8(_REG_VGA_GR_DATA, vgt_state->saveGR[0x10]);
    vgt_write_mmio_reg_8(_REG_VGA_GR_INDEX, 0x11);
    vgt_write_mmio_reg_8(_REG_VGA_GR_DATA, vgt_state->saveGR[0x11]);
    vgt_write_mmio_reg_8(_REG_VGA_GR_INDEX, 0x18);
    vgt_write_mmio_reg_8(_REG_VGA_GR_DATA, vgt_state->saveGR[0x18]);

	/* Attribute controller registers */
    vgt_read_mmio_reg_8(st01);
    for (i = 0; i <= 0x14; i++) {
        vgt_read_mmio_reg_8(st01);
        vgt_write_mmio_reg_8(_REG_VGA_AR_INDEX, (i | 0));
        vgt_write_mmio_reg_8(_REG_VGA_AR_DATA_WRITE, vgt_state->saveAR[i]);
    }
    vgt_read_mmio_reg_8(st01);/* switch back to index mode */
    vgt_write_mmio_reg_8(_REG_VGA_AR_INDEX, __vreg(vgt, (_REG_VGA_AR_INDEX | 0x20)));
    vgt_read_mmio_reg_8(st01);

	/* VGA color palette registers */
    vgt_restore_mmio_reg_8(_REG_VGA_DACMASK);
}

static bool vgt_pipe_enabled(struct vgt_device *vgt, enum vgt_pipe pipe)
{
    u32 dpll_reg;
    if (VGT_HAS_PCH_SPLIT(vgt))
        dpll_reg = (pipe == PIPE_A) ? _REG_PCH_DPLL_A : _REG_PCH_DPLL_B;
    else
        dpll_reg = (pipe == PIPE_A) ? _REG_DPLL_A : _REG_DPLL_B;

    /* FIXME: just use vreg ??, do we first have a copy of all these hw configurations */
    return (__vreg(vgt, dpll_reg) & DPLL_VCO_ENABLE);
}

static void vgt_restore_palette(struct vgt_device *vgt, enum vgt_pipe pipe)
{
    int i;
    struct pgt_device *pdev = vgt->pdev;
    unsigned long reg = (pipe == PIPE_A ? _REG_PALETTE_A : _REG_PALETTE_B);

    if (!vgt_pipe_enabled(vgt, pipe))
        return;

    if (VGT_HAS_PCH_SPLIT(vgt))
        reg = (pipe == PIPE_A) ? _REG_LGC_PALETTE_A : _REG_LGC_PALETTE_B;

    for (i = 0; i < 256; i++)
        vgt_restore_mmio_reg(reg + (i << 2));
}

/* FIXME: what is modeset ??? */
//static void i915_restore_modeset_reg(struct drm_device *dev)
static void vgt_restore_modeset_reg(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    int reg_index;

    /* default support DRIVER_MODESET */
    /* TODO: checking feature DRIVER_MODESET */
    if (vgt_driver_check_feature(vgt, VGT_DRIVER_MODESET))
        return;

    /* Fences */
    /* TODO: only support snb now */
    for (reg_index = 0; reg_index < 16; reg_index++)
        vgt_restore_mmio_reg_64(_REG_FENCE_0_LOW + (reg_index * 8));


    /* TODO: only support snb, default as support PCH_SPLIT */
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_PCH_DREF_CONTROL);
        vgt_restore_mmio_reg(_REG_DISP_ARB_CTL);
    }

	/* Pipe & plane A info */
	/* Prime the clock */
    /* FIXME: since we support PCH split, so use PCH_DPLL_A */
    if (__vreg(vgt, _REG_PCH_DPLL_A) & DPLL_VCO_ENABLE) //FIXME: or sreg ???
    {
        vgt_write_mmio_reg(_REG_PCH_DPLL_A, __vreg(vgt, (_REG_PCH_DPLL_A & ~DPLL_VCO_ENABLE)));
        VGT_POST_READ(pdev, _REG_PCH_DPLL_A);
        udelay(150);
    }
    vgt_restore_mmio_reg(_REG_PCH_FPA0);
    vgt_restore_mmio_reg(_REG_PCH_FPA1);

	/* Actually enable it */
    vgt_restore_mmio_reg(_REG_PCH_DPLL_A);
    VGT_POST_READ(pdev, _REG_PCH_DPLL_A);
	udelay(150);
    if (VGT_GEN(vgt) >= 4 && !VGT_HAS_PCH_SPLIT(dev)) {
        vgt_restore_mmio_reg(_REG_DPLL_A_MD);
        VGT_POST_READ(pdev, _REG_DPLL_A_MD);
    }
	udelay(150);

	/* Restore mode */
    vgt_restore_mmio_reg(_REG_HTOTAL_A);
    vgt_restore_mmio_reg(_REG_HBLANK_A);
    vgt_restore_mmio_reg(_REG_HSYNC_A);
    vgt_restore_mmio_reg(_REG_VTOTAL_A);
    vgt_restore_mmio_reg(_REG_VBLANK_A);
    vgt_restore_mmio_reg(_REG_VSYNC_A);

    if (!VGT_HAS_PCH_SPLIT(vgt))
        vgt_restore_mmio_reg(_REG_BCLRPAT_A);

    if(VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_PIPEA_DATA_M1);
        vgt_restore_mmio_reg(_REG_PIPEA_DATA_N1);
        vgt_restore_mmio_reg(_REG_PIPEA_LINK_M1);
        vgt_restore_mmio_reg(_REG_PIPEA_LINK_N1);

        vgt_restore_mmio_reg(_REG_FDI_RXA_CTL);
        vgt_restore_mmio_reg(_REG_FDI_TXA_CTL);

        vgt_restore_mmio_reg(_REG_PFA_CTL_1);
        vgt_restore_mmio_reg(_REG_PFA_WIN_SZ);
        vgt_restore_mmio_reg(_REG_PFA_WIN_POS);

        vgt_restore_mmio_reg(_REG_TRANSACONF);
        vgt_restore_mmio_reg(_REG_TRANS_HTOTAL_A);
        vgt_restore_mmio_reg(_REG_TRANS_HBLANK_A);
        vgt_restore_mmio_reg(_REG_TRANS_HSYNC_A);
        vgt_restore_mmio_reg(_REG_TRANS_VTOTAL_A);
        vgt_restore_mmio_reg(_REG_TRANS_VBLANK_A);
        vgt_restore_mmio_reg(_REG_TRANS_VSYNC_A);
    }

    /* Restore plane info */
    vgt_restore_mmio_reg(_REG_DSPASIZE);
    vgt_restore_mmio_reg(_REG_DSPAPOS);
    vgt_restore_mmio_reg(_REG_PIPEASRC);
    vgt_restore_mmio_reg(_REG_DSPALINOFF);
    vgt_restore_mmio_reg(_REG_DSPASTRIDE);
    if (VGT_GEN(vgt) >= 4) {
        vgt_restore_mmio_reg(_REG_DSPASURF);
        vgt_restore_mmio_reg(_REG_DSPATILEOFF);
    }

    vgt_restore_mmio_reg(_REG_PIPEACONF);

    vgt_restore_palette(vgt, PIPE_A);
	/* Enable the plane */
    vgt_restore_mmio_reg(_REG_DSPACNTR);
    __sreg(vgt, _REG_DSPBCNTR)= VGT_MMIO_READ(pdev, _REG_DSPBLINOFF);
    VGT_MMIO_WRITE(pdev, _REG_DSPBLINOFF, __sreg(vgt, _REG_DSPBCNTR));

    /* Pipe & plane B info */
    /* FIXME: since we support PCH split, so use PCH_DPLL_B */
    if (__vreg(vgt, _REG_PCH_DPLL_B) & DPLL_VCO_ENABLE) //FIXME: or sreg ???
    {
        vgt_write_mmio_reg(_REG_PCH_DPLL_B, __vreg(vgt, (_REG_PCH_DPLL_B & ~DPLL_VCO_ENABLE)));
        VGT_POST_READ(pdev, _REG_PCH_DPLL_B);
        udelay(150);
    }
    vgt_restore_mmio_reg(_REG_PCH_FPB0);
    vgt_restore_mmio_reg(_REG_PCH_FPB1);
	/* Actually enable it */
    vgt_restore_mmio_reg(_REG_PCH_DPLL_B);
    VGT_POST_READ(pdev, _REG_PCH_DPLL_B);
    udelay(150);
    if (VGT_GEN(vgt) >= 4 && !VGT_HAS_PCH_SPLIT(dev)) {
        vgt_restore_mmio_reg(_REG_DPLL_B_MD);
        VGT_POST_READ(pdev, _REG_DPLL_B_MD);
    }
    udelay(150);

	/* Restore mode */
    vgt_restore_mmio_reg(_REG_HTOTAL_B);
    vgt_restore_mmio_reg(_REG_HBLANK_B);
    vgt_restore_mmio_reg(_REG_HSYNC_B);
    vgt_restore_mmio_reg(_REG_VTOTAL_B);
    vgt_restore_mmio_reg(_REG_VBLANK_B);
    vgt_restore_mmio_reg(_REG_VSYNC_B);
    if (!VGT_HAS_PCH_SPLIT(vgt))
        vgt_restore_mmio_reg(_REG_BCLRPAT_B);

    if(VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_PIPEB_DATA_M1);
        vgt_restore_mmio_reg(_REG_PIPEB_DATA_N1);
        vgt_restore_mmio_reg(_REG_PIPEB_LINK_M1);
        vgt_restore_mmio_reg(_REG_PIPEB_LINK_N1);

        vgt_restore_mmio_reg(_REG_FDI_RXB_CTL);
        vgt_restore_mmio_reg(_REG_FDI_TXB_CTL);

        vgt_restore_mmio_reg(_REG_PFB_CTL_1);
        vgt_restore_mmio_reg(_REG_PFB_WIN_SZ);
        vgt_restore_mmio_reg(_REG_PFB_WIN_POS);

        vgt_restore_mmio_reg(_REG_TRANSBCONF);
        vgt_restore_mmio_reg(_REG_TRANS_HTOTAL_B);
        vgt_restore_mmio_reg(_REG_TRANS_HBLANK_B);
        vgt_restore_mmio_reg(_REG_TRANS_HSYNC_B);
        vgt_restore_mmio_reg(_REG_TRANS_VTOTAL_B);
        vgt_restore_mmio_reg(_REG_TRANS_VBLANK_B);
        vgt_restore_mmio_reg(_REG_TRANS_VSYNC_B);
    }

    /* Restore plane info */
    vgt_restore_mmio_reg(_REG_DSPBSIZE);
    vgt_restore_mmio_reg(_REG_DSPBPOS);
    vgt_restore_mmio_reg(_REG_PIPEBSRC);
    vgt_restore_mmio_reg(_REG_DSPBLINOFF);
    vgt_restore_mmio_reg(_REG_DSPBSTRIDE);
    if (VGT_GEN(vgt) >= 4) {
        vgt_restore_mmio_reg(_REG_DSPBSURF);
        vgt_restore_mmio_reg(_REG_DSPBTILEOFF);
    }

    vgt_restore_mmio_reg(_REG_PIPEBCONF);

	vgt_restore_palette(vgt, PIPE_B);
	/* Enable the plane */
    vgt_restore_mmio_reg(_REG_DSPBCNTR);
    __sreg(vgt, _REG_DSPBCNTR)= VGT_MMIO_READ(pdev, _REG_DSPBLINOFF);
    VGT_MMIO_WRITE(pdev, _REG_DSPBLINOFF, __sreg(vgt, _REG_DSPBCNTR));

	/* Cursor state */
    vgt_restore_mmio_reg(_REG_CURAPOS);
    vgt_restore_mmio_reg(_REG_CURACNTR);
    vgt_restore_mmio_reg(_REG_CURABASE);
    vgt_restore_mmio_reg(_REG_CURBPOS);
    vgt_restore_mmio_reg(_REG_CURBCNTR);
    vgt_restore_mmio_reg(_REG_CURBBASE);

    if (VGT_GEN(vgt) == 2)
        vgt_restore_mmio_reg(_REG_CURSIZE);
}


//extern void intel_disable_fbc(struct drm_device *dev);
//static void i915_restore_display(struct drm_device *dev)
static void vgt_restore_display(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;

	/* Display arbitration */
    vgt_restore_mmio_reg(_REG_DSPARB);

	/* Display port ratios (must be done before clock is set) */
    // TODO: snb does not support this
    if (VGT_SUPPORT_INTEGRATED_DP(pdev)) {
        BUG();
    }

	/* This is only meaningful in non-KMS mode */
	/* Don't restore them in KMS mode */
    vgt_restore_modeset_reg(vgt);

	/* CRT state */
	if (VGT_HAS_PCH_SPLIT(vgt))
        vgt_restore_mmio_reg(_REG_PCH_ADPA);
    else
        vgt_restore_mmio_reg(_REG_ADPA);

	/* LVDS state */
    if (VGT_GEN(vgt) >= 4 && !VGT_HAS_PCH_SPLIT(vgt))
        vgt_restore_mmio_reg(_REG_BLC_PWM_CTL2);

    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_PCH_LVDS);
    } else {
        //else if (...) {} TODO: not related current snb platform
        BUG();
    }

    /* TODO: not related current snb platform */
	/*if (!IS_I830(dev) && !IS_845G(dev) && !HAS_PCH_SPLIT(dev))
		I915_WRITE(PFIT_CONTROL, dev_priv->savePFIT_CONTROL);
    */

    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_BLC_PWM_PCH_CTL1);
        vgt_restore_mmio_reg(_REG_BLC_PWM_PCH_CTL2);
        vgt_restore_mmio_reg(_REG_BLC_PWM_CPU_CTL);
        vgt_restore_mmio_reg(_REG_BLC_PWM_CPU_CTL2);
        vgt_restore_mmio_reg(_REG_PCH_PP_ON_DELAYS);
        vgt_restore_mmio_reg(_REG_PCH_PP_OFF_DELAYS);
        vgt_restore_mmio_reg(_REG_PCH_PP_DIVISOR);
        vgt_restore_mmio_reg(_REG_PCH_PP_CONTROL);
        vgt_restore_mmio_reg(_REG_RSTDBYCTL);
    } else {
        //TODO: not related current snb platform
        not_done();
    }

	/* Display Port state */
    if (VGT_SUPPORT_INTEGRATED_DP(vgt)) {
        vgt_restore_mmio_reg(_REG_DP_B);
        vgt_restore_mmio_reg(_REG_DP_C);
        vgt_restore_mmio_reg(_REG_DP_D);
    }

    /* FIXME: how to do this ???
     refer: drivers/gpu/drm/drm_pci.c
     int drm_get_pci_dev
    */
	//intel_disable_fbc(dev);

    /* TODO: snb does not support fbc */
    //if (VGT_I915_HAS_FBC(vgt)) { ... }

	/* VGA state */
    if (VGT_HAS_PCH_SPLIT(vgt))
        vgt_restore_mmio_reg(_REG_CPU_VGACNTRL);
    else
        vgt_restore_mmio_reg(_REG_VGACNTRL);

    vgt_restore_mmio_reg(_REG_VGA0);
    vgt_restore_mmio_reg(_REG_VGA1);
    vgt_restore_mmio_reg(_REG_VGA_PD);
    VGT_POST_READ(pdev, _REG_VGA_PD);
    udelay(150);

    vgt_restore_vga(vgt);
}

void
vgt_i2c_reset(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    if (VGT_HAS_PCH_SPLIT(vgt))
        vgt_write_mmio_reg(_REG_PCH_GMBUS0, 0);
    else
        vgt_write_mmio_reg(_REG_GMBUS0, 0);
}

//pci_save_state()
int vgt_pci_save_state(struct vgt_device *vgt)
{
    int i;
    for (i = 0; i < 16; i++)
        vgt_emulate_cfg_read(vgt,  i * 4, vgt->state.cfg_space, 4);

    /* FIXME: gen6 is neither pcie or pcix device */
    //pci_save_pcie_state()
    //pci_save_pcix_state()

    return 0;
}

static u8 vgt_read_indexed(struct vgt_device *vgt, u16 index_port, u16 data_port, u8 index)
{
    struct pgt_device *pdev = vgt->pdev;
    vgt_write_mmio_reg_8(index_port, index);
    return vgt_read_mmio_reg_8(data_port);
}

static void vgt_write_indexed(struct vgt_device *vgt, u16 index_port, u16 data_port, u8 off, u8 val)
{
    struct pgt_device *pdev = vgt->pdev;
    vgt_write_mmio_reg_8(index_port, off);
    vgt_write_mmio_reg_8(data_port, val);
}

static u8 vgt_read_ar(struct vgt_device *vgt, u16 st01, u8 reg, u16 palette_enable)
{
    struct pgt_device *pdev = vgt->pdev;
    vgt_read_mmio_reg_8(st01);
    vgt_write_mmio_reg_8(_REG_VGA_AR_INDEX, (reg | palette_enable));
    return  vgt_read_mmio_reg_8(_REG_VGA_AR_DATA_READ);
}

static void vgt_save_vga(struct vgt_device *vgt)
{
	int i;
	u16 cr_index, cr_data, st01;
    struct pgt_device *pdev = vgt->pdev;
    vgt_state_t *vgt_state = &vgt->state;

	/* VGA color palette registers */
	//dev_priv->saveDACMASK = I915_READ8(VGA_DACMASK);
    vgt_save_mmio_reg_8(_REG_VGA_DACMASK);

	/* MSR bits */
    vgt_save_mmio_reg_8(_REG_VGA_MSR_READ);
    if (__vreg(vgt, _REG_VGA_MSR_READ) & VGA_MSR_CGA_MODE) {
        cr_index = _REG_VGA_CR_INDEX_CGA;
        cr_data = _REG_VGA_CR_DATA_CGA;
        st01 = _REG_VGA_ST01_CGA;
    } else {
        cr_index = _REG_VGA_CR_INDEX_MDA;
        cr_data = _REG_VGA_CR_DATA_MDA;
        st01 = _REG_VGA_ST01_MDA;
    }

	/* CRT controller regs */
    vgt_write_indexed(vgt, cr_index, cr_data, 0x11,
            vgt_read_indexed(vgt, cr_index, cr_data, 0x11));
    for (i = 0; i <= 0x24; i++)
        vgt_state->saveCR[i] =
            vgt_read_indexed(vgt, cr_index, cr_data, i);
	/* Make sure we don't turn off CR group 0 writes */
    vgt_state->saveCR[0x11] &= ~0x80;

	/* Attribute controller registers */
    vgt_read_mmio_reg_8(st01);
    vgt_save_mmio_reg_8(_REG_VGA_AR_INDEX);
    for (i = 0; i <= 0x14; i++)
        vgt_state->saveAR[i] = vgt_read_ar(vgt, st01, i, 0);
    vgt_read_mmio_reg_8(st01);
    vgt_restore_mmio_reg_8(_REG_VGA_AR_INDEX);
    vgt_read_mmio_reg_8(st01);

	/* Graphics controller registers */
	for (i = 0; i < 9; i++)
        vgt_state->saveGR[i] =
            vgt_read_indexed(vgt, _REG_VGA_GR_INDEX, _REG_VGA_GR_DATA, i);

	vgt_state->saveGR[0x10] =
		vgt_read_indexed(vgt, _REG_VGA_GR_INDEX, _REG_VGA_GR_DATA, 0x10);
	vgt_state->saveGR[0x11] =
		vgt_read_indexed(vgt, _REG_VGA_GR_INDEX, _REG_VGA_GR_DATA, 0x11);
	vgt_state->saveGR[0x18] =
		vgt_read_indexed(vgt, _REG_VGA_GR_INDEX, _REG_VGA_GR_DATA, 0x18);

	/* Sequencer registers */
	for (i = 0; i < 8; i++)
		vgt_state->saveSR[i] =
			vgt_read_indexed(vgt, _REG_VGA_SR_INDEX, _REG_VGA_SR_DATA, i);
}

/* FIXME: do we need to save MMIO, do we need to update vregs ??? */
static void vgt_save_palette(struct vgt_device *vgt, enum vgt_pipe pipe)
{
    int i;
    struct pgt_device *pdev = vgt->pdev;
    unsigned long reg = (pipe == PIPE_A ? _REG_PALETTE_A : _REG_PALETTE_B);

    if (!vgt_pipe_enabled(vgt, pipe))
        return;

    if (VGT_HAS_PCH_SPLIT(vgt))
        reg = (pipe == PIPE_A) ? _REG_LGC_PALETTE_A : _REG_LGC_PALETTE_B;

    for (i = 0; i < 256; i++)
        vgt_save_mmio_reg(reg + (i << 2));
}

static void vgt_save_modeset_reg(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    int i;

    if (vgt_driver_check_feature(vgt, VGT_DRIVER_MODESET))
        return;

	/* Cursor state */
    vgt_save_mmio_reg(_REG_CURACNTR);
    vgt_save_mmio_reg(_REG_CURAPOS);
    vgt_save_mmio_reg(_REG_CURABASE);
    vgt_save_mmio_reg(_REG_CURBCNTR);
    vgt_save_mmio_reg(_REG_CURBPOS);
    vgt_save_mmio_reg(_REG_CURBBASE);
	//if (IS_GEN2(dev))
	//	dev_priv->saveCURSIZE = I915_READ(CURSIZE);

    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PCH_DREF_CONTROL);
        vgt_save_mmio_reg(_REG_DISP_ARB_CTL);
    }

	/* Pipe & plane A info */
    vgt_save_mmio_reg(_REG_PIPEACONF);
    vgt_save_mmio_reg(_REG_PIPEASRC);
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PCH_FPA0);
        vgt_save_mmio_reg(_REG_PCH_FPA1);
        vgt_save_mmio_reg(_REG_PCH_DPLL_A);
    } else {
        vgt_save_mmio_reg(_REG_FPA0);
        vgt_save_mmio_reg(_REG_FPA1);
        vgt_save_mmio_reg(_REG_DPLL_A);
    }

    if (VGT_GEN(vgt) >= 4 && !VGT_HAS_PCH_SPLIT(vgt))
        vgt_save_mmio_reg(_REG_DPLL_A_MD);

    vgt_save_mmio_reg(_REG_HTOTAL_A);
    vgt_save_mmio_reg(_REG_HBLANK_A);
    vgt_save_mmio_reg(_REG_HSYNC_A);
    vgt_save_mmio_reg(_REG_VTOTAL_A);
    vgt_save_mmio_reg(_REG_VBLANK_A);
    vgt_save_mmio_reg(_REG_VSYNC_A);

    if (!VGT_HAS_PCH_SPLIT(vgt))
        vgt_save_mmio_reg(_REG_BCLRPAT_A);

    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PIPEA_DATA_M1);
        vgt_save_mmio_reg(_REG_PIPEA_DATA_N1);
        vgt_save_mmio_reg(_REG_PIPEA_LINK_M1);
        vgt_save_mmio_reg(_REG_PIPEA_LINK_N1);

        vgt_save_mmio_reg(_REG_FDI_TXA_CTL);
        vgt_save_mmio_reg(_REG_FDI_RXA_CTL);

        vgt_save_mmio_reg(_REG_PFA_CTL_1);
        vgt_save_mmio_reg(_REG_PFA_WIN_SZ);
        vgt_save_mmio_reg(_REG_PFA_WIN_POS);

        vgt_save_mmio_reg(_REG_TRANSACONF);
        vgt_save_mmio_reg(_REG_TRANS_HTOTAL_A);
        vgt_save_mmio_reg(_REG_TRANS_HBLANK_A);
        vgt_save_mmio_reg(_REG_TRANS_HSYNC_A);
        vgt_save_mmio_reg(_REG_TRANS_VTOTAL_A);
        vgt_save_mmio_reg(_REG_TRANS_VBLANK_A);
        vgt_save_mmio_reg(_REG_TRANS_VSYNC_A);
    }

    vgt_save_mmio_reg(_REG_DSPACNTR);
    vgt_save_mmio_reg(_REG_DSPASTRIDE);
    vgt_save_mmio_reg(_REG_DSPASIZE);
    vgt_save_mmio_reg(_REG_DSPAPOS);
    vgt_save_mmio_reg(_REG_DSPALINOFF);

    if (VGT_GEN(vgt) >= 4) {
        vgt_save_mmio_reg(_REG_DSPASURF);
        vgt_save_mmio_reg(_REG_DSPATILEOFF);
    }

    vgt_save_palette(vgt, PIPE_A);
    vgt_save_mmio_reg(_REG_PIPEASTAT);

	/* Pipe & plane B info */
    vgt_save_mmio_reg(_REG_PIPEBCONF);
    vgt_save_mmio_reg(_REG_PIPEBSRC);
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PCH_FPB0);
        vgt_save_mmio_reg(_REG_PCH_FPB1);
        vgt_save_mmio_reg(_REG_PCH_DPLL_B);
    } else {
        vgt_save_mmio_reg(_REG_FPB0);
        vgt_save_mmio_reg(_REG_FPB1);
        vgt_save_mmio_reg(_REG_DPLL_B);
    }

    if (VGT_GEN(vgt) >= 4 && !VGT_HAS_PCH_SPLIT(vgt))
        vgt_save_mmio_reg(_REG_DPLL_B_MD);

    vgt_save_mmio_reg(_REG_HTOTAL_B);
    vgt_save_mmio_reg(_REG_HBLANK_B);
    vgt_save_mmio_reg(_REG_HSYNC_B);
    vgt_save_mmio_reg(_REG_VTOTAL_B);
    vgt_save_mmio_reg(_REG_VBLANK_B);
    vgt_save_mmio_reg(_REG_VSYNC_B);

    if (!VGT_HAS_PCH_SPLIT(vgt))
        vgt_save_mmio_reg(_REG_BCLRPAT_B);

    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PIPEB_DATA_M1);
        vgt_save_mmio_reg(_REG_PIPEB_DATA_N1);
        vgt_save_mmio_reg(_REG_PIPEB_LINK_M1);
        vgt_save_mmio_reg(_REG_PIPEB_LINK_N1);

        vgt_save_mmio_reg(_REG_FDI_TXB_CTL);
        vgt_save_mmio_reg(_REG_FDI_RXB_CTL);

        vgt_save_mmio_reg(_REG_PFB_CTL_1);
        vgt_save_mmio_reg(_REG_PFB_WIN_SZ);
        vgt_save_mmio_reg(_REG_PFB_WIN_POS);

        vgt_save_mmio_reg(_REG_TRANSBCONF);
        vgt_save_mmio_reg(_REG_TRANS_HTOTAL_B);
        vgt_save_mmio_reg(_REG_TRANS_HBLANK_B);
        vgt_save_mmio_reg(_REG_TRANS_HSYNC_B);
        vgt_save_mmio_reg(_REG_TRANS_VTOTAL_B);
        vgt_save_mmio_reg(_REG_TRANS_VBLANK_B);
        vgt_save_mmio_reg(_REG_TRANS_VSYNC_B);
    }

    vgt_save_mmio_reg(_REG_DSPBCNTR);
    vgt_save_mmio_reg(_REG_DSPBSTRIDE);
    vgt_save_mmio_reg(_REG_DSPBSIZE);
    vgt_save_mmio_reg(_REG_DSPBPOS);
    vgt_save_mmio_reg(_REG_DSPBLINOFF);

    if (VGT_GEN(vgt) >= 4) {
        vgt_save_mmio_reg(_REG_DSPBSURF);
        vgt_save_mmio_reg(_REG_DSPBTILEOFF);
    }

    vgt_save_palette(vgt, PIPE_B);
    vgt_save_mmio_reg(_REG_PIPEBSTAT);

	/* Fences */
    /* TODO: only save gen 6 now */
    for (i = 0; i< 16; i++)
        vgt_save_mmio_reg_64(_REG_FENCE_0_LOW + i * 8);
}

static int vgt_save_display(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;

	/* Display arbitration control */
	//dev_priv->saveDSPARB = I915_READ(DSPARB);
    vgt_save_mmio_reg(_REG_DSPARB);

	/* This is only meaningful in non-KMS mode */
	/* Don't save them in KMS mode */
    // TODO & FIXME: since by default support DRIVER_MODESET,
    //        this function will do nothing but returns
	//i915_save_modeset_reg(dev);
    vgt_save_modeset_reg(vgt);

	/* CRT state */
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_save_mmio_reg(_REG_PCH_ADPA);
    } else {
        vgt_save_mmio_reg(_REG_ADPA);
    }

    if (VGT_HAS_PCH_SPLIT(pdev)) {
        vgt_save_mmio_reg(_REG_PCH_PP_CONTROL);
        vgt_save_mmio_reg(_REG_BLC_PWM_PCH_CTL1);
        vgt_save_mmio_reg(_REG_BLC_PWM_PCH_CTL2);
        vgt_save_mmio_reg(_REG_BLC_PWM_CPU_CTL);
        vgt_save_mmio_reg(_REG_BLC_PWM_CPU_CTL2);
        vgt_save_mmio_reg(_REG_PCH_LVDS);
    } else {
        /* TODO: snb gfx save should not go to here */
        not_done();
    }

    /* TODO: will not go into this */
    /*
	if (!IS_I830(dev) && !IS_845G(dev) && !HAS_PCH_SPLIT(dev))
		dev_priv->savePFIT_CONTROL = I915_READ(PFIT_CONTROL);
    */

    if (VGT_HAS_PCH_SPLIT(pdev)) {
        vgt_save_mmio_reg(_REG_PCH_PP_ON_DELAYS);
        vgt_save_mmio_reg(_REG_PCH_PP_OFF_DELAYS);
        vgt_save_mmio_reg(_REG_PCH_PP_DIVISOR);
    } else {
        /* TODO: will not go into this */
        not_done();
    }

    /* TODO: will not go into this */
	/* Display Port state */
    if (VGT_SUPPORT_INTEGRATED_DP(pdev)) {
        not_done();
    }

	/* Only save FBC state on the platform that supports FBC */
    if (VGT_I915_HAS_FBC(pdev)) {
        not_done();
    }

	/* VGA state */
    vgt_save_mmio_reg(_REG_VGA0);
    vgt_save_mmio_reg(_REG_VGA1);
    vgt_save_mmio_reg(_REG_VGA_PD);
    if (VGT_HAS_PCH_SPLIT(pdev))
        vgt_save_mmio_reg(_REG_CPU_VGACNTRL);
    else
        vgt_save_mmio_reg(_REG_VGACNTRL);

    vgt_save_vga(vgt);

    return 0;
}

/* This function will be called from vgt_context.c */
int vgt_save_state(struct vgt_device *vgt)
{
    int i;
    struct pgt_device *pdev = vgt->pdev;

    /* put 2 functions in i915_drm_freeze here */
    // FIXME: is this totally software stuff ???
	//drm_kms_helper_poll_disable(dev);

    // TODO: did not do pcie/pcix save, since 00:02.0 exposed as PCI device
    //pci_save_state()
    vgt_pci_save_state(vgt);

    /* i915_save_state go from here */

	//pci_read_config_byte(dev->pdev, LBB, &dev_priv->saveLBB);
    vgt_emulate_cfg_read(vgt, _REG_LBB, vgt->state.cfg_space, 1);

    /* Any lock we need ? */
	//mutex_lock(&dev->struct_mutex);

	/* Hardware status page */
    vgt_save_mmio_reg(_REG_HWS_PGA);

	vgt_save_display(vgt);

    if (VGT_HAS_PCH_SPLIT(pdev)) {
        vgt_save_mmio_reg(_REG_DEIER);
        vgt_save_mmio_reg(_REG_DEIMR);
        vgt_save_mmio_reg(_REG_GTIER);
        vgt_save_mmio_reg(_REG_GTIMR);
        vgt_save_mmio_reg(_REG_FDI_RXA_IMR);
        vgt_save_mmio_reg(_REG_FDI_RXB_IMR);
        vgt_save_mmio_reg(_REG_RSTDBYCTL);
        /* FIXME: in i915 side, it is called PCH_PORT_HOTPLUG */
        vgt_save_mmio_reg(_REG_SHOTPLUG_CTL);
    } else {
        vgt_save_mmio_reg(_REG_IER);
        vgt_save_mmio_reg(_REG_IMR);
    }

    /* supposed will not go into this */
    /*
	if (IS_IRONLAKE_M(dev))
		ironlake_disable_drps(dev);
    */

    /* GT_THREAD_P_REQ - GT Thread P-State Request
     * FIXME: we need to change power management ??? */
    /*
	if (IS_GEN6(dev))
		gen6_disable_rps(dev);
    */

	/* Cache mode state */
    vgt_save_mmio_reg(_REG_CACHE_MODE_0);

	/* Memory Arbitration state */
    vgt_save_mmio_reg(_REG_MI_ARB_STATE);

	/* Scratch space */
    for (i = 0; i < 16; i++) {
        vgt_save_mmio_reg(_REG_SWF00 + (i << 2));
        vgt_save_mmio_reg(_REG_SWF10 + (i << 2));
    }
	for (i = 0; i < 3; i++)
		vgt_save_mmio_reg(_REG_SWF30 + (i << 2));

    /* FIXME: how we need any lock ??? */
	//mutex_unlock(&dev->struct_mutex);

    return 0;
}

/* FIXME: what about plane B ??? */
static void  vgt_flush_display_plane(struct vgt_device *vgt)
{
    vgt_reg_t reg;
    struct pgt_device *pdev = vgt->pdev;
    printk("vGT: %s\n", __func__);
    reg = VGT_MMIO_READ(pdev, _REG_DSPALINOFF);
    VGT_MMIO_WRITE(pdev, _REG_DSPALINOFF, reg);
    reg = VGT_MMIO_READ(pdev, _REG_DSPASURF);
    VGT_MMIO_WRITE(pdev, _REG_DSPASURF, reg);
}

/* This function will be called from vgt_context.c */
//int i915_restore_state(struct drm_device *dev)
int vgt_restore_state(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    int i;
    char *cfg_space;

	//pci_write_config_byte(dev->pdev, LBB, dev_priv->saveLBB);
    cfg_space = &vgt->state.cfg_space[0];
    vgt_emulate_cfg_write(vgt, _REG_LBB,
            (void*)(cfg_space + ((_REG_LBB) & ~3)), 1);

    /* FIXME: any lock we need ???  */
	//mutex_lock(&dev->struct_mutex);

    vgt_restore_mmio_reg(_REG_HWS_PGA);

    vgt_restore_display(vgt);

	/* Interrupt state */
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_DEIER);
        vgt_restore_mmio_reg(_REG_DEIMR);
        vgt_restore_mmio_reg(_REG_GTIER);
        vgt_restore_mmio_reg(_REG_GTIMR);
        vgt_restore_mmio_reg(_REG_FDI_RXA_IMR);
        vgt_restore_mmio_reg(_REG_FDI_RXB_IMR);
        vgt_restore_mmio_reg(_REG_SHOTPLUG_CTL);
    } else {
        /* FIXME: Is this MMIO ??? */
        vgt_restore_mmio_reg(_REG_IER);
        vgt_restore_mmio_reg(_REG_IMR);
    }

    /* FIXME: do we need lock ??? */
    //mutex_unlock(&dev->struct_mutex);

    /* TODO: FIXME: how to check these supported feature ??? and do init gating */
	//if (drm_core_check_feature(dev, DRIVER_MODESET))
	//	intel_init_clock_gating(dev);
    /* FIXME: In intel_init_clock_gating we saw display A surface activated */
    /* FIXME: refer ironlake_update_plane() */
#define vgt_restore_sreg(reg)       \
    do {    \
        VGT_MMIO_WRITE(vgt->pdev, (reg), __sreg(vgt, (reg))); \
    } while (0);

    /* FIXME: this part of code come from ironlake_update_plane */
    printk("vGT: restoring DSPAXXX ...\n");
    vgt_restore_sreg(_REG_DSPACNTR);
    vgt_restore_sreg(_REG_DSPASTRIDE);
    vgt_restore_sreg(_REG_DSPASURF);
    vgt_restore_sreg(_REG_DSPATILEOFF);
    vgt_restore_sreg(_REG_DSPALINOFF);
    VGT_MMIO_READ(vgt->pdev, _REG_DSPACNTR);
    printk("vGT: restoring DSPAXXX done!\n");

    /* FIXME: snb is ironlake ??? */
	//if (IS_IRONLAKE_M(dev)) {
	//	ironlake_enable_drps(dev);
	//	intel_init_emon(dev);
	//}

    if (VGT_GEN(vgt) == 6) {
        /* TODO: power management ? */
		//gen6_enable_rps(dev_priv);
		//gen6_update_ring_freq(dev_priv);
        printk("%s: %d: not emulated yet\n", __func__, __LINE__);
    }

    /* TODO: lock we need ? */
	//mutex_lock(&dev->struct_mutex);

	/* Cache mode state */
    vgt_write_mmio_reg(_REG_CACHE_MODE_0, __vreg(vgt, _REG_CACHE_MODE_0) | 0xffff0000);

	/* Memory arbitration state */
    vgt_write_mmio_reg(_REG_MI_ARB_STATE, __vreg(vgt, _REG_MI_ARB_STATE) | 0xffff0000);

    /* FIXME: mmio ??? */
	for (i = 0; i < 16; i++) {
        vgt_restore_mmio_reg(_REG_SWF00 + (i << 2));
        vgt_restore_mmio_reg(_REG_SWF10 + (i << 2));
    }
    for (i = 0; i < 3; i++) {
        vgt_restore_mmio_reg(_REG_SWF30 + (i << 2));
    }

    /* TODO: lock we need ? */
	//mutex_unlock(&dev->struct_mutex);

    /* FIXME: do we need to to do this ??? */
    vgt_i2c_reset(vgt);

    vgt_flush_display_plane(vgt);

    return 0;
}

