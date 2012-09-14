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
#include <linux/kgdb.h>
#include <asm/bitops.h>
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>
#include <xen/vgt.h>
#include <xen/vgt-parser.h>
#include "vgt_drv.h"

#undef VGT_DEBUG
#define vgt_printk(fmt, args...)	\
	do {														\
		printk("%s: %d: vGT: "fmt"\n", __func__, __LINE__, ##args);	\
	} while (0)

#define show_sreg(vgt, reg)		\
	do {							\
		vgt_reg_t val;				\
		ASSERT(!((reg) & 0x3));		\
		val = __sreg(vgt, (reg));		\
		vgt_printk("vgt(%d): sreg(%08x) value(%08x)", vgt->vm_id, (reg), val); \
	} while (0)

/* copied & modified from include/drm/drmP.h */
#define VGT_DRIVER_MODESET     0x2000
u32 __vgt_driver_cap = VGT_DRIVER_MODESET;
//#define vgt_driver_check_feature(feature)   (__vgt_driver_cap & (feature))

#if 0
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
#endif

/* FIXME: snb_devinfo copied from  */
/* static const struct intel_device_info intel_sandybridge_d_info = {
 */
static struct vgt_intel_device_info snb_devinfo = {
    .gen = 6,
	.pch = PCH_CPT,
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
#define VGT_PCH(pdev) (vgt_devinfo->pch)
#define VGT_IS_IVB(pdev) (vgt_devinfo->is_ivybridge)

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

#define vgt_restore_mmio_reg(offset)                            \
    do {                                                        \
        if (!reg_is_owner(vgt, (offset))) {                     \
            dprintk("vGT: restore non-owner reg(%x)\n", (offset)); \
            break;                                              \
        }                                                       \
        __sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg(vgt, (offset))); \
        VGT_MMIO_WRITE(pdev, (offset), __sreg(vgt, (offset)));      \
    } while(0)

#define vgt_restore_sreg(reg)       \
    do {    \
        VGT_MMIO_WRITE(vgt->pdev, (reg), __sreg(vgt, (reg))); \
    } while (0);

#define vgt_restore_mmio_reg_64(offset)                         \
    do {                                                        \
        if (!reg_is_owner(vgt, (offset))) {                     \
            dprintk("vGT: restore non-owner reg(%x)\n", (offset)); \
            break;                                              \
        }                                                       \
        __sreg(vgt, (offset)) = mmio_g2h_gmadr(vgt, (offset), __vreg(vgt, (offset)));  \
        __sreg(vgt, (offset) + 4) = mmio_g2h_gmadr(vgt, (offset) + 4, __vreg(vgt, (offset) + 4));  \
        VGT_MMIO_WRITE_BYTES(pdev, (offset), __sreg64(vgt, (offset)), 8);             \
    } while(0)

#define vgt_restore_mmio_reg_8(offset) \
    do {                                    \
        if (!reg_is_owner(vgt, (offset))) {                     \
            dprintk("vGT: restore non-owner reg(%x)\n", (offset)); \
            break;                                              \
        }                                                       \
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
        __vreg(vgt, (offset)) = mmio_h2g_gmadr(vgt, (offset), __sreg(vgt, (offset)));   \
        __vreg(vgt, (offset) + 4) = mmio_h2g_gmadr(vgt, (offset) + 4, __sreg(vgt, (offset) + 4));   \
    } while(0)

#define not_done()              \
    do {                        \
        printk(KERN_WARNING"%s: %d:  not done yet!\n", __func__, __LINE__);   \
        BUG();                  \
    } while(0)

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
    //vgt_restore_modeset_reg(vgt);

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

#if 0
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

    //vgt_restore_vga(vgt);
#endif
}

/* FIXME: No need to operate on i2c ? */
#if 0
void
vgt_i2c_reset(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    if (VGT_HAS_PCH_SPLIT(vgt))
        vgt_write_mmio_reg(_REG_PCH_GMBUS0, 0);
    else
        vgt_write_mmio_reg(_REG_GMBUS0, 0);
}
#endif

/* No need to save PCI configure */
#if 0
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
#endif

#if 0
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
#endif

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
    //vgt_save_modeset_reg(vgt);

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
		BUG();
    //vgt_save_vga(vgt);

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
    /* FIXME: no need to save pci state */
#if 0
    vgt_pci_save_state(vgt);
#endif

    /* i915_save_state go from here */

	//pci_read_config_byte(dev->pdev, LBB, &dev_priv->saveLBB);
    /* FIXME: no need to save PCI configure */
#if 0
    vgt_emulate_cfg_read(vgt, _REG_LBB, vgt->state.cfg_space, 1);
#endif

    /* Any lock we need ? */
	//mutex_lock(&dev->struct_mutex);

	/* Hardware status page */
    /* FIXME: _REG_HWS_PGA is only used in i915 for dmah, this
     * not used by any other vGT code, it seems like a legacy register
     */
    vgt_save_mmio_reg(_REG_HWS_PGA);

	vgt_save_display(vgt);

    if (VGT_HAS_PCH_SPLIT(pdev)) {
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

/* intel_flush_display_plane */
static void vgt_flush_display_plane(struct vgt_device *vgt,
		enum vgt_plane plane)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t reg_data;

	ASSERT(plane < PLANE_C);

	vgt_printk("flush display %s", VGT_PLANE_NAME(plane));
	reg_data = VGT_MMIO_READ(pdev, VGT_DSPLINOFF(plane));
	VGT_MMIO_WRITE(pdev, VGT_DSPLINOFF(plane), reg_data);
	reg_data = VGT_MMIO_READ(pdev, VGT_DSPSURF(plane));
	VGT_MMIO_WRITE(pdev, VGT_DSPSURF(plane), reg_data);
}

/* This function will be called from vgt_context.c */
//int i915_restore_state(struct drm_device *dev)
int vgt_restore_state(struct vgt_device *vgt)
{
    struct pgt_device *pdev = vgt->pdev;
    int i;
    //char *cfg_space;

	//pci_write_config_byte(dev->pdev, LBB, dev_priv->saveLBB);
    /* FIXME: no need to restore pci configure */
#if 0
    cfg_space = &vgt->state.cfg_space[0];
    vgt_emulate_cfg_write(vgt, _REG_LBB,
            (void*)(cfg_space + ((_REG_LBB) & ~3)), 1);
#endif

    /* FIXME: any lock we need ???  */
	//mutex_lock(&dev->struct_mutex);
    vgt_restore_mmio_reg(_REG_HWS_PGA);

    vgt_restore_display(vgt);

	/* Interrupt state */
    if (VGT_HAS_PCH_SPLIT(vgt)) {
        vgt_restore_mmio_reg(_REG_FDI_RXA_IMR);
        vgt_restore_mmio_reg(_REG_FDI_RXB_IMR);
    }

    /* FIXME: do we need lock ??? */
    //mutex_unlock(&dev->struct_mutex);

    /* TODO: FIXME: how to check these supported feature ??? and do init gating */
	//if (drm_core_check_feature(dev, DRIVER_MODESET))
	//	intel_init_clock_gating(dev);
    /* FIXME: In intel_init_clock_gating we saw display A surface activated */
    /* FIXME: refer ironlake_update_plane() */

    /* FIXME: this part of code come from ironlake_update_plane */
    printk("vGT: restoring DSPAXXX ...\n");
    vgt_restore_sreg(_REG_DSPACNTR);
    vgt_restore_sreg(_REG_DSPASTRIDE);
    vgt_restore_sreg(_REG_DSPASURF);
    vgt_restore_sreg(_REG_DSPATILEOFF);
    vgt_restore_sreg(_REG_DSPALINOFF);
    VGT_POST_READ(vgt->pdev, _REG_DSPACNTR);
    printk("vGT: restoring DSPAXXX done!\n");

    printk("vGT: restoring DSPBXXX ...\n");
    vgt_restore_sreg(_REG_DSPBCNTR);
    vgt_restore_sreg(_REG_DSPBSTRIDE);
    vgt_restore_sreg(_REG_DSPBSURF);
    vgt_restore_sreg(_REG_DSPBTILEOFF);
    vgt_restore_sreg(_REG_DSPBLINOFF);
    VGT_POST_READ(vgt->pdev, _REG_DSPACNTR);
    printk("vGT: restoring DSPBXXX done!\n");

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

    /* FIXME: left i2c what it remains */
#if 0
    vgt_i2c_reset(vgt);
#endif

    vgt_flush_display_plane(vgt, PIPE_A);
    vgt_flush_display_plane(vgt, PIPE_B);

    return 0;
}

/*
 * Display mode setting
 *
 * Currently we only support mode setting for lvds
 */
#define for_each_snb_pipe(p) for ((p) = 0; (p) < I915_MAX_PIPES - 1; (p)++)
static void  vgt_lvds_mode_fixup(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	struct pgt_device *pdev = vgt->pdev;
	enum vgt_pipe pipe;

	/* FIXME: copied from intel_lvds.c : intel_lvds_mode_fixup()
	 *
	 * Enable automatic panel scaling for non-native modes so that they fill
	 * the screen.  Should be enabled before the pipe is enabled, according
	 * to register description and PRM.
	 * Change the value here to see the borders for debugging
	 */
	/* FIXME: cannot find it in Bspec,legacy register ?  */
	for_each_snb_pipe(pipe) {
		VGT_MMIO_WRITE(pdev, VGT_BCLRPAT(pipe), 0);
	}
}

/* copied this from drivers/gpu/drm/i915/intel_drv.h
 * FIXME: DO NOT call this in intr env */
static __inline__ bool vgt_can_sleep(void)
{
	//if (in_atomic() || in_dbg_master() || irqs_disabled())
		return false;
	//return true;
}

#define _wait_for(COND, MS, W) ({	\
	int ret__ = 0;					\
	int ms__ = MS;					\
									\
	while (!(COND) && ms__--)		\
		mdelay(1);					\
									\
	if (!ms__ && !(COND))			\
		ret__ = -ETIMEDOUT;			\
	ret__;							\
})
#define wait_for(COND, MS) _wait_for(COND, MS, 1)
//#define wait_for(COND, MS) (udelay(1000), 0)

#if 0
static void vgt_lvds_disable_encoder(struct vgt_device *vgt)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	/* disable backlight */
	if(VGT_HAS_PCH_SPLIT(pdev)) {
		reg_data = VGT_MMIO_READ(pdev, _REG_BLC_PWM_CPU_CTL);
		reg_data &= ~VGT_BACKLIGHT_DUTY_CYCLE_MASK;
		VGT_MMIO_WRITE(pdev, _REG_BLC_PWM_CPU_CTL, reg_data);
	} else {
		BUG();
	}

	/* disable panel power */
	reg_data = VGT_MMIO_READ(pdev,_REG_PCH_PP_CONTROL);
	reg_data &= ~_REGBIT_POWER_TARGET_ON;
	VGT_MMIO_WRITE(pdev, _REG_PCH_PP_CONTROL, reg_data);
	if (wait_for((VGT_MMIO_READ(pdev, _REG_PCH_PP_STATUS) & _REGBIT_PANEL_POWER_ON) == 0, 1000)) {
		vgt_printk("time out for waiting panel power off!");
	}

	/* PFIT_CONTROL is used for platfroms without PCH */
	/*
	if (intel_lvds->pfit_control) {
		I915_WRITE(PFIT_CONTROL, 0);
		intel_lvds->pfit_dirty = true;
	}
	*/

	/* Disable lvds port */
	vgt_printk();
	reg_data = VGT_MMIO_READ(pdev, _REG_PCH_LVDS);
	reg_data &= ~_REGBIT_LVDS_PORT_ENABLE;
	VGT_MMIO_WRITE(pdev, _REG_PCH_LVDS, reg_data);
	VGT_POST_READ(pdev, _REG_PCH_LVDS);
}
#endif


void vgt_wait_for_vblank(struct vgt_device *vgt, enum vgt_pipe pipe)
{
	vgt_reg_t o_cnt, n_cnt;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	reg = VGT_PIPE_FRMCOUNT(pipe);
	o_cnt = VGT_MMIO_READ(pdev, reg);
	mdelay(50);
	n_cnt = VGT_MMIO_READ(pdev, reg);
	if (n_cnt != o_cnt) {
		vgt_printk("vblank on %s done!", VGT_PIPE_NAME(pipe));
		return;
	} else {
		/* For debug purpose */
#if 0
		vgt_reg_t deier, deiir, deimr, pipeconf,
		deier = VGT_MMIO_READ(pdev, _REG_DEIER);
		deimr = VGT_MMIO_READ(pdev, _REG_DEIMR);
		deiir = VGT_MMIO_READ(pdev, _REG_DEIIR);
		pipeconf = VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe));
		vgt_printk("deier(%08x), deiir(%08x), deimr(%08x), pipeconf(%08x)"
				" delay another 50 ms...",
				deier, deiir, deimr, pipeconf);
#endif
		vgt_printk("vblank on %s time-out!", VGT_PIPE_NAME(pipe));
	}
}

static void vgt_assert_planes_disabled(struct vgt_device *vgt,
		enum vgt_plane plane)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	/* Planes are fixed with the pipe on SNB */
	if (VGT_HAS_PCH_SPLIT(pdev)) {
		reg_data = VGT_MMIO_READ(pdev, VGT_DSPCNTR(plane));
		if (reg_data & _REGBIT_PRIMARY_PLANE_ENABLE) {
			vgt_printk("assertion failed, %s should be disabled.",
					VGT_PLANE_NAME(plane));
		}
		return;
	}

	/* Otherwise we need to check both planes against the pipe */
	/* TODO: check assert_planes_disabled() for this part */
	BUG();
}

void vgt_wait_for_pipe_off(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	if (VGT_GEN(pdev) >= 4) {
		reg = VGT_PIPECONF(pipe);
		if (wait_for((VGT_MMIO_READ(pdev, reg) & _REGBIT_PIPE_STAT_ENABLED) == 0, 100))
			vgt_printk("%s pipe_off wait time out!",
					VGT_PIPE_NAME(pipe));
	} else
		BUG();
}

static void vgt_cpt_phase_pointer_disable(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	reg_data =  VGT_MMIO_READ(pdev, _REG_SOUTH_CHICKEN1);

	reg_data &= ~(VGT_FDI_PHASE_SYNC_EN(pipe));
	VGT_MMIO_WRITE(pdev, _REG_SOUTH_CHICKEN1, reg_data); /* once to disable... */
	reg_data &= ~(VGT_FDI_PHASE_SYNC_OVR(pipe));
	VGT_MMIO_WRITE(pdev, _REG_SOUTH_CHICKEN1, reg_data); /* then again to lock */

	VGT_POST_READ(pdev, _REG_SOUTH_CHICKEN1);
}

static void vgt_cpt_phase_pointer_enable(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	/* FIXME: no need to consider sreg ? */
	ASSERT(pipe < PIPE_C);
	reg = _REG_SOUTH_CHICKEN1;

	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data |= VGT_FDI_PHASE_SYNC_OVR(pipe);
	VGT_MMIO_WRITE(pdev, reg, reg_data); /* once to unlock... */
	reg_data |= VGT_FDI_PHASE_SYNC_EN(pipe);
	VGT_MMIO_WRITE(pdev, reg, reg_data); /* then again to enable */
	VGT_POST_READ(pdev, reg);
}

static void vgt_ironlake_fdi_disable(struct vgt_device *vgt, enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	/* disable CPU FDI tx and PCH FDI rx */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_FDI_TX_ENABLE));
	VGT_POST_READ(pdev, reg);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~(0x7 << 16);
	reg_data |= (VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe)) & _REGBIT_PIPE_BPC_MASK) << 11;
	VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_FDI_RX_ENABLE));

	VGT_POST_READ(pdev, reg);
	udelay(100);

	/* Ironlake workaround, disable clock pointer after downing FDI */
	if (VGT_PCH(pdev) == PCH_CPT) {
		vgt_cpt_phase_pointer_disable(vgt, pipe);
	} else {
		BUG();
	}

	/* still set train pattern 1 */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~_REGBIT_FDI_LINK_TRAIN_NONE;
	reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_1;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (VGT_PCH(pdev) == PCH_CPT) {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT;
	} else {
		BUG();
	}
	/* BPC in FDI rx is consistent with that in PIPECONF */
	reg_data &= ~VGT_FDI_RX_CTL_BPC_MASK;
	reg_data |= (VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe)) & _REGBIT_PIPE_BPC_MASK) << 11;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, reg);
	udelay(100);
}

static bool vgt_dp_pipe_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, u32 port_sel, u32 val)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	if ((val & _REGBIT_DP_PORT_ENABLE) == 0)
		return false;

	if (VGT_PCH(pdev) == PCH_CPT) {
		reg = VGT_TRANS_DP_CTL(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);
		if ((reg_data & _REGBIT_TRANS_DP_PORT_SEL_MASK) != port_sel)
			return false;
	} else {
		BUG();
	}
	return true;
}

static bool vgt_adpa_pipe_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, u32 val)
{
	//struct pgt_device *pdev = vgt->pdev;
	ASSERT(pipe < PIPE_C);

	if ((val & _REGBIT_ADPA_DAC_ENABLE) == 0)
		return false;

	if (VGT_PCH(pdev) == PCH_CPT) {
		if ((val & _REGBIT_PORT_TRANS_SEL_MASK) != _REGBIT_PORT_TRANS_SEL_CPT(pipe))
			return false;
	} else {
		BUG();
	}
	return true;
}

static bool vgt_lvds_pipe_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, u32 val)
{
	//struct pgt_device *pdev = vgt->pdev;
	ASSERT(pipe < PIPE_C);

	if ((val & _REGBIT_LVDS_PORT_ENABLE) == 0)
		return false;
	if (VGT_PCH(pdev) == PCH_CPT) {
		if ((val & _REGBIT_PORT_TRANS_SEL_MASK) != _REGBIT_PORT_TRANS_SEL_CPT(pipe))
			return false;
	} else {
		BUG();
	}
	return true;
}

static bool vgt_hdmi_pipe_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, u32 val)
{
	//struct pgt_device *pdev = vgt->pdev;
	ASSERT(pipe < PIPE_C);

	if ((val & _REGBIT_HDMI_PORT_ENABLE) == 0)
		return false;
	if (VGT_PCH(pdev) == PCH_CPT) {
		if ((val & _REGBIT_PORT_TRANS_SEL_MASK) != _REGBIT_PORT_TRANS_SEL_CPT(pipe))
			return false;

	} else {
		BUG();
	}

	return true;
}

static void vgt_disable_pch_dp(struct vgt_device *vgt,
		enum vgt_pipe pipe, unsigned int reg, u32 port_sel)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);
	ASSERT(!(reg & 0x3));

	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_dp_pipe_enabled(vgt, pipe, port_sel, reg_data)) {
		vgt_printk("Disabling pch dp %x on %s", reg, VGT_PIPE_NAME(pipe));
		reg_data &= ~_REGBIT_DP_PORT_ENABLE;
		VGT_MMIO_WRITE(pdev, reg, reg_data);
	}
}

static void vgt_disable_pch_hdmi(struct vgt_device *vgt,
		enum vgt_pipe pipe, unsigned int reg)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);
	ASSERT(!(reg & 0x3));

	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_hdmi_pipe_enabled(vgt, pipe, reg_data)) {
		vgt_printk("Disabling pch HDMI %x on %s", reg, VGT_PIPE_NAME(pipe));
		reg_data &= ~_REGBIT_HDMI_PORT_ENABLE;
		VGT_MMIO_WRITE(pdev, reg, reg_data);
	}
}

static void vgt_disable_pch_ports(struct vgt_device *vgt, enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	reg_data = VGT_MMIO_READ(pdev, _REG_PCH_PP_CONTROL);
	reg_data |= _REGBIT_PANEL_UNLOCK_REGS;
	VGT_MMIO_WRITE(pdev, _REG_PCH_PP_CONTROL, reg_data);

	/* actually it is PCH_DP_B/C/D if compared with i915 naming  */
	vgt_disable_pch_dp(vgt, pipe, _REG_DP_B_CTL, _REGBIT_TRANS_DP_PORT_SEL_B);
	vgt_disable_pch_dp(vgt, pipe, _REG_DP_C_CTL, _REGBIT_TRANS_DP_PORT_SEL_C);
	vgt_disable_pch_dp(vgt, pipe, _REG_DP_D_CTL, _REGBIT_TRANS_DP_PORT_SEL_D);

	/* Disable ADPA */
	reg = _REG_PCH_ADPA;
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_adpa_pipe_enabled(vgt, pipe, reg_data))
		VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_ADPA_DAC_ENABLE));

	/* Disable LVDS */
	reg = _REG_PCH_LVDS;
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_lvds_pipe_enabled(vgt, pipe, reg_data)) {
		vgt_printk("disabling lvds on %s, _REG_PCH_LVDS(%08x)",
				VGT_PIPE_NAME(pipe), reg_data);
		VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_LVDS_PORT_ENABLE));
		VGT_POST_READ(pdev, reg);
		udelay(100);
	}

	/* Disable HDMI */
	vgt_disable_pch_hdmi(vgt, pipe, _REG_HDMI_B_CTL);
	vgt_disable_pch_hdmi(vgt, pipe, _REG_HDMI_C_CTL);
	vgt_disable_pch_hdmi(vgt, pipe, _REG_HDMI_D_CTL);
}

#define state_string(v) ((v) == true ? "on" : "off")

static void vgt_assert_fdi_tx(struct vgt_device *vgt,
		enum vgt_pipe pipe, bool state)
{
	vgt_reg_t reg_data;
	bool cur_state;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	reg_data = VGT_MMIO_READ(pdev, VGT_FDI_TX_CTL(pipe));
	cur_state = !!(reg_data & _REGBIT_FDI_TX_ENABLE);
	if (cur_state != state)
		vgt_printk("FDI TX state assertion failed"
				"(expected %s, current %s)",
				state_string(state), state_string(cur_state));
}

static void vgt_assert_fdi_rx(struct vgt_device *vgt,
		enum vgt_pipe pipe, bool state)
{
	vgt_reg_t reg_data;
	bool cur_state;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	reg_data = VGT_MMIO_READ(pdev, VGT_FDI_RX_CTL(pipe));
	cur_state = !!(reg_data & _REGBIT_FDI_RX_ENABLE);

	if (cur_state != state)
		vgt_printk("FDI RX state assertion failed "
				"(expected %s, current %s)",
				state_string(state), state_string(cur_state));
}

#define vgt_assert_fdi_tx_enabled(d, p) vgt_assert_fdi_tx(d, p, true)
#define vgt_assert_fdi_tx_disabled(d, p) vgt_assert_fdi_tx(d, p, false)
#define vgt_assert_fdi_rx_enabled(d, p) vgt_assert_fdi_rx(d, p, true)
#define vgt_assert_fdi_rx_disabled(d, p) vgt_assert_fdi_rx(d, p, false)

static void vgt_assert_pch_dp_disabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, unsigned int reg, u32 port_sel)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);
	ASSERT(!(reg & 0x3));

	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_dp_pipe_enabled(vgt, pipe, port_sel, reg_data))
		vgt_printk("PCH DP (0x%08x) enabled on %s(transcoder), "
				"should be disabled", reg, VGT_PIPE_NAME(pipe));
}

static void vgt_assert_pch_hdmi_disabled(struct vgt_device *vgt,
		enum vgt_pipe pipe, unsigned int reg)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);
	ASSERT(!(reg & 0x3));

	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_hdmi_pipe_enabled(vgt, pipe, reg_data))
		vgt_printk("PCH HDMI (0x%08x) enabled on transcoder %s, "
				"should be disabled",
				reg, VGT_PIPE_NAME(pipe));
}

static void vgt_assert_transcoder_disable(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	bool enabled;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	reg = VGT_TRANSCONF(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	/* FIXME: should we check the "enable status" field (bit 30) ? */
	enabled = !!(reg_data & _REGBIT_TRANS_ENABLE);
	if (enabled)
		vgt_printk("transcoder assertion failed, "
				"should be off on %s, but is still active",
				VGT_PIPE_NAME(pipe));
}

static void vgt_assert_pch_ports_disabled(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	vgt_assert_pch_dp_disabled(vgt, pipe, _REG_DP_B_CTL, _REGBIT_TRANS_DP_PORT_SEL_B);
	vgt_assert_pch_dp_disabled(vgt, pipe, _REG_DP_C_CTL, _REGBIT_TRANS_DP_PORT_SEL_C);
	vgt_assert_pch_dp_disabled(vgt, pipe, _REG_DP_D_CTL, _REGBIT_TRANS_DP_PORT_SEL_D);

	reg = _REG_PCH_ADPA;
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_adpa_pipe_enabled(vgt, pipe, reg_data))
		vgt_printk("PCH VGA enabled on transcoder %s, should be disabled",
				VGT_PIPE_NAME(pipe));

	reg = _REG_PCH_LVDS;
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (vgt_lvds_pipe_enabled(vgt, pipe, reg_data))
		vgt_printk("PCH LVDS enabled on transcoder %s, should be disabled",
				VGT_PIPE_NAME(pipe));

	vgt_assert_pch_hdmi_disabled(vgt, pipe, _REG_HDMI_B_CTL);
	vgt_assert_pch_hdmi_disabled(vgt, pipe, _REG_HDMI_C_CTL);
	vgt_assert_pch_hdmi_disabled(vgt, pipe, _REG_HDMI_D_CTL);
}

static void vgt_disable_transcoder(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	/* FDI relies on the transcoder */
	vgt_assert_fdi_tx_disabled(vgt, pipe);
	vgt_assert_fdi_rx_disabled(vgt, pipe);

	/* Ports must be off as well */
	vgt_assert_pch_ports_disabled(vgt, pipe);

	reg = VGT_TRANSCONF(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~_REGBIT_TRANS_ENABLE;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	if (wait_for((VGT_MMIO_READ(pdev, reg) & _REGBIT_TRANS_STATE_ENABLED) == 0, 50))
		vgt_printk("failed to disable transcoder %d", pipe);
}

static void vgt_disable_pch_pll(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	vgt_reg_t pll_mask = _REGBIT_TRANSC_DPLL_ENABLE
		| _REGBIT_TRANSC_DPLLB_SEL;
	vgt_reg_t pll_sel = _REGBIT_TRANSC_DPLL_ENABLE;

	if (pipe > 1)
		return;

	/* PCH only available on ILK+ */
	BUG_ON(VGT_GEN(pdev) < 5);

	vgt_assert_transcoder_disable(vgt, pipe);

	if (pipe == PIPE_A)
		pll_sel |= _REGBIT_TRANSC_DPLLA_SEL;
	else if (pipe == PIPE_B)
		pll_sel |= _REGBIT_TRANSC_DPLLB_SEL;

	/* FIXME: why just check the transcoder C and return ??? */
	if ((VGT_MMIO_READ(pdev, _REG_PCH_DPLL_SEL) & pll_mask) == pll_sel)
		return;

	reg = VGT_PCH_DPLL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~_REGBIT_DPLL_VCO_ENABLE;
	VGT_MMIO_WRITE(pdev, reg, reg_data);
	VGT_POST_READ(pdev, reg);
	udelay(200);
}

static void vgt_update_watermarks(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t reg_data;
	unsigned int reg;

	reg = _REG_WM0_PIPEA_ILK;
	reg_data = __sreg(vgt, reg);
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	/* FIXME: How can we dectect plane B is also enabled ?? */
# if 0
	reg = _REG_WM0_PIPEB_ILK;
	reg_data = __sreg(vgt, reg);
	VGT_MMIO_WRITE(pdev, reg, reg_data);
#endif

	if (VGT_IS_IVB(pdev)) {
		reg = _REG_WM0_PIPEC_IVB;
		reg_data = __sreg(vgt, reg);
		VGT_MMIO_WRITE(pdev, reg, reg_data);
	}

	VGT_MMIO_WRITE(pdev, _REG_WM3_LP_ILK, 0);
	VGT_MMIO_WRITE(pdev, _REG_WM2_LP_ILK, 0);
	VGT_MMIO_WRITE(pdev, _REG_WM1_LP_ILK, 0);

	/* FIXME: if we detected the enabled is the power of 2,
	 * we will not update swm. For the case of only with lvds,
	 * we should upate self-reflesh watermark
	 */
	reg = _REG_WM1_LP_ILK;
	reg_data = __sreg(vgt, reg);
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	reg = _REG_WM2_LP_ILK;
	reg_data = __sreg(vgt, reg);
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	reg = _REG_WM3_LP_ILK;
	reg_data = __sreg(vgt, reg);
	VGT_MMIO_WRITE(pdev, reg, reg_data);
}

/* TODO: the proper solution is: use a kernel thread D to do
 * display switch and it will sleep when it find ring buffer
 * is not empty; The render context switch will check some
 * flag and help to clear such "RING_WAIT" bit and wakeup thread
 * D and let it finish its display switch
 */
static void vgt_clear_scanline_wait(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t reg_data;
	unsigned int reg;

	if (VGT_GEN(pdev) == 2)
		return;

	reg = RB_CTL(pdev, RING_BUFFER_RCS);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (reg_data & _RING_CTL_RB_WAIT)
		VGT_MMIO_WRITE(pdev, reg, reg_data);
}

/* refered  ironlake_crtc_prepare() part e). */
bool vgt_ironlake_crtc_disable(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;
	enum vgt_pipe pipe = port_struct->attached_pipe;
	enum vgt_plane plane = port_struct->attached_plane;

	ASSERT((pipe < I915_MAX_PIPES) && (plane < I915_MAX_PLANES));

	/* FIXME: we must check if there are any pending page flips.
	 * Such pending flip may overwrite surface base address that
	 * that we are going to change.
	 * Since we have not meet such situation yet, leave it for future
	 * complete solution.
	 */
	/*
	intel_crtc_wait_for_pending_flips(crtc);
	*/

	/* FIXME: Handling for remain vblank events, left it for future complete
	 * sulutions */
	/* drm_vblank_off(dev, pipe) */

	/* FIXME: Disable the cursor, currently only for CURSOR A,
	 * And operation on cursor A can only be applied in X mode,
	 * not in fb console mode ?
	 */
	/* intel_crtc_update_cursor(crtc, false)*/
	vgt_restore_sreg(VGT_CURPOS(pipe));
	reg_data = VGT_MMIO_READ(pdev, VGT_CURCNTR(pipe));
	reg_data &= ~(_REGBIT_CURSOR_MODE | _REGBIT_GAMMA_ENABLE);
	VGT_MMIO_WRITE(pdev, VGT_CURCNTR(pipe), reg_data);
	VGT_MMIO_WRITE(pdev, VGT_CURBASE(pipe), 0);

	/* Disable plane
	 * FIXME: no description about _REG_DSPA/BSTAT intel_disable_plane()
	 * TODO: as a separate functioin: vgt_disable_plane(vgt, pipe)
	 */
	reg_data = VGT_MMIO_READ(pdev, VGT_DSPCNTR(plane));
	if (reg_data & _REGBIT_PRIMARY_PLANE_ENABLE) {
		VGT_MMIO_WRITE(pdev, VGT_DSPCNTR(plane), (reg_data & ~_REGBIT_PRIMARY_PLANE_ENABLE));
		vgt_flush_display_plane(vgt, plane);
		vgt_printk();
		vgt_wait_for_vblank(vgt, plane);
	}

	/* Disable pipe */
	vgt_assert_planes_disabled(vgt, plane);

	/*FIXME: intel_disable_pipe(): watch out quirks of QUIRK_PIPEA_FORCE*/
	reg_data = VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe));
	if (reg_data & _REGBIT_PIPE_ENABLE) {
		VGT_MMIO_WRITE(pdev, VGT_PIPECONF(pipe), (reg_data & ~_REGBIT_PIPE_ENABLE));
		vgt_wait_for_pipe_off(vgt, pipe);
	}

	/* Assert pipe off */
	reg = VGT_PIPECONF(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if ((reg_data & _REGBIT_PIPE_STAT_ENABLED)
			|| (reg_data & _REGBIT_PIPE_ENABLE))
		BUG();

	/* Disable Panel fitter */
	VGT_MMIO_WRITE(pdev, VGT_PF_CTL(pipe), 0);
	VGT_MMIO_WRITE(pdev, VGT_PF_WIN_SZ(pipe), 0);


	/* Disable fdi */
	vgt_ironlake_fdi_disable(vgt, pipe);

	/* FIXME: COPIED FROM intel_display.c
	 * This is a horrible layering violation; we should be doing this in
	 * the connector/encoder ->prepare instead, but we don't always have
	 * enough information there about the config to know whether it will
	 * actually be necessary or just cause undesired flicker.
	 */
	/* Disable pch ports */
	vgt_disable_pch_ports(vgt, pipe);

	vgt_disable_transcoder(vgt, pipe);

	if(VGT_PCH(pdev) == PCH_CPT) {
		reg = VGT_TRANS_DP_CTL(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);

		reg_data &= ~(_REGBIT_TRANS_DP_OUTPUT_ENABLE
				| _REGBIT_TRANS_DP_PORT_SEL_MASK);
		reg_data |= _REGBIT_TRANS_DP_PORT_SEL_NONE;
		VGT_MMIO_WRITE(pdev, reg, reg_data);

		/* diable DPLL_SEL */
		/* FIXME: not sure why we use such kind of disabling */
		reg = _REG_PCH_DPLL_SEL;
		reg_data = VGT_MMIO_READ(pdev, reg);
		switch (pipe) {
			case 0:
				reg_data &= ~(_REGBIT_TRANSA_DPLL_ENABLE
						| _REGBIT_TRANSA_DPLLB_SEL);
				break;
			case 1:
				reg_data &= ~(_REGBIT_TRANSB_DPLL_ENABLE
						| _REGBIT_TRANSB_DPLLB_SEL);
				break;
			case 2:
				/* C shares PLL A or B */
				reg_data &= ~(_REGBIT_TRANSC_DPLL_ENABLE
						| _REGBIT_TRANSC_DPLLB_SEL);
				break;
			default:
				BUG();
		}
		VGT_MMIO_WRITE(pdev, reg, reg_data);
	}

	/* disable PCH DPLL (if(ivybridage && pipe == 2) then no_pll = true) */
	if (!(VGT_IS_IVB(pdev) && pipe == PIPE_C))
		vgt_disable_pch_pll(vgt, pipe);

	/* switch from PCDclk to Rawclk */
	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_FDI_PCDCLK));

	/* disable CPU FDI_TX_PLL */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_FDI_TX_PLL_ENABLE));

	VGT_POST_READ(pdev, reg);
	udelay(100);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	VGT_MMIO_WRITE(pdev, reg, (reg_data & ~_REGBIT_FDI_RX_PLL_ENABLE));

	/* wait for the clocks to turn off */
	VGT_POST_READ(pdev, reg);
	udelay(100);

	/*FIXME: not sure why update this at the end of this process */
	vgt_update_watermarks(vgt);

	vgt_clear_scanline_wait(vgt);

	return true;
}

static void vgt_ironlake_update_plane(struct vgt_device *vgt,
		enum vgt_plane plane)
{
	unsigned int reg;
	vgt_reg_t dspcntr;
	struct pgt_device *pdev = vgt->pdev;

	/* FIXME: thus this can be called from ivb */
	ASSERT(plane < I915_MAX_PLANES);

	reg = VGT_DSPCNTR(plane);
	dspcntr = VGT_MMIO_READ(pdev, reg);

	/* Mask out pixel format bits in case we change it */
	dspcntr &= ~_REGBIT_DISPPLANE_SRC_PIXFMT_MASK;
	/* Get pixel format from sreg of DSPCNTR(plane)*/
	dspcntr |= __sreg(vgt, reg) & _REGBIT_DISPPLANE_SRC_PIXFMT_MASK;
	switch ((dspcntr & _REGBIT_DISPPLANE_SRC_PIXFMT_MASK)
			>> DSPCNTR_SRC_PIXFMT_SHIFT) {
		/* 8 bpp */
		case 0x2:
			break;
		/* 16-bit G5G6R5X */
		case 0x5:
			break;
		/* 32-bit B8G8R8X8 */
		case 0x6:
			break;
		/* 32-bit R10G10B10X2 */
		case 0x8:
			break;
		/* 32-bit B10G10R10X2*/
		case 0xa:
			break;
		/* 64-bit R16G16B16X16 */
		case 0xc:
			break;
		/* 32-bit R8G8B8X8 */
		case 0xe:
			break;
		default:
			vgt_printk("DSPCNTR: Unknown pixel format");
			BUG();
	}

	/* Tiled(X) or linear(fb console) */
	if (__sreg(vgt, reg) & _REGBIT_DISPPLANE_TILED)
		dspcntr |= _REGBIT_DISPPLANE_TILED;
	else
		dspcntr &= ~_REGBIT_DISPPLANE_TILED;

	/* Trickle feed must be disabled */
	dspcntr |= _REGBIT_DISPPLANE_TRICKLE_FEED_DISABLE;

	/* FIXME: Plane enable bit still not set yet */
	VGT_MMIO_WRITE(pdev, reg, dspcntr);

	reg = VGT_DSPSTRIDE(plane);
	vgt_restore_sreg(reg);
	reg = VGT_DSPSURF(plane);
	vgt_restore_sreg(reg);
	reg = VGT_DSPTILEOFF(plane);
	vgt_restore_sreg(reg);
	reg = VGT_DSPLINOFF(plane);
	vgt_restore_sreg(reg);

	reg = VGT_DSPCNTR(plane);
	VGT_POST_READ(pdev, reg);
}

void vgt_pipe_set_base(struct vgt_device *vgt,
		enum vgt_plane plane)
{
	/* FIXME: we do not have intel_pin_and_fence_fb_obj() to protect,
	 * and also we do not wait for intel_finish_fb() which is waiting
	 * for gpu fliping page. So the rest of this function works like
	 * intel_pipe_set_base_atomic()
	 */

	/* fb object should be pinned & idle & fenced
	 * and just update base pointer
	 */
	vgt_printk();
	vgt_ironlake_update_plane(vgt, plane);

	/* FIXME: 2nd param should be pipe, not plane
	 * and currently we suppose PIPE_X == PLANE_X
	 */
	//vgt_printk();
	//vgt_wait_for_vblank(vgt, plane);

	/* TODO: update fbc ...
	 * */
}

bool vgt_ironlake_crtc_mode_set(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t pipeconf, dspcntr, pch_dpll;
	unsigned int reg;
	bool is_lvds;
	enum vgt_pipe pipe = port_struct->attached_pipe;
	enum vgt_plane plane = port_struct->attached_plane;

	/* FIXME: we need to scan hw status to
	 * decide which type of port/pipe/monitors
	 * are used, currently, hard coded to lvds
	 */
	if (port_struct->output_type == VGT_OUTPUT_LVDS)
		is_lvds = true;
	else
		is_lvds = false;

	/* enable vblank during the interim
	 * FIXME: still not sure what is this for */
	//drm_vblank_pre_modeset(dev, pipe);

	/* FIXME: updata cursor, but not sure
	 * if this is coherent with
	 * i9xx_update_cursor(), will track i915's behavior to fix this later */
	vgt_restore_sreg(VGT_CURPOS(pipe));
	vgt_restore_sreg(VGT_CURCNTR(pipe));
	vgt_restore_sreg(VGT_CURBASE(pipe));

	/* FDI link */
	/* software calculation only, nothing need to do */

	/* FIXME: we cannot just restore pipeconf, since there are configure
	 * and control bit(bit[31]), and the control bit should only be set
	 * when all configures are ready
	 */
	/* determine panel color depth (configure info) */
	reg = VGT_PIPECONF(pipe);
	/* default PIPEA_CONF = 0xc0000050 */
	pipeconf = __sreg(vgt, VGT_PIPECONF(pipe))
		& _REGBIT_PIPE_BPC_MASK;
	VGT_MMIO_WRITE(pdev, reg, pipeconf);

	pipeconf = VGT_MMIO_READ(pdev, reg);

	/* PCH eDP needs FDI, but CPU eDP does not */
	/* intel_crtc->no_pll = true, only when ivb use PIPE_C */
	if (!(VGT_IS_IVB(pdev) && pipe == PIPE_C)) {

		/*if (!has_edp_encoder ||
		    intel_encoder_is_pch_edp(&has_edp_encoder->base)) { */
		//show_sreg(vgt, VGT_PCH_FP0(pipe));
		vgt_restore_sreg(VGT_PCH_FP0(pipe));

		/* PCH_DPLL default: 89086008
		 * FIXME: big risk directly use sreg
		 * PCH_DPLL will be enabled in
		 * vgt_enable_pll() */
		reg = VGT_PCH_DPLL(pipe);
		pch_dpll = __sreg(vgt, reg) & ~_REGBIT_DPLL_VCO_ENABLE;
		VGT_MMIO_WRITE(pdev, reg, pch_dpll);

		VGT_POST_READ(pdev, reg);
		udelay(150);

	} else {
		BUG();
	}

	/* The LVDS pin pair needs to be on before the DPLLs are enabled.
	 * This is an exception to the general rule that mode_set doesn't turn
	 * things on.
	 */
	if (is_lvds) {
		vgt_restore_sreg(_REG_PCH_LVDS);
	}

	/* TODO: For non-DP output, clear any trans DP clock recovery setting.*/
	VGT_MMIO_WRITE(pdev, VGT_TRANSDATA_M1(pipe), 0);
	VGT_MMIO_WRITE(pdev, VGT_TRANSDATA_N1(pipe), 0);
	VGT_MMIO_WRITE(pdev, VGT_TRANSDATA_M2(pipe), 0);
	VGT_MMIO_WRITE(pdev, VGT_TRANSDATA_N2(pipe), 0);

	/* FIXME: when (ivb && pipe == PIPE_C) && edp used, we should not
	 * execute following code snippet
	 */
	if (true) {
		reg = VGT_PCH_DPLL(pipe);

		VGT_MMIO_WRITE(pdev, reg, pch_dpll);

		/* Wait for the clocks to stabilize */
		VGT_POST_READ(pdev, reg);
		udelay(150);

		/* The pixel multiplier can only be updated once the
		 * DPLL is enabled and the clocks are stable.
		 *
		 * So write it again.
		 */
		VGT_MMIO_WRITE(pdev, reg, pch_dpll);
	}

	/* enabling/disabling CxSR downclocking */
	reg = VGT_PCH_FP1(pipe);
	vgt_restore_sreg(reg);

	reg = VGT_VSYNCSHIFT(pipe);
	vgt_restore_sreg(reg);

	reg = VGT_HTOTAL(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_HBLANK(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_HSYNC(pipe);
	vgt_restore_sreg(reg);

	reg = VGT_VTOTAL(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_VBLANK(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_VSYNC(pipe);
	vgt_restore_sreg(reg);

	/* pipesrc controls the size that is scaled from, which should
	 * always be the user's requested size.
	 */
	reg = VGT_PIPESRC(pipe);
	vgt_restore_sreg(reg);

	reg = VGT_PIPE_DATA_M1(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_PIPE_DATA_N1(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_PIPE_LINK_M1(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_PIPE_LINK_N1(pipe);
	vgt_restore_sreg(reg);

	/* ignore eDP part for late handling */
	/*
	if (has_edp_encoder &&
	    !intel_encoder_is_pch_edp(&has_edp_encoder->base)) {
		ironlake_set_pll_edp(crtc, adjusted_mode->clock);
	}
	*/

	/* FIXME: actually no bits are changed, this is
	 * because we did not encounbter the changing
	 * bit fields must be configured case by case
	 * But the writing upon pipeconf can trigger
	 * vblank ???
	 */
	reg = VGT_PIPECONF(pipe);
	VGT_MMIO_WRITE(pdev, reg, pipeconf);
	VGT_POST_READ(pdev, reg);

	/* wait for vblank event to make all these configures work */
	vgt_wait_for_vblank(vgt, pipe);

	/* FIXME: gamma_enable is mandatory, so we did not
	 * check if bit GAMMA_ENABLE in the sreg */
	dspcntr = _REGBIT_DISPPLANE_GAMMA_ENABLE;
	reg = VGT_DSPCNTR(plane);
	VGT_MMIO_WRITE(pdev, reg, dspcntr);
	VGT_POST_READ(pdev, reg);

	vgt_pipe_set_base(vgt, plane);

	vgt_update_watermarks(vgt);

	//drm_vblank_post_modeset(dev, pipe);
	return true;
}

/* refer ironlake_fdi_pll_enable() */
static void vgt_ironlake_fdi_pll_enable(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	/* Write the TU size bits so error detection works */
	/* FIXME: why i915 do it like this:
		I915_WRITE(FDI_RX_TUSIZE1(pipe),
		   I915_READ(PIPE_DATA_M1(pipe)) & TU_SIZE_MASK);
	*/
	reg = VGT_FDI_RX_TUSIZE1(pipe);
	vgt_restore_sreg(reg);

	/* enable PCH FDI RX PLL, wait warmup plus DMI latency
	 * default value [0x00000040]*/
	/* FIXME: BIG WARNING: do not pretend reg initial value from sreg,
	 * you can compare reg value with sreg, but you should never use
	 * sreg as the one you get from real hw. ,
	 * this can be very big risk*/
	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~((0x7 << 19) | (0x7 << 16));
	reg_data |= __sreg(vgt, reg) & _REGBIT_FDI_RX_PORT_WIDTH_MASK;
	reg_data |= (VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe)) & _REGBIT_PIPE_BPC_MASK) << 11;
	reg_data |= _REGBIT_FDI_RX_PLL_ENABLE;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, reg);
	udelay(200);

	/* switch from Rawclk to PCDclk */
	reg_data = VGT_MMIO_READ(pdev, reg);
	VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_PCDCLK));

	VGT_POST_READ(pdev, reg);
	udelay(200);

	/* Enable CPU FDI TX PLL, always on for Ironlake */
	/* default sreg[VGT_FDI_TX_CTL(PIPE_A)] = b0044000
	 * FIXME: watchout recovery of other fields.
	 * but dom0 has default:
	 */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);

	if(!(reg_data & _REGBIT_FDI_TX_PLL_ENABLE)) {
		reg_data |= _REGBIT_FDI_TX_PLL_ENABLE;
		/* FIXME: not sure why in i915 it has 001c4000 here,
		 * so forcely set bit 19 and bit 20*/
		reg_data |= 0x3 << 19;
		VGT_MMIO_WRITE(pdev, reg, reg_data);
		VGT_POST_READ(pdev, reg);
		udelay(100);
	}
}

static void vgt_crtc_load_lut(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg,i;

	ASSERT(pipe < PIPE_C);

	reg = VGT_PALETTE(pipe);
	if (VGT_HAS_PCH_SPLIT(pdev))
		reg = VGT_LGC_PALETTE(pipe);

	for (i = 0; i < 256; i++) {
		vgt_restore_sreg(reg + i * 4);
	}
}

static void vgt_assert_fdi_tx_pll_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg;
	vgt_reg_t reg_data;

	ASSERT(pipe < PIPE_C);

	if (VGT_GEN(pdev) == 5)
		return;

	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (!(reg_data & _REGBIT_FDI_TX_PLL_ENABLE))
		vgt_printk("FDI TX PLL assertion failed, should be active but disabled");
}

static void vgt_assert_fdi_rx_pll_enabled(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg;
	vgt_reg_t reg_data;

	ASSERT(pipe < PIPE_C);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (!(reg_data & _REGBIT_FDI_RX_PLL_ENABLE))
		vgt_printk("FDI RX PLL assertion failed, should be active but disabled");
}

static void vgt_assert_pipe(struct vgt_device *vgt,
		enum vgt_pipe pipe, bool state)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg;
	vgt_reg_t reg_data;
	bool cur_state;

	/* FIXME: quirks have not been added yet */
	/* if we need the pipe A quirk it must be always on */
	/*
	if (pipe == PIPE_A && dev_priv->quirks & QUIRK_PIPEA_FORCE)
		state = true;
	*/
	ASSERT(pipe < PIPE_C);
	reg = VGT_PIPECONF(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	cur_state = !!(reg_data & _REGBIT_PIPE_ENABLE);
	if (cur_state != state)
		vgt_printk("%s assertion failed (expect %s, current %s)",
				VGT_PIPE_NAME(pipe), state_string(state), state_string(cur_state));

}
#define vgt_assert_pipe_enabled(d, p) vgt_assert_pipe((d), (p), true)

static void vgt_enable_pipe(struct vgt_device *vgt,
		enum vgt_pipe pipe, bool is_pch_port)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	if(!VGT_HAS_PCH_SPLIT(pdev)) {
		BUG();
	} else {
		if (is_pch_port) {
			vgt_assert_fdi_rx_pll_enabled(vgt, pipe);
			vgt_assert_fdi_tx_pll_enabled(vgt, pipe);
		} else
			BUG();
	}

	reg = VGT_PIPECONF(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (reg_data & _REGBIT_PIPE_ENABLE)
		return;

	/* FIXME: vGT need this extra delay for pipe enabling
	 * I915 does not have such checking.
	 * Not sure the root cause.
	 * This need to be fixed later
	 */
	VGT_MMIO_WRITE(pdev, reg, reg_data | _REGBIT_PIPE_ENABLE);
	if (wait_for((VGT_MMIO_READ(pdev, reg) & _REGBIT_PIPE_STAT_ENABLED) != 0, 200))
		vgt_printk("pipe %s enabling failed", VGT_PIPE_NAME(pipe));

	vgt_wait_for_vblank(vgt, pipe);
}

static void vgt_enable_plane(struct vgt_device *vgt,
		enum vgt_plane plane, enum vgt_pipe pipe)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg;
	vgt_reg_t reg_data;

	ASSERT(pipe < PIPE_C);
	ASSERT(plane < PLANE_C);

	vgt_assert_pipe_enabled(vgt, pipe);

	reg = VGT_DSPCNTR(plane);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (reg_data & _REGBIT_PRIMARY_PLANE_ENABLE)
		return;

	reg_data = __sreg(vgt, reg);
	/* Previously, Plane is supposed to be enabled for each vm
	 * that in "display switch". But for some reason, vm can
	 * trun it off (like power saving?). So we forcely turn
	 * it on in case it block the process.(disabled plane will block
	 * the display switch ?). In this situation, it change hardware
	 * status that vm does not aware. This may cause potential issues.
	 * */
	//ASSERT(reg_data & _REGBIT_PRIMARY_PLANE_ENABLE);
	//vgt_restore_sreg(reg);
	if (!(reg_data & _REGBIT_PRIMARY_PLANE_ENABLE)) {
		vgt_printk("VGT(%d): plane should be enabled in"
				"DSPCNTR(%d), actually not!",
				vgt->vgt_id, plane);
		VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_PRIMARY_PLANE_ENABLE));
	} else
		vgt_restore_sreg(reg);

	vgt_flush_display_plane(vgt, plane);
	vgt_wait_for_vblank(vgt, pipe);
}

static const int snb_b_fdi_train_param[] = {
	_REGBIT_FDI_LINK_TRAIN_400MV_0DB_SNB_B,
	_REGBIT_FDI_LINK_TRAIN_400MV_6DB_SNB_B,
	_REGBIT_FDI_LINK_TRAIN_600MV_3_5DB_SNB_B,
	_REGBIT_FDI_LINK_TRAIN_800MV_0DB_SNB_B,
};

/* The FDI link training functions for SNB/Cougarpoint. */
static void vgt_gen6_fdi_link_train(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg,i;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	/* Train 1: umask FDI RX Interrupt symbol_lock and bit_lock bit
	   for train result */
	/* FIXME: we do not recover these registers, so sreg is fine,
	 * but what about vreg ??? */
	reg = VGT_FDI_RX_IMR(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~(_REGBIT_FDI_RX_SYMBOL_LOCK |
			_REGBIT_FDI_RX_BIT_LOCK);
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, reg);
	udelay(150);

	/* enable CPU FDI TX and PCH FDI RX */
	reg = VGT_FDI_TX_CTL(pipe);
	/* FIXME:suppose (fdi_lanes -1)
	 * saved in sreg[21:19], but can we trust it,
	 * suppose we just have 1 fdi_lane ???
	 */
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~_REGBIT_FDI_TX_PORT_WIDTH_MASK;
	//ASSERT((__sreg(vgt, reg) & _REGBIT_FDI_TX_PORT_WIDTH_MASK) == 0);
	//reg_data |= (0 << 19);
	reg_data |= (__sreg(vgt, reg) & _REGBIT_FDI_TX_PORT_WIDTH_MASK);
	reg_data &= ~_REGBIT_FDI_LINK_TRAIN_NONE;
	reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_1;
	reg_data &= ~_REGBIT_FDI_LINK_TRAIN_VOL_EMP_MASK;
	/* SNB-B */
	reg_data |= _REGBIT_FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	/* FIXME: sreg should be b0044000 =
	 * FDI_TX_ENABLE | LINK_NORMAL(00b)
	 * | FD_PLL_ENABLE | ENHANCED_FRAMING_ENABLE */
	VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_TX_ENABLE));

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (VGT_PCH(pdev) == PCH_CPT) {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT;
	} else {
		BUG();
	}
	/* FIXME: sreg should be 80022350 =
	 * FDI_RX_ENABLE | ...(normal configure)*/
	VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_RX_ENABLE));

	VGT_POST_READ(pdev, reg);
	udelay(150);

	if (VGT_PCH(pdev) == PCH_CPT) {
		vgt_cpt_phase_pointer_enable(vgt, pipe);
	}

	for (i = 0; i < 4; i++) {
		reg = VGT_FDI_TX_CTL(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_VOL_EMP_MASK;
		reg_data |= snb_b_fdi_train_param[i];
		VGT_MMIO_WRITE(pdev, reg, reg_data);

		VGT_POST_READ(pdev, reg);
		udelay(150);

		reg = VGT_FDI_RX_IIR(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);
		vgt_printk("FDI_RX_IIR 0x%08x", reg_data);

		if (reg_data & _REGBIT_FDI_RX_BIT_LOCK) {
			VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_RX_BIT_LOCK));
			vgt_printk("FDI train 1 done.");
			break;
		}
	}
	if (i == 4)
		vgt_printk("FDI train 1 failed!");

	/* Train 2 */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data &= ~_REGBIT_FDI_LINK_TRAIN_NONE;
	reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_2;
	if (VGT_GEN(pdev) == 6) {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_VOL_EMP_MASK;
		/* SNB-B */
		reg_data |= _REGBIT_FDI_LINK_TRAIN_400MV_0DB_SNB_B;
	}
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (VGT_PCH(pdev) == PCH_CPT) {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		reg_data |= _REGBIT_FDI_LINK_TRAIN_PATTERN_2_CPT;
	} else {
		BUG();
	}
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, reg);
	udelay(150);

	for (i = 0; i < 4; i++) {
		reg = VGT_FDI_TX_CTL(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_VOL_EMP_MASK;
		reg_data |= snb_b_fdi_train_param[i];
		VGT_MMIO_WRITE(pdev, reg, reg_data);

		VGT_POST_READ(pdev, reg);
		udelay(500);

		reg = VGT_FDI_RX_IIR(pipe);
		reg_data = VGT_MMIO_READ(pdev, reg);
		vgt_printk("FDI_RX_IIR 0x%08x", reg_data);

		if (reg_data & _REGBIT_FDI_RX_SYMBOL_LOCK) {
			VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_RX_SYMBOL_LOCK));
			vgt_printk("FDI train 2 done.");
			break;
		}
	}
	if (i == 4)
		vgt_printk("FDI train 2 failed!");

	vgt_printk("FDI train done.\n");
}

static void vgt_assert_pch_refclk_enabled(struct vgt_device *vgt)
{
	/* FIXME: since pch refclk was enabled when driver loading, so guest
	 * cannot stop it by itself ??? or we can reinit it for other domain
	 * as one of our complete services */
	vgt_reg_t reg_data;
	bool enabled;
	struct pgt_device *pdev = vgt->pdev;
	reg_data = VGT_MMIO_READ(pdev, _REG_PCH_DREF_CONTROL);
	enabled = !!(reg_data & (_REGBIT_DREF_SSC_SOURCE_MASK
				| _REGBIT_DREF_NONSPREAD_SOURCE_MASK
				| _REGBIT_DREF_SUPERSPREAD_SOURCE_MASK));
	if (!enabled) {
		vgt_printk("PCH refclk assertion failure, should be active but is disabled");
		BUG();
	}
}

/* enable PCH PLL
 * The PCH PLL needs to be enabled before the PCH transcoder, since it
 * drives the transcoder clock.
 */
static void vgt_enable_pch_pll(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	/*FIXME: follow original function */
	if (pipe > PIPE_B)
		return;

	/* PCH only available on ILK+ */
	BUG_ON(VGT_GEN(pdev) < 5);

	/* PCH refclock must be enabled first */
	vgt_assert_pch_refclk_enabled(vgt);

	/* FIXME: default sreg = 89086008 */
	reg = VGT_PCH_DPLL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	reg_data |= _REGBIT_DPLL_VCO_ENABLE;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, reg);
	udelay(200);
}

static void vgt_assert_panel_unlocked(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;
	bool locked = true;
	enum vgt_pipe panel_pipe = PIPE_A;

	ASSERT(VGT_HAS_PCH_SPLIT(pdev));

	reg_data = VGT_MMIO_READ(pdev, _REG_PCH_PP_CONTROL);
	if (!(reg_data & _REGBIT_POWER_TARGET_ON) ||
			((reg_data & _REGBIT_PANEL_UNLOCK_REGS) == _REGBIT_PANEL_UNLOCK_REGS))
		locked = false;

	if(VGT_MMIO_READ(pdev, _REG_PCH_LVDS) & _REGBIT_LVDS_PIPEB_SELECT)
		panel_pipe = PIPE_B;

	if (panel_pipe == pipe && locked)
		vgt_printk("panel assertion failed, %s locked.", VGT_PIPE_NAME(pipe));

}

static void vgt_fdi_normal_train(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	/* enable normal train */
	reg = VGT_FDI_TX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (VGT_IS_IVB(pdev)) {
		/* not finished yet */
		BUG();
	} else {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_NONE;
		reg_data |= _REGBIT_FDI_LINK_TRAIN_NONE
			| _REGBIT_FDI_TX_ENHANCE_FRAME_ENABLE;
	}
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	reg = VGT_FDI_RX_CTL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	if (VGT_PCH(pdev) == PCH_CPT) {
		reg_data &= ~_REGBIT_FDI_LINK_TRAIN_PATTERN_MASK_CPT;
		reg_data |= _REGBIT_FDI_LINK_TRAIN_NORMAL_CPT;
	} else {
		BUG();
	}
	VGT_MMIO_WRITE(pdev, reg, (reg_data | _REGBIT_FDI_RX_ENHANCE_FRAME_ENABLE));

	/* wait one idle pattern time */
	VGT_POST_READ(pdev, reg);
	udelay(1000);

	/* IVB wants error correction enabled */
	/* TODO */
}

static void vgt_assert_pch_pll(struct vgt_device *vgt,
		enum vgt_pipe pipe, bool state)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	bool cur_state;
	struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < I915_MAX_PIPES);

	if (VGT_PCH(pdev) == PCH_CPT) {
		reg_data = VGT_MMIO_READ(pdev, _REG_PCH_DPLL_SEL);
		if (!((reg_data >> (4 * pipe)) & 8))
			vgt_printk("transcoder %d PLL not enabled", pipe);

		pipe = (reg_data >> (4 * pipe)) & 1;
	}

	reg = VGT_PCH_DPLL(pipe);
	reg_data = VGT_MMIO_READ(pdev, reg);
	cur_state = !!(reg_data & _REGBIT_DPLL_VCO_ENABLE);
	if (cur_state != state)
		vgt_printk("PCH PLL state assertion failed (expected %s, current %s)",
				state_string(state), state_string(cur_state));
}
#define vgt_assert_pch_pll_enabled(d, p) vgt_assert_pch_pll((d), (p), true)

static void vgt_enable_transcoder(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	unsigned int reg;
	vgt_reg_t transconf;
	struct pgt_device *pdev = vgt->pdev;

	/* PCH only available on ILK+ */
	BUG_ON(VGT_GEN(pdev) < 5);

	ASSERT(pipe < I915_MAX_PIPES);

	/* Make sure PCH DPLL is enabled */
	vgt_assert_pch_pll_enabled(vgt, pipe);

	/* FDI must be feeding us bits for PCH ports */
	vgt_assert_fdi_tx_enabled(vgt, pipe);
	vgt_assert_fdi_rx_enabled(vgt, pipe);

	/* TRANSCONF: configure BPC and INTERLACE,
	 * FIXME: just retore these config ???
	 * PIPEACONF(0x70008): sreg value: c0000050 =
	 * progressive_fetch_and_progressive_display |
	 * 6_bits_per_color | ...
	 *
	 * TRANSACONF(0xf0008): sreg value: c0000000 =
	 * progressive | ...
	 *
	 * PCH_PCT does not care BPC in transconf
	 */
	reg = VGT_TRANSCONF(pipe);
	transconf = VGT_MMIO_READ(pdev, reg);

	if (VGT_PCH(pdev) == PCH_IBX) {
		BUG();
	}

	transconf &= ~_REGBIT_TRANS_INTERLACE_MASK;
	transconf |= __sreg(vgt, reg) & _REGBIT_TRANS_INTERLACE_MASK;
	VGT_MMIO_WRITE(pdev, reg, (transconf | _REGBIT_TRANS_ENABLE));

	if (wait_for((VGT_MMIO_READ(pdev, reg) & _REGBIT_TRANS_STATE_ENABLED), 100))
		vgt_printk("failed to enable transcoder on %s", VGT_PIPE_NAME(pipe));

}

static void vgt_ironlake_pch_enable(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	//vgt_reg_t reg_data;
	unsigned int reg;
	struct pgt_device *pdev = vgt->pdev;

	/* For PCH output, training FDI link */
	vgt_gen6_fdi_link_train(vgt, pipe);

	vgt_enable_pch_pll(vgt, pipe);

	if (VGT_PCH(pdev) == PCH_CPT) {
		/* FIXME: Just restore sreg, default 0x8 =
		 * TRANSA_DPLL_ENABLE | TRANSA_DALLA_SEL
		 */
		VGT_POST_READ(pdev, _REG_PCH_DPLL_SEL);
		vgt_restore_sreg(_REG_PCH_DPLL_SEL);
	}

	/* set transcoder timing, panel must allow it */
	vgt_assert_panel_unlocked(vgt, pipe);
	/* FIXME: TRANS_HTOTAL_X should be the same as HTOTAL_X
	 * since it is the copy of HTOTAL_X, so we just do restore
	 * operation without reading hw reg of HTOTAL_X */
	//vgt_printk("HTOTAL(%08x)", VGT_MMIO_READ(pdev, VGT_HTOTAL(pipe)));
	//vgt_printk("SINCE WE HANG HERE, JUST DO NOT REVOCER THESE REGS");
#if 1
	reg = VGT_TRANS_HTOTAL(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_TRANS_HBLANK(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_TRANS_HSYNC(pipe);
	vgt_restore_sreg(reg);

	reg = VGT_TRANS_VTOTAL(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_TRANS_VBLANK(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_TRANS_VSYNC(pipe);
	vgt_restore_sreg(reg);
	reg = VGT_TRANS_VSYNCSHIFT(pipe);
	vgt_restore_sreg(reg);
#endif
	vgt_fdi_normal_train(vgt, pipe);

	/* For PCH DP, enable TRANS_DP_CTL */
	/* TODO */

	vgt_enable_transcoder(vgt, pipe);
}

static void vgt_update_cursor(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	//unsigned int reg;
	//vgt_reg_t reg_data;
	//struct pgt_device *pdev = vgt->pdev;

	ASSERT(pipe < PIPE_C);

	/* Update cursor is just this kind of easy ? */
	vgt_restore_sreg(VGT_CURPOS(pipe));
	vgt_restore_sreg(VGT_CURCNTR(pipe));
	vgt_restore_sreg(VGT_CURBASE(pipe));
}


static void vgt_ironlake_crtc_enable(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	unsigned int reg;
	vgt_reg_t reg_data, pch_pf_size;
	enum vgt_pipe pipe = port_struct->attached_pipe;
	enum vgt_plane plane = port_struct->attached_plane;
	struct pgt_device *pdev = vgt->pdev;
	bool output_lvds = true, output_edp = false;

	ASSERT(pipe < PIPE_C);

	vgt_update_watermarks(vgt);

	/* FIXME: how to recover each bit fields of
	 * PCH_LVDS ???, plenty of timing and pin
	 * stuff */
	reg = _REG_PCH_LVDS;
	reg_data = VGT_MMIO_READ(pdev, reg);
	if ((reg_data & _REGBIT_LVDS_PORT_ENABLE) == 0) {
		reg_data |= _REGBIT_LVDS_PORT_ENABLE;
		VGT_MMIO_WRITE(pdev, reg, reg_data);
	}

	/* FIXME: suppose it must be pch port */
	vgt_ironlake_fdi_pll_enable(vgt, pipe);

	/* Enable panel fitting for LVDS */
	/* FIXME: this is only for lvds and eDP, if this kind of ports
	 * are connected */
	reg = VGT_PF_WIN_SZ(pipe);
	pch_pf_size = __sreg(vgt,reg);
	if (pch_pf_size && (output_lvds || output_edp)) {

		/* FIXME: default 0 when no page fitting */
		reg = VGT_PF_CTL(pipe);
		reg_data = _REGBIT_PF_ENABLE | _REGBIT_PF_FILTER_MED_3x3;
		VGT_MMIO_WRITE(pdev, reg, reg_data);

		reg = VGT_PF_WIN_POS(pipe);
		vgt_restore_sreg(reg);

		reg = VGT_PF_WIN_SZ(pipe);
		vgt_restore_sreg(reg);
	}

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	vgt_crtc_load_lut(vgt, pipe);

	vgt_enable_pipe(vgt, pipe, true);
	/* FIXME: suppose we use PIPE_A and PLANE_A */
	vgt_enable_plane(vgt, pipe, plane);

	/* FIXME: suppose it must be pch port */
	vgt_ironlake_pch_enable(vgt, pipe);

	/* TODO: no support for fbc right now */

	vgt_update_cursor(vgt, pipe);
}

static void vgt_pch_panel_set_backlight(struct vgt_device *vgt)
{
	//struct pgt_device *pdev = vgt->pdev;
	vgt_restore_sreg(_REG_BLC_PWM_CPU_CTL);
}

static void vgt_lvds_enable(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	unsigned int reg;
	vgt_reg_t reg_data;
	struct pgt_device *pdev = vgt->pdev;

	if (!VGT_HAS_PCH_SPLIT(pdev))
		BUG();

	/* FIXME: we should directly use vgt_restore_sreg() */
	reg = _REG_PCH_LVDS;
	reg_data = __sreg(vgt,reg) | _REGBIT_LVDS_PORT_ENABLE;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	/* FIXME FIXME FIXME: whatever we will recover panel fitter */
	/* we did not recover pfit_control related thing
	*/
	reg = _REG_PCH_PP_CONTROL;
	reg_data = __sreg(vgt, _REG_PCH_PP_CONTROL) | _REGBIT_POWER_TARGET_ON;
	VGT_MMIO_WRITE(pdev, reg, reg_data);

	VGT_POST_READ(pdev, _REG_PCH_LVDS);

	if(wait_for((VGT_MMIO_READ(pdev, _REG_PCH_PP_STATUS) & _REGBIT_PANEL_POWER_ON) != 0, 1000))
		vgt_printk("timed out waiting for panel to power on");

	//setup backlight ...continue...
	vgt_pch_panel_set_backlight(vgt);
}

static void vgt_crt_set_dpms(struct vgt_device *vgt,
		unsigned int mode)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg;
	vgt_reg_t reg_data;

	reg = _REG_PCH_ADPA;
	reg_data = VGT_MMIO_READ(pdev, reg);

	if (mode)
		reg_data |= _REGBIT_ADPA_DAC_ENABLE;
	else
		reg_data &= ~_REGBIT_ADPA_DAC_ENABLE;

	VGT_MMIO_WRITE(pdev, reg, reg_data);
}

static void vgt_crt_prepare(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	vgt_crt_set_dpms(vgt, false);
}

static void vgt_crt_mode_set(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
#define VGT_ADPA_HOTPLUG_BITS (_REGBIT_ADPA_CRT_HOTPLUG_PERIOD_128 |		\
		_REGBIT_ADPA_CRT_HOTPLUG_WARMUP_10MS |		\
		_REGBIT_ADPA_CRT_HOTPLUG_SAMPLE_4S |			\
		_REGBIT_ADPA_CRT_HOTPLUG_VOLTAGE_50 |		\
		_REGBIT_ADPA_CRT_HOTPLUG_VOLREF_325MV |		\
		_REGBIT_ADPA_CRT_HOTPLUG_ENABLE)
	struct pgt_device *pdev = vgt->pdev;
	unsigned int reg = _REG_PCH_ADPA;
	vgt_reg_t sreg_data = __sreg(vgt, reg);
	vgt_reg_t dac_ctrl = VGT_ADPA_HOTPLUG_BITS;

	/* configure horizontal polarity */
	dac_ctrl |= sreg_data & _REGBIT_ADPA_HSYNC_ACTIVE_HIGH;
	/* configure vertical polarity */
	dac_ctrl |= sreg_data & _REGBIT_ADPA_VSYNC_ACTIVE_HIGH;

	/* choose transcoder (which pipe to use)*/
	dac_ctrl |= sreg_data & PORT_TRANS_SEL_MASK;

	VGT_MMIO_WRITE(pdev, reg, dac_ctrl);
}

static void vgt_cpt_verify_modeset(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int chk2_reg, dsl_reg;
	vgt_reg_t old_dsl;

	ASSERT(pipe < I915_MAX_PIPES);

	chk2_reg = VGT_TRANS_CHICKEN2(pipe);
	dsl_reg = VGT_PIPEDSL(pipe);

	old_dsl = VGT_MMIO_READ(pdev, dsl_reg);
	udelay(500);
	if (wait_for(VGT_MMIO_READ(pdev, dsl_reg) != old_dsl, 5)) {
		/* Bspec: Enable the override prior to enabling the transcoder.
		 * Disable the override after disabling the transcoder
		 */
		VGT_MMIO_WRITE(pdev, chk2_reg, _REGBIT_TRANS_AUTOTRAIN_GEN_STALL_DISABLE);
		udelay(250);
		VGT_MMIO_WRITE(pdev, chk2_reg, 0);
		if (wait_for(VGT_MMIO_READ(pdev, dsl_reg) != old_dsl, 5))
			vgt_printk("mode set failed: %s stuck", VGT_PIPE_NAME(pipe));
	}
}

static void vgt_crt_commit(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	enum vgt_pipe pipe = port_struct->attached_pipe;

	vgt_crt_set_dpms(vgt, true);
	if (VGT_HAS_PCH_SPLIT(pdev))
		vgt_cpt_verify_modeset(vgt, pipe);
}


#define IS_CPU_EDP(dp) (false)

#if 0
static void vgt_ironlake_edp_backlight_off(struct vgt_device *vgt)
{
	BUG();
}

static void vgt_ironlake_edp_panel_off(struct vgt_device *vgt)
{
	BUG();
}

static void vgt_ironlake_edp_panel_vdd_on(struct vgt_device *vgt)
{
	BUG();
}
#endif

/* copied from pack_aux() */
static u32 vgt_pack_aux(u8 *src, int bytes)
{
	int	i;
	u32 v = 0;

	if (bytes > 4)
		bytes = 4;
	for (i = 0; i < bytes; i++)
		v |= ((u32) src[i]) << ((3-i) * 8);
	return v;
}

/* copied from pack_aux() */
static void vgt_unpack_aux(u32 src, u8 *dst, int bytes)
{
	int i;
	if (bytes > 4)
		bytes = 4;
	for (i = 0; i < bytes; i++)
		dst[i] = src >> ((3-i) * 8);
}

static int vgt_dp_aux_ch(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp,
		u8 *send, int send_bytes,
		u8 *recv, int recv_sz)
{
	int i,j, recv_bytes;
	vgt_reg_t temp;
	unsigned int ctrl_reg, data_reg, clk_div;
	struct pgt_device *pdev = vgt->pdev;
	unsigned int dp_ctrl_reg = vgt_dp->dp_ctrl_reg;

	vgt_printk("dp_ctrl_reg offset (0x%08x)", dp_ctrl_reg);
	ASSERT((vgt_dp) && (!(dp_ctrl_reg & 0x3)));
	ASSERT(send_bytes <= 20);

	/* aux control & data reg index */
	ctrl_reg = dp_ctrl_reg + 0x10;
	data_reg = ctrl_reg + 4;

	/* TODO: */
	//intel_dp_check_edp(intel_dp);

	if (VGT_HAS_PCH_SPLIT(pdev))
		clk_div = 63;
	else
		BUG();

	vgt_printk();
	/* wait for previous transaction to complete */
	for (i = 0; i < 3; i++) {
		temp = VGT_MMIO_READ(pdev, ctrl_reg);
		if (!(temp & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY))
			break;
		msleep(1);
	}

	if (i == 3) {
		vgt_printk("ERROR: dp_aux_ch busy, status(0x%08x)",
				VGT_MMIO_READ(pdev, ctrl_reg));
		return -EBUSY;
	}

	/* Bspec try at least 3 times, follow i915, we try 5 times */
	vgt_printk("send_bytes(%d)", send_bytes);
	for (i = 0; i < 5; i++) {
		for (j = 0; j < send_bytes; j += 4)
			VGT_MMIO_WRITE(pdev,
					(data_reg + j),
					vgt_pack_aux(send + j, send_bytes - j));

		temp = _REGBIT_DP_AUX_CH_CTL_SEND_BUSY
			| _REGBIT_DP_AUX_CH_CTL_TIME_OUT_400us
			| (send_bytes << _REGBIT_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT)
			| (5 << _DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT)
			| (clk_div << _DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT)
			| _REGBIT_DP_AUX_CH_CTL_DONE
			| _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR
			| _REGBIT_DP_AUX_CH_CTL_RECV_ERR;

		VGT_MMIO_WRITE(pdev, ctrl_reg, temp);

		for (;;) {
			vgt_printk("aux ctrl reg(0x%08x)", ctrl_reg);
			temp = VGT_MMIO_READ(pdev, ctrl_reg);
			vgt_printk();
			if (!(temp & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY))
				break;
			udelay(100);
		}

		vgt_printk();
		/* clear done status and any errors */
		VGT_MMIO_WRITE(pdev, ctrl_reg,
				(temp
				| _REGBIT_DP_AUX_CH_CTL_DONE
				| _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR
				| _REGBIT_DP_AUX_CH_CTL_RECV_ERR));

		if (temp & (_REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR
					| _REGBIT_DP_AUX_CH_CTL_RECV_ERR))
			continue;

		if (temp & _REGBIT_DP_AUX_CH_CTL_DONE)
			break;
	}

	if (!(temp & _REGBIT_DP_AUX_CH_CTL_DONE)) {
		vgt_printk("dp aux channel not done, status(0x%08x)",
				temp);
		return -EBUSY;
	}

	/* check for recv error */
	if (temp & _REGBIT_DP_AUX_CH_CTL_RECV_ERR) {
		vgt_printk("dp aux channel error of receiving, status(0x%08x)",
				temp);
		return -EIO;
	}

	/* timeout error, possiblely not connected */
	if (temp & _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR) {
		vgt_printk("dp aux channel time-out, status(0x%08x)",
				temp);
		return -ETIMEDOUT;
	}

	vgt_printk();

	recv_bytes = (temp & _REGBIT_DP_AUX_CH_CTL_MESSAGE_SIZE_MASK)
		>> _REGBIT_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT;

	if (recv_bytes > recv_sz)
		recv_bytes = recv_sz;

	for (i = 0; i < recv_bytes; i += 4)
		vgt_unpack_aux(VGT_MMIO_READ(pdev, (data_reg + i)),
				recv + i, recv_bytes - i);
	vgt_printk();

	return recv_bytes;
}

/* Write data to the aux channel in native mode */
static int vgt_dp_aux_native_write(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp,
		u16 address, u8 *send, int send_bytes)
{
	int retval;
	u8 msg[20];
	int msg_bytes;
	u8 ack;

	/* TODO: */
	//intel_dp_check_edp(intel_dp);
	if (send_bytes > 16)
		return -EINVAL;

	/* FIXME: COPIED REST OF THE FUNC FROM:
	 * intel_dp_aux_native_write()
	 */
	vgt_printk();
	msg[0] = AUX_NATIVE_WRITE << 4;
	msg[1] = address >> 8;
	msg[2] = address & 0xff;
	msg[3] = send_bytes - 1;
	memcpy(&msg[4], send, send_bytes);
	msg_bytes = send_bytes + 4;
	for (;;) {
		retval = vgt_dp_aux_ch(vgt, vgt_dp, msg, msg_bytes, &ack, 1);
		if (retval < 0)
			return retval;
		if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK)
			break;
		else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
			udelay(100);
		else
			return -EIO;
	}
	return send_bytes;
}

static void vgt_dp_sink_dpms(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp, int mode)
{
	int retval, i;
	u16 address = DP_SET_POWER;
	u8 byte = DP_SET_POWER_D0;
	vgt_printk();
	if (mode != DRM_MODE_DPMS_ON)
		BUG();
	else {
		for (i = 0; i < 3; i++) {
			vgt_printk();
			retval = vgt_dp_aux_native_write(vgt,
					vgt_dp, address, &byte, 1);
			vgt_printk();
			if (retval == 1)
				break;

			msleep(1);
		}
	}

}

/* copied from intel_dp_link_down */
static void vgt_dp_link_down(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int dp_ctrl_reg = vgt_dp->dp_ctrl_reg;
	vgt_reg_t dp_ctrl;

	ASSERT(!(dp_ctrl_reg & 0x3));

	vgt_printk();
	dp_ctrl = VGT_MMIO_READ(pdev, dp_ctrl_reg);
	vgt_printk();

	if ((dp_ctrl & _REGBIT_DP_PORT_ENABLE) == 0)
		return;

	/* TODO: neither pch edp nor cpu edp are supported supported yet */
	/* FIXME: simplied condition from this:
	 * (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))*/
	vgt_printk();
	if (VGT_HAS_PCH_SPLIT(pdev)) {
		/* FIXME:  we do no have variable 'DP' ? */
		dp_ctrl &= ~_REGBIT_DP_LINK_TRAIN_MASK_CPT;
		vgt_printk("dp_ctrl value(0x%08x)", (dp_ctrl | _REGBIT_DP_LINK_TRAIN_PAT_IDLE_CPT));
		VGT_MMIO_WRITE(pdev,
				dp_ctrl_reg,
				(dp_ctrl | _REGBIT_DP_LINK_TRAIN_PAT_IDLE_CPT));
	} else
		BUG();
	VGT_POST_READ(pdev, dp_ctrl_reg);
	vgt_printk();

	msleep(17);

	vgt_printk();
	dp_ctrl &= ~(_REGBIT_DP_AUDIO_OUTPUT_ENABLE
			| _REGBIT_DP_PORT_ENABLE);
	VGT_MMIO_WRITE(pdev, dp_ctrl_reg, dp_ctrl);
	VGT_POST_READ(pdev, dp_ctrl_reg);

	vgt_printk();
	/* TODO: not sure how long we need
	 * to wait yet, arbitratly put 100ms */
	msleep(100);
}

/* refer intel_dp_prepare() */
static void vgt_dp_power_down(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp)
{
	/* TODO: call folowing functions when it is edp */
	/*
	vgt_ironlake_edp_backlight_off(vgt);
	vgt_ironlake_edp_panel_off(vgt);
	vgt_ironlake_edp_panel_vdd_on(vgt)
	*/

	vgt_dp_sink_dpms(vgt, vgt_dp, DRM_MODE_DPMS_ON);
	vgt_printk();
	vgt_dp_link_down(vgt, vgt_dp);
	vgt_printk();

	/* TODO: called when it is edp */
	/*
	vgt_ironlake_edp_panel_vdd_off
	*/
}

static void vgt_dp_prepare(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	struct vgt_dp_port *vgt_dp;

	ASSERT(port_struct);
	ASSERT((port_struct->private));

	vgt_dp = port_struct->private;
	vgt_dp_power_down(vgt, vgt_dp);
}

/* copy of intel_dp_aux_native_read */
static int vgt_dp_aux_native_read(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp, u16 address,
		u8 *recv, int recv_bytes)
{
	u8 msg[4];
	int msg_bytes;
	u8 reply[20];
	int reply_bytes;
	u8 ack;
	int retval;

	/* TODO: not support eDP yet */
	//intel_dp_check_edp(intel_dp);
	msg[0] = AUX_NATIVE_READ << 4;
	msg[1] = address >> 8;
	msg[2] = address & 0xff;
	msg[3] = recv_bytes - 1;

	msg_bytes = 4;
	reply_bytes = recv_bytes + 1;

	for (;;) {
		retval = vgt_dp_aux_ch(vgt, vgt_dp,
				msg, msg_bytes,
				reply, reply_bytes);
		if (retval == 0)
			return -EPROTO;
		if (retval < 0)
			return retval;
		ack = reply[0];
		if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK) {
			memcpy(recv, reply + 1, retval -1);
			return retval - 1;
		} else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
			udelay(100);
		else
			return -EIO;
	}
}

static bool vgt_dp_aux_native_read_retry(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp, u16 address, u8 *recv,
		int recv_bytes)
{
	int retval, i;
	for (i = 0; i < 3; i++) {
		retval = vgt_dp_aux_native_read(vgt,
				vgt_dp, address,
				recv, recv_bytes);
		if (retval == recv_bytes)
			return true;
		msleep(1);
	}
	return false;
}

static bool vgt_dp_set_link_train(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp,
		vgt_reg_t dp_ctrl, u8 dp_train_pat)
{
	struct pgt_device *pdev = vgt->pdev;
	int retval;
	/* FIXME: hardcoded variable train_set,
	 * in i915, it should be set in
	 * intel_dp_mode_set() */
	//u8 train_set[4] = {0, 0, 0, 0};

	VGT_MMIO_WRITE(pdev, vgt_dp->dp_ctrl_reg, dp_ctrl);
	VGT_POST_READ(pdev, vgt_dp->dp_ctrl_reg);

	vgt_dp_aux_native_write(vgt, vgt_dp,
			DP_TRAINING_PATTERN_SET,
			&dp_train_pat, 1);

	/* FIXME: */
	/* lane_count, use printk & dmesg, tempemrarily
	 * hardcoded as 130,
	 * lane_count == link_configure[1]*/
	retval = vgt_dp_aux_native_write(vgt,
			vgt_dp,
			DP_TRAINING_LANE0_SET,
			vgt_dp->train_set,
			vgt_dp->lane_count);

	if (retval != vgt_dp->lane_count)
		return false;

	return true;
}

static bool vgt_dp_get_link_status(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp,
		u8 link_status[DP_LINK_STATUS_SIZE])
{
	return vgt_dp_aux_native_read_retry(vgt,
			vgt_dp,
			DP_LANE0_1_STATUS,
			link_status,
			DP_LINK_STATUS_SIZE);

}

static u8 vgt_get_lane_status(u8 link_status[DP_LINK_STATUS_SIZE], int lane)
{
	int s = (lane & 1) * 4;
	u8 l = link_status[lane >> 1];

	return (l >> s) & 0xf;
}

/* copied from intel_clock_recovery_ok() */
static bool vgt_clock_recovery_ok(u8 link_status[DP_LINK_STATUS_SIZE],
		int lane_count)
{
	int lane;
	u8 lane_status;

	for (lane = 0; lane < lane_count; lane++) {
		lane_status = vgt_get_lane_status(link_status, lane);
		if ((lane_status & DP_LANE_CR_DONE) == 0)
			return false;
	}
	return true;
}

/* DPCD access
 * 1) use DP_AUX_CTRL and DP_AUX_CTRL
 *	  to access, DPCD[0 ~ 7FFFFh], totally
 *    512 kB
 * 2) Writing to DP_AUX_CTRL and read
 *    data back from DP_AUX_DATA, each port (B/C/D)
 *    has 5 MMIO register (totally 20 bytes)
 */
static u8 vgt_get_adjust_request_voltage(u8 adjust_request[2],
		int lane)
{
	int s = ((lane & 1) ?
			DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT
			: DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT);
	u8 l = adjust_request[lane >> 1];

	return ((l >> s) & 3) << DP_TRAIN_VOLTAGE_SWING_SHIFT;
}

static u8 vgt_get_adjust_request_pre_emphasis(u8 adjust_request[2],
		int lane)
{
	int s = ((lane & 1) ?
			DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT :
			DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT);
	u8 l = adjust_request[lane >> 1];

	return ((l >> s) & 3) << DP_TRAIN_PRE_EMPHASIS_SHIFT;
}

static u8 vgt_dp_voltage_max(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp)
{
	//struct pgt_device *pdev = vgt->pdev;

	if ((VGT_GEN(pdev) == 7) && IS_CPU_EDP(vgt_dp))
		return DP_TRAIN_VOLTAGE_SWING_800;
	else if ((VGT_PCH(pdev) == PCH_CPT)
			&& !IS_CPU_EDP(vgt_dp))
		return DP_TRAIN_VOLTAGE_SWING_1200;
	else
		return DP_TRAIN_VOLTAGE_SWING_800;
}

static u8 vgt_dp_pre_emphasis_max(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp, u8 volt_swing)
{
	//struct pgt_device *pdev = vgt->pdev;
	if ((VGT_GEN(pdev) == 7) && IS_CPU_EDP(vgt_dp))
		BUG(); //TODO
	else {
		switch (volt_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_400:
			return DP_TRAIN_PRE_EMPHASIS_6;
		case DP_TRAIN_VOLTAGE_SWING_600:
			return DP_TRAIN_PRE_EMPHASIS_6;
		case DP_TRAIN_VOLTAGE_SWING_800:
			return DP_TRAIN_PRE_EMPHASIS_3_5;
		case DP_TRAIN_VOLTAGE_SWING_1200:
		default:
			return DP_TRAIN_PRE_EMPHASIS_0;
		}
	}
}

static void vgt_get_adjust_train(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp,
		u8 link_stat[DP_LINK_STATUS_SIZE])
{
	u8 v = 0;
	u8 p = 0;
	int lane;
	u8 *adjust_request = link_stat
		+ (DP_ADJUST_REQUEST_LANE0_1 - DP_LANE0_1_STATUS);
	u8 volt_max;
	u8 pre_emp_max;

	for (lane = 0; lane < vgt_dp->lane_count; lane++) {
		u8 this_v = vgt_get_adjust_request_voltage(adjust_request, lane);
		u8 this_p = vgt_get_adjust_request_pre_emphasis(adjust_request, lane);
		if (this_v > v)
			v = this_v;
		if (this_p > p)
			p = this_p;
	}

	volt_max = vgt_dp_voltage_max(vgt, vgt_dp);
	if (v >= volt_max)
		v = volt_max | DP_TRAIN_MAX_SWING_REACHED;

	pre_emp_max = vgt_dp_pre_emphasis_max(vgt, vgt_dp, v);
	if (p >= pre_emp_max)
		p = pre_emp_max | DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

	for (lane = 0; lane < 4; lane++)
		vgt_dp->train_set[lane] = v | p;
}

/* copy of intel_dp_sinal_levels */
static u32 vgt_dp_signal_levels(u8 train_set)
{
	u32 signal_levels = 0;
	switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_400:
		default:
			signal_levels |= _REGBIT_DP_VOLTAGE_0_4;
			break;
		case DP_TRAIN_VOLTAGE_SWING_600:
			signal_levels |= _REGBIT_DP_VOLTAGE_0_6;
			break;
		case DP_TRAIN_VOLTAGE_SWING_800:
			signal_levels |= _REGBIT_DP_VOLTAGE_0_8;
			break;
		case DP_TRAIN_VOLTAGE_SWING_1200:
			signal_levels |= _REGBIT_DP_VOLTAGE_1_2;
			break;
	}
	switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
		case DP_TRAIN_PRE_EMPHASIS_0:
		default:
			signal_levels |= _REGBIT_DP_PRE_EMPHASIS_0;
			break;
		case DP_TRAIN_PRE_EMPHASIS_3_5:
			signal_levels |= _REGBIT_DP_PRE_EMPHASIS_3_5;
			break;
		case DP_TRAIN_PRE_EMPHASIS_6:
			signal_levels |= _REGBIT_DP_PRE_EMPHASIS_6;
			break;
		case DP_TRAIN_PRE_EMPHASIS_9_5:
			signal_levels |= _REGBIT_DP_PRE_EMPHASIS_9_5;
			break;
	}
	return signal_levels;
}

static void vgt_dp_start_link_train(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp)
{
	int i, voltage_tries, loop_tries;
	u8 voltage;
	bool clock_recovery = false;
	vgt_reg_t dp_ctrl = vgt_dp->dp_ctrl;
	//struct pgt_device *pdev = vgt->pdev;
	/* Move config from intel_dp_mode_set here */

	/* i915: in intel_dp_mode_set() */
	/*
	intel_dp->link_configuration[0] = intel_dp->link_bw;
	intel_dp->link_configuration[1] = intel_dp->lane_count;
	intel_dp->link_configuration[8] = DP_SET_ANSI_8B10B;
	*/
	/* FIXME: hack this from printk & dmesg :( */
	/* TODO: move this into dp_mode_set() */
	vgt_dp->link_configuration[0] = 10;
	vgt_dp->link_configuration[1] = 130;
	vgt_dp->link_configuration[8] = DP_SET_ANSI_8B10B;

	vgt_dp_aux_native_write(vgt, vgt_dp,
			DP_LINK_BW_SET, vgt_dp->link_configuration,
			DP_LINK_CONFIGURATION_SIZE);

	dp_ctrl |= _REGBIT_DP_PORT_ENABLE;

	if ((VGT_PCH(pdev) == PCH_CPT)
			&& ((VGT_GEN(pdev) == 7) || !IS_CPU_EDP(vgt_dp)))
		dp_ctrl &= ~_REGBIT_DP_LINK_TRAIN_MASK_CPT;
	else
		dp_ctrl &= ~_REGBIT_DP_LINK_TRAIN_MASK;

	voltage_tries = 0;
	loop_tries = 0;
	voltage = 0xff;
	for (;;) {
		u8 link_status[DP_LINK_STATUS_SIZE];
		u32 signal_levels;

		if ((VGT_GEN(pdev) == 7) && IS_CPU_EDP(vgt_dp))
			BUG();
		else if ((VGT_GEN(pdev) == 6) && IS_CPU_EDP(vgt_dp))
			BUG();
		else {
			signal_levels = vgt_dp_signal_levels(vgt_dp->train_set[0]);
			dp_ctrl = (dp_ctrl
					& ~(_REGBIT_DP_VOLTAGE_MASK
						| _REGBIT_DP_PRE_EMPHASIS_MASK))
					| signal_levels;
		}

		if (VGT_PCH(vgt) == PCH_CPT
				&& ((VGT_GEN(pdev) == 7) || !IS_CPU_EDP(vgt_dp)))
			dp_ctrl |= _REGBIT_DP_LINK_TRAIN_PAT_1_CPT;
		else
			BUG();

		if (dp_ctrl != 0x80080004)
			vgt_printk("dp_ctrl_reg value(0x%08x) not coherent with i915",
					dp_ctrl);

		/* FIXME: dp_ctrl_reg: hardcoded as 0x80080004 */
		if (!vgt_dp_set_link_train(vgt,
					vgt_dp, 0x80080004,
					DP_TRAINING_PATTERN_1
					| DP_LINK_SCRAMBLING_DISABLE))
			break;

		udelay(100);
		if (!vgt_dp_get_link_status(vgt, vgt_dp, link_status)) {
			vgt_printk("failed to get link status\n");
			break;
		}

		if (vgt_clock_recovery_ok(link_status, vgt_dp->lane_count)) {
			vgt_printk("clock recovery OK\n");
			clock_recovery = true;
			break;
		}

		/* TODO: no hardware written below  */
		/* check to see if we've tried the max voltage */
		for (i = 0; i < vgt_dp->lane_count; i++)
			if ((vgt_dp->train_set[i] & DP_TRAIN_MAX_SWING_REACHED) == 0)
				break;
		if (i == vgt_dp->lane_count) {
			loop_tries++;
			if (loop_tries == 5) {
				vgt_printk("too many tries, give up");
				break;
			}
			memset(vgt_dp->train_set, 0, 4);
			voltage_tries = 0;
			continue;
		}

		/* check if we try the same voltage 5 times */
		if ((vgt_dp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK) == voltage){
			voltage_tries++;
			if (voltage_tries == 5) {
				vgt_printk("too many voltage retries, give up");
				break;
			}
		} else
			voltage_tries = 0;

		voltage = vgt_dp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK;

		/* Compute new train_set */
		vgt_get_adjust_train(vgt, vgt_dp, link_status);
	}

	vgt_dp->dp_ctrl = dp_ctrl;
}

static u8 vgt_dp_link_status(u8 link_status[DP_LINK_STATUS_SIZE], int r)
{
	return link_status[r - DP_LANE0_1_STATUS];
}

/* Check to see if channel eq is done on all channels */
#define CHANNEL_EQ_BITS (DP_LANE_CR_DONE|\
			 DP_LANE_CHANNEL_EQ_DONE|\
			 DP_LANE_SYMBOL_LOCKED)
static bool vgt_channel_eq_ok(struct vgt_dp_port *vgt_dp, u8 link_status[DP_LINK_STATUS_SIZE])
{
	u8 lane_align;
	u8 lane_status;
	int lane;

	lane_align = vgt_dp_link_status(link_status,
			DP_LANE_ALIGN_STATUS_UPDATED);

	if ((lane_align & DP_INTERLANE_ALIGN_DONE) == 0)
		return false;

	for (lane = 0; lane < vgt_dp->lane_count; lane++) {
		lane_status = vgt_get_lane_status(link_status, lane);
		if ((lane_status & CHANNEL_EQ_BITS) != CHANNEL_EQ_BITS)
			return false;
	}
	return true;
}

static void vgt_dp_complete_link_train(struct vgt_device *vgt,
		struct vgt_dp_port *vgt_dp)
{
	u8 value;
	int tries = 0;
	int cr_tries = 0;
	bool chn_eq = false;
	vgt_reg_t dp_ctrl = vgt_dp->dp_ctrl;
	struct pgt_device *pdev = vgt->pdev;

	for (;;) {
		u32 signal_levels;
		u8 link_status[DP_LINK_STATUS_SIZE];

		if (tries > 5) {
			vgt_printk("failed to train DP, aborting\n");
			vgt_dp_link_down(vgt, vgt_dp);
			break;
		}

		/* use all kinds of workaround here */
		if ((VGT_GEN(pdev) == 7) && IS_CPU_EDP(vgt_dp))
			BUG();
		else if ((VGT_GEN(pdev) == 6) && IS_CPU_EDP(vgt_dp))
			BUG();
		else {
			signal_levels = vgt_dp_signal_levels(vgt_dp->train_set[0]);
			dp_ctrl = (dp_ctrl &
					~(_REGBIT_DP_VOLTAGE_MASK|_REGBIT_DP_PRE_EMPHASIS_MASK))
					| signal_levels;
		}

		if ((VGT_PCH(pdev) == PCH_CPT) && ((VGT_GEN(pdev) == 7) || !IS_CPU_EDP(vgt_dp)))
			dp_ctrl |= _REGBIT_DP_LINK_TRAIN_PAT_2_CPT;
		else
			BUG();

		if (dp_ctrl != 0x80080104)
			vgt_printk("warning: dp_ctrl value(0x%08x), should be 80080104",
					dp_ctrl);

		if (!vgt_dp_set_link_train(vgt, vgt_dp,
					0x80080104,
					DP_TRAINING_PATTERN_2
					| DP_LINK_SCRAMBLING_DISABLE))
			break;

		udelay(400);
		if (!vgt_dp_get_link_status(vgt, vgt_dp, link_status))
			break;

		if (vgt_dp->lane_count != 2)
			vgt_printk("lane_count should be 2, but now is %d",
					vgt_dp->lane_count);
		/* make sure clock is still ok */
		if (!vgt_clock_recovery_ok(link_status, 2)) {
			vgt_dp_start_link_train(vgt, vgt_dp);
			cr_tries++;
			continue;
		}

		if (vgt_channel_eq_ok(vgt_dp, link_status)) {
			chn_eq = true;
			break;
		}

		/* try 5 times, then try clock recovery if that fails */
		if (tries > 5) {
			vgt_dp_link_down(vgt, vgt_dp);
			vgt_dp_start_link_train(vgt, vgt_dp);
			tries = 0;
			cr_tries++;
			continue;
		}

		/* Compute new intel_dp->train_set as requested by target */
		vgt_get_adjust_train(vgt, vgt_dp, link_status);
		tries++;
	}

	if ((VGT_PCH(pdev) == PCH_CPT)
			&& ((VGT_GEN(pdev) == 7) || !IS_CPU_EDP(vgt_dp)))
		dp_ctrl = dp_ctrl | _REGBIT_DP_LINK_TRAIN_OFF_CPT;
	else
		BUG();

	if (dp_ctrl != 0x80080304)
		vgt_printk("dp_ctrl should be 0x80080304, but now it is 0x%08x",
				dp_ctrl);

	VGT_MMIO_WRITE(pdev, vgt_dp->dp_ctrl_reg, 0x80080304);
	VGT_POST_READ(pdev, vgt_dp->dp_ctrl_reg);

	value = DP_TRAINING_PATTERN_DISABLE;
	vgt_dp_aux_native_write(vgt, vgt_dp,
			DP_TRAINING_PATTERN_SET, &value, 1);
}

static void vgt_dp_commit(struct vgt_device *vgt,
		struct vgt_port_struct *port_struct)
{
	struct vgt_dp_port *vgt_dp;
	ASSERT(!port_struct);
	ASSERT(!(port_struct->private));

	vgt_dp = port_struct->private;
	/* TODO: eDP not supported yet */
	/* ironlake_edp_panel_vdd_on(intel_dp); */
	vgt_dp_sink_dpms(vgt, vgt_dp, DRM_MODE_DPMS_ON);
	vgt_dp_start_link_train(vgt, vgt_dp);
	/* eDP to be done */
	//ironlake_edp_panel_on(intel_dp);
	//ironlake_edp_panel_vdd_off(intel_dp, true);
	vgt_dp_complete_link_train(vgt, vgt_dp);
	//ironlake_edp_backlight_on(intel_dp);

	if (VGT_PCH(pdev) == PCH_CPT)
		vgt_cpt_verify_modeset(vgt, port_struct->attached_pipe);
}

//static void intel_dp_sink_dpms(intel_dp, DRM_MODE_DPMS_ON);

struct vgt_port_dsp_set_funcs lvds_dsp_set_funcs = {
	.mode_fixup = vgt_lvds_mode_fixup,
	.commit = vgt_lvds_enable,
};

struct vgt_port_dsp_set_funcs crt_dsp_set_funcs = {
	.prepare = vgt_crt_prepare,
	.mode_set = vgt_crt_mode_set,
	.commit = vgt_crt_commit,
};

struct vgt_dp_port default_dp_priv = {
	.dp_ctrl_reg = _REG_DP_B_CTL,
	.is_pch_edp = false,
};

struct vgt_port_dsp_set_funcs dp_dsp_set_funcs = {
	.prepare = vgt_dp_prepare,
	.commit = vgt_dp_commit,
};

int init_vgt_port_struct(struct vgt_device *vgt,
		enum vgt_pipe pipe,
		enum vgt_plane plane,
		enum vgt_output_type otype)
{
	struct vgt_port_struct *port;

	ASSERT(pipe < I915_MAX_PIPES);

	port = kzalloc(sizeof(struct vgt_port_struct), GFP_KERNEL);
	if (!port) {
		return -ENOMEM;
	}

	port->enabled = true;
	port->output_type = otype;
	port->attached_pipe = pipe;
	port->attached_plane = plane;

	switch (otype) {
		case VGT_OUTPUT_LVDS:
			port->port_dsp_set_funcs = &lvds_dsp_set_funcs;
			break;
		case VGT_OUTPUT_ANALOG:
			port->port_dsp_set_funcs = &crt_dsp_set_funcs;
			break;
		case VGT_OUTPUT_DISPLAYPORT:
			port->port_dsp_set_funcs = &dp_dsp_set_funcs;
			break;
		case VGT_OUTPUT_HDMI:
			BUG();
			break;
		case VGT_OUTPUT_EDP:
			BUG();
			break;
		default:
			BUG();
	}

	vgt->attached_port[pipe] = port;

	return 0;
}

struct vgt_dp_port *init_vgt_dp_port_private(
		unsigned int dp_ctrl_reg,
		bool is_pch_edp)
{
	struct vgt_dp_port *p = (struct vgt_dp_port *)kzalloc(
			sizeof(struct vgt_dp_port),
			GFP_KERNEL);
	if (!p) {
		kfree(p);
		return NULL;
	}

	ASSERT(!(dp_ctrl_reg & 0x3));

	p->dp_ctrl_reg = dp_ctrl_reg;
	p->is_pch_edp = is_pch_edp;

	return p;
}

static void vgt_destroy_single_port(struct vgt_device *vgt,
		enum vgt_pipe pipe)
{
	struct vgt_port_struct *port_struct = vgt->attached_port[pipe];
	ASSERT(pipe < I915_MAX_PIPES);
	if (port_struct) {
		if (port_struct->private) {
			kfree(port_struct->private);
			port_struct->private = NULL;
		}
		kfree(port_struct);
		vgt->attached_port[pipe] = NULL;
	}
}

void vgt_destroy_attached_port(struct vgt_device *vgt)
{
	enum vgt_pipe pipe;
	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; pipe++)
		vgt_destroy_single_port(vgt, pipe);
}

struct vgt_port_output_struct vgt_port_table[] = {
	{_REG_PCH_ADPA, _REGBIT_ADPA_DAC_ENABLE, PORT_TRANS_SEL_MASK, VGT_OUTPUT_ANALOG},
	{_REG_PCH_LVDS, _REGBIT_LVDS_PORT_ENABLE, LVDS_TRANS_SEL_MASK, VGT_OUTPUT_LVDS},
	{_REG_HDMI_B_CTL, _REGBIT_HDMI_PORT_ENABLE, HDMI_TRANS_SEL_MASK, VGT_OUTPUT_HDMI},
	{_REG_HDMI_C_CTL, _REGBIT_HDMI_PORT_ENABLE, HDMI_TRANS_SEL_MASK, VGT_OUTPUT_HDMI},
	{_REG_HDMI_D_CTL, _REGBIT_HDMI_PORT_ENABLE, HDMI_TRANS_SEL_MASK, VGT_OUTPUT_HDMI},

	/* DP on sandybridge, its trans_selection is not in ctrl reg */
	{_REG_TRANS_DP_A_CTL, _REGBIT_TRANS_DP_OUTPUT_ENABLE, _REGBIT_TRANS_DP_PORT_SEL_MASK, VGT_OUTPUT_DISPLAYPORT},
	{_REG_TRANS_DP_B_CTL, _REGBIT_TRANS_DP_OUTPUT_ENABLE, _REGBIT_TRANS_DP_PORT_SEL_MASK, VGT_OUTPUT_DISPLAYPORT},
	{_REG_TRANS_DP_C_CTL, _REGBIT_TRANS_DP_OUTPUT_ENABLE, _REGBIT_TRANS_DP_PORT_SEL_MASK, VGT_OUTPUT_DISPLAYPORT},
};

static enum vgt_pipe
vgt_get_pipe_from_ctrl_reg( struct vgt_device *vgt,
		struct vgt_port_output_struct *port)
{
	//enum vgt_pipe pipe;
	//struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t ctrl,
			  enable_bitmask = (1 << 31),
			  pipe_sel_bitmask = (3 << 29);

	ASSERT(port);
	ASSERT((port->ctrl_reg & 0x3) == 0);

	/* FIXME: read unused pipe ctrl reg, can cause hang ? */
	ctrl = __sreg(vgt, port->ctrl_reg);
	//ctrl = VGT_MMIO_READ(pdev, port->ctrl_reg);
	if ((ctrl & enable_bitmask) == 0)
		return I915_MAX_PIPES;

	switch (ctrl & pipe_sel_bitmask) {
		case 0:
			return PIPE_A;
		case (1 << 29):
			return PIPE_B;
		case (2 << 29):
			return PIPE_C;
		default:
			return I915_MAX_PIPES;
	}
}

static unsigned int
vgt_get_dp_from_transcoder(struct vgt_device *vgt,
		struct vgt_port_output_struct *port)
{
	//int dp_ctrl_reg;
	//struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t trans_dp_ctrl,
			  dp_sel_mask = (3 << 29),
			  enable_bitmask = (1 << 31);

	ASSERT(port);
	ASSERT((port->ctrl_reg & 0x3) == 0);

	trans_dp_ctrl = __sreg(vgt, port->ctrl_reg);
	//trans_dp_ctrl = VGT_MMIO_READ(pdev, port->ctrl_reg);
	if ((trans_dp_ctrl & enable_bitmask) == 0)
		return 0;

	switch (trans_dp_ctrl & dp_sel_mask) {
		case 0:
			return _REG_DP_B_CTL;
		case (1 << 29):
			return _REG_DP_C_CTL;
		case (2 << 29):
			return _REG_DP_D_CTL;
		default:
			return 0;
	}
}

/* Call it when first display-switch or after any hot-plug events */
static void vgt_detect_attached_ports(struct vgt_device *vgt)
{
	int i,ret;
	struct pgt_device *pdev = vgt->pdev;
	enum vgt_pipe pipe, max_pipe;
	struct vgt_port_output_struct *port;
	unsigned int dp_ctrl_reg;
	struct vgt_dp_port *vgt_dp;

	if (pdev->is_sandybridge)
		max_pipe = PIPE_B;
	else
		max_pipe = PIPE_C;

	vgt_destroy_attached_port(vgt);

	for (i = 0; i < sizeof(vgt_port_table)/sizeof(struct vgt_port_output_struct); i++) {
		port = &(vgt_port_table[i]);
		switch (port->ctrl_reg) {
			case _REG_PCH_ADPA:
			case _REG_PCH_LVDS:
				pipe = vgt_get_pipe_from_ctrl_reg(vgt, port);
				if (pipe != I915_MAX_PIPES) {
					vgt_printk("VGT(%d): detect port(0x%08x) use %s",
							vgt->vgt_id, port->ctrl_reg,
							VGT_PIPE_NAME(pipe));

					ret = init_vgt_port_struct(vgt, pipe,
							pipe, port->output_type);
					if (ret < 0)
						BUG();
				}
				break;
			case _REG_TRANS_DP_A_CTL:
			case _REG_TRANS_DP_B_CTL:
			case _REG_TRANS_DP_C_CTL:
				dp_ctrl_reg = vgt_get_dp_from_transcoder(vgt, port);
				if (dp_ctrl_reg != 0) {

					pipe = ((dp_ctrl_reg - _REG_TRANS_DP_A_CTL) >> 12);
					ASSERT(pipe < I915_MAX_PIPES);

					vgt_printk("detect DP port(0x%08x) use %s",
							dp_ctrl_reg, VGT_PIPE_NAME(pipe));

					ret = init_vgt_port_struct(vgt, pipe,
							pipe, port->output_type);

					/* TODO: default, not EDP */
					vgt_dp = init_vgt_dp_port_private(dp_ctrl_reg, false);
					if (vgt_dp == 0)
						BUG();

					/* Attach dp specific datastructure */
					vgt->attached_port[pipe]->private = vgt_dp;
				}
				break;
			case _REG_HDMI_B_CTL:
			case _REG_HDMI_C_CTL:
			case _REG_HDMI_D_CTL:
				break;
			default:
				BUG(); /* BUG() is only good for debugging :( */
		}
	}
}

static void vgt_scan_ports_for_all_domains(struct pgt_device *pdev)
{
	int i;
	struct vgt_device *vgt;
	ASSERT(pdev);
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt)
			vgt_detect_attached_ports(vgt);
	}
}

bool need_scan_attached_ports = true;
bool vgt_reinitialize_mode(struct vgt_device *cur_vgt,
		struct vgt_device *next_vgt)
{
	/* FIXME: default use DP_B */
	struct vgt_port_struct *port_struct;
	struct vgt_port_dsp_set_funcs *dsp_set_funcs;
	enum vgt_pipe pipe;

	ASSERT((cur_vgt && next_vgt));

	if (cur_vgt == next_vgt)
		return false;

	/* the early version refered bool drm_crtc_helper_set_mode()
	 */
	if (need_scan_attached_ports) {
		vgt_scan_ports_for_all_domains(cur_vgt->pdev);
		//need_scan_attached_ports = false;
	}

	/* FIXME: only support SNB */
	for (pipe = 0; pipe < I915_MAX_PIPES - 1; pipe++) {
		/* walk each pipe(port) that is enabled for incoming dpy owners */
		if (next_vgt->attached_port[pipe]) {

			port_struct = next_vgt->attached_port[pipe];

			dsp_set_funcs = port_struct->port_dsp_set_funcs;
			ASSERT(dsp_set_funcs != NULL);

			/* step A */
			/* fixup user mode settings
			 * encoder_funcs->mode_fixup(encoder, mode,
			 */
			/* 1) mode_fixup for lvds */
			/* 2) Nothing to do with vgt_crt_mode_fixup */
			/* 3) Nothing to dp with vgt_dp_mode fixup */
			if (dsp_set_funcs->mode_fixup)
				dsp_set_funcs->mode_fixup(cur_vgt, port_struct);

			/* step B */
			/* ctrc_funcs->mode_fixup()
			 * Nothing to do with all encoders/ports
			 * */

			/* step C */
			/* Prepare the encoders and CRTCs before setting the mode.
			   encoder_funcs->prepare(encoder);
			   */
			/* 1) Nothing to do with lvds */
			/* 2) power off with crt */
			/* 3) XXX with dp */
			if (dsp_set_funcs->prepare)
				dsp_set_funcs->prepare(cur_vgt, port_struct);

			/* step D
			 * FIXME: what's the purpose of "->get_ctrc()" ?
			   drm_crtc_prepare_encoders(dev);
			   It is just used to disable unused encoder, or to disable
			   encoders whose crts are going to be updated (i915 seems not
			   support these usages)
			 */
			/* Here we begin the real part for display mode set sequence */
			/* 1) drm_encoder_disable() for lvds */
			/* FIXME: How about no lvds disabling */
			//vgt_lvds_disable_encoder(cur_vgt);
			/* 2) nothing to do with crt, no callback registered */
			/* 3) DP: nothing to do with it */

			/* step E */
			/* ctrc_funcs->prepare */
			/* crtc disable */
			vgt_ironlake_crtc_disable(cur_vgt, port_struct);

			/*************** End of the current vgt *****************/

			/* step F */
			/* crtc_funcs->mode_set() */
			vgt_ironlake_crtc_mode_set(next_vgt, port_struct);

			/* step G */
			/* encoder_funcs->mode_set() */
			/* 1) LVDS nothing to do */
			/* 2) CRT*/
			/* 3) DP intel_dp_mode_set(): if we don't support audio
			 *    nothing to do here */
			if (dsp_set_funcs->mode_set)
				dsp_set_funcs->mode_set(next_vgt, port_struct);

			/* step H */
			/* crtc_funcs->commit() */
			vgt_ironlake_crtc_enable(next_vgt, port_struct);

			/* step I */
			/* encoder_funcs->commit(encoder) */
			if (dsp_set_funcs->commit)
				dsp_set_funcs->commit(next_vgt, port_struct);
		}
	}

	return true;
}
