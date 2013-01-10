/*
 * Interface between Gfx dricer and vGT enabled hypervisor
 *
 * This file is provided under GPLv2 license.
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
 */

#ifndef _VGT_IF_H
#define _VGT_IF_H


/* Reserve 32KB for vGT shared infor: 0x78000-0x7FFFF */
#define VGT_PVINFO_PAGE	0x78000
#define VGT_PVINFO_SIZE	0x8000

/*
 * The following structure pages are defined in GEN MMIO space for virtualization.
 * (One page for now)
 */
#define    VGT_MAGIC         0x4776544776544776    /* 'vGTvGTvG' */
#define    VGT_VERSION_MAJOR 1
#define    VGT_VERSION_MINOR 0

struct vgt_if {
    uint64_t  magic;      /* VGT_MAGIC */
    uint16_t  version_major;
    uint16_t  version_minor;
    uint32_t  vgt_id;       /* ID of vGT instance */
    uint32_t  rsv2[12];	    /* pad to offset 0x40 */
    /*
     *  Data structure to describe the balooning info of resources.
     *  Each VM can only have one portion of continuous area for now.
     *  (May support scattered resource in future)
     *  (next starting from offset 0x40)
     */
    struct {
        /* Aperture register balooning */
        struct    {
           uint32_t  my_base;
           uint32_t  my_size;
        } low_gmadr;		/* aperture */
        /* GMADR register balooning */
        struct    {
           uint32_t  my_base;
           uint32_t  my_size;
        } high_gmadr;		/* non aperture */
        /* allowed fence registers */
        uint32_t fence_num;
        uint32_t  rsv2[3];
    } avail_rs;			/* available/assigned resource */
    uint32_t  rsv3[0x200-0x60];	/* pad to half page */
    /*
     * The bottom half page is for the response from Gfx driver to hypervisor.
     */
    uint16_t  drv_version_major;
    uint16_t  drv_version_minor;
    uint32_t  display_ready;/* ready for display owner switch */
    /*
     * driver reported status/error code
     *     0: if the avail_rs is sufficient to driver
     *  Bit 2,1,0 set indicating
     *       Insufficient low_gmadr, high_gmadr, fence resources.
     *  Other bits are reserved.
     */
    uint32_t  rs_insufficient;
    /*
     * The driver is required to update the following field with minimal
     * required resource size.
     */
    uint32_t  min_low_gmadr;
    uint32_t  min_high_gmadr;
    uint32_t  min_fence_num;
    uint32_t  rsv4[0x200-6];    /* pad to one page */
};

#define vgt_info_off(x)        (VGT_PVINFO_PAGE + (long)&((struct vgt_if*) NULL)->x)

#endif /* _VGT_IF_H */
