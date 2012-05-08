/*
 * Interface between Gfx dricer and vGT enabled hypervisor
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


/* Reserve 32KB for vGT shared infor: 0x78000-0x7FFFF */
#define VGT_PVINFO_PAGE	0x78000
#define VGT_PVINFO_SIZE	0x8000

/*
 * The following structure pages are defined in GEN MMIO space for virtualization.
 * (One page for now)
 */
#define    VGT_MAGIC         0x4776544776544776    /* 'vGTvGTvG' */
struct vgt_if {
    uint64_t  magic;      /* VGT_MAGIC */
    uint16_t  version_major;
    uint16_t  version_minor;
    uint32_t  rsv1;
    uint32_t  display_ready;/* ready for display owner switch */
    uint32_t  vgt_id;       /* ID of vGT instance */
    uint32_t  rsv2[10];	    /* pad to offset 0x40 */
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
        } aperture;
        /* GMADR register balooning */
        struct    {
           uint32_t  my_base;
           uint32_t  my_size;
        } gmadr;
        /* fence register balooning */
        struct    {
           uint32_t  my_base;
           uint32_t  my_size;
        } fence;
        uint32_t  rsv2[2];
    } avail_rs;			/* available/assigned resource */
    uint32_t  rsv3[0x400-0x60];	/* pad to one page */
};

#define vgt_info_off(x)        (VGT_PVINFO_PAGE + (long)&((struct vgt_if*) NULL)->x)
