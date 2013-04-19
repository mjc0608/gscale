/*
 * vgt.h: core header file for vGT driver
 *
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

#ifndef _VGT_H_
#define _VGT_H_

// structures
struct vgt_device;
typedef struct {
    bool (*mem_read)(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
    bool (*mem_write)(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
    bool (*cfg_read)(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
    bool (*cfg_write)(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
    bool boot_time;	/* in boot time dom0 access is always passed through */
    bool initialized;	/* whether vgt_ops can be referenced */
} vgt_ops_t;
extern vgt_ops_t *vgt_ops;
#define vgt_is_dom0(id)	(id == 0)

/* get the bits high:low of the data, high and low is starting from zero*/
#define VGT_GET_BITS(data, high, low)	(((data) & ((1 << ((high) + 1)) - 1)) >> (low))
/* get one bit of the data, bit is starting from zeor */
#define VGT_GET_BIT(data, bit)		VGT_GET_BITS(data, bit, bit)

bool vgt_emulate_write(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
bool vgt_emulate_read(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
bool vgt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);
bool vgt_emulate_cfg_read(struct vgt_device *vgt, unsigned int off, void *p_data, int bytes);

// function prototype definitions
// defined in arch specific file
extern int xen_register_vgt_driver(vgt_ops_t *ops);
extern int xen_start_vgt(struct pci_dev *pdev);
extern void xen_vgt_dom0_ready(struct vgt_device *vgt);
extern void xen_deregister_vgt_device(struct vgt_device *vgt);

extern int hcall_mmio_read(
        unsigned long port,
        unsigned int bytes,
        unsigned long *val);

extern int hcall_mmio_write(
        unsigned long port,
        unsigned int bytes,
        unsigned long val);

extern int hcall_vgt_ctrl(unsigned long ctrl_op);
/*
 * if this macro is defined, vgt will map GMA [0,64M] to the same page as [128M,192M] in GTT
 * this macro should be used together with DOM0_NON_IDENTICAL macro
 * it is only for debuging purpose
 * */
//#define DOM0_DUAL_MAP

/* save the fixed/translated guest address
 * restore the address after the command is executed
*/
#define VGT_ENABLE_ADDRESS_FIX_SAVE_RESTORE

// MMIO definitions

#endif /* _VGT_H */
