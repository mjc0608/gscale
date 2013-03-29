/*
 * PCI Configuration Space virtualization
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 */

#include <asm/xen/hypercall.h>

#include <xen/vgt.h>
#include "vgt.h"

typedef union _SCI_REG_DATA{
	uint16_t data;
	struct {
		uint16_t trigger:1; /* bit 0: trigger SCI */
		uint16_t reserve:14;
		uint16_t method:1; /* bit 15: 1 - SCI, 0 - SMI */
	};
} SCI_REG_DATA;

static bool vgt_cfg_sci_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	printk("VM%d Read SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint16_t*)p_data);

	return true;
}

static bool vgt_cfg_sci_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	SCI_REG_DATA sci_reg;

	printk("VM%d Write SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint32_t*)p_data);

	if( (bytes == 2) || (bytes == 4)){
		memcpy (&vgt->state.cfg_space[offset], p_data, bytes);
	} else {
		printk("Warning: VM%d vgt_cfg_sci_write invalid bytes=%d, ignore it\n", vgt->vm_id, bytes);
		return false;
	}

	sci_reg.data = *(uint16_t*)(vgt->state.cfg_space + offset);
	sci_reg.method = 1; /* set method to SCI */
	if (sci_reg.trigger == 1){
		printk("SW SCI Triggered by VM%d\n", vgt->vm_id);
		/* TODO: add SCI emulation */
		sci_reg.trigger = 0; /* SCI completion indicator */
	}

	memcpy (&vgt->state.cfg_space[offset], &sci_reg.data , 2);

	return true;
}

bool vgt_emulate_cfg_read(struct vgt_device *vgt, unsigned int offset, void *p_data, int bytes)
{

	ASSERT ((offset + bytes) <= VGT_CFG_SPACE_SZ);
	memcpy(p_data, &vgt->state.cfg_space[offset], bytes);

	/* TODO: hooks */
	offset &= ~3;
	switch (offset) {
		case 0:
		case 4:
		break;
		case VGT_REG_CFG_SWSCI_TRIGGER:
			vgt_cfg_sci_read(vgt, offset, p_data, bytes);
			break;
		default:
		break;
	}
	return true;
}

bool vgt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, int bytes)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint32_t *cfg_reg, new, size;
	bool rc = true;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
			ASSERT((bytes == 4) && (off & 3) == 0);

			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
			if ( new == 0xFFFFFFFF || new == 0xFFFFF800 ) {
				/*
				 * Power-up software can determine how much address
				 * space the device requires by writing a value of
				 * all 1's to the register and then reading the value
				 * back. The device will return 0's in all don't-care
				 * address bits.
				 */
				new = new & ~(size-1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
			} else {
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_apperture(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt);
			}
			break;

		case VGT_REG_CFG_SPACE_MSAC:
			printk("Guest write MSAC %x, %d: Not supported yet\n",
					*(char *)p_data, bytes);
			break;

		case VGT_REG_CFG_SWSCI_TRIGGER:
			rc = vgt_cfg_sci_write(vgt, off, p_data, bytes);
			break;

		case VGT_REG_CFG_SPACE_BAR1+4:
		case VGT_REG_CFG_SPACE_BAR0+4:
		case VGT_REG_CFG_SPACE_BAR2+4:
			ASSERT((bytes == 4) && (off & 3) == 0);
			if (*(uint32_t *)p_data == 0xFFFFFFFF)
				/* BAR size is not beyond 4G, so return all-0 in uppper 32 bit */
				*cfg_reg = 0;
			else
				*cfg_reg = *(uint32_t*)p_data;
			break;
		case 0x90:
		case 0x94:
		case 0x98:
			printk("vGT: write to MSI capa(%x) with val (%x)\n", off, *(uint32_t *)p_data);
		default:
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			break;
	}
	/*
	 * Assume most Dom0's cfg writes should be propagated to
	 * the real conf space. In the case where propagation is required
	 * but value needs be changed (sReg), do it here
	 */
	return rc;
}

void vgt_hvm_write_cf8_cfc(struct vgt_device *vgt,
	unsigned int port, unsigned int bytes, unsigned long val)
{
	vgt_dbg("vgt_hvm_write_cf8_cfc %x %d %lx\n", port, bytes, val);
	if ( (port & ~3) == 0xcf8 ) {
		ASSERT (bytes == 4);
		ASSERT ((port & 3) == 0);
		vgt->last_cf8 = (uint32_t) val;
	}
	else {
		ASSERT((vgt->last_cf8 & 3) == 0);
		ASSERT(((bytes == 4) && ((port & 3) == 0)) ||
			((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
		vgt_emulate_cfg_write (vgt,
			(vgt->last_cf8 & 0xfc) + (port & 3),
			&val, bytes);
	}
}

void vgt_hvm_read_cf8_cfc(struct vgt_device *vgt,
	unsigned int port, unsigned int bytes, unsigned long *val)
{
	unsigned long data;

	if ((port & ~3)== 0xcf8) {
		memcpy(val, (uint8_t*)&vgt->last_cf8 + (port & 3), bytes);
	}
	else {
//		ASSERT ( (vgt->last_cf8 & 3) == 0);
		ASSERT ( ((bytes == 4) && ((port & 3) == 0)) ||
			((bytes == 2) && ((port & 1) == 0)) || (bytes ==1));
		vgt_emulate_cfg_read(vgt, (vgt->last_cf8 & 0xfc) + (port & 3),
					&data, bytes);
		memcpy(val, &data, bytes);
	}
	vgt_dbg("VGT: vgt_cfg_read_emul port %x bytes %x got %lx\n",
			port, bytes, *val);
}
