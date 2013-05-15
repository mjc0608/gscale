/*
 * PCI Configuration Space virtualization
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

#include <asm/xen/hypercall.h>

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

/*
 * emulate the GetBIOSData function with 'supported calls' subfunction
 */
static void vgt_hvm_opregion_handle_request(struct vgt_device *vgt, uint32_t swsci)
{
	uint32_t *scic, *parm;
	scic = vgt->state.opregion_va + VGT_OPREGION_REG_SCIC;
	parm = vgt->state.opregion_va + VGT_OPREGION_REG_PARM;

	if (!(swsci & _REGBIT_CFG_SWSCI_SCI_SELECT)) {
		vgt_warn("VM%d requesting SMI service\n", vgt->vm_id);
		return;
	}
	/* ignore non 0->1 trasitions */
	if ((vgt->state.cfg_space[VGT_REG_CFG_SWSCI_TRIGGER] &
				_REGBIT_CFG_SWSCI_SCI_TRIGGER) ||
			!(swsci & _REGBIT_CFG_SWSCI_SCI_TRIGGER)) {
		return;
	}

	if (!vgt_opregion_is_capability_get(*scic)) {
		vgt_warn("VM%d requesting runtime service: func 0x%x, subfunc 0x%x\n",
				vgt->vm_id, VGT_OPREGION_FUNC(*scic),
				VGT_OPREGION_SUBFUNC(*scic));
		return;
	}

	*scic = 0;
	*parm = 0;
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
	u8 old_cmd, cmd_changed; /* we don't care the high 8 bits */
	bool rc = true;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_COMMAND:
			old_cmd = vgt->state.cfg_space[off];
			cmd_changed = old_cmd ^ (*(u8*)p_data);
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			if (cmd_changed & _REGBIT_CFG_COMMAND_MEMORY) {
				if (old_cmd & _REGBIT_CFG_COMMAND_MEMORY) {
					 vgt_hvm_map_apperture(vgt, 0);
					/* need unset trap area? */
				} else {

					vgt_hvm_map_apperture(vgt, 1);
					vgt_hvm_set_trap_area(vgt);
				}
			} else {
				vgt_dbg("need to trap the PIO BAR? "
					"old_cmd=0x%x, cmd_changed=%0x",
					old_cmd, cmd_changed);
			}
			break;
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
			ASSERT((bytes == 4) && (off & 3) == 0);

			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
			if (new == 0xFFFFFFFF) {
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
			new = *(uint32_t *)p_data;
			if (vgt->vm_id == 0)
				rc = vgt_cfg_sci_write(vgt, off, p_data, bytes);
			else
				vgt_hvm_opregion_handle_request(vgt, new);
			break;

		case VGT_REG_CFG_OPREGION:
			new = *(uint32_t *)p_data;
			if (vgt->vm_id == 0) {
				/* normally domain 0 shouldn't write this reg */
				memcpy(&vgt->state.cfg_space[off], p_data, bytes);
			} else if (vgt->state.opregion_va == NULL) {
				vgt_hvm_opregion_init(vgt, new);
				memcpy(&vgt->state.cfg_space[off], p_data, bytes);
			} else
				vgt_warn("VM%d write OPREGION multiple times",
						vgt->vm_id);
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
