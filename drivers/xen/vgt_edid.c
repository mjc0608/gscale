/*
 * vGT EDID virtualization module
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
 */

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include "vgt_edid.h"

#define DEBUG_VGT_EDID

typedef enum {
	VGT_EDID_INFO = 1,
	VGT_EDID_WARN = 2,
	VGT_EDID_ERROR = 3,
} vgt_edid_log_t;

#define EDID_LOG(log, emu, fmt, args...) 			\
	do {							\
		printk("[VGT_EDID");				\
		if (emu == 0x12345678)				\
			printk("]");				\
		else if (emu)					\
			printk("-EM]");				\
		else						\
			printk("-HW]");				\
		if (log == VGT_EDID_INFO) {			\
			printk("INFO: ");			\
		} else if (log == VGT_EDID_WARN) {		\
			printk("WARN: ");			\
		} else if (log == VGT_EDID_ERROR) {		\
			printk("ERROR: ");			\
		}						\
		printk(fmt, ##args);				\
		if (log == VGT_EDID_ERROR) {			\
			BUG();					\
		}						\
	} while (0)

#ifdef DEBUG_VGT_EDID

static int vgt_edid_log_level = 2;

#define ASSERT(x)						\
do { if (!(x)) 							\
{printk("Assert at %s line %d\n", __FILE__, __LINE__); 		\
BUG();}} while (0);

#define EDID_MSG(log, emu, fmt, args...) 			\
	do {							\
		if (log >= vgt_edid_log_level) {		\
			EDID_LOG(log, emu, fmt, ##args);	\
		}						\
	} while (0)

#define EDID_MSG_EH(log, fmt, args...) 				\
	EDID_MSG(log, 0x12345678, fmt, ##args)

#else /* DEBUG_VGT_EDID */

#define ASSERT(x)
#define EDID_MSG(log, emu, fmt, args...) 			\
	do {							\
		if (log >= VGT_EDID_WARN) {			\
			EDID_LOG(log, emu, fmt, ##args);	\
		}						\
	} while (0)

#define EDID_MSG_EH(log, fmt, args...) 				\
	EDID_MSG(log, 0x12345678, fmt, ##args)

#endif /* DEBUG_VGT_EDID */

/**************************************************************************
 *
 * EDID Slave implementation
 *
 *************************************************************************/
static void edid_select(void)
{
	return;
}

static void edid_snap_read_byte(void *slave, unsigned char value)
{
	//TODO: Check edid state
	vgt_edid_t *edid = (vgt_edid_t *)slave;
	EDID_MSG(VGT_EDID_INFO, false,
			"edid_snap_read_byte with offset %d and value %d\n",
			edid->current_write,
			value);
	if (edid->current_write < EDID_SIZE) {
		edid->edid_data->edid_block[edid->current_write] = value;
		edid->current_write ++;
	} else {
		EDID_MSG(VGT_EDID_ERROR, false,
			"edid_snap_read_byte() exceeds the size of EDID!\n");
	}
}

static void edid_snap_write_byte(void *slave, unsigned char value)
{
	/*
	 * Interface for the virtual EDID update operations.
	 * In EDID case, which is read only, the only case
	 * to have this is the writing of offset for reading.
	 * The "current_write" below is the offset for snapshot
	 * The "current_read" below is the offset for emulation
	 * set them both to be the input value.
	 */
	vgt_edid_t *edid = (vgt_edid_t *)slave;
        ASSERT((edid->current_write == 0) &&
	       (edid->current_read == 0));
        edid->current_write = value;
        edid->current_read = value;
	return;
}

static unsigned char edid_get_snap_byte(void *slave)
{
	vgt_edid_t *edid = (vgt_edid_t *)slave;
	if (edid->current_read >= EDID_SIZE) {
		EDID_MSG(VGT_EDID_ERROR, true,
			"edid_get_snap_byte() exceeds the size of EDID!\n");
	}
	EDID_MSG(VGT_EDID_INFO, true,
			"edid_get_snap_byte with offset %d and value %d\n",
			edid->current_read,
			edid->edid_data->edid_block[edid->current_read]);
	return edid->edid_data->edid_block[edid->current_read ++];
}

/* edid_snap_stop()
 *
 * finalize the EDID result, check whether there are extensions.
 * Check whether the src(this) is a valid snapshot.
 * If so, assign the src to dest;
 * otherwise, free src directly.
 *
 * It is only called when not working in emulation mode.
 */
static void edid_snap_stop(void *dst, vgt_i2c_slave_t *this)
{
	vgt_edid_data_t **dest = (vgt_edid_data_t **)dst;
	vgt_edid_t *edid = (vgt_edid_t *)this;
	if (!(edid->edid_data)) {
		return;
	}
	// TODO, handle the EDID having extensions.
	if (edid->current_write == EDID_SIZE) {
		/* TODO, more validation of the result. Currently just
		 * check the size.
		 */
		EDID_MSG(VGT_EDID_INFO, false,
		"edid_snap_stop is called with successful EDID info!\n");
		edid->current_write = edid->current_read = 0;
		*dest = edid->edid_data;
		edid->edid_data = NULL;
	} else {
		EDID_MSG(VGT_EDID_INFO, false,
		"edid_snap_stop is called with failed EDID info! (size:%d)\n",
		edid->current_write);
		kfree(edid->edid_data);
		edid->edid_data = NULL;
	}
	return;
}

vgt_edid_data_t *vgt_create_edid(void)
{
	vgt_edid_data_t *edid = NULL;
#if 0
	/* The creation of edid is disabled for two reasons:
	 * 1, The edid creation dynamically is only needed in EDID snapshot.
	 *   with the new approach of reading once in init code, the snapshot
	 *   is not needed any more.
	 *
	 * 2, the snapshot approach has potential memory leak issue if
	 *   driver code does not trigger stop function as expected. So if
	 *   snapshot is needed in future for some reason, we need to handle
	 *   the issue, although it is not a big problem.
	 */

	edid = kmalloc(sizeof(vgt_edid_data_t), GFP_KERNEL);
	if (!edid) {
		EDID_MSG(VGT_EDID_ERROR, false,
				"Failed to allocate edid memory!\n");
		return NULL;
	}

	memset(edid, 0, sizeof(vgt_edid_data_t));
#endif
	return edid;
}

/**************************************************************************
 *
 * GMBUS interface for I2C access
 *
 *************************************************************************/

/* TODO GMBUS definition copied from vgt_reg.h.
 * Could be removed after someday someone clean up vgt_reg.h
 * to make it smaller so that it could be included instead.
 */
#define _REG_PCH_GMBUS0		0xc5100
#define _REG_PCH_GMBUS1		0xc5104
#define _REG_PCH_GMBUS2		0xc5108
#define _REG_PCH_GMBUS3		0xc510c
#define _GMBUS_SATOER		(1<<10)

void vgt_i2c_handle_gmbus_read(vgt_i2c_bus_t *i2c_bus,
				unsigned int offset, void *p_data)
{
	int i;
	bool gmbus_emulate = (i2c_bus->gmbus.pedid && *i2c_bus->gmbus.pedid);
	vgt_edid_t *edid = (vgt_edid_t *)i2c_bus->current_slave;

	ASSERT(offset == _REG_PCH_GMBUS2 || offset == _REG_PCH_GMBUS3);

	if (offset == _REG_PCH_GMBUS2) {
		int value = 0;
		if (gmbus_emulate && edid) {
			value = (edid->current_read & 0x1ff);
		}

		if (i2c_bus->gmbus.inuse) {
			value |= _GMBUS_INUSE;
		} else {
			i2c_bus->gmbus.inuse = true;
		}

		/* always in hdw_ready and wait state */
		value |= _GMBUS_HW_READY_BIT;
		value |= _GMBUS_HW_WAIT_PHASE;

		if (edid && !(edid->edid_data)) {
			/* It is not valid EDID reading
			 * from really existed monitor.
			 */
			value |= _GMBUS_SATOER;
		}
		else {
			value |= _GMBUS_ACTIVE;
			if (i2c_bus->state == VGT_I2C_WAIT) {
				value |= _GMBUS_HW_WAIT_PHASE;
			}
		}
		*(int *)p_data = value;
	} else {
		if (!edid) {
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
				"GMBUS_3 reading without slave setting!\n");
			*(int *)p_data = 0;
			return;
		}
		ASSERT(offset == _REG_PCH_GMBUS3);
		ASSERT(i2c_bus->state == VGT_I2C_RECEIVE);
		if (gmbus_emulate) {
			*(int *)p_data = 0;
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
				"GMBUS reading snapshot for EDID!\n");
			for (i = 0; i < 4; ++ i) {
				unsigned char buffer =
					i2c_bus->current_slave->get_byte_from_snap(
						i2c_bus->current_slave);
				*(int *)p_data |= (buffer << (i << 3));
			}

			if (i2c_bus->gmbus.total_byte_count
				<= edid->current_read) {
				/* finished reading */
				if (i2c_bus->gmbus.cycle_type & 0x4) {
					EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
				"GMBUS goes to stop stage from data cycle.\n");
					vgt_init_i2c_bus(i2c_bus);
				} else {
					i2c_bus->state = VGT_I2C_WAIT;
				}
			}
		} else {
			int value = *(int *)p_data;
			unsigned num = 4;
			if (i2c_bus->gmbus.total_byte_count) {
				int l = i2c_bus->gmbus.total_byte_count -
						edid->current_write;
				ASSERT(l >= 0);
				if (l < 4) {
					num = l;
				}
			}

			for (i = 0; i < num; ++ i) {
				i2c_bus->current_slave->snap_read_byte(
					i2c_bus->current_slave,value & 0xff);
				value >>= 8;
			}
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
				"GMBUS doing snapshot for EDID!\n");
			/* There is no other better place to check the end of
			 * EDID reading...
			 */
			if (edid->current_write == EDID_SIZE) {
				i2c_bus->current_slave->snap_stop(
						i2c_bus->gmbus.pedid,
						i2c_bus->current_slave);
			}
		}
	}
	return;
}

/* vgt_i2c_handle_gmbus_write()
 *
 * Handle the write of GMBUS1, GMBUS2 and GMBUS3. The GMBUS0
 * access is handled in gmbus_mmio_write() because there are
 * some init logic there.
 */
void vgt_i2c_handle_gmbus_write(vgt_i2c_bus_t *i2c_bus,
				unsigned int offset, void *p_data)
{
	bool gmbus_emulate = (i2c_bus->gmbus.pedid && *i2c_bus->gmbus.pedid);

	if (offset == _REG_PCH_GMBUS1) {
		int value = *(int *)p_data;
		unsigned total_byte_count = 0;
		unsigned slave_addr = value & 0xff;
		I2C_STATE i2c_state = (slave_addr & 0x1) ? VGT_I2C_RECEIVE :
							   VGT_I2C_SEND;
		if (((value & (1 << 31)) == 0) &&
		    (i2c_bus->gmbus.gmbus1 & (1 << 31))) {
			/* According to PRM Vol3, part 3, Section 2.2.3.2,
			 * the write to SW_CLR_INT with 1 and 0 will reset
			 * GMBUS.
			 */
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS goes to stop stage by GMBUS reset.\n");
			vgt_init_i2c_bus(i2c_bus);
			return;
		}

		i2c_bus->gmbus.gmbus1 = value;
		slave_addr >>= 1;
		total_byte_count = (*(int *)p_data >> _GMBUS1_BYTE_LENGTH_POSI)
					& 0x1ff;

		if (slave_addr == EDID_ADDR) {
			i2c_bus->current_slave_addr = slave_addr;
			i2c_bus->current_slave =
					(vgt_i2c_slave_t *)&i2c_bus->edid_slave;
			if (gmbus_emulate) {
				i2c_bus->edid_slave.edid_data =
					*i2c_bus->gmbus.pedid;
			} else {
				i2c_bus->edid_slave.edid_data =
					(vgt_edid_data_t *) vgt_create_edid();
				EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
					"GMBUS created new EDID data.\n");
			}
			i2c_bus->state = i2c_state;
			i2c_bus->gmbus.total_byte_count = total_byte_count;

			if (*(int *)p_data & _GMBUS1_CYCLE_INDEX) {
				unsigned index = (*(int *)p_data >>
						_GMBUS1_BYTE_INDEX_POSI)
						& 0xff;
				i2c_bus->current_slave->snap_write_byte(
						i2c_bus->current_slave,
						index);
			}
		} else if (slave_addr != 0) {
			EDID_MSG(VGT_EDID_WARN, gmbus_emulate,
				"Slave that is not supported yet[addr:0x%x]!\n",
				slave_addr);
			dump_stack();
		}

		i2c_bus->gmbus.cycle_type = ((value >> 25) & 0x7);
		switch (i2c_bus->gmbus.cycle_type) {
		case GMBUS_NOCYCLE:
			/* ignore */
			break;
		case NIDX_NS_W:
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS is set NIDX_NS_W cycle.\n");
			break;
		case NIDX_STOP:
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS is set NIDX_STOP cycle.\n");
			break;
		case GMBUS_STOP:
			/* TODO: assert the wait stage? */
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS goes to stop stage by stop command.\n");
			vgt_init_i2c_bus(i2c_bus);
			break;
		case IDX_NS_W:
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS is set IDX_NS_W cycle.\n");
			break;
		case IDX_STOP:
			EDID_MSG(VGT_EDID_INFO, gmbus_emulate,
			"GMBUS is set IDX_STOP cycle.\n");
			break;
		default:
			EDID_MSG(VGT_EDID_WARN, gmbus_emulate,
			"Not supported CYCLE type written from GMBUS1!\n");
			break;
		}

	} else if (offset == _REG_PCH_GMBUS2) {
		if ((*(int *)p_data) & _GMBUS_INUSE) {
			i2c_bus->gmbus.inuse = false;
		}
	} else if (offset == _REG_PCH_GMBUS3) {
		int buffer = *(int *)p_data;
		if (i2c_bus->current_slave) {
			ASSERT(i2c_bus->state == VGT_I2C_SEND);

			/* Below only write the lower 8-bits. For EDID case,
			 * it should not happen. For normal case, the count
			 * has to be taken into account to know how many
			 * bytes in the 4-byte word will be written.
			 */
			i2c_bus->current_slave->snap_write_byte(
						i2c_bus->current_slave,
						buffer);
			/* Should not happen for EDID */
			ASSERT(0);
		} else {
			/* not yet set slave. Ignore it as kind of reset
			 * of register value
			 */
		}
	} else {
		EDID_MSG(VGT_EDID_WARN, gmbus_emulate,
			"Other gmbus register access? addr: 0x%x\n", offset);
		dump_stack();
	}

	return;
}

/**************************************************************************
 *
 * Aux CH interface for I2C access
 *
 *************************************************************************/

/* vgt_get_aux_ch_reg()
 *
 * return the AUX_CH register according to its lower 8 bits of the address
 */
static inline AUX_CH_REGISTERS vgt_get_aux_ch_reg(unsigned int offset)
{
	AUX_CH_REGISTERS reg;
	switch (offset & 0xff) {
	case 0x10:
		reg = AUX_CH_CTL;
		break;
	case 0x14:
		reg = AUX_CH_DATA1;
		break;
	case 0x18:
		reg = AUX_CH_DATA2;
		break;
	case 0x1c:
		reg = AUX_CH_DATA3;
		break;
	case 0x20:
		reg = AUX_CH_DATA4;
		break;
	case 0x24:
		reg = AUX_CH_DATA5;
		break;
	default:
		reg = AUX_CH_INV;
		break;
	}
	return reg;
}

void vgt_i2c_handle_aux_ch_read(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t **pedid,
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data)
{
	bool auxch_emulate;
	AUX_CH_REGISTERS reg;

	if (!i2c_bus->aux_ch.i2c_over_aux_ch || !i2c_bus->aux_ch.aux_ch_mot) {
		return;
	}

	auxch_emulate = (*pedid != NULL);
	reg = vgt_get_aux_ch_reg(offset);

	ASSERT(auxch_emulate);

	*(unsigned int *)p_data = *i2c_bus->aux_ch.aux_registers[port_idx][reg];
	EDID_MSG(VGT_EDID_INFO, auxch_emulate,
	"read reg[0x%x] with value:0x%x\n", offset, *(unsigned int *)p_data);
	return;
}

#define AUX_CTL_MSG_LENGTH(reg) \
	((reg & _DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >> \
		_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT)

void vgt_i2c_handle_aux_ch_write(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t **pedid,
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data)
{
	int msg_length, ret_msg_size;
	bool auxch_emulate;
	int msg, addr, ctrl, op;
	int value = *(int *)p_data;
	int aux_data_for_write = 0;
	AUX_CH_REGISTERS reg = vgt_get_aux_ch_reg(offset);
	auxch_emulate = (*pedid != NULL);

	EDID_MSG(VGT_EDID_INFO, auxch_emulate,
	"vgt_i2c_handle_aux_ch_write with offset:0x%x, port_idx:0x%x, value:0x%x\n",
		offset, port_idx, value);

	if (reg != AUX_CH_CTL) {
		*i2c_bus->aux_ch.aux_registers[port_idx][reg] = value;
		return;
	}

	msg_length = AUX_CTL_MSG_LENGTH(value);
	// check the msg in DATA register.
	msg = *i2c_bus->aux_ch.aux_registers[port_idx][reg + 1];
	addr = (msg >> 8) & 0xffff;
	ctrl = (msg >> 24)& 0xff;
	op = ctrl >> 4;
	if (!(value & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY)) {
		/* The ctl write to clear some states */
		return;
	}
#ifdef AUX_CH_WORKAROUND
	if ((op == VGT_AUX_NATIVE_WRITE) ||
	    (op == VGT_AUX_NATIVE_READ)) {
		/* do not handle it */
		return;
	}
#endif /* AUX_CH_WORKAROUND */

	if (!auxch_emulate) {
		return;
	}
	ASSERT(auxch_emulate);

	/* Always set the wanted value for vms. */
	ret_msg_size = (((op & 0x1) == VGT_AUX_I2C_READ) ? 2 : 1);
	*i2c_bus->aux_ch.aux_registers[port_idx][reg] =
		_REGBIT_DP_AUX_CH_CTL_DONE |
		((ret_msg_size << _DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) &
		  _DP_AUX_CH_CTL_MESSAGE_SIZE_MASK);

	if (msg_length == 3) {
		if (!(op & VGT_AUX_I2C_MOT)) {
			/* stop */
			EDID_MSG(VGT_EDID_INFO, auxch_emulate,
				"AUX_CH: stop. reset I2C!\n");
			vgt_init_i2c_bus(i2c_bus);
		} else {
			/* start or restart */
			EDID_MSG(VGT_EDID_INFO, auxch_emulate,
				"AUX_CH: start or restart I2C!\n");
			i2c_bus->aux_ch.i2c_over_aux_ch = true;
			i2c_bus->aux_ch.aux_ch_mot = true;
			if (addr == 0) {
				/* reset the address */
				EDID_MSG(VGT_EDID_INFO, auxch_emulate,
					"AUX_CH: reset I2C!\n");
				vgt_init_i2c_bus(i2c_bus);
			} else if (addr == EDID_ADDR) {
				EDID_MSG(VGT_EDID_INFO, auxch_emulate,
					"AUX_CH: setting EDID_ADDR!\n");
				i2c_bus->current_slave_addr = EDID_ADDR;
				i2c_bus->current_slave =
					(vgt_i2c_slave_t *)&i2c_bus->edid_slave;
				i2c_bus->edid_slave.edid_data = *pedid;
			} else {
				EDID_MSG(VGT_EDID_WARN, auxch_emulate,
		"Not supported address access [0x%x]with I2C over AUX_CH!\n",
				addr);
			}
		}
	} else if ((op & 0x1) == VGT_AUX_I2C_WRITE) {
		int write_length;
		int write_value;

		write_length = msg_length - 4;

		EDID_MSG(VGT_EDID_INFO, auxch_emulate,
			"AUX_CH WRITE length is:%d\n", write_length);

		ASSERT(write_length == 1);
		write_value = *i2c_bus->aux_ch.aux_registers[port_idx][reg + 2]
				 & 0xff;
		ASSERT(write_value == 0);
	} else {
		ASSERT((op & 0x1) == VGT_AUX_I2C_READ);
		ASSERT(msg_length == 4);
		if (i2c_bus->current_slave) {
			/* seems that data burst of aux_ch for i2c can only work
			 * for write. The read can always read just one byte
			 */
			unsigned char val = i2c_bus->current_slave->
				get_byte_from_snap(i2c_bus->current_slave);
			aux_data_for_write = (val << 16);
		}
	}

	/* write the return value in AUX_CH_DATA reg which includes:
	 *  ACK of I2C_WRITE
	 *  returned byte if it is READ
	 */
	aux_data_for_write |= (VGT_AUX_I2C_REPLY_ACK & 0xff) << 24;
	*i2c_bus->aux_ch.aux_registers[port_idx][reg + 1] = aux_data_for_write;

	return;
}

/**************************************************************************
 * I2C
 *************************************************************************/
void vgt_init_i2c_bus(vgt_i2c_bus_t *i2c_bus)
{
	i2c_bus->current_slave_addr = 0;
	i2c_bus->current_slave = NULL;
	i2c_bus->state = VGT_I2C_STOP;

	/* initialize the edid_slave
	 */
	i2c_bus->edid_slave.slave.select = edid_select;
	i2c_bus->edid_slave.slave.snap_read_byte = edid_snap_read_byte;
	i2c_bus->edid_slave.slave.snap_write_byte = edid_snap_write_byte;
	i2c_bus->edid_slave.slave.get_byte_from_snap = edid_get_snap_byte;
	i2c_bus->edid_slave.slave.snap_stop = edid_snap_stop;
	i2c_bus->edid_slave.current_read = 0;
	i2c_bus->edid_slave.current_write = 0;
	i2c_bus->edid_slave.edid_data = NULL;

#ifdef ENABLE_GPIO_EMULATION
	memset(&i2c_bus->bitbang, 0, sizeof(vgt_i2c_bitbang_t));
	/* default line state is high level, which means
	 * line is free
	 */
	i2c_bus->bitbang.sda_state = VGT_I2C_HIGH;
	i2c_bus->bitbang.sda_state = VGT_I2C_HIGH;
#endif /* ENABLE_GPIO_EMULATION */

	memset(&i2c_bus->gmbus, 0, sizeof(vgt_i2c_gmbus_t));

	i2c_bus->aux_ch.i2c_over_aux_ch = false;
	i2c_bus->aux_ch.aux_ch_mot = false;
	/*two arrays of aux_shadow_reg and aux_registers are initialized once
	 * in vgt_init_aux_ch_vregs(), which is not supposed to change again
	 */
}
