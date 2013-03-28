/*
 * GPIO virtualization module
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

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include "edid.h"

#define DEBUG_VGT_EDID

typedef enum {
	VGT_EDID_INFO = 1,
	VGT_EDID_WARN = 2,
	VGT_EDID_ERROR = 3,
} vgt_edid_log_t;

#define EDID_LOG(log, emu, fmt, args...)			\
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
do { if (!(x))							\
{printk("Assert at %s line %d\n", __FILE__, __LINE__);		\
BUG();}} while (0);

#define EDID_MSG(log, emu, fmt, args...)			\
	do {							\
		if (log >= vgt_edid_log_level) {		\
			EDID_LOG(log, emu, fmt, ##args);	\
		}						\
	} while (0)

#define EDID_MSG_EH(log, fmt, args...)				\
	EDID_MSG(log, 0x12345678, fmt, ##args)

#else /* DEBUG_VGT_EDID */

#define ASSERT(x)
#define EDID_MSG(log, emu, fmt, args...)			\
	do {							\
		if (log >= VGT_EDID_WARN) {			\
			EDID_LOG(log, emu, fmt, ##args);	\
		}						\
	} while (0)

#define EDID_MSG_EH(log, fmt, args...)				\
	EDID_MSG(log, 0x12345678, fmt, ##args)

#endif /* DEBUG_VGT_EDID */

#define bit_bang i2c_bus->bitbang

/**************************************************************************
 *
 * GPIO interface for I2C access with bit-banging
 *
 *************************************************************************/
static inline I2C_LINE_STATE vgt_get_i2c_out_value(int value, I2C_LINE line)
{
	int ret;
	if (line == VGT_I2C_SCL) {
		ASSERT(VGT_IS_I2C_SCL(value));
		ret = VGT_GET_I2C_OUT_SCL(value);
	} else {
		ASSERT(VGT_IS_I2C_SDA(value));
		ret = VGT_GET_I2C_OUT_SDA(value);
	}

	return ((ret == 0) ? VGT_I2C_LOW : VGT_I2C_HIGH);
}

static inline I2C_LINE_STATE vgt_get_i2c_in_value(int value, I2C_LINE line)
{
	ASSERT(line == VGT_I2C_SDA);
	return ((value & _GPIO_DATA_VAL_IN) ? VGT_I2C_HIGH : VGT_I2C_LOW);
}

/* vgt_recognize_mmio_command()
 *
 * recognize the real operation corresponding to the MMIO command. The reason
 * to have this function is because that I2C one bit read or write corresponds
 * to multiple MMIO commands. This function will return the I2C one bit
 * operations at the end of MMIO command sequence.
 *
 * This function will modify below fields in vgt_i2c_bitbang_t:
 *	current_line
 *	write_enabled
 *	read_enabled
 *	pending_write_bit
 */
static I2C_COMMANDS vgt_recognize_mmio_command(I2C_COMMANDS cmd,
				vgt_i2c_bitbang_t *bitbang, int value)
{
	I2C_COMMANDS ret;
	if (cmd == GPIO_MMIO_WRITE) {
		if (VGT_IS_I2C_SCL(value)) {
			bitbang->current_line = VGT_I2C_SCL;
			bitbang->write_enabled = bitbang->read_enabled = true;
			bitbang->pending_write_bit =
				vgt_get_i2c_out_value(value, VGT_I2C_SCL);
		} else if (VGT_IS_I2C_SDA(value)){
			bitbang->current_line = VGT_I2C_SDA;
			bitbang->write_enabled = bitbang->read_enabled = true;
			bitbang->pending_write_bit =
				vgt_get_i2c_out_value(value, VGT_I2C_SDA);
		} else {
			/* since the DIR_MASK is used to recognize the
			 * SCL/SDA command, it the bit is not set, it
			 * is usually the second MMIO WRITE in I2C READ BIT.
			 */
			bitbang->write_enabled = false;
		}
		return I2C_COMMAND_IGNORE;
	} else {
		ASSERT(cmd == GPIO_MMIO_READ);
		if (bitbang->write_enabled) {
			ret = I2C_BIT_WRITE;
		} else {
			if (bitbang->read_enabled) {
				ret = I2C_BIT_READ;
			} else {
				/* for instance, the read of reserved bit
				 * from GPIO MMIO
				 */
				ret = I2C_COMMAND_IGNORE;
			}
		}
		bitbang->write_enabled = bitbang->read_enabled = false;
	}

	return ret;
}

/* vgt_i2c_handle_bit_read()
 *
 * Use the bit value from hardware to update the the storage in i2c slave
 * when mmio_read happens
 */
static int vgt_i2c_handle_bit_read(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t **pedid, void * p_data)
{
	int bit_value;
	int value = *(int *)p_data;
	bool i2c_emulate;

	i2c_emulate = (*pedid != NULL);

	/* ignore the read of scl. */
	if (bit_bang.current_line == VGT_I2C_SCL) {
		if (i2c_emulate) {
			int ret_val = 0;
			if (bit_bang.scl_state == VGT_I2C_HIGH) {
				ret_val = _GPIO_CLOCK_VAL_IN;
			}
			ret_val |= (_GPIO_CLOCK_VAL_MASK |
					_GPIO_CLOCK_DIR_MASK |
					_GPIO_CLOCK_DIR_IN);
			*((int *)p_data) = ret_val;
		}
		return 0;
	}

	/* should not happen in SCL low level */
	ASSERT(bit_bang.scl_state == VGT_I2C_HIGH);

	if (i2c_bus->state == VGT_I2C_SEND) {
		/* ACK of SENDING. */
		ASSERT(i2c_bus->current_slave);
		ASSERT(bit_bang.bit_length == 8);
		{
			i2c_bus->current_slave->snap_write_byte(
						i2c_bus->current_slave,
						bit_bang.buffer);
			bit_bang.buffer = bit_bang.bit_length = 0;
			bit_bang.sda_ack_state = false;
		}
		return 0;
	} else if (i2c_bus->state == VGT_I2C_START) {
		if (bit_bang.bit_length == 8) {
			/* ACK of 7-bit address writing */
			unsigned int addr = bit_bang.buffer >> 1;
			unsigned int is_read = bit_bang.buffer & 1;
			i2c_bus->state = (is_read == 1 ? VGT_I2C_RECEIVE :
							VGT_I2C_SEND);

			EDID_MSG(VGT_EDID_INFO, i2c_emulate,
				"I2C state is changed to be %s!\n",
					(is_read == 1) ? "RECEIVE" : "SEND");

			if (addr == EDID_ADDR) {
				if (!i2c_emulate && !(i2c_bus->current_slave)) {
					i2c_bus->current_slave_addr = addr;

					i2c_bus->current_slave =
					(vgt_i2c_slave_t *)&i2c_bus->edid_slave;

					i2c_bus->edid_slave.edid_data =
					(vgt_edid_data_t *) vgt_create_edid();
				} else if (i2c_bus->current_slave) {
					if (addr !=
						i2c_bus->current_slave_addr) {
						EDID_MSG(VGT_EDID_ERROR, i2c_emulate,
				"I2C slave was assigned to another address!!!\n");
					}
				} else {
					/* is emulate mode */
					i2c_bus->current_slave_addr = addr;

					i2c_bus->current_slave =
					(vgt_i2c_slave_t *)&i2c_bus->edid_slave;

					i2c_bus->edid_slave.edid_data = *pedid;
				}
				ASSERT(i2c_bus->current_slave);
				i2c_bus->current_slave->select();
			} else {
				EDID_MSG(VGT_EDID_WARN, i2c_emulate,
		"Unknown I2C device was accessed!(addr:0x%x) \
Please add handler!\n",addr);
				dump_stack();
			}
			bit_bang.buffer = bit_bang.bit_length = 0;
			bit_bang.sda_ack_state = false;
		}
		return 0;
	} else if (i2c_bus->state == VGT_I2C_STOP) {
		/* This happens in the dummy read in i2c_start, in which the start
		 * state is not yet setup. Safe to ignore it.
		 */
		return 0;
	}

	ASSERT(i2c_bus->current_slave_addr);
	ASSERT(i2c_bus->state == VGT_I2C_RECEIVE);

	if (i2c_emulate) {
		int ret_val;
		if (bit_bang.bit_length == 0) {
			bit_bang.buffer = i2c_bus->current_slave->get_byte_from_snap(
						i2c_bus->current_slave);
		}
		bit_value = ((bit_bang.buffer &
				(0x80 >> bit_bang.bit_length)) != 0);

		ret_val = 0;
		if (bit_value) {
			ret_val = _GPIO_DATA_VAL_IN;
		}
		ret_val |= (_GPIO_DATA_VAL_MASK |
					_GPIO_DATA_DIR_MASK |
					_GPIO_DATA_DIR_IN);
		*((int *)p_data) = ret_val;

	} else {
		bit_bang.sda_state = vgt_get_i2c_in_value(value, VGT_I2C_SDA);
		bit_value = (bit_bang.sda_state == VGT_I2C_LOW ? 0 : 1);
		bit_bang.buffer = (bit_bang.buffer << 1) | bit_value;
	}

	bit_bang.bit_length ++;
	ASSERT(bit_bang.bit_length <= 8);
	if (bit_bang.bit_length == 8) {
		bit_bang.sda_ack_state = true;
	}

	EDID_MSG(VGT_EDID_INFO, i2c_emulate,
			"read bit SDA value %d\n", bit_value);

	return 0;
}

/* vgt_i2c_handle_bit_write()
 *
 * Perform necessary operations in response to the write bit on I2C bus.
 * At the begining, there could be two bytes writing:
 * One is the writing of 7-bit address and R/W bit to I2C bus;
 * the other is the byte offset.
 * Notice that 10-bit address is not supported right now because of no need.
 *
 * In the case of EDID reading, there is only one byte writing, which is the
 * DDC address of 0x50. Extended EDID requires offset writing but the EEDID is
 * not used anyway.
 */
static int vgt_i2c_handle_bit_write(vgt_i2c_bus_t *i2c_bus,
					vgt_edid_data_t **pedid)
{
	int bit_value = 0;
	bool i2c_emulate;

	i2c_emulate = (*pedid != NULL);

	if (bit_bang.current_line == VGT_I2C_SCL) {
		bit_bang.scl_state = bit_bang.pending_write_bit;
		return 0;
	} else {
		I2C_LINE_STATE new_sda_state;
		ASSERT(bit_bang.current_line == VGT_I2C_SDA);

		new_sda_state = bit_bang.pending_write_bit;
		EDID_MSG(VGT_EDID_INFO, i2c_emulate,
				"Dealing with one I2C SDA bit write: %d\n",
				new_sda_state);

		if (IS_I2C_START(bit_bang.scl_state,
				bit_bang.sda_state,
				new_sda_state)) {

			/* start new I2C transmission */
			bit_bang.sda_state = new_sda_state;
			if (i2c_bus->state == VGT_I2C_STOP) {
				EDID_MSG(VGT_EDID_INFO, i2c_emulate,
					"I2C state is changed to be START!\n");
			} else {
				EDID_MSG(VGT_EDID_INFO, i2c_emulate,
					"I2C is restarted.\n");
			}
			i2c_bus->state = VGT_I2C_START;
			bit_bang.buffer = bit_bang.bit_length = 0;
			return 0;
		} else if (IS_I2C_STOP(bit_bang.scl_state,
					bit_bang.sda_state,
					new_sda_state)) {
			/* stop the I2C data transmission */
			ASSERT(new_sda_state == VGT_I2C_HIGH);
			EDID_MSG(VGT_EDID_INFO, i2c_emulate,
				"I2C state is changed to be STOP!\n");
			if (!i2c_emulate && i2c_bus->current_slave) {
				i2c_bus->current_slave->snap_stop(
						pedid,
						i2c_bus->current_slave);
			}
			vgt_init_i2c_bus(i2c_bus);
			return 0;
		}

		bit_bang.sda_state = new_sda_state;
	}

	/* write sda bit; */
	if (i2c_bus->state == VGT_I2C_RECEIVE) {
		ASSERT(i2c_bus->current_slave);
		/* ACK of read */
		if (bit_bang.sda_ack_state == true) {
			ASSERT(bit_bang.bit_length == 8);
			if (!i2c_emulate) {
				i2c_bus->current_slave->snap_read_byte(
						i2c_bus->current_slave,
						bit_bang.buffer);
			}
			bit_bang.buffer = bit_bang.bit_length = 0;
			bit_bang.sda_ack_state = false;
		} else {
			/* there are some write commands in clock low level
			 * to set SDA to be high. Those are safe to ignore.
			 */
		}
	} else if ((i2c_bus->state == VGT_I2C_START) ||
		(i2c_bus->state == VGT_I2C_SEND)) {
		if (!bit_bang.sda_ack_state) {
			bit_value = (bit_bang.sda_state == VGT_I2C_LOW ? 0 : 1);
			bit_bang.buffer = (bit_bang.buffer << 1) | bit_value;
			bit_bang.bit_length ++;
			ASSERT(bit_bang.bit_length <= 8);
			if (bit_bang.bit_length == 8) {
				bit_bang.sda_ack_state = true;
			}
		}
	}

	return 0;
}

void vgt_i2c_handle_gpio_read(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t **pedid, void *p_data)
{
	I2C_COMMANDS cmd;

	cmd = vgt_recognize_mmio_command(GPIO_MMIO_READ,
					&bit_bang, *(int *)p_data);
	if (cmd == I2C_COMMAND_IGNORE) {
		return;
	}

	if (cmd == I2C_BIT_READ) {
		vgt_i2c_handle_bit_read(i2c_bus,
					pedid,
					p_data);
	} else {
		ASSERT(cmd == I2C_BIT_WRITE);
		vgt_i2c_handle_bit_write(i2c_bus, pedid);
	}

	return;
}

void vgt_i2c_handle_gpio_write(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t **pedid, void *p_data)
{
	I2C_COMMANDS cmd;

	cmd = vgt_recognize_mmio_command(GPIO_MMIO_WRITE,
					&bit_bang, *(int *)p_data);
	ASSERT(cmd == I2C_COMMAND_IGNORE);

	return;
}
