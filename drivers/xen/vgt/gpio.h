/*
 * vGT header file for GPIO virtualization
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

#ifndef _VGT_MMIO_GPIO_H_
#define _VGT_MMIO_GPIO_H_

#define _GPIO_CLOCK_DIR_MASK		(1 << 0)
#define _GPIO_CLOCK_DIR_VAL		(1 << 1)
#define _GPIO_CLOCK_DIR_IN		(0 << 1)
#define _GPIO_CLOCK_DIR_OUT		(1 << 1)
#define _GPIO_CLOCK_VAL_MASK		(1 << 2)
#define _GPIO_CLOCK_VAL_OUT		(1 << 3)
#define _GPIO_CLOCK_VAL_IN		(1 << 4)
#define _GPIO_CLOCK_PULLUP_DISABLE	(1 << 5)
#define _GPIO_DATA_DIR_MASK		(1 << 8)
#define _GPIO_DATA_DIR_VAL		(1 << 9)
#define _GPIO_DATA_DIR_IN		(0 << 9)
#define _GPIO_DATA_DIR_OUT		(1 << 9)
#define _GPIO_DATA_VAL_MASK		(1 << 10)
#define _GPIO_DATA_VAL_OUT		(1 << 11)
#define _GPIO_DATA_VAL_IN		(1 << 12)
#define _GPIO_DATA_PULLUP_DISABLE	(1 << 13)

typedef enum {
	VGT_I2C_LOW = 0,
	VGT_I2C_HIGH,
}I2C_LINE_STATE;

/* I2C_LINE info is set by I2C write command, and used by
 * I2C read commands.
 */
typedef enum {
	VGT_I2C_SDA,
	VGT_I2C_SCL
} I2C_LINE;

/* types of commands. GPIO_MMIO_READ/WRITE corresponds to the real
 * MMIO instructions. I2C_BIT_READ/WRITE corresponds to the logic
 * bit operations. In i915, it represents a sequence of MMIO commands.
 */
typedef enum {
	I2C_COMMAND_IGNORE = 0,
	GPIO_MMIO_READ,
	GPIO_MMIO_WRITE,
	I2C_BIT_READ,
	I2C_BIT_WRITE,
} I2C_COMMANDS;

#define VGT_IS_I2C_SDA(val) ((val) & _GPIO_DATA_DIR_MASK)

#define VGT_IS_I2C_SCL(val) ((val) & _GPIO_CLOCK_DIR_MASK)

/* Default value of SCL or SDA line is 1.
 * Return default value if no out value is specified.
 */
#define VGT_GET_I2C_OUT_SCL(val) 				\
(((((val) & _GPIO_CLOCK_DIR_VAL) == _GPIO_CLOCK_DIR_OUT) && 	\
     ((val) & _GPIO_CLOCK_VAL_MASK)) ? 				\
	(((val) & _GPIO_CLOCK_VAL_OUT) != 0) : 1)

#define VGT_GET_I2C_OUT_SDA(val) 				\
(((((val) & _GPIO_DATA_DIR_VAL) == _GPIO_DATA_DIR_OUT) && 	\
     ((val) & _GPIO_DATA_VAL_MASK)) ? 				\
	(((val) & _GPIO_DATA_VAL_OUT) != 0) : 1)

#define VGT_HAS_SDA_IN_VALUE(val)				\
((((val) & _GPIO_DATA_DIR_VAL) == _GPIO_DATA_DIR_IN) &&		\
 ((val) & _GPIO_DATA_VAL_MASK))

#define IS_ENABLE_SDA_VAL_IN(val) 				\
(((val) & _GPIO_DATA_DIR_MASK) &&				\
 (((val) & _GPIO_DATA_DIR_VAL) == _GPIO_DATA_DIR_IN))

#define IS_I2C_START(scl, sda_old, sda_new) 			\
(((scl) == VGT_I2C_HIGH) && 					\
 ((sda_old) == VGT_I2C_HIGH) &&					\
 ((sda_new) == VGT_I2C_LOW))

#define IS_I2C_STOP(scl, sda_old, sda_new) 			\
(((scl) == VGT_I2C_HIGH) && 					\
 ((sda_old) == VGT_I2C_LOW) &&					\
 ((sda_new) == VGT_I2C_HIGH))

/* Introduction of vgt_i2c_bitbang_t
 *
 * The data structure of vgt_i2c_bitbang_t has the needed information for
 * tracking the state changes of I2C bit-banging data transmission algorithm.
 * its fields will be desicribed below and the reason for having them will
 * be talked as well.
 *
 * current_line:	The variable shows which line is currently I2C
 * 			READ/WRITE is happening. The reason to have this is
 * 			otherwise I2C READ cannot know the data source. The
 * 			variable is set in the I2C WRITE command according
 * 			to the DIR mask.
 * sda_state:		Indicate the level(LOW/HIGH) of SDA line. The value
 * 			of current level is used to know the level change. That
 * 			kind of information is used to recognize the I2C_START,
 * 			I2C_STOP etc.
 * scl_state:		Indicate the level(LOW/HIGH) of SCL line. The emulation
 * 			needs to knwo the clock info for two purposes: one is to
 * 			recognize some state changes together with sda_state;
 * 			another is to ignore some useless commands which happens
 * 			in SCL low level. For instance, the read in SCL LOW.
 * 			(is it really needed?)
 * write_enabled:
 * read_enabled:	The two boolean variables are set during the I2C WRITE
 * 			command and will be consumed by I2C WRITE or READ
 * 			command. The reason to have this fields is related to
 * 			the i915 I2C command implementation. The I2C read-bit
 * 			in i915 is implemented as follow:
 *
 * 				reserved = GPIO_MMIO_READ
 * 				GPIO_MMIO_WRITE (reserved|DIR_IN|DIR_MASK)
 * 				GPIO_MMIO_WRITE (reserved)
 * 				value = GPIO_MMIO_READ
 *
 *			and the write-bit is implemented as follow:
 *
 *			write 1:
 * 				GPIO_MMIO_WRITE (reserved|DIR_IN|DIR_MASK)
 * 				value = GPIO_MMIO_READ
 * 			write 0:
 * 				GPIO_MMIO_WRITE (reserved|DIR_OUT|DIR_MASK|
 * 						 OUT_VALUE|VALUE_MASK)
 * 				value = GPIO_MMIO_READ
 *
 *			And the headache is that the the "write 1" above is
 *			quite similar to read. The solution here is to have
 *			the GPIO_MMIO_READ to be the check point of bit read
 *			or write. The DIR_MASK write command will enable both
 *			"read" and "write". The WRITE with "DIR_MASK" cleared
 *			will consume the "read". The READ will consume both
 *			"read" and "write". If there is "write" enabled, it is
 *			considered as write. We could assume that "read" is
 *			always enabled.
 * sda_ack_state:	After 8-bit data operation(read/write), the bit-banging
 * 			will enter this state; It is needed to know that some
 * 			read/write needs to be ignored. For instance, after
 * 			sending 8-bit value, there could still be SDA write to
 * 			set SDA line high level. This should actually be
 * 			ignored.
 * pending_write_bit:	record the write value in the WRITE command if "DIR"
 * 			mask is enabled. The value needs to be pending because
 *			it is still not yet known whether it is really for
 *			writing bit or reading bit.
 * buffer:		The 8-bit buffer for bit-banging operations.
 * bit_length:		Length of the buffer that currently has been consumed.
 * 			The value will be adjusted in clock down side.
 */
typedef struct {
	I2C_LINE current_line;
	I2C_LINE_STATE sda_state;
	I2C_LINE_STATE scl_state;
	bool write_enabled;
	bool read_enabled;
	bool sda_ack_state;
	I2C_LINE_STATE pending_write_bit;
	unsigned char buffer;
	int bit_length;
} vgt_i2c_bitbang_t;

#endif /*_VGT_MMIO_GPIO_H_*/
