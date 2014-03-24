/*
 * vGT header file for EDID virtualization
 *
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
#ifndef _VGT_EDID_H_
#define _VGT_EDID_H_

#define EDID_SIZE		128
#define EDID_ADDR		0x50 /* Linux hvm EDID addr (TODO: how to get this addr ? )*/

#define VGT_AUX_NATIVE_WRITE			0x8
#define VGT_AUX_NATIVE_READ			0x9
#define VGT_AUX_I2C_WRITE			0x0
#define VGT_AUX_I2C_READ			0x1
#define VGT_AUX_I2C_STATUS			0x2
#define VGT_AUX_I2C_MOT				0x4
#define VGT_AUX_I2C_REPLY_ACK			(0x0 << 6)

#define _REGBIT_DP_AUX_CH_CTL_SEND_BUSY (1 << 31)
#define _REGBIT_DP_AUX_CH_CTL_DONE (1 << 30)
#define _DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT 	20
#define _DP_AUX_CH_CTL_MESSAGE_SIZE_MASK 	(0x1f << 20)

typedef struct {
	bool data_valid;
	unsigned char edid_block[EDID_SIZE];
}vgt_edid_data_t;

typedef enum {
	GMBUS_NOCYCLE	= 0x0,
	NIDX_NS_W	= 0x1,
	IDX_NS_W	= 0x3,
	GMBUS_STOP	= 0x4,
	NIDX_STOP	= 0x5,
	IDX_STOP	= 0x7
}gmbus_cycle_type_t;

/*
 * States of GMBUS
 *
 * GMBUS0-3 could be related to the EDID virtualization. Another two GMBUS
 * registers, GMBUS4 (interrupt mask) and GMBUS5 (2 byte indes register), are
 * not considered here. Below describes the usage of GMBUS registers that are
 * cared by the EDID virtualization
 *
 * GMBUS0:
 * 	R/W
 * 	port selection. value of bit0 - bit2 corresponds to the GPIO registers.
 *
 * GMBUS1:
 * 	R/W Protect
 * 	Command and Status.
 * 	bit0 is the direction bit: 1 is read; 0 is write.
 * 	bit1 - bit7 is slave 7-bit address.
 * 	bit16 - bit24 total byte count (ignore?)
 *
 * GMBUS2:
 * 	Most of bits are read only except bit 15 (IN_USE)
 * 	Status register
 * 	bit0 - bit8 current byte count
 * 	bit 11: hardware ready;
 *
 * GMBUS3:
 *	Read/Write
 *	Data for transfer
 */

/* From hw specs, Other phases like START, ADDRESS, INDEX
 * are invisible to GMBUS MMIO interface. So no definitions
 * in below enum types
 */
typedef enum {
	GMBUS_IDLE_PHASE = 0,
	GMBUS_DATA_PHASE,
	GMBUS_WAIT_PHASE,
	//GMBUS_STOP_PHASE,
	GMBUS_MAX_PHASE
} vgt_gmbus_phase_t;

typedef struct {
	unsigned port;
	unsigned total_byte_count; /* from GMBUS1 */
	gmbus_cycle_type_t cycle_type;
	vgt_gmbus_phase_t phase;
	/* TODO
	 * would WRITE(GMBUS0, 0) clear "inuse" bit?
	 * current implementation is to clear it.
	 */
	//bool inuse;
	vgt_edid_data_t *pedid;
} vgt_i2c_gmbus_t;

/*
 * States of AUX_CH
 */

/* TODO: move below definition to common header file, and let vgt_edid.c
 * include that file. Right now vgt_reg.h has too many contents. We do
 * not want to include that until it is divided into simpler ones.
 */
typedef unsigned int vgt_register_t;

#define VGT_DP_NUM 4
typedef enum {
	VGT_DP_NA = -1,
	VGT_DPA_IDX = 0,
	VGT_DPB_IDX,
	VGT_DPC_IDX,
	VGT_DPD_IDX
}VGT_DP_PORTS_IDX;

#define AUX_REGISTER_NUM 6
typedef enum {
	AUX_CH_INV = -1,
	AUX_CH_CTL = 0,
	AUX_CH_DATA1,
	AUX_CH_DATA2,
	AUX_CH_DATA3,
	AUX_CH_DATA4,
	AUX_CH_DATA5
}AUX_CH_REGISTERS;

typedef unsigned int aux_reg_t;

static inline VGT_DP_PORTS_IDX vgt_get_dp_port_idx(unsigned int offset)
{
	VGT_DP_PORTS_IDX port_idx;

	if (offset >= _REG_DPA_AUX_CH_CTL
		&& offset <= _REG_DPA_AUX_CH_CTL + AUX_REGISTER_NUM * sizeof(vgt_reg_t)) {
		return VGT_DPA_IDX;
	}

	switch (((offset & 0xff00) >> 8) - 0x41) {
	case 0:
		port_idx = VGT_DPB_IDX;
		break;
	case 1:
		port_idx = VGT_DPC_IDX;
		break;
	case 2:
		port_idx = VGT_DPD_IDX;
		break;
	default:
		port_idx = VGT_DP_NA;
		break;
	}
	return port_idx;
}

/* AUX_CH has multiple sets of registers including ctl, data etc,
 * each of which is for a DP port, like DP_B, DP_C or DP_D.
 *
 * vgt_device has vreg and sreg definition for those AUX_CH registers. Here
 * we simply reuse the vreg registers for full virtualization.
 *
 * Remember that only MMIO save/restore cannot handle display switch
 * completely. Consider internal state like the MOT of AUX_CH, the current
 * reading/writing offset, they are also important. In the current
 * implementation, each vgt instance has its own aux_ch hardware state. The
 * information is tracked there. The save/restore in display switch does not
 * handle them.
 *
 * The only problem is when there is write that needs to be flushed to
 * hardware. There is no example yet. If there is an example and if we have
 * to pass-through the write, we need to consider the hardware resource
 * conflicts between vms. Below is one possible solution:
 *
 * Let the current "middle of transaction" finish first. Introduce a
 * semorphore for the resources shared among VMs. Other VM cannot use
 * the resource even if it is the display owner. Timer should be introduced
 * to release the semorphore if one VM holds it for too long time.
 *
 */

/*
 * TODO:
 *
 * DP spec said that the AUX_CH cannot be accessed simultaneously. However, it
 * is not quite clear whether a VM could access different AUX_CH simultaneously.
 * For the simplicity, we assume that different port will not be in MOT at the
 * same time. It's hard to report error when it happens. Cannot distinguish
 * with restart ...
 */

typedef struct {
	bool i2c_over_aux_ch;
	bool aux_ch_mot;
	aux_reg_t *aux_registers[VGT_DP_NUM][AUX_REGISTER_NUM];
}vgt_i2c_aux_ch_t;

/*
 * Interface of I2C slave
 */
typedef struct VGT_I2C_SLAVE_T{
	void (*start) (void);
	void (*select) (void);

	/*
	 * It is interface used during snapshot when a byte transmission
	 * has finished from slave to master. People could define its
	 * logic here to capture the byte value.
	 */
	void (*snap_read_byte) (void *slave, unsigned char value);

	/*
	 * It is interface used when a byte transmission has finished
	 * from master to slave. People could define its logic here
	 * to modify the snopshot for the device. In the case that the
	 * snapshot is not built, the function usually does nothing.
	 */
	void (*snap_write_byte) (void *slave, unsigned char value);

	/* It is interface to get a snapshot byte. The snapshot must
	 * be available when calling this function.
	 */
	unsigned char (*get_byte_from_snap) (void *slave);

	void (*snap_stop) (void *dest,
			struct VGT_I2C_SLAVE_T *src);
} vgt_i2c_slave_t;

/*
 * State of EDID
 */
typedef enum {
	VGT_I2C_STOP = 0,
	VGT_I2C_START,
	VGT_I2C_SEND,
	VGT_I2C_RECEIVE,
	VGT_I2C_WAIT
}I2C_STATE;

/*
 * i2c
 */

typedef struct {
	vgt_i2c_slave_t slave;
	int current_read;
	int current_write;
	vgt_edid_data_t *edid_data;
}vgt_edid_t;

/* Only sequential I2C devices access is supported. That is, when assigning
 * new slave to current_slave, the current_slave must be NULL, or with the
 * same slave_addr.
 */
typedef struct {
	/* it is 7-bit address in the first byte of bit stream */
	int current_slave_addr;
	vgt_i2c_slave_t *current_slave;
	I2C_STATE state;
	vgt_edid_t edid_slave;

	/* different implementations below */
	vgt_i2c_gmbus_t gmbus;
	vgt_i2c_aux_ch_t aux_ch;
}vgt_i2c_bus_t;

vgt_edid_data_t *vgt_create_edid(void);

void vgt_init_i2c_bus(vgt_i2c_bus_t *i2c_bus);

bool vgt_i2c_handle_gmbus_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);

bool vgt_i2c_handle_gmbus_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);

void vgt_i2c_handle_aux_ch_read(vgt_i2c_bus_t *i2c_bus,
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data);

void vgt_i2c_handle_aux_ch_write(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t *edid,
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data);

bool vgt_is_edid_valid(u8 *raw_edid);

#endif /*_VGT_EDID_H_*/
