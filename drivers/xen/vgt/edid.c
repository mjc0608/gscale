/*
 * vGT EDID virtualization module
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

#include <xen/vgt.h>
#include <xen/vgt-if.h>
#include "reg.h"
#include "vgt.h"
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

	edid = kmalloc(sizeof(vgt_edid_data_t), GFP_ATOMIC);
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

/* GMBUS0 */
static bool vgt_gmbus0_mmio_write(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes)
{
	vgt_edid_data_t *edid_data = NULL;
	vgt_reg_t wvalue = *(vgt_reg_t *)p_data;
	switch (wvalue & _GMBUS_PIN_SEL_MASK) {
	case 0: /* disabled. Be treated as reset */
		edid_data = NULL;
		break;
	case 1: /* LCTRCLK */
		printk("vGT(%d): WARNING: Accessing LCTRCLK which is not supported!\n",
			vgt->vgt_id);
		break;
	case 2: /* Analog Mon */
		edid_data = vgt->vgt_edids[EDID_VGA];
		break;
	case 3: /* LVDS */
		edid_data = vgt->vgt_edids[EDID_LVDS];
		break;
	case 4: /* Port C use */
		/* TODO: how about DP ??? */
		edid_data = vgt->vgt_edids[EDID_HDMIC];
		break;
	case 5: /* Port B use */
		/* TODO: how about DP ??? */
		edid_data = vgt->vgt_edids[EDID_HDMIB];
		break;
	case 6: /* Port D use */
		/* TODO: how about DP ??? */
		edid_data = vgt->vgt_edids[EDID_HDMID];
		break;
	case 7:
		printk("vGT(%d): WARNING: GMBUS accessing reserved port!!!!\n", vgt->vgt_id);
		break;
	default:
		printk("vGT(%d): EDID unknown ERROR!\n", vgt->vgt_id);
	}

	vgt_init_i2c_bus(&vgt->vgt_i2c_bus);
	//vgt->vgt_i2c_bus.state = VGT_I2C_SEND;
	vgt->vgt_i2c_bus.gmbus.pedid = edid_data;
	vgt->vgt_i2c_bus.gmbus.phase = GMBUS_IDLE_PHASE;

	/* Initialize status reg
	 * FIXME: never clear _GMBUS_HW_WAIT */
	__vreg(vgt, _REG_PCH_GMBUS2) &= ~ _GMBUS_ACTIVE;
	__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_HW_RDY | _GMBUS_HW_WAIT;
	if (!edid_data)
		__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_NAK;
	else
		__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_NAK;

	memcpy(p_data, (char *)vgt->state.vReg + offset, bytes);
	return true;
}

/* TODO: */
void vgt_reset_gmbus_controller(struct vgt_device *vgt)
{
	/* TODO: clear gmbus0 ? */
	//__vreg(vgt, _REG_PCH_GMBUS0) = 0;
	//__vreg(vgt, _REG_PCH_GMBUS1) = 0;
	__vreg(vgt, _REG_PCH_GMBUS2) = _GMBUS_HW_RDY;
	//__vreg(vgt, _REG_PCH_GMBUS3) = 0;
	//__vreg(vgt, _REG_PCH_GMBUS4) = 0;
	//__vreg(vgt, _REG_PCH_GMBUS5) = 0;
	vgt->vgt_i2c_bus.gmbus.phase = GMBUS_IDLE_PHASE;
}


static bool vgt_gmbus1_mmio_write(struct vgt_device *vgt, unsigned int offset,
void *p_data, unsigned int bytes)
{
	u32 slave_addr;
	vgt_i2c_bus_t *i2c_bus = &vgt->vgt_i2c_bus;

	vgt_reg_t wvalue = *(vgt_reg_t *)p_data;
	if (__vreg(vgt, offset) & _GMBUS_SW_CLR_INT) {
		if (!(wvalue & _GMBUS_SW_CLR_INT)) {
			__vreg(vgt, offset) &= ~_GMBUS_SW_CLR_INT;
			vgt_reset_gmbus_controller(vgt);
		}
		/* TODO: "This bit is cleared to zero when an event
		 * causes the HW_RDY bit transition to occur "*/
	} else {
		/* per bspec setting this bit can cause:
		 1) INT status bit cleared
		 2) HW_RDY bit asserted
		 */
		if (wvalue & _GMBUS_SW_CLR_INT) {
			__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_INT_STAT;
			__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_HW_RDY;
		}

		/* For virtualization, we suppose that HW is always ready,
		 * so _GMBUS_SW_RDY should always be cleared
		 */
		if (wvalue & _GMBUS_SW_RDY)
			wvalue &= ~_GMBUS_SW_RDY;

		i2c_bus->gmbus.total_byte_count =
			gmbus1_total_byte_count(wvalue);
		slave_addr = gmbus1_slave_addr(wvalue);
		i2c_bus->current_slave_addr = slave_addr;

		/* vgt gmbus only support EDID */
		if (slave_addr == EDID_ADDR) {
			i2c_bus->current_slave = (vgt_i2c_slave_t *)&i2c_bus->edid_slave;
			i2c_bus->edid_slave.edid_data = i2c_bus->gmbus.pedid;

		} else if (slave_addr != 0) {
			vgt_err("vGT(%d): unsupported gmbus slave addr(%x)\n",
					vgt->vgt_id, slave_addr);
			i2c_bus->current_slave = (vgt_i2c_slave_t *)&i2c_bus->edid_slave;
			i2c_bus->edid_slave.edid_data = i2c_bus->gmbus.pedid;
		}

		if (wvalue & _GMBUS_CYCLE_INDEX) {
			i2c_bus->current_slave->snap_write_byte(
					i2c_bus->current_slave,
					gmbus1_slave_index(wvalue));
		}

		i2c_bus->gmbus.cycle_type = gmbus1_bus_cycle(wvalue);
		switch (gmbus1_bus_cycle(wvalue)) {
			case GMBUS_NOCYCLE:
				break;
			case GMBUS_STOP:
				/* From spec:
				This can only cause a STOP to be generated
				if a GMBUS cycle is generated, the GMBUS is
				currently in a data/wait/idle phase, or it is in a
				WAIT phase
				 */
				if (gmbus1_bus_cycle(__vreg(vgt, offset)) != GMBUS_NOCYCLE) {
					vgt_init_i2c_bus(i2c_bus);
					/* After the 'stop' cycle, hw state would become
					 * 'stop phase' and then 'idle phase' after a few
					 * milliseconds. In emulation, we just set it as
					 * 'idle phase' ('stop phase' is not
					 * visible in gmbus interface)
					 */
					i2c_bus->gmbus.phase = GMBUS_IDLE_PHASE;
					/*
					FIXME: never clear _GMBUS_WAIT
					__vreg(vgt, _REG_PCH_GMBUS2) &=
						~(_GMBUS_ACTIVE | _GMBUS_HW_WAIT);
					*/
					__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_ACTIVE;
				}
				break;
			case NIDX_NS_W:
			case IDX_NS_W:
			case NIDX_STOP:
			case IDX_STOP:
				/* From hw spec the GMBUS phase
				 * transition like this:
				 * START (-->INDEX) -->DATA
				 */
				i2c_bus->gmbus.phase = GMBUS_DATA_PHASE;
				__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_ACTIVE;
				/* FIXME: never clear _GMBUS_WAIT */
				//__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_HW_WAIT;
				break;
			default:
				vgt_err("Unknown/reserved GMBUS cycle detected!");
				break;
		}
		/* From hw spec the WAIT state will be
		 * cleared:
		 * (1) in a new GMBUS cycle
		 * (2) by generating a stop
		 */
		/* FIXME: never clear _GMBUS_WAIT
		if (gmbus1_bus_cycle(wvalue) != GMBUS_NOCYCLE)
			__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_HW_WAIT;
		*/

		__vreg(vgt, offset) = wvalue;
	}
	return true;
}

bool vgt_gmbus3_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	BUG();
	return true;
}

bool vgt_gmbus3_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_i2c_bus_t *i2c_bus = &vgt->vgt_i2c_bus;
	int byte_left = i2c_bus->gmbus.total_byte_count
		- i2c_bus->edid_slave.current_read;
	int i, byte_count = byte_left;
	vgt_reg_t reg_data = 0;
	unsigned char byte_data;

	/* Data can only be recevied if previous settings correct */
	if (__vreg(vgt, _REG_PCH_GMBUS1) & _GMBUS_SLAVE_READ) {
		if (byte_left <= 0) {
			memcpy((char *)p_data, (char *)vgt->state.vReg + offset, bytes);
			return true;
		}

		if (byte_count > 4)
			byte_count = 4;
		for (i = 0; i< byte_count; i++) {
			byte_data = i2c_bus->current_slave->get_byte_from_snap(
					i2c_bus->current_slave);
			reg_data |= (byte_data << (i << 3));
		}

		memcpy((char *)p_data, (char *)&reg_data, byte_count);
		memcpy((char *)vgt->state.vReg + offset, (char *)&reg_data, byte_count);

		if (byte_left <= 4) {
			switch (i2c_bus->gmbus.cycle_type) {
				case NIDX_STOP:
				case IDX_STOP:
					i2c_bus->gmbus.phase = GMBUS_IDLE_PHASE;
					break;
				case NIDX_NS_W:
				case IDX_NS_W:
				default:
					i2c_bus->gmbus.phase = GMBUS_WAIT_PHASE;
					break;
			}
			//if (i2c_bus->gmbus.phase == GMBUS_WAIT_PHASE)
			//__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_HW_WAIT;

			vgt_init_i2c_bus(i2c_bus);
		}

		/* Read GMBUS3 during send operation, return the latest written value */
	} else {
		memcpy((char *)p_data, (char *)vgt->state.vReg + offset, bytes);
		printk("vGT(%d): warning: gmbus3 read with nothing retuned\n",
				vgt->vgt_id);
	}

	return true;
}

static bool vgt_gmbus2_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_reg_t value = __vreg(vgt, offset);
	if (!(__vreg(vgt, offset) & _GMBUS_IN_USE)) {
		__vreg(vgt, offset) |= _GMBUS_IN_USE;
	}

	memcpy(p_data, (void *)&value, bytes);
	return true;
}

bool vgt_gmbus2_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_reg_t wvalue = *(vgt_reg_t *)p_data;
	if (wvalue & _GMBUS_IN_USE)
		__vreg(vgt, offset) &= ~_GMBUS_IN_USE;
	/* All other bits are read-only */
	return true;
}

bool vgt_i2c_handle_gmbus_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes <= 8 && !(offset & (bytes - 1)));
	switch (offset) {
		case _REG_PCH_GMBUS2:
			return vgt_gmbus2_mmio_read(vgt, offset, p_data, bytes);
		case _REG_PCH_GMBUS3:
			return vgt_gmbus3_mmio_read(vgt, offset, p_data, bytes);
		default:
			memcpy(p_data, (char *)vgt->state.vReg + offset, bytes);
	}
	return true;
}

bool vgt_i2c_handle_gmbus_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes <= 8 && !(offset & (bytes - 1)));
	switch (offset) {
		case _REG_PCH_GMBUS0:
			return vgt_gmbus0_mmio_write(vgt, offset, p_data, bytes);
		case _REG_PCH_GMBUS1:
			return vgt_gmbus1_mmio_write(vgt, offset, p_data, bytes);
		case _REG_PCH_GMBUS2:
			return vgt_gmbus2_mmio_write(vgt, offset, p_data, bytes);
		/* TODO: */
		case _REG_PCH_GMBUS3:
			BUG();
			return false;
		default:
			memcpy((char *)vgt->state.vReg + offset, p_data, bytes);
	}
	return true;
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
	 * ACK of I2C_WRITE
	 * returned byte if it is READ
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

	memset(&i2c_bus->gmbus, 0, sizeof(vgt_i2c_gmbus_t));

	i2c_bus->aux_ch.i2c_over_aux_ch = false;
	i2c_bus->aux_ch.aux_ch_mot = false;
	/*two arrays of aux_shadow_reg and aux_registers are initialized once
	 * in vgt_init_aux_ch_vregs(), which is not supposed to change again
	 */
}
