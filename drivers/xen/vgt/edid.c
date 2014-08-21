/*
 * vGT EDID virtualization module
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

#include <linux/delay.h>
#include <linux/slab.h>
#include <drm/drmP.h>

#include "vgt.h"

#define DEBUG_VGT_EDID

typedef enum {
	VGT_EDID_INFO = 1,
	VGT_EDID_WARN = 2,
	VGT_EDID_ERROR = 3,
} vgt_edid_log_t;

static const char *vgt_port_name[] = {
	"CRT",
	"DP_A",
	"DP_B",
	"DP_C",
	"DP_D",
	"HDMI_B",
	"HDMI_C",
	"HDMI_D"
};

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

#define IS_SHARED_PORT(port) \
	(((port) >= VGT_DP_B && (port) <= VGT_DP_D) ||	\
			((port) >= VGT_HDMI_B && (port) <= VGT_HDMI_D))

static inline int SHARED_PORT_IDX(int port)
{
	ASSERT(IS_SHARED_PORT(port));
	if (port >= VGT_DP_B && port <= VGT_DP_D) {
		return port - VGT_DP_B + 1;
	} else {
		ASSERT (port >= VGT_HDMI_B && port <= VGT_HDMI_D);
		return port - VGT_HDMI_B + 1;
	}
}

static inline bool vgt_port_equivalent(int id, int given_index)
{
	if (given_index == -1) {
		/* -1 is an alias id which is equivalent with all the ports */
		return true;
	} else if (id == given_index) {
		return true;
	} else {
		if (IS_SHARED_PORT(given_index) && IS_SHARED_PORT(id)) {
			return (SHARED_PORT_IDX(id) ==
					SHARED_PORT_IDX(given_index));
		}
	}
	return false;
}

static const u8 edid_header[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
};

int vgt_edid_header_is_valid(const u8 *raw_edid)
{
	int i, score = 0;
	for (i = 0; i < sizeof(edid_header); i++)
		if (raw_edid[i] == edid_header[i])
			score++;
	return score;
}

bool vgt_is_edid_valid(u8 *raw_edid)
{
	bool is_valid = false;
	int score, i;
	u8 check_sum = 0;

	score = vgt_edid_header_is_valid(raw_edid);

	check_sum = 0;
	for (i = 0; i < EDID_SIZE; ++i) {
		check_sum += raw_edid[i];
	}
	if (check_sum) {
		vgt_err("EDID check sum is invalid\n");
	}

	if ((score == 8) && (check_sum == 0)) {
		is_valid = true;
	}
	return is_valid;
}

static inline void vgt_clear_edid(struct gt_port *port)
{
	if (port && port->edid && port->edid->data_valid) {
		port->edid->data_valid = false;
	}
}

static inline void vgt_clear_dpcd(struct gt_port *port)
{
	if (port && port->dpcd && port->dpcd->data_valid) {
		port->dpcd->data_valid = false;
	}
}

/**************************************************************************
 *
 * EDID propagation to vgt instance
 *
 *************************************************************************/
/* vgt_propagate_edid
 *
 * Propagate the EDID information stored in pdev to vgt device.
 * Right now the vgt uses the same EDID. In future, there could be
 * policy to change the EDID that is used by vgt instances.
 */
void vgt_propagate_edid(struct vgt_device *vgt, int index)
{
	struct gt_port *port;
	vgt_edid_data_t *edid;
	enum vgt_port_type type;
	int i;

	if (index == -1) { /* -1 index means "ALL" */
		for (i = 0; i < I915_MAX_PORTS; i++) {
			vgt_propagate_edid(vgt, i);
		}
		return;
	}

	if (index < 0 || index >= I915_MAX_PORTS) {
		vgt_warn("Wrong port index input! Will do nothing!\n");
		return;
	}

	port = &vgt->ports[index];
	if (port->type >= 0 && port->type < VGT_PORT_MAX)
		clear_bit(vgt->ports[index].type, vgt->presented_ports);
	vgt_clear_edid(port);

	edid = vgt->pdev->ports[index].edid;
	type = vgt->pdev->ports[index].type;

	if (edid && edid->data_valid) {

		vgt_info ("EDID_PROPAGATE: Propagate %s EDID[%s] for vm %d\n",
			VGT_PORT_NAME(index), vgt_port_name[type], vgt->vm_id);

		if (!port->edid) {
			BUG_ON(in_interrupt());
			port->edid = kmalloc(sizeof(vgt_edid_data_t), GFP_ATOMIC);
			if (port->edid == NULL) {
				vgt_err("Insufficient memory!\n");
				BUG();
				return;
			}
		}

		memcpy(port->edid, edid, sizeof(vgt_edid_data_t));
		port->type = type;
		port->port_override = index;
		set_bit(type, vgt->presented_ports);

		if (vgt_debug & VGT_DBG_DPY) {
			vgt_info("EDID_PROPAGATE: %s EDID[%s] is:\n",
				VGT_PORT_NAME(index),
				vgt_port_name[type]);
			vgt_print_edid(edid);
		}
	} else {
		vgt_info ("EDID_PROPAGATE: Clear %s for vm %d\n",
					VGT_PORT_NAME(index), vgt->vm_id);
	}
}

void vgt_propagate_dpcd(struct vgt_device *vgt, int index)
{
	struct vgt_dpcd_data *dpcd = NULL;
	int i;

	if (index == -1) { /* -1 index means "ALL" */
		for (i = 0; i < I915_MAX_PORTS; i++) {
			vgt_propagate_dpcd(vgt, i);
		}
		return;
	}

	if (index < 0 || index >= I915_MAX_PORTS) {
		vgt_warn("Wrong port index input! Will do nothing!\n");
		return;
	}

	/* Assumption: Port is DP if it has DPCD */
	if (vgt->pdev->ports[index].dpcd && vgt->pdev->ports[index].dpcd->data_valid) {
		dpcd = vgt->pdev->ports[index].dpcd;
	}

	vgt_clear_dpcd(&vgt->ports[index]);

	if (dpcd) {
		vgt_info("DPCD_PROPAGATE: Propagate DPCD %d\n", index);
		if (!vgt->ports[index].dpcd) {
			vgt->ports[index].dpcd = kmalloc(
				sizeof(struct vgt_dpcd_data), GFP_ATOMIC);
			if (vgt->ports[index].dpcd == NULL) {
				vgt_err("Insufficient memory!\n");
				BUG();
				return;
			}
		}

		memcpy(vgt->ports[index].dpcd, dpcd,
			sizeof(struct vgt_dpcd_data));

		/* mask off CP */
		vgt->ports[index].dpcd->data[DPCD_SINK_COUNT] &=
			~DPCD_CP_READY_MASK;

		if ((vgt_debug & VGT_DBG_DPY) && vgt->ports[index].dpcd->data_valid) {
			vgt_info("DPCD_PROPAGATE: DPCD[%d] is:\n", index);
			vgt_print_dpcd(vgt->ports[index].dpcd);
		}
	}
}

void vgt_clear_port(struct vgt_device *vgt, int index)
{
	int i;
	struct gt_port *port;

	if (index == -1) { /* -1 index means "ALL" */
		for (i = 0; i < I915_MAX_PORTS; i++) {
			vgt_clear_port(vgt, i);
		}
		return;
	}

	if (index < 0 || index >= I915_MAX_PORTS) {
		vgt_warn("Wrong port index input! Will do nothing!\n");
		return;
	}

	port = &vgt->ports[index];
	vgt_clear_edid(port);
	vgt_clear_dpcd(port);
	
	if (port->type >= 0 && port->type < VGT_PORT_MAX) {
		clear_bit(vgt->ports[index].type,
			  vgt->presented_ports);
	}

	port->type = VGT_PORT_MAX;
}

/**************************************************************************
 *
 * EDID Slave implementation
 *
 *************************************************************************/

static unsigned char edid_get_byte(void *slave)
{
	vgt_edid_t *edid = (vgt_edid_t *)slave;
	if (edid->current_read >= EDID_SIZE) {
		EDID_MSG(VGT_EDID_ERROR, true,
			"edid_get_byte() exceeds the size of EDID!\n");
	}
	EDID_MSG(VGT_EDID_INFO, true,
			"edid_get_byte with offset %d and value %d\n",
			edid->current_read,
			edid->edid_data->edid_block[edid->current_read]);
	if (edid->edid_data && edid->edid_data->data_valid) {
		return edid->edid_data->edid_block[edid->current_read ++];
	} else {
		return 0;
	}
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
	struct gt_port *port = NULL;

	switch (wvalue & _GMBUS_PIN_SEL_MASK) {
	case 0: /* disabled. Be treated as reset */
		edid_data = NULL;
		break;
	case 1: /* LCTRCLK */
		printk("vGT(%d): WARNING: Accessing LCTRCLK which is not supported!\n",
			vgt->vgt_id);
		break;
	case 2: /* Analog Mon */
		port = &vgt->ports[port_type_to_port(VGT_CRT)];
		break;
	case 4: /* Port C use */
		port = &vgt->ports[port_type_to_port(VGT_HDMI_C)];
		break;
	case 5: /* Port B use */
		port = &vgt->ports[port_type_to_port(VGT_HDMI_B)];
		break;
	case 6: /* Port D use */
		port = &vgt->ports[port_type_to_port(VGT_HDMI_D)];
		break;
	case 7:
		printk("vGT(%d): WARNING: GMBUS accessing reserved port!!!!\n", vgt->vgt_id);
		break;
	default:
		printk("vGT(%d): EDID unknown ERROR!\n", vgt->vgt_id);
	}

	if (port) 
		edid_data = port->edid;

	vgt_init_i2c_bus(&vgt->vgt_i2c_bus);
	//vgt->vgt_i2c_bus.state = VGT_I2C_SEND;
	vgt->vgt_i2c_bus.gmbus.phase = GMBUS_IDLE_PHASE;

	/* Initialize status reg
	 * FIXME: never clear _GMBUS_HW_WAIT */
	__vreg(vgt, _REG_PCH_GMBUS2) &= ~ _GMBUS_ACTIVE;
	__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_HW_RDY | _GMBUS_HW_WAIT;
	if (edid_data && edid_data->data_valid && !(port->dpcd && port->dpcd->data_valid)) {
		__vreg(vgt, _REG_PCH_GMBUS2) &= ~_GMBUS_NAK;
		vgt->vgt_i2c_bus.gmbus.pedid = edid_data;
	} else
		__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_NAK;

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
	if (!vgt->vgt_i2c_bus.gmbus.pedid) {
		__vreg(vgt, _REG_PCH_GMBUS2) |= _GMBUS_NAK;
	}
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
			vgt_dbg(VGT_DBG_DPY, "vGT(%d): unsupported gmbus slave addr(%x)\n",
					vgt->vgt_id, slave_addr);
			i2c_bus->current_slave = (vgt_i2c_slave_t *)&i2c_bus->edid_slave;
			i2c_bus->edid_slave.edid_data = i2c_bus->gmbus.pedid;
		}

		if (wvalue & _GMBUS_CYCLE_INDEX) {
			i2c_bus->edid_slave.current_read = gmbus1_slave_index(wvalue);
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
			byte_data = i2c_bus->current_slave->get_byte(
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
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data)
{
	AUX_CH_REGISTERS reg;

	if (!i2c_bus->aux_ch.i2c_over_aux_ch || !i2c_bus->aux_ch.aux_ch_mot) {
		return;
	}

	reg = vgt_get_aux_ch_reg(offset);
	*(unsigned int *)p_data = *i2c_bus->aux_ch.aux_registers[port_idx][reg];

	return;
}

#define AUX_CTL_MSG_LENGTH(reg) \
	((reg & _DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >> \
		_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT)

void vgt_i2c_handle_aux_ch_write(vgt_i2c_bus_t *i2c_bus,
				vgt_edid_data_t *edid,
				unsigned int offset,
				VGT_DP_PORTS_IDX port_idx, void *p_data)
{
	int msg_length, ret_msg_size;
	bool edid_presented;
	int msg, addr, ctrl, op;
	int value = *(int *)p_data;
	int aux_data_for_write = 0;
	AUX_CH_REGISTERS reg = vgt_get_aux_ch_reg(offset);
	edid_presented = (edid && edid->data_valid);

	EDID_MSG(VGT_EDID_INFO, edid_presented,
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

	/* Always set the wanted value for vms. */
	ret_msg_size = (((op & 0x1) == VGT_AUX_I2C_READ) ? 2 : 1);
	*i2c_bus->aux_ch.aux_registers[port_idx][reg] =
		_REGBIT_DP_AUX_CH_CTL_DONE |
		((ret_msg_size << _DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) &
		_DP_AUX_CH_CTL_MESSAGE_SIZE_MASK);

	if (msg_length == 3) {
		if (!(op & VGT_AUX_I2C_MOT)) {
			/* stop */
			EDID_MSG(VGT_EDID_INFO, edid_presented,
				"AUX_CH: stop. reset I2C!\n");
			vgt_init_i2c_bus(i2c_bus);
		} else {
			/* start or restart */
			EDID_MSG(VGT_EDID_INFO, edid_presented,
				"AUX_CH: start or restart I2C!\n");
			i2c_bus->aux_ch.i2c_over_aux_ch = true;
			i2c_bus->aux_ch.aux_ch_mot = true;
			if (addr == 0) {
				/* reset the address */
				EDID_MSG(VGT_EDID_INFO, edid_presented,
					"AUX_CH: reset I2C!\n");
				vgt_init_i2c_bus(i2c_bus);
			} else if (addr == EDID_ADDR) {
				EDID_MSG(VGT_EDID_INFO, edid_presented,
					"AUX_CH: setting EDID_ADDR!\n");
				i2c_bus->current_slave_addr = EDID_ADDR;
				i2c_bus->current_slave =
					(vgt_i2c_slave_t *)&i2c_bus->edid_slave;
				i2c_bus->edid_slave.edid_data = edid;
			} else {
				EDID_MSG(VGT_EDID_WARN, edid_presented,
		"Not supported address access [0x%x]with I2C over AUX_CH!\n",
				addr);
			}
		}
	} else if ((op & 0x1) == VGT_AUX_I2C_WRITE) {
		/* TODO
		 * We only support EDID reading from I2C_over_AUX. And
		 * we do not expect the index mode to be used. Right now
		 * the WRITE operation is ignored. It is good enough to
		 * support the gfx driver to do EDID access.
		 */
#if 0
		int write_length;
		int write_value;

		write_length = msg_length - 4;

		EDID_MSG(VGT_EDID_INFO, edid_presented,
			"AUX_CH WRITE length is:%d\n", write_length);

		ASSERT(write_length == 1);
		write_value = *i2c_bus->aux_ch.aux_registers[port_idx][reg + 2]
				 & 0xff;
		ASSERT(write_value == 0);
#endif
	} else {
		ASSERT((op & 0x1) == VGT_AUX_I2C_READ);
		ASSERT(msg_length == 4);
		if (i2c_bus->current_slave) {
			/* seems that data burst of aux_ch for i2c can only work
			 * for write. The read can always read just one byte
			 */
			unsigned char val =
				i2c_bus->current_slave->get_byte(i2c_bus->current_slave);
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
	i2c_bus->edid_slave.slave.get_byte = edid_get_byte;
	i2c_bus->edid_slave.current_read = 0;
	i2c_bus->edid_slave.edid_data = NULL;

	memset(&i2c_bus->gmbus, 0, sizeof(vgt_i2c_gmbus_t));

	i2c_bus->aux_ch.i2c_over_aux_ch = false;
	i2c_bus->aux_ch.aux_ch_mot = false;
	/*two arrays of aux_shadow_reg and aux_registers are initialized once
	 * in vgt_init_aux_ch_vregs(), which is not supposed to change again
	 */
}
