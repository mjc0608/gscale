/*
 * vGT core headers
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

#ifndef _VGT_DRV_H_
#define _VGT_DRV_H_

#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <xen/interface/hvm/ioreq.h>
#include "vgt_edid.h"
#include "vgt_reg.h"
#include <xen/vgt-if.h>
#include <linux/hashtable.h>

struct pgt_device;
extern struct vgt_device *dom0_vgt;
extern void show_ringbuffer(struct pgt_device *pdev, int ring_id, int bytes);
extern void show_debug(struct pgt_device *pdev, int ring_id);
#define ASSERT(x)							\
	do {								\
		if (!(x)) { 						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__); 			\
			if (vgt_dom0)					\
				show_ringbuffer(vgt_dom0->pdev, 0, 64);	\
			BUG();						\
		}							\
	} while (0);
#define ASSERT_NUM(x, y)						\
	do {								\
		if (!(x)) { 						\
			printk("Assert at %s line %d para 0x%llx\n",	\
				__FILE__, __LINE__, (u64)y);		\
			if (vgt_dom0)					\
				show_ringbuffer(vgt_dom0->pdev, 0, 64);	\
			BUG();						\
		}							\
	} while (0);

extern bool hvm_render_owner;
extern bool hvm_dpy_owner;
extern bool hvm_owner;
extern bool hvm_super_owner;
extern bool vgt_primary;
extern bool vgt_debug;
extern bool novgt;
extern bool fastpath_dpy_switch;
extern int fastmode;
extern int disable_ppgtt;
extern int enable_video_switch;
extern bool use_old_ctx_switch;
extern int dom0_aperture_sz;
extern int dom0_gm_sz;
extern int dom0_fence_sz;
extern bool bypass_scan;

#define dprintk(fmt, a...)	\
	do { if (vgt_debug) printk("vGT:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##a); } while (0)

#define snb_device(dev)	1
#define ivb_device(dev)	0

typedef uint32_t vgt_reg_t;

enum vgt_event_type {

	// GT
	IRQ_RCS_MI_USER_INTERRUPT = 0,
	IRQ_RCS_DEBUG,
	IRQ_RCS_MMIO_SYNC_FLUSH,
	IRQ_RCS_CMD_STREAMER_ERR,
	IRQ_RCS_PIPE_CONTROL,
	IRQ_RCS_L3_PARITY_ERR,		/* IVB */
	IRQ_RCS_WATCHDOG_EXCEEDED,
	IRQ_RCS_PAGE_DIRECTORY_FAULT,
	IRQ_RCS_AS_CONTEXT_SWITCH,
	IRQ_RCS_MONITOR_BUFF_HALF_FULL,	/* IVB */

	IRQ_VCS_MI_USER_INTERRUPT,
	IRQ_VCS_MMIO_SYNC_FLUSH,
	IRQ_VCS_CMD_STREAMER_ERR,
	IRQ_VCS_MI_FLUSH_DW,
	IRQ_VCS_WATCHDOG_EXCEEDED,
	IRQ_VCS_PAGE_DIRECTORY_FAULT,
	IRQ_VCS_AS_CONTEXT_SWITCH,

	IRQ_BCS_MI_USER_INTERRUPT,
	IRQ_BCS_MMIO_SYNC_FLUSH,
	IRQ_BCS_CMD_STREAMER_ERR,
	IRQ_BCS_MI_FLUSH_DW,
	IRQ_BCS_PAGE_DIRECTORY_FAULT,
	IRQ_BCS_AS_CONTEXT_SWITCH,

	// DISPLAY
	IRQ_PIPE_A_FIFO_UNDERRUN,
	IRQ_PIPE_A_CRC_ERR,
	IRQ_PIPE_A_CRC_DONE,
	IRQ_PIPE_A_VSYNC,
	IRQ_PIPE_A_LINE_COMPARE,
	IRQ_PIPE_A_ODD_FIELD,
	IRQ_PIPE_A_EVEN_FIELD,
	IRQ_PIPE_A_VBLANK,
	IRQ_PIPE_B_FIFO_UNDERRUN,	// This is an active high level for the duration of the Pipe B FIFO underrun
	IRQ_PIPE_B_CRC_ERR,	// This is an active high pulse on the Pipe B CRC error
	IRQ_PIPE_B_CRC_DONE,	// This is an active high pulse on the Pipe B CRC done
	IRQ_PIPE_B_VSYNC,	// This is an active high level for the duration of the Pipe B vertical sync
	IRQ_PIPE_B_LINE_COMPARE,	// This is an active high level for the duration of the selected Pipe B scan lines
	IRQ_PIPE_B_ODD_FIELD,	// This is an active high level for the duration of the Pipe B interlaced odd field
	IRQ_PIPE_B_EVEN_FIELD,	// This is an active high level for the duration of the Pipe B interlaced even field
	IRQ_PIPE_B_VBLANK,	// This is an active high level for the duration of the Pipe B vertical blank
	IRQ_DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	IRQ_DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	IRQ_GSE,
	IRQ_DP_A_HOTPLUG,
	IRQ_AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	IRQ_PCH_IRQ,	// Only the rising edge of the PCH Display interrupt will cause the IIR to be set here
	IRQ_PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	IRQ_POISON,		// This is an active high pulse on receiving the poison message
	IRQ_GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	IRQ_PRIMARY_A_FLIP_DONE,
	IRQ_PRIMARY_B_FLIP_DONE,	// This is an active high pulse when a primary plane B flip is done
	IRQ_SPRITE_A_FLIP_DONE,
	IRQ_SPRITE_B_FLIP_DONE,	// This is an active high pulse when a sprite plane B flip is done
	IRQ_PIPE_C_VBLANK,		/* IVB DE */
	IRQ_PIPE_C_VSYNC,
	IRQ_PIPE_C_LINE_COMPARE,
	IRQ_PRIMARY_C_FLIP_DONE,
	IRQ_SPRITE_C_FLIP_DONE,
	IRQ_ERROR_INTERRUPT_COMBINED,

	// PM
	IRQ_GV_DOWN_INTERVAL,
	IRQ_GV_UP_INTERVAL,
	IRQ_RP_DOWN_THRESHOLD,
	IRQ_RP_UP_THRESHOLD,
	IRQ_FREQ_DOWNWARD_TIMEOUT_RC6,
	IRQ_PCU_THERMAL,
	IRQ_PCU_PCODE2DRIVER_MAILBOX,

	// PCH
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_A,	// This is an active high level indicating content protection is requested by audio azalia verb programming for transcoder A
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_B,
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_B,
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_B,
	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_C,
	IRQ_AUDIO_CP_CHANGE_TRANSCODER_C,
	IRQ_AUDIO_CP_REQUEST_TRANSCODER_C,
	IRQ_ERR_AND_DBG,
	IRQ_GMBUS,
	IRQ_SDVO_B_HOTPLUG,
	IRQ_CRT_HOTPLUG,
	IRQ_DP_B_HOTPLUG,
	IRQ_DP_C_HOTPLUG,
	IRQ_DP_D_HOTPLUG,
	IRQ_AUX_CHENNEL_B,
	IRQ_AUX_CHENNEL_C,
	IRQ_AUX_CHENNEL_D,
	IRQ_AUDIO_POWER_STATE_CHANGE_B,
	IRQ_AUDIO_POWER_STATE_CHANGE_C,
	IRQ_AUDIO_POWER_STATE_CHANGE_D,

	IRQ_RESERVED,
	IRQ_MAX,
};

/*
 * Define registers of a ring buffer per hardware  register layout.
 */
typedef struct {
    vgt_reg_t  tail;
    vgt_reg_t  head;
    vgt_reg_t  start;
    vgt_reg_t  ctl;
} vgt_ringbuffer_t;
#define _tail_reg_(ring_reg_off)	\
		(ring_reg_off & ~(sizeof(vgt_ringbuffer_t)-1))

#define  MAX_ENGINES		5

#define _vgt_mmio_va(pdev, x)		((char*)pdev->gttmmio_base_va+x)	/* PA to VA */
#define _vgt_mmio_pa(pdev, x)		(pdev->gttmmio_base+x)	/* PA to VA */
#define sleep_ns(x)	{long y=1UL*x/2; while (y-- > 0) ;}
#define sleep_us(x)	{long y=500UL*x; while (y-- > 0) ;}

#define SIZE_1KB			(1024UL)
#define SIZE_1MB			(1024UL*1024UL)

/* Maximum VMs supported by vGT. Actual number is device specific */
#define VGT_MAX_VMS			4
#define VGT_RSVD_APERTURE_SZ		(64*SIZE_1MB)	/* reserve 64MB for vGT itself */

#define VGT_MAX_GM_SIZE			(2*SIZE_1MB*SIZE_1KB)
#define VGT_GM_BITMAP_BITS		(VGT_MAX_GM_SIZE/SIZE_1MB)
#define VGT_MAX_NUM_FENCES		16
#define VGT_FENCE_BITMAP_BITS	VGT_MAX_NUM_FENCES
#define VGT_RSVD_APERTURE_BITMAP_BITS (VGT_RSVD_APERTURE_SZ/PAGE_SIZE)

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*64)	/* use 256 KB for now */
#define VGT_APERTURE_PER_INSTANCE_SZ		(4*SIZE_1MB)	/* 4MB per instance (?) */
extern unsigned long vgt_id_alloc_bitmap;
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#define REG_SIZE    		sizeof(vgt_reg_t)        /* size of gReg/sReg[0] */
#define REG_INDEX(reg)		((reg) / REG_SIZE)
#define VGT_MMIO_SPACE_SZ	(2*SIZE_1MB)
#define VGT_CFG_SPACE_SZ	256
#define VGT_BAR_NUM		4
typedef struct {
    uint64_t    mmio_base_gpa;	/* base guest physical address of the MMIO registers */
    vgt_reg_t	*vReg;		/* guest view of the register state */
    vgt_reg_t	*sReg;		/* Shadow (used by hardware) state of the register */
    uint8_t	cfg_space[VGT_CFG_SPACE_SZ];
    bool	bar_mapped[VGT_BAR_NUM];
    uint64_t	gt_mmio_base;	/* bar0/GTTMMIO  */
    uint64_t	aperture_base;	/* bar1: guest aperture base */
//    uint64_t	gt_gmadr_base;	/* bar1/GMADR */

    uint32_t	bar_size[VGT_BAR_NUM];	/* 0: GTTMMIO, 1: GMADR, 2: PIO bar size */

    /* FIXME: take them as part of vReg/sReg ??? */
    /* save indexed MMIO */
    uint8_t saveSR[8];  /* sequencer data register */
    uint8_t saveGR[25]; /* CRT controller register */
    uint8_t saveAR[21];
    uint8_t saveCR[37];
} vgt_state_t;

#define VGT_PPGTT_PDE_ENTRIES  512 /* current 512 entires for 2G mapping */

typedef struct {
	vgt_reg_t base;
	vgt_reg_t cache_ctl;
	vgt_reg_t mode;
} vgt_ring_ppgtt_t;

typedef struct {
	dma_addr_t shadow_addr;
	struct page	*pte_page;
	struct vm_struct *guest_pte_vm;
} vgt_ppgtt_pte_t;

typedef struct {
	dma_addr_t	virtual_phyaddr;
	dma_addr_t	shadow_pte_maddr;
	bool		big_page;	/* 32K page */
} vgt_ppgtt_pde_t;

#define __vreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))
#define __vreg8(vgt, off) (*(char *)((char *)vgt->state.vReg + off))
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.sReg + off))
#define __sreg8(vgt, off) (*(char *)((char *)vgt->state.sReg + off))
#define __vreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.vReg + off))
#define __sreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.sReg + off))
#define vgt_vreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.vReg + off))
#define vgt_sreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.sReg + off))

#define RB_DWORDS_TO_SAVE	32
typedef	uint32_t	rb_dword;
typedef struct {
	vgt_ringbuffer_t	vring;		/* guest view ring */
	vgt_ringbuffer_t	sring;		/* shadow ring */
	vgt_reg_t	phys_tail;	/* temproray tail reg for context S/R */
	rb_dword	save_buffer[RB_DWORDS_TO_SAVE];
	/* In aperture, partitioned & 4KB aligned. */
	/* 64KB alignment requirement for walkaround. */
	uint64_t	context_save_area;
	bool	initialized;	/* whether it includes an valid context */
	bool	stateless;	/* whether the engine requires special context switch */
	/* ppgtt info */
	vgt_ring_ppgtt_t	vring_ppgtt_info; /* guest view */
	vgt_ring_ppgtt_t	sring_ppgtt_info; /* shadow info */
	u8 has_ppgtt_base_set : 1;	/* Is PP dir base set? */
	u8 has_ppgtt_mode_enabled : 1;  /* Is ring's mode reg PPGTT enable set? */

	/* statistics */
	uint64_t nr_cmd_ring; /* cmd issued in ring buffer*/
	uint64_t nr_cmd_batch; /* cmd issued in batch buffer */
} vgt_state_ring_t;

struct vgt_device;
typedef bool (*vgt_mmio_read)(struct vgt_device *vgt, unsigned int offset,
	 void *p_data, unsigned int bytes);
typedef bool (*vgt_mmio_write)(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);

struct vgt_mmio_entry {
	struct hlist_node hlist;
	unsigned int base;
	vgt_mmio_read	read;
	vgt_mmio_write	write;
};

struct vgt_wp_page_entry {
	struct hlist_node hlist;
	unsigned int pfn;
	int idx;	/* shadow PTE index */
};

#define	VGT_HASH_BITS	6

/*
 * Ring ID definition.
 */
#define RING_BUFFER_RCS		0
#define RING_BUFFER_VCS		1
#define RING_BUFFER_BCS		2
#define RING_BUFFER_VECS	3
#define RING_BUFFER_VCS2	4

typedef bool (*submit_context_command_t) (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);
bool rcs_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);
bool default_submit_context_command (struct vgt_device *vgt,
	int ring_id, rb_dword *cmds, int bytes);

extern enum vgt_pipe surf_used_pipe;

struct vgt_intel_device_info {
	u8 gen;
	u8 pch;
	u8 is_mobile:1;
	u8 is_i85x:1;
	u8 is_i915g:1;
	u8 is_i945gm:1;
	u8 is_g33:1;
	u8 need_gfx_hws:1;
	u8 is_g4x:1;
	u8 is_pineview:1;
	u8 is_broadwater:1;
	u8 is_crestline:1;
	u8 is_ivybridge:1;
	u8 is_haswell:1;
	u8 has_fbc:1;
	u8 has_pipe_cxsr:1;
	u8 has_hotplug:1;
	u8 cursor_needs_physical:1;
	u8 has_overlay:1;
	u8 overlay_needs_physical:1;
	u8 supports_tv:1;
	u8 has_bsd_ring:1;
	u8 has_blt_ring:1;
};

struct pgt_device;

extern int vgt_thread(void *priv);
extern void vgt_destroy(void);
extern void vgt_destroy_debugfs(struct vgt_device *vgt);
extern void vgt_release_debugfs(void);
extern int vgt_initialize(struct pci_dev *dev);
extern bool vgt_register_mmio_handler(unsigned int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write);

extern bool need_scan_attached_ports;
extern bool vgt_reinitialize_mode(struct vgt_device *cur_vgt,
		struct vgt_device *next_vgt);
extern int vgt_hvm_info_init(struct vgt_device *vgt);
extern int vgt_hvm_io_init(struct vgt_device *vgt);
extern void vgt_hvm_info_deinit(struct vgt_device *vgt);
extern int vgt_hvm_enable(struct vgt_device *vgt);
extern void vgt_init_aux_ch_vregs(vgt_i2c_bus_t *i2c_bus, vgt_reg_t *vregs);

struct vgt_irq_virt_state;

struct vgt_hvm_info{
	shared_iopage_t *iopage;
	/* iopage_vma->addr is just iopage. We need iopage_vma on VM destroy */
	struct vm_struct *iopage_vma;

	int nr_vcpu;
	int* evtchn_irq; /* the event channle irqs to handle HVM io request
				 index is vcpu id */
};

struct vgt_statistics {
	u64	schedule_in_time;	/* TSC time when it is last scheduled in */
	u64	allocated_cycles;
	u64	used_cycles;
	u64	irq_num;
	u64	events[IRQ_MAX];
	u64	last_propogation;
	u64	last_blocked_propogation;
	u64	last_injection;
};

/* per-VM structure */
struct vgt_device {
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device  *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */
	vgt_reg_t		last_scan_head[MAX_ENGINES];

	struct vgt_port_struct *attached_port[I915_MAX_PIPES]; /* one port per PIPE */
	vgt_i2c_bus_t		vgt_i2c_bus;	/* i2c bus state emulaton for reading EDID */
	vgt_edid_data_t		*vgt_edids[EDID_MAX];	/* per display EDID information */

	uint64_t	aperture_base;
	void		*aperture_base_va;
	uint64_t 	aperture_sz;
	uint64_t 	gm_sz;
	uint64_t	aperture_offset;	/* address fix for visible GM */
	uint64_t	hidden_gm_offset;	/* address fix for invisible GM */
	int			fence_base;
	int			fence_sz;

#define VMEM_BUCK_SHIFT		20
#define VMEM_BUCK_SIZE		(1<<VMEM_BUCK_SHIFT)
	uint64_t	vmem_sz;
	struct vm_struct **vmem_vma;

	uint64_t   vgtt_sz; /* virtual GTT size in byte */
	uint32_t   *vgtt; /* virtual GTT table for guest to read */

	vgt_reg_t	saved_wakeup;		/* disable PM before switching */

	struct vgt_irq_virt_state *irq_vstate;
	struct vgt_hvm_info  *hvm_info;
        uint32_t        last_cf8;
	struct kobject kobj;
	struct vgt_statistics	stat;		/* statistics info */

	bool		ballooning;		/* VM supports ballooning */
	void*		opregion_va;
	uint32_t	opregion_pa;

	struct work_struct fb_debugfs_work;

	/* PPGTT info: currently not per-ring but assume three rings share same
	 * table.
	 */
	u32 ppgtt_base;
	bool ppgtt_initialized;
	bool need_ppgtt_setup;
	DECLARE_BITMAP(enabled_rings, MAX_ENGINES);
	DECLARE_BITMAP(started_rings, MAX_ENGINES);
	DECLARE_HASHTABLE(wp_table, VGT_HASH_BITS);
	vgt_ppgtt_pde_t	shadow_pde_table[VGT_PPGTT_PDE_ENTRIES];	 /* current max PDE entries should be 512 for 2G mapping */
	vgt_ppgtt_pte_t shadow_pte_table[VGT_PPGTT_PDE_ENTRIES]; /* Current PTE number is same as PDE entries */

	/* force removal from the render run queue */
	bool force_removal;

	/* Temporary flag for VEBOX guest driver support.
	 * Linux VM will have official VEBOX support until kernel 3.9.
	 * Windows driver already enables VEBOX support now.
	 * So in order to determine whether VM has turned on VEBOX on HSW, this
	 * flag is used. Will remove in future when VM drivers all have VEBOX
	 * support. */
	bool vebox_support;

	/* virtual force wake request */
	struct list_head v_force_wake_req;
};

extern struct vgt_device *vgt_dom0;
enum vgt_owner_type {
	VGT_OT_NONE = 0,		// No owner type
	VGT_OT_RCS,                  // the owner directly operating render command buffers
	VGT_OT_BCS,                 // the owner directly operating blitter command buffers
	VGT_OT_VCS,                   // the owner directly operating video command buffers
	VGT_OT_RENDER,                      // the owner directly operating all render buffers (render/blit/video)
	VGT_OT_DISPLAY,                 // the owner having its content directly shown on one or several displays
	VGT_OT_PM,                      // the owner handling GEN power management activities
	VGT_OT_MGMT,                    // the owner managing display/monitor resources
	VGT_OT_MAX,
};

/* owner type of the reg, up to 16 owner type */
#define VGT_REG_OWNER		(0xF)
/*
 * TODO:
 * Allows pReg access from any VM but w/o save/restore,
 * since we don't know the actual bit detail or virtualization
 * policy yet. the examples include many workaround registers.
 * regs marked with this flag should be cleared before final
 * release, since this way is unsafe.
 */
#define VGT_REG_WORKAROUND	(1 << 4)
/* reg contains address, requiring fix */
#define VGT_REG_ADDR_FIX	(1 << 5)
/* Status bit updated from HW */
#define VGT_REG_HW_STATUS	(1 << 6)
/* Virtualized */
#define VGT_REG_VIRT	(1 << 7)
/* Mode ctl registers with high 16 bits as the mask bits */
#define VGT_REG_MODE_CTL	(1 << 8)
/* VMs have different settings on this reg */
#define VGT_REG_NEED_SWITCH	(1 << 9)
/* This reg has been tracked in vgt_base_reg_info */
#define VGT_REG_TRACKED		(1 << 10)
/* This reg has been accessed by a VM */
#define VGT_REG_ACCESSED	(1 << 11)
/* Virtualized, but accessible by dom0 at boot time */
#define VGT_REG_BOOTTIME	(1 << 12)
/* This reg is saved/restored at context switch time */
#define VGT_REG_SAVED		(1 << 13)
/* index into another auxillary table. Maximum 256 entries now */
#define VGT_REG_INDEX_SHIFT	16
#define VGT_REG_INDEX_MASK	(0xFFFF << VGT_REG_INDEX_SHIFT)
typedef u32 reg_info_t;

#define VGT_AUX_TABLE_NUM	256
/* suppose a reg won't set both bits */
typedef union {
	struct {
		vgt_reg_t mask;
	} mode_ctl;
	struct {
		vgt_reg_t mask;
	} addr_fix;
} vgt_aux_entry_t;

struct vgt_irq_host_state;
#define VGT_VBIOS_PAGES 16

enum vgt_uevent_type {
    CRT_HOTPLUG_IN = 0,
    CRT_HOTPLUG_OUT,
	DP_A_HOTPLUG_IN,
	DP_A_HOTPLUG_OUT,
	SDVO_B_HOTPLUG_IN,
	SDVO_B_HOTPLUG_OUT,
	DP_B_HOTPLUG_IN,
	DP_B_HOTPLUG_OUT,
	DP_C_HOTPLUG_IN,
	DP_C_HOTPLUG_OUT,
	DP_D_HOTPLUG_IN,
	DP_D_HOTPLUG_OUT,
	HDMI_B_HOTPLUG_IN,
	HDMI_B_HOTPLUG_OUT,
	HDMI_C_HOTPLUG_IN,
	HDMI_C_HOTPLUG_OUT,
	HDMI_D_HOTPLUG_IN,
	HDMI_D_HOTPLUG_OUT,
	VGT_ENABLE_VGA,
	VGT_DISABLE_VGA,
    UEVENT_MAX
};

#define VGT_MAX_UEVENT_VARS 20
struct vgt_uevent_info {
    char *uevent_name;
	int vm_id;
    enum kobject_action action;
    char *env_var_table[VGT_MAX_UEVENT_VARS];
    bool (*vgt_uevent_handler)(struct vgt_uevent_info *uevent_entry, struct pgt_device *dev);
};

void inline vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent);

typedef union {
	uint32_t cmd;
	struct {
		uint32_t action : 1;
		uint32_t port_sel: 3;
		uint32_t rsvd_4_7 : 4;
		uint32_t vmid : 8;
		uint32_t rsvd_16_31 : 16;
	};
} vgt_hotplug_cmd_t;

enum vgt_output_type {
	VGT_OUTPUT_ANALOG = 0,
	VGT_OUTPUT_DISPLAYPORT,
	VGT_OUTPUT_EDP,
	VGT_OUTPUT_LVDS,
	VGT_OUTPUT_HDMI,
	VGT_OUTPUT_MAX
};

struct vgt_port_struct {
	bool enabled;
	enum vgt_output_type output_type;
	enum vgt_pipe attached_pipe;
	enum vgt_plane attached_plane;
	struct vgt_port_dsp_set_funcs *port_dsp_set_funcs;
	void *private;
};

/* Both DP and eDP port use this */
struct vgt_dp_port {
	unsigned int dp_ctrl_reg;
	vgt_reg_t dp_ctrl;
	u8 link_bw;
	u8 lane_count;
	u8 link_configuration[DP_LINK_CONFIGURATION_SIZE];
	u8 train_set[4];
	bool is_pch_edp;
};

struct vgt_port_dsp_set_funcs {
	void (*mode_fixup)(struct vgt_device *vgt,
			struct vgt_port_struct *port_struct);
	void (*prepare)(struct vgt_device *vgt,
			struct vgt_port_struct *port_struct);
	void (*mode_set)(struct vgt_device *vgt,
			struct vgt_port_struct *port_struct);
	void (*commit)(struct vgt_device *vgt,
			struct vgt_port_struct *port_struct);
	void (*detect)(struct vgt_device *vgt,
			struct vgt_port_struct *port_struct);
};


void vgt_destroy_attached_port(struct vgt_device *vgt);
struct vgt_dp_port *init_vgt_dp_port_private(
		unsigned int dp_ctrl_reg,
		bool is_pch_edp);
int init_vgt_port_struct(struct vgt_device *vgt,
		enum vgt_pipe pipe,
		enum vgt_plane plane,
		enum vgt_output_type otype);

struct pgt_statistics {
	u64	irq_num;
	u64	last_pirq;
	u64	last_virq;
	u64	pirq_cycles;
	u64	virq_cycles;
	u64	irq_delay_cycles;
	u64	events[IRQ_MAX];
};

/* per-device structure */
struct pgt_device {
	struct list_head	list;

	struct pci_bus *pbus;	/* parent bus of the device */
	struct pci_dev *pdev;	/* the gfx device bound to */
	int bus;		/* parent bus number */
	int devfn;		/* device function number */

	struct task_struct *p_thread;
	wait_queue_head_t event_wq;
	wait_queue_head_t destroy_wq;
	uint32_t request;

	uint64_t ctx_check;	/* the number of checked count in vgt thread */
	uint64_t ctx_switch;	/* the number of context switch count in vgt thread */
	uint32_t magic;		/* the magic number for checking the completion of context switch */

	vgt_reg_t *initial_mmio_state;	/* copy from physical at start */
	uint8_t initial_cfg_space[VGT_CFG_SPACE_SZ];	/* copy from physical at start */
	uint32_t bar_size[VGT_BAR_NUM];
	uint64_t total_gm_sz;	/* size of available GM space, e.g 2M GTT is 2GB */

	uint64_t gttmmio_base;	/* base of GTT and MMIO */
	void *gttmmio_base_va;	/* virtual base of mmio */
	uint64_t gmadr_base;	/* base of GMADR */
	void *gmadr_va;		/* virtual base of GMADR */
	u32 mmio_size;
	u32 gtt_size;
	int reg_num;

	int max_engines;	/* supported max engines */
	u32 ring_mmio_base[MAX_ENGINES];
	u32 ring_psmi[MAX_ENGINES];
	u32 ring_mi_mode[MAX_ENGINES];
	submit_context_command_t submit_context_command[MAX_ENGINES];

	vgt_edid_data_t		*pdev_edids[EDID_MAX];	/* per display EDID information */

	 /* 1 bit corresponds to 1MB in the GM space */
	DECLARE_BITMAP(gm_bitmap, VGT_GM_BITMAP_BITS);

	/* 1 bit corresponds to 1 fence register */
	DECLARE_BITMAP(fence_bitmap, VGT_FENCE_BITMAP_BITS);

	/* 1 bit corresponds to 1 PAGE(4K) in aperture */
	DECLARE_BITMAP(rsvd_aperture_bitmap, VGT_RSVD_APERTURE_BITMAP_BITS);

	uint64_t rsvd_aperture_sz;
	uint64_t rsvd_aperture_base;
	uint64_t scratch_page;		/* page used for data written from GPU */
	uint64_t ctx_switch_rb_page;	/* page used as ring buffer for context switch */
	uint64_t batch_buffer_page;	/* page used to map batch buffer */
	uint64_t dummy_area;

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct vgt_device *prev_owner[VGT_OT_MAX];	/* previous owner list of different engines */
	struct list_head rendering_runq_head;
	struct list_head rendering_idleq_head;
	spinlock_t lock;

	reg_info_t *reg_info;	/* virtualization policy for a given reg */
	struct vgt_irq_host_state *irq_hstate;

	uint64_t vgtt_sz; /* in bytes */
	uint32_t *vgtt; /* virtual GTT table for guest to read*/

	/* FIXME: Temperary solution: use this to tell hvm domain if external
	 * monitors are detected. This should be per vm, that
	 * we can decide how many monitors each vm can use, this
	 * data structure is mainly to provide the "read only"
	 * protection upon virtual regs
	 */
	DECLARE_BITMAP(port_detect_status, VGT_PORT_MAX);

	/* Add workqueue to handle non-critical tasks
	 * vgt_thread may be dedicated used for rendering context
	 * switch. When subimit task that require access vreg/sreg/hwreg
	 * , you need firstly get the lock (locks may be fine-grained later)
	 * */
	struct workqueue_struct *pgt_wq;

	u8 is_sandybridge : 1;
	u8 is_ivybridge : 1;
	u8 is_haswell : 1;
	u8 enable_ppgtt : 1;
	u8 in_ctx_switch : 1;
	u8 probe_ports : 1;

	vgt_aux_entry_t vgt_aux_table[VGT_AUX_TABLE_NUM];
	int at_index;

	struct pgt_statistics stat;
};

extern struct list_head pgt_devices;
/*
 * MI_STORE_DATA is used widely for synchronization between GPU and driver,
 * which suppports the destination in either a specific hardware status
 * page, or any other aperture pages mapped to main memory. We don't want
 * to switch the hardware status page from the VM, so adopt the latter form
 * with a scratch page created as the destination with layout defined as
 * below:
 */
#define VGT_DATA_CTX_MAGIC	0x0	/* the magic number used in the context switch */
#define vgt_data_ctx_magic(d)		(d->scratch_page + VGT_DATA_CTX_MAGIC)

#define vgt_get_owner(d, t)		(d->owner[t])
#define vgt_get_previous_owner(d, t)	(d->prev_owner[t])
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_RENDER))
#define current_display_owner(d)	(vgt_get_owner(d, VGT_OT_DISPLAY))
#define current_pm_owner(d)		(vgt_get_owner(d, VGT_OT_PM))
#define current_mgmt_owner(d)		(vgt_get_owner(d, VGT_OT_MGMT))
#define is_current_render_owner(vgt)	(vgt && vgt == current_render_owner(vgt->pdev))
#define is_current_display_owner(vgt)	(vgt && vgt == current_display_owner(vgt->pdev))
#define is_current_pm_owner(vgt)	(vgt && vgt == current_pm_owner(vgt->pdev))
#define is_current_mgmt_owner(vgt)	(vgt && vgt == current_mgmt_owner(vgt->pdev))
#define previous_render_owner(d)	(vgt_get_previous_owner(d, VGT_OT_RENDER))
#define previous_display_owner(d)	(vgt_get_previous_owner(d, VGT_OT_DISPLAY))
#define previous_pm_owner(d)		(vgt_get_previous_owner(d, VGT_OT_PM))
#define previous_mgmt_owner(d)		(vgt_get_previous_owner(d, VGT_OT_MGMT))
#define vgt_ctx_check(d)		(d->ctx_check)
#define vgt_ctx_switch(d)		(d->ctx_switch)
extern void do_vgt_display_switch(struct pgt_device *pdev);
extern struct vgt_device *next_display_owner;

#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ADDR_FIX)
#define reg_hw_status(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_HW_STATUS)
#define reg_virt(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_VIRT)
#define reg_mode_ctl(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_MODE_CTL)
#define reg_workaround(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_WORKAROUND)
#define reg_need_switch(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_NEED_SWITCH)
#define reg_is_tracked(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_TRACKED)
#define reg_is_accessed(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ACCESSED)
#define reg_boottime(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_BOOTTIME)
#define reg_is_saved(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_SAVED)
#define reg_get_owner(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER)
#define reg_invalid(pdev, reg)		(!pdev->reg_info[REG_INDEX(reg)])
#define reg_aux_index(pdev, reg)	\
	((pdev->reg_info[REG_INDEX(reg)] & VGT_REG_INDEX_MASK) >> VGT_REG_INDEX_SHIFT)
#define reg_has_aux_info(pdev, reg)	(reg_mode_ctl(pdev, reg) | reg_addr_fix(pdev, reg))
#define reg_aux_mode_mask(pdev, reg)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].mode_ctl.mask)
#define reg_aux_addr_mask(pdev, index)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].addr_fix.mask)

static inline void reg_set_hw_status(struct pgt_device *pdev, vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_HW_STATUS;
}

static inline void reg_set_virt(struct pgt_device *pdev, vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_VIRT;
}

static inline void reg_set_boottime(struct pgt_device *pdev, vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_BOOTTIME;
}

/* mask bits for addr fix */
static inline void reg_set_addr_fix(struct pgt_device *pdev,
	vgt_reg_t reg, vgt_reg_t mask)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->at_index <= VGT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->vgt_aux_table[pdev->at_index].addr_fix.mask = mask;
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ADDR_FIX |
		(pdev->at_index << VGT_REG_INDEX_SHIFT);
	pdev->at_index++;
}

/* mask bits for mode mask */
static inline void reg_set_mode_ctl(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->at_index <= VGT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_MODE_CTL |
		(pdev->at_index << VGT_REG_INDEX_SHIFT);
	pdev->at_index++;
}

/* if the type is invalid, we assume dom0 always has the permission */
static inline bool reg_is_owner(struct vgt_device *vgt, vgt_reg_t reg)
{
	enum vgt_owner_type type;

	type = vgt->pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER;
	return vgt == vgt_get_owner(vgt->pdev, type);
}

static inline void reg_set_owner(struct pgt_device *pdev,
	vgt_reg_t reg, enum vgt_owner_type type)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= type & VGT_REG_OWNER;
}

static inline void reg_set_workaround(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_WORKAROUND;
}

static inline void reg_set_tracked(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_TRACKED;
}

static inline void reg_set_accessed(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ACCESSED;
}

static inline void reg_set_saved(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_SAVED;
}

static inline void reg_update_handlers(struct pgt_device *pdev,
	vgt_reg_t reg, int size, vgt_mmio_read read, vgt_mmio_write write)
{
	ASSERT_NUM(reg_is_tracked(pdev, reg), reg);
	/* TODO search attr table to update fields there */
	vgt_register_mmio_handler(reg, size, read, write);
}

/* request types to wake up main thread */
#define VGT_REQUEST_IRQ		0	/* a new irq pending from device */
#define VGT_REQUEST_UEVENT	1
#define VGT_REQUEST_PPGTT_INIT  2	/* shadow ppgtt init request */
#define VGT_REQUEST_SCHED 	3	/* immediate reschedule requested */

static inline void vgt_raise_request(struct pgt_device *pdev, uint32_t flag)
{
	set_bit(flag, (void *)&pdev->request);
	if (waitqueue_active(&pdev->event_wq))
		wake_up(&pdev->event_wq);
}

/* check whether a reg access should happen on real hw */
static inline bool reg_hw_access(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;

	/* regs accessible by dom0 at boot time */
	if (vgt_ops->boot_time && reg_boottime(pdev, reg))
		return true;

	/* filter out PVINFO PAGE to avoid clobbered by hvm_super_owner */
	if (reg >= VGT_PVINFO_PAGE && reg < VGT_PVINFO_PAGE + VGT_PVINFO_SIZE)
		return false;

	/* super owner give full access to HVM instead of dom0 */
	if (hvm_super_owner && vgt->vgt_id)
		return true;

	/* allows access from any VM. dangerous!!! */
	if (reg_workaround(pdev, reg))
		return true;

	/* normal phase of passthrough registers if vgt is the owner */
	if (reg_is_owner(vgt, reg))
		return true;

	//ASSERT(reg_virt(pdev, reg));
	return false;
}

#define D_SNB	(1 << 0)
#define D_IVB	(1 << 1)
#define D_HSW	(1 << 2)
#define D_GEN7PLUS (D_IVB | D_HSW)
#define D_ALL	(D_SNB | D_IVB | D_HSW)

typedef struct {
	u32			reg;
	int			size;
	u32			flags;
	vgt_reg_t		addr_mask;
	int			device;
	vgt_mmio_read		read;
	vgt_mmio_write		write;
} reg_attr_t;

static inline bool vgt_match_device_attr(struct pgt_device *pdev, reg_attr_t *attr)
{
	if (pdev->is_sandybridge)
		return attr->device & D_SNB;
	if (pdev->is_ivybridge)
		return attr->device & D_IVB;
	if (pdev->is_haswell)
		return attr->device & D_HSW;

	return false;
}

/*
 * Below are some wrappers for commonly used policy flags.
 * Add on demand to feed your requirement
 */
/* virtualized */
#define F_VIRT			VGT_OT_NONE | VGT_REG_VIRT
/* virtualized, but allow hw access from dom0 at boot time */
#define F_BOOTTIME		F_VIRT | VGT_REG_BOOTTIME
/*
 * render context
 * 	- render owner access pReg
 * 	- non-render owner access vReg
 */
#define F_RDR			VGT_OT_RENDER
/* render context, require address fix */
#define F_RDR_ADRFIX		F_RDR | VGT_REG_ADDR_FIX
/* render context, status updated by hw */
#define F_RDR_HWSTS		F_RDR | VGT_REG_HW_STATUS
/* render context, mode register (high 16 bits as write mask) */
#define F_RDR_MODE		F_RDR | VGT_REG_MODE_CTL
/*
 * display context
 * 	- display owner access pReg
 * 	- non-display owner access vReg
 */
#define F_DPY			VGT_OT_DISPLAY
/* display context, require address fix */
#define F_DPY_ADRFIX		F_DPY | VGT_REG_ADDR_FIX
/* display context, require address fix, status updated by hw */
#define F_DPY_HWSTS_ADRFIX	F_DPY_ADRFIX | VGT_REG_HW_STATUS
/*
 * pm context
 * 	- pm owner access pReg
 * 	- non-pm owner access vReg
 */
#define F_PM			VGT_OT_PM
/*
 * workaround reg
 * 	- any VM directly access pReg
 * 	- no save/restore
 * 	- dangerous as a workaround only
 */
#define F_WA			VGT_OT_NONE | VGT_REG_WORKAROUND
/*
 * suppose owned by management domain (e.g. dom0) only
 * (consider whether it's necessary)
 */
#define F_MGMT			VGT_OT_MGMT

extern int vgt_ctx_switch;
extern bool fastpath_dpy_switch;
extern void vgt_toggle_ctx_switch(bool enable);
extern void vgt_setup_reg_info(struct pgt_device *pdev);
extern bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev);
extern bool vgt_initial_mmio_setup (struct pgt_device *pdev);

/* definitions for physical aperture/GM space */
#define phys_aperture_sz(pdev)		(pdev->bar_size[1])
#define phys_aperture_pages(pdev)	(phys_aperture_sz(pdev) >> GTT_PAGE_SHIFT)
#define phys_aperture_base(pdev)	(pdev->gmadr_base)
#define phys_aperture_vbase(pdev)	(pdev->gmadr_va)

#define gm_sz(pdev)			(pdev->total_gm_sz)
#define gm_base(pdev)			(0ULL)
#define gm_pages(pdev)			(gm_sz(pdev) >> GTT_PAGE_SHIFT)
#define hidden_gm_base(pdev)		(phys_aperture_sz(pdev))

#define aperture_2_gm(pdev, addr)	(addr - phys_aperture_base(pdev))
#define v_aperture(pdev, addr)		(phys_aperture_vbase(pdev) + (addr))

#define vm_aperture_sz(pdev)		(pdev->vm_aperture_sz)
#define vm_gm_sz(pdev)			(pdev->vm_gm_sz)
#define vm_gm_hidden_sz(pdev)		(vm_gm_sz(pdev) - vm_aperture_sz(pdev))

/*
 * Aperture/GM virtualization
 *
 * NOTE: the below description says dom0's aperture starts at a non-zero place,
 * this is only true if you enable the dom0's kernel parameter
 * dom0_aperture_starts_at_128MB: now by default dom0's aperture starts at 0 of
 * the GM space since dom0 is the first vm to request for GM space.
 *
 * GM is split into two parts: the 1st part visible to CPU through an aperture
 * window mapping, and the 2nd part only accessible from GPU. The virtualization
 * policy is like below:
 *
 *                | VM1 | VM2 | DOM0| RSVD|    VM1   |    VM2   |
 *                ------------------------------------------------
 * Aperture Space |/////|\\\\\|xxxxx|ooooo|                     v
 * (Dev2_BAR)     v                       v                     v
 *                v                       v                     v
 * GM space       v   (visibale part)     v   (invisible part)  v
 * (start from 0) |/////|\\\\\|xxxxx|ooooo|//////////|\\\\\\\\\\|
 *                ^     ^                 ^          ^
 *                |     |  _______________|          |
 *                |     | /          ________________|
 * VM1 GM space   |     |/          /
 * (start from 0) |/////|//////////|
 */

/* definitions for vgt's aperture/gm space */
#define vgt_aperture_base(vgt)		(vgt->aperture_base)
#define vgt_aperture_vbase(vgt)		(vgt->aperture_base_va)
#define vgt_aperture_offset(vgt)	(vgt->aperture_offset)
#define vgt_hidden_gm_offset(vgt)	(vgt->hidden_gm_offset)
#define vgt_aperture_sz(vgt)		(vgt->aperture_sz)
#define vgt_gm_sz(vgt)			(vgt->gm_sz)
#define vgt_hidden_gm_sz(vgt)		(vgt_gm_sz(vgt) - vgt_aperture_sz(vgt))

#define vgt_aperture_end(vgt)		\
	(vgt_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_visible_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_aperture_offset(vgt))
#define vgt_visible_gm_end(vgt)		\
	(vgt_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_hidden_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_hidden_gm_offset(vgt))
#define vgt_hidden_gm_end(vgt)		\
	(vgt_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

/*
 * the view of the aperture/gm space from the VM's p.o.v
 *
 * when the VM supports ballooning, this view is the same as the
 * view of vGT driver.
 *
 * when the VM does not support ballooning, this view starts from
 * GM space ZERO
 */
#define vgt_guest_aperture_base(vgt)	\
	(vgt->ballooning ?		\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf) + vgt_aperture_offset(vgt) :	\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf))
#define vgt_guest_aperture_end(vgt)	\
	(vgt_guest_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_visible_gm_base(vgt)	\
	(vgt->ballooning ? vgt_visible_gm_base(vgt) : gm_base(vgt->pdev))
#define vgt_guest_visible_gm_end(vgt)	\
	(vgt_guest_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_hidden_gm_base(vgt)	\
	(vgt->ballooning ? 		\
		vgt_hidden_gm_base(vgt) :	\
		vgt_guest_visible_gm_end(vgt) + 1)
#define vgt_guest_hidden_gm_end(vgt)	\
	(vgt_guest_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

/* translate a guest aperture address to host aperture address */
static inline uint64_t g2h_aperture(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t offset;

	ASSERT_NUM((g_addr >= vgt_guest_aperture_base(vgt)) &&
		(g_addr <= vgt_guest_aperture_end(vgt)), g_addr);

	offset = g_addr - vgt_guest_aperture_base(vgt);
	return vgt_aperture_base(vgt) + offset;
}

/* translate a host aperture address to guest aperture address */
static inline uint64_t h2g_aperture(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t offset;

	ASSERT_NUM((h_addr >= vgt_aperture_base(vgt)) &&
		(h_addr <= vgt_aperture_end(vgt)), h_addr);

	offset = h_addr - vgt_aperture_base(vgt);
	return vgt_guest_aperture_base(vgt) + offset;
}

/* check whether a guest GM address is within the CPU visible range */
static inline bool g_gm_is_visible(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_visible_gm_base(vgt)) &&
		(g_addr <= vgt_guest_visible_gm_end(vgt));
}

/* check whether a guest GM address is out of the CPU visible range */
static inline bool g_gm_is_hidden(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_hidden_gm_base(vgt)) &&
		(g_addr <= vgt_guest_hidden_gm_end(vgt));
}

static inline bool g_gm_is_valid(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_gm_is_visible(vgt, g_addr) || g_gm_is_hidden(vgt, g_addr);
}

/* check whether a host GM address is within the CPU visible range */
static inline bool h_gm_is_visible(struct vgt_device *vgt, uint64_t h_addr)
{
	return (h_addr >= vgt_visible_gm_base(vgt)) &&
		(h_addr <= vgt_visible_gm_end(vgt));
}

/* check whether a host GM address is out of the CPU visible range */
static inline bool h_gm_is_hidden(struct vgt_device *vgt, uint64_t h_addr)
{
	return (h_addr >= vgt_hidden_gm_base(vgt)) &&
		(h_addr <= vgt_hidden_gm_end(vgt));
}

static inline bool h_gm_is_valid(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_gm_is_visible(vgt, h_addr) || h_gm_is_hidden(vgt, h_addr);
}

/* for a guest GM address, return the offset within the CPU visible range */
static inline uint64_t g_gm_visible_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_visible_gm_base(vgt);
}

/* for a guest GM address, return the offset within the hidden range */
static inline uint64_t g_gm_hidden_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_hidden_gm_base(vgt);
}

/* for a host GM address, return the offset within the CPU visible range */
static inline uint64_t h_gm_visible_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_visible_gm_base(vgt);
}

/* for a host GM address, return the offset within the hidden range */
static inline uint64_t h_gm_hidden_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_hidden_gm_base(vgt);
}

/* translate a guest gm address to host gm address */
static inline uint64_t g2h_gm(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t h_addr;

	ASSERT_NUM(g_gm_is_valid(vgt, g_addr), g_addr);

	if (g_gm_is_visible(vgt, g_addr))	/* aperture */
		h_addr = vgt_visible_gm_base(vgt) +
			g_gm_visible_offset(vgt, g_addr);
	else	/* hidden GM space */
		h_addr = vgt_hidden_gm_base(vgt) +
			g_gm_hidden_offset(vgt, g_addr);

	return h_addr;
}

/* translate a host gm address to guest gm address */
static inline uint64_t h2g_gm(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t g_addr;

	ASSERT_NUM(h_gm_is_valid(vgt, h_addr), h_addr);

	if (h_gm_is_visible(vgt, h_addr))
		g_addr = vgt_guest_visible_gm_base(vgt) +
			h_gm_visible_offset(vgt, h_addr);
	else
		g_addr = vgt_guest_hidden_gm_base(vgt) +
			h_gm_hidden_offset(vgt, h_addr);

	return g_addr;
}

extern unsigned long rsvd_aperture_alloc(struct pgt_device *pdev,
		unsigned long size);
extern void rsvd_aperture_free(struct pgt_device *pdev, unsigned long start,
		unsigned long size);

extern dma_addr_t dummy_addr;
/*
 * check whether a structure pointed by MMIO, or an instruction filled in
 * the command buffer, may cross the visible and invisible boundary. That
 * should be avoid since physically two parts are not contiguous
 */
static inline bool check_g_gm_cross_boundary(struct vgt_device *vgt,
	uint64_t g_start, uint64_t size)
{
	if (!vgt_hidden_gm_offset(vgt))
		return false;

	return g_gm_is_visible(vgt, g_start) &&
		g_gm_is_hidden(vgt, g_start + size - 1);
}

#define GTT_BASE(pdev)		(pdev->gttmmio_base + pdev->mmio_size)
#define GTT_VBASE(pdev)		(pdev->gttmmio_base_va + pdev->mmio_size)
#define GTT_SIZE				(2* SIZE_1MB)
#define reg_is_mmio(pdev, reg)	\
	(reg >= 0 && reg < pdev->mmio_size)
#define reg_is_gtt(pdev, reg)	\
	(reg >= pdev->mmio_size && reg < pdev->mmio_size + pdev->gtt_size)

#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_PAE_MASK        ((1UL <<12) - (1UL << 4)) /* bit 11:4 */
#define GTT_ENTRY_SIZE		4

#define GTT_INDEX(pdev, addr)		\
	((u32)((addr - gm_base(pdev)) >> GTT_PAGE_SHIFT))

#define GTT_OFFSET_TO_INDEX(offset)		((offset) >> 2)

#define GTT_ADDR(pdev, index)		\
	(GTT_BASE(pdev) + index * GTT_ENTRY_SIZE)

#define GTT_VADDR(pdev, index)		\
	((u32*)GTT_VBASE(pdev) + index)

static inline uint32_t g2h_gtt_index(struct vgt_device *vgt, uint32_t g_index)
{
	uint64_t g_addr = g_index << GTT_PAGE_SHIFT;

	return (uint32_t)(g2h_gm(vgt, g_addr) >> GTT_PAGE_SHIFT);
}

static inline uint32_t h2g_gtt_index(struct vgt_device *vgt, uint32_t h_index)
{
	uint64_t h_addr = h_index << GTT_PAGE_SHIFT;

	return (uint32_t)(h2g_gm(vgt, h_addr) >> GTT_PAGE_SHIFT);
}

static inline struct ioreq * vgt_get_hvm_ioreq(struct vgt_device *vgt, int vcpu)
{
	return &(vgt->hvm_info->iopage->vcpu_ioreq[vcpu]);
}

static inline void __REG_WRITE(struct pgt_device *pdev,
	unsigned long reg, unsigned long val, int bytes)
{
	int ret;

	/*
	 * TODO: a simple mechanism to capture registers being
	 * saved/restored at render/display context switch time.
	 * It's not accurate, since vGT's normal mmio access
	 * within that window also falls here. But suppose that
	 * set is small for now.
	 *
	 * In the future let's wrap interface like vgt_restore_vreg
	 * for accurate tracking purpose.
	 */
	if (pdev->in_ctx_switch)
		reg_set_saved(pdev, reg);
	/* TODO: any license issue? */
	ret = hcall_mmio_write(_vgt_mmio_pa(pdev, reg), bytes, val);
	//ASSERT(ret == X86EMUL_OKAY);
}

static inline unsigned long __REG_READ(struct pgt_device *pdev,
	unsigned long reg, int bytes)
{
	unsigned long data;
	int ret;

	if (pdev->in_ctx_switch)
		reg_set_saved(pdev, reg);
	/* TODO: any license issue? */
	ret = hcall_mmio_read(_vgt_mmio_pa(pdev, reg), bytes, &data);
	//ASSERT(ret == X86EMUL_OKAY);

	return data;
}

#define VGT_MMIO_READ_BYTES(pdev, mmio_offset, bytes)	\
		__REG_READ(pdev, mmio_offset, bytes)

#define VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, val, bytes)	\
		__REG_WRITE(pdev, mmio_offset, val,  bytes)

#define VGT_MMIO_WRITE(pdev, mmio_offset, val)	\
		VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, (unsigned long)val, REG_SIZE)

#define VGT_MMIO_READ(pdev, mmio_offset)		\
		((vgt_reg_t)VGT_MMIO_READ_BYTES(pdev, mmio_offset, REG_SIZE))

#define VGT_MMIO_WRITE64(pdev, mmio_offset, val)	\
		__REG_WRITE(pdev, mmio_offset, val, 8)

#define VGT_MMIO_READ64(pdev, mmio_offset, val)		\
		__REG_READ(pdev, mmio_offset, 8)

#define VGT_REG_IS_ALIGNED(reg, bytes) (!((reg)&((bytes)-1)))
#define VGT_REG_ALIGN(reg, bytes) ((reg) & ~((bytes)-1))

#define vgt_restore_vreg(vgt, off)		\
	VGT_MMIO_WRITE(vgt->pdev, off, __vreg(vgt, off))

#define ARRAY_NUM(x)		(sizeof(x) / sizeof(x[0]))

/* Save/Restore display context */
int vgt_save_state(struct vgt_device *vgt);
int vgt_restore_state(struct vgt_device *vgt);

/*
 * Activate a VGT instance to render runqueue.
 */
static inline void vgt_enable_render(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	ASSERT(spin_is_locked(&pdev->lock));
	if (bitmap_empty(vgt->enabled_rings, MAX_ENGINES))
		printk("vGT-%d: Enable render but no ring is enabled yet\n",
			vgt->vgt_id);
	/* remove from idle queue */
	list_del(&vgt->list);
	/* add to run queue */
	list_add(&vgt->list, &pdev->rendering_runq_head);
	printk("vGT-%d: add to render run queue!\n", vgt->vgt_id);
}

/* now we scheduler all render rings together */
static inline void vgt_enable_ring(struct vgt_device *vgt, int ring_id)
{
	int enable = bitmap_empty(vgt->enabled_rings, MAX_ENGINES);

	set_bit(ring_id, (void *)vgt->enabled_rings);
	if (enable)
		vgt_enable_render(vgt);
}

/*
 * Remove a VGT instance from render runqueue.
 */
static inline void vgt_disable_render(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	ASSERT(spin_is_locked(&pdev->lock));
	if (!bitmap_empty(vgt->enabled_rings, MAX_ENGINES))
		printk("vGT-%d: disable render with enabled rings\n",
			vgt->vgt_id);
	/* remove from run queue */
	list_del(&vgt->list);
	/* add to idle queue */
	list_add(&vgt->list, &pdev->rendering_idleq_head);
	printk("vGT-%d: remove from render run queue!\n", vgt->vgt_id);
}

static inline void vgt_disable_ring(struct vgt_device *vgt, int ring_id)
{
	struct pgt_device *pdev = vgt->pdev;
	/* multiple disables */
	if (!test_and_clear_bit(ring_id, (void *)vgt->enabled_rings)) {
		printk("vGT-%d: disable a disabled ring (%d)\n",
			vgt->vgt_id, ring_id);
		return;
	}

	/* request to remove from runqueue if all rings are disabled */
	if (bitmap_empty(vgt->enabled_rings, MAX_ENGINES)) {
		ASSERT(spin_is_locked(&pdev->lock));
		if (current_render_owner(pdev) == vgt)
			vgt_raise_request(pdev, VGT_REQUEST_SCHED);
		else
			vgt_disable_render(vgt);
	}
}

vgt_reg_t mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t g_value);
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value);

static inline bool is_ring_empty(struct pgt_device *pdev, int ring_id)
{
	vgt_reg_t head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	vgt_reg_t tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));

	head &= RB_HEAD_OFF_MASK;
	/*
	 * FIXME: PRM said bit2-20 for head count, but bit3-20 for tail count
	 * however doing that makes tail always head/2.
	 */
	tail &= RB_TAIL_OFF_MASK;
	return (head == tail);
}

#define VGT_POST_READ(pdev, reg)		\
	do {					\
		vgt_reg_t val;			\
		val = VGT_MMIO_READ(pdev, reg);	\
	} while (0)

static inline bool is_ring_enabled (struct pgt_device *pdev, int ring_id)
{
	return (VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id)) & 1);	/* bit 0: enable/disable RB */
}

/* FIXME: use readl/writel as Xen doesn't trap GTT access now */
static inline u32 vgt_read_gtt(struct pgt_device *pdev, u32 index)
{
//	printk("vgt_read_gtt: index=0x%x, gtt_addr=%lx\n", index, GTT_ADDR(pdev, index));
	return VGT_MMIO_READ(pdev, pdev->mmio_size + index*GTT_ENTRY_SIZE);
	//return readl(GTT_VADDR(pdev, index));
}

static inline void vgt_write_gtt(struct pgt_device *pdev, u32 index, u32 val)
{
//	printk("vgt_write_gtt: index=0x%x, gtt_addr=%lx\n", index, GTT_ADDR(pdev, index));
	VGT_MMIO_WRITE(pdev, pdev->mmio_size + index*GTT_ENTRY_SIZE , val);
	//writel(val, GTT_VADDR(pdev, index));
}

static inline void vgt_pci_bar_write_32(struct vgt_device *vgt, uint32_t bar_offset, uint32_t val)
{
	uint32_t* cfg_reg;

	/* BAR offset should be 32 bits algiend */
	cfg_reg = (uint32_t*)&vgt->state.cfg_space[bar_offset & ~3];

	/* only write the bits 31-4, leave the 3-0 bits unchanged, as they are read-only */
	*cfg_reg = (val & 0xFFFFFFF0) | (*cfg_reg & 0xF);
}

#define vgt_clear_reg_bit(pdev, reg, bit)		\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val &= ~(1 << bit);			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
		VGT_POST_READ(pdev, reg);		\
	} while (0)

#define vgt_set_reg_bit(pdev, reg, bit)			\
	do {						\
		uint32_t val;				\
		val = VGT_MMIO_READ(pdev, reg);		\
		val |= 1 << bit;			\
		VGT_MMIO_WRITE(pdev, reg, val);		\
		VGT_POST_READ(pdev, reg);		\
	} while (0)

#define VGT_FIRST_RCS_EVENT	IRQ_RCS_MI_USER_INTERRUPT
#define VGT_LAST_RCS_EVENT	IRQ_RCS_MONITOR_BUFF_HALF_FULL
#define VGT_RCS_EVENT(e)	(e >= VGT_FIRST_RCS_EVENT && e <= VGT_LAST_RCS_EVENT)

#define VGT_FIRST_VCS_EVENT	IRQ_VCS_MI_USER_INTERRUPT
#define VGT_LAST_VCS_EVENT	IRQ_VCS_AS_CONTEXT_SWITCH
#define VGT_VCS_EVENT(e)	(e >= IRQ_VCS_MI_USER_INTERRUPT && e <= IRQ_VCS_AS_CONTEXT_SWITCH)

#define VGT_FIRST_BCS_EVENT	IRQ_BCS_MI_USER_INTERRUPT
#define VGT_LAST_BCS_EVENT	IRQ_BCS_AS_CONTEXT_SWITCH
#define VGT_BCS_EVENT(e)	(e >= IRQ_BCS_MI_USER_INTERRUPT && e <= IRQ_BCS_AS_CONTEXT_SWITCH)

#define VGT_FIRST_RENDER_EVENT	VGT_FIRST_RCS_EVENT
#define VGT_LAST_RENDER_EVENT	VGT_LAST_BCS_EVENT
#define VGT_RENDER_EVENT(e)	(e >= VGT_FIRST_RENDER_EVENT && e <= IRQ_BCS_AS_CONTEXT_SWITCH)

#define VGT_FIRST_DPY_EVENT	IRQ_PIPE_A_FIFO_UNDERRUN
#define VGT_LAST_DPY_EVENT	IRQ_ERROR_INTERRUPT_COMBINED
#define VGT_DPY_EVENT(e)	(e >= VGT_FIRST_DPY_EVENT && e <= VGT_LAST_DPY_EVENT)

#define VGT_FIRST_PM_EVENT	IRQ_GV_DOWN_INTERVAL
#define VGT_LAST_PM_EVENT	IRQ_PCU_PCODE2DRIVER_MAILBOX
#define VGT_PM_EVENT(e)		(e >= VGT_FIRST_PM_EVENT && e <= VGT_LAST_PM_EVENT)

#define VGT_FIRST_PCH_EVENT	IRQ_FDI_RX_INTERRUPTS_TRANSCODER_A
#define VGT_LAST_PCH_EVENT	IRQ_AUDIO_POWER_STATE_CHANGE_D
#define VGT_PCH_EVENT(e)	(e >= VGT_FIRST_PCH_EVENT && e <= VGT_LAST_PCH_EVENT)

#define VGT_FIRST_EVENT		VGT_FIRST_RCS_EVENT
#define VGT_LAST_EVENT		IRQ_RESERVED
#define	VGT_IRQ_BITWIDTH	32

struct vgt_irq_info_entry;
struct vgt_irq_info;

typedef void (*vgt_core_handler_t)(struct pgt_device *dev, enum vgt_event_type event);

typedef void (*vgt_event_handler_t)(
	struct pgt_device *dev,
	int bit,
	struct vgt_irq_info_entry *entry,
	struct vgt_irq_info *info,
	bool physical,
	struct vgt_device *vgt);

typedef void (*vgt_emulate_handler_t)(struct vgt_device *vstate, enum vgt_event_type event, bool enable);

struct vgt_irq_info_entry {
	enum vgt_event_type event;
	vgt_event_handler_t event_handler;
	vgt_emulate_handler_t emul_handler;
};

/* per-device level-1 interrupt bit definitions */
struct vgt_irq_info {
	char *name;
	int reg_base;
	int table_size;
	void (*propogate_virtual_event)(struct vgt_device *vstate,
		int bit, struct vgt_irq_info *info);
	struct vgt_irq_info_entry table[VGT_IRQ_BITWIDTH];
};

#define VGT_DPY_EMUL_PERIOD	500000000	// 500ms for now

struct vgt_irq_ops {
	void (*init) (struct pgt_device *dev);

	void (*exit) (struct pgt_device *dev);

	irqreturn_t (*interrupt) (struct pgt_device *dev);

	void (*handle_virtual_interrupt) (struct pgt_device *dev, enum vgt_owner_type type);

	void (*toggle_hw_event) (struct pgt_device *dev,
			enum vgt_event_type event, int bit, bool enable);

	void (*save) (struct vgt_device *vstate,
			enum vgt_owner_type type);

	void (*restore) (struct vgt_device *vstate,
			enum vgt_owner_type type);

	enum vgt_event_type (*get_event_type_from_bit) (
			struct pgt_device *dev, uint32_t reg, uint32_t bit);

	int (*get_bit_from_event) (struct pgt_device *dev,
			enum vgt_event_type event, struct vgt_irq_info *info);

	struct vgt_irq_info *(*get_irq_info_from_event) (
			struct pgt_device *dev, enum vgt_event_type event);

	struct vgt_irq_info *(*get_irq_info_from_owner) (
			struct pgt_device *dev, enum vgt_owner_type owner);

	char *(*get_reg_name)(struct pgt_device *dev, uint32_t reg);
};

union vgt_event_state {
	/* common state for bit based status */
	vgt_reg_t val;

	/* command stream error */
	struct {
		int eir_reg;
		vgt_reg_t eir_val;
	} cmd_err;
};

/* structure containing device specific IRQ state */
struct vgt_irq_host_state {
	struct pgt_device *pdev;
	/*
	 * ownership table for each IRQ event. a default table is defined
	 * but we hope it be instance specific so that it's flexible enough
	 */
	enum vgt_owner_type *event_owner_table;
	/* vGT core event handler table */
	vgt_core_handler_t *core_handlers;
	/* always emulated events */
	DECLARE_BITMAP(emulated_events, IRQ_MAX);
	spinlock_t lock;

	struct vgt_irq_ops *ops;

	int i915_irq;
	int pirq;

	union vgt_event_state states[IRQ_MAX];

	/* master bit enable status from all VMs */
	u64 master_enable;
	/* pch enable status from all VMs */
	u64 pch_enable;
	u64 pch_unmask;

	/* cached pending events */
	u32 gt_iir;
	u32 de_iir;
	u32 pm_iir;
	u32 sde_iir;

	/* display/mgmt mask for DE and PCH registers */
	u32 de_dpy_mask;
	u32 de_mgmt_mask;
	u32 pch_dpy_mask;
	u32 pch_mgmt_mask;
};

struct vgt_emul_timer {
	struct hrtimer timer;
	DECLARE_BITMAP(events, IRQ_MAX);
	u64 period;
};

struct vgt_watchdog_timer {
	struct hrtimer timer;
};

struct vgt_device;
/* structure containing instance specific IRQ state */
struct vgt_irq_virt_state {
	struct vgt_device *vgt;
	DECLARE_BITMAP(emulated_events, IRQ_MAX);
	DECLARE_BITMAP(enabled_events, IRQ_MAX);
	struct vgt_emul_timer dpy_timer;
	struct vgt_watchdog_timer watchdog_timer;
	bool irq_pending;
	bool pch_irq_pending;
};

#define vgt_event_owner_table(d)	(d->irq_hstate->event_owner_table)
#define vgt_get_event_owner_type(d, e)	(d->irq_hstate->event_owner_table[e])
#define vgt_core_event_handlers(d)	(d->irq_hstate->core_handlers)
#define vgt_always_emulated_events(d)	(d->irq_hstate->emulated_events)
#define vgt_get_irq_ops(d)		(d->irq_hstate->ops)
#define vgt_i915_irq(d)			(d->irq_hstate->i915_irq)
#define vgt_pirq(d)			(d->irq_hstate->pirq)
#define vgt_master_enable(d)		(d->irq_hstate->master_enable)
#define vgt_pch_enable(d)		(d->irq_hstate->pch_enable)
#define vgt_pch_unmask(d)		(d->irq_hstate->pch_unmask)
#define vgt_pm_iir(d)			(d->irq_hstate->pm_iir)
#define vgt_de_iir(d)			(d->irq_hstate->de_iir)
#define vgt_gt_iir(d)			(d->irq_hstate->gt_iir)
#define vgt_sde_iir(d)			(d->irq_hstate->sde_iir)
#define vgt_de_dpy_mask(d)		(d->irq_hstate->de_dpy_mask)
#define vgt_de_mgmt_mask(d)		(d->irq_hstate->de_mgmt_mask)
#define vgt_pch_dpy_mask(d)		(d->irq_hstate->pch_dpy_mask)
#define vgt_pch_mgmt_mask(d)		(d->irq_hstate->pch_mgmt_mask)
#define vgt_event_state(d, e)		(d->irq_hstate->states[e])

#define vgt_get_id(s)			(s->vgt_id)
#define vgt_state_emulated_events(s)		(s->irq_vstate->emulated_events)
#define vgt_state_enabled_events(s)	(s->irq_vstate->enabled_events)
#define vgt_dpy_timer(s)		(s->irq_vstate->dpy_timer)
#define vgt_event_cnt(s, e)		(s->irq_vstate->event_cnt[e])

#define vgt_event_owner_is_core(d, e)	\
	(vgt_core_event_handlers(d) && (vgt_core_event_handlers(d))[e])
#define vgt_get_event_owner(d, e)	\
	(vgt_get_owner(d, vgt_get_event_owner_type(d, e)))
#define vgt_event_owner_is_null(d, e)	\
	!vgt_event_owner_is_core(d, e) && !vgt_get_event_owner(d, e)

#define vgt_core_event_handler(d, e)	\
	((vgt_core_event_handlers(d))[e])

//#ifdef VGT_DEBUG
#define vgt_trace_irq_event(i, t)	\
	printk("vGT (%s): catch event (%s) in reg (%x).\n",	\
		(i)->name, vgt_irq_name[(t)],			\
		vgt_iir((i)->reg_base))
//#else
//#define vgt_trace_irq_event(i, t)
//#endif

#define vgt_isr(base)	(base)
#define vgt_imr(base)	(base + 0x4)
#define vgt_iir(base)	(base + 0x8)
#define vgt_ier(base)	(base + 0xC)
#define vgt_imr_to_isr(vgt, reg)	(__vreg(vgt, reg - 0x4))
#define vgt_imr_to_iir(vgt, reg)	(__vreg(vgt, reg + 0x4))
#define vgt_imr_to_ier(vgt, reg)	(__vreg(vgt, reg + 0x8))
#define vgt_ier_to_isr(vgt, reg)	(__vreg(vgt, reg - 0xC))
#define vgt_ier_to_iir(vgt, reg)	(__vreg(vgt, reg - 0x4))
#define vgt_ier_to_imr(vgt, reg)	(__vreg(vgt, reg - 0x8))


extern uint8_t vgt_irq_warn_once[IRQ_MAX];
#define VGT_IRQ_WARN(i, t, msg)					\
	printk("!!!vGT (%s): event (%s) in reg (%x): " msg,	\
		(i)->name, vgt_irq_name[(t)],			\
		vgt_iir((i)->reg_base))
#define VGT_IRQ_WARN_ONCE(i, t, msg)			\
	do {						\
		if (!vgt_irq_warn_once[(t)]) { 		\
			vgt_irq_warn_once[(t)] = 1;	\
			VGT_IRQ_WARN(i, t, msg);	\
		}					\
	} while (0);

/* FIXME: IIR can handle 2 pending requests */
static inline void vgt_set_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->irq_pending = true;
}

static inline void vgt_clear_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->irq_pending = false;
}

static inline bool vgt_has_irq_pending(struct vgt_device *vstate)
{
	return vstate->irq_vstate->irq_pending;
}

/* propogation to DE is postponed since there may have multiple PCH events pending */
static inline void vgt_set_pch_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->pch_irq_pending = true;
}

static inline void vgt_clear_pch_irq_pending(struct vgt_device *vstate)
{
	vstate->irq_vstate->pch_irq_pending = false;
}

static inline bool vgt_has_pch_irq_pending(struct vgt_device *vstate)
{
	return vstate->irq_vstate->pch_irq_pending;
}

/*
 * Note. clear physical pending bit, and then forward to virtual
 * register which includes more bits other than interrupt pending
 * bit
 */
#define vgt_forward_and_clear_bit(state, reg, bit)		\
	do {							\
		uint32_t val;					\
		val = vgt_read32(reg);				\
		val &= ~(bit);					\
		vgt_write32(reg, val);				\
		*vgt_vreg(state, reg) |= bit;			\
	} while (0);

/* for IIR type registers */
#define vgt_forward_and_clear_reg(state, reg)			\
	do {							\
		uint32_t val;					\
		val = vgt_read32(reg);				\
		vgt_write32(val);				\
		*vgt_vreg(state, reg) = val;			\
	} while (0)

extern struct vgt_irq_ops snb_irq_ops;
void vgt_install_irq(struct pci_dev *pdev);
void inject_hvm_virtual_interrupt(struct vgt_device *vgt);
void inject_dom0_virtual_interrupt(struct vgt_device *vgt);
int vgt_vstate_irq_init(struct vgt_device *vgt);
void vgt_vstate_irq_exit(struct vgt_device *vgt);
int vgt_irq_init(struct pgt_device *pgt);
void vgt_irq_exit(struct pgt_device *pgt);
void vgt_irq_save_context(struct vgt_device *vstate, enum vgt_owner_type owner);
void vgt_irq_restore_context(struct vgt_device *vstate, enum vgt_owner_type owner);

void vgt_show_irq_state(struct vgt_device *vgt);
void vgt_propogate_pch_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info);
void vgt_propogate_virtual_event(struct vgt_device *vstate,
	int bit, struct vgt_irq_info *info);
void vgt_propogate_emulated_event(struct vgt_device *vstate,
	enum vgt_event_type event);
void vgt_irq_handle_event(struct pgt_device *dev, void *iir,
	struct vgt_irq_info *info, bool physical,
	enum vgt_owner_type o_type);
void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event, bool check);
void vgt_handle_virtual_interrupt(struct pgt_device *pdev, enum vgt_owner_type type);
void vgt_default_event_handler(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_unexpected_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_host_only_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_weak_event(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_cmd_stream_error(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_phase_in(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_histogram(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_crt_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_sdvo_b_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_dp_hdmi_b_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_dp_hdmi_c_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_dp_hdmi_d_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_dp_a_hotplug(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);

void vgt_trigger_display_hot_plug(struct pgt_device *dev, vgt_hotplug_cmd_t hotplug_cmd);

void vgt_handle_aux_channel(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);
void vgt_handle_gmbus(struct pgt_device *dev,
	int bit, struct vgt_irq_info_entry *entry, struct vgt_irq_info *info,
	bool physical, struct vgt_device *vgt);

void vgt_signal_uevent(struct pgt_device *dev);

void vgt_emulate_watchdog(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
void vgt_emulate_dpy_status(struct vgt_device *vstate, enum vgt_event_type event, bool enable);
bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
extern char *vgt_irq_name[IRQ_MAX];

typedef struct {
	int vm_id;
	int aperture_sz; /* in MB */
	int gm_sz;       /* in MB */
	int fence_sz;

	int vgt_primary; /* 0/1: config the vgt device as secondary/primary VGA,
						-1: means the ioemu doesn't supply a value */
} vgt_params_t;

int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp);
void vgt_release_instance(struct vgt_device *vgt);
int vgt_init_sysfs(struct pgt_device *pdev);
extern void vgt_set_display_pointer(int vm_id);
extern ssize_t vgt_get_display_pointer(char *buf);
extern void vgt_probe_edid(struct pgt_device *pdev, int index);
extern void vgt_propagate_edid(struct vgt_device *vgt, int index);
extern void vgt_clear_edid(struct vgt_device *vgt, int index);

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,	void *p_data, unsigned int bytes);
bool default_mmio_write(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes);
bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes);

void vgt_set_all_vreg_bit(struct pgt_device *pdev, unsigned int bit, unsigned int offset);
void vgt_clear_all_vreg_bit(struct pgt_device *pdev, unsigned int bit, unsigned int offset);

extern bool gtt_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

extern bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

#define INVALID_ADDR (~0UL)
extern unsigned long vgt_gma_2_gpa(struct vgt_device *vgt, unsigned long gma);

extern void* vgt_gma_to_va(struct vgt_device *vgt, unsigned long gma, bool ppgtt);

extern int gtt_p2m(struct vgt_device *vgt, uint32_t p_gtt_val, uint32_t *m_gtt_val);

extern unsigned long g2m_pfn(int vm_id, unsigned long g_pfn);

extern void* vgt_vmem_gpa_2_va(struct vgt_device *vgt, unsigned long gpa);

extern unsigned long gtt_pte_get_pfn(struct pgt_device *pdev, u32 pte);

#define INVALID_MFN  (~0UL)

extern void vgt_add_wp_page_entry(struct vgt_device *vgt, struct vgt_wp_page_entry *e);
extern void vgt_del_wp_page_entry(struct vgt_device *vgt, unsigned int pfn);

extern bool vgt_init_shadow_ppgtt(struct vgt_device *vgt);
extern bool vgt_setup_ppgtt(struct vgt_device *vgt);
extern void vgt_destroy_shadow_ppgtt(struct vgt_device *vgt);
extern bool vgt_ppgtt_handle_pte_wp(struct vgt_device *vgt, struct vgt_wp_page_entry *e,
			     unsigned int offset, void *p_data, unsigned int bytes);
extern void vgt_ppgtt_switch(struct vgt_device *vgt);

extern struct dentry *vgt_init_debugfs(struct pgt_device *pdev);
extern int vgt_create_debugfs(struct vgt_device *vgt);
/* Debugfs: if you want to enable dumping framebuffer,
 * you should define VGT_DEBUGFS_DUMP_FB, since Windows
 * can update surface base frequently, for performance
 * concern by default we disable dumping framebuffer.
 */

/* klog facility for buck printk */
extern int vgt_klog_init(void);
extern void vgt_klog_cleanup(void);
extern void klog_printk(const char *fmt, ...);
#undef VGT_DEBUGFS_DUMP_FB

typedef struct {
    char *node_name;
    u64 *stat;
} debug_statistics_t;

extern u64 gtt_mmio_rcnt;
extern u64 gtt_mmio_wcnt;
extern u64 gtt_mmio_wcycles;
extern u64 gtt_mmio_rcycles;
extern u64 mmio_rcnt;
extern u64 mmio_wcnt;
extern u64 mmio_wcycles;
extern u64 mmio_rcycles;
extern u64 ring_mmio_rcnt;
extern u64 ring_mmio_wcnt;
extern u64 ring_tail_mmio_wcnt;
extern u64 ring_tail_mmio_wcycles;
extern u64 context_switch_cost;
extern u64 context_switch_num;
extern u64 ring_0_idle;
extern u64 ring_0_busy;

struct vgt_port_output_struct {
	unsigned int ctrl_reg;
	vgt_reg_t enable_bitmask;
	vgt_reg_t select_bitmask;
	enum vgt_output_type output_type;
};

#endif	/* _VGT_DRV_H_ */
