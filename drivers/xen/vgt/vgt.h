/*
 * vGT core headers
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

#ifndef _VGT_DRV_H_
#define _VGT_DRV_H_

#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/hashtable.h>
#include <linux/pci.h>

#include <xen/interface/hvm/ioreq.h>
#include <xen/interface/platform.h>
#include <xen/vgt-if.h>
#include <xen/vgt.h>

typedef uint32_t vgt_reg_t;

#include "reg.h"
#include "devtable.h"
#include "edid.h"

struct pgt_device;
struct vgt_device;
extern struct vgt_device *vgt_dom0;
extern struct pgt_device *perf_pgt;
extern struct list_head pgt_devices;
extern struct pgt_device default_device;
extern void show_ringbuffer(struct pgt_device *pdev, int ring_id, int bytes);
extern void show_batchbuffer(struct pgt_device *pdev, u32 addr, bool in_ppgtt);
extern void show_mode_settings(struct pgt_device *pdev);
extern void show_debug(struct pgt_device *pdev, int ring_id);

extern bool hvm_render_owner;
extern bool hvm_display_owner;
extern bool hvm_super_owner;
extern bool hvm_boot_foreground;
extern bool vgt_primary;
extern bool vgt_debug;
extern bool vgt_enabled;
extern bool fastpath_dpy_switch;
extern bool shadow_tail_based_qos;
extern bool event_based_qos;
extern int enable_video_switch;
extern int dom0_aperture_sz;
extern int dom0_gm_sz;
extern int dom0_fence_sz;
extern bool bypass_scan;
extern bool bypass_dom0_addr_check;
extern bool render_engine_reset;

enum vgt_event_type {
	// GT
	RCS_MI_USER_INTERRUPT = 0,
	RCS_DEBUG,
	RCS_MMIO_SYNC_FLUSH,
	RCS_CMD_STREAMER_ERR,
	RCS_PIPE_CONTROL,
	RCS_L3_PARITY_ERR,		/* IVB */
	RCS_WATCHDOG_EXCEEDED,
	RCS_PAGE_DIRECTORY_FAULT,
	RCS_AS_CONTEXT_SWITCH,
	RCS_MONITOR_BUFF_HALF_FULL,	/* IVB */

	VCS_MI_USER_INTERRUPT,
	VCS_MMIO_SYNC_FLUSH,
	VCS_CMD_STREAMER_ERR,
	VCS_MI_FLUSH_DW,
	VCS_WATCHDOG_EXCEEDED,
	VCS_PAGE_DIRECTORY_FAULT,
	VCS_AS_CONTEXT_SWITCH,

	BCS_MI_USER_INTERRUPT,
	BCS_MMIO_SYNC_FLUSH,
	BCS_CMD_STREAMER_ERR,
	BCS_MI_FLUSH_DW,
	BCS_PAGE_DIRECTORY_FAULT,
	BCS_AS_CONTEXT_SWITCH,

	VECS_MI_FLUSH_DW,

	// DISPLAY
	PIPE_A_FIFO_UNDERRUN,
	PIPE_A_CRC_ERR,
	PIPE_A_CRC_DONE,
	PIPE_A_VSYNC,
	PIPE_A_LINE_COMPARE,
	PIPE_A_ODD_FIELD,
	PIPE_A_EVEN_FIELD,
	PIPE_A_VBLANK,
	PIPE_B_FIFO_UNDERRUN,	// This is an active high level for the duration of the Pipe B FIFO underrun
	PIPE_B_CRC_ERR,	// This is an active high pulse on the Pipe B CRC error
	PIPE_B_CRC_DONE,	// This is an active high pulse on the Pipe B CRC done
	PIPE_B_VSYNC,	// This is an active high level for the duration of the Pipe B vertical sync
	PIPE_B_LINE_COMPARE,	// This is an active high level for the duration of the selected Pipe B scan lines
	PIPE_B_ODD_FIELD,	// This is an active high level for the duration of the Pipe B interlaced odd field
	PIPE_B_EVEN_FIELD,	// This is an active high level for the duration of the Pipe B interlaced even field
	PIPE_B_VBLANK,	// This is an active high level for the duration of the Pipe B vertical blank
	DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	GSE,
	DP_A_HOTPLUG,
	AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	PCH_IRQ,	// Only the rising edge of the PCH Display interrupt will cause the IIR to be set here
	PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	POISON,		// This is an active high pulse on receiving the poison message
	GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	PRIMARY_A_FLIP_DONE,
	PRIMARY_B_FLIP_DONE,	// This is an active high pulse when a primary plane B flip is done
	SPRITE_A_FLIP_DONE,
	SPRITE_B_FLIP_DONE,	// This is an active high pulse when a sprite plane B flip is done
	PIPE_C_VBLANK,		/* IVB DE */
	PIPE_C_VSYNC,
	PIPE_C_LINE_COMPARE,
	PRIMARY_C_FLIP_DONE,
	SPRITE_C_FLIP_DONE,
	ERROR_INTERRUPT_COMBINED,

	// PM
	GV_DOWN_INTERVAL,
	GV_UP_INTERVAL,
	RP_DOWN_THRESHOLD,
	RP_UP_THRESHOLD,
	FREQ_DOWNWARD_TIMEOUT_RC6,
	PCU_THERMAL,
	PCU_PCODE2DRIVER_MAILBOX,

	// PCH
	FDI_RX_INTERRUPTS_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_CHANGE_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_REQUEST_TRANSCODER_A,	// This is an active high level indicating content protection is requested by audio azalia verb programming for transcoder A
	FDI_RX_INTERRUPTS_TRANSCODER_B,
	AUDIO_CP_CHANGE_TRANSCODER_B,
	AUDIO_CP_REQUEST_TRANSCODER_B,
	FDI_RX_INTERRUPTS_TRANSCODER_C,
	AUDIO_CP_CHANGE_TRANSCODER_C,
	AUDIO_CP_REQUEST_TRANSCODER_C,
	ERR_AND_DBG,
	GMBUS,
	SDVO_B_HOTPLUG,
	CRT_HOTPLUG,
	DP_B_HOTPLUG,
	DP_C_HOTPLUG,
	DP_D_HOTPLUG,
	AUX_CHENNEL_B,
	AUX_CHENNEL_C,
	AUX_CHENNEL_D,
	AUDIO_POWER_STATE_CHANGE_B,
	AUDIO_POWER_STATE_CHANGE_C,
	AUDIO_POWER_STATE_CHANGE_D,

	EVENT_RESERVED,
	EVENT_MAX,
};

#define vgt_info(fmt, s...)	\
	do { printk(KERN_INFO "vGT info:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define vgt_warn(fmt, s...)	\
	do { printk(KERN_WARNING "vGT warning:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define vgt_err(fmt, s...)	\
	do { printk(KERN_ERR "vGT error:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define vgt_dbg(fmt, s...)	\
	do { if (vgt_debug) printk(KERN_DEBUG "vGT debug:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

/*
 * Define registers of a ring buffer per hardware register layout.
 */
typedef struct {
	vgt_reg_t tail;
	vgt_reg_t head;
	vgt_reg_t start;
	vgt_reg_t ctl;
} vgt_ringbuffer_t;

#define SIZE_1KB		(1024UL)
#define SIZE_1MB		(1024UL*1024UL)

#define VGT_RSVD_RING_SIZE	(16 * SIZE_1KB)
struct vgt_rsvd_ring {
	struct pgt_device *pdev;
	void *virtual_start;
	int start;
	uint64_t null_context;
	uint64_t indirect_state;
	int id;

	u32 head;
	u32 tail;
	int size;
	/* whether the engine requires special context switch */
	bool	stateless;
	/* whether the engine requires context switch */
	bool	need_switch;
};

#define _tail_reg_(ring_reg_off)	\
		(ring_reg_off & ~(sizeof(vgt_ringbuffer_t)-1))

#define _vgt_mmio_va(pdev, x)		((char*)pdev->gttmmio_base_va+x)	/* PA to VA */
#define _vgt_mmio_pa(pdev, x)		(pdev->gttmmio_base+x)			/* PA to VA */
#define sleep_ns(x)	{long y=1UL*x/2; while (y-- > 0) ;}
#define sleep_us(x)	{long y=500UL*x; while (y-- > 0) ;}

#define VGT_RING_TIMEOUT	500	/* in ms */

/* Maximum VMs supported by vGT. Actual number is device specific */
#define VGT_MAX_VMS			4
#define VGT_RSVD_APERTURE_SZ		(64*SIZE_1MB)	/* reserve 64MB for vGT itself */

#define VGT_MAX_GM_SIZE			(2*SIZE_1MB*SIZE_1KB)
#define VGT_GM_BITMAP_BITS		(VGT_MAX_GM_SIZE/SIZE_1MB)
#define VGT_MAX_NUM_FENCES		16
#define VGT_FENCE_BITMAP_BITS	VGT_MAX_NUM_FENCES
#define VGT_RSVD_APERTURE_BITMAP_BITS (VGT_RSVD_APERTURE_SZ/PAGE_SIZE)
#define VGT_APERTURE_PAGES	(VGT_RSVD_APERTURE_SZ >> PAGE_SHIFT)

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*64)	/* use 256 KB for now */
#define SZ_INDIRECT_STATE		(4096)		/* use 4KB for now */
#define VGT_APERTURE_PER_INSTANCE_SZ		(4*SIZE_1MB)	/* 4MB per instance (?) */
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#define REG_SIZE			sizeof(vgt_reg_t)		/* size of gReg/sReg[0] */
#define REG_INDEX(reg)		((reg) / REG_SIZE)
#define VGT_MMIO_SPACE_SZ	(2*SIZE_1MB)
#define VGT_CFG_SPACE_SZ	256
#define VGT_BAR_NUM		4
typedef struct {
	uint64_t	mmio_base_gpa;	/* base guest physical address of the MMIO registers */
	vgt_reg_t	*vReg;		/* guest view of the register state */
	vgt_reg_t	*sReg;		/* Shadow (used by hardware) state of the register */
	uint8_t	cfg_space[VGT_CFG_SPACE_SZ];
	bool	bar_mapped[VGT_BAR_NUM];
	uint64_t	gt_mmio_base;	/* bar0/GTTMMIO */
	uint64_t	aperture_base;	/* bar1: guest aperture base */
//	uint64_t	gt_gmadr_base;	/* bar1/GMADR */

	uint32_t	bar_size[VGT_BAR_NUM];	/* 0: GTTMMIO, 1: GMADR, 2: PIO bar size */

	/* OpRegion state */
	void		*opregion_va;
	uint64_t	opregion_gfn[VGT_OPREGION_PAGES];
} vgt_state_t;

#define VGT_PPGTT_PDE_ENTRIES	512 /* current 512 entires for 2G mapping */

typedef struct {
	vgt_reg_t base;
	vgt_reg_t cache_ctl;
	vgt_reg_t mode;
} vgt_ring_ppgtt_t;

typedef struct {
	dma_addr_t shadow_addr;
	struct page	*pte_page;
	void *guest_pte_va;
} vgt_ppgtt_pte_t;

typedef struct {
	dma_addr_t	virtual_phyaddr;
	dma_addr_t	shadow_pte_maddr;
	bool		big_page;	/* 32K page */
	u32		entry;
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
	/* In aperture, partitioned & 4KB aligned. */
	/* 64KB alignment requirement for walkaround. */
	uint64_t	context_save_area;	/* VGT default context space */
	uint32_t	active_vm_context;
	/* ppgtt info */
	vgt_ring_ppgtt_t	vring_ppgtt_info; /* guest view */
	vgt_ring_ppgtt_t	sring_ppgtt_info; /* shadow info */
	u8 has_ppgtt_base_set : 1;	/* Is PP dir base set? */
	u8 has_ppgtt_mode_enabled : 1;	/* Is ring's mode reg PPGTT enable set? */

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
	unsigned int align_bytes;
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
enum vgt_ring_id {
	RING_BUFFER_RCS = 0,
	RING_BUFFER_VCS,
	RING_BUFFER_BCS,
	RING_BUFFER_VECS,
	MAX_ENGINES
};

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

extern bool vgt_do_render_context_switch(struct pgt_device *pdev);
extern void vgt_destroy(void);
extern void vgt_destroy_debugfs(struct vgt_device *vgt);
extern void vgt_release_debugfs(void);
extern int vgt_initialize(struct pci_dev *dev);
extern bool vgt_register_mmio_handler(unsigned int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write);
extern void vgt_clear_mmio_table(void);

extern bool need_scan_attached_ports;
extern bool vgt_reinitialize_mode(struct vgt_device *cur_vgt,
		struct vgt_device *next_vgt);
extern int vgt_hvm_info_init(struct vgt_device *vgt);
extern int vgt_hvm_opregion_init(struct vgt_device *vgt, uint32_t gpa);
extern void vgt_hvm_info_deinit(struct vgt_device *vgt);
extern int vgt_hvm_enable(struct vgt_device *vgt);
extern int vgt_pause_domain(struct vgt_device *vgt);
extern void vgt_crash_domain(struct vgt_device *vgt);
extern void vgt_init_aux_ch_vregs(vgt_i2c_bus_t *i2c_bus, vgt_reg_t *vregs);

struct vgt_irq_virt_state;

#define MAX_HVM_VCPUS_SUPPORTED 128
struct vgt_hvm_info{
	shared_iopage_t *iopage;
	DECLARE_BITMAP(ioreq_pending, MAX_HVM_VCPUS_SUPPORTED);
	wait_queue_head_t io_event_wq;
	struct task_struct *emulation_thread;

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
	u64	events[EVENT_MAX];

	/* actually this is the number of pending
	* interrutps, check this in vgt_check_pending_events,
	* one injection can deliver more than one events
	*/
	u64	pending_events;
	u64	last_propagation;
	u64	last_blocked_propagation;
	u64	last_injection;
};

/* per-VM structure */
typedef cycles_t vgt_tslice_t;
struct vgt_sched_info {
	vgt_tslice_t start_time;
	vgt_tslice_t end_time;
	vgt_tslice_t actual_end_time;
	vgt_tslice_t rb_empty_delay;	/* cost for "wait rendering engines empty */

	int32_t priority;
	int32_t weight;
	int64_t time_slice;
	/* more properties and policies should be added in*/
};

#define VGT_TBS_DEFAULT_PERIOD (15 * 1000000) /* 15 ms */

struct vgt_hrtimer {
	struct hrtimer timer;
	u64 period;
};

#define VGT_TAILQ_RB_POLLING_PERIOD (2 * 1000000)
#define VGT_TAILQ_SIZE (SIZE_1MB)
#define VGT_TAILQ_MAX_ENTRIES ((VGT_TAILQ_SIZE)/sizeof(u32))
#define VGT_TAILQ_IDX_MASK (VGT_TAILQ_MAX_ENTRIES - 1)
/* Maximum number of tail can be cached is (VGT_TAILQ_MAX_ENTRIES - 1) */
struct vgt_tailq {
	u32 __head;
	u32 __tail;
	u32 *__buf_tail;  /* buffer to save tail value caught by tail-write */
	u32 *__buf_cmdnr; /* buffer to save cmd nr for each tail-write */
};
#define vgt_tailq_idx(idx) ((idx) & VGT_TAILQ_IDX_MASK)

/* DPCD start */
#define DPCD_SIZE	0x700

struct vgt_dpcd_data {
	u8 data[DPCD_SIZE];
};

enum dpcd_index {
	DPCD_DPA = 0,
	DPCD_DPB,
	DPCD_DPC,
	DPCD_DPD,
	DPCD_MAX
};

/* DPCD addresses */
#define DPCD_REV			0x000
#define DPCD_TRAINING_PATTERN_SET	0x102
#define	DPCD_SINK_COUNT			0x200
#define DPCD_LANE0_1_STATUS		0x202
#define DPCD_LANE2_3_STATUS		0x203
#define DPCD_LANE_ALIGN_STATUS_UPDATED	0x204
#define DPCD_SINK_STATUS		0x205

/* link training */
#define DPCD_TRAINING_PATTERN_SET_MASK	0x03
#define DPCD_LINK_TRAINING_DISABLED	0x00
#define DPCD_TRAINING_PATTERN_1		0x01
#define DPCD_TRAINING_PATTERN_2		0x02

#define DPCD_CP_READY_MASK		(1 << 6)

/* lane status */
#define DPCD_LANES_CR_DONE		0x11
#define DPCD_LANES_EQ_DONE		0x22
#define DPCD_SYMBOL_LOCKED		0x44

#define DPCD_INTERLANE_ALIGN_DONE	0x01

#define DPCD_SINK_IN_SYNC		0x03

/* DPCD end */

struct vgt_device {
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;	/* FIXME: used for context switch ?? */
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */
	vgt_reg_t		last_scan_head[MAX_ENGINES];

	struct vgt_port_struct *attached_port[I915_MAX_PIPES]; /* one port per PIPE */
	vgt_i2c_bus_t		vgt_i2c_bus;	/* i2c bus state emulaton for reading EDID */
	vgt_edid_data_t		*vgt_edids[VGT_PORT_MAX];	/* per display EDID information */
	struct vgt_dpcd_data		*vgt_dpcds[DPCD_MAX];	/* per display DPCD information */

	DECLARE_BITMAP(presented_ports, VGT_PORT_MAX);

	uint64_t	aperture_base;
	void		*aperture_base_va;
	uint64_t	aperture_sz;
	uint64_t	gm_sz;
	uint64_t	aperture_offset;	/* address fix for visible GM */
	uint64_t	hidden_gm_offset;	/* address fix for invisible GM */
	int			fence_base;
	int			fence_sz;

	bool		bypass_addr_check;

#define VMEM_1MB		(1ULL << 20)	/* the size of the first 1MB */
#define VMEM_BUCK_SHIFT		20
#define VMEM_BUCK_SIZE		(1ULL << VMEM_BUCK_SHIFT)
#define VMEM_BUCK_MASK		(~(VMEM_BUCK_SIZE - 1))
	uint64_t	vmem_sz;
	/* for the 1st 1MB memory of HVM: each vm_struct means one 4K-page */
	struct vm_struct **vmem_vma_low_1mb;
	/* for >1MB memory of HVM: each vm_struct means 1MB */
	struct vm_struct **vmem_vma;

	uint64_t vgtt_sz; /* virtual GTT size in byte */
	uint32_t *vgtt; /* virtual GTT table for guest to read */

	vgt_reg_t	saved_wakeup;		/* disable PM before switching */

	struct vgt_hvm_info *hvm_info;
		uint32_t		last_cf8;
	struct kobject kobj;
	struct vgt_statistics	stat;		/* statistics info */

	bool		ballooning;		/* VM supports ballooning */

	/* PPGTT info: currently not per-ring but assume three rings share same
	* table.
	 */
	u32 ppgtt_base;
	bool ppgtt_initialized;
	DECLARE_BITMAP(enabled_rings, MAX_ENGINES);
	DECLARE_BITMAP(started_rings, MAX_ENGINES);
	DECLARE_HASHTABLE(wp_table, VGT_HASH_BITS);
	vgt_ppgtt_pde_t	shadow_pde_table[VGT_PPGTT_PDE_ENTRIES];	/* current max PDE entries should be 512 for 2G mapping */
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

	/* embedded context scheduler information */
	struct vgt_sched_info sched_info;

	/* Tail Queue (used to cache tail-writingt) */
	struct vgt_tailq rb_tailq[MAX_ENGINES];

	bool has_context;

	atomic_t crashing;
	/*
	 * Have HVM been visible from boot time?
	 * Used when hvm_boot_foreground mode is enabled.
	 */
	bool hvm_boot_foreground_visible;

	bool warn_untrack;
};

enum vgt_owner_type {
	VGT_OT_NONE = 0,		// No owner type
	VGT_OT_RENDER,			// the owner directly operating all render buffers (render/blit/video)
	VGT_OT_DISPLAY,			// the owner having its content directly shown on one or several displays
	VGT_OT_CONFIG,			// the owner is always dom0 (PM, workarounds, etc.)
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
#define VGT_REG_PASSTHROUGH	(1 << 4)
/* reg contains address, requiring fix */
#define VGT_REG_ADDR_FIX	(1 << 5)
/* Status bit updated from HW */
#define VGT_REG_HW_STATUS	(1 << 6)
/* Virtualized */
#define VGT_REG_VIRT		(1 << 7)
/* Mode ctl registers with high 16 bits as the mask bits */
#define VGT_REG_MODE_CTL	(1 << 8)
/* VMs have different settings on this reg */
#define VGT_REG_NEED_SWITCH	(1 << 9)
/* This reg has been tracked in vgt_base_reg_info */
#define VGT_REG_TRACKED		(1 << 10)
/* This reg has been accessed by a VM */
#define VGT_REG_ACCESSED	(1 << 11)
/* This reg is saved/restored at context switch time */
#define VGT_REG_SAVED		(1 << 12)
/* Policies not impacted by the superowner mode */
#define VGT_REG_STICKY		(1 << 13)
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

/* PLUG_OUT must equal to PLUG_IN + 1
 * hot plug handler code has such assumption. Actually it might
 * be OK to send HOTPLUG only, not necessarily differ IN aond
 * OUT.
 */
enum vgt_uevent_type {
	CRT_HOTPLUG_IN = 0,
	CRT_HOTPLUG_OUT,
	PORT_A_HOTPLUG_IN,
	PORT_A_HOTPLUG_OUT,
	PORT_B_HOTPLUG_IN,
	PORT_B_HOTPLUG_OUT,
	PORT_C_HOTPLUG_IN,
	PORT_C_HOTPLUG_OUT,
	PORT_D_HOTPLUG_IN,
	PORT_D_HOTPLUG_OUT,
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

void vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent);

enum vgt_trace_type {
	VGT_TRACE_READ,
	VGT_TRACE_WRITE
};

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

struct pgt_statistics {
	u64	irq_num;
	u64	last_pirq;
	u64	last_virq;
	u64	pirq_cycles;
	u64	virq_cycles;
	u64	irq_delay_cycles;
	u64	events[EVENT_MAX];
};

#define PCI_BDF2(b,df)  ((((b) & 0xff) << 8) | ((df) & 0xff))

struct vgt_mmio_dev;

/* per-device structure */
struct pgt_device {
	struct list_head	list; /* list node for 'pgt_devices' */

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
	uint64_t gmadr_base;	/* base of GMADR */
	void *gmadr_va;		/* virtual base of GMADR */
	u32 mmio_size;
	u32 gtt_size;
	int reg_num;

	int max_engines;	/* supported max engines */
	u32 ring_mmio_base[MAX_ENGINES];
	u32 ring_mi_mode[MAX_ENGINES];
	u32 ring_idle[MAX_ENGINES];
	u8 ring_idle_bit[MAX_ENGINES];
	u8 ring_idle_check;

	vgt_edid_data_t		*pdev_edids[VGT_PORT_MAX];	/* per display EDID information */
	struct vgt_dpcd_data	*pdev_dpcds[DPCD_MAX];	/* per display DPCD information */

	 /* 1 bit corresponds to 1MB in the GM space */
	DECLARE_BITMAP(gm_bitmap, VGT_GM_BITMAP_BITS);

	/* 1 bit corresponds to 1 fence register */
	DECLARE_BITMAP(fence_bitmap, VGT_FENCE_BITMAP_BITS);

	/* 1 bit corresponds to 1 PAGE(4K) in aperture */
	DECLARE_BITMAP(rsvd_aperture_bitmap, VGT_RSVD_APERTURE_BITMAP_BITS);

	/* 1 bit corresponds to 1 vgt virtual force wake request */
	DECLARE_BITMAP(v_force_wake_bitmap, VGT_MAX_VMS);
	spinlock_t v_force_wake_lock;

	struct page *dummy_page;
	struct page *(*rsvd_aperture_pages)[VGT_APERTURE_PAGES];

	uint64_t rsvd_aperture_sz;
	uint64_t rsvd_aperture_base;
	uint64_t scratch_page;		/* page used for data written from GPU */
	uint64_t ctx_switch_rb_page;	/* page used as ring buffer for context switch */
	uint64_t batch_buffer_page;	/* page used to map batch buffer */
	uint64_t dummy_area;

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct vgt_device *foreground_vm;		/* current visible domain on display. */
	struct list_head rendering_runq_head; /* reuse this for context scheduler */
	struct list_head rendering_idleq_head; /* reuse this for context scheduler */
	spinlock_t lock;

	reg_info_t *reg_info;	/* virtualization policy for a given reg */
	struct vgt_irq_host_state *irq_hstate;

	uint64_t vgtt_sz; /* in bytes */
	uint32_t *vgtt; /* virtual GTT table for guest to read*/

	DECLARE_BITMAP(detected_ports, VGT_PORT_MAX);

	DECLARE_BITMAP(dpy_emul_request, VGT_MAX_VMS);

	u8 gen_dev_type;

	u8 enable_ppgtt : 1;
	u8 in_ctx_switch : 1;

	vgt_aux_entry_t vgt_aux_table[VGT_AUX_TABLE_NUM];
	int at_index;

	struct pgt_statistics stat;

	struct vgt_mmio_dev *mmio_dev;

	struct vgt_rsvd_ring ring_buffer[MAX_ENGINES]; /* vGT ring buffer */

	uint32_t opregion_pa;
	void *opregion_va;
};

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
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_RENDER))
#define current_display_owner(d)	(vgt_get_owner(d, VGT_OT_DISPLAY))
#define current_foreground_vm(d)	(d->foreground_vm)
#define current_config_owner(d)		(vgt_get_owner(d, VGT_OT_CONFIG))
#define is_current_render_owner(vgt)	(vgt && vgt == current_render_owner(vgt->pdev))
#define is_current_display_owner(vgt)	(vgt && vgt == current_display_owner(vgt->pdev))
#define is_current_config_owner(vgt)	(vgt && vgt == current_config_owner(vgt->pdev))
#define vgt_ctx_check(d)		(d->ctx_check)
#define vgt_ctx_switch(d)		(d->ctx_switch)
extern void do_vgt_fast_display_switch(struct vgt_device *pdev);

#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ADDR_FIX)
#define reg_hw_status(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_HW_STATUS)
#define reg_virt(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_VIRT)
#define reg_mode_ctl(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_MODE_CTL)
#define reg_passthrough(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_PASSTHROUGH)
#define reg_need_switch(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_NEED_SWITCH)
#define reg_is_tracked(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_TRACKED)
#define reg_is_accessed(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ACCESSED)
#define reg_is_saved(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_SAVED)
#define reg_is_sticky(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_STICKY)
#define reg_get_owner(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER)
#define reg_invalid(pdev, reg)		(!pdev->reg_info[REG_INDEX(reg)])
#define reg_aux_index(pdev, reg)	\
	((pdev->reg_info[REG_INDEX(reg)] & VGT_REG_INDEX_MASK) >> VGT_REG_INDEX_SHIFT)
#define reg_has_aux_info(pdev, reg)	(reg_mode_ctl(pdev, reg) | reg_addr_fix(pdev, reg))
#define reg_aux_mode_mask(pdev, reg)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].mode_ctl.mask)
#define reg_aux_addr_mask(pdev, index)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].addr_fix.mask)

/*
 * Kernel BUG() doesn't work, because bust_spinlocks try to unblank screen
 * which may call into i915 and thus cause undesired more errors on the
 * screen
 */
static inline void vgt_panic(void)
{
	int i;
	struct pgt_device *pdev = &default_device;

	for (i = 0; i < pdev->max_engines; i++) {
		show_debug(pdev, i);
		show_ringbuffer(pdev, i, 64);
	}

	dump_stack();
	printk("________end of stack dump_________\n");
	panic("FATAL VGT ERROR\n");
}
#define ASSERT(x)							\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__);			\
			vgt_panic();					\
		}							\
	} while (0);
#define ASSERT_NUM(x, y)						\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d para 0x%llx\n",	\
				__FILE__, __LINE__, (u64)y);		\
			vgt_panic();					\
		}							\
	} while (0);

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

static inline void reg_set_passthrough(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_PASSTHROUGH;
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

static inline void reg_set_sticky(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_STICKY;
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
#define VGT_REQUEST_CTX_SWITCH	2	/* immediate reschedule(context switch) requested */

static inline void vgt_raise_request(struct pgt_device *pdev, uint32_t flag)
{
	set_bit(flag, (void *)&pdev->request);
	if (waitqueue_active(&pdev->event_wq))
		wake_up(&pdev->event_wq);
}

static inline bool vgt_chk_raised_request(struct pgt_device *pdev, uint32_t flag)
{
	return !!(test_bit(flag, (void *)&pdev->request));
}

/* check whether a reg access should happen on real hw */
static inline bool reg_hw_access(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;

	/*
	 * In superowner mode, all registers, except those explicitly marked
	 * as sticky, are virtualized to Dom0 while passthrough to the 1st
	 * HVM.
	 */
	if (hvm_super_owner && !reg_is_sticky(pdev, reg)) {
		if (vgt->vgt_id)
			return true;
		else
			return false;
	}

	/* allows access from any VM. dangerous!!! */
	if (reg_passthrough(pdev, reg))
		return true;

	/* normal phase of passthrough registers if vgt is the owner */
	if (reg_is_owner(vgt, reg))
		return true;

	//ASSERT(reg_virt(pdev, reg));
	return false;
}

#define IS_SNB(pdev)	((pdev)->gen_dev_type == XEN_IGD_SNB)
#define IS_IVB(pdev)	((pdev)->gen_dev_type == XEN_IGD_IVB)
#define IS_HSW(pdev)	((pdev)->gen_dev_type == XEN_IGD_HSW)

#define D_SNB	(1 << 0)
#define D_IVB	(1 << 1)
#define D_HSW	(1 << 2)
#define D_GEN7PLUS	(D_IVB | D_HSW)
#define D_GEN75PLUS	(D_HSW)
#define D_HSW_PLUS	(D_HSW)
#define D_IVB_PLUS	(D_IVB | D_HSW)
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

typedef struct {
	u32			reg;
	int			size;
} reg_list_t;

static inline unsigned int vgt_gen_dev_type(struct pgt_device *pdev)
{
	if (IS_SNB(pdev))
		return D_SNB;
	if (IS_IVB(pdev))
		return D_IVB;
	if (IS_HSW(pdev))
		return D_HSW;
	WARN_ONCE(1, KERN_ERR "vGT: unknown GEN type!\n");
	return 0;
}

static inline bool vgt_match_device_attr(struct pgt_device *pdev, reg_attr_t *attr)
{
	return attr->device & vgt_gen_dev_type(pdev);
}

/*
 * Below are some wrappers for commonly used policy flags.
 * Add on demand to feed your requirement
 */
/* virtualized */
#define F_VIRT			VGT_OT_NONE | VGT_REG_VIRT

/*
 * config context (global setting, pm, workaround, etc.)
 * 	- config owner access pReg
 *      - non-config owner access vReg
 * (dom0 is the unique config owner)
 */
#define F_DOM0			VGT_OT_CONFIG

/*
 * render context
 *	- render owner access pReg
 *	- non-render owner access vReg
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
 *	- display owner access pReg
 *	- non-display owner access vReg
 */
#define F_DPY			VGT_OT_DISPLAY
/* display context, require address fix */
#define F_DPY_ADRFIX		F_DPY | VGT_REG_ADDR_FIX
/* display context, require address fix, status updated by hw */
#define F_DPY_HWSTS_ADRFIX	F_DPY_ADRFIX | VGT_REG_HW_STATUS

/*
 * passthrough reg (DANGEROUS!)
 *	- any VM directly access pReg
 *	- no save/restore
 *	- dangerous as a workaround only
 */
#define F_PT			VGT_OT_NONE | VGT_REG_PASSTHROUGH

extern int vgt_ctx_switch;
extern bool vgt_validate_ctx_switch;
extern bool fastpath_dpy_switch;
extern void vgt_toggle_ctx_switch(bool enable);
extern void vgt_kick_ringbuffers(struct vgt_device *vgt);
extern void vgt_setup_reg_info(struct pgt_device *pdev);
extern bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev);
extern bool vgt_initial_mmio_setup (struct pgt_device *pdev);
extern void vgt_initial_opregion_setup(struct pgt_device *pdev);
extern void state_vreg_init(struct vgt_device *vgt);
extern void state_sreg_init(struct vgt_device *vgt);

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
	(vgt->ballooning ?		\
		vgt_hidden_gm_base(vgt) :	\
		vgt_guest_visible_gm_end(vgt) + 1)
#define vgt_guest_hidden_gm_end(vgt)	\
	(vgt_guest_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

#if 0
/* These unused functions are for non-ballooning case. */
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
#endif

/* check whether a guest GM address is within the CPU visible range */
static inline bool g_gm_is_visible(struct vgt_device *vgt, uint64_t g_addr)
{
	if (vgt->bypass_addr_check)
		return true;

	return (g_addr >= vgt_guest_visible_gm_base(vgt)) &&
		(g_addr <= vgt_guest_visible_gm_end(vgt));
}

/* check whether a guest GM address is out of the CPU visible range */
static inline bool g_gm_is_hidden(struct vgt_device *vgt, uint64_t g_addr)
{
	if (vgt->bypass_addr_check)
		return true;

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
	if (vgt->bypass_addr_check)
		return true;

	return (h_addr >= vgt_visible_gm_base(vgt)) &&
		(h_addr <= vgt_visible_gm_end(vgt));
}

/* check whether a host GM address is out of the CPU visible range */
static inline bool h_gm_is_hidden(struct vgt_device *vgt, uint64_t h_addr)
{
	if (vgt->bypass_addr_check)
		return true;

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

	if (vgt->bypass_addr_check)
		return g_addr;

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

	if (vgt->bypass_addr_check)
		return h_addr;

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

#if 0
/* This unused function is for non-ballooning case. */
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
#endif

#define GTT_SIZE				(2* SIZE_1MB)
#define reg_is_mmio(pdev, reg)	\
	(reg >= 0 && reg < pdev->mmio_size)
#define reg_is_gtt(pdev, reg)	\
	(reg >= pdev->mmio_size && reg < pdev->mmio_size + pdev->gtt_size)

#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_PAE_MASK		((1UL <<12) - (1UL << 4)) /* bit 11:4 */
#define GTT_ENTRY_SIZE		4

#define GTT_INDEX(pdev, addr)		\
	((u32)((addr - gm_base(pdev)) >> GTT_PAGE_SHIFT))

#define GTT_OFFSET_TO_INDEX(offset)		((offset) >> 2)

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
	ret = hcall_mmio_read(_vgt_mmio_pa(pdev, reg), bytes, &data);
	//ASSERT(ret == X86EMUL_OKAY);

	return data;
}

#define VGT_MMIO_READ_BYTES(pdev, mmio_offset, bytes)	\
		__REG_READ(pdev, mmio_offset, bytes)

#define VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, val, bytes)	\
		__REG_WRITE(pdev, mmio_offset, val, bytes)

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

/* context scheduler */
#define CYCLES_PER_USEC	0x10c7ull
#define VGT_DEFAULT_TSLICE (4 * 1000 * CYCLES_PER_USEC)
#define ctx_start_time(vgt) ((vgt)->sched_info.start_time)
#define ctx_end_time(vgt) ((vgt)->sched_info.end_time)
#define ctx_remain_time(vgt) ((vgt)->sched_info.time_slice)
#define ctx_actual_end_time(vgt) ((vgt)->sched_info.actual_end_time)
#define ctx_rb_empty_delay(vgt) ((vgt)->sched_info.rb_empty_delay)

#define vgt_get_cycles() ({		\
	cycles_t __ret;				\
	rdtsc_barrier();			\
	__ret = get_cycles();		\
	rdtsc_barrier();			\
	__ret;						\
	})

#define RB_HEAD_TAIL_EQUAL(head, tail) \
	(((head) & RB_HEAD_OFF_MASK) == ((tail) & RB_TAIL_OFF_MASK))

extern bool event_based_qos;
extern struct vgt_device *next_sched_vgt;
extern bool vgt_vrings_empty(struct vgt_device *vgt);

/* context scheduler facilities functions */
static inline bool vgt_runq_is_empty(struct pgt_device *pdev)
{
	return (list_empty(&pdev->rendering_runq_head));
}

static inline void vgt_runq_insert(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	list_add(&vgt->list, &pdev->rendering_runq_head);
}

static inline void vgt_runq_remove(struct vgt_device *vgt)
{
	list_del(&vgt->list);
}

static inline void vgt_idleq_insert(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	list_add(&vgt->list, &pdev->rendering_idleq_head);
}

static inline void vgt_idleq_remove(struct vgt_device *vgt)
{
	list_del(&vgt->list);
}

static inline int vgt_nr_in_runq(struct pgt_device *pdev)
{
	int count = 0;
	struct list_head *pos;
	list_for_each(pos, &pdev->rendering_runq_head)
		count++;
	return count;
}

static inline void vgt_init_sched_info(struct vgt_device *vgt)
{
	ctx_remain_time(vgt) = VGT_DEFAULT_TSLICE;
	ctx_start_time(vgt) = 0;
	ctx_end_time(vgt) = 0;
	ctx_actual_end_time(vgt) = 0;
	ctx_rb_empty_delay(vgt) = 0;
}

/* main context scheduling process */
extern void vgt_sched_ctx(struct pgt_device *pdev);
extern void vgt_setup_countdown(struct vgt_device *vgt);
extern void vgt_initialize_ctx_scheduler(struct pgt_device *pdev);
extern void vgt_cleanup_ctx_scheduler(struct pgt_device *pdev);

extern void __raise_ctx_sched(struct vgt_device *vgt);
#define raise_ctx_sched(vgt) \
	if (event_based_qos)	\
		__raise_ctx_sched((vgt))

extern bool shadow_tail_based_qos;
int vgt_init_rb_tailq(struct vgt_device *vgt);
void vgt_destroy_rb_tailq(struct vgt_device *vgt);
int vgt_tailq_pushback(struct vgt_tailq *tailq, u32 tail, u32 cmdnr);
u32 vgt_tailq_last_stail(struct vgt_tailq *tailq);
/*
 *
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
/* whenever there is a ring enabled, the render(context switch ?) are enabled */
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

	clear_bit(ring_id, (void *)vgt->started_rings);

	/* multiple disables */
	if (!test_and_clear_bit(ring_id, (void *)vgt->enabled_rings)) {
		printk("vGT-%d: disable a disabled ring (%d)\n",
			vgt->vgt_id, ring_id);
		return;
	}

	/* request to remove from runqueue if all rings are disabled */
	if (bitmap_empty(vgt->enabled_rings, MAX_ENGINES)) {
		ASSERT(spin_is_locked(&pdev->lock));
		if (current_render_owner(pdev) == vgt) {
			next_sched_vgt = vgt_dom0;
			vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
		} else
			vgt_disable_render(vgt);
	}
}

static inline bool is_ring_empty(struct pgt_device *pdev, int ring_id)
{
	vgt_reg_t head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	vgt_reg_t tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));

	head &= RB_HEAD_OFF_MASK;
	/*
	 * PRM said bit2-20 for head count, but bit3-20 for tail count:
	 * this means: HW increases HEAD by 4, and SW must increase TAIL
	 * by 8(SW must add padding of MI_NOOP if necessary).
	 */
	tail &= RB_TAIL_OFF_MASK;
	return (head == tail);
}

#define VGT_POST_READ(pdev, reg)		\
	do {					\
		vgt_reg_t val;			\
		val = VGT_MMIO_READ(pdev, reg);	\
	} while (0)

#define VGT_READ_CTL(pdev, id)	VGT_MMIO_READ(pdev, RB_CTL(pdev, id))
#define VGT_WRITE_CTL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_CTL(pdev, id), val)
#define VGT_POST_READ_CTL(pdev, id)	VGT_POST_READ(pdev, RB_CTL(pdev,id))

#define VGT_READ_HEAD(pdev, id)	VGT_MMIO_READ(pdev, RB_HEAD(pdev, id))
#define VGT_WRITE_HEAD(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, id), val)
#define VGT_POST_READ_HEAD(pdev, id)	VGT_POST_READ(pdev, RB_HEAD(pdev,id))

#define VGT_READ_TAIL(pdev, id)	VGT_MMIO_READ(pdev, RB_TAIL(pdev, id))
#define VGT_WRITE_TAIL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, id), val)
#define VGT_POST_READ_TAIL(pdev, id)	VGT_POST_READ(pdev, RB_TAIL(pdev,id))

#define VGT_READ_START(pdev, id) VGT_MMIO_READ(pdev, RB_START(pdev, id))
#define VGT_WRITE_START(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_START(pdev, id), val)
#define VGT_POST_READ_START(pdev, id)	VGT_POST_READ(pdev, RB_START(pdev,id))

static inline bool is_ring_enabled (struct pgt_device *pdev, int ring_id)
{
	return (VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id)) & 1);	/* bit 0: enable/disable RB */
}
extern void vgt_ring_init(struct pgt_device *pdev, int id);

static inline u32 vgt_read_gtt(struct pgt_device *pdev, u32 index)
{
	return VGT_MMIO_READ(pdev, pdev->mmio_size + index*GTT_ENTRY_SIZE);
}

static inline void vgt_write_gtt(struct pgt_device *pdev, u32 index, u32 val)
{
	VGT_MMIO_WRITE(pdev, pdev->mmio_size + index*GTT_ENTRY_SIZE , val);
}

static inline void vgt_pci_bar_write_32(struct vgt_device *vgt, uint32_t bar_offset, uint32_t val)
{
	uint32_t* cfg_reg;

	/* BAR offset should be 32 bits algiend */
	cfg_reg = (uint32_t*)&vgt->state.cfg_space[bar_offset & ~3];

	/* only write the bits 31-4, leave the 3-0 bits unchanged, as they are read-only */
	*cfg_reg = (val & 0xFFFFFFF0) | (*cfg_reg & 0xF);
}

static inline int vgt_pci_mmio_is_enabled(struct vgt_device *vgt)
{
	return vgt->state.cfg_space[VGT_REG_CFG_COMMAND] &
		_REGBIT_CFG_COMMAND_MEMORY;
}

struct vgt_irq_host_state;
typedef void (*vgt_event_phys_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event);
typedef void (*vgt_event_virt_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt);

struct vgt_irq_ops {
	void (*init_irq) (struct vgt_irq_host_state *hstate);
	irqreturn_t (*irq_handler) (struct vgt_irq_host_state *hstate);
	void (*check_pending_irq) (struct vgt_device *vgt);
};

/* the list of physical interrupt control register groups */
enum vgt_irq_type {
	IRQ_INFO_GT,
	IRQ_INFO_DPY,
	IRQ_INFO_PCH,
	IRQ_INFO_PM,
	IRQ_INFO_MAX,
};

#define VGT_IRQ_BITWIDTH	32
/* device specific interrupt bit definitions */
struct vgt_irq_info {
	char *name;
	int reg_base;
	enum vgt_event_type bit_to_event[VGT_IRQ_BITWIDTH];
};

#define	EVENT_FW_ALL 0	/* event forwarded to all instances */
#define	EVENT_FW_DOM0 1	/* event forwarded to dom0 only */
#define	EVENT_FW_NONE 2	/* no forward */

/* the handoff state from p-event to v-event */
union vgt_event_state {
	/* common state for bit based status */
	vgt_reg_t val;

	/* command stream error */
	struct {
		int eir_reg;
		vgt_reg_t eir_val;
	} cmd_err;
};

/* per-event information */
struct vgt_event_info {
	/* device specific info */
	int			bit;	/* map to register bit */
	union vgt_event_state	state;	/* handoff state*/
	struct vgt_irq_info	*info;	/* register info */

	/* device neutral info */
	int			policy;	/* forwarding policy */
	vgt_event_phys_handler_t	p_handler;	/* for p_event */
	vgt_event_virt_handler_t	v_handler;	/* for v_event */
};

/* structure containing device specific IRQ state */
struct vgt_irq_host_state {
	struct pgt_device *pdev;
	struct vgt_irq_ops *ops;
	int i915_irq;
	int pirq;
	struct vgt_irq_info	*info[IRQ_INFO_MAX];
	struct vgt_event_info	events[EVENT_MAX];
	DECLARE_BITMAP(pending_events, EVENT_MAX);
};

#define vgt_get_event_phys_handler(h, e)	(h->events[e].p_handler)
#define vgt_get_event_virt_handler(h, e)	(h->events[e].v_handler)
#define vgt_set_event_val(h, e, v)	(h->events[e].state.val = v)
#define vgt_get_event_val(h, e)		(h->events[e].state.val)
#define vgt_get_event_policy(h, e)	(h->events[e].policy)
#define vgt_get_irq_info(h, e)		(h->events[e].info)
#define vgt_get_irq_ops(p)		(p->irq_hstate->ops)

/* common offset among interrupt control registers */
#define regbase_to_isr(base)	(base)
#define regbase_to_imr(base)	(base + 0x4)
#define regbase_to_iir(base)	(base + 0x8)
#define regbase_to_ier(base)	(base + 0xC)

static inline void vgt_clear_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
{
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	ASSERT(!(offset & 0x3));
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) & (~value);
			__vreg(vgt, offset) = vreg_data;
		}
	}
}

static inline void vgt_set_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
 {
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	ASSERT(!(offset & 0x3));

	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) | value;
			__vreg(vgt, offset) = vreg_data;
		}
	}
}


void vgt_forward_events(struct pgt_device *pdev);
void vgt_install_irq(struct pci_dev *pdev);
int vgt_irq_init(struct pgt_device *pgt);
void vgt_irq_exit(struct pgt_device *pgt);

void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event);

void vgt_trigger_display_hot_plug(struct pgt_device *dev, vgt_hotplug_cmd_t hotplug_cmd);

void vgt_signal_uevent(struct pgt_device *dev);

bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
extern char *vgt_irq_name[EVENT_MAX];

typedef struct {
	int vm_id;
	int aperture_sz; /* in MB */
	int gm_sz;	/* in MB */
	int fence_sz;

	int vgt_primary; /* 0/1: config the vgt device as secondary/primary VGA,
						-1: means the ioemu doesn't supply a value */
} vgt_params_t;

ssize_t get_avl_vm_aperture_gm_and_fence(struct pgt_device *pdev, char *buf,
		ssize_t buf_sz);
vgt_reg_t mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t g_value);
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value);
unsigned long rsvd_aperture_alloc(struct pgt_device *pdev, unsigned long size);
void rsvd_aperture_free(struct pgt_device *pdev, unsigned long start, unsigned long size);
int allocate_vm_aperture_gm_and_fence(struct vgt_device *vgt, vgt_params_t vp);
void free_vm_aperture_gm_and_fence(struct vgt_device *vgt);
int alloc_vm_rsvd_aperture(struct vgt_device *vgt);
void free_vm_rsvd_aperture(struct vgt_device *vgt);
void initialize_gm_fence_allocation_bitmaps(struct pgt_device *pdev);
void vgt_init_reserved_aperture(struct pgt_device *pdev);

int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp);
void vgt_release_instance(struct vgt_device *vgt);
int vgt_init_sysfs(struct pgt_device *pdev);
void vgt_destroy_sysfs(void);
extern void vgt_set_display_pointer(int vm_id);
extern ssize_t vgt_get_display_pointer(char *buf);
extern void vgt_probe_edid(struct pgt_device *pdev, int index);
extern void vgt_propagate_edid(struct vgt_device *vgt, int index);
extern void vgt_clear_edid(struct vgt_device *vgt, int index);
extern void vgt_probe_dpcd(struct pgt_device *pdev, int index);
extern void vgt_propagate_dpcd(struct vgt_device *vgt, int index);
extern void vgt_clear_dpcd(struct vgt_device *vgt, int index);

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,	void *p_data, unsigned int bytes);
bool default_mmio_write(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes);
bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes);

bool ring_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

bool ring_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

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

#define INVALID_MFN	(~0UL)

extern void vgt_add_wp_page_entry(struct vgt_device *vgt, struct vgt_wp_page_entry *e);
extern void vgt_del_wp_page_entry(struct vgt_device *vgt, unsigned int pfn);

extern bool vgt_init_shadow_ppgtt(struct vgt_device *vgt);
extern bool vgt_setup_ppgtt(struct vgt_device *vgt);
extern void vgt_destroy_shadow_ppgtt(struct vgt_device *vgt);
extern bool vgt_ppgtt_handle_pte_wp(struct vgt_device *vgt, struct vgt_wp_page_entry *e,
				unsigned int offset, void *p_data, unsigned int bytes);
extern void vgt_ppgtt_switch(struct vgt_device *vgt);
extern void vgt_try_setup_ppgtt(struct vgt_device *vgt);
extern int ring_ppgtt_mode(struct vgt_device *vgt, int ring_id, u32 off, u32 mode);

extern struct dentry *vgt_init_debugfs(struct pgt_device *pdev);
extern int vgt_create_debugfs(struct vgt_device *vgt);

/* command parser interface */
extern int vgt_cmd_parser_init(struct pgt_device *pdev);
extern void vgt_cmd_parser_exit(void);
extern int vgt_scan_vring(struct vgt_device *vgt, int ring_id);

/* klog facility for buck printk */
extern int vgt_klog_init(void);
extern void vgt_klog_cleanup(void);
extern void klog_printk(const char *fmt, ...);

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
extern u64 vm_pending_irq[VGT_MAX_VMS];

struct vgt_port_output_struct {
	unsigned int ctrl_reg;
	vgt_reg_t enable_bitmask;
	vgt_reg_t select_bitmask;
	enum vgt_output_type output_type;
};

struct vgt_mmio_dev {
	int devid_major;
	char *dev_name;
	struct class *class;
	struct cdev cdev;
	struct device *devnode[VGT_MAX_VMS];
};
#define VGT_MMIO_DEV_NAME "vgt_mmio"
int vgt_init_mmio_device(struct pgt_device *pdev);
void vgt_cleanup_mmio_dev(struct pgt_device *pdev);
int vgt_create_mmio_dev(struct vgt_device *vgt);
void vgt_destroy_mmio_dev(struct vgt_device *vgt);

/* copy from drmP.h */
static __inline__ bool drm_can_sleep(void)
{
	if (in_atomic() || irqs_disabled())
		return false;
	return true;
}

#define _wait_for(COND, MS, W) ({					\
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS);	\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			ret__ = -ETIMEDOUT;				\
			break;						\
		}							\
		if (W && drm_can_sleep()) {				\
			msleep(W);					\
		} else {						\
			cpu_relax();					\
		}							\
	}								\
	ret__;								\
})

#define wait_for(COND, MS) _wait_for(COND, MS, 1)

/* invoked likely in irq disabled condition */
#define wait_for_atomic(COND, MS) ({					\
	unsigned long cnt = MS*100;					\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (!(--cnt)) {						\
			ret__ = -ETIMEDOUT;				\
			break;						\
		}							\
		udelay(10);						\
	}								\
	ret__;								\
})

extern reg_attr_t vgt_base_reg_info[];
extern reg_list_t vgt_sticky_regs[];
extern int vgt_get_base_reg_num(void);
extern int vgt_get_sticky_reg_num(void);

void vgt_hvm_write_cf8_cfc(struct vgt_device *vgt,
	unsigned int port, unsigned int bytes, unsigned long val);
void vgt_hvm_read_cf8_cfc(struct vgt_device *vgt,
	unsigned int port, unsigned int bytes, unsigned long *val);

int vgt_hvm_opregion_map(struct vgt_device *vgt, int map);
struct vm_struct *map_hvm_iopage(struct vgt_device *vgt);
int hvm_get_parameter_by_dom(domid_t domid, int idx, uint64_t *value);
int xen_get_nr_vcpu(int vm_id);
int vgt_hvm_set_trap_area(struct vgt_device *vgt);
int vgt_hvm_map_apperture (struct vgt_device *vgt, int map);
int setup_gtt(struct pgt_device *pdev);
void check_gtt(struct pgt_device *pdev);
void free_gtt(struct pgt_device *pdev);
uint64_t vgt_get_gtt_size(struct pci_bus *bus);
uint32_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off);
int vgt_get_hvm_max_gpfn(int vm_id);
int vgt_hvm_vmem_init(struct vgt_device *vgt);
void vgt_vmem_destroy(struct vgt_device *vgt);
void* vgt_vmem_gpa_2_va(struct vgt_device *vgt, unsigned long gpa);
struct vgt_device *vmid_2_vgt_device(int vmid);
extern void vgt_print_dpcd(struct vgt_dpcd_data *dpcd);

#define ASSERT_VM(x, vgt)						\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__);			\
			if (atomic_cmpxchg(&(vgt)->crashing, 0, 1))	\
				break;					\
			vgt_warn("Killing VM%d\n", (vgt)->vm_id);	\
			if (!vgt_pause_domain((vgt)))			\
				vgt_crash_domain((vgt));		\
		}							\
	} while (0)

#endif	/* _VGT_DRV_H_ */
