/*
 * vGT ftrace header
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

/* TODO: tracing read/write per VM */
#if !defined(_VGT_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _VGT_TRACE_H_

#include <linux/types.h>
#include <linux/stringify.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM vgt
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)

TRACE_EVENT(vgt_mmio_rw,
		TP_PROTO(bool write, u32 vm_id, u32 offset, void *pd,
			int bytes),

		TP_ARGS(write, vm_id, offset, pd, bytes),

		TP_STRUCT__entry(
			__field(bool, write)
			__field(u32, vm_id)
			__field(u32, offset)
			__field(int, bytes)
			__field(u64, value)
			),

		TP_fast_assign(
			__entry->write = write;
			__entry->vm_id = vm_id;
			__entry->offset = offset;
			__entry->bytes = bytes;

			memset(&__entry->value, 0, sizeof(u64));
			memcpy(&__entry->value, pd, bytes);
		),

		TP_printk("VM%u %s offset 0x%x data 0x%llx byte %d\n",
				__entry->vm_id,
				__entry->write ? "write" : "read",
				__entry->offset,
				__entry->value,
				__entry->bytes)
);

#endif /* _VGT_TRACE_H_ */

/* This part must be out of protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE vgt_trace
#include <trace/define_trace.h>
