/*
 * Copyright (c) 2012-2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _VGT_DEVTABLE_H
#define _VGT_DEVTABLE_H

static inline int _is_sandybridge(int devid)
{
    int ret = 0;

    switch (devid) {
	case 0x0102:
	case 0x0112:
	case 0x0122:
	case 0x0106:
	case 0x0116:
	case 0x0126:
	case 0x010A:
            ret = 1;
            break;
        default:
            break;
    }
    return ret;
}

static inline int _is_ivybridge(int devid)
{
    int ret = 0;

    switch (devid) {
	case 0x0156:
	case 0x0166:
	case 0x0152:
	case 0x0162:
	case 0x015a:
	case 0x016a:
            ret = 1;
            break;
        default:
            break;
    }
    return ret;
}

static inline int _is_haswell(int devid)
{
    int ret = 0;

    switch (devid) {
	case 0x0400:
	case 0x0402:
	case 0x0404:
	case 0x0406:
	case 0x0408:
	case 0x040a:
	case 0x0412:
	case 0x0416:
	case 0x041a:
	case 0x0422:
	case 0x0426:
	case 0x042a:
	case 0x0a02:
	case 0x0a06:
	case 0x0a0a:
	case 0x0a12:
	case 0x0a16:
	case 0x0a1a:
	case 0x0a22:
	case 0x0a26:
	case 0x0a2a:
	case 0x0c02:
	case 0x0c04:
	case 0x0c06:
	case 0x0c0a:
	case 0x0c12:
	case 0x0c16:
	case 0x0c1a:
	case 0x0c22:
	case 0x0c26:
	case 0x0c2a:
	case 0x0d12:
	case 0x0d16:
	case 0x0d1a:
	case 0x0d22:
	case 0x0d26:
	case 0x0d2a:
	case 0x0d32:
	case 0x0d36:
	case 0x0d3a:
            ret = 1;
            break;
        default:
            break;
    }
    return ret;
}

#endif  /* _VGT_DEVTABLE_H */
