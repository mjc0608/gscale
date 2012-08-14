
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
	case 0x0402:
	case 0x0412:
	case 0x0422:
	case 0x040a:
	case 0x041a:
	case 0x042a:
	case 0x0406:
	case 0x0416:
	case 0x0426:
	case 0x0c06:
	case 0x0c16:
	case 0x0c26:
	case 0x0c02:
	case 0x0c12:
	case 0x0c22:
	case 0x0c0a:
	case 0x0c1a:
	case 0x0c2a:
            ret = 1;
            break;
        default:
            break;
    }
    return ret;
}

#endif  /* _VGT_DEVTABLE_H */
