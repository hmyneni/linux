/*
 * Copyright 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _UAPI_MISC_VAS_H
#define _UAPI_MISC_VAS_H

#include <asm/ioctl.h>

#define VAS_MAGIC		'v'
#define VAS_FLAGS_PIN_WINDOW	0x1
#define VAS_FLAGS_HIGH_PRI	0x2
#define VAS_RSVD_TXBUF_MAX	16

#define VAS_GZIP_TX_WIN_OPEN		_IOW(VAS_MAGIC, 0x20, struct vas_tx_win_open_attr)

struct vas_tx_win_open_attr {
	__u32	version;
	__s16	vas_id;	/* specific instance of vas or -1 for default */
	__u64	reserved1;
	__u64	flags;
	__u64	reserved2;
	__u32	tc_mode;
	__u32	rsvd_txbuf;
	__u64	reserved3[6];
};

#endif /* _UAPI_MISC_VAS_H */
