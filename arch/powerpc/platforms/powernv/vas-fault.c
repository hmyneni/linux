/*
 * Copyright 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) "vas: " fmt

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <asm/icswx.h>

#include "vas.h"

/*
 * TODO: These need tuning
 */
#define VAS_FAULT_WIN_FIFO_SIZE		(64 << 10)
#define VAS_FAULT_WIN_WCREDS		64


/*
 * Fault window is opened per VAS instance. NX use FIFO in this window
 * to send fault CRBs upon page faults.
 */
int vas_setup_fault_window(struct vas_instance *vinst)
{
	struct vas_rx_win_attr attr;

	vinst->fault_fifo_size = VAS_FAULT_WIN_FIFO_SIZE;
	vinst->fault_fifo = kmalloc(vinst->fault_fifo_size, GFP_KERNEL);
	if (!vinst->fault_fifo) {
		pr_err("Unable to alloc %d bytes for fault_fifo\n",
				vinst->fault_fifo_size);
		return -ENOMEM;
	}

	memset(&attr, 0, sizeof(attr));
	attr.rx_fifo_size = vinst->fault_fifo_size;
	attr.rx_fifo = vinst->fault_fifo;

	attr.wcreds_max = VAS_FAULT_WIN_WCREDS;
	attr.tc_mode = VAS_THRESH_DISABLED;
	attr.pin_win = true;
	attr.tx_win_ord_mode = true;
	attr.rx_win_ord_mode = true;
	attr.fault_win = true;

	/*
	 * 3.1.4.32: Local Notification Control Register. notify_disable is
	 * true and interrupt disable is false for Fault windows
	 */
	attr.notify_disable = true;

	attr.lnotify_lpid = 0;
	attr.lnotify_pid = mfspr(SPRN_PID);
	attr.lnotify_tid = mfspr(SPRN_PID);

	vinst->fault_win = vas_rx_win_open(vinst->vas_id, VAS_COP_TYPE_FAULT,
					&attr);

	if (IS_ERR(vinst->fault_win)) {
		pr_err("VAS: Error %ld opening FaultWin\n",
			PTR_ERR(vinst->fault_win));
		kfree(vinst->fault_fifo);
		return PTR_ERR(vinst->fault_win);
	}

	pr_devel("VAS: Created FaultWin %d, LPID/PID/TID [%d/%d/%d]\n",
			vinst->fault_win->winid, attr.lnotify_lpid,
			attr.lnotify_pid, attr.lnotify_tid);

	return 0;
}

/*
 * We do not remove VAS instances. The following functions are needed
 * when we suppost VAS hotplug.
 */
#if 0
/*
 * Close the fault window and free the receive FIFO.
 *
 * TODO: vas_win_close() will block till pending requests are drained.
 * 	 The fault thread itself allocates the FIFO, opens the window
 * 	 and when done, closes the window and frees the FIFO.
 * 	 Are there any other race condition to watch for here or in
 * 	 vas_win_close()?
 *
 */
int vas_cleanup_fault_window(struct vas_instance *vinst)
{
	int rc;

	rc = vas_win_close(vinst->fault_win);
	if (rc < 0) {
		pr_err("VAS Fault handler %d: error %d closing window\n",
				vinst->vas_id, rc);
	}

	kfree(vinst->fault_fifo);
	vinst->fault_fifo = NULL;

	return rc;
}
#endif
