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
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <asm/icswx.h>
#include <asm/opal-api.h>
#include <asm/opal.h>

#include "vas.h"

/*
 * TODO: These need tuning
 */
#define VAS_FAULT_WIN_FIFO_SIZE		(64 << 10)
#define VAS_FAULT_WIN_WCREDS		64


static irqreturn_t vas_irq_handler(int virq, void *data)
{
	struct vas_instance *vinst = data;

	pr_devel("VAS %d: virq %d\n", vinst->vas_id, virq);

	return IRQ_HANDLED;
}

int vas_setup_irq_mapping(struct vas_instance *vinst)
{
	int rc;
	uint32_t virq;
	int32_t girq_be, girq;
	uint64_t port_be, port;
	char devname[64];

	if (!opal_check_token(OPAL_VAS_GET_TRIGGER_PORT))
		return -ENODEV;

	snprintf(devname, sizeof(devname), "vas-inst-%d", vinst->vas_id);

	girq = 0;
	port = 0ULL;
	rc = opal_vas_get_trigger_port(vinst->vas_id, &girq_be, &port_be);
	if (rc)
		return -EINVAL;

	pr_devel("IRQ trigger %d, port 0x%llx, rc %d\n", girq, port, rc);
	girq = be32_to_cpu(girq_be);
	port = be64_to_cpu(port_be);

	virq = irq_create_mapping(NULL, girq);
	if (!virq) {
		pr_err("Inst%d: Unable to map global irq %d\n",
			vinst->vas_id, girq);
		return -EINVAL;
	}

	rc = request_irq(virq, vas_irq_handler, 0, devname, vinst);
	if (rc) {
		pr_err("Inst#%d: request_irq() returns %d\n",
			vinst->vas_id, rc);
		return rc;
	}

	vinst->hwirq = girq;
	vinst->irq_port = port;

	return 0;
}

void vas_free_irq_mapping(struct vas_instance *vinst)
{
	unsigned int irq;

	if (!vinst->hwirq)
		return;

	irq = irq_find_mapping(NULL, vinst->hwirq);
	if (!irq) {
		pr_err("Receieved unknown hwirq %d\n", vinst->hwirq);
		WARN_ON_ONCE(true);
		return;
	}

	free_irq(irq, vinst);
	vinst->hwirq = 0;
}

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
