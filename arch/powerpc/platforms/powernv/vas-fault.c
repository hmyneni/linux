// SPDX-License-Identifier: GPL-2.0+
/*
 * VAS Fault handling.
 * Copyright 2019, IBM Corporation
 */

#define pr_fmt(fmt) "vas: " fmt

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/mmu_context.h>
#include <asm/icswx.h>

#include "vas.h"

/*
 * The maximum FIFO size for fault window can be 8MB
 * (VAS_RX_FIFO_SIZE_MAX). Using 4MB FIFO since each VAS
 * instance will be having fault window.
 * 8MB FIFO can be used if expects more faults for each VAS
 * instance.
 */
#define VAS_FAULT_WIN_FIFO_SIZE	(4 << 20)

/*
 * Process CRBs that we receive on the fault window.
 */
irqreturn_t vas_fault_handler(int irq, void *data)
{
	struct vas_instance *vinst = data;
	struct coprocessor_request_block buf, *crb;
	struct vas_window *window;
	void *fifo;

	/*
	 * VAS can interrupt with multiple page faults. So process all
	 * valid CRBs within fault FIFO until reaches invalid CRB.
	 * NX updates nx_fault_stamp in CRB and pastes in fault FIFO.
	 * kernel retrives send window from parition send window ID
	 * (pswid) in nx_fault_stamp. So pswid should be non-zero and
	 * use this to check whether CRB is valid.
	 * After reading CRB entry, it is reset with 0's in fault FIFO.
	 *
	 * In case kernel receives another interrupt with different page
	 * fault and CRBs are processed by the previous handling, will be
	 * returned from this function when it sees invalid CRB (means 0's).
	 */
	do {
		mutex_lock(&vinst->mutex);

		/*
		 * Advance the fault fifo pointer to next CRB.
		 * Use CRB_SIZE rather than sizeof(*crb) since the latter is
		 * aligned to CRB_ALIGN (256) but the CRB written to by VAS is
		 * only CRB_SIZE in len.
		 */
		fifo = vinst->fault_fifo + (vinst->fault_crbs * CRB_SIZE);
		crb = fifo;

		/*
		 * NX pastes nx_fault_stamp in fault FIFO for each fault.
		 * So use pswid to check whether fault CRB is valid.
		 * pswid returned from NX will be in _be32, but just
		 * checking non-zero value to make sure the CRB is valid.
		 * Return if reached invalid CRB.
		 */
		if (!crb->stamp.nx.pswid) {
			mutex_unlock(&vinst->mutex);
			return IRQ_HANDLED;
		}

		vinst->fault_crbs++;
		if (vinst->fault_crbs == (vinst->fault_fifo_size / CRB_SIZE))
			vinst->fault_crbs = 0;

		crb = &buf;
		memcpy(crb, fifo, CRB_SIZE);
		memset(fifo, 0, CRB_SIZE);
		mutex_unlock(&vinst->mutex);

		pr_devel("VAS[%d] fault_fifo %p, fifo %p, fault_crbs %d\n",
				vinst->vas_id, vinst->fault_fifo, fifo,
				vinst->fault_crbs);

		window = vas_pswid_to_window(vinst,
				be32_to_cpu(crb->stamp.nx.pswid));

		if (IS_ERR(window)) {
			/*
			 * We got an interrupt about a specific send
			 * window but we can't find that window and we can't
			 * even clean it up (return credit).
			 * But we should not get here.
			 */
			pr_err("VAS[%d] fault_fifo %p, fifo %p, pswid 0x%x, fault_crbs %d bad CRB?\n",
				vinst->vas_id, vinst->fault_fifo, fifo,
				be32_to_cpu(crb->stamp.nx.pswid),
				vinst->fault_crbs);

			WARN_ON_ONCE(1);
			return IRQ_HANDLED;
		}

	} while (true);

	return IRQ_HANDLED;
}

/*
 * Fault window is opened per VAS instance. NX pastes fault CRB in fault
 * FIFO upon page faults.
 */
int vas_setup_fault_window(struct vas_instance *vinst)
{
	struct vas_rx_win_attr attr;

	vinst->fault_fifo_size = VAS_FAULT_WIN_FIFO_SIZE;
	vinst->fault_fifo = kzalloc(vinst->fault_fifo_size, GFP_KERNEL);
	if (!vinst->fault_fifo) {
		pr_err("Unable to alloc %d bytes for fault_fifo\n",
				vinst->fault_fifo_size);
		return -ENOMEM;
	}

	vas_init_rx_win_attr(&attr, VAS_COP_TYPE_FAULT);

	attr.rx_fifo_size = vinst->fault_fifo_size;
	attr.rx_fifo = vinst->fault_fifo;

	/*
	 * Max creds is based on number of CRBs can fit in the FIFO.
	 * (fault_fifo_size/CRB_SIZE). If 8MB FIFO is used, max creds
	 * will be 0xffff since the receive creds field is 16bits wide.
	 */
	attr.wcreds_max = vinst->fault_fifo_size / CRB_SIZE;
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
