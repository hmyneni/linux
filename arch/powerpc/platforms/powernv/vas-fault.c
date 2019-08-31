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
#include <linux/sched/signal.h>
#include <linux/mmu_context.h>
#include <asm/icswx.h>
#include <asm/opal-api.h>
#include <asm/opal.h>

#include "vas.h"

/*
 * TODO: These need tuning
 */
#define VAS_FAULT_WIN_FIFO_SIZE		(64 << 10)
#define VAS_FAULT_WIN_WCREDS		64

struct task_struct *fault_handler;

void vas_wakeup_fault_handler(int virq, void *arg)
{
	struct vas_instance *vinst = arg;

	atomic_inc(&vinst->pending_crbs);

	atomic_inc(&vinst->pending_faults);

	wake_up(&vinst->fault_wq);
}

static void dump_crb(struct coprocessor_request_block *crb)
{
	struct data_descriptor_entry *dde;
	struct nx_fault_stamp *nx;

	/* TODO Convert to CPU format before print? */

	dde = &crb->source;
	pr_devel("SrcDDE: addr 0x%llx, len %d, count %d, idx %d, flags %d\n",
			be64_to_cpu(dde->address), be32_to_cpu(dde->length),
			dde->count, dde->index, dde->flags);

	dde = &crb->target;
	pr_devel("TgtDDE: addr 0x%llx, len %d, count %d, idx %d, flags %d\n",
			be64_to_cpu(dde->address), be32_to_cpu(dde->length),
			dde->count, dde->index, dde->flags);

	nx = &crb->stamp.nx;
	pr_devel("NX Stamp: PSWID 0x%x, FSA 0x%llx, flags 0x%x, FS 0x%x\n",
			be32_to_cpu(nx->pswid), crb_nx_fault_addr(crb),
			nx->flags, be32_to_cpu(nx->fault_status));
}

/*
 * Check if the fault occurred in the CSB itself. Return true if so, false
 * otherwise.
 */
static bool fault_in_csb(struct coprocessor_request_block *crb)
{
	u64 fault_addr, csb_start, csb_end;

	fault_addr = crb_nx_fault_addr(crb);
	csb_start = crb_csb_addr(crb);
	csb_end = csb_start + sizeof(struct coprocessor_status_block);

	if (fault_addr >= csb_start && fault_addr < csb_end) {
		pr_err("CSB Fault: csb start/end 0x%llx/0x%llx, addr 0x%llx\n",
				csb_start, csb_end, fault_addr);
		return true;
	}

	return false;
}

static void notify_process(pid_t pid, u64 fault_addr)
{
	int rc;
	struct siginfo info;

	memset(&info, 0, sizeof(info));

	info.si_signo = SIGSEGV;
	info.si_errno = 0;	/* TODO */
	info.si_code = 0;	/* TODO */

	info.si_addr = (void *)fault_addr;

	rcu_read_lock();
	rc = kill_pid_info(SIGSEGV, &info, find_vpid(pid));
	rcu_read_unlock();

	pr_devel("%s(): pid %d kill_proc_info() rc %d\n", __func__, pid, rc);
}

/*
 * Update the CSB to indicate a translation error.
 *
 * If the fault is in the CSB address itself or if we are unable to
 * update the CSB, send a signal to the process, because we have no
 * other way of notifying the user process.
 *
 * Remaining settings in the CSB are based on wait_for_csb() of
 * NX-GZIP.
 */
static void update_csb(int pid, struct coprocessor_request_block *crb)
{
	int rc;
	void __user *csb_addr;
	struct task_struct *tsk;
	struct coprocessor_status_block csb;

	if (fault_in_csb(crb))
		goto notify;

	csb_addr = (void *)__be64_to_cpu(crb->csb_addr);

	csb.cc = CSB_CC_TRANSLATION;
	csb.ce = CSB_CE_TERMINATION;
	csb.cs = 0;
	csb.count = 0;

	/*
	 * Returns the fault address in CPU format since it is passed with
	 * signal. But is the user space expects BE format, need changes.
	 * i.e either kernel (here) or user should convert to CPU format.
	 * Not both!
	 */
	csb.address = crb_nx_fault_addr(crb);
	csb.flags = CSB_V;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		/*
		 * vas_win_close() waits for any pending CRBs and pending
		 * send window credits. In case of this fault CRB, the send
		 * credit is not yet returned. So we should NOT end up with a
		 * non-existent task for this fault CRB.
		 */
		WARN_ON_ONCE(!tsk);
		rcu_read_unlock();
		return;
	}

	if (tsk->flags & PF_EXITING) {
		rcu_read_unlock();
		return;
	}

	get_task_struct(tsk);
	rcu_read_unlock();

	use_mm(tsk->mm);

	rc = copy_to_user(csb_addr, &csb, sizeof(csb));

	unuse_mm(tsk->mm);
	put_task_struct(tsk);

	if (rc) {
		pr_err("CSB: Error updating CSB address 0x%p signalling\n",
				csb_addr);
		goto notify;
	}

	return;

notify:
	notify_process(pid, crb_nx_fault_addr(crb));
}

static void dump_fifo(struct vas_instance *vinst)
{
	int i;
	unsigned long *fifo = vinst->fault_fifo;

	pr_err("Fault fifo size %d, max crbs %d, crb size %lu\n",
			vinst->fault_fifo_size,
			vinst->fault_fifo_size / CRB_SIZE,
			sizeof(struct coprocessor_request_block));

	pr_err("Fault FIFO Dump:\n");
	for (i = 0; i < 64; i+=4, fifo+=4) {
		pr_err("[%.3d, %p]: 0x%.16lx 0x%.16lx 0x%.16lx 0x%.16lx\n",
			i, fifo, *fifo, *(fifo+1), *(fifo+2), *(fifo+3));
	}
}

/*
 * Process a CRB that we receive on the fault window.
 */
static void process_fault_crb(struct vas_instance *vinst)
{
	void *fifo;
	struct vas_window *window;
	struct coprocessor_request_block buf;
	struct coprocessor_request_block *crb;

	if (!atomic_read(&vinst->pending_crbs))
		return;

	crb = &buf;

	/*
	 * Advance the fault fifo pointer to next CRB.
	 * Use CRB_SIZE rather than sizeof(*crb) since the latter is
	 * aligned to CRB_ALIGN (256) but the CRB written to by VAS is
	 * only CRB_SIZE in len.
	 */
	fifo = vinst->fault_fifo + atomic_read(&vinst->fault_crbs) * CRB_SIZE;

	pr_devel("VAS[%d] fault_fifo %p, fifo %p, faults %d pending %d\n",
			vinst->vas_id, vinst->fault_fifo, fifo,
			atomic_read(&vinst->fault_crbs),
			atomic_read(&vinst->pending_faults));

	memcpy(crb, fifo, CRB_SIZE);
	memset(fifo, 0, CRB_SIZE);

	/*
	 * TODO: If we see any race issues, may usse a mutex instead of
	 * atomic here, since we have to wrap the fault_crbs back to 0.
	 */
	atomic_inc(&vinst->fault_crbs);
	if (atomic_read(&vinst->fault_crbs) == vinst->fault_fifo_size/CRB_SIZE)
		atomic_set(&vinst->fault_crbs, 0);

	atomic_dec(&vinst->pending_crbs);
	atomic_dec(&vinst->pending_faults);

	dump_crb(crb);

	window = vas_pswid_to_window(vinst, crb_nx_pswid(crb));

	if (IS_ERR(window)) {
		/*
		 * What now? We got an interrupt about a specific send
		 * window but we can't find that window and we can't
		 * even clean it up (return credit).
		 * But we should not get here.
		 */
		dump_fifo(vinst);
		pr_err("VAS[%d] fault_fifo %p, fifo %p, pswid 0x%x faults %d, "
				"pending %d bad CRB?\n", vinst->vas_id,
				vinst->fault_fifo, fifo, crb_nx_pswid(crb),
				atomic_read(&vinst->fault_crbs),
				atomic_read(&vinst->pending_faults));

		WARN_ON_ONCE(1);
		return;
	}

	update_csb(vas_window_pid(window), crb);

	vas_return_credit(window, true);

	return;
}

/*
 * VAS Fault handler thread. One thread for all instances of VAS.
 *
 * Process CRBs posted on any instance of VAS.
 */
static int fault_handler_func(void *arg)
{
	struct vas_instance *vinst = (struct vas_instance *)arg;

	do {
		if (signal_pending(current))
			flush_signals(current);

		wait_event_interruptible(vinst->fault_wq,
				atomic_read(&vinst->pending_faults) ||
				kthread_should_stop());

		if (kthread_should_stop())
			break;

		process_fault_crb(vinst);

	} while (!kthread_should_stop());

	return 0;
}

/*
 * Create a thread that processes the fault CRBs.
 */
int vas_setup_fault_handler(struct vas_instance *vinst)
{
	vinst->fault_handler = kthread_run(fault_handler_func, (void *)vinst,
			"vas-fault-%u", vinst->vas_id);

	if (IS_ERR(vinst->fault_handler))
		return PTR_ERR(vinst->fault_handler);

	return 0;
}

static irqreturn_t vas_irq_handler(int virq, void *data)
{
	struct vas_instance *vinst = data;

	pr_devel("VAS %d: virq %d\n", vinst->vas_id, virq);
	vas_wakeup_fault_handler(virq, data);

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

void vas_cleanup_fault_handler(struct vas_instance *vinst)
{
	kthread_stop(vinst->fault_handler);
}
#endif
