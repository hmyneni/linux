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
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/sched/signal.h>
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

void vas_wakeup_fault_handler(int virq, void *arg)
{
	struct vas_instance *vinst = arg;

	atomic_inc(&vinst->pending_fault);
	wake_up(&vinst->fault_wq);
}

static void dump_crb(struct coprocessor_request_block *crb)
{
	struct data_descriptor_entry *dde;
	struct nx_fault_stamp *nx;

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
		be32_to_cpu(nx->pswid),
		be64_to_cpu(crb->stamp.nx.fault_storage_addr),
		nx->flags, be32_to_cpu(nx->fault_status));
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
static void update_csb(struct vas_window *window,
			struct coprocessor_request_block *crb)
{
	int rc;
	struct pid *pid;
	void __user *csb_addr;
	struct task_struct *tsk;
	struct siginfo info;
	struct coprocessor_status_block csb;

	/*
	 * NX user space windows can not be opened for task->mm=NULL
	 * and faults will not be generated for kernel requests.
	 */
	if (!window->mm || !window->user_win)
		return;

	csb_addr = (void *)be64_to_cpu(crb->csb_addr);

	csb.cc = CSB_CC_TRANSLATION;
	csb.ce = CSB_CE_TERMINATION;
	csb.cs = 0;
	csb.count = 0;

	/*
	 * Returns the fault address in CPU format since it is passed with
	 * signal. But if the user space expects BE format, need changes.
	 * i.e either kernel (here) or user should convert to CPU format.
	 * Not both!
	 */
	csb.address = be64_to_cpu(crb->stamp.nx.fault_storage_addr);
	csb.flags = 0;

	pid = window->pid;
	tsk = get_pid_task(pid, PIDTYPE_PID);
	/*
	 * Send window will be closed after processing all NX requests
	 * and process exits after closing all windows. In multi-thread
	 * applications, thread may not exists, but does not close FD
	 * (means send window) upon exit. Parent thread (tgid) can use
	 * and close the window later.
	 * pid and mm references are taken when window is opened by
	 * process (pid). So tgid is used only when child thread opens
	 * a window and exits without closing it in multithread tasks.
	 */
	if (!tsk) {
		pid = window->tgid;
		tsk = get_pid_task(pid, PIDTYPE_PID);
		/*
		 * Parent thread will be closing window during its exit.
		 * So should not get here.
		 */
		if (!tsk)
			return;
	}

	/* Nothing to do if the task is exiting. */
	if (tsk->flags & PF_EXITING) {
		put_task_struct(tsk);
		return;
	}

	use_mm(window->mm);
	rc = copy_to_user(csb_addr, &csb, sizeof(csb));
	/*
	 * User space polls on csb.flags (first byte). So add barrier
	 * then copy first byte with csb flags update.
	 */
	smp_mb();
	if (!rc) {
		csb.flags = CSB_V;
		rc = copy_to_user(csb_addr, &csb, sizeof(u8));
	}
	unuse_mm(window->mm);
	put_task_struct(tsk);

	/* Success */
	if (!rc)
		return;

	pr_err("VAS: Invalid CSB address 0x%p signalling pid(%d). Window pid= %d, tgid = %d, window ID = %d\n",
			csb_addr, pid_vnr(pid), vas_window_pid(window),
			pid_vnr(window->tgid), window->winid);

	memset(&info, 0, sizeof(info));
	info.si_signo = SIGSEGV;
	info.si_errno = EFAULT;
	info.si_code = SEGV_MAPERR;
	info.si_addr = csb_addr;

	/*
	 * process will be polling on csb.flags after request is sent to
	 * NX. So generally CSB update should not fail except when an
	 * application does not follow the process properly. So an error
	 * message will be displayed and leave it to user space whether
	 * to ignore or handle this signal.
	 */
	rcu_read_lock();
	rc = kill_pid_info(SIGSEGV, &info, pid);
	rcu_read_unlock();

	pr_devel("%s(): pid %d kill_proc_info() rc %d\n", __func__,
			pid_vnr(pid), rc);
}

static void dump_fifo(struct vas_instance *vinst, void *entry)
{
	int i;
	unsigned long *fifo = entry;

	pr_err("Fault fifo size %d, max crbs %d, crb size %lu\n",
			vinst->fault_fifo_size,
			vinst->fault_fifo_size / CRB_SIZE,
			sizeof(struct coprocessor_request_block));

	pr_err("Fault FIFO Entry Dump:\n");
	for (i = 0; i < CRB_SIZE; i += 4, fifo += 4) {
		pr_err("[%.3d, %p]: 0x%.16lx 0x%.16lx 0x%.16lx 0x%.16lx\n",
			i, fifo, *fifo, *(fifo+1), *(fifo+2), *(fifo+3));
	}
}

/*
 * Process CRBs that we receive on the fault window.
 */
static void process_fault_crbs(struct vas_instance *vinst)
{
	struct coprocessor_request_block buf;
	struct coprocessor_request_block *crb, *entry;
	struct vas_window *window;
	void *fifo;

	crb = &buf;
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
		entry = fifo;

		/*
		 * NX pastes nx_fault_stamp in fault FIFO for each fault.
		 * So use pswid to check whether fault CRB is valid.
		 * Also use csb_addr tio make overcome FIFO race between
		 * NX and kernel.
		 * Return if reached invalid CRB.
		 */
		if (entry->stamp.nx.pswid == FIFO_ENTRY_INVALID ||
				entry->csb_addr == CSB_ADDR_INVALID) {
			mutex_unlock(&vinst->mutex);
			return;
		}

		vinst->fault_crbs++;
		if (vinst->fault_crbs == (vinst->fault_fifo_size / CRB_SIZE))
			vinst->fault_crbs = 0;

		memcpy(crb, fifo, CRB_SIZE);
		/*
		 * Invalidate this entry in fault FIFO
		 */
		entry->stamp.nx.pswid = FIFO_ENTRY_INVALID;
		entry->csb_addr = CSB_ADDR_INVALID;
		/*
		 * Return credit for the fault window.
		 */
		vas_return_credit(vinst->fault_win, 0);
		mutex_unlock(&vinst->mutex);

		pr_devel("VAS[%d] fault_fifo %p, fifo %p, fault_crbs %d\n",
				vinst->vas_id, vinst->fault_fifo, fifo,
				vinst->fault_crbs);

		dump_crb(crb);
		window = vas_pswid_to_window(vinst,
				be32_to_cpu(crb->stamp.nx.pswid));

		if (IS_ERR(window)) {
			/*
			 * We got an interrupt about a specific send
			 * window but we can't find that window and we can't
			 * even clean it up (return credit).
			 * But we should not get here.
			 */
			dump_fifo(vinst, (void *)crb);
			pr_err("VAS[%d] fault_fifo %p, fifo %p, pswid 0x%x, fault_crbs %d bad CRB?\n",
				vinst->vas_id, vinst->fault_fifo, fifo,
				be32_to_cpu(crb->stamp.nx.pswid),
				vinst->fault_crbs);

			WARN_ON_ONCE(1);
			return;
		}

		update_csb(window, crb);
		/*
		 * Return credit for send window after processing
		 * fault CRB.
		 */
		vas_return_credit(window, 1);
	} while (true);

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
					atomic_read(&vinst->pending_fault) ||
					kthread_should_stop());

		if (kthread_should_stop())
			break;

		atomic_dec(&vinst->pending_fault);
		process_fault_crbs(vinst);

	} while (!kthread_should_stop());

	return 0;
}

/*
 * Create a thread that processes the fault CRBs.
 */
int vas_setup_fault_handler(struct vas_instance *vinst)
{
	vinst->fault_handler = kthread_run(fault_handler_func, (void *)vinst,
				"vas%u-irq%u", vinst->vas_id, vinst->virq);

	if (IS_ERR(vinst->fault_handler))
		return PTR_ERR(vinst->fault_handler);

	return 0;
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

	/*
	 * Invalidate all CRB entries initially. If NX generates continuous
	 * faults, VAS handling thread may continue reading entries while
	 * NX paste entries in to fault FIFO. We use only pswid and csb_addr
	 * to process fault CRB. So to overcome any race issue between NX
	 * and kernel, work-around is to invalidate with 0xff.
	 */
	memset(vinst->fault_fifo, FIFO_ENTRY_INVALID, vinst->fault_fifo_size);
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
