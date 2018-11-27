/*
 * Driver for IBM PowerNV gzip compression accelerator
 *
 * Copyright (C) 2015 Dan Streetman, IBM Corp
 *
 * This program is free sogzipare; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Sogzipare Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "nx-842.h"

#include <linux/timer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/switch_to.h>
#include <asm/prom.h>
#include <asm/vas.h>
#include <asm/reg.h>
#include <asm/opal-api.h>
#include <asm/opal.h>
#include <asm/mmu_context.h>
#include <uapi/asm/vas-api.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Haren Myneni <haren@us.ibm.com>");
MODULE_DESCRIPTION("gzip H/W Compression driver for IBM PowerNV processors");
MODULE_ALIAS_CRYPTO("gzip");
MODULE_ALIAS_CRYPTO("gzip-nx");

/* # of requests allowed per RxFIFO at a time. 0 for unlimited */
#define MAX_CREDITS_PER_RXFIFO	(1024)

struct nxgzip_coproc {
	unsigned int chip_id;
	unsigned int ct;
	unsigned int ci;	/* Coprocessor instance, used with icswx */
	struct {
		struct vas_window *rxwin;
		int id;
	} vas;
	struct list_head list;
};

/*
 * The driver creates the device /dev/crypto/nx-gzip that can be
 * used as follows:
 *
 *	fd = open("/dev/crypto/nx-gzip", O_RDWR);
 *	rc = ioctl(fd, VAS_GZIP_TX_WIN_OPEN, &attr);
 *	paste_addr = mmap(NULL, PAGE_SIZE, prot, MAP_SHARED, fd, 0ULL).
 *	vas_copy(&crb, 0, 1);
 *	vas_paste(paste_addr, 0, 1);
 *
 * where "vas_copy" and "vas_paste" are defined in copy-paste.h.
 */

static char	*nxgzip_dev_name = "nx-gzip";
static atomic_t	nxgzip_instid = ATOMIC_INIT(0);

/*
 * Wrapper object for the nx-gzip device - there is just one instance of
 * this node for the whole system.
 */
struct nxgzip_dev {
	struct cdev cdev;
	struct device *device;
	char *name;
	dev_t devt;
	struct class *class;
} nxgzip_device;

/*
 * One instance per open of a nx-gzip device. Each nxgzip_instance is
 * associated with a VAS window after the caller issues VAS_FTW_SETUP
 * ioctl.
 */
struct nxgzip_instance {
	int id;
	struct vas_window *txwin;
};

static LIST_HEAD(nxgzip_coprocs);

static char *nxgzip_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "crypto/%s", dev_name(dev));
}

static int nxgzip_open(struct inode *inode, struct file *fp)
{
	struct nxgzip_instance *instance;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance)
		return -ENOMEM;

	instance->id = atomic_inc_return(&nxgzip_instid);

	fp->private_data = instance;
	return 0;
}

static int validate_gzip_setup_attr(struct vas_tx_win_open_attr *uattr)
{
	if (uattr->version != 1 || uattr->reserved1 ||
				   uattr->reserved2) {
		pr_err("%s: version %d, reserved 0x%llx, 0x%llx\n",
				__func__, uattr->version,
				uattr->reserved1, uattr->reserved2);
		return -EINVAL;
	}

	if (uattr->flags & VAS_FLAGS_PIN_WINDOW && !capable(CAP_SYS_ADMIN))
		return -EPERM;

	return 0;
}

/*
 * Unsupported ioctl: we continue to use struct vas_gzip_setup_attr for
 * 	this ioctl also.
 */
static int nx_ioc_gzip_tx_win_open(struct file *fp, unsigned long arg)
{
	int rc, vasid, cop;
	struct vas_tx_win_attr txattr;
	struct vas_tx_win_open_attr uattr;
	void __user *uptr = (void *)arg;
	struct vas_window *txwin;
	struct nxgzip_instance *nxti = fp->private_data;
	struct mm_struct *mm = current->mm;

	if (!mm || !nxti)
		return -EINVAL;

	if (nxti->txwin)
		return -EEXIST;

	rc = copy_from_user(&uattr, uptr, sizeof(uattr));
	if (rc) {
		pr_err("%s(): copy_from_user() returns %d\n", __func__, rc);
		return -EFAULT;
	}

	rc = validate_gzip_setup_attr(&uattr);
	if (rc)
		return rc;

	if (uattr.flags & VAS_FLAGS_HIGH_PRI)
		cop = VAS_COP_TYPE_GZIP_HIPRI;
	else
		cop = VAS_COP_TYPE_GZIP;

	vasid = uattr.vas_id;

	vas_init_tx_win_attr(&txattr, cop);

	txattr.lpid = mfspr(SPRN_LPID);
	txattr.pidr = mfspr(SPRN_PID);
	txattr.pid = task_pid_nr(current);
	txattr.user_win = true;
	txattr.rsvd_txbuf_count = false;
	txattr.tc_mode = uattr.tc_mode;
	txattr.pswid = false;
	txattr.wcreds_max = 1024;

	if (uattr.flags & VAS_FLAGS_PIN_WINDOW)
		txattr.pin_win = true;

	pr_devel("Pid %d: Opening txwin, cop %d, PIDR %ld\n", txattr.pidr,
				cop, mfspr(SPRN_PID));

	txwin = vas_tx_win_open(vasid, cop, &txattr);
	if (IS_ERR(txwin)) {
		pr_devel("%s() vas_tx_win_open() failed, %ld\n", __func__,
					PTR_ERR(txwin));
		return PTR_ERR(txwin);
	}

	nxti->txwin = txwin;

	mm_context_add_copro(mm);

	return 0;
}

static int nxgzip_release(struct inode *inode, struct file *fp)
{
	struct nxgzip_instance *instance;

	instance = fp->private_data;

	if (instance && instance->txwin) {
		vas_win_close(instance->txwin);
		instance->txwin = NULL;

		if (current->mm)
			mm_context_remove_copro(current->mm);
	}

	/*
	 * TODO We don't know here if user has other receive windows
	 *      open, so we can't really call clear_thread_tidr().
	 *      So, once the process calls set_thread_tidr(), the
	 *      TIDR value sticks around until process exits, resulting
	 *      in an extra copy in restore_sprs().
	 */

	kfree(instance);
	fp->private_data = NULL;
	atomic_dec(&nxgzip_instid);

	return 0;
}

static int nxgzip_mmap(struct file *fp, struct vm_area_struct *vma)
{
	int rc;
	pgprot_t prot;
	u64 paste_addr;
	unsigned long pfn;
	struct nxgzip_instance *instance = fp->private_data;

	if ((vma->vm_end - vma->vm_start) > PAGE_SIZE) {
		pr_debug("%s(): size 0x%zx, PAGE_SIZE 0x%zx\n", __func__,
				(vma->vm_end - vma->vm_start), PAGE_SIZE);
		return -EINVAL;
	}

	/* Ensure instance has an open send window */
	if (!instance->txwin) {
		pr_debug("%s(): No send window open?\n", __func__);
		return -EINVAL;
	}

	paste_addr = vas_win_paste_addr(instance->txwin);
	pfn = paste_addr >> PAGE_SHIFT;

	/* flags, page_prot from cxl_mmap(), except we want cachable */
	vma->vm_flags |= VM_IO | VM_PFNMAP;
	vma->vm_page_prot = pgprot_cached(vma->vm_page_prot);

	prot = __pgprot(pgprot_val(vma->vm_page_prot) | _PAGE_DIRTY);

	rc = remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
			vma->vm_end - vma->vm_start, prot);

	pr_devel("%s(): paste addr %llx at %lx, rc %d\n", __func__,
			paste_addr, vma->vm_start, rc);

	return rc;
}

static long nxgzip_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case VAS_GZIP_TX_WIN_OPEN:
		return nx_ioc_gzip_tx_win_open(fp, arg);

	default:
		return -EINVAL;
	}
}

const struct file_operations nxgzip_fops = {
	.owner = THIS_MODULE,
	.open = nxgzip_open,
	.release = nxgzip_release,
	.mmap = nxgzip_mmap,
	.unlocked_ioctl = nxgzip_ioctl,
};

int nxgzip_device_create(void)
{
	int rc = -EINVAL;
	dev_t devno;

	rc = alloc_chrdev_region(&nxgzip_device.devt, 1, 1, "nx-gzip");
	if (rc) {
		pr_err("Unable to allocate nxgzip major number: %i\n", rc);
		return rc;
	}

	pr_devel("NX-GZIP device allocated, dev [%i,%i]\n",
			MAJOR(nxgzip_device.devt), MINOR(nxgzip_device.devt));

	nxgzip_device.class = class_create(THIS_MODULE, "nxgzip");
	if (IS_ERR(nxgzip_device.class)) {
		pr_debug("Unable to create NX-GZIP class\n");
		rc = PTR_ERR(nxgzip_device.class);
		goto err;
	}
	nxgzip_device.class->devnode = nxgzip_devnode;

	cdev_init(&nxgzip_device.cdev, &nxgzip_fops);

	devno = MKDEV(MAJOR(nxgzip_device.devt), 0);
	if (cdev_add(&nxgzip_device.cdev, devno, 1)) {
		pr_debug("NX-GZIP: cdev_add() failed\n");
		goto err;
	}

	nxgzip_device.device = device_create(nxgzip_device.class, NULL,
			devno, NULL, nxgzip_dev_name, MINOR(devno));
	if (IS_ERR(nxgzip_device.device)) {
		pr_debug("Unable to create nxgzip-%d\n", MINOR(devno));
		goto err;
	}

	pr_devel("%s: Added dev [%d,%d]\n", __func__, MAJOR(devno),
			MINOR(devno));

	return 0;

err:
	unregister_chrdev_region(nxgzip_device.devt, 1);
	return rc;
}

void nxgzip_device_delete(void)
{
	dev_t devno;

	cdev_del(&nxgzip_device.cdev);
	devno = MKDEV(MAJOR(nxgzip_device.devt), MINOR(nxgzip_device.devt));
	device_destroy(nxgzip_device.class, devno);

	class_destroy(nxgzip_device.class);
	unregister_chrdev_region(nxgzip_device.devt, 1);
}

static inline void nxgzip_add_coprocs_list(struct nxgzip_coproc *coproc,
					int chipid)
{
	coproc->chip_id = chipid;
	INIT_LIST_HEAD(&coproc->list);
	list_add(&coproc->list, &nxgzip_coprocs);
}

static int __init vas_cfg_coproc_info(struct device_node *dn, int chip_id,
					int vasid, int *ct)
{
	struct vas_window *rxwin = NULL;
	struct vas_rx_win_attr rxattr;
	struct nxgzip_coproc *coproc;
	u32 lpid, pid, tid, fifo_size;
	u64 rx_fifo;
	const char *priority;
	int ret;

	ret = of_property_read_u64(dn, "rx-fifo-address", &rx_fifo);
	if (ret) {
		pr_err("Missing rx-fifo-address property\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "rx-fifo-size", &fifo_size);
	if (ret) {
		pr_err("Missing rx-fifo-size property\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "lpid", &lpid);
	if (ret) {
		pr_err("Missing lpid property\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "pid", &pid);
	if (ret) {
		pr_err("Missing pid property\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "tid", &tid);
	if (ret) {
		pr_err("Missing tid property\n");
		return ret;
	}

	ret = of_property_read_string(dn, "priority", &priority);
	if (ret) {
		pr_err("Missing priority property\n");
		return ret;
	}

	coproc = kzalloc(sizeof(*coproc), GFP_KERNEL);
	if (!coproc)
		return -ENOMEM;

	if (!strcmp(priority, "High"))
		coproc->ct = VAS_COP_TYPE_GZIP_HIPRI;
	else if (!strcmp(priority, "Normal"))
		coproc->ct = VAS_COP_TYPE_GZIP;
	else {
		pr_err("Invalid RxFIFO priority value\n");
		ret =  -EINVAL;
		goto err_out;
	}

	vas_init_rx_win_attr(&rxattr, coproc->ct);
	rxattr.rx_fifo = (void *)rx_fifo;
	rxattr.rx_fifo_size = fifo_size;
	rxattr.lnotify_lpid = lpid;
	rxattr.lnotify_pid = pid;
	rxattr.lnotify_tid = tid;
	rxattr.wcreds_max = MAX_CREDITS_PER_RXFIFO;

	/*
	 * Open a VAS receice window which is used to configure RxFIFO
	 * for NX.
	 */
	rxwin = vas_rx_win_open(vasid, coproc->ct, &rxattr);
	if (IS_ERR(rxwin)) {
		ret = PTR_ERR(rxwin);
		pr_err("setting RxFIFO with VAS failed: %d\n",
			ret);
		goto err_out;
	}

	coproc->vas.rxwin = rxwin;
	coproc->vas.id = vasid;
	nxgzip_add_coprocs_list(coproc, chip_id);

	/*
	 * (lpid, pid, tid) combination has to be unique for each
	 * coprocessor instance in the system. So to make it
	 * unique, skiboot uses coprocessor type such as 842 or
	 * GZIP for pid and provides this value to kernel in pid
	 * device-tree property.
	 */
	*ct = pid;

	return 0;

err_out:
	kfree(coproc);
	return ret;
}


static int __init nxgzip_powernv_probe_vas(struct device_node *pn)
{
	struct device_node *dn;
	int chip_id, vasid, ct, ret = 0;
	int nx_fifo_found = 0;

	chip_id = of_get_ibm_chip_id(pn);
	if (chip_id < 0) {
		pr_err("ibm,chip-id missing\n");
		return -EINVAL;
	}

	vasid = chip_to_vas_id(chip_id);
	if (vasid < 0) {
		pr_err("Unable to map chip_id %d to vasid\n", chip_id);
		return -EINVAL;
	}

	for_each_child_of_node(pn, dn) {
		if (of_device_is_compatible(dn, "ibm,p9-nx-gzip")) {
			ret = vas_cfg_coproc_info(dn, chip_id, vasid, &ct);
			if (ret) {
				of_node_put(dn);
				return ret;
			}
			nx_fifo_found++;
		}
	}

	if (!nx_fifo_found) {
		pr_err("NXgzip FIFO nodes are missing\n");
		return -EINVAL;
	}

	/*
	 * Initialize NX instance for both high and normal priority FIFOs.
	 */
	if (opal_check_token(OPAL_NX_COPROC_INIT)) {
		ret = opal_nx_coproc_init(chip_id, ct);
		if (ret) {
			pr_err("Failed to initialize NX for chip(%d): %d\n",
				chip_id, ret);
			ret = opal_error_code(ret);
		}
	} else
		pr_warn("Firmware doesn't support NX initialization\n");

	return ret;
}

static void nxgzip_delete_coprocs(void)
{
	struct nxgzip_coproc *coproc, *n;

	list_for_each_entry_safe(coproc, n, &nxgzip_coprocs, list) {
		if (coproc->vas.rxwin)
			vas_win_close(coproc->vas.rxwin);

		list_del(&coproc->list);
		kfree(coproc);
	}
}

static __init int nxgzip_powernv_init(void)
{
	struct device_node *dn;
	int ret;

	for_each_compatible_node(dn, NULL, "ibm,power9-nx") {
		ret = nxgzip_powernv_probe_vas(dn);
		if (ret) {
			nxgzip_delete_coprocs();
			return ret;
		}
	}

	if (list_empty(&nxgzip_coprocs)) {
		pr_err("NXgzip: No devices found\n");
			return -ENODEV;
	}

	ret = nxgzip_device_create();

	return ret;
}
module_init(nxgzip_powernv_init);

static void __exit nxgzip_powernv_exit(void)
{
	nxgzip_device_delete();
	nxgzip_delete_coprocs();
}
module_exit(nxgzip_powernv_exit);
