/*
 * linux/drivers/mmc/card/nysa_sdio.c - SDIO interface to Nysa based FPGA images
 * Author: David McCoy
 * Created: August 7, 2015
 *
 * Based on drivers/mmc/card/sdio_uart.c - SDIO UART/GPS driver
 * by	Nicolas Pitre
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#include <linux/device.h>
#include <linux/compat.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>


#define UART_NR		8	/* Number of UARTs this driver can handle */
/*XXX I'll need to find a Major number of my own but right
 * now I'll just piggy back off of nysa_sdio for now
 */
#define SDIO_NYSA_MAJOR     153
#define N_SDIO_NYSA_MINORS  32
static DECLARE_BITMAP(minors, N_SDIO_NYSA_MINORS);

/* XXX NO BUENO!!! I NEED A VENDOR ID!! */
#define VENDOR_ID 0x00
#define DEVICE_ID 0x01





/* SDIO Nysa Structure -----------------------------------------------------*/
struct nysa_sdio_data {
  dev_t               devt;
	struct sdio_func	  *sdio_func;
  unsigned            users;
  u8                  *buffer;
	struct mutex		    buf_lock;
	struct list_head	  device_entry;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported Transfer Message");


/* FOPS --------------------------------------------------------------------*/
static ssize_t
nysa_sdio_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t			status = 0;
  //Read Here
  return status;
}

static ssize_t
nysa_sdio_write(struct file *filp, const char __user *buf,
    size_t count, loff_t *f_pos)

{
	ssize_t			status = 0;
  //Write Here
  return status;
}
static long
nysa_sdio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	ssize_t			status = 0;
  //IOCTL Here

	return status;
};

/*
static long
nysa_sdio_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return nysa_sdio_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
*/

static int nysa_sdio_open(struct inode *inode, struct file *filp)
{
	struct nysa_sdio_data	*nysa_sdio;
	int			status = -ENXIO;
  mutex_lock(&device_list_lock);
  list_for_each_entry(nysa_sdio, &device_list, device_entry) {
    if (nysa_sdio->devt == inode->i_rdev) {
      //Found
      status = 0;
      break;
    }
  }
  if (status == 0) {
    if (!nysa_sdio->buffer) {
      nysa_sdio->buffer = kmalloc(bufsiz, GFP_KERNEL);
      if (!nysa_sdio->buffer) {
        dev_dbg(&nysa_sdio->sdio_func->dev, "open/ENOMEM\n");
        status = -ENOMEM;
      }
    }
    if (status == 0) {
      nysa_sdio->users++;
      filp->private_data = nysa_sdio;
      nonseekable_open(inode, filp);
    }
  } else
    pr_debug("nysa_sdio: Nothing for minor %d\n", iminor(inode));
  
  mutex_unlock(&device_list_lock);
  return status;
};
static int nysa_sdio_release(struct inode *inode, struct file *filp)
{
  struct      nysa_sdio_data  *nysa_sdio = NULL;
	ssize_t			status = 0;

  mutex_lock(&device_list_lock);
  nysa_sdio = filp->private_data;
  filp->private_data = NULL;

  /* last close? */
  nysa_sdio->users--;
  if (nysa_sdio->users == 0) {
    int dofree;
    kfree(nysa_sdio->buffer);
    nysa_sdio->buffer = NULL;

    dofree = (nysa_sdio->sdio_func == NULL);
    if (dofree)
      kfree(nysa_sdio);
  }
  mutex_unlock(&device_list_lock);

  return status;
};


static const struct file_operations nysa_sdio_fops = {
  .write          = nysa_sdio_write,
  .read           = nysa_sdio_read,
  .unlocked_ioctl = nysa_sdio_ioctl,
  //.compat_ioctl   = nysa_sdio_compat_ioctl,
  .open           = nysa_sdio_open,
  .release        = nysa_sdio_release,
  .llseek         = no_llseek,
};

/* PROBE/REMOVE -------------------------------------------------------------*/
static struct class *nysa_sdio_class;

static int nysa_sdio_probe(struct sdio_func *func,
  const struct sdio_device_id *id)
{
  int status = 0;
  struct nysa_sdio_data *nysa_sdio_dev = NULL;
  unsigned long minor = 0;

  /* XXX: Need a Vendor ID!!!! */
  if (func->class   != SDIO_CLASS_NONE  ||
      func->vendor  != VENDOR_ID        ||
      func->device  != DEVICE_ID){
    pr_warning("SDIO Function %s: not SDIO_NYSA SDIO Device\n", sdio_func_id(func));
    return -ENOSYS;
  }
  pr_debug("SDIO Fuction %s: Found!\n", sdio_func_id(func));

  /* Allocate driver data */
  nysa_sdio_dev = kzalloc(sizeof(*nysa_sdio_dev), GFP_KERNEL); 
  if (!nysa_sdio_dev)
    return -ENOMEM;

  nysa_sdio_dev->sdio_func = func;

  mutex_init(&nysa_sdio_dev->buf_lock);
  INIT_LIST_HEAD(&nysa_sdio_dev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SDIO_NYSA_MINORS);
  if (minor < N_SDIO_NYSA_MINORS) {
    struct device *dev;
    nysa_sdio_dev->devt = MKDEV(SDIO_NYSA_MAJOR, minor);
    /* The device create took one more input variable to indicate the CS,
     * but the only
     */
    dev = device_create(nysa_sdio_class, &func->dev, nysa_sdio_dev->devt,
            nysa_sdio_dev, "nysa_%s.%d",
            //func->card->host->index, func->num);
            mmc_hostname(func->card->host), func->num);
    //status = PTR_ERR_OR_ZERO(dev);
    if (IS_ERR(dev)) {
      return dev;
    }
    else {
      return 0;
    }
  } else {
    //dev_dbg(&func->dev, "no minor number available!\n");
    pr_warning("No minor numbers available\n");
  }
  if (status == 0)
    sdio_set_drvdata(func, nysa_sdio_dev); 
  else
    kfree(nysa_sdio_dev);

  /* Initialize the Driver Data */
  return status;
}

static void nysa_sdio_remove(struct sdio_func *func)
{
  struct nysa_sdio_data * nysa_sdio = sdio_get_drvdata(func);

  nysa_sdio->sdio_func = NULL;
  
  /* Prevent new opennings */
  mutex_lock(&device_list_lock);
  list_del(&nysa_sdio->device_entry);
  device_destroy(nysa_sdio_class, nysa_sdio->devt);
  clear_bit(MINOR(nysa_sdio->devt), minors);
  if (nysa_sdio->users == 0)
    kfree(nysa_sdio);
  mutex_unlock(&device_list_lock);
}

/* MODULE BOILER PLATE ------------------------------------------------------*/

static const struct sdio_device_id nysa_sdio_ids[] = {
	{ SDIO_DEVICE(VENDOR_ID, DEVICE_ID) },
  { /* end: all zeros */              },
};

MODULE_DEVICE_TABLE(sdio, nysa_sdio_ids);

static struct sdio_driver nysa_sdio_driver = {
  .probe    = nysa_sdio_probe,
  .remove   = nysa_sdio_remove,
  .name     = "nysa_sdio",
  .id_table = nysa_sdio_ids,
};


static int __init nysa_sdio_init(void)
{

  int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

  /* No more than 256 minor numbers */
	BUILD_BUG_ON(N_SDIO_NYSA_MINORS > 256);
  status = register_chrdev(SDIO_NYSA_MAJOR, "nysa_sdio", &nysa_sdio_fops);
  if (status < 0)
    return status;

  nysa_sdio_class = class_create(THIS_MODULE, "nysa_sdio");
  if (IS_ERR(nysa_sdio_class)) {
    unregister_chrdev(SDIO_NYSA_MAJOR, nysa_sdio_driver.name);
    return PTR_ERR(nysa_sdio_class);
  }
  status = sdio_register_driver(&nysa_sdio_driver);
  if (status < 0) {
    class_destroy(nysa_sdio_class);
    unregister_chrdev(SDIO_NYSA_MAJOR, nysa_sdio_driver.name);
  }
  return status;
}

static void __exit nysa_sdio_exit(void)
{
	sdio_unregister_driver(&nysa_sdio_driver);
  class_destroy(nysa_sdio_class);
  unregister_chrdev(SDIO_NYSA_MAJOR, nysa_sdio_driver.name);
}

module_init(nysa_sdio_init);
module_exit(nysa_sdio_exit);

MODULE_AUTHOR("David McCoy <dave.mccoy@cospandesign.com");
MODULE_DESCRIPTION("User mode SDIO device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("sdio:nysa_sdio");

