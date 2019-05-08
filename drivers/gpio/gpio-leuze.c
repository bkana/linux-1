#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/signal.h>
#include <asm-generic/io.h>
#include <asm-generic/current.h>
#include <linux/sched/signal.h>
#include <linux/rcupdate.h>
#include <linux/io.h>
#define SIGLEUZE 44
#define REG_CURRENT_TASK _IOW('a','a', int32_t* )
#define IOCTL_SET_VARIABLES 0
#define START_ADDR 0x48200270
/* GPIO Port */
static unsigned int gpioSync = 20;
static unsigned int irqNumber;

/* Signaling to Application */
static struct task_struct *task = NULL;

int32_t value = 0;
dev_t dev = 0;
static struct class *dev_class;
static struct cdev leuze_cdev;
pid_t pid;
void __iomem *mem;
extern ktime_t tick_period;

static int leuze_open(struct inode *inode, struct file *file);
static int leuze_release(struct inode *inode, struct file *file);
static long leuze_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static irq_handler_t leuze_irq_handler(unsigned int irq, void *dev_id,
					  struct pt_regs *regs);

typedef struct _ioctl_arg
{
	unsigned long pid;
}ioctl_arg_t;

static struct file_operations fops =
{
		.owner	        = THIS_MODULE,
		.open	        = leuze_open,
		.unlocked_ioctl = leuze_ioctl,
		.release 		= leuze_release,
};

struct siginfo info;
struct task_struct *t = NULL;


static int leuze_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "Device File Opened...\n");
	return 0;
}
static int leuze_release(struct inode *inode, struct file *file)
{
	struct task_struct *ref_task = get_current();
	printk(KERN_INFO "Device File Closed...\n");

	/* delete task */
	if(ref_task){
		task = NULL;
	}
	return 0;
}
static long leuze_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ioctl_arg_t args;

	switch (cmd) {
		case IOCTL_SET_VARIABLES:
			memset(&info, 0, sizeof(struct siginfo));
			info.si_signo = SIGLEUZE;
			info.si_code  = SI_QUEUE;
			info.si_int   = 1234;
			  if (copy_from_user(&args, (ioctl_arg_t *)arg, sizeof(ioctl_arg_t))) return -EACCES;
			  pid = args.pid;
			  t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
				// find the task with that pid

			break;
		default:
			break;
	}

	return 0;
}

static int __init leuze_init(void)
{
	int result = 0;

	gpio_request(gpioSync, "sysfs");	/* Set up the gpioSync */
	gpio_direction_input(gpioSync);
	gpio_export(gpioSync, false);

	irqNumber = gpio_to_irq(gpioSync);


	/* Allocating Major number */
	if((alloc_chrdev_region(&dev, 0, 1, "leuze_Dev")) < 0){
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));

	/* creating cdev structure */
	cdev_init(&leuze_cdev, &fops);

	/* Adding character device to the system */
	if((cdev_add(&leuze_cdev, dev, 1)) < 0){
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	/* Creating struct class */
	if((dev_class = class_create(THIS_MODULE, "leuze_class")) == NULL){
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if((device_create(dev_class, NULL, dev, NULL, "dsp_sync_input")) == NULL){
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}
    
    mem = ioremap(START_ADDR, 4);
    if((request_irq(irqNumber, (irq_handler_t) leuze_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "dsp_sync_input", NULL))){
    	printk(KERN_INFO "cannot register IRQ");
    	goto irq;
    }
	return 0;

irq:
	free_irq(irqNumber, NULL);
r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(dev, 1);
	return -1;
}

static void __exit leuze_exit(void)
{
	free_irq(irqNumber, NULL);
	gpio_unexport(gpioSync);
	gpio_free(gpioSync);
	device_destroy(dev_class,dev);
	class_destroy(dev_class);
	cdev_del(&leuze_cdev);
	unregister_chrdev_region(dev, 1); tick_period = 1000000; //reset period to 1 ms
	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

static irq_handler_t leuze_irq_handler(unsigned int irq, void *dev_id,
					  struct pt_regs *regs)
{
	//send_sig_info(SIGLEUZE, &info, t);
	if(t != NULL){
		if(&info)
		{
               tick_period = 100000000; //set period to 100 ms
		iowrite32(1U << 4, mem);
		send_sig_info(SIGLEUZE, &info, t);
		return (irq_handler_t) IRQ_HANDLED;
		}
	}
	else{
		//printk(KERN_INFO "pid_task error  T = %d \n", t);
		return (irq_handler_t) IRQ_NONE;
	}
}

module_init(leuze_init);
module_exit(leuze_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berrux Kana < bkana@leuze.de >");
MODULE_DESCRIPTION("A GPIO Interrupt driver to send the Signal to the userspace");
MODULE_VERSION("0.1");
