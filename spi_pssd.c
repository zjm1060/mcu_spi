#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h> 
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/time.h>

MODULE_LICENSE("GPL");

#define BOARD_NAME "pssd"

typedef struct {
	short data[8][512];
}chip_data;

struct spi_data_struct{
    unsigned int header;
    unsigned int serial;
    unsigned int samples_pre_cycle;
    unsigned int chips;
    unsigned int sec;
    unsigned int micro_sec;
    float freq;

    chip_data chip[2];
};

struct spi_pssd_data {
	struct spi_device *client;

    struct semaphore read_sem;
    wait_queue_head_t rxq;

    /* character device */
    dev_t cdevNum;
    struct cdev cdev;
    struct class *cls;

    unsigned long irq_count;
    unsigned long read_count;
    struct gpio_desc *irq_gpio;

    void *buffer;
    ssize_t bufferSize;

    unsigned int irq;
};

static struct of_device_id spi_pssd_driver_ids[] = {
	{
		.compatible = "mcu,spi-pssd",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_pssd_driver_ids);

static struct spi_device_id spi_pssd[] = {
	{"spi_pssd", 0},
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_pssd);

// static inline int __spi_read(struct spi_device *spi, void *buf, size_t len)
// {
// 	struct spi_transfer	t = {
// 			.rx_buf		= buf,
// 			.len		= len,
//             .delay_usecs = 100,
// 		};

// 	return spi_sync_transfer(spi, &t, 1);
// }

static int spi_pssd_open(struct inode *inode, struct file *filp)
{
    struct spi_pssd_data *devInfo = container_of(inode->i_cdev, struct spi_pssd_data, cdev);

    filp->private_data = devInfo;

    devInfo->read_count = devInfo->irq_count;

    return 0;
}

static ssize_t spi_pssd_read(struct file *filp,char __user *buf,size_t size,loff_t *ppos) 
{
    struct spi_pssd_data * devInfo = (struct spi_pssd_data *) filp->private_data;
    ssize_t bytesDone = 0;

    // printk("try read %lu\n", size);

    if (down_interruptible(&devInfo->read_sem)) {
		return -ERESTARTSYS;
	}

    // if(devInfo->irq_count == devInfo->read_count)
    // {
    //     printk("need wait\n");
    // }

    while(devInfo->irq_count == devInfo->read_count){
        up(&devInfo->read_sem);
        if(wait_event_interruptible(devInfo->rxq, devInfo->read_count != devInfo->irq_count)){
            return -ERESTARTSYS;
        }
        if(down_interruptible(&devInfo->read_sem)){
            return -ERESTARTSYS;
        }
    }

    if(size > devInfo->bufferSize)
        size = devInfo->bufferSize;
    
    // printk("try copy data %lu bytes\n", size);
    bytesDone = copy_to_user(buf, devInfo->buffer, size);
    // printk("copy data %lu bytes\n", bytesDone);

    devInfo->read_count ++;//= devInfo->irq_count;

    up(&devInfo->read_sem);

    return bytesDone==0?size:0;
}

struct file_operations fileOps = {
	.owner =    THIS_MODULE,
	.read =     spi_pssd_read,
	// .write =    fpga_write,
	.open =     spi_pssd_open,
	// .release =  fpga_close,
};


static irqreturn_t spi_pssd_irq_thread_handler(int irq, void *dev_id)
{
    struct spi_pssd_data *devInfo = dev_id;
    struct spi_data_struct *sd = devInfo->buffer;

    spi_read(devInfo->client, devInfo->buffer, devInfo->bufferSize);
    devInfo->irq_count ++;

    sd->header = 0x77478507UL;
    sd->serial = devInfo->irq_count;

    wmb();

    wake_up_interruptible(&devInfo->rxq);

    return IRQ_HANDLED;
}

static irqreturn_t  spi_pssd_gpio_irq_handler(int irq, void *dev_id) 
{
    // struct spi_pssd_data *devInfo = dev_id;    

	// printk("gpio_irq: Interrupt was triggered and ISR was called! %d\n", devInfo->irq_count);
	return IRQ_WAKE_THREAD;
}

static int setup_chrdev(struct spi_pssd_data *devInfo){
	/*
	Setup the /dev/deviceName to allow user programs to read/write to the driver.
	*/

	int devMinor = 0;
	int devMajor = 0; 
	int devNum = -1;

	int result = alloc_chrdev_region(&devInfo->cdevNum, devMinor, 1 /* one device*/, BOARD_NAME);
	if (result < 0) {
		printk(KERN_WARNING "Can't get major ID\n");
		return -1;
	}
	devMajor = MAJOR(devInfo->cdevNum);
	devNum = MKDEV(devMajor, devMinor);
	
	//Initialize and fill out the char device structure
	cdev_init(&devInfo->cdev, &fileOps);
	devInfo->cdev.owner = THIS_MODULE;
	devInfo->cdev.ops = &fileOps;
	result = cdev_add(&devInfo->cdev, devNum, 1 /* one device */);
	if (result) {
		printk(KERN_NOTICE "Error %d adding char device for BBN FPGA driver with major/minor %d / %d", result, devMajor, devMinor);
		return -1;
	}

    devInfo->cls = class_create(THIS_MODULE, BOARD_NAME);
    if(!devInfo->cls){
        printk(KERN_ERR "can't register class for fpga\n");
    }

    device_create(devInfo->cls,NULL,devInfo->cdevNum,NULL,BOARD_NAME);

	return 0;
}

static int spi_pssd_probe(struct spi_device *client)
{
	struct spi_pssd_data *devInfo;
	int ret;

	printk("spi_pssd - Now I am in the Probe function!\n");

    // if(!device_property_present(&client->dev, "interrupt")) {
	// 	printk("spi_pssd - Error! Device property 'interrupt' not found!\n");
	// 	return -1;
	// }

    

	devInfo = devm_kzalloc(&client->dev, sizeof(struct spi_pssd_data), GFP_KERNEL);
	if(!devInfo) {
		printk("spi_pssd - Error! Out of memory\n");
		return -ENOMEM;
	}

    devInfo->bufferSize = sizeof(struct spi_data_struct);
    printk("bufferSize: %lu\n", devInfo->bufferSize);

    devInfo->buffer = devm_kzalloc(&client->dev, devInfo->bufferSize, GFP_KERNEL);

	devInfo->client = client;

    devInfo->irq_gpio = devm_gpiod_get_optional(&client->dev, "int",
						    GPIOD_IN);
    if (IS_ERR(devInfo->irq_gpio)) {
		printk("invalid int-gpios property in node\n");
		return PTR_ERR(devInfo->irq_gpio);
	}                        

    gpiod_direction_input(devInfo->irq_gpio);

    devInfo->irq = gpiod_to_irq(devInfo->irq_gpio);

    if(devInfo->irq){
        // ret = devm_request_irq(&client->dev, devInfo->irq, (irq_handler_t)gpio_irq_handler, IRQF_TRIGGER_RISING, BOARD_NAME, devInfo);
        ret = devm_request_threaded_irq(&client->dev, devInfo->irq, spi_pssd_gpio_irq_handler, spi_pssd_irq_thread_handler, IRQF_TRIGGER_RISING, BOARD_NAME, devInfo);
        if(ret){
            printk("Error!\nCan not request interrupt nr.: %d\n", devInfo->irq);
            return -1;
        }
    }

    init_waitqueue_head(&devInfo->rxq);

    sema_init(&devInfo->read_sem, 1);

	ret = spi_setup(client);
	if(ret < 0) {
		printk("spi_pssd - Error! Failed to set up the SPI Bus\n");
		return ret;
	}
	
	spi_set_drvdata(client, devInfo);    

	return setup_chrdev(devInfo);
}

static int spi_pssd_remove(struct spi_device *client) 
{
    struct spi_pssd_data *devInfo = spi_get_drvdata(client);

    printk("spi_pssd - Now I am in the remove function.\n");

    // gpio_free(devInfo->interrupt);
    disable_irq(devInfo->irq);

    // free_irq(devInfo->irq, &client->dev);

    unregister_chrdev_region(devInfo->cdevNum, 1);

    device_destroy(devInfo->cls,devInfo->cdevNum);

    class_destroy(devInfo->cls);

    cdev_del(&devInfo->cdev);

	return 0;
}

static struct spi_driver spi_pssd_driver = {
	.probe = spi_pssd_probe,
	.remove = spi_pssd_remove,
	.id_table = spi_pssd,
	.driver = {
		.name = "spi_pssd",
		.of_match_table = spi_pssd_driver_ids,
	},
};

module_spi_driver(spi_pssd_driver);