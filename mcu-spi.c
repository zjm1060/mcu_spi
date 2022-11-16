#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h> 
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

MODULE_LICENSE("GPL");

#define BOARD_NAME "mcu_afe"

struct spi_afe_data {
	struct spi_device *client;

    struct semaphore read_sem;
    wait_queue_head_t rxq;

    /* character device */
    dev_t cdevNum;
    struct cdev cdev;
    struct class *cls;

    uint32_t irq_count;
    uint32_t read_count;
    struct gpio_desc *irq_gpio;

    void *buffer;
    ssize_t bufferSize;

    unsigned int irq;
};

static struct of_device_id spi_afe_driver_ids[] = {
	{
		.compatible = "mcu,spi-afe",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_afe_driver_ids);

static struct spi_device_id spi_afe[] = {
	{"spi_afe", 0},
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_afe);

static int spi_afe_open(struct inode *inode, struct file *filp)
{
    struct spi_afe_data *devInfo = container_of(inode->i_cdev, struct spi_afe_data, cdev);

    filp->private_data = devInfo;

    return 0;
}

static ssize_t spi_afe_read(struct file *filp,char __user *buf,size_t size,loff_t *ppos) 
{
    struct spi_afe_data * devInfo = (struct spi_afe_data *) filp->private_data;
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

    devInfo->read_count ++;

    up(&devInfo->read_sem);

    return bytesDone==0?size:0;
}

struct file_operations fileOps = {
	.owner =    THIS_MODULE,
	.read =     spi_afe_read,
	// .write =    fpga_write,
	.open =     spi_afe_open,
	// .release =  fpga_close,
};


static irqreturn_t spi_afe_irq_thread_handler(int irq, void *dev_id)
{
    struct spi_afe_data *devInfo = dev_id;

    spi_read(devInfo->client, devInfo->buffer, devInfo->bufferSize);

    wake_up_interruptible(&devInfo->rxq);

    return IRQ_HANDLED;
}

static irqreturn_t  spi_afe_gpio_irq_handler(int irq, void *dev_id) 
{
    struct spi_afe_data *devInfo = dev_id;

    devInfo->irq_count ++;

	// printk("gpio_irq: Interrupt was triggered and ISR was called! %d\n", devInfo->irq_count);
	return IRQ_WAKE_THREAD;
}

static int setup_chrdev(struct spi_afe_data *devInfo){
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

static int spi_afe_probe(struct spi_device *client)
{
	struct spi_afe_data *devInfo;
	int ret;

	printk("spi_afe - Now I am in the Probe function!\n");

    // if(!device_property_present(&client->dev, "interrupt")) {
	// 	printk("spi_afe - Error! Device property 'interrupt' not found!\n");
	// 	return -1;
	// }

    

	devInfo = devm_kzalloc(&client->dev, sizeof(struct spi_afe_data), GFP_KERNEL);
	if(!devInfo) {
		printk("spi_afe - Error! Out of memory\n");
		return -ENOMEM;
	}

    devInfo->bufferSize = 10240;

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
        ret = devm_request_threaded_irq(&client->dev, devInfo->irq, spi_afe_gpio_irq_handler, spi_afe_irq_thread_handler, IRQF_TRIGGER_RISING, BOARD_NAME, devInfo);
        if(ret){
            printk("Error!\nCan not request interrupt nr.: %d\n", devInfo->irq);
            return -1;
        }
    }

    init_waitqueue_head(&devInfo->rxq);

    sema_init(&devInfo->read_sem, 1);

	ret = spi_setup(client);
	if(ret < 0) {
		printk("spi_afe - Error! Failed to set up the SPI Bus\n");
		return ret;
	}
	
	spi_set_drvdata(client, devInfo);    

	return setup_chrdev(devInfo);
}

static int spi_afe_remove(struct spi_device *client) 
{
    struct spi_afe_data *devInfo = spi_get_drvdata(client);

    printk("spi_afe - Now I am in the remove function.\n");

    // gpio_free(devInfo->interrupt);
    disable_irq(devInfo->irq);

    // free_irq(devInfo->irq, &client->dev);

    unregister_chrdev_region(devInfo->cdevNum, 1);

    device_destroy(devInfo->cls,devInfo->cdevNum);

    class_destroy(devInfo->cls);

    cdev_del(&devInfo->cdev);

	return 0;
}

static struct spi_driver spi_afe_driver = {
	.probe = spi_afe_probe,
	.remove = spi_afe_remove,
	.id_table = spi_afe,
	.driver = {
		.name = "spi_afe",
		.of_match_table = spi_afe_driver_ids,
	},
};

module_spi_driver(spi_afe_driver);