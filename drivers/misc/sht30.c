#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/uaccess.h>







static int sht30_i2c_probe(struct i2c_client *client,
                                      const struct i2c_device_id *id)
{
	unsigned char command[2] ={0x37,0x80};
	unsigned char command1[2] ={0x2C,0x06};
	unsigned char buf[4];
	unsigned char buf1[4];

        if (!i2c_check_functionality(client->adapter,
                                     I2C_FUNC_SMBUS_WORD_DATA)) {
                dev_err(&client->dev, "SMBUS Word Data not Supported\n");
                return -EIO;
        }
	printk("sht30_i2c_probe the addr is %x\r\n",client->addr);

	i2c_master_send(client,command,2);
	i2c_master_recv(client,buf,4);

	printk("sht30_i2c_probe %x\r\n",buf[0]);	
	printk("sht30_i2c_probe %x\r\n",buf[1]);	
	printk("sht30_i2c_probe %x\r\n",buf[2]);	
	printk("sht30_i2c_probe %x\r\n",buf[3]);	

	i2c_master_send(client,command1,2);
        i2c_master_recv(client,buf1,4);

        printk("sht30_i2c_probe %x\r\n",buf1[0]);
        printk("sht30_i2c_probe %x\r\n",buf1[1]);
        printk("sht30_i2c_probe %x\r\n",buf1[2]);
        printk("sht30_i2c_probe %x\r\n",buf1[3]);
	
	
        return 0;
}

static int sht30_i2c_remove(struct i2c_client *client)
{
        return 0;
}

static const struct i2c_device_id sht30_id[] = {
        { "sht30", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, sht30_id);

static struct i2c_driver sht30_i2c_driver = {
        .driver = {
                .name   = "sht30",
                .owner  = THIS_MODULE,
        },
        .probe          = sht30_i2c_probe,
        .remove         = sht30_i2c_remove,
        .id_table       = sht30_id,
};

module_i2c_driver(sht30_i2c_driver);

MODULE_AUTHOR("zhengsj <419019430@qq.com>");
MODULE_DESCRIPTION("SHT30 I2C bus driver");
MODULE_LICENSE("GPL");

