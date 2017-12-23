/* Sensirion SHT21 humidity and temperature sensor driver
 *
 * Copyright (C) 2010 Urs Fleisch <urs.fleisch@sensirion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Data sheet available (5/2010) at
 * http://www.sensirion.com/en/pdf/product_information/Datasheet-humidity-sensor-SHT21.pdf
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>

/**
 * struct sht21 - SHT21 device specific data
 * @hwmon_dev: device registered with hwmon
 * @lock: mutex to protect measurement values
 * @valid: only 0 before first measurement is taken
 * @last_update: time of last update (jiffies)
 * @temperature: cached temperature measurement value
 * @humidity: cached humidity measurement value
 */
struct sht30 {
	struct i2c_client *client;
	struct mutex lock;
	char valid;
	unsigned long last_update;
	int temperature;
	int humidity;
};

/**
 * sht30_temp_ticks_to_millicelsius() - convert raw temperature ticks to
 * milli celsius
 * @ticks: temperature ticks value received from sensor
 */
static inline unsigned int sht30_temp_ticks_to_millicelsius(unsigned short ticks)
{
	return (21875 * ticks >> 13) - 45000;
}

/**
 * sht30_rh_ticks_to_per_cent_mille() - convert raw humidity ticks to
 * one-thousandths of a percent relative humidity
 * @ticks: humidity ticks value received from sensor
 */
static inline unsigned int sht30_rh_ticks_to_per_cent_mille(int ticks)
{
	return (12500 * ticks >> 13);
}



/**
 * sht30_update_measurements() - get updated measurements from device
 * @dev: device
 *
 * Returns 0 on success, else negative errno.
 */
static int sht30_update_measurements(struct device *dev)
{
	int ret = 0;
	struct sht30 *sht30 = dev_get_drvdata(dev);
	struct i2c_client *client = sht30->client;
	unsigned char command[2] ={0x2c,0x06};
	unsigned char buf[4];
	unsigned short temp1;
	unsigned short humidity1;

	mutex_lock(&sht30->lock);
	/*
	 * Data sheet 2.4:
	 * SHT30 should not be active for more than 10% of the time - e.g.
	 * maximum two measurements per second at 12bit accuracy shall be made.
	 */
	if (time_after(jiffies, sht30->last_update + HZ / 2) || !sht30->valid) {

		i2c_master_send(client,command,2);
        	i2c_master_recv(client,buf,4);
		
//		printk("sht30_i2c_probe %x\r\n",buf[0]);
  //      	printk("sht30_i2c_probe %x\r\n",buf[1]);
    //    	printk("sht30_i2c_probe %x\r\n",buf[2]);
      //  	printk("sht30_i2c_probe %x\r\n",buf[3]);


		temp1 = ((buf[0]<<8) | (buf[1]));
		humidity1 = ((buf[2]<<8) | (buf[3]));

	//	printk("%d \r\n", temp1);
	//	printk("%d \r\n", humidity1);

		sht30->temperature = sht30_temp_ticks_to_millicelsius(temp1);

		sht30->humidity =    sht30_rh_ticks_to_per_cent_mille(humidity1);

		sht30->last_update = jiffies;
		sht30->valid = 1;
	}
out:
	mutex_unlock(&sht30->lock);

	return 0;
}

/**
 * sht30_show_temperature() - show temperature measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t sht30_show_temperature(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct sht30 *sht30 = dev_get_drvdata(dev);
	int ret;

	ret = sht30_update_measurements(dev);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", sht30->temperature);
}

/**
 * sht30_show_humidity() - show humidity measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t sht30_show_humidity(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct sht30 *sht30 = dev_get_drvdata(dev);
	int ret;

	ret = sht30_update_measurements(dev);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", sht30->humidity);
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, sht30_show_temperature,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, sht30_show_humidity,
	NULL, 0);

static struct attribute *sht30_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(sht30);

static int sht30_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct sht30 *sht30;
	unsigned char command[2] ={0x37,0x80};
        unsigned char buf[4];



	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
			"adapter does not support SMBus word transactions\n");
		return -ENODEV;
	}

        printk("sht30_i2c_probe the addr is %x\r\n",client->addr);

        i2c_master_send(client,command,2);
        i2c_master_recv(client,buf,4);

        printk("sht30_i2c_probe %x\r\n",buf[0]);
        printk("sht30_i2c_probe %x\r\n",buf[1]);
        printk("sht30_i2c_probe %x\r\n",buf[2]);
        printk("sht30_i2c_probe %x\r\n",buf[3]);




	sht30 = devm_kzalloc(dev, sizeof(*sht30), GFP_KERNEL);
	if (!sht30)
		return -ENOMEM;

	sht30->client = client;

	mutex_init(&sht30->lock);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   sht30, sht30_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* Device ID table */
static const struct i2c_device_id sht30_id[] = {
	{ "sht30", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sht30_id);

static struct i2c_driver sht30_driver = {
	.driver.name = "sht30",
	.probe       = sht30_probe,
	.id_table    = sht30_id,
};

module_i2c_driver(sht30_driver);

MODULE_AUTHOR("goembed <www.goembed.com>");
MODULE_DESCRIPTION("Sensirion SHT30 humidity and temperature sensor driver");
MODULE_LICENSE("GPL");
