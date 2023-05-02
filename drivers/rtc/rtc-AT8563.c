/*
 * AnalogTek AT8563 RTC driver
 *
 * Copyright (C) 2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 *
 * based on rtc-AT8563
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#define AT8563_USED     "\n \
												  \n+++++++++++++++++++++++++++++++++ \
												  \n++++++ AT8563 new used +++++++++ \
												  \n+++++++++++++++++++++++++++++++++ \
												  \n"

#define AT8563_I2C_NAME 	        "AT8563"
static struct i2c_driver AT8563_driver;												  
static __u32 twi_id = 2;
static const unsigned short normal_i2c[2] = {0x51, I2C_CLIENT_END};

#define AT8563_CTL1		0x00
#define AT8563_CTL1_TEST	BIT(7)
#define AT8563_CTL1_STOP	BIT(5)
#define AT8563_CTL1_TESTC	BIT(3)

#define AT8563_CTL2		0x01
#define AT8563_CTL2_TI_TP	BIT(4)
#define AT8563_CTL2_AF		BIT(3)
#define AT8563_CTL2_TF		BIT(2)
#define AT8563_CTL2_AIE	BIT(1)
#define AT8563_CTL2_TIE	BIT(0)

#define AT8563_SEC		0x02
#define AT8563_SEC_VL		BIT(7)
#define AT8563_SEC_MASK	0x7f

#define AT8563_MIN		0x03
#define AT8563_MIN_MASK	0x7f

#define AT8563_HOUR		0x04
#define AT8563_HOUR_MASK	0x3f

#define AT8563_DAY		0x05
#define AT8563_DAY_MASK	0x3f

#define AT8563_WEEKDAY		0x06
#define AT8563_WEEKDAY_MASK	0x07

#define AT8563_MONTH		0x07
#define AT8563_MONTH_CENTURY	BIT(7)
#define AT8563_MONTH_MASK	0x1f
#define AT8563_CENTURY_MASK	0x80

#define AT8563_YEAR		0x08

#define AT8563_ALM_MIN		0x09
#define AT8563_ALM_HOUR	0x0a
#define AT8563_ALM_DAY		0x0b
#define AT8563_ALM_WEEK	0x0c

/* Each alarm check can be disabled by setting this bit in the register */
#define AT8563_ALM_BIT_DISABLE	BIT(7)

#define AT8563_CLKOUT		0x0d
#define AT8563_CLKOUT_ENABLE	BIT(7)
#define AT8563_CLKOUT_32768	0
#define AT8563_CLKOUT_1024	1
#define AT8563_CLKOUT_32	2
#define AT8563_CLKOUT_1	3
#define AT8563_CLKOUT_MASK	3

#define AT8563_TMR_CTL		0x0e
#define AT8563_TMR_CTL_ENABLE	BIT(7)
#define AT8563_TMR_CTL_4096	0
#define AT8563_TMR_CTL_64	1
#define AT8563_TMR_CTL_1	2
#define AT8563_TMR_CTL_1_60	3
#define AT8563_TMR_CTL_MASK	3

#define AT8563_TMR_CNT		0x0f

struct AT8563 {
	struct i2c_client	*client;
	struct rtc_device	*rtc;
	bool			valid;
};

/*
 * RTC handling
 */

static int AT8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct AT8563 *AT8563 = i2c_get_clientdata(client);
	u8 buf[7];
	int ret;
    int century = 0;

	printk("%s------niotongyuan-------\n",__func__);
	//if (!AT8563->valid) {
		//dev_warn(&client->dev, "no valid clock/calendar values available\n");
		//return -EPERM;
	//}

	ret = i2c_smbus_read_i2c_block_data(client, AT8563_SEC, 7, buf);

	tm->tm_sec = bcd2bin(buf[0] & AT8563_SEC_MASK);
	tm->tm_min = bcd2bin(buf[1] & AT8563_MIN_MASK);
	tm->tm_hour = bcd2bin(buf[2] & AT8563_HOUR_MASK);
	tm->tm_mday = bcd2bin(buf[3] & AT8563_DAY_MASK);
	tm->tm_wday = bcd2bin(buf[4] & AT8563_WEEKDAY_MASK); /* 0 = Sun */
	//tm->tm_mon = bcd2bin(buf[5] & AT8563_MONTH_MASK) - 1; /* 0 = Jan */
	tm->tm_mon = bcd2bin(buf[5] & AT8563_MONTH_MASK); /* 0 = Jan */
    //century=bcd2bin(buf[5] & AT8563_CENTURY_MASK) >> 7 ;
    //printk("%s:----niotong----century=%d\n",__func__,century);
	#if 0
	tm->tm_year = bcd2bin(buf[6]) + 100*(century==1?0:1);
	#else
	tm->tm_year = bcd2bin(buf[6]) + 100;
	#endif
    if(rtc_valid_tm(tm)){
        tm->tm_sec = 0;
        tm->tm_min = 0;
        tm->tm_hour = 0;
        tm->tm_mday = 1;
        tm->tm_wday = 0;
        tm->tm_mon = 0;
        tm->tm_year = 100;

	    i2c_smbus_write_byte_data(client, AT8563_CTL1,AT8563_CTL1_STOP);
        buf[0] = bin2bcd(tm->tm_sec);
        buf[1] = bin2bcd(tm->tm_min);
        buf[2] = bin2bcd(tm->tm_hour);
        buf[3] = bin2bcd(tm->tm_mday);
        buf[4] = bin2bcd(tm->tm_wday);
        buf[5] = bin2bcd(tm->tm_mon);
	    buf[6] = bin2bcd(tm->tm_year-100);
	    i2c_smbus_write_i2c_block_data(client, AT8563_SEC, 7, buf);
	    i2c_smbus_write_byte_data(client, AT8563_CTL1, 0);
    }

	printk("%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	return 0;
}

static int AT8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct AT8563 *AT8563 = i2c_get_clientdata(client);
	u8 buf[7];
	int ret;

	printk("%s------niotongyuan-------\n",__func__);
	/* Years >= 2100 are to far in the future, 19XX is to early */
	//if (tm->tm_year < 100 || tm->tm_year >= 200)
	if (tm->tm_year >= 170)
		return -EINVAL;

	buf[0] = bin2bcd(tm->tm_sec);
	buf[1] = bin2bcd(tm->tm_min);
	buf[2] = bin2bcd(tm->tm_hour);
	buf[3] = bin2bcd(tm->tm_mday);
	buf[4] = bin2bcd(tm->tm_wday);
	//buf[5] = bin2bcd(tm->tm_mon + 1);
	buf[5] = bin2bcd(tm->tm_mon);

	/*
	 * While the AT8563 has a century flag in the month register,
	 * it does not seem to carry it over a subsequent write/read.
	 * So we'll limit ourself to 100 years, starting at 2000 for now.
	 */
#if 0
    if(tm->tm_year >= 100){
        //buf[5] = buf[5] & 0x7f;       //set bit7 0
        printk("----year>=100---niotong-----%x---\n",buf[5]);
	    buf[6] = bin2bcd(tm->tm_year - 100);
    }else{
        //buf[5] = buf[5] | 0x80;      //set bit8 1
        printk("----year<100---niotong-----%x---\n",buf[5]);
	    buf[6] = bin2bcd(tm->tm_year);
    }
#else
	buf[6] = bin2bcd(tm->tm_year-100);
#endif

	printk("%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	/*
	 * CTL1 only contains TEST-mode bits apart from stop,
	 * so no need to read the value first
	 */
	ret = i2c_smbus_write_byte_data(client, AT8563_CTL1,
						AT8563_CTL1_STOP);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_i2c_block_data(client, AT8563_SEC, 7, buf);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, AT8563_CTL1, 0);
	if (ret < 0)
		return ret;

	//AT8563->valid = true;

	return 0;
}

static int AT8563_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data;

	printk("%s------niotongyuan-------\n",__func__);
	data = i2c_smbus_read_byte_data(client, AT8563_CTL2);
	if (data < 0)
		return data;

	if (enabled)
		data |= AT8563_CTL2_AIE;
	else
		data &= ~AT8563_CTL2_AIE;

	return i2c_smbus_write_byte_data(client, AT8563_CTL2, data);
};

static int AT8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rtc_time *alm_tm = &alm->time;
	u8 buf[4];
	int ret;

	printk("%s------niotongyuan-------\n",__func__);
	ret = i2c_smbus_read_i2c_block_data(client, AT8563_ALM_MIN, 4, buf);
	if (ret < 0)
		return ret;

	/* The alarm only has a minute accuracy */
	alm_tm->tm_sec = -1;

	alm_tm->tm_min = (buf[0] & AT8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[0] & AT8563_MIN_MASK);
	alm_tm->tm_hour = (buf[1] & AT8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[1] & AT8563_HOUR_MASK);
	alm_tm->tm_mday = (buf[2] & AT8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[2] & AT8563_DAY_MASK);
	alm_tm->tm_wday = (buf[3] & AT8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[3] & AT8563_WEEKDAY_MASK);

	alm_tm->tm_mon = -1;
	alm_tm->tm_year = -1;

	ret = i2c_smbus_read_byte_data(client, AT8563_CTL2);
	if (ret < 0)
		return ret;

	if (ret & AT8563_CTL2_AIE)
		alm->enabled = 1;

	return 0;
}

static int AT8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rtc_time *alm_tm = &alm->time;
	u8 buf[4];
	int ret;

	printk("%s------niotongyuan-------\n",__func__);
	/*
	 * The alarm has no seconds so deal with it
	 */
	if (alm_tm->tm_sec) {
		alm_tm->tm_sec = 0;
		alm_tm->tm_min++;
		if (alm_tm->tm_min >= 60) {
			alm_tm->tm_min = 0;
			alm_tm->tm_hour++;
			if (alm_tm->tm_hour >= 24) {
				alm_tm->tm_hour = 0;
				alm_tm->tm_mday++;
				if (alm_tm->tm_mday > 31)
					alm_tm->tm_mday = 0;
			}
		}
	}

	ret = i2c_smbus_read_byte_data(client, AT8563_CTL2);
	if (ret < 0)
		return ret;

	ret &= ~AT8563_CTL2_AIE;

	ret = i2c_smbus_write_byte_data(client, AT8563_CTL2, ret);
	if (ret < 0)
		return ret;

	buf[0] = (alm_tm->tm_min < 60 && alm_tm->tm_min >= 0) ?
			bin2bcd(alm_tm->tm_min) : AT8563_ALM_BIT_DISABLE;

	buf[1] = (alm_tm->tm_hour < 24 && alm_tm->tm_hour >= 0) ?
			bin2bcd(alm_tm->tm_hour) : AT8563_ALM_BIT_DISABLE;

	buf[2] = (alm_tm->tm_mday <= 31 && alm_tm->tm_mday >= 1) ?
			bin2bcd(alm_tm->tm_mday) : AT8563_ALM_BIT_DISABLE;

	buf[3] = (alm_tm->tm_wday < 7 && alm_tm->tm_wday >= 0) ?
			bin2bcd(alm_tm->tm_wday) : AT8563_ALM_BIT_DISABLE;

	ret = i2c_smbus_write_i2c_block_data(client, AT8563_ALM_MIN, 4, buf);
	if (ret < 0)
		return ret;

	return AT8563_rtc_alarm_irq_enable(dev, alm->enabled);
}

static const struct rtc_class_ops AT8563_rtc_ops = {
	.read_time		= AT8563_rtc_read_time,
	.set_time		= AT8563_rtc_set_time,
	.alarm_irq_enable	= AT8563_rtc_alarm_irq_enable,
	.read_alarm		= AT8563_rtc_read_alarm,
	.set_alarm		= AT8563_rtc_set_alarm,
};

/*
 * The alarm interrupt is implemented as a level-low interrupt in the
 * AT8563, while the timer interrupt uses a falling edge.
 * We don't use the timer at all, so the interrupt is requested to
 * use the level-low trigger.
 */
static irqreturn_t AT8563_irq(int irq, void *dev_id)
{
	struct AT8563 *AT8563 = (struct AT8563 *)dev_id;
	struct i2c_client *client = AT8563->client;
	struct mutex *lock = &AT8563->rtc->ops_lock;
	int data, ret;
	printk("lhg %s: irq = %d\n",__func__,irq);
	mutex_lock(lock);

	/* Clear the alarm flag */

	data = i2c_smbus_read_byte_data(client, AT8563_CTL2);
	if (data < 0) {
		dev_err(&client->dev, "%s: error reading i2c data %d\n",
			__func__, data);
		goto out;
	}

	data &= ~AT8563_CTL2_AF;

	ret = i2c_smbus_write_byte_data(client, AT8563_CTL2, data);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error writing i2c data %d\n",
			__func__, ret);
	}

out:
	mutex_unlock(lock);
	return IRQ_HANDLED;
}

static int AT8563_init_device(struct i2c_client *client)
{
	int ret;

	printk("%s:-------niotong------debug---- \n",__func__);
	/* Clear stop flag if present */
	ret = i2c_smbus_write_byte_data(client, AT8563_CTL1, 0);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(client, AT8563_CTL2);
	if (ret < 0)
		return ret;

	/* Disable alarm and timer interrupts */
	ret &= ~AT8563_CTL2_AIE;
	ret &= ~AT8563_CTL2_TIE;

	/* Clear any pending alarm and timer flags */
	if (ret & AT8563_CTL2_AF)
		ret &= ~AT8563_CTL2_AF;

	if (ret & AT8563_CTL2_TF)
		ret &= ~AT8563_CTL2_TF;

	ret &= ~AT8563_CTL2_TI_TP;

	return i2c_smbus_write_byte_data(client, AT8563_CTL2, ret);
}

static int AT8563_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct AT8563 *AT8563;
	int ret;

	printk("%s:-------niotong------debug---- \n",__func__);
	AT8563 = devm_kzalloc(&client->dev, sizeof(*AT8563), GFP_KERNEL);
	if (!AT8563)
		return -ENOMEM;

	AT8563->client = client;
	i2c_set_clientdata(client, AT8563);

	device_set_wakeup_capable(&client->dev, true);

	ret = AT8563_init_device(client);
	if (ret) {
		dev_err(&client->dev, "could not init device, %d\n", ret);
		return ret;
	}
#if 0
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, AT8563_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						client->name, AT8563);
		if (ret < 0) {
			dev_err(&client->dev, "irq %d request failed, %d\n",
				client->irq, ret);
			return ret;
		}
	}
#endif
	/* check state of calendar information */
	ret = i2c_smbus_read_byte_data(client, AT8563_SEC);
	printk("lhg ret = %d\n",ret);
	if (ret < 0)
		return ret;

    //add by regan ,this can fix low rtc battery cause time error
    if(ret & AT8563_SEC_VL){
        ret &= ~AT8563_SEC_VL;
	    ret = i2c_smbus_write_byte_data(client, AT8563_SEC, 0);
    }
	//dev_dbg(&client->dev, "rtc information is %s\n",
		//AT8563->valid ? "valid" : "invalid");

	//AT8563->rtc = rtc_device_register(AT8563_driver.driver.name,
	//			&client->dev, &AT8563_rtc_ops, THIS_MODULE);
	AT8563->rtc = devm_rtc_device_register(&client->dev,
				AT8563_driver.driver.name, &AT8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(AT8563->rtc))
		return PTR_ERR(AT8563->rtc);

	/* the AT8563 alarm only supports a minute accuracy */
	AT8563->rtc->uie_unsupported = 1;

	return 0;
}

int AT8563_i2c_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;		
	
	ret=i2c_transfer(client->adapter, &msg,1);
	return ret;
}
bool AT8563_i2c_test(struct i2c_client * client)
{
	int ret,retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.

	for(retry=0; retry < 2; retry++)
	{
		ret =AT8563_i2c_write_bytes(client, test_data, 1);	//Test i2c.
		if (ret == 1)
			break;
		msleep(50);
	}

	return ret==1 ? true : false;
}
static int AT8563_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	const char *type_name = AT8563_I2C_NAME;
	int ret;
	printk("%s:-------niotong------debug---- \n",__func__);
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
                return -ENODEV;
    
	if(twi_id == adapter->nr){
		strlcpy(info->type, type_name, 20);
    	printk("%s: addr= %x\n",__func__,client->addr);
        ret = AT8563_i2c_test(client);
        if(!ret){
        	printk("%s:I2C connection might be something wrong \n",__func__);
        	return -ENODEV;
        }else{      
            printk("I2C connection sucess!\n");
            strlcpy(info->type, AT8563_I2C_NAME, I2C_NAME_SIZE);
			printk("%s", AT8563_USED);
    		return 0;	
	    }

	}else{
		printk("%s:%d wrong i2c id:%d, expect id is :%d\n", __func__, __LINE__,
			client->adapter->nr, 3);
		return -ENODEV;
	}
}
static const struct of_device_id AT8563_match[] = {
	{.compatible = "allwinner,AT8563"},
	{}
};
static const struct i2c_device_id AT8563_id[] = {
	{ AT8563_I2C_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, AT8563_id);

static struct i2c_driver AT8563_driver = {
	.class = I2C_CLASS_HWMON,
	.id_table	= AT8563_id,
	.probe		= AT8563_probe,
	//.remove		= pcf8563_remove,
	.driver		= {
		.owner = THIS_MODULE,
		.name	= AT8563_I2C_NAME,
		.of_match_table = AT8563_match,
	},
	.detect        = AT8563_detect,
	.address_list = normal_i2c,
};

static int __init AT8563_drv_init(void)
{
	int ret = -1;
//if need enable power,change here
#if 0
	struct regulator *rtc_power_ldo;
	printk("%s:-------niotong------debug---- \n",__func__);
	rtc_power_ldo = regulator_get(NULL, "axp22_dldo3");
	if(!rtc_power_ldo){
		printk("%s:-------niotong------could not get rtc_power_ldo---- \n",__func__);
		return ret;
	}else{
		regulator_set_voltage(rtc_power_ldo,3000000,3000000);
	}
	regulator_enable(rtc_power_ldo);
#endif
	//AT8563_driver.detect = AT8563_detect;
	printk("%s:-------fly----init rtc---- \n",__func__);
	ret = i2c_add_driver(&AT8563_driver);
	return ret;
}
static void __exit AT8563_drv_exit(void)
{
	//printk("%s:-------niotong------debug---- \n",__func__);
	i2c_del_driver(&AT8563_driver);
	return;
}
//module_i2c_driver(AT8563_driver);
module_init(AT8563_drv_init);
module_exit(AT8563_drv_exit);

MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("AT8563 RTC driver");
MODULE_LICENSE("GPL");
