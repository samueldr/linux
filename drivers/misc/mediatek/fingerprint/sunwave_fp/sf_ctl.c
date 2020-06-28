/**
 * The device control driver for sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson L. <mailto: liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the Free 
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>

#include "nt_smc_call.h"

#include "mt_spi.h"
#include "mt_spi_hal.h"

#include "sf_ctl.h"

//liukangping@wind-mobi.com 20170117 begin
#ifdef CONFIG_WIND_DEVICE_INFO
#include <../../wind_device_info/wind_device_info.h>
#endif
//liukangping@wind-mobi.com 20170117 end


struct spi_device *sf_spi = NULL;
struct mt_spi_t *mt_spi = NULL;

#define MODULE_NAME "sf_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

#ifndef CONFIG_OF
# error "error: this driver 'MODULE_NAME' only support dts."
#endif

/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define SF_DRV_VERSION "v0.9.1-rXXXX_yyyymmdd"

//liukangping@wind-mobi.com 20170117 begin
#ifdef CONFIG_WIND_DEVICE_INFO
extern wind_device_info_t wind_device_info;
#endif
//liukangping@wind-mobi.com 20170117 end

struct sf_ctl_device {
	struct miscdevice miscdev;
	int irq_num;
	struct work_struct work_queue;
	struct input_dev *input;
};
#if 0
typedef enum {
    SF_PIN_STATE_INT_SET,
    SF_PIN_STATE_MISO_SPI,
    SF_PIN_STATE_MISO_PULLHIGH,
    SF_PIN_STATE_MISO_PULLLOW,
    SF_PIN_STATE_RST_SET,
    SF_PIN_STATE_RST_CLR,

    /* Array size */
    SF_PIN_STATE_MAX
} sf_pin_state_t;

static const char *sf_pinctrl_state_names[SF_PIN_STATE_MAX] = {
    "power_on", "power_off", "reset_low", "reset_high", "eint_set",
	"fingerprint_irq","miso_spi", "miso_pullhigh", "miso_pulllow","reset_high", "reset_low",
};
#endif

typedef enum {
	SF_PIN_STATE_INT_SET,
	
	SF_PIN_STATE_MSIO_SPI,
	SF_PIN_STATE_MSIO_HIGH,
	SF_PIN_STATE_MSIO_LOW,
	
    SF_PIN_STATE_RST_SET,
    SF_PIN_STATE_RST_CLR,

    /* Array size */
    SF_PIN_STATE_MAX
} sf_pin_state_t;

static const char *sf_pinctrl_state_names[SF_PIN_STATE_MAX] = {
	"fingerprint_irq", "miso_spi", "miso_pullhigh", "miso_pulllow", "reset_high",  "reset_low",
};

static struct pinctrl *sf_pinctrl = NULL;
static struct pinctrl_state *sf_pin_states[SF_PIN_STATE_MAX] = {NULL, };

/*static int sf_ctl_device_power(bool on) // lanh 2016-11-21
{
    int err = 0;
    sf_pin_state_t state = on ? SF_PIN_STATE_PWR__ON : SF_PIN_STATE_PWR_OFF;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[state]);

    return err;
} */

static int sf_ctl_device_reset(void)
{
	int err = 0;
	xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

	/*err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_CLR]);
    msleep(1);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_SET]);
    msleep(100);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_CLR]);*/
	
	err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_SET]);
    msleep(1);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_CLR]);
    msleep(100);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_SET]);

    return err;
}

static void sf_spi_clock_enable(bool on)
{
	if (on) {
       mt_spi_enable_clk(mt_spi); // lanh 2016-11-21
    } else {
       mt_spi_disable_clk(mt_spi); // lanh 2016-11-21
    }
	
}

static void sf_ctl_device_event(struct work_struct *ws)
{
    struct sf_ctl_device *sf_ctl_dev =
            container_of(ws, struct sf_ctl_device, work_queue);
    char *uevent_env[2] = { "SPI_STATE=finger", NULL };
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    
    kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj,
            KOBJ_CHANGE, uevent_env);
}

static irqreturn_t sf_ctl_device_irq(int irq, void *dev_id)
{
	struct sf_ctl_device *sf_ctl_dev = (struct sf_ctl_device*)dev_id;

	disable_irq_nosync(irq);
	xprintk(KERN_DEBUG, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);

	schedule_work(&sf_ctl_dev->work_queue);

	enable_irq(irq);
	return IRQ_HANDLED;
}

//wangbing@wind-mobi.com 20170804 begin >> note: add the finnger gesture input event about DOUBLE CLICK
//liukangping add key begin 20170316
static int sf_ctl_report_key_event(struct input_dev *input, sf_key_event_t *kevent)
{
        int err = 0;
        unsigned int key_code = KEY_UNKNOWN;
        xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
        xprintk(KERN_INFO, "sunwave -- kevent->key = %d\n", kevent->key);
        switch (kevent->key)
        {
        case SF_KEY_HOME:
                key_code = KEY_HOME;
                break;
        case SF_KEY_MENU:
                key_code = KEY_MENU;
                break;
        case SF_KEY_BACK:
                key_code = KEY_BACK;
                break;
        case SF_KEY_F11:
                key_code = KEY_DEL_EOL;
                break;
        case SF_KEY_ENTER:
                key_code = KEY_INS_LINE;
                break;
        case SF_KEY_UP:
                key_code = KEY_UP;
                break;
        case SF_KEY_LEFT:
                key_code = KEY_LEFT;
                break;
        case SF_KEY_RIGHT:
                key_code = KEY_RIGHT;
                break;
        case SF_KEY_DOWN:
                key_code = KEY_DOWN;
                break;
        case SF_KEY_WAKEUP:
                key_code = KEY_WAKEUP;
                break;
        case SF_KEY_DOUBLE_CLICK:
                key_code = KEY_DEL_EOS;
                break;
        default:
                break;
        }
//liukangping add key end 20170316
        // xprintk(KERN_INFO, "[wind_fp][%s][%d] input key code = %d\n", __func__, __LINE__, key_code);
        input_report_key(input, key_code, kevent->value);
        input_sync(input);
        return err;
}
//wangbing@wind-mobi.com 20170804 end

static const char* sf_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char *)version;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long sf_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = (struct miscdevice*)filp->private_data;
	struct sf_ctl_device *sf_ctl_dev =
			container_of(dev, struct sf_ctl_device, miscdev);
	int err = 0;
	sf_key_event_t kevent;
	//zhangkaiyuan@wind-mobi.com 20170413 begin
	sf_ic_info_t ic_info_data;
	//zhangkaiyuan@wind-mobi.com 20170413 end
	xprintk(KERN_DEBUG, "%s(cmd = 0x%08x, ..)\n", __FUNCTION__, cmd);

	switch (cmd) {

	case SF_IOC_INIT_DRIVER: {
	    // TODO:
	    break;
	}

	case SF_IOC_DEINIT_DRIVER: {
	    // TODO:
	    break;
	}

	case SF_IOC_RESET_DEVICE: {
		sf_ctl_device_reset();
		break;
	}

    case SF_IOC_ENABLE_IRQ: {
        // TODO:
        break;
    }

    case SF_IOC_DISABLE_IRQ: {
        // TODO:
        break;
    }

    case SF_IOC_ENABLE_SPI_CLK: {
        // TODO:
		printk("%s SF_IOC_ENABLE_SPI_CLK enter", __func__);
		sf_spi_clock_enable(true);
        break;
    }

    case SF_IOC_DISABLE_SPI_CLK: {
        // TODO:
		printk("%s SF_IOC_DISABLE_SPI_CLK enter", __func__);
		sf_spi_clock_enable(false);
        break;
    }

    case SF_IOC_ENABLE_POWER: {
        // TODO:
        break;
    }

    case SF_IOC_DISABLE_POWER: {
        // TODO:
        break;
    }

	case SF_IOC_REPORT_KEY_EVENT: {
        if (copy_from_user(&kevent, (sf_key_event_t *)arg, sizeof(sf_key_event_t))) {
            xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
            err = (-EFAULT);
            break;
        }
		//liukangping add key begin 20170316
		xprintk(KERN_INFO, "sunwave -- report key event");
		//liukangping add key end 20170316
        err = sf_ctl_report_key_event(sf_ctl_dev->input, &kevent);
	    break;
	}

	case SF_IOC_SYNC_CONFIG: {
	    // TODO:
	    break;
	}

	case SF_IOC_GET_VERSION: {
        if (copy_to_user((void *)arg, sf_ctl_get_version(), SF_DRV_VERSION_LEN)) {
            xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
            err = (-EFAULT);
            break;
        }
	    break;
	}
	
	/*add zhangkaiyuan@wind-mobi.com 20170413 start.*/
	case SF_IOC_SET_IC_INFO: {
		if (copy_from_user(&ic_info_data, (sf_ic_info_t *)arg, sizeof(sf_ic_info_t))) {
            xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
            err = (-EFAULT);
            break;
        }
		sprintf(wind_device_info.fp_module_info.ic_name, "%s", ic_info_data.ic_name);
		wind_device_info.fp_module_info.vendor = ic_info_data.vendor; 
		wind_device_info.fp_module_info.fwvr = ic_info_data.fwvr;
		xprintk(KERN_INFO, "ic_name = %s, fp_module_info.vendor = %02x, fp_module_info.fwvr = %02x\n", 
				wind_device_info.fp_module_info.ic_name, wind_device_info.fp_module_info.vendor, wind_device_info.fp_module_info.fwvr);
		break;
	}
	/*add zhangkaiyuan@wind-mobi.com 20170413 end.*/

	default:
		err = (-EINVAL);
		break;
	}
	return err;
}

static int sf_ctl_open(struct inode *inode, struct file *filp)
{
	xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
	return 0;
}

static int sf_ctl_release(struct inode *inode, struct file *filp)
{
	xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////

static struct file_operations sf_ctl_fops = {
	.owner		    = THIS_MODULE,
	.unlocked_ioctl = sf_ctl_ioctl,
	.open		    = sf_ctl_open,
	.release	    = sf_ctl_release,
};

static struct sf_ctl_device sf_ctl_dev = {
	.miscdev = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "sunwave_fp",
		.fops	= &sf_ctl_fops,
	}, 0,
};
#if 1
#ifdef CONFIG_OF
static const struct of_device_id sf_of_match[] = {
	/*{ .compatible = "mediatek,fingerprint", },
	{ .compatible = "mediatek,sunwave-fp", },
	{ .compatible = "sunwave,sunwave-fp", },
	{},*/
	{ .compatible = "mediatek,fingerprint", },
	{ .compatible = "mediatek,goodix-fp", },
	{ .compatible = "goodix,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, sf_of_match);
#endif



static int sf_probe(struct spi_device *spi) {
	int err = 0;
	
	sf_spi = spi;

	xprintk(KERN_ERR, "lanhai %s enter\n", __func__);
	
	/*sf_spi->mode = SPI_MODE_0;
	sf_spi->bits_per_word = 8;
	sf_spi->max_speed_hz = 1 * 1000 * 1000;
	memcpy(&spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
	sf_spi->controller_data = (void *)&spi_mcc;

	spi_setup(sf_spi); */
	
	//hebiao@wind-mobi.com 20161105 begin
#ifdef CONFIG_WIND_DEVICE_INFO
	sprintf(wind_device_info.fp_module_info.ic_name, "%s", "sunwave");
	wind_device_info.fp_module_info.vendor = 0x02; //hilitai is 0x01
	wind_device_info.fp_module_info.fwvr = 0x3A;
#endif
//hebiao@wind-mobi.com 20161105 end	
	
	mt_spi = spi_master_get_devdata(sf_spi->master);
	if (!mt_spi) {
        xprintk(KERN_ERR, "fail to get mediatek spi device.\n");
        dump_stack();
        return (-ENODEV);
    }

	//sf_spi_clock_enable(true); //zhangkaiyuan@wind-mobi.com 20170614

	xprintk(KERN_ERR, "%s leave\n", __func__);
    return err;
}

static int sf_remove(struct spi_device *spi) {
	return 0;
}

static struct spi_driver sf_spi_driver = {
	.driver = {
		.name = "sunwave-fp",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sf_of_match,
#endif
	},
	.probe = sf_probe,
	.remove = sf_remove,
};
#endif
// see sf_spi.c
extern int  sf_spi_platform_init(void);
extern void sf_spi_platform_exit(void);

////////////////////////////////////////////////////////////////////////////////

static int sf_ctl_init_gpio_pins(void)
{
    int i, err = 0;
    struct platform_device *pdev = NULL;
    struct device_node *dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    dev_node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        xprintk(KERN_ERR, "of_find_device_by_node(..) failed.\n");
        return (-ENODEV);
    }

    sf_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!sf_pinctrl) {
        xprintk(KERN_ERR, "devm_pinctrl_get(..) failed.\n");
        return (-ENODEV);
    }

    for (i = 0; i < SF_PIN_STATE_MAX; ++i) {
        sf_pin_states[i] = pinctrl_lookup_state(sf_pinctrl,
                sf_pinctrl_state_names[i]);
        if (!sf_pin_states[i]) {
            xprintk(KERN_ERR, "can't find '%s' pinctrl_state.\n",
                    sf_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < SF_PIN_STATE_MAX) {
        xprintk(KERN_ERR, "%s() failed.\n", __FUNCTION__);
    }
	


    return err;
}

static int sf_ctl_init_irq(void)
{
    int err = 0;
    struct device_node *dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter. SF_PIN_STATE_INT_SET = %d\n", __FUNCTION__, SF_PIN_STATE_INT_SET);

    /* Initialize the INT pin. */
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_INT_SET]);

    /* Get the irq number. */
    dev_node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }
    sf_ctl_dev.irq_num = irq_of_parse_and_map(dev_node, 0);
    xprintk(KERN_INFO, "irq number is %d.\n", sf_ctl_dev.irq_num);

    /* Register interrupt callback. */
    err = request_irq(sf_ctl_dev.irq_num, sf_ctl_device_irq,
        IRQF_TRIGGER_FALLING, "sf-irq", (void*)&sf_ctl_dev);
    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    return err;
}

static int sf_ctl_init_input(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    sf_ctl_dev.input = input_allocate_device();
    if (!sf_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }
    sf_ctl_dev.input->name = "sf-keys";

    __set_bit(EV_KEY  , sf_ctl_dev.input->evbit );
    __set_bit(KEY_HOME, sf_ctl_dev.input->keybit);
    __set_bit(KEY_MENU, sf_ctl_dev.input->keybit);
    __set_bit(KEY_BACK, sf_ctl_dev.input->keybit);
	__set_bit(KEY_F11, sf_ctl_dev.input->keybit);
	__set_bit(KEY_ENTER  , sf_ctl_dev.input->evbit );
    __set_bit(KEY_UP, sf_ctl_dev.input->keybit);
    __set_bit(KEY_LEFT, sf_ctl_dev.input->keybit);
    __set_bit(KEY_RIGHT, sf_ctl_dev.input->keybit);
	__set_bit(KEY_DOWN, sf_ctl_dev.input->keybit);
	__set_bit(KEY_WAKEUP, sf_ctl_dev.input->keybit);
	//liukangping add key begin 20170316
	__set_bit(KEY_DEL_EOL, sf_ctl_dev.input->keybit);
	__set_bit(KEY_DEL_EOS, sf_ctl_dev.input->keybit);
	__set_bit(KEY_INS_LINE, sf_ctl_dev.input->keybit);
	#define GF_INPUT_SINGLE_CLICK_KEY  
#define GF_INPUT_DOUBLE_CLICK_KEY  
#define GF_INPUT_LONG_PRESS_KEY  
//liukangping add key end 20170316
    err = input_register_device(sf_ctl_dev.input);
    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(sf_ctl_dev.input);
        sf_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init sf_ctl_driver_init(void)
{
	int err = 0;
	uint64_t sf_vendor_id = 0x00;
	
	get_t_device_id(&sf_vendor_id);
	xprintk(KERN_ERR, "%s sf_vendor_id = 0x%llx\n", __func__, sf_vendor_id);
	if(sf_vendor_id != 0x02) {
		xprintk(KERN_ERR, "%s, Failed to read ic id, 0x%llx\n", __func__, sf_vendor_id);
		return -EINVAL;
	}
	
	/* Initialize the GPIO pins. */
	err = spi_register_driver(&sf_spi_driver); 
	if (err < 0) {
		xprintk(KERN_ERR, "%s, Failed to register SPI driver.\n", __func__);
		return -EINVAL;
	} 
	xprintk(KERN_ERR, "%s spi register success", __func__);
	
	err = sf_ctl_init_gpio_pins();
    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_gpio_pins failed with %d.\n", err);
        return err;
    }


    /* Initialize the interrupt callback. */
    err = sf_ctl_init_irq();
    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_irq failed with %d.\n", err);
        return err;
    }

    /* Initialize the input subsystem. */
    err = sf_ctl_init_input();
    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_input failed with %d.\n", err);

        free_irq(sf_ctl_dev.irq_num, (void*)&sf_ctl_dev);
        return err;
    }

   // err = sf_ctl_device_power(true); // lanh 2016-11-21

	/* Register as a miscellaneous device. */
	err = misc_register(&sf_ctl_dev.miscdev);
	if (err) {
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);

        input_unregister_device(sf_ctl_dev.input);
        free_irq(sf_ctl_dev.irq_num, (void*)&sf_ctl_dev);
    	return err;
    }
    INIT_WORK(&sf_ctl_dev.work_queue, sf_ctl_device_event);

    //err = sf_spi_platform_init();
	xprintk(KERN_INFO, "sunwave fingerprint device control driver registered.\n");
	xprintk(KERN_INFO, "driver version: '%s'. err = %d\n", sf_ctl_get_version(), err);
	return err;
}

static void __exit sf_ctl_driver_exit(void)
{
    if (sf_ctl_dev.input) {
        input_unregister_device(sf_ctl_dev.input);
    }

    if (sf_ctl_dev.irq_num >= 0) {
    	free_irq(sf_ctl_dev.irq_num, (void*)&sf_ctl_dev);
    }
	misc_deregister(&sf_ctl_dev.miscdev);

	//sf_spi_platform_exit();
	xprintk(KERN_INFO, "sunwave fingerprint device control driver released.\n");
}

module_init(sf_ctl_driver_init);
module_exit(sf_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for sunwave's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Langson L. <liangzh@sunwavecorp.com>");

