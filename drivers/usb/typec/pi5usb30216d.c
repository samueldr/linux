/*
 * PI5USB30216D.c -- PI5USB30216D USB TYPE-C Controller device driver
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/errno.h>
#include <linux/err.h>

/*#include <linux/platform_device.h>*/
/*#include <linux/workqueue.h>*/
/*#include <linux/switch.h>*/
#include <linux/timer.h>
/*#include <linux/wakelock.h>*/
#include <linux/delay.h>
#include <linux/types.h>

/******************************************************************************
* Register addresses
******************************************************************************/
/* 0x00 - 0x07 chip id */
#define REG_CTR                     0x02
#define REG_INT                     0x03
#define REG_CC                      0x04

/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_CTR (0x02)    */
#define MOD_POWERSAVING_STATE_SHIFT    7
#define MOD_POWERSAVING_STATE          (0x01 << MOD_POWERSAVING_STATE_SHIFT)
#define MOD_PORTSET_STATE_SHIFT    1
#define MOD_PORTSET_STATE          (0x03 << MOD_POWERSAVING_STATE_SHIFT)

/*    REG_INT (0x03)    */
#define ATTACHED_STATE_SHIFT    0
#define ATTACHED_STATE          (0x01 << ATTACHED_STATE_SHIFT)
#define DETACHED_STATE_SHIFT    1
#define DETACHED_STATE          (0x01 << DETACHED_STATE_SHIFT)

/*    REG_CC (0x04)    */
#define MOD_CC_CONNECT_STATE_SHIFT    0
#define MOD_CC_CONNECT_STATE          (0x03 << MOD_CC_CONNECT_STATE_SHIFT)
#define MOD_PORT_STATE_SHIFT    2
#define MOD_PORT_STATE          (0x07 << MOD_PORT_STATE_SHIFT)
#define MOD_VBUS_STATE_SHIFT    7
#define MOD_VBUS_STATE          (0x01 << MOD_VBUS_STATE_SHIFT)

/******************************************************************************
* Constants
******************************************************************************/
enum cable_attach_state {
	CABLE_NOT_ATTACHED = 0,
	CABLE_ATTACHED
};

enum cable_state {
	CABLE_STATE_NOT_ATTACHED = 0,
	CABLE_STATE_AS_DFP,
	CABLE_STATE_AS_UFP,
	CABLE_STATE_TO_ACCESSORY
};

enum cable_dir_state {
	ORIENT_CC2 = 0,
	ORIENT_CC1
};

/* Type-C Attrs */
struct typec_parameters {
	enum cable_attach_state active_cable_attach;	/*if an active_cable is attached */
	enum cable_state attach_state;	/*DFP->UFP or UFP->DFP */
	enum cable_dir_state cable_dir;	/*cc1 or cc2 */
};

struct PI5USB30216D_info {
	struct i2c_client *i2c;
	struct device *dev_t;
	struct mutex mutex;
	struct class *device_class;

	struct pinctrl *pinctrl;
	struct pinctrl_state *cc_int_cfg;
	struct pinctrl_state *enpin_output0;
	struct pinctrl_state *enpin_output1;

	int irq_gpio;

	struct typec_parameters type_c_param;
};

int usb_typec_id = 0;
static char pi5usb302_id = 0x20;
static char pi5usb302_reg_data[4] = { 0x00 };

/* i2c operate interfaces */
static int PI5USB30216D_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct PI5USB30216D_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("PI5USB30216D_read reg(0x%x) error(%d)\n", reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int PI5USB30216D_read_blockdata(struct i2c_client *i2c, u8 reg, u8 length, u8 *dest)
{
	struct PI5USB30216D_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_master_recv(i2c, dest, length);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("PI5USB30216D_read_blockdata reg(0x%x) error(%d)\n", reg, ret);
		return ret;
	}
#if 0
	pr_debug("PI5USB30216D_read_blockdata ret(%d) length(%d)\n", ret, length);
	{
		int i = 0;

		for (i = 0; i < length; i++)
			pr_debug("PI5USB30216D_read_blockdata data(%x)\n", dest[i]);
	}
#endif
	return ret;
}

static int PI5USB30216D_write_blockdata(struct i2c_client *i2c, u8 reg, u8 length, u8 *dest)
{
	struct PI5USB30216D_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_master_send(i2c, dest, length);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
			pr_err("PI5USB30216D_write_blockdata reg(0x%x) error(%d)\n", reg, ret);
		return ret;
	}
	/*pr_debug("PI5USB30216D_write_blockdata ret(%d) length(%d)\n",ret,length); */
	return ret;
}

/* Config DFP/UFP/DRP mode */
static ssize_t reg_val_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct PI5USB30216D_info *info = dev_get_drvdata(dev);
	int ret;

	ret = PI5USB30216D_read_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);
	if (ret < 0) {
			pr_err("mode_select_show read reg fail!\n");
			return ret;
	}
	return snprintf(buf, PAGE_SIZE, "%x %x %x %x\n",
		pi5usb302_reg_data[0], pi5usb302_reg_data[1],
		pi5usb302_reg_data[2], pi5usb302_reg_data[3]);
}

static ssize_t reg_val_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t size)
{
	struct PI5USB30216D_info *info = dev_get_drvdata(dev);
	int ret;

	ret = PI5USB30216D_write_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);
	if (ret < 0) {
			pr_err("mode_select_show read reg fail!\n");
			return ret;
	}
	return size;
}

/**************************************************************************/
#define TYPE_C_ATTR(field, format_string)    \
static ssize_t      \
field ## _show(struct device *dev, struct device_attribute *attr,   \
			char *buf)               \
{                                   \
	struct PI5USB30216D_info *info = dev_get_drvdata(dev); \
	return snprintf(buf, PAGE_SIZE,     \
	format_string, info->type_c_param.field);   \
}                   \
static DEVICE_ATTR(field, S_IRUGO, field ## _show, NULL);

static DEVICE_ATTR(reg_val, S_IRUGO | S_IWUSR,
					reg_val_show, reg_val_store);

static struct device_attribute *usb_typec_attributes[] = {
	&dev_attr_reg_val,
	/*end */
	NULL
};

/******************************************************************************/

static void process_register(struct PI5USB30216D_info *info, u8 *pi5usb302_reg_data)
{
	u8 val = 0, tmp = 0;

	/* check cable attach */
	tmp = pi5usb302_reg_data[2];
	val = tmp & ATTACHED_STATE;
	info->type_c_param.active_cable_attach = val;
	/*pr_debug("INT_ATTACHED_STATE(%x)\n",val); */

	/* check cable dir */
	tmp = pi5usb302_reg_data[3];
	val = tmp & MOD_CC_CONNECT_STATE;
	info->type_c_param.cable_dir = val;
	/*pr_debug("PI5USB30216D:MOD_CC_CONNECT_STATE(%x)\n",val);  0x01: cc1   0x10: cc2   */

	/* check attach state */
	tmp = pi5usb302_reg_data[3];
	val = (tmp & MOD_PORT_STATE) >> MOD_PORT_STATE_SHIFT;
	info->type_c_param.attach_state = val;
	/*pr_debug("PI5USB30216D:MOD_PORT_STATE(%x)\n",val);  0x01: device   0x10: host   */
	pr_debug("PI5USB30216D attach(%x) mode(%x)\n", info->type_c_param.active_cable_attach,
		info->type_c_param.attach_state);
}

static irqreturn_t PI5USB30216D_irq_thread(int irq, void *handle)
{
	struct PI5USB30216D_info *info = (struct PI5USB30216D_info *)handle;
	int ret;

	ret = PI5USB30216D_read_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);

	if (ret < 0)
		return IRQ_HANDLED;

	process_register(info, pi5usb302_reg_data);
	return IRQ_HANDLED;
}

static int PI5USB30216D_pinctrl_init(struct PI5USB30216D_info *info)
{
	struct pinctrl_state *set_state;

	info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		pr_err("%s: pinctrl not defined\n", __func__);
		return PTR_ERR(info->pinctrl);
	}

	set_state = pinctrl_lookup_state(info->pinctrl, "PI5USB30216D_intpin");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("%s: PI5USB30216D_intpin lookup failed\n", __func__);
		info->pinctrl = NULL;
		return PTR_ERR(set_state);
	}
	info->cc_int_cfg = set_state;

	set_state = pinctrl_lookup_state(info->pinctrl, "PI5USB30216D_enpin_output0");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("%s: PI5USB30216D_enpin_output0 lookup failed\n", __func__);
		info->enpin_output0 = NULL;
		return PTR_ERR(set_state);
	}
	info->enpin_output0 = set_state;

	set_state = pinctrl_lookup_state(info->pinctrl, "PI5USB30216D_enpin_output1");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("%s: PI5USB30216D_enpin_output1 lookup failed\n", __func__);
		info->enpin_output1 = NULL;
		return PTR_ERR(set_state);
	}
	info->enpin_output1 = set_state;

	/* get more pins conig here, if need */
	return 0;
}

static int PI5USB30216D_eint_init(struct PI5USB30216D_info *info)
{
	int retval;
	struct device_node *node;
	/*int eint_gpio, eint_debounce; */
	/*u32 ints[2] = {0, 0}; */
	int irq;

	node = of_find_compatible_node(NULL, NULL, "ti,PI5USB30216D");
	if (node) {
		node = of_find_compatible_node(NULL, NULL, "ti,PI5USB30216D_eint");
		if (node) {
/*from mtk to qcom change,++*/
			/* irq_line */
			retval = of_get_named_gpio(node, "pi5usb,irq_gpio", 0);
			if (retval < 0) {
				pr_err("%s: error invalid irq gpio err: %d\n", __func__, retval);
			}
			info->irq_gpio = retval;

#if 0
			retval = of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
			if (!retval) {
				eint_gpio = ints[0];
				eint_debounce = ints[1];
				info->irq_gpio = irq_of_parse_and_map(node, 0);
				pr_debug("PI5USB30216D eint_gpio(%d) irqnum(%d)\n", eint_gpio, info->irq_gpio);
			}
#endif
/*from mtk to qcom change,-- */
		}
	} else {
		pr_debug("PI5USB30216D cannot get the node\n");
		return 1;
	}

/*from mtk to qcom change,++ */
	irq = gpio_to_irq(info->irq_gpio);

	if (irq >= 0) {
		info->irq_gpio = irq;
	} else {
		pr_err("%s: error gpio_to_irq returned %d\n", __func__, irq);
		return 1;
	}
/*from mtk to qcom change,-- */
	retval = request_threaded_irq(info->irq_gpio, NULL, PI5USB30216D_irq_thread,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, "PI5USB30216D_irq", info);
	if (retval != 0) {
		pr_debug("PI5USB30216D request_irq fail, ret %d, irqnum %d!!!\n", retval, info->irq_gpio);
		return 1;
	}
	return 0;
}

static void PI5USB30216D_enable(struct PI5USB30216D_info *info, int value)
{
	if (value) {
		pinctrl_select_state(info->pinctrl, info->enpin_output1);
		mdelay(100);
		/*mdelay(70); */
	} else {
		pinctrl_select_state(info->pinctrl, info->enpin_output0);
		mdelay(1);
	}
}

static int PI5USB30216D_check_id(struct PI5USB30216D_info *info)
{
	u8 reg_val = 0;

	PI5USB30216D_read_reg(info->i2c, 0, &reg_val);
	if (pi5usb302_id != reg_val) {
		pr_debug("PI5USB30216D_check_id fail(%x)\n", reg_val);
		return -ENODEV;
	}
	pr_debug("PI5USB30216D_check_id success\n");
	PI5USB30216D_read_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);
	pi5usb302_reg_data[1] = (pi5usb302_reg_data[1]&(~MOD_PORTSET_STATE))|0x04;
	PI5USB30216D_write_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);
	PI5USB30216D_read_blockdata(info->i2c, 0x00, 0x04, pi5usb302_reg_data);
	return 0;
}

static int PI5USB30216D_create_device(struct PI5USB30216D_info *info)
{
	struct device_attribute **attrs = usb_typec_attributes;
	struct device_attribute *attr;
	int err;

	info->device_class = class_create(THIS_MODULE, "type-c");
	if (IS_ERR(info->device_class))
		return PTR_ERR(info->device_class);

	info->dev_t = device_create(info->device_class, NULL, 0, NULL, "PI5USB30216D");
	if (IS_ERR(info->dev_t))
		return PTR_ERR(info->dev_t);

	dev_set_drvdata(info->dev_t, info);

	while ((attr = *attrs++)) {
		err = device_create_file(info->dev_t, attr);
		if (err) {
			device_destroy(info->device_class, 0);
			return err;
		}
	}
	return 0;
}

static void PI5USB30216D_destroy_device(struct PI5USB30216D_info *info)
{
	struct device_attribute **attrs = usb_typec_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(info->dev_t, attr);

	device_destroy(info->device_class, 0);
	class_destroy(info->device_class);
	info->device_class = NULL;
}

static int PI5USB30216D_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct PI5USB30216D_info *info;
	/*struct device_node *np = client->dev.of_node; */
	int ret = 0;

	if (usb_typec_id != 0) {
		pr_debug("no PI5USB30216D\n");
		return -ENODEV;
	}
	info = kzalloc(sizeof(struct PI5USB30216D_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: error kzalloc\n", __func__);
		return -ENOMEM;
	}
	info->i2c = client;

	/* initialize pinctrl */
	if (!PI5USB30216D_pinctrl_init(info)) {
		ret = pinctrl_select_state(info->pinctrl, info->cc_int_cfg);
		if (ret) {
			pr_err("%s: error initialize pinctrl\n", __func__);
			goto err_pinctrl;
		}
	}
	PI5USB30216D_enable(info, 1);
	/* config more gpio(s) here, if need */

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	/* create device and sysfs nodes */
	ret = PI5USB30216D_create_device(info);
	if (ret) {
		pr_err("%s: create device failed\n", __func__);
		goto err_device_create;
	}
	ret = PI5USB30216D_check_id(info);
	if (ret != 0) {
		pr_err("%s: PI5USB30216D_check_id fail\n", __func__);
		goto err_device_init;
	}

	if (!PI5USB30216D_eint_init(info)) {
		pr_debug("PI5USB30216D_eint_init success\n");
	} else {
		dev_err(&client->dev, "error failed to request IRQ\n");
		ret = -ENODEV;
		goto err_device_init;
	}
	usb_typec_id = 1;
	pr_debug("PI5USB30216D usb type-c ship finish probe\n");
	return 0;

 err_device_init:
	PI5USB30216D_destroy_device(info);
 err_device_create:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
 err_pinctrl:
	kfree(info);
	info = NULL;
	return ret;
}

static int PI5USB30216D_remove(struct i2c_client *client)
{
	struct PI5USB30216D_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}

	PI5USB30216D_destroy_device(info);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);

	kfree(info);
	return 0;
}

extern void otg_dis_vbus(void);
static void PI5USB30216D_shutdown(struct i2c_client *client)
{
	/* PM interface, fix me if need */
	/* struct PI5USB30216D_info *info = i2c_get_clientdata(client);
	 *pr_debug("PI5USB30216D_shutdown\n");
	 *disable_irq(info->irq_gpio);
	 *otg_dis_vbus();
	 *PI5USB30216D_enable(info,0);
	 */
}

static const struct of_device_id PI5USB30216D_dt_match[] = {
	{
	 .compatible = "ti,PI5USB30216D",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, PI5USB30216D_dt_match);

static const struct i2c_device_id PI5USB30216D_id_table[] = {
	{
	 .name = "PI5USB30216D",
	 },
};

static struct i2c_driver PI5USB30216D_i2c_driver = {
	.driver = {
		   .name = "PI5USB30216D",
		   .of_match_table = of_match_ptr(PI5USB30216D_dt_match),
		   },
	.probe = PI5USB30216D_probe,
	.remove = PI5USB30216D_remove,
	.shutdown = PI5USB30216D_shutdown,
	.id_table = PI5USB30216D_id_table,
};

static __init int PI5USB30216D_i2c_init(void)
{
	return i2c_add_driver(&PI5USB30216D_i2c_driver);
}

static __exit void PI5USB30216D_i2c_exit(void)
{
	i2c_del_driver(&PI5USB30216D_i2c_driver);
}

module_init(PI5USB30216D_i2c_init);
module_exit(PI5USB30216D_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for PI5USB30216D USB Type-C");
MODULE_LICENSE("GPL v2");
