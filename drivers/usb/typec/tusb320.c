/*
 * tusb320.c -- TI TUSB320 USB TYPE-C Controller device driver
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

/*#include <linux/platform_device.h> */
/*#include <linux/workqueue.h> */
/*#include <linux/switch.h> */
#include <linux/timer.h>
/*#include <linux/wakelock.h> */
#include <linux/delay.h>
#include <linux/types.h>


/******************************************************************************
* Register addresses
******************************************************************************/
/* 0x00 - 0x07 chip id */
#define REG_MOD                     0x08
#define REG_INT                     0x09
#define REG_SET                     0x0A
#define REG_CTL                     0X45

/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_MOD (0x08)    */
#define MOD_ACTIVE_CABLE_DETECTION  0X01    /*RU*/
#define MOD_ACCESSORY_CONNECTED_SHIFT   1
#define MOD_ACCESSORY_CONNECTED         (0x07 << MOD_ACCESSORY_CONNECTED_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_DETECT_SHIFT    4
#define MOD_CURRENT_MODE_DETECT         (0x03 << MOD_CURRENT_MODE_DETECT_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_ADVERTISE_SHIFT    6
#define MOD_CURRENT_MODE_ADVERTISE          (0x03 << MOD_CURRENT_MODE_ADVERTISE_SHIFT)    /*RW*/

/*    REG_INT (0x09)    */
#define INT_DRP_DUTY_CYCLE_SHIFT    1
#define INT_DRP_DUTY_CYCLE          (0x03 << INT_DRP_DUTY_CYCLE_SHIFT)      /*RW*/
#define INT_VCONN_FAULT_SHIFT       3
#define INT_VCONN_FAULT             (0x01 << INT_VCONN_FAULT_SHIFT)         /*RCU*/
#define INT_INTERRUPT_STATUS_SHIFT  4
#define INT_INTERRUPT_STATUS        (0x01 << INT_INTERRUPT_STATUS_SHIFT)    /*RCU*/
#define INT_CABLE_DIR_SHIFT         5
#define INT_CABLE_DIR               (0x01 << INT_CABLE_DIR_SHIFT)           /*RU*/
#define INT_ATTACHED_STATE_SHIFT    6
#define INT_ATTACHED_STATE          (0x03 << INT_ATTACHED_STATE_SHIFT)      /*RU*/

/*    REG_SET (0x0A)    */
#define SET_I2C_SOFT_RESET_SHIFT    3
#define SET_I2C_SOFT_RESET          (0x01 << SET_I2C_SOFT_RESET_SHIFT)  /*RSU*/
#define SET_MODE_SELECT_SHIFT       4
#define SET_MODE_SELECT             (0x03 << SET_MODE_SELECT_SHIFT)     /*RW*/
#define SET_DEBOUNCE_SHIFT          6
#define SET_DEBOUNCE                (0x03 << SET_DEBOUNCE_SHIFT)        /*RW*/

/*    REG_CTR (0x45)    */
#define CTR_DISABLE_RD_RP_SHIFT     2
#define CTR_DISABLE_RD_RP           (0x01 << CTR_DISABLE_RD_RP_SHIFT)   /*RW*/

/******************************************************************************
* Register values
******************************************************************************/
/* SET_MODE_SELECT */
#define SET_MODE_SELECT_HARDWARE  0x00
#define SET_MODE_SELECT_SNK       0x01
#define SET_MODE_SELECT_SRC       0x02
#define SET_MODE_SELECT_DRP       0x03
/* MOD_CURRENT_MODE_ADVERTISE */
#define MOD_CURRENT_MODE_ADVERTISE_DEFAULT      0x00
#define MOD_CURRENT_MODE_ADVERTISE_MID          0x01
#define MOD_CURRENT_MODE_ADVERTISE_HIGH         0x02
/* MOD_CURRENT_MODE_DETECT */
#define MOD_CURRENT_MODE_DETECT_DEFAULT      0x00
#define MOD_CURRENT_MODE_DETECT_MID          0x01
#define MOD_CURRENT_MODE_DETECT_ACCESSARY    0x02
#define MOD_CURRENT_MODE_DETECT_HIGH         0x03


/******************************************************************************
* Constants
******************************************************************************/

enum drp_toggle_type {
	TOGGLE_DFP_DRP_30 = 0,
	TOGGLE_DFP_DRP_40,
	TOGGLE_DFP_DRP_50,
	TOGGLE_DFP_DRP_60
};

enum current_adv_type {
	HOST_CUR_USB = 0,   /*default 500mA or 900mA*/
	HOST_CUR_1P5,      /*1.5A*/
	HOST_CUR_3A       /*3A*/
};

enum current_det_type {
	DET_CUR_USB = 0,    /*default 500mA or 900mA*/
	DET_CUR_1P5,
	DET_CUR_ACCESSORY,  /*charg through accessory 500mA*/
	DET_CUR_3A
};

enum accessory_attach_type {
	ACCESSORY_NOT_ATTACHED = 0,
	ACCESSORY_AUDIO = 4,
	ACCESSORY_CHG_THRU_AUDIO = 5,
	ACCESSORY_DEBUG = 6
};

enum cable_attach_type {
		CABLE_NOT_ATTACHED = 0,
		CABLE_ATTACHED
};

enum cable_state_type {
		CABLE_STATE_NOT_ATTACHED = 0,
		CABLE_STATE_AS_DFP,
		CABLE_STATE_AS_UFP,
		CABLE_STATE_TO_ACCESSORY
};

enum cable_dir_type {
		ORIENT_CC2 = 0,
		ORIENT_CC1
};

enum vconn_fault_type {
		VCONN_NO_FAULT = 0,
		VCONN_FAULT
};

enum cc_modes_type {
		MODE_UNKNOWN = 0,
		MODE_PORT_PIN,   /*According to hardware port pin*/
		MODE_UFP,
		MODE_DFP,
		MODE_DRP
};

/* Type-C Attrs */
struct type_c_parameters {
		enum current_det_type current_det;         /*charging current on UFP*/
		enum accessory_attach_type accessory_attach;     /*if an accessory is attached*/
		enum cable_attach_type active_cable_attach;         /*if an active_cable is attached*/
		enum cable_state_type attach_state;        /*DFP->UFP or UFP->DFP*/
		enum cable_dir_type cable_dir;           /*cc1 or cc2*/
		enum vconn_fault_type vconn_fault;         /*vconn fault*/
};

struct tusb320_info {
		struct i2c_client  *i2c;
		struct device  *dev_t;
		struct mutex  mutex;
		struct class  *device_class;

		struct pinctrl  *pinctrl;
		struct pinctrl_state  *cc_int_cfg;
		struct pinctrl_state  *enpin_output0;
		struct pinctrl_state  *enpin_output1;

		int irq_gpio;

		struct type_c_parameters type_c_param;
};

extern int usb_typec_id;
static char tusb_id[8] = {0x30, 0x32, 0x33, 0x42, 0x53, 0x55, 0x54, 0x00};

/* i2c operate interfaces */
static int tusb320_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
		struct tusb320_info *info = i2c_get_clientdata(i2c);
		int ret;

		mutex_lock(&info->mutex);
		ret = i2c_smbus_read_byte_data(i2c, reg);
		mutex_unlock(&info->mutex);
		if (ret < 0) {
			pr_err("tusb320_read reg(0x%x) error(%d)\n", reg, ret);
			return ret;
		}

		ret &= 0xff;
		*dest = ret;
		return 0;
}

/*
static int tusb320_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
		struct tusb320_info *info = i2c_get_clientdata(i2c);
		int ret;

		mutex_lock(&info->mutex);
		ret = i2c_smbus_write_byte_data(i2c, reg, value);
		mutex_unlock(&info->mutex);
		if (ret < 0)
			pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);

		return ret;
}
*/
static int tusb320_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
		struct tusb320_info *info = i2c_get_clientdata(i2c);
		int ret;
		u8 old_val, new_val;

		mutex_lock(&info->mutex);
		ret = i2c_smbus_read_byte_data(i2c, reg);

		if (ret >= 0) {
			old_val = ret & 0xff;
			new_val = (val & mask) | (old_val & (~mask));
			ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
		}
		mutex_unlock(&info->mutex);
		return ret;
}

#if 0
static int tusb320_read_i2c_block(struct i2c_client *i2c, u8 offset, u16 count, u8 *values)
{
	int ret;
	u8 device_id = 0x47<<1;
	struct i2c_msg msg;

	int i;

	msg.flags = i2c->flags & I2C_M_TEN;
	msg.addr = device_id >> 1;
	msg.buf = &offset;
	msg.len = 1;
	#ifdef CONFIG_MTK_I2C_EXTENSION
	msg.timing = i2c->timing;
	msg.ext_flag = i2c->ext_flag;
	#endif

	for (i = 0; i < 5; i++) {
		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret != 1) {
			pr_err("%s:%d I2c read failed, retry 0x%02x:0x%02x\n", __func__, __LINE__,
			       device_id, offset);
			msleep(20);
		} else {
			break;
		}
	}
	msg.flags = i2c->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.addr = device_id >> 1;
	msg.buf = values;
	msg.len = count;
	#ifdef CONFIG_MTK_I2C_EXTENSION
	msg.timing = i2c->timing;
	msg.ext_flag = i2c->ext_flag;
	#endif

	for (i = 0; i < 5; i++) {
		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret != 1) {
			pr_err("%s:%d I2c read failed, retry 0x%02x:0x%02x\n", __func__, __LINE__,
			       device_id, offset);
			msleep(20);
		} else {
			break;
		}
	}
	#if 0 /* it is ok */
	int ret;

	ret = i2c_master_send(i2c, &offset, 1);
	ret = i2c_master_recv(i2c, values, 1);
	#endif
	return ret;
}

int tusb320__write_i2c_block(struct i2c_client *i2c, u8 offset, u16 count, const u8 *values)
{
	int ret;
	u8 device_id = 0x47<<1;

	struct i2c_msg msg;
	u8 *buffer;

	int i;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		/*pr_debug(KERN_ERR, "%s:%d buffer allocation failed\n",
		   __func__, __LINE__); */
		return -ENOMEM;
	}
	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = i2c->flags & I2C_M_TEN;
	msg.addr = device_id >> 1;
	msg.buf = buffer;
	msg.len = count + 1;
	#ifdef CONFIG_MTK_I2C_EXTENSION
	msg.timing = i2c->timing;
	msg.ext_flag = i2c->ext_flag;
	#endif


	for (i = 0; i < 5; i++) {
		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret != 1) {
			pr_err("%s:%d I2c write failed, retry 0x%02x:0x%02x\n", __func__, __LINE__,
			       device_id, offset);
			ret = -EIO;
			msleep(20);
		} else {
			ret = 0;
			break;
		}
	}

	kfree(buffer);
	#if 0
	int ret;
	u8 *buffer;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		/*pr_debug(KERN_ERR, "%s:%d buffer allocation failed\n",
		   __func__, __LINE__); */
		return -ENOMEM;
	}
	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	ret = i2c_master_send(i2c, buffer, count);
	#endif
	return ret;
}
#endif

/* Config DFP/UFP/DRP mode */
/* e.g #echo 1 >/sys/class/type-c/tusb320/mode_select */
static ssize_t mode_select_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
		struct tusb320_info *info = dev_get_drvdata(dev);
		u8 value;
		int ret;

		ret = tusb320_read_reg(info->i2c, REG_SET, &value);
		if (ret < 0) {
			pr_err("mode_select_show read reg fail!\n");
			return ret;
		}
		value = (value & SET_MODE_SELECT) >> SET_MODE_SELECT_SHIFT;
		return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t mode_select_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
		struct tusb320_info *info = dev_get_drvdata(dev);
		int mode;
		u8 value;
		int ret;

		if (sscanf(buf, "%d", &mode) != 1)
			return -EINVAL;

		if (mode == MODE_DFP)
			value = SET_MODE_SELECT_SRC;
		else if (mode == MODE_UFP)
			value = SET_MODE_SELECT_SNK;
		else if (mode == MODE_DRP)
			value = SET_MODE_SELECT_DRP;
		else if (mode == MODE_PORT_PIN)
			value = SET_MODE_SELECT_HARDWARE;
		else
			return -EINVAL;

		value = value << SET_MODE_SELECT_SHIFT;

		ret = tusb320_update_reg(info->i2c, REG_SET, value, SET_MODE_SELECT);
		if (ret < 0) {
			pr_err("mode_select_store update reg fail!\n");
		}
		return ret;
}


/* Advertise current when act as DFP */
static ssize_t current_advertise_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
		struct tusb320_info *info = dev_get_drvdata(dev);
		u8 value;
		int ret;

		ret = tusb320_read_reg(info->i2c, REG_MOD, &value);
		if (ret < 0) {
			pr_err("current_advertise_show read reg fail!\n");
			return ret;
		}
		value = (value & MOD_CURRENT_MODE_ADVERTISE) >> MOD_CURRENT_MODE_ADVERTISE_SHIFT;
		return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t current_advertise_store(struct device *dev,
						struct device_attribute *attr, const char *buf, size_t size)
{
		struct tusb320_info *info = dev_get_drvdata(dev);
		int mode;
		u8 value;
		int ret;

		if (sscanf(buf, "%d", &mode) != 1)
			return -EINVAL;

		if (mode == HOST_CUR_USB)
			value = MOD_CURRENT_MODE_ADVERTISE_DEFAULT;
		else if (mode == HOST_CUR_1P5)
			value = MOD_CURRENT_MODE_ADVERTISE_MID;
		else if (mode == HOST_CUR_3A)
			value = MOD_CURRENT_MODE_ADVERTISE_HIGH;
		else
			return -EINVAL;

		value = value << MOD_CURRENT_MODE_ADVERTISE_SHIFT;

		ret = tusb320_update_reg(info->i2c, REG_MOD, value, MOD_CURRENT_MODE_ADVERTISE);
		if (ret < 0) {
			pr_err("current_advertise_store update reg fail!\n");
		}
		return ret;
}

/* Detct current when act as UFP */
static ssize_t current_detect_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
		struct tusb320_info *info = dev_get_drvdata(dev);
		u8 value;
		int ret;

		ret = tusb320_read_reg(info->i2c, REG_MOD, &value);
		if (ret < 0) {
			pr_err("current_detect_show read reg fail!\n");
			return ret;
		}
		value = (value & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT;

		if (value == MOD_CURRENT_MODE_DETECT_DEFAULT)
			return snprintf(buf, PAGE_SIZE, "500mA or 900mA\n");
		else if (value == MOD_CURRENT_MODE_DETECT_MID)
			return snprintf(buf, PAGE_SIZE, "mid 1P5A\n");
		else if (value == MOD_CURRENT_MODE_DETECT_HIGH)
			return snprintf(buf, PAGE_SIZE, "high 3A\n");
		else if (value == MOD_CURRENT_MODE_DETECT_ACCESSARY)
			return snprintf(buf, PAGE_SIZE, "accessary 500mA\n");
		else
			return snprintf(buf, PAGE_SIZE, "unknown\n");
}

/**************************************************************************/
#define TYPE_C_ATTR(field, format_string)    \
static ssize_t      \
field ## _show(struct device *dev, struct device_attribute *attr,   \
			char *buf)               \
{                                   \
		struct tusb320_info *info = dev_get_drvdata(dev); \
		return snprintf(buf, PAGE_SIZE,     \
			format_string, info->type_c_param.field);   \
}                   \
static DEVICE_ATTR(field, S_IRUGO, field ## _show, NULL);

static DEVICE_ATTR(mode_select, S_IRUGO | S_IWUSR,
						mode_select_show, mode_select_store);
static DEVICE_ATTR(current_advertise, S_IRUGO | S_IWUSR,
						current_advertise_show, current_advertise_store);
static DEVICE_ATTR(current_det_string, S_IRUGO, current_detect_show, NULL);

TYPE_C_ATTR(current_det, "%d\n")
TYPE_C_ATTR(accessory_attach, "%d\n")
TYPE_C_ATTR(active_cable_attach, "%d\n")
TYPE_C_ATTR(attach_state, "%d\n")
TYPE_C_ATTR(cable_dir, "%d\n")
TYPE_C_ATTR(vconn_fault, "%d\n")

static struct device_attribute *usb_typec_attributes[] = {
		&dev_attr_mode_select,
		&dev_attr_current_advertise,
		&dev_attr_current_det_string,
		&dev_attr_current_det,
		&dev_attr_accessory_attach,
		&dev_attr_active_cable_attach,
		&dev_attr_attach_state,
		&dev_attr_cable_dir,
		&dev_attr_vconn_fault,
		/*end*/
		NULL
};
/******************************************************************************/

static void process_mode_register(struct tusb320_info *info, u8 status)
{
		u8 val;
		u8 tmp = status;

		/* check current_detect */
		val = ((tmp & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT);
		info->type_c_param.current_det = val;

		/* check accessory attch */
		tmp = status;
		val = ((tmp & MOD_ACCESSORY_CONNECTED) >> MOD_ACCESSORY_CONNECTED_SHIFT);
		info->type_c_param.accessory_attach = val;

		/* check cable attach */
		tmp = status;
		val = (tmp & MOD_ACTIVE_CABLE_DETECTION);
		info->type_c_param.active_cable_attach = val;
}

static void process_interrupt_register(struct tusb320_info *info, u8 status)
{
		u8 val;
		u8 tmp = status;

		/* check attach state */
		val = ((tmp & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT);
		info->type_c_param.attach_state = val;

		/* check cable dir */
		tmp = status;
		val = ((tmp & INT_CABLE_DIR) >> INT_CABLE_DIR_SHIFT);
		info->type_c_param.cable_dir = val;

		/* check vconn fault */
		tmp = status;
		val = ((tmp & INT_VCONN_FAULT) >> INT_VCONN_FAULT_SHIFT);
		info->type_c_param.vconn_fault = val;
}

static irqreturn_t tusb320_irq_thread(int irq, void *handle)
{
		struct tusb320_info *info = (struct tusb320_info *)handle;
		u8 mod_val = 0, int_val = 0;
		int ret;

		ret = tusb320_read_reg(info->i2c, REG_MOD, &mod_val);
		if (ret)
			goto done;

		process_mode_register(info, mod_val);

		ret = tusb320_read_reg(info->i2c, REG_INT, &int_val);
		if (ret)
			goto done;

		process_interrupt_register(info, int_val);

		tusb320_update_reg(info->i2c, REG_INT, (0x1 << INT_INTERRUPT_STATUS_SHIFT), INT_INTERRUPT_STATUS);
done:
		pr_info("tusb320_irq mod(0x%x) int(0x%x)\n", mod_val, int_val);
		return IRQ_HANDLED;
}

#if 0
static int  tusb320_suspend(struct i2c_client *client, pm_message_t message)
{
		/* PM interface, fix me if need */
		return 0;
}

static int  tusb320_resume(struct i2c_client *client)
{
		/* PM interface, fix me if need */
		return 0;
}
#endif
static int tusb320_pinctrl_init(struct tusb320_info *info)
{
		struct pinctrl_state *set_state;

		info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
		if (IS_ERR_OR_NULL(info->pinctrl)) {
			pr_err("%s: pinctrl not defined\n", __func__);
			return PTR_ERR(info->pinctrl);
		}

		set_state = pinctrl_lookup_state(info->pinctrl, "tusb320_intpin");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("%s: tusb320_intpin lookup failed\n", __func__);
			info->pinctrl = NULL;
			return PTR_ERR(set_state);
		}
		info->cc_int_cfg = set_state;

		set_state = pinctrl_lookup_state(info->pinctrl, "tusb320_enpin_output0");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("%s: tusb320_enpin_output0 lookup failed\n", __func__);
			info->enpin_output0 = NULL;
			return PTR_ERR(set_state);
		}
		info->enpin_output0 = set_state;

		set_state = pinctrl_lookup_state(info->pinctrl, "tusb320_enpin_output1");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("%s: tusb320_enpin_output1 lookup failed\n", __func__);
			info->enpin_output1 = NULL;
			return PTR_ERR(set_state);
		}
		info->enpin_output1 = set_state;

		/* get more pins conig here, if need */
		return 0;
}

static int tusb320_eint_init(struct tusb320_info *info)
{
	int retval;
	struct device_node *node;
	/*int eint_gpio, eint_debounce;*/
	/*u32 ints[2] = {0, 0};*/
	int irq;

	node = of_find_compatible_node(NULL, NULL, "ti,tusb320");
	if (node) {
		node = of_find_compatible_node(NULL, NULL, "ti,tusb320_eint");
		if (node) {
			/*from mtk to qcom change,++*/
			/* irq_line */
			retval = of_get_named_gpio(node, "tusb320,irq_gpio", 0);
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
			    pr_info("tusb320 eint_gpio(%d) irqnum(%d)\n", eint_gpio, info->irq_gpio);

			}
			#endif
/*from mtk to qcom change,-- */
		}
	} else {
		pr_info("tusb320 cannot get the node\n");
		return 1;
	}

/*from mtk to qcom change,++ */
		irq = gpio_to_irq(info->irq_gpio);

	if (irq < 0) {
		pr_err("%s: error gpio_to_irq returned %d\n", __func__, irq);
		return 1;
	}

		info->irq_gpio = irq;

/*from mtk to qcom change,-- */
	retval = request_threaded_irq(info->irq_gpio, NULL, tusb320_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "tusb320_irq", info);
	if (retval != 0) {
		pr_info("tusb320 request_irq fail, ret %d, irqnum %d!!!\n", retval, info->irq_gpio);
		return 1;
	}
	return 0;
}

static void tusb320_enable(struct tusb320_info *info, int value)
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

static int tusb320_check_id(struct tusb320_info *info)
{
	u8 reg_val = 0;
	int i;

	for (i = 0; i < 7; i++) {
		reg_val = 0;
		tusb320_read_reg(info->i2c, i, &reg_val);
		if (tusb_id[i] != reg_val) {
			pr_info("tusb320_check_id fail\n");
			return -ENODEV;
		}
	}
	pr_info("tusb320_check_id success\n");
	return 0;
}

static int tusb320_initialization(struct tusb320_info *info)
{
	int ret = 0;
	u8 reg_val = 0;

    /* do initialization here, before enable irq,
     * clear irq,
     * config DRP/UFP/DFP mode,
     * and etc..
     */

	ret = tusb320_read_reg(info->i2c, REG_MOD, &reg_val);
	return ret;
}

static int tusb320_create_device(struct tusb320_info *info)
{
		struct device_attribute **attrs = usb_typec_attributes;
		struct device_attribute *attr;
		int err;

		info->device_class = class_create(THIS_MODULE, "type-c");
		if (IS_ERR(info->device_class))
			return PTR_ERR(info->device_class);

		info->dev_t = device_create(info->device_class, NULL, 0, NULL, "tusb320");
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

static void tusb320_destroy_device(struct tusb320_info *info)
{
		struct device_attribute **attrs = usb_typec_attributes;
		struct device_attribute *attr;

		while ((attr = *attrs++))
			device_remove_file(info->dev_t, attr);

		device_destroy(info->device_class, 0);
		class_destroy(info->device_class);
		info->device_class = NULL;
}

static int tusb320_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
		struct tusb320_info *info;
		/*struct device_node *np = client->dev.of_node;*/
		int ret;

	if (usb_typec_id != 0) {
		pr_info("no tusb320\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct tusb320_info), GFP_KERNEL);

	if (!info) {
		pr_err("%s: error kzalloc\n", __func__);
		return -ENOMEM;
	}
	info->i2c = client;

	/* initialize pinctrl */
	if (!tusb320_pinctrl_init(info)) {
		ret  = pinctrl_select_state(info->pinctrl, info->cc_int_cfg);
		if (ret) {
			pr_err("%s: error initialize pinctrl\n", __func__);
			goto err_pinctrl;
		}
	}
	tusb320_enable(info, 1);

	/* config more gpio(s) here, if need */

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	/* create device and sysfs nodes */
	ret = tusb320_create_device(info);
	if (ret) {
		pr_err("%s: create device failed\n", __func__);
		goto err_device_create;
	}

	ret = tusb320_check_id(info);
	if (ret != 0) {
		pr_err("%s: tusb320_check_id fail\n", __func__);
		goto err_device_init;
	}

	ret = tusb320_initialization(info);
	if (ret < 0) {
		pr_err("%s: fails to do initialization %d\n", __func__, ret);
		goto err_device_init;
	}

	if (!tusb320_eint_init(info)) {
		pr_info("tusb320_eint_init success\n");
	} else {
		dev_err(&client->dev, "error failed to request IRQ\n");
		ret = -ENODEV;
		goto err_device_init;
	}

	usb_typec_id = 2;
	pr_info("tusb320 usb type-c ship finish probe\n");
	return 0;

err_device_init:
	tusb320_destroy_device(info);
err_device_create:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
err_pinctrl:
	kfree(info);
	info = NULL;
	return ret;
}

static int tusb320_remove(struct i2c_client *client)
{
	struct tusb320_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}

	tusb320_destroy_device(info);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);

	kfree(info);
	return 0;
}

extern void otg_dis_vbus(void);
static void  tusb320_shutdown(struct i2c_client *client)
{
	/* PM interface, fix me if need */
	/*struct tusb320_info *info = i2c_get_clientdata(client);
	*pr_info("tusb320_shutdown\n");
	*disable_irq(info->irq_gpio);
	*otg_dis_vbus();
	*tusb320_enable(info,0);
	*/
}


static const struct of_device_id tusb320_dt_match[] = {
	{
		.compatible = "ti,tusb320",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tusb320_dt_match);

static const struct i2c_device_id tusb320_id_table[] = {
	{
		.name = "tusb320",
	},
};

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.of_match_table = of_match_ptr(tusb320_dt_match),
	},
	.probe    = tusb320_probe,
	.remove   = tusb320_remove,
	/*.suspend  = tusb320_suspend,*/
	/*.resume  = tusb320_resume,*/
	.shutdown = tusb320_shutdown,
	.id_table = tusb320_id_table,
};

static __init int tusb320_i2c_init(void)
{
	return i2c_add_driver(&tusb320_i2c_driver);
}

static __exit void tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}

module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for TUSB320 USB Type-C");
MODULE_LICENSE("GPL v2");
