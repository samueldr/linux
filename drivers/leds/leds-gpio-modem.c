/*
 * LEDs driver for GPIOs
 *

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <soc/qcom/sysmon.h>

struct gpio_led_data {
	struct led_classdev cdev;
	unsigned gpio;
	struct work_struct work;
	u8 new_level;
	u8 can_sleep;
	u8 active_low;
	u8 blinking;
	int (*platform_gpio_blink_set)(unsigned gpio, int state,
					unsigned long *delay_on,
					unsigned long *delay_off);
};

struct zte_leds_data_type {
	unsigned char gpio;
	unsigned char level;
	unsigned char active_low;
	unsigned char blink;
	unsigned char blink_on;
	unsigned char blink_off;
};

struct zte_leds_qmi_data_type {
	struct zte_leds_data_type red;
	struct zte_leds_data_type green;
	struct zte_leds_data_type blue;
};

/* fix the shutdown dead issue, start*/
static int modem_online_status = 0;
void set_modem_online_value(int v)
{
	pr_info("%s: modem_online_status is set to %d\n", __func__, v);
	modem_online_status = 1;
}

int get_modem_online_status(void)
{
	return modem_online_status;
}
/* fix the shutdown dead issue, end*/

#define BLINK_TIME_QMI_UNIT_MS 50
#define BLINK_TIME_QMI_MAX_MS  (0xFE*BLINK_TIME_QMI_UNIT_MS)
#define LEDS_QMI_DATA_LEN 12

#define QMI_SSCTL_SUBSYS_NAME_LENGTH 15

#define LED_SHOW_BUFFER_MAX_LEN 10
/*0xFF for invalid gpio num*/
struct zte_leds_qmi_data_type zte_gpio_leds_qmi_data;

struct mutex leds_mutex;

/* blink on time is the real time(unit ms) */
void update_leds_qmi_data(const char *name, int gpio, int level, int active_low,
			  int blink, int blink_on_time, int blink_off_time)
{
	struct zte_leds_data_type *data = NULL;

	if (!strcmp(name, "red"))
		data = &(zte_gpio_leds_qmi_data.red);
	else if (!strcmp(name, "green"))
		data = &(zte_gpio_leds_qmi_data.green);
	else if (!strcmp(name, "blue"))
		data = &(zte_gpio_leds_qmi_data.blue);

	if (blink_on_time > BLINK_TIME_QMI_MAX_MS)
		blink_on_time = BLINK_TIME_QMI_MAX_MS;
	if (blink_off_time > BLINK_TIME_QMI_MAX_MS)
		blink_off_time = BLINK_TIME_QMI_MAX_MS;

	if (data) {
		if (gpio_is_valid(gpio))
			data->gpio = gpio - gpio_to_chip(gpio)->base;
		if (level >= 0)
			data->level = level ? 1 : 0;
		if (active_low >= 0)
			data->active_low = active_low ? 1 : 0;
		if (blink >= 0)
			data->blink = blink ? 1 : 0;
		if (blink_on_time >= 0)
			data->blink_on =
			    (blink_on_time + BLINK_TIME_QMI_UNIT_MS -
			     1) / BLINK_TIME_QMI_UNIT_MS;
		if (blink_off_time >= 0)
			data->blink_off =
			    (blink_off_time + BLINK_TIME_QMI_UNIT_MS -
			     1) / BLINK_TIME_QMI_UNIT_MS;
	}
}

void get_leds_qmi_data(const char *name, int *gpio_from_base, int *level,
		       int *active_low, int *blink, int *blink_on_time,
		       int *blink_off_time)
{
	struct zte_leds_data_type *data = NULL;

	if (!strcmp(name, "red"))
		data = &(zte_gpio_leds_qmi_data.red);
	else if (!strcmp(name, "green"))
		data = &(zte_gpio_leds_qmi_data.green);
	else if (!strcmp(name, "blue"))
		data = &(zte_gpio_leds_qmi_data.blue);

	if (data) {
		if (gpio_from_base)
			*gpio_from_base = data->gpio;
		if (level)
			*level = data->level;
		if (active_low)
			*active_low = data->active_low;
		if (blink)
			*blink = data->blink;
		if (blink_on_time)
			*blink_on_time =
			    data->blink_on * BLINK_TIME_QMI_UNIT_MS;
		if (blink_off_time)
			*blink_off_time =
			    data->blink_off * BLINK_TIME_QMI_UNIT_MS;
	}
}

/*buf size if 15 bytes
  * byte 0, red gpio
  * byte 1, green gpio
  * byte 2, blue gpio
  * btye 3, leds level
  * byte 4, leds blink status
  * byte 5 active low status
  * byte 6, red blink on time(unit 50ms)
  * byte 7, red blink off time(unit 50ms)
  * byte 8, green blink on time(unit 50ms)
  * byte 9, green blink off time(unit 50ms)
  * byte 10, blue blink on time(unit 50ms)
  * byte 11, blue blink off time(unit 50ms)
 */
unsigned char qmi_data_exclusive_or_mask[LEDS_QMI_DATA_LEN] = {
	/*byte 0 */ 0xFE, /*1 */ 0xFE, /*2 */ 0xFE, /*3 */ 0x70,
	/*4 */ 0x70, /*5 */ 0x70, /*6 */ 0xFE, /*7 */ 0xFE,
	/*8 */ 0xFE, /*9 */ 0xFE, /*10 */ 0xFE, /*11 */ 0xFE
};

void leds_qmi_data_process_before_send(struct zte_leds_qmi_data_type *data,
				       unsigned char *buf)
{
	struct zte_leds_data_type *red = &(zte_gpio_leds_qmi_data.red);
	struct zte_leds_data_type *green = &(zte_gpio_leds_qmi_data.green);
	struct zte_leds_data_type *blue = &(zte_gpio_leds_qmi_data.blue);
	int i;

	buf[0] = red->gpio;
	buf[1] = green->gpio;
	buf[2] = blue->gpio;
	buf[3] = (red->level) | (green->level << 1) | (blue->level << 2);
	buf[4] = (red->blink) | (green->blink << 1) | (blue->blink << 2);
	buf[5] = (red->active_low) | (green->active_low << 1) | (blue->active_low << 2);
	buf[6] = red->blink_on;	/*unit 50ms */
	buf[7] = red->blink_off;
	buf[8] = green->blink_on;  /*unit 50ms */
	buf[9] = green->blink_off;
	buf[10] = blue->blink_on;	/*unit 50ms */
	buf[11] = blue->blink_off;
	buf[12] = '\0';

	for (i = 0; i < LEDS_QMI_DATA_LEN; i++)
		buf[i] ^= qmi_data_exclusive_or_mask[i];
}

void qmi_send_led_data(void)
{
	unsigned char buf[QMI_SSCTL_SUBSYS_NAME_LENGTH] = { 0, };
	int rc;

	struct zte_leds_data_type *data = &(zte_gpio_leds_qmi_data.red);

	if (data->gpio != 0xFF)
		pr_info
		    ("leds red:gpio %d,  level %d, blink %d, blink_on %d, blink_off %d\n",
		     data->gpio, data->level, data->blink,
		     data->blink_on * BLINK_TIME_QMI_UNIT_MS,
		     data->blink_off * BLINK_TIME_QMI_UNIT_MS);

	data = &(zte_gpio_leds_qmi_data.green);
	if (data->gpio != 0xFF)
		pr_info
		    ("leds green:gpio %d,  level %d, blink %d, blink_on %d, blink_off %d\n",
		     data->gpio, data->level, data->blink,
		     data->blink_on * BLINK_TIME_QMI_UNIT_MS,
		     data->blink_off * BLINK_TIME_QMI_UNIT_MS);

	data = &(zte_gpio_leds_qmi_data.blue);
	if (data->gpio != 0xFF)
		pr_info
		    ("leds blue:gpio %d,  level %d, blink %d, blink_on %d, blink_off %d\n",
		     data->gpio, data->level, data->blink,
		     data->blink_on * BLINK_TIME_QMI_UNIT_MS,
		     data->blink_off * BLINK_TIME_QMI_UNIT_MS);

	leds_qmi_data_process_before_send(&zte_gpio_leds_qmi_data, buf);

	/* fix the shutdown dead issue, start */
	if (get_modem_online_status() == 0) {
		pr_emerg("%s: modem is not online, quit sending LEDS set data\n", __func__);
		return;
	}
	/* fix the shutdown dead issue, end */

	rc = sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET, buf);
}

void leds_qmi_data_process_after_send(unsigned char *buf,
				      struct zte_leds_qmi_data_type *data)
{
	struct zte_leds_data_type *red = &(data->red);
	struct zte_leds_data_type *green = &(data->green);
	struct zte_leds_data_type *blue = &(data->blue);
	int i;

	for (i = 0; i < LEDS_QMI_DATA_LEN; i++)
		buf[i] ^= qmi_data_exclusive_or_mask[i];

	red->gpio = buf[0];
	green->gpio = buf[1];
	blue->gpio = buf[2];

	red->level = (buf[3] & 0x1) ? 1 : 0;
	green->level = (buf[3] & 0x2) ? 1 : 0;
	blue->level = (buf[3] & 0x4) ? 1 : 0;

	red->blink = (buf[4] & 0x1) ? 1 : 0;
	green->blink = (buf[4] & 0x2) ? 1 : 0;
	blue->blink = (buf[4] & 0x4) ? 1 : 0;

	red->active_low = (buf[5] & 0x1) ? 1 : 0;
	green->active_low = (buf[5] & 0x2) ? 1 : 0;
	blue->active_low = (buf[5] & 0x4) ? 1 : 0;

	red->blink_on = buf[6];
	red->blink_off = buf[7];
	green->blink_on = buf[8];
	green->blink_off = buf[9];
	blue->blink_on = buf[10];
	blue->blink_off = buf[11];
}

static void gpio_led_set(struct led_classdev *led_cdev,
			 enum led_brightness value)
{
	struct gpio_led_data *led_dat =
	    container_of(led_cdev, struct gpio_led_data, cdev);
	int level;

	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

	if (led_dat->active_low)
		level = !level;

	/* Setting GPIOs with I2C/etc requires a task context, and we don't
	 * seem to have a reliable way to know if we're already in one; so
	 * let's just assume the worst.
	 */
	mutex_lock(&leds_mutex);

	gpio_set_value(led_dat->gpio, level);
	led_dat->blinking = 0;
	update_leds_qmi_data(led_dat->cdev.name, -1, level, -1, led_dat->blinking, -1, -1);
	qmi_send_led_data();
	mutex_unlock(&leds_mutex);
}

static ssize_t led_blink_solid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int blink = 0;

	get_leds_qmi_data(led_cdev->name, NULL, NULL, NULL, &blink, NULL, NULL);

	return snprintf(buf, LED_SHOW_BUFFER_MAX_LEN, "%d\n", blink);
}

static ssize_t led_blink_solid_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct gpio_led_data *led_dat =
	    container_of(led_cdev, struct gpio_led_data, cdev);
	long blink;
	int err;

	mutex_lock(&leds_mutex);
	err = kstrtol(buf, 10, &blink);
	if (!err) {
		led_dat->blinking = blink;
		update_leds_qmi_data(led_cdev->name, -1, -1, -1, blink, -1, -1);
		qmi_send_led_data();
	}
	mutex_unlock(&leds_mutex);
	return size;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);

static ssize_t led_blink_on_solid_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int blink_on = 0;

	get_leds_qmi_data(led_cdev->name, NULL, NULL, NULL, NULL, &blink_on,
			  NULL);

	return snprintf(buf, LED_SHOW_BUFFER_MAX_LEN, "%d\n", blink_on);
}

static ssize_t led_blink_on_solid_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	long blink_on;
	int err;

	mutex_lock(&leds_mutex);
	err = kstrtol(buf, 10, &blink_on);
	if (!err) {
		update_leds_qmi_data(led_cdev->name, -1, -1, -1, -1, blink_on,
				     -1);
	}
	mutex_unlock(&leds_mutex);
	return size;
}

static DEVICE_ATTR(delay_on, 0644, led_blink_on_solid_show,
		   led_blink_on_solid_store);

static ssize_t led_blink_off_solid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int blink_off = 0;

	get_leds_qmi_data(led_cdev->name, NULL, NULL, NULL, NULL, NULL,
			  &blink_off);

	return snprintf(buf, LED_SHOW_BUFFER_MAX_LEN, "%d\n", blink_off);
}

static ssize_t led_blink_off_solid_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct gpio_led_data *led_dat =
	    container_of(led_cdev, struct gpio_led_data, cdev);
	long blink_off;
	int on_time = 0;
	int off_time = 0;
	int err;

	mutex_lock(&leds_mutex);
	err = kstrtol(buf, 10, &blink_off);
	if (!err) {
		update_leds_qmi_data(led_cdev->name, -1, -1, -1, -1, -1,
				     blink_off);
	}
	get_leds_qmi_data(led_cdev->name, NULL, NULL, NULL, NULL, &on_time,
			  &off_time);
	if (on_time > 0 && off_time > 0) {
		led_dat->blinking = 1;
		update_leds_qmi_data(led_cdev->name, -1, -1, -1, 1, -1, -1);
	} else {
		led_dat->blinking = 0;
		update_leds_qmi_data(led_cdev->name, -1, -1, -1, 0, -1, -1);
	}
	qmi_send_led_data();
	mutex_unlock(&leds_mutex);
	return size;
}

static DEVICE_ATTR(delay_off, 0644, led_blink_off_solid_show,
		   led_blink_off_solid_store);

static int create_gpio_led(const struct gpio_led *template,
			   struct gpio_led_data *led_dat, struct device *parent,
			   gpio_blink_set_t blink_set)
{
	int ret, state;

	led_dat->gpio = -1;

	/* skip leds that aren't available */
	if (!gpio_is_valid(template->gpio)) {
		dev_info(parent, "Skipping unavailable LED gpio %d (%s)\n",
			 template->gpio, template->name);
		return 0;
	}

	ret = devm_gpio_request(parent, template->gpio, template->name);
	if (ret < 0)
		return ret;

	led_dat->cdev.name = template->name;
	led_dat->cdev.default_trigger = template->default_trigger;
	led_dat->gpio = template->gpio;
	led_dat->can_sleep = gpio_cansleep(template->gpio);
	led_dat->active_low = template->active_low;
	led_dat->blinking = 0;
	update_leds_qmi_data(led_dat->cdev.name, led_dat->gpio,
			     led_dat->active_low, -1, -1, -1, -1);
	/*
	if (blink_set) {
	      led_dat->platform_gpio_blink_set = blink_set;
	      led_dat->cdev.blink_set = gpio_blink_set;
	}
	*/
	led_dat->cdev.brightness_set = gpio_led_set;
	if (template->default_state == LEDS_GPIO_DEFSTATE_KEEP)
		state =
		    (!!gpio_get_value_cansleep(led_dat->gpio)) ^ led_dat->
		    active_low;
	else
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
	led_dat->cdev.brightness = state ? LED_FULL : LED_OFF;
	if (!template->retain_state_suspended)
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

	ret = gpio_direction_output(led_dat->gpio, led_dat->active_low ^ state);
	if (ret < 0)
		return ret;

	/* INIT_WORK(&led_dat->work, gpio_led_work); */

	ret = led_classdev_register(parent, &led_dat->cdev);
	if (ret < 0)
		return ret;

	if (strcmp(led_dat->cdev.name, "red")
	    || strcmp(led_dat->cdev.name, "green")
	    || strcmp(led_dat->cdev.name, "blue")) {
		ret = device_create_file(led_dat->cdev.dev, &dev_attr_blink);
		if (ret) {
			pr_err("leds %s: create dev_attr_blink failed\n",
			       led_dat->cdev.name);
			return ret;
		}
		ret = device_create_file(led_dat->cdev.dev, &dev_attr_delay_on);
		if (ret) {
			pr_err("leds %s: create dev_attr_blink_on failed\n",
			       led_dat->cdev.name);
			return ret;
		}
		ret =
		    device_create_file(led_dat->cdev.dev, &dev_attr_delay_off);
		if (ret) {
			pr_err("leds %s: create dev_attr_blink_off failed\n",
			       led_dat->cdev.name);
			return ret;
		}
	}

	return 0;
}

static void delete_gpio_led(struct gpio_led_data *led)
{
	if (!gpio_is_valid(led->gpio))
		return;
	led_classdev_unregister(&led->cdev);
	cancel_work_sync(&led->work);
}

struct gpio_leds_priv {
	int num_leds;
	struct gpio_led_data leds[];
};

static inline int sizeof_gpio_leds_priv(int num_leds)
{
	return sizeof(struct gpio_leds_priv) +
	    (sizeof(struct gpio_led_data) * num_leds);
}

/* Code to create from OpenFirmware platform devices */
#ifdef CONFIG_OF_GPIO
static struct gpio_leds_priv *gpio_leds_create_of(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct gpio_leds_priv *priv;
	int count, ret;

	/* count LEDs in this device, so we know how much to allocate */
	count = of_get_child_count(np);
	if (!count)
		return ERR_PTR(-ENODEV);

	for_each_child_of_node(np, child)
	    if (of_get_gpio(child, 0) == -EPROBE_DEFER)
		return ERR_PTR(-EPROBE_DEFER);

	priv = devm_kzalloc(&pdev->dev, sizeof_gpio_leds_priv(count),
			    GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct gpio_led led = { };
		enum of_gpio_flags flags;
		const char *state;

		led.gpio = of_get_gpio_flags(child, 0, &flags);
		led.active_low = flags & OF_GPIO_ACTIVE_LOW;
		led.name =
		    of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
		    of_get_property(child, "linux,default-trigger", NULL);
		state = of_get_property(child, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "keep"))
				led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
			else if (!strcmp(state, "on"))
				led.default_state = LEDS_GPIO_DEFSTATE_ON;
			else
				led.default_state = LEDS_GPIO_DEFSTATE_OFF;
		}

		led.retain_state_suspended =
		    (unsigned)of_property_read_bool(child,
						    "retain-state-suspended");

		ret = create_gpio_led(&led, &priv->leds[priv->num_leds++],
				      &pdev->dev, NULL);
		if (ret < 0) {
			of_node_put(child);
			devm_kfree(&pdev->dev, priv);
			goto err;
		}
	}

	return priv;

err:
	for (count = priv->num_leds - 2; count >= 0; count--)
		delete_gpio_led(&priv->leds[count]);
	return ERR_PTR(-ENODEV);
}

static const struct of_device_id of_gpio_leds_match[] = {
	{.compatible = "gpio-modem-leds",},
	{},
};
#else /* CONFIG_OF_GPIO */
static struct gpio_leds_priv *gpio_leds_create_of(struct platform_device *pdev)
{
	return ERR_PTR(-ENODEV);
}
#endif /* CONFIG_OF_GPIO */

static int gpio_led_probe(struct platform_device *pdev)
{
	struct gpio_led_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_leds_priv *priv;
	struct pinctrl *pinctrl;
	int i, ret = 0;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			 "pins are not configured from the driver\n");

	memset(&zte_gpio_leds_qmi_data, 0,
	       sizeof(struct zte_leds_qmi_data_type));
	zte_gpio_leds_qmi_data.red.gpio = 0xFF;
	zte_gpio_leds_qmi_data.green.gpio = 0xFF;
	zte_gpio_leds_qmi_data.blue.gpio = 0xFF;
	update_leds_qmi_data("red", -1, 0, 0, 0, 500, 1500);
	update_leds_qmi_data("green", -1, 0, 0, 0, 500, 1500);
	update_leds_qmi_data("blue", -1, 0, 0, 0, 500, 1500);

	mutex_init(&leds_mutex);

	if (pdata && pdata->num_leds) {
		priv = devm_kzalloc(&pdev->dev,
				    sizeof_gpio_leds_priv(pdata->num_leds),
				    GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->num_leds = pdata->num_leds;
		for (i = 0; i < priv->num_leds; i++) {
			ret = create_gpio_led(&pdata->leds[i],
					      &priv->leds[i],
					      &pdev->dev,
					      pdata->gpio_blink_set);
			if (ret < 0) {
				/* On failure: unwind the led creations */
				for (i = i - 1; i >= 0; i--)
					delete_gpio_led(&priv->leds[i]);
				devm_kfree(&pdev->dev, priv);
				return ret;
			}
		}
	} else {
		priv = gpio_leds_create_of(pdev);
		if (IS_ERR(priv))
			return PTR_ERR(priv);
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int gpio_led_remove(struct platform_device *pdev)
{
	struct gpio_leds_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num_leds; i++)
		delete_gpio_led(&priv->leds[i]);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver gpio_led_driver = {
	.probe = gpio_led_probe,
	.remove = gpio_led_remove,
	.driver = {
		   .name = "gpio-modem-leds",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(of_gpio_leds_match),
		   },
};

module_platform_driver(gpio_led_driver);
