/*
 * @file   silead_fp_mtk.c
 * @brief  Contains silead_fp device implements for Mediatek platform.
 *
 *
 * Copyright 2016-2017 Slead Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/5/2    0.1.0      Init version
 * Bill Yu    2018/5/20   0.1.1      Default wait 3ms after reset
 * Bill Yu    2018/6/5    0.1.2      Support chip enter power down
 * Bill Yu    2018/6/27   0.1.3      Expand pwdn I/F
 *
 */

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include "silead_fp.h"

int silfp_parse_dts(struct silfp_data *fp_dev)
{
	int ret = 0;
	int status = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	/*---avdd port---*/
	fp_dev->avdd_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "avdd-gpios", 0);
	LOG_MSG_DEBUG(I_LOG, "[%s] enter, fp_dev->avdd_port : %d\n", __func__, fp_dev->avdd_port);

	if (gpio_is_valid(fp_dev->avdd_port)) {
		ret = gpio_request(fp_dev->avdd_port, "SILFP_AVDD_PIN");
		if (ret < 0) {
			LOG_MSG_DEBUG(E_LOG, "[%s] Failed to request avdd_port = %d, ret = %d",
					  __func__, (s32)fp_dev->avdd_port, ret);
			status = -ENODEV;
			goto err_avdd;
		} else {
			LOG_MSG_DEBUG(I_LOG, "[%s] Success to request avdd_port!\n", __func__);
			gpio_direction_output(fp_dev->avdd_port, 1);
		}
	}

	/*---int port---*/
	fp_dev->int_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "irq-gpios", 0);
	LOG_MSG_DEBUG(I_LOG, "[%s] enter, fp_dev->int_port : %d\n", __func__, fp_dev->int_port);

	if (gpio_is_valid(fp_dev->int_port)) {
		ret = gpio_request(fp_dev->int_port, "SILFP_INT_IRQ");
		if (ret < 0) {
			LOG_MSG_DEBUG(E_LOG, "[%s] Failed to request int_port =%d, ret = %d",
					 __func__, (s32)fp_dev->int_port, ret);
			status = -ENODEV;
			goto err_irq_port;
		} else {
			LOG_MSG_DEBUG(I_LOG, "[%s] Success to request int_port!\n", __func__);
			gpio_direction_input(fp_dev->int_port);
			fp_dev->irq = gpio_to_irq(fp_dev->int_port);
			fp_dev->irq_is_disable = 0;

			ret  = request_irq(fp_dev->irq,
				silfp_irq_handler,
				IRQ_TYPE_EDGE_RISING,/*IRQ_TYPE_LEVEL_HIGH,irq_table[ts->int_trigger_type]*/
				"silfp",
				fp_dev);
			if (ret < 0) {
				LOG_MSG_DEBUG(E_LOG, "[%s] Failed to request_irq (%d), ret=%d",
				__func__, fp_dev->irq, ret);
				status = -ENODEV;
				goto err_irq_request;
			} else {
				LOG_MSG_DEBUG(I_LOG, "[%s] Enable_irq_wake.\n", __func__);
				enable_irq_wake(fp_dev->irq);
				silfp_irq_disable(fp_dev);
			}
		}
	}

	/*---rst port---*/
	fp_dev->rst_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "rst-gpios", 0);
	LOG_MSG_DEBUG(I_LOG, "[%s] enter, fp_dev->rst_port : %d\n", __func__, fp_dev->rst_port);

	if (gpio_is_valid(fp_dev->rst_port)) {
		ret = gpio_request(fp_dev->rst_port, "SILFP_RST_PIN");
		if (ret < 0) {
			LOG_MSG_DEBUG(E_LOG, "[%s] Failed to request rst_port = %d, ret = %d",
				__func__, (s32)fp_dev->rst_port, ret);
			status = -ENODEV;
			goto err_rst_port;
		} else {
			LOG_MSG_DEBUG(I_LOG, "[%s] Success to request rst_port!\n", __func__);
			gpio_direction_output(fp_dev->rst_port, 1);
		}
	}
	LOG_MSG_DEBUG(D_LOG, "[%s] Done.\n", __func__);

	return status;

err_rst_port:
	fp_dev->rst_port = 0;

err_irq_request:
	gpio_free(fp_dev->int_port);

err_irq_port:
	fp_dev->int_port = 0;

err_avdd:
	fp_dev->avdd_port = 0;
	return status;
}

void silfp_cleanup(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter.\n", __func__);

	if (gpio_is_valid(fp_dev->avdd_port)) {
		gpio_set_value(fp_dev->avdd_port, 0);
		gpio_free(fp_dev->avdd_port);
		LOG_MSG_DEBUG(I_LOG, "silead set avdd_port low and remove avdd_port success\n");
		fp_dev->avdd_port = 0;
	}
	if (gpio_is_valid(fp_dev->int_port)) {
		gpio_free(fp_dev->int_port);
		LOG_MSG_DEBUG(I_LOG, "silead remove int_port success\n");
		fp_dev->int_port = 0;
	}
	if (gpio_is_valid(fp_dev->rst_port)) {
		gpio_set_value(fp_dev->rst_port, 0);
		gpio_free(fp_dev->rst_port);
		LOG_MSG_DEBUG(I_LOG, "silead set rst_port low and remove rst_port success\n");
		fp_dev->rst_port = 0;
	}
}

void silfp_hw_poweron(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter.\n", __func__);

	if (gpio_is_valid(fp_dev->avdd_port)) {
		gpio_set_value(fp_dev->avdd_port, 1);
		msleep(20);
		LOG_MSG_DEBUG(I_LOG, "----silead power on ok ----\n");
	}
	fp_dev->power_is_off = 0;
}

void silfp_hw_poweroff(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter.\n", __func__);

	if (gpio_is_valid(fp_dev->avdd_port)) {
		gpio_set_value(fp_dev->avdd_port, 0);
		LOG_MSG_DEBUG(I_LOG, "----silead power off ok ----\n");
	}
	fp_dev->power_is_off = 1;
}

void silfp_hw_reset(struct silfp_data *fp_dev, u8 delay)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter.\n", __func__);

	if (gpio_is_valid(fp_dev->rst_port)) {
		gpio_set_value(fp_dev->rst_port, 0);
		msleep(20);
		gpio_set_value(fp_dev->rst_port, 1);
		msleep(delay);
		LOG_MSG_DEBUG(I_LOG, "----silead hw reset ok----\n");
	}
}

void silfp_set_spi(struct silfp_data *fp_dev, bool enable)
{

	LOG_MSG_DEBUG(D_LOG, "%s: AP no needed!\n", __func__);

}

int silfp_resource_init(struct silfp_data *fp_dev, struct fp_dev_init_t *dev_info)
{
	int ret = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	if (atomic_read(&fp_dev->init)) {
		atomic_inc(&fp_dev->init);
		LOG_MSG_DEBUG(I_LOG, "[%s] dev already init(%d).\n", __func__, atomic_read(&fp_dev->init));
		return ret;
	}

	ret = silfp_parse_dts(fp_dev);
	/* If here not poweron, the chip_id will not corret*/
	silfp_hw_poweron(fp_dev);

	if (!ret) {
		if (silfp_input_init(fp_dev)) {
			goto err_input;
		}
		atomic_set(&fp_dev->init, 1);
	}

	dev_info->reserve = PKG_SIZE;
	dev_info->reserve <<= 12;

	LOG_MSG_DEBUG(I_LOG, "[%s] done.\n", __func__);
	return ret;

err_input:
	if (fp_dev->rst_port > 0) {
		gpio_free(fp_dev->rst_port);
	}
	return ret;
}

/* End of file spilead_fp_platform.c */
