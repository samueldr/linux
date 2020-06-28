/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "fp_vendor.h"
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <imsg_log.h>

int fp_spi_enable = 1;

static uint8_t __fp_ta_name[MAX_TA_NAME_LEN];

void set_fp_ta_name(uint8_t *ta_name_from_vendor, uint8_t fp_name_len)
{
	memset(__fp_ta_name, 0, sizeof(__fp_ta_name));

	strncpy(__fp_ta_name, DEFAULT_FP_TA_NAME, strlen(DEFAULT_FP_TA_NAME));/*set FP_TA default name*/

	if (ta_name_from_vendor == NULL) {
		IMSG_ERROR("ta_name_from_vendor is NULL");
		return;
	}

	if ((fp_name_len > MAX_TA_NAME_LEN) || (fp_name_len == 0)) {
		IMSG_ERROR("ta_name_from_vendor length is invalid");
		return;
	}

	if (strcmp("fp_server", ta_name_from_vendor) == 0) {
		IMSG_ERROR("maybe you can use other fp_ta_name instead of default name");
		return;
	}

	strncpy(__fp_ta_name, ta_name_from_vendor, fp_name_len);
}

void get_fp_ta_name(char *ta_name_to_user)
{
	memcpy(ta_name_to_user, __fp_ta_name, strlen(__fp_ta_name));
	IMSG_INFO("fp_ta_name = %s", ta_name_to_user);
}

int get_fp_spi_enable(void)
{
	if (fp_spi_enable == 0)
		IMSG_ERROR("ERROR fp_spi_enable==0");

	IMSG_INFO("fp_spi_enable==%d", fp_spi_enable);

	return fp_spi_enable;
}
