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

#ifndef __FP_VENDOR_H__
#define __FP_VENDOR_H__
#include <linux/types.h>

#define MAX_TA_NAME_LEN 32
#define DEFAULT_FP_TA_NAME "fp_server"

void set_fp_ta_name(uint8_t *ta_name_from_vendor, uint8_t fp_name_len);
void get_fp_ta_name(char *ta_name_to_user);
/*
 * return 1 if fp driver can call spi in ree, else return 0
*/
int get_fp_spi_enable(void);

extern int fp_spi_enable;

#endif  /*__FP_VENDOR_H__*/
