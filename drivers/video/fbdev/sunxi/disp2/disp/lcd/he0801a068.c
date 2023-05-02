/* drivers/video/sunxi/disp2/disp/lcd/he0801a068.c
 *
 * Copyright (c) 2017 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * he0801a-068 panel driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "he0801a068.h"

static void lcd_power_on(u32 sel);
static void lcd_power_off(u32 sel);
static void lcd_bl_open(u32 sel);
static void lcd_bl_close(u32 sel);

static void lcd_panel_init(u32 sel);
static void lcd_panel_exit(u32 sel);

#define panel_reset(sel, val) sunxi_lcd_gpio_set_value(sel, 0, val)

static void lcd_cfg_panel_info(struct panel_extend_para *info)
{
	u32 i = 0, j = 0;
	u32 items;
	u8 lcd_gamma_tbl[][2] = {
		{0, 0},
		{15, 15},
		{30, 30},
		{45, 45},
		{60, 60},
		{75, 75},
		{90, 90},
		{105, 105},
		{120, 120},
		{135, 135},
		{150, 150},
		{165, 165},
		{180, 180},
		{195, 195},
		{210, 210},
		{225, 225},
		{240, 240},
		{255, 255},
	};

	u32 lcd_cmap_tbl[2][3][4] = {
	{
		{LCD_CMAP_G0, LCD_CMAP_B1, LCD_CMAP_G2, LCD_CMAP_B3},
		{LCD_CMAP_B0, LCD_CMAP_R1, LCD_CMAP_B2, LCD_CMAP_R3},
		{LCD_CMAP_R0, LCD_CMAP_G1, LCD_CMAP_R2, LCD_CMAP_G3},
		},
		{
		{LCD_CMAP_B3, LCD_CMAP_G2, LCD_CMAP_B1, LCD_CMAP_G0},
		{LCD_CMAP_R3, LCD_CMAP_B2, LCD_CMAP_R1, LCD_CMAP_B0},
		{LCD_CMAP_G3, LCD_CMAP_R2, LCD_CMAP_G1, LCD_CMAP_R0},
		},
	};

	items = sizeof(lcd_gamma_tbl) / 2;
	for (i = 0; i < items - 1; i++) {
		u32 num = lcd_gamma_tbl[i+1][0] - lcd_gamma_tbl[i][0];

		for (j = 0; j < num; j++) {
			u32 value = 0;

			value = lcd_gamma_tbl[i][1] +
				((lcd_gamma_tbl[i+1][1] - lcd_gamma_tbl[i][1])
				* j) / num;
			info->lcd_gamma_tbl[lcd_gamma_tbl[i][0] + j] =
							(value<<16)
							+ (value<<8) + value;
		}
	}
	info->lcd_gamma_tbl[255] = (lcd_gamma_tbl[items-1][1]<<16) +
					(lcd_gamma_tbl[items-1][1]<<8)
					+ lcd_gamma_tbl[items-1][1];

	memcpy(info->lcd_cmap_tbl, lcd_cmap_tbl, sizeof(lcd_cmap_tbl));

}

static s32 lcd_open_flow(u32 sel)
{
	LCD_OPEN_FUNC(sel, lcd_power_on, 10);
	LCD_OPEN_FUNC(sel, lcd_panel_init, 10);
	LCD_OPEN_FUNC(sel, sunxi_lcd_tcon_enable, 50);
	LCD_OPEN_FUNC(sel, lcd_bl_open, 0);

	return 0;
}

static s32 lcd_close_flow(u32 sel)
{
	LCD_CLOSE_FUNC(sel, lcd_bl_close, 0);
	LCD_CLOSE_FUNC(sel, sunxi_lcd_tcon_disable, 0);
	LCD_CLOSE_FUNC(sel, lcd_panel_exit, 200);
	LCD_CLOSE_FUNC(sel, lcd_power_off, 500);

	return 0;
}

static void lcd_power_on(u32 sel)
{
	sunxi_lcd_power_enable(sel, 0);
	sunxi_lcd_delay_ms(10);
	sunxi_lcd_power_enable(sel, 1);
	sunxi_lcd_delay_ms(10);
	sunxi_lcd_pin_cfg(sel, 1);
	sunxi_lcd_delay_ms(50);
	panel_reset(sel, 1);
	sunxi_lcd_delay_ms(100);
	panel_reset(sel, 0);
	sunxi_lcd_delay_ms(100);
	panel_reset(sel, 1);
	sunxi_lcd_delay_ms(100);

}

static void lcd_power_off(u32 sel)
{
	sunxi_lcd_pin_cfg(sel, 0);
	sunxi_lcd_delay_ms(20);
	panel_reset(sel, 0);
	sunxi_lcd_delay_ms(5);
	sunxi_lcd_power_disable(sel, 1);
	sunxi_lcd_delay_ms(5);
	sunxi_lcd_power_disable(sel, 0);
}

static void lcd_bl_open(u32 sel)
{
	sunxi_lcd_pwm_enable(sel);
	sunxi_lcd_backlight_enable(sel);
}

static void lcd_bl_close(u32 sel)
{
	sunxi_lcd_backlight_disable(sel);
	sunxi_lcd_pwm_disable(sel);
}
#if 1
#define REGFLAG_DELAY 0XFE
#define REGFLAG_END_OF_TABLE 0xFD // END OF REGISTERS MARKER
struct LCM_setting_table {
    u8 cmd;
    u32 count;
    u8 para_list[64];
};
static struct LCM_setting_table LCM_he0801a068_setting[] = {
#if 0 /* 1200x1920*/
{REGFLAG_DELAY,REGFLAG_DELAY,{100}},
{0x8F,1,{0xA5}},
{REGFLAG_DELAY,REGFLAG_DELAY,{1}},
{0x01,1,{0x00}},
{REGFLAG_DELAY,REGFLAG_DELAY,{20}},
{0x8F,1,{0xA5}},
{REGFLAG_DELAY,REGFLAG_DELAY,{1}},
{0x83,1,{0x00}},
{0x84,1,{0x00}},
{0x8C,1,{0x80}},
{0xCD,1,{0x6C}},
{0xC0,1,{0x8B}},
{0xC8,1,{0xF0}},
{0x8B,1,{0x10}},
{0xA9,1,{0x20}},
{0x97,1,{0x00}},
{0x83,1,{0xAA}},
{0x84,1,{0x11}},
{0xA9,1,{0x4B}},
{0x85,1,{0x04}},
{0x86,1,{0x08}},
{0x98,1,{0xC1}},
{0x83,1,{0xBB}},
{0x84,1,{0x22}},
{0x94,1,{0xBA}},
{0x90,1,{0x40}},
{0xA1,1,{0xFF}},
{0xA2,1,{0xFE}},
{0xA3,1,{0xFA}},
{0xA4,1,{0xF7}},
{0xA5,1,{0xF3}},
{0xA6,1,{0xF1}},
{0xA7,1,{0xED}},
{0xA8,1,{0xEB}},
{0xA9,1,{0xE9}},
{0xAA,1,{0xE6}},
{0xAF,1,{0x00}},
{0xB0,1,{0x35}},
{0xB1,1,{0x89}},
{0xB2,1,{0x99}},
{0xB3,1,{0x99}},
{0xB4,1,{0x0D}},
{0xB5,1,{0x1A}},
{0xB6,1,{0x16}},
{0x9A,1,{0x10}},
{0x9B,1,{0x00}},
{0x96,1,{0xE6}},
{0x99,1,{0x06}},
{0x11,1,{0x00}},
{0x8F,1,{0x00}},
{0x11,1,{0x00}},
{REGFLAG_DELAY,REGFLAG_DELAY,{50}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,REGFLAG_DELAY,{120}},	
{REGFLAG_END_OF_TABLE,REGFLAG_END_OF_TABLE,{}}
#endif/* 800x1280*/
{0xEE,    1,     {0x50}},			
{0xEA,    2,     {0x85,0x55}},			
{0x30,    1,     {0x00}},			
{0x31,    1,     {0x00}},			
{0x90,    2,     {0x50,0x15}},			
{0x24,    1,     {0x20}},			
{0x99,    1,     {0x00}},			
{0x79,    1,     {0x00}},			
{0x95,    1,     {0x74}},			
{0x7A,    1,     {0x20}},			
{0x97,    1,     {0x08}},			
{0x7D,    1,     {0x08}},			
{0x56,    1,     {0x83}},			
{0xEE,    1,     {0x60}},			
{0x30,    1,     {0x01}},			
{0x27,    1,     {0x22}},			
{0x3A,    1,     {0xA4}},			
{0x3B,    1,     {0x00}},			
{0x3C,    1,     {0x1A}},			
{0x3D,    1,     {0x11}},			
{0x3E,    1,     {0x93}},			
{0x42,    1,     {0x64}},			
{0x43,    1,     {0x64}},			
{0x44,    1,     {0x0B}},			
{0x46,    1,     {0x4E}},			
{0x8B,    1,     {0x90}},			
{0x8D,    1,     {0x45}},			
{0x91,    1,     {0x11}},			
{0x92,    1,     {0x11}},			
{0x93,    1,     {0x9F}},			
{0x9A,    1,     {0x00}},			
{0x9C,    1,     {0x80}},			
{0x47,    5,     {0x0F,0x24,0x2C,0x39,0x36}},			
{0x5A,    5,     {0x0F,0x24,0x2C,0x39,0x36}},			
{0x4C,    5,     {0x4A,0x40,0x51,0x31,0x2F}},			
{0x5F,    5,     {0x4A,0x40,0x51,0x31,0x2F}},			
{0x51,    5,     {0x2D,0x10,0x25,0x1F,0x30}},			
{0x64,    5,     {0x2D,0x10,0x25,0x1F,0x30}},			
{0x56,    4,     {0x37,0x46,0x5B,0x7F}},			
{0x69,    4,     {0x37,0x46,0x5B,0x7F}},			
{0xEE,    1,     {0x70}},			
{0x00,    4,     {0x03,0x07,0x00,0x01}},			
{0x04,    4,     {0x08,0x0C,0x55,0x01}},			
{0x0C,    2,     {0x05,0x05}},			
{0x10,    5,     {0x05,0x08,0x00,0x01,0x05}},			
{0x15,    5,     {0x00,0x15,0x0D,0x08,0x00}},			
{0x29,    2,     {0x05,0x05}},			
{0x60,    5,     {0x3C,0x3C,0x07,0x05,0x17}},			
{0x65,    5,     {0x15,0x13,0x11,0x01,0x03}},			
{0x6A,    5,     {0x3C,0x3C,0x3C,0x3C,0x3C}},			
{0x6F,    5,     {0x3C,0x3C,0x3C,0x3C,0x3C}},			
{0x74,    2,     {0x3C,0x3C}},			
{0x80,    5,     {0x3C,0x3C,0x06,0x04,0x16}},			
{0x85,    5,     {0x14,0x12,0x10,0x00,0x02}},			
{0x8A,    5,     {0x3C,0x3C,0x3C,0x3C,0x3C}},			
{0x8F,    5,     {0x3C,0x3C,0x3C,0x3C,0x3C}},			
{0x94,    2,     {0x3C,0x3C}},			
{0xEA,    2,     {0x00,0x00}},			
{0xEE,    1,     {0x00}},			
{0x11,    0,     {}},
{REGFLAG_DELAY, REGFLAG_DELAY, {50}},
{0x29,    0,     {}},
{REGFLAG_DELAY, REGFLAG_DELAY, {120}},
{REGFLAG_END_OF_TABLE, REGFLAG_END_OF_TABLE, {}}
};
#endif 
static void lcd_panel_init(u32 sel)
{
#if 1
	int i=0;
	printk("he0801a068:lcd panel init\n");
	//sunxi_lcd_dsi_clk_enable(sel);
	for(i=0;;i++)
	{
		if(LCM_he0801a068_setting[i].count == REGFLAG_END_OF_TABLE)
			break;
		else if (LCM_he0801a068_setting[i].count == REGFLAG_DELAY)
			sunxi_lcd_delay_ms(LCM_he0801a068_setting[i].para_list[0]);
		else
			sunxi_lcd_dsi_dcs_write(sel,LCM_he0801a068_setting[i].cmd,LCM_he0801a068_setting[i].para_list,LCM_he0801a068_setting[i].count);
	}
	sunxi_lcd_dsi_clk_enable(sel);
#else 
	//sunxi_lcd_dsi_clk_enable(sel);
	printf("uboot78->>>he0801a068:lcd panel init\n");
	sunxi_lcd_delay_ms(100);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x8F, 0xA5);
	sunxi_lcd_delay_ms(10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x01, 0x00);
	sunxi_lcd_delay_ms(20);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x8F, 0xA5);
	sunxi_lcd_delay_ms(10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0x00);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x00);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x8C, 0x8E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCD, 0x6C);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC0, 0x8B);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC8, 0xF0);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x8B, 0x10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA9, 0x20);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x97, 0x00);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFA, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFD, 0x13);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0xAA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x11);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA9, 0x4B);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0xBB);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x22);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x94, 0xB9);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA1, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA2, 0xFA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA3, 0xF3);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA4, 0xED);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA5, 0xE7);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA6, 0xE2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA7, 0xDC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA8, 0xD7);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xA9, 0xD1);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xAA, 0xCC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xB4, 0x1C);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xB5, 0x38);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xB6, 0x30);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x96, 0x00);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x90, 0x40);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0xAA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x11);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC0, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC1, 0x09);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC2, 0x16);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC3, 0x24);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC4, 0x32);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC5, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC6, 0x48);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC7, 0x51);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC8, 0x59);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC9, 0xD1);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCA, 0xDA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCB, 0xF7);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCC, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCD, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD0, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD1, 0x1E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD2, 0x2F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD3, 0x52);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD4, 0x58);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD5, 0xB0);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD6, 0xB6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD7, 0xBC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD8, 0xCF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD9, 0xD8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDA, 0xE2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDB, 0xEC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDC, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE0, 0x15);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE1, 0x17);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE2, 0x20);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE3, 0x30);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE4, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE5, 0x4A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE6, 0x54);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE7, 0x5D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE8, 0x65);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE9, 0xDD);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEA, 0xE6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEB, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEC, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xED, 0x10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF0, 0x14);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF1, 0x2E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF2, 0x3F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF3, 0x62);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF4, 0x6A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF5, 0xC2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF6, 0xCA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF7, 0xD2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF8, 0xDB);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF9, 0xE4);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFA, 0xEE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFB, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFC, 0xFE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0xBB);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x22);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC0, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC1, 0x09);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC2, 0x16);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC3, 0x24);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC4, 0x32);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC5, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC6, 0x48);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC7, 0x51);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC8, 0x59);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC9, 0xD1);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCA, 0xDA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCB, 0xF7);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCC, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCD, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD0, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD1, 0x1E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD2, 0x2F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD3, 0x52);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD4, 0x58);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD5, 0xB0);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD6, 0xB6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD7, 0xBC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD8, 0xCF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD9, 0xD8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDA, 0xE2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDB, 0xEC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDC, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE0, 0x15);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE1, 0x17);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE2, 0x20);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE3, 0x30);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE4, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE5, 0x4A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE6, 0x54);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE7, 0x5D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE8, 0x65);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE9, 0xDD);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEA, 0xE6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEB, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEC, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xED, 0x10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF0, 0x14);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF2, 0x3F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF1, 0x2E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF3, 0x62);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF4, 0x6A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF5, 0xC2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF6, 0xCA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF7, 0xD2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF8, 0xDB);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF9, 0xE4);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFA, 0xEE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFB, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFC, 0xFE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x83, 0xCC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x84, 0x33);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC0, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC1, 0x09);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC2, 0x16);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC3, 0x24);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC4, 0x32);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC5, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC6, 0x48);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC7, 0x51);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC8, 0x59);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xC9, 0xD1);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCA, 0xDA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCB, 0xF7);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCC, 0x07);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCD, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xCF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD0, 0x0A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD1, 0x1E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD3, 0x52);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD2, 0x2F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD4, 0x58);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD5, 0xB0);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD6, 0xB6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD7, 0xBC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD8, 0xCF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xD9, 0xD8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDA, 0xE2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDB, 0xEC);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDC, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xDF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE0, 0x15);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE1, 0x17);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE2, 0x20);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE3, 0x30);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE4, 0x3E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE5, 0x4A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE6, 0x54);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE7, 0x5D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE8, 0x65);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xE9, 0xDD);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEA, 0xE6);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEB, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEC, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xED, 0x10);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEE, 0x0D);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xEF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF0, 0x14);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF1, 0x2E);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF2, 0x3F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF3, 0x62);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF4, 0x6A);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF6, 0xCA);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF5, 0xC2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF7, 0xD2);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF8, 0xDB);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xF9, 0xE4);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFA, 0xEE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFB, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFC, 0xFE);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFD, 0xFF);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFE, 0xF8);
	sunxi_lcd_dsi_dcs_write_1para(sel,0xFF, 0x0F);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x11, 0x00);
	sunxi_lcd_dsi_dcs_write_1para(sel,0x8F, 0x00);
	sunxi_lcd_dsi_clk_enable(sel);
#endif
}

static void lcd_panel_exit(u32 sel)
{
	sunxi_lcd_dsi_dcs_write_0para(sel, 0x10);
	sunxi_lcd_delay_ms(80);
	sunxi_lcd_dsi_dcs_write_0para(sel, 0x28);
	sunxi_lcd_delay_ms(50);
}

/*sel: 0:lcd0; 1:lcd1*/
static s32 lcd_user_defined_func(u32 sel, u32 para1, u32 para2, u32 para3)
{
	return 0;
}

struct __lcd_panel he0801a068_panel = {
	/* panel driver name, must mach the name of
	 * lcd_drv_name in sys_config.fex
	 */
	.name = "he0801a068",
	.func = {
		.cfg_panel_info = lcd_cfg_panel_info,
			.cfg_open_flow = lcd_open_flow,
			.cfg_close_flow = lcd_close_flow,
			.lcd_user_defined_func = lcd_user_defined_func,
	},
};
