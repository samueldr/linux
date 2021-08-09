/*
 * Driver for the Solomon SSD1307 OLED controller
 *
 * Copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/socinfo.h>

#define SECOND_PANEL_WIDTH		128
#define SECOND_PANEL_HEIGHT		160

#define SECOND_PANEL_DC_HIGH "second_panel_dc_high"
#define SECOND_PANEL_DC_LOW "second_panel_dc_low"
#define SECOND_PANEL_RST_HIGH "second_panel_rst_high"
#define SECOND_PANEL_RST_LOW "second_panel_rst_low"
#define SECOND_PANEL_VDDIO_HIGH "second_panel_vddio_high"
#define SECOND_PANEL_VDDIO_LOW "second_panel_vddio_low"

struct second_panel_par *g_par;

struct mdss_scond_panel_cmd {
	int size;
	char *payload;
	int wait;
};

struct second_panel_par {
	struct spi_device *client;
	struct fb_info *info;
	struct delayed_work		test_work;
	int  reset;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_dc_high;
	struct pinctrl_state	*pins_dc_low;
	struct pinctrl_state	*pins_rst_high;
	struct pinctrl_state	*pins_rst_low;
	struct pinctrl_state	*pins_vddio_high;
	struct pinctrl_state	*pins_vddio_low;
	struct regulator  *second_panel_vddio;
	struct regulator  *second_panel_vdd;
};

static struct fb_fix_screeninfo second_panel_fix = {
	.id				= "ST7735S",
	.type			= FB_TYPE_PACKED_PIXELS,
	.visual			= FB_VISUAL_MONO10,
	.xpanstep		= 1,
	.ypanstep		= 1,
	.ywrapstep		= 1,
	.line_length		= SECOND_PANEL_WIDTH * 2,
	.accel			= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo second_panel_var = {
	.xres			= SECOND_PANEL_WIDTH,
	.yres			= SECOND_PANEL_HEIGHT,
	.xres_virtual		= SECOND_PANEL_WIDTH,
	.yres_virtual		= SECOND_PANEL_HEIGHT,
	.bits_per_pixel	= 16,
};

unsigned int startup_flag = 0;
unsigned char second_panel_buf[SECOND_PANEL_HEIGHT*SECOND_PANEL_WIDTH * 2] = {0};


extern int set_second_spi_backlight(int enable);





static int st7735s_second_write_array(struct spi_device *client, u8 *cmd, u32 len)
{
	int ret = 0;

	ret = spi_write(client, cmd, len);
	if (ret) {
		pr_err("Couldn't send spi command, ret %d, len %d\n", ret, len);
		goto error;
	}

error:

	return ret;
}

static void second_panel_power_on(struct spi_device *client)
{
	pr_info("spi %s\n", __func__);

}

static void second_panel_power_off(struct spi_device *client)
{

	pr_info("spi %s--do nothing\n", __func__);

}

static int lcd_inited = 0;
static void second_panel_reset(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct second_panel_par *par = info->par;
	int ret;

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_high);
	if (ret)
		pr_err("select SECOND_PANEL_RST_HIGH failed with %d\n", ret);

	msleep(20);

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_low);
	if (ret)
		pr_err("select SECOND_PANEL_RST_LOW failed with %d\n", ret);

	msleep(20);

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_high);
	if (ret)
		pr_err("select SECOND_PANEL_RST_HIGH failed with %d\n", ret);
	msleep(120);
	pr_info("spi %s\n", __func__);
}

void second_panel_sleep(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct second_panel_par *par = info->par;
	char display_off_command[] = {0x28, 0x10};
	int ret;

	if (!lcd_inited) {
		second_panel_power_on(client);
		second_panel_reset(client);
		pr_info("Power on and reset only for the first time\n");
	}

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
	if (ret) {
		pr_err("select pins_dc_low failed with %d\n", ret);
		return;
	}

	st7735s_second_write_array(par->client, display_off_command, sizeof(display_off_command));


	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_low);
	if (ret)
		pr_err("select pins_rst_low failed with %d\n", ret);

	second_panel_power_off(client);

	lcd_inited = 0;

	pr_info("spi %s\n", __func__);

}

/* check lcd id floating 2016.10.08 start*/
extern struct gpio_chip *chip_debug;
extern unsigned gpio_level_show_pm(int *id, struct gpio_chip *chip);
extern unsigned gpio_pull_store_pm(int *id, struct gpio_chip *chip, u32 values);
static int panel_id_is_floating = -1;
uint32_t check_panel_id_is_floating(int id)
{
	uint32_t id_is_floating, id_status;

	/*should not remove.*/
	id_is_floating = 0;
	id_status = 0;

	if (id < 0) {
		/*pr_info("LCD panel id is not configed yet,do not check! return!\n");*/
		return id_is_floating;
	}
	if (id >= 0)
		gpio_pull_store_pm(&id, chip_debug, 3);
	mdelay(5);

	if (id >= 0)
		id_status = gpio_level_show_pm(&id, chip_debug);
	mdelay(2);

	if (id >= 0)
		gpio_pull_store_pm(&id, chip_debug, 1);
	mdelay(5);

	if (id >= 0) {
		if (id_status != gpio_level_show_pm(&id, chip_debug)) {
			pr_info("LCD id:%d is floating!\n", id);
			id_is_floating = 1;
		} else {
			id_is_floating = 0;
		}
	}
	pr_info("LCD panel id floating pin number = %d\n", id_is_floating);

	return id_is_floating;

}

/* check lcd id floating 2016.10.08 end*/

unsigned int panel_id;
static void second_panel_getid(void)
{
	int id_num = 130;
	unsigned id_status;

	id_status = gpio_level_show_pm(&id_num, chip_debug);

	pr_info("second panel id_status=%d\n", id_status);

	if (id_status == 0)
		panel_id = 0;
	else
		panel_id = 1;
	if (check_panel_id_is_floating(id_num) == 1)
		panel_id = 2;

}

static void second_panel_init(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct second_panel_par *par = info->par;
	int ret, i;

#define CONFIG_ZTE_LCD_CHARON_SKYWORTH_BOE_PANEL
#ifdef CONFIG_ZTE_LCD_CHARON_SKYWORTH_BOE_PANEL
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd0[] = {
	0xfe,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd1[] = {
	0xfe,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd2[] = {
	0xef,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd3[] = {
	0xb3, 0x03,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd4[] = {
	0xb6, 0x01,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd5[] = {
	0xa3, 0x11,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd6[] = {
	0x21,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd7[] = {
	0x36, 0xd0,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd8[] = {
	0x3a, 0x05,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd9[] = {
	0xb4, 0x21,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd10[] = {
	0xf0, 0x31, 0x4c, 0x24, 0x58, 0xa8, 0x26, 0x28, 0x00, 0x2c, 0x0c, 0x0c, 0x15, 0x15, 0x0f,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd11[] = {
	0xf1, 0x0e, 0x2d, 0x24, 0x3e, 0x99, 0x12, 0x13, 0x00, 0x0a, 0x0d, 0x0d, 0x14, 0x13, 0x0f,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd12[] = {
	0xfe,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd13[] = {
	0xff,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd14[] = {
	0x11,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd15[] = {
	0x29,
};
char lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd16[] = {
	0x2c,
};
struct mdss_scond_panel_cmd helitai_st7735s_hsd_1p8_spi_cmd_on_command[] = {
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd0, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd1, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd2, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd3, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd4, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd5, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd6, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd7, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd8, 0x00},
	{0x02, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd9, 0x00},
	{0x0f, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd10, 0x00},
	{0x0f, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd11, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd12, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd13, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd14, 0x78},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd15, 0x00},
	{0x01, lcetron_gc9104_ctc_1p8_spi_cmd_on_cmd16, 0x00},
};
#else
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd0[] = {
		0x11,
	};
	/*char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd1[] = {
		0xf0, 0x01
	};*/
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd2[] = {
		0xb1, 0x01, 0x2c, 0x2d
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd3[] = {
		0xb2, 0x01, 0x08, 0x05
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd_TE[] = {
		0x35, 0x00,
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd4[] = {
		0xb3, 0x01, 0x08, 0x05, 0x05, 0x08, 0x05
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd5[] = {
		0xb4, 0x03
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd6[] = {
		0xC0, 0x28, 0x08, 0x04
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd7[] = {
		0xC1, 0xc0
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd8[] = {
		0xC2, 0x0d, 0x00
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd9[] = {
		0xC3, 0x8d, 0x2a
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd10[] = {
		0xC4, 0x8d, 0xee
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd11[] = {
		0xC5, 0x2d		/* vcom */
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd11_1[] = {
		0x36, 0xd8		/* 0x36 regitster change to 0Xd8 to resolve red and blue channel inversion */
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd12[] = {
		0Xe0, 0X07, 0X18, 0X0c, 0X15, 0X2e, 0X2a, 0X23, 0X28, 0X28, 0X28, 0X2e, 0X39, 0X00, 0X03, 0x02, 0x10
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd13[] = {
		0Xe1, 0X06, 0X23, 0X0d, 0X17, 0X35, 0X30, 0X2a, 0X2d, 0X2c, 0X29, 0X31, 0X3b, 0X00, 0X02, 0x03, 0x12
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd14[] = {
		0xfc, 0X8c
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd14_1[] = {
		0x3a, 0X05
	};
	/*char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd15[] = {
		0x36, 0Xd8
	};*/
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd16[] = {
		0x29
	};
	char helitai_st7735s_hsd_1p8_spi_cmd_on_cmd17[] = {
		0x2c,
	};

	struct mdss_scond_panel_cmd helitai_st7735s_hsd_1p8_spi_cmd_on_command[] = {
		{0x01, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd0, 0x78},
		/*{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd1, 0x00},*/
		{0x04, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd2, 0x00},
		{0x04, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd3, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd_TE, 0x00},
		{0x07, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd4, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd5, 0x00},
		{0x04, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd6, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd7, 0x00},
		{0x03, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd8, 0x00},
		{0x03, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd9, 0x00},
		{0x03, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd10, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd11, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd11_1, 0x00},
		{0x11, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd12, 0x00},
		{0x11, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd13, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd14, 0x00},
		{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd14_1, 0x00},
		/*{0x02, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd15, 0x00},*/
		{0x01, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd16, 0x00},
		{0x01, helitai_st7735s_hsd_1p8_spi_cmd_on_cmd17, 0x00},
	};
#endif

#ifdef CONFIG_ZTE_LCD_CHARON_COE_IVO_PANEL
	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd0[] = {
	0x11,
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd1[] = {
	0xb1, 0x05, 0x3c, 0x3c
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd2[] = {
	0xb2, 0x05, 0x3c, 0x3c
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd_TE[] = {
	0x35, 0x00,
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd3[] = {
	0xb3, 0x05, 0x3c, 0x3c, 0x05, 0x3c, 0x3c
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd4[] = {
	0xB4, 0x03,	/* 03H-->Dot inversion  ;07H-->Column Inversion */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd5[] = {
	0xC0, 0xA4, 0x04, 0x04
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd6[] = {
	0xC1, 0xc6
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd7[] = {
	0xC2, 0x0d, 0x00
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd8[] = {
	0xC3, 0x8d, 0x2a
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd9[] = {
	0xC4, 0x8d, 0xee
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd10[] = {
	0xC5, 0x10,		/* vcom */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd11[] = {
	0x36, 0xc0		/* MX, MY, RGB mode */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd12[] = {
	0Xe0, 0X08, 0X17, 0X16, 0X1A, 0X38, 0X31, 0X29,
	0X2c, 0X2A, 0X27, 0X2e, 0X39, 0X00, 0X03, 0x00, 0x10
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd13[] = {
	0Xe1, 0X09, 0X17, 0X16, 0X1B, 0X39, 0X32, 0X2a, 0X2c,
	0X2a, 0X27, 0X2f, 0X39, 0X00, 0X03, 0x01, 0x10
};


	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd14[] = {
	0x3a, 0X05
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd15[] = {
	0x29
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd16[] = {
	0x2c,
};


	struct mdss_scond_panel_cmd coe_st7735s_ivo_1p8_spi_cmd_on_command[] = {
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd0, 0x78},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd1, 0x00},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd2, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd_TE, 0x00},
	{0x07, coe_st7735s_ivo_1p8_spi_cmd_on_cmd3, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd4, 0x00},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd5, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd6, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd7, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd8, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd9, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd10, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd11, 0x00},
	{0x11, coe_st7735s_ivo_1p8_spi_cmd_on_cmd12, 0x00},
	{0x11, coe_st7735s_ivo_1p8_spi_cmd_on_cmd13, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd14, 0x00},
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd15, 0x00},
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd16, 0x00},

};

#else
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd0[] = {
		0x11,
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd1[] = {
		0xb1, 0x01, 0x2c, 0x2d
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd2[] = {
		0xb2, 0x01, 0x08, 0x05
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd_TE[] = {
		0x35, 0x00,
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd3[] = {
		0xb3, 0x01, 0x08, 0x05, 0x05, 0x08, 0x05
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd4[] = {
		0xB4, 0x03,	/* 03H-->Dot inversion  ;07H-->Column Inversion */
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd5[] = {
		0xC0, 0x28, 0x08, 0x04
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd6[] = {
		0xC1, 0xc0
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd7[] = {
		0xC2, 0x0d, 0x00
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd8[] = {
		0xC3, 0x8d, 0x2a
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd9[] = {
		0xC4, 0x8d, 0xee
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd10[] = {
		0xC5, 0x2d,		/* vcom */
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd11[] = {
		0x36, 0xc0		/* MX, MY, RGB mode */
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd12[] = {
		0Xe0, 0X07, 0X18, 0X0c, 0X15, 0X2e, 0X2a, 0X23, 0X28, 0X28, 0X28, 0X2e, 0X39, 0X00, 0X03, 0x02, 0x10
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd13[] = {
		0Xe1, 0X06, 0X23, 0X0d, 0X17, 0X35, 0X30, 0X2a, 0X2d, 0X2c, 0X29, 0X31, 0X3b, 0X00, 0X02, 0x03, 0x12
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd14[] = {
		0xfc, 0X8c
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd15[] = {
		0x3a, 0X05
	};
	static char coe_st7735s_ctc_1p8_spi_cmd_on_cmd16[] = {
		0x29
	};
	char coe_st7735s_ctc_1p8_spi_cmd_on_cmd17[] = {
		0x2c,
	};

	struct mdss_scond_panel_cmd coe_st7735s_ctc_1p8_spi_cmd_on_command[] = {
		{0x01, coe_st7735s_ctc_1p8_spi_cmd_on_cmd0, 0x78},
		{0x04, coe_st7735s_ctc_1p8_spi_cmd_on_cmd1, 0x00},
		{0x04, coe_st7735s_ctc_1p8_spi_cmd_on_cmd2, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd_TE, 0x00},
		{0x07, coe_st7735s_ctc_1p8_spi_cmd_on_cmd3, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd4, 0x00},
		{0x04, coe_st7735s_ctc_1p8_spi_cmd_on_cmd5, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd6, 0x00},
		{0x03, coe_st7735s_ctc_1p8_spi_cmd_on_cmd7, 0x00},
		{0x03, coe_st7735s_ctc_1p8_spi_cmd_on_cmd8, 0x00},
		{0x03, coe_st7735s_ctc_1p8_spi_cmd_on_cmd9, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd10, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd11, 0x00},
		{0x11, coe_st7735s_ctc_1p8_spi_cmd_on_cmd12, 0x00},
		{0x11, coe_st7735s_ctc_1p8_spi_cmd_on_cmd13, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd14, 0x00},
		{0x02, coe_st7735s_ctc_1p8_spi_cmd_on_cmd15, 0x00},
		{0x01, coe_st7735s_ctc_1p8_spi_cmd_on_cmd16, 0x00},
		{0x01, coe_st7735s_ctc_1p8_spi_cmd_on_cmd17, 0x00},

	};


	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd0[] = {
	0x11,
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd1[] = {
	0xb1, 0x01, 0x2c, 0x2d
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd2[] = {
	0xb2, 0x01, 0x08, 0x05
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd_TE[] = {
	0x35, 0x00,
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd3[] = {
	0xb3, 0x01, 0x08, 0x05, 0x05, 0x08, 0x05
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd4[] = {
	0xB4, 0x03,	/* 03H-->Dot inversion  ;07H-->Column Inversion */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd5[] = {
	0xC0, 0x28, 0x08, 0x04
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd6[] = {
	0xC1, 0xc0
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd7[] = {
	0xC2, 0x0d, 0x00
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd8[] = {
	0xC3, 0x8d, 0x2a
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd9[] = {
	0xC4, 0x8d, 0xee
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd10[] = {
	0xC5, 0x2d,		/* vcom */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd11[] = {
	0x36, 0xc0		/* MX, MY, RGB mode */
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd12[] = {
	0Xe0, 0X07, 0X18, 0X0c, 0X15, 0X2e, 0X2a, 0X23,
	0X28, 0X28, 0X28, 0X2e, 0X39, 0X00, 0X03, 0x02, 0x10
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd13[] = {
	0Xe1, 0X06, 0X23, 0X0d, 0X17, 0X35, 0X30, 0X2a, 0X2d,
	0X2c, 0X29, 0X31, 0X3b, 0X00, 0X02, 0x03, 0x12
};


	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd14[] = {
	0xfc, 0X8c
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd15[] = {
	0x3a, 0X05
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd16[] = {
	0x29
};

	char coe_st7735s_ivo_1p8_spi_cmd_on_cmd17[] = {
	0x2c,
};


	struct mdss_scond_panel_cmd coe_st7735s_ivo_1p8_spi_cmd_on_command[] = {
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd0, 0x78},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd1, 0x00},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd2, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd_TE, 0x00},
	{0x07, coe_st7735s_ivo_1p8_spi_cmd_on_cmd3, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd4, 0x00},
	{0x04, coe_st7735s_ivo_1p8_spi_cmd_on_cmd5, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd6, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd7, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd8, 0x00},
	{0x03, coe_st7735s_ivo_1p8_spi_cmd_on_cmd9, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd10, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd11, 0x00},
	{0x11, coe_st7735s_ivo_1p8_spi_cmd_on_cmd12, 0x00},
	{0x11, coe_st7735s_ivo_1p8_spi_cmd_on_cmd13, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd14, 0x00},
	{0x02, coe_st7735s_ivo_1p8_spi_cmd_on_cmd15, 0x00},
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd16, 0x00},
	{0x01, coe_st7735s_ivo_1p8_spi_cmd_on_cmd17, 0x00},

};

#endif



	if (lcd_inited) {
		pr_info("Second panel has been initialized already, not need to do again\n");
		return;
	}

	second_panel_power_on(client);

	second_panel_reset(client);

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	if (panel_id == 0) {
#ifdef CONFIG_ZTE_LCD_CHARON_COE_IVO_PANEL
		pr_info("%s: second panel is coe ivo!\n", __func__);
		for (i = 0; i < ARRAY_SIZE(coe_st7735s_ivo_1p8_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			st7735s_second_write_array(client, coe_st7735s_ivo_1p8_spi_cmd_on_command[i].payload, 1);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (coe_st7735s_ivo_1p8_spi_cmd_on_command[i].size > 1) {
				st7735s_second_write_array(client,
						coe_st7735s_ivo_1p8_spi_cmd_on_command[i].payload + 1,
						coe_st7735s_ivo_1p8_spi_cmd_on_command[i].size - 1);
			}

			if (coe_st7735s_ivo_1p8_spi_cmd_on_command[i].wait != 0)
				msleep(coe_st7735s_ivo_1p8_spi_cmd_on_command[i].wait);
			}
#else
		pr_info("%s: second panel is coe ctc!\n", __func__);
		for (i = 0; i < ARRAY_SIZE(coe_st7735s_ctc_1p8_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			st7735s_second_write_array(client, coe_st7735s_ctc_1p8_spi_cmd_on_command[i].payload, 1);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (coe_st7735s_ctc_1p8_spi_cmd_on_command[i].size > 1) {
				st7735s_second_write_array(client,
						coe_st7735s_ctc_1p8_spi_cmd_on_command[i].payload + 1,
						coe_st7735s_ctc_1p8_spi_cmd_on_command[i].size - 1);
			}

			if (coe_st7735s_ctc_1p8_spi_cmd_on_command[i].wait != 0)
				msleep(coe_st7735s_ctc_1p8_spi_cmd_on_command[i].wait);
		}

#endif

	}
	if (panel_id == 2) {
		pr_info("%s: second panel is coe ivo!\n", __func__);
		for (i = 0; i < ARRAY_SIZE(coe_st7735s_ivo_1p8_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			st7735s_second_write_array(client, coe_st7735s_ivo_1p8_spi_cmd_on_command[i].payload, 1);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (coe_st7735s_ivo_1p8_spi_cmd_on_command[i].size > 1) {
				st7735s_second_write_array(client,
						coe_st7735s_ivo_1p8_spi_cmd_on_command[i].payload + 1,
						coe_st7735s_ivo_1p8_spi_cmd_on_command[i].size - 1);
			}

			if (coe_st7735s_ivo_1p8_spi_cmd_on_command[i].wait != 0)
				msleep(coe_st7735s_ivo_1p8_spi_cmd_on_command[i].wait);
		}
	} else if (panel_id == 1) {
		pr_info("%s: second panel is lcetron!\n", __func__);
		for (i = 0; i < ARRAY_SIZE(helitai_st7735s_hsd_1p8_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			st7735s_second_write_array(client, helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].payload, 1);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].size > 1) {
				st7735s_second_write_array(client,
						helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].payload + 1,
						helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].size - 1);
			}

			if (helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].wait != 0)
				msleep(helitai_st7735s_hsd_1p8_spi_cmd_on_command[i].wait);
		}
	}

	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
		if (ret) {
			pr_err("select pins_dc_low failed with %d\n", ret);
			return;
		}
	}

	lcd_inited = 1;

	pr_info("spi %s end\n", __func__);

}


static void buffer_swap_byte(unsigned char *buf, int len)
{
	int i;
	unsigned char temp;

	for (i = 0; i < len;) {
		temp = buf[i];
		buf[i] = buf[i + 1];
		buf[i + 1] = temp;
		i += 2;
	}
}

static void second_panel_write_refresh(struct second_panel_par *par)
{
	char refresh_command[] = {0x2c};
	int ret;

	pr_info("spi %s\n", __func__);

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
	if (ret) {
		pr_err("select pins_dc_low failed with %d\n", ret);
		return;
	}

	st7735s_second_write_array(par->client, refresh_command, sizeof(refresh_command));
}

static void second_panel_output_image(struct second_panel_par *par, unsigned char *srcbuf)
{
	int ret, page_bytes;

	pr_info("spi %s\n", __func__);

	page_bytes = SECOND_PANEL_HEIGHT*SECOND_PANEL_WIDTH * 2;

	second_panel_write_refresh(par);

	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			return;
		}
	}

	buffer_swap_byte(srcbuf, page_bytes);
	st7735s_second_write_array(par->client, srcbuf, page_bytes);

}

/*
void second_change_565_mode_test(unsigned char r, unsigned char g, unsigned char b, unsigned char* p)
{
	unsigned char temp;

	temp = r/8;
	*(p+1) = temp*8;
	temp = g/4;
	*(p+1) += (temp/8);

	*p = temp%8;
	*p *= 32;
	temp = b/8;
	*p += temp;
}

void second_boot_linux_lcd_precheck_565_mode_test(void)
{

	unsigned int i = 0, j=0;
#define FMT_565_HEAD_BYTE_NUM 0
#define PANEL_565_SIZE 128*160*2
	{
		for(i = 0, j=0; i < 128*160*3; i+=3, j+=2)
			second_change_565_mode_test(0xFF, 0x00, 0x00, second_panel_buf+j);

	}
}


static void second_panel_display_test(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct second_panel_par *par = info->par;
	int size;
	pr_info("spi %s\n", __func__);
	second_boot_linux_lcd_precheck_565_mode_test();

	size = SECOND_PANEL_WIDTH * SECOND_PANEL_HEIGHT * 2;
	second_panel_init(par->client);
	second_panel_output_image(par, second_panel_buf);
	msleep(100);
}
*/


static void second_panel_update_display(struct second_panel_par *par)
{
	u8 *vmem = par->info->screen_base;

	second_panel_output_image(par, vmem);
}



static ssize_t second_panel_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct second_panel_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len = SECOND_PANEL_WIDTH * SECOND_PANEL_HEIGHT * 2;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);
	pr_info("spi %s count %d\n", __func__, count);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	second_panel_update_display(par);

	*ppos += count;
	if (*ppos >= total_size)
		*ppos  = 0;

	if (startup_flag) {
		msleep(100);
		set_second_spi_backlight(1);
		startup_flag = 0;
	}

	return count;
}



#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
static int second_panel_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	/* Get frame buffer memory range. */
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	/*struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;*/

	if (!start)
		return -EINVAL;

	if ((vma->vm_end <= vma->vm_start) ||
	    (off >= len) ||
	    ((vma->vm_end - vma->vm_start) > (len - off)))
		return -EINVAL;

	/* Set VM flags. */
	start &= PAGE_MASK;
	off += start;
	if (off < start)
		return -EINVAL;

	/*
	pr_err("%s again, start 0x%x, vm_start 0x%x, end 0x%x, len 0x%x, off 0x%x\n",
		__func__, (unsigned int)start, (unsigned int)vma->vm_start, (unsigned int)vma->vm_end,
			(unsigned int)len, (unsigned int)off);
	*/

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start,
				virt_to_phys((void *)off) >> PAGE_SHIFT, /* shall convert to physical address first */
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int second_panel_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	/*struct second_panel_par *par = info->par;*/
	if (var->rotate != FB_ROTATE_UR)
		return -EINVAL;
	if (var->grayscale != info->var.grayscale)
		return -EINVAL;

	if ((var->xres_virtual <= 0) || (var->yres_virtual <= 0))
		return -EINVAL;

	if (info->fix.smem_start) {
		u32 len = var->xres_virtual * var->yres_virtual *
			(var->bits_per_pixel / 8);
		if (len > info->fix.smem_len)
			return -EINVAL;
	}

	if ((var->xres == 0) || (var->yres == 0))
		return -EINVAL;

	if (var->xoffset > (var->xres_virtual - var->xres))
		return -EINVAL;

	if (var->yoffset > (var->yres_virtual - var->yres))
		return -EINVAL;

	return 0;
}

static int second_panel_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct second_panel_par *par = info->par;
	u8 *vmem = NULL;

	/*
	pr_err("%s, yoffset %d, yres %d, screen_base 0x%x, start 0x%x, screen_base 0x%x, start 0x%x\n",
		__func__, var->yoffset, var->yres, (unsigned int)par->info->screen_base,
		(unsigned int)par->info->fix.smem_start, (unsigned int)info->screen_base,
		(unsigned int)info->fix.smem_start);
	*/
	if (var->yoffset == var->yres)
		vmem = par->info->screen_base + SECOND_PANEL_WIDTH * SECOND_PANEL_HEIGHT * 2;
	else if (var->yoffset == 0)
		vmem = par->info->screen_base;
	else
		return -EINVAL;

	second_panel_output_image(par, vmem);
	return 0;
}


int second_panel_blank(int blank_mode, struct fb_info *info)
{
	if (blank_mode == FB_BLANK_UNBLANK) {
		u8 *vmem = g_par->info->screen_base;

		memset(vmem, 0, SECOND_PANEL_WIDTH * SECOND_PANEL_HEIGHT * 2);
		startup_flag = 1;
		second_panel_init(g_par->client);
		second_panel_output_image(g_par, vmem);
	} else {
		set_second_spi_backlight(0);
		second_panel_sleep(g_par->client);
	}

	pr_info("spi %s, blank_mode %d\n", __func__, blank_mode);

	return 0;
}

static int mdss_fb_ioctl_second(struct fb_info *info, unsigned int cmd,
			 unsigned long arg, struct file *file)
{
	if (!info || !info->par) {
		pr_info("spi error,%s info is null\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static struct fb_ops second_panel_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var = second_panel_check_var,	/* vinfo check */
	.fb_write	= second_panel_write,
	.fb_blank = second_panel_blank,	/* blank display */
	.fb_pan_display = second_panel_pan_display,	/* pan display */
	.fb_ioctl_v2 = mdss_fb_ioctl_second,	/* perform fb specific ioctl */
	/*
	.fb_fillrect	= st7735s_second_fillrect,
	.fb_copyarea	= st7735s_second_copyarea,
	.fb_imageblit	= st7735s_second_imageblit,
	*/
	.fb_mmap = second_panel_mmap,
};

#ifdef OUTPUT_LOGO_TEST
#define SPI_TEST_PERIOD_MS	2000
static void spi_test_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct second_panel_par *par = container_of(dwork,
				struct second_panel_par, test_work);

	second_panel_display_test(par->client);
	schedule_delayed_work(&par->test_work, msecs_to_jiffies(SPI_TEST_PERIOD_MS));
}
#endif



static int second_panel_prob(struct spi_device *client)
{
	struct fb_info *info;
	u32 vmem_size = SECOND_PANEL_WIDTH * SECOND_PANEL_HEIGHT * 2;
	struct second_panel_par *par;
	u8 *vmem;
	int ret;

	pr_info("%s\n", __func__);

	if (!client->dev.of_node) {
		dev_err(&client->dev, "No device tree data found!\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct second_panel_par), &client->dev);
	if (!info) {
		dev_err(&client->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}

	vmem_size = vmem_size > PAGE_SIZE ? vmem_size:PAGE_SIZE;
	vmem = kmalloc(vmem_size, GFP_KERNEL);		/*not to use devm_kzalloc(), have offset 0x10*/
	if (!vmem) {
		dev_err(&client->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto probe_error;
	}

	info->fbops = &second_panel_ops;
	info->fix = second_panel_fix;
	info->var = second_panel_var;
	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;
	memset(vmem, 0, vmem_size);
	ret = register_framebuffer(info);

	if (ret) {
		dev_err(&client->dev, "Couldn't register the framebuffer\n");
		/*goto probe_error;*/
	}

	par = info->par;
	g_par = par;
	par->info = info;
	par->client = client;

	dev_info(&client->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n",
		info->node, info->fix.id, vmem_size);

	par->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(par->pinctrl)) {
		pr_err("%s, error devm_pinctrl_get(), par->pinctrl 0x%x\n", __func__, (unsigned int)par->pinctrl);
		goto probe_error;
	} else {
		par->pins_dc_high = pinctrl_lookup_state(par->pinctrl, SECOND_PANEL_DC_HIGH);
		if (IS_ERR_OR_NULL(par->pins_dc_high)) {
			pr_err("%s, error pinctrl_lookup_state() for SECOND_PANEL_DC_HIGH\n", __func__);
			goto probe_error;
		}

		par->pins_dc_low = pinctrl_lookup_state(par->pinctrl, SECOND_PANEL_DC_LOW);
		if (IS_ERR_OR_NULL(par->pins_dc_low)) {
			pr_err("%s, error pinctrl_lookup_state() for SECOND_PANEL_DC_LOW\n", __func__);
			goto probe_error;
		}

		par->pins_rst_high = pinctrl_lookup_state(par->pinctrl, SECOND_PANEL_RST_HIGH);
		if (IS_ERR_OR_NULL(par->pins_rst_high)) {
			pr_err("%s, error pinctrl_lookup_state() for SECOND_PANEL_RST_HIGH\n", __func__);
			goto probe_error;
		}

		par->pins_rst_low = pinctrl_lookup_state(par->pinctrl, SECOND_PANEL_RST_LOW);
		if (IS_ERR_OR_NULL(par->pins_rst_low)) {
			pr_err("%s, error pinctrl_lookup_state() for SECOND_PANEL_RST_LOW\n", __func__);
			goto probe_error;
		}
	}


	spi_set_drvdata(client, info);
	startup_flag = 1;
	set_second_spi_backlight(0);

	second_panel_getid();
	second_panel_init(client);


#ifdef OUTPUT_LOGO_TEST
	INIT_DELAYED_WORK(&par->test_work, spi_test_work);
	schedule_delayed_work(&par->test_work, msecs_to_jiffies(20000));
#endif
	if (socinfo_get_charger_flag() == 0) {
		/*don't close it in power-off charger*/
		lcd_inited = 1;
		set_second_spi_backlight(0);
		second_panel_sleep(g_par->client);
	}


	return 0;

probe_error:
	unregister_framebuffer(info);

	framebuffer_release(info);
	return ret;
}

static int st7735s_second_spi_remove(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);

	unregister_framebuffer(info);

	framebuffer_release(info);

	return 0;
}

static void st7735s_second_spi_shutdown(struct spi_device *client)
{
	int ret;

	ret = pinctrl_select_state(g_par->pinctrl, g_par->pins_rst_low);
	if (ret)
		pr_err("select SECOND_PANEL_RST_LOW failed with %d\n", ret);
	pr_err("st7735s_second_spi_shutdown\n");

	msleep(120);
}

static const struct of_device_id st7735s_second_spi_of_match[] = {
	{ .compatible = "qcom,mdss-spi-second-panel" },
	{},
};
MODULE_DEVICE_TABLE(of, st7735s_second_spi_of_match);

static struct spi_driver second_panel_spi_driver = {
	.probe = second_panel_prob,
	.remove = st7735s_second_spi_remove,
	.shutdown = st7735s_second_spi_shutdown,
	.driver = {
		.name = "second_panel_spi",
		.of_match_table = of_match_ptr(st7735s_second_spi_of_match),
		.owner = THIS_MODULE,
	},
};


static int __init second_panel_spi_init(void)
{
	pr_info("%s\n", __func__);

	return spi_register_driver(&second_panel_spi_driver);
}

static void __exit second_panel_spi_exit(void)
{
	spi_unregister_driver(&second_panel_spi_driver);
}

module_init(second_panel_spi_init);
module_exit(second_panel_spi_exit);



