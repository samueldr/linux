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

#define FIRST_PANEL_WIDTH		240
#define FIRST_PANEL_HEIGHT		320

#define FIRST_PANEL_DC_HIGH "first_panel_dc_high"
#define FIRST_PANEL_DC_LOW "first_panel_dc_low"
#define FIRST_PANEL_RST_HIGH "first_panel_rst_high"
#define FIRST_PANEL_RST_LOW "first_panel_rst_low"
#define FIRST_PANEL_BACKLIGHT_HIGH "first_panel_backlight_high"
#define FIRST_PANEL_BACKLIGHT_LOW "first_panel_backlight_low"
#define FIRST_PANEL_VDDIO_HIGH "first_panel_vddio_high"
#define FIRST_PANEL_VDDIO_LOW "first_panel_vddio_low"
#define SPI_TIMER_PERIOD_MS	2000
#define SPI_USE_TE

struct mdss_first_panel_cmd {
	int size;
	char *payload;
	int wait;
};

struct first_panel_par {
	struct spi_device *client;
	struct fb_info *info;
	struct delayed_work		timer_work;
	int  reset;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_dc_high;
	struct pinctrl_state	*pins_dc_low;
	struct pinctrl_state	*pins_rst_high;
	struct pinctrl_state	*pins_rst_low;
	struct pinctrl_state	*pins_vddio_high;
	struct pinctrl_state	*pins_vddio_low;
	struct pinctrl_state	*pins_backlight_high;
	struct pinctrl_state	*pins_backlight_low;
	struct regulator  *first_panel_vddio;
	struct regulator  *first_panel_vdd;
	int panel_backlight_enable;
	int panel_backlight_level;
	int g_spi_cs_gpio;
	int g_spi_te_gpio;
	struct completion te_irq_comp;
	struct mutex spi_tx_mutex;
	struct workqueue_struct		*workqueue;
	struct work_struct		work;
	unsigned int buf_read_index;
	unsigned int buf_write_index;
	/* The framebuffer notifier block */
	struct notifier_block fb_notif;

};

static struct fb_fix_screeninfo first_panel_fix = {
	.id				= "ST7789V2",
	.type			= FB_TYPE_PACKED_PIXELS,
	.visual			= FB_VISUAL_MONO10,
	.xpanstep		= 1,
	.ypanstep		= 1,
	.ywrapstep		= 1,
	.line_length		= FIRST_PANEL_WIDTH * 2,
	.accel			= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo first_panel_var = {
	.xres			= FIRST_PANEL_WIDTH,
	.yres			= FIRST_PANEL_HEIGHT,
	.xres_virtual		= FIRST_PANEL_WIDTH,
	.yres_virtual		= FIRST_PANEL_HEIGHT,
	.bits_per_pixel	= 16,
};

static unsigned int startup_flag = 0;
static unsigned char first_panel_buf[FIRST_PANEL_HEIGHT*FIRST_PANEL_WIDTH * 2] = {0};



/*#define  OUTPUT_LOGO_TEST		1*/


static void first_panel_power_on(struct spi_device *client)
{
	pr_info("spi %s\n", __func__);

}

static void first_panel_power_off(struct spi_device *client)
{

	pr_info("spi %s--do nothing\n", __func__);

}

static int lcd_inited = 0;
static struct first_panel_par *g_par = NULL;
#define SPI_PANEL_CS_GPIO	18
#define SPI_PANEL_TE_GPIO	24
static int g_spi_cs_gpio = -1;
#define MAX_READ_SPEED_HZ	4800000
static uint max_write_speed_hz = 40000000;
#define SPI_PANEL_COMMAND_LEN	1
u8 esd_result[1];
u8 esd_check_value = 0x9c;

static int st7789v2_first_spi_read_data(u8 reg_addr, u8 *data, u8 len)
{
	int rc = 0;
	u32 max_speed_hz;
	u8 memory_write_reg = 0x2c;
	u8 empty_pack[] = {0x29, 0x29, 0x29};
	struct spi_transfer t[4] = {
		[0] = {
			.tx_buf = &reg_addr,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		},
		[2] = {
			.tx_buf = &empty_pack,
			.len = 3,
		},
		[3] = {
			.tx_buf = &memory_write_reg,
			.len = 1,
		}
	};
	struct spi_message m;

	if (!g_par) {
		pr_err("%s: spi client not available\n", __func__);
		return -EINVAL;
	}
	if (!g_par->client) {
		pr_err("%s: spi client not available\n", __func__);
		return -EINVAL;
	}

	g_par->client->bits_per_word = 8;
	max_speed_hz = g_par->client->max_speed_hz;
	g_par->client->max_speed_hz = MAX_READ_SPEED_HZ;
	gpio_set_value(g_par->g_spi_cs_gpio, 0);
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	rc = spi_sync(g_par->client, &m);

	spi_message_init(&m);
	spi_message_add_tail(&t[2], &m);
	rc = spi_sync(g_par->client, &m);
	spi_message_init(&m);
	spi_message_add_tail(&t[3], &m);
	rc = spi_sync(g_par->client, &m);
	gpio_set_value(g_par->g_spi_cs_gpio, 1);

	g_par->client->max_speed_hz = max_speed_hz;

	return rc;

}

int first_panel_spi_tx_command(const void *buf)
{
	int rc = 0;
	struct spi_transfer t = {
		.tx_buf = buf,
		.len    = SPI_PANEL_COMMAND_LEN,
	};
	struct spi_message m;

	if (!g_par) {
		pr_err("%s: spi client not available\n", __func__);
		return -EINVAL;
	}
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	max_write_speed_hz = g_par->client->max_speed_hz;
	g_par->client->max_speed_hz = ZTE_FEATURE_SPI_PANEL_SPEED_HZ;
#endif
	g_par->client->bits_per_word = 8;
	gpio_set_value(g_par->g_spi_cs_gpio, 0);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(g_par->client, &m);
	gpio_set_value(g_par->g_spi_cs_gpio, 1);
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	g_par->client->max_speed_hz = max_write_speed_hz;
#endif


	return rc;
}

int first_panel_spi_tx_parameter(const void *buf, size_t len)
{
	int rc = 0;
	struct spi_transfer t = {
		.tx_buf = buf,
		.len    = len,
	};
	struct spi_message m;

	if (!g_par->client) {
		pr_err("%s: spi client not available\n", __func__);
		return -EINVAL;
	}
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	max_write_speed_hz = g_par->client->max_speed_hz;
	g_par->client->max_speed_hz = ZTE_FEATURE_SPI_PANEL_SPEED_HZ;
#endif
	g_par->client->bits_per_word = 8;
	gpio_set_value(g_par->g_spi_cs_gpio, 0);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(g_par->client, &m);
	gpio_set_value(g_par->g_spi_cs_gpio, 1);
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	g_par->client->max_speed_hz = max_write_speed_hz;
#endif


	return rc;
}

int first_panel_spi_tx_pixel(const void *buf, size_t len)
{
	int rc = 0;
	struct spi_transfer t = {
		.tx_buf = buf,
		.len    = len,
		};
	struct spi_message m;

	if (!g_par->client) {
		pr_err("%s: spi client not available\n", __func__);
		return -EINVAL;
	}
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	max_write_speed_hz = g_par->client->max_speed_hz;
	g_par->client->max_speed_hz = ZTE_FEATURE_SPI_PANEL_SPEED_HZ;
#endif

	g_par->client->bits_per_word = 16;
	gpio_set_value(g_par->g_spi_cs_gpio, 0);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(g_par->client, &m);
	gpio_set_value(g_par->g_spi_cs_gpio, 1);
#ifdef ZTE_FEATURE_SPI_PANEL_SPEED_HZ
	g_par->client->max_speed_hz = max_write_speed_hz;
#endif

	return rc;
}


static void first_panel_reset(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct first_panel_par *par = info->par;
	int ret;

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_high);
	if (ret)
		pr_err("select FIRST_PANEL_RST_HIGH failed with %d\n", ret);

	msleep(20);

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_low);
	if (ret)
		pr_err("select FIRST_PANEL_RST_LOW failed with %d\n", ret);

	msleep(20);

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_high);
	if (ret)
		pr_err("select FIRST_PANEL_RST_HIGH failed with %d\n", ret);
	msleep(120);
	pr_info("spi %s\n", __func__);
}

void first_panel_sleep(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct first_panel_par *par = info->par;
	char display_off_command_28[] = {0x28};
	char display_off_command_10[] = {0x10};
	int ret;

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
	if (ret) {
		pr_err("select pins_dc_low failed with %d\n", ret);
		return;
	}

	first_panel_spi_tx_command(display_off_command_28);
	first_panel_spi_tx_command(display_off_command_10);

	ret = pinctrl_select_state(par->pinctrl, par->pins_rst_low);
	if (ret)
		pr_err("select pins_rst_low failed with %d\n", ret);
	msleep(120);

	first_panel_power_off(client);

	lcd_inited = 0;

	pr_info("spi %s\n", __func__);

}


/* check lcd id floating 2016.10.08 start*/
extern struct gpio_chip *chip_debug;
extern unsigned gpio_level_show_pm(int *id, struct gpio_chip *chip);
extern unsigned gpio_pull_store_pm(int *id, struct gpio_chip *chip, u32 values);
static int panel_id_is_floating = -1;
extern uint32_t check_panel_id_is_floating(int id);

/* check lcd id floating 2016.10.08 end*/

static unsigned int panel_id;
void first_panel_getid(void)
{
	int id_num = 61;
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


static void first_panel_write_refresh(struct first_panel_par *par)
{
	char refresh_command[] = {0x2c};
	int ret;

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
	if (ret) {
		pr_err("select pins_dc_low failed with %d\n", ret);
		return;
	}

	first_panel_spi_tx_command(&refresh_command[0]);
}


#define GPIO_PWM_BL_MAX_LIMIT 95
#define GPIO_PWM_BL_MIN_LIMIT 2
extern void zte_set_gpio_pwm_duty(int level);


int set_first_spi_backlight_level(int level)
{
	if (g_par == NULL)
		return -EINVAL;
	mutex_lock(&g_par->spi_tx_mutex);
	if (g_par->panel_backlight_enable) {
		zte_set_gpio_pwm_duty(level);
	} else {
		zte_set_gpio_pwm_duty(0);
	}
	g_par->panel_backlight_level = level;
	mutex_unlock(&g_par->spi_tx_mutex);
	pr_info("set_first_spi_backlight_level %d\n", level);
	return 0;
}

int set_first_spi_backlight(int enable)
{
	if (g_par == NULL)
		return -EINVAL;

	mutex_lock(&g_par->spi_tx_mutex);
	g_par->panel_backlight_enable = enable;
	zte_set_gpio_pwm_duty(g_par->panel_backlight_level);
	mutex_unlock(&g_par->spi_tx_mutex);
	pr_info("set_first_spi_backlight %d\n", enable);
	return 0;
}

static void first_panel_init(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);
	struct first_panel_par *par = info->par;
	int ret, i;
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd0[] = {
		0xfe,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd1[] = {
		0xef,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd2[] = {
		0x36, 0x48,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd2_1[] = {
		0x3a, 0x05,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd_TE[] = {
		0x35, 0x00,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd3[] = {
		0x21,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd4[] = {
		0x86, 0x98,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd5[] = {
		0x89, 0x03,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd6[] = {
		0x8b, 0x84,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd7[] = {
		0x8d, 0x22,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd8[] = {
		0x8e, 0x0f,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd9[] = {
		0xe8, 0x12, 0x00,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd10[] = {
		0xff, 0x62,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd11[] = {
		0x99, 0x3e,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd12[] = {
		0x9d, 0x4b,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd13[] = {
		0xc3, 0x30,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd14[] = {
		0xc4, 0x0b,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd15[] = {
		0xc9, 0x00,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd16[] = {
		0xf0, 0x88, 0x00, 0x1f, 0x16, 0x0b, 0x3b,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd17[] = {
		0xf1, 0x4f, 0xb7, 0x97, 0x25, 0x29, 0xbf,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd18[] = {
		0xf2, 0x4b, 0x00, 0x00, 0x19, 0x0d, 0x39,
	};

	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd19[] = {
		0xf3, 0x4a, 0xf5, 0xb4, 0x1d, 0x21, 0xcf,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd20[] = {
		0x11,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd21[] = {
		0x29,
	};
	static char lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd22[] = {
		0x2c,
	};

	static struct mdss_first_panel_cmd skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[] = {
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd0, 0x00},
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd1, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd2, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd2_1, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd_TE, 0x00},
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd3, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd4, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd5, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd6, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd7, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd8, 0x00},
	{0x03, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd9, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd10, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd11, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd12, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd13, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd14, 0x00},
	{0x02, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd15, 0x00},
	{0x07, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd16, 0x00},
	{0x07, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd17, 0x00},
	{0x07, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd18, 0x00},
	{0x07, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd19, 0x00},
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd20, 0x78},
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd21, 0x00},
	{0x01, lcetron_gc9307_ctc_2p8_qvga_spi_cmd_on_cmd22, 0x00},
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd0[] = {
		0x11,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd1[] = {
		0x36, 0x00,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd2[] = {
		0x3A, 0x05,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd_TE[] = {
		0x35, 0x00,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd3[] = {
		0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd4[] = {
		0xB7, 0x35,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd5[] = {
		0xBB, 0x11,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd6[] = {
		0xC0, 0x2C,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd7[] = {
		0xC2, 0x01,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd8[] = {
		0xC3, 0x12,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd9[] = {
		0xC4, 0x20,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd10[] = {
		0xC6, 0x15,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd11[] = {
		0xD0, 0xA4, 0xA1,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd11_1[] = {
		0x44, 0x00, 0x01,
	};


	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd12[] = {
		0xe0, 0xd0, 0x05, 0x0c, 0x13, 0x14, 0x2c, 0x3C, 0x45, 0x4d, 0x3B, 0x17, 0x16, 0x1F, 0x23,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd13[] = {
		0xe1, 0xd0, 0x05, 0x0c, 0x13, 0x14, 0x2c, 0x3C, 0x45, 0x4d, 0x3B, 0x17, 0x16, 0x1F, 0x23,
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd15[] = {
		0x29
	};

	static char coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd16[] = {
		0x2c
	};


	static struct mdss_first_panel_cmd coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[] = {
		{0x01, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd0, 0x78},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd1, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd2, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd_TE, 0x00},
		{0x06, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd3, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd4, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd5, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd6, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd7, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd8, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd9, 0x00},
		{0x02, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd10, 0x00},
		{0x03, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd11, 0x00},
		{0x03, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd11_1, 0x00},
		{0x0f, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd12, 0x00},
		{0x0f, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd13, 0x00},
		{0x01, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd15, 0x1A},
		{0x01, coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_cmd16, 0x00},
	};


	if (lcd_inited) {
		pr_info("first panel has been initialized already, not need to do again\n");
		return;
	}

	first_panel_power_on(client);

	first_panel_reset(client);

	if (!(par->pinctrl)) {
		pr_err("%s error! There is no pinctrl !\n", __func__);
		return;
	}

	if (panel_id == 0) {
		pr_info("%s: first panel is skyworth !\n", __func__);
		for (i = 0; i < ARRAY_SIZE(skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			first_panel_spi_tx_command(skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].payload);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].size > 1) {
				first_panel_spi_tx_parameter(
					skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].payload + 1,
						skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].size - 1);
			}

			if (skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].wait != 0)
				msleep(skyworth_st7789v2_cdy_2p8_qvga_spi_cmd_on_command[i].wait);
			}
	} else {
		pr_info("%s: first panel is coe !\n", __func__);
		for (i = 0; i < ARRAY_SIZE(coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command); i++) {

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
			if (ret) {
				pr_err("select pins_dc_low failed with %d\n", ret);
				return;
			}

			first_panel_spi_tx_command(coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].payload);

			ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
			if (ret) {
				pr_err("select pins_dc_high failed with %d\n", ret);
				return;
			}

			if (coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].size > 1) {
				first_panel_spi_tx_parameter(
					coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].payload + 1,
						coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].size - 1);
			}

			if (coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].wait != 0)
				msleep(coe_st7789v2_ivo_2p8_qvga_spi_cmd_on_command[i].wait);
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

	pr_info("spi %s\n", __func__);

}

/*static void buffer_swap_byte(unsigned char *buf, int len)
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
*/
static void first_panel_dead_reset(void)
{
	pr_err("first_panel_dead_reset!!!\n");
	first_panel_sleep(g_par->client);
	msleep(100);
	first_panel_init(g_par->client);
}

static void first_panel_output_image(struct first_panel_par *par, unsigned char *srcbuf)
{
	int ret, page_bytes;

	page_bytes = FIRST_PANEL_HEIGHT*FIRST_PANEL_WIDTH * 2;
	first_panel_write_refresh(par);
	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			return;
		}
	}
	first_panel_spi_tx_pixel(srcbuf, page_bytes);
}


static void first_panel_update_display(struct first_panel_par *par)
{
	u8 *vmem = NULL;
	int te_irq;
	static unsigned int old_read_index = 0;
	mutex_lock(&g_par->spi_tx_mutex);
	if (g_par->buf_read_index == old_read_index) {
		mutex_unlock(&g_par->spi_tx_mutex);
		return;
	}
	vmem = par->info->screen_base + (g_par->buf_read_index%2)*FIRST_PANEL_WIDTH * FIRST_PANEL_HEIGHT * 2;
	first_panel_output_image(par, vmem);
	old_read_index = g_par->buf_read_index;
	mutex_unlock(&g_par->spi_tx_mutex);
}

static ssize_t first_panel_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct first_panel_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	pr_info("first_panel_write lcd_inited %d startup_flag %d\n", lcd_inited, startup_flag);

	total_size = info->fix.smem_len = FIRST_PANEL_WIDTH * FIRST_PANEL_HEIGHT * 2;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;


	if (g_par == NULL)
		return -EINVAL;
	mutex_lock(&g_par->spi_tx_mutex);

	if (lcd_inited == 0) {
		mutex_unlock(&g_par->spi_tx_mutex);
		return -EINVAL;
	}
	dst = (void __force *) (info->screen_base + (g_par->buf_write_index%2)*total_size);
	g_par->buf_read_index = g_par->buf_write_index;
	g_par->buf_write_index++;

	if (copy_from_user(dst, buf, count)) {
		mutex_unlock(&g_par->spi_tx_mutex);
		return -EFAULT;
	}
	mutex_unlock(&g_par->spi_tx_mutex);

	*ppos += count;
	if (*ppos >= total_size)
		*ppos  = 0;

	if (startup_flag) {
		msleep(100);
		set_first_spi_backlight(1);
		startup_flag = 0;
		schedule_delayed_work(&par->timer_work, msecs_to_jiffies(SPI_TIMER_PERIOD_MS));
	}

	return count;
}



#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
static int first_panel_mmap(struct fb_info *info, struct vm_area_struct *vma)
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


	pr_info("%s again, start 0x%x, vm_start 0x%x, end 0x%x, len 0x%x, off 0x%x\n",
		__func__, (unsigned int)start, (unsigned int)vma->vm_start, (unsigned int)vma->vm_end,
			(unsigned int)len, (unsigned int)off);


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

static int first_panel_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	/*struct first_panel_par *par = info->par;*/
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

static int first_panel_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct first_panel_par *par = info->par;
	u8 *vmem = NULL;


	pr_info("%s, yoffset %d, yres %d, screen_base 0x%x, start 0x%x, screen_base 0x%x, start 0x%x\n",
		__func__, var->yoffset, var->yres, (unsigned int)par->info->screen_base,
		(unsigned int)par->info->fix.smem_start, (unsigned int)info->screen_base,
		(unsigned int)info->fix.smem_start);

	if (var->yoffset == var->yres)
		vmem = par->info->screen_base + FIRST_PANEL_WIDTH * FIRST_PANEL_HEIGHT * 2;
	else if (var->yoffset == 0)
		vmem = par->info->screen_base;
	else
		return -EINVAL;

	first_panel_output_image(par, vmem);
	return 0;
}


int first_panel_blank(int blank_mode, struct fb_info *info)
{
	pr_info("%s blank_mode %d lcd_inited %d\n", __func__, blank_mode, lcd_inited);

	if (g_par == NULL)
		return 0;
	if (blank_mode == FB_BLANK_UNBLANK) {
		mutex_lock(&g_par->spi_tx_mutex);
		if (lcd_inited) {
			mutex_unlock(&g_par->spi_tx_mutex);
			return 0;
		}
		startup_flag = 1;
		g_par->buf_read_index = 0;
		g_par->buf_write_index = 1;
		first_panel_init(g_par->client);
		mutex_unlock(&g_par->spi_tx_mutex);
	} else if (blank_mode == FB_BLANK_POWERDOWN) {
		set_first_spi_backlight(0);
		mutex_lock(&g_par->spi_tx_mutex);
		first_panel_sleep(g_par->client);
		g_par->buf_read_index = 0;
		g_par->buf_write_index = 1;
		mutex_unlock(&g_par->spi_tx_mutex);
	}

	pr_info("%s blank_mode %d lcd_inited %d startup_flag %d\n", __func__, blank_mode, lcd_inited, startup_flag);
	return 0;
}

static int mdss_fb_ioctl_first(struct fb_info *info, unsigned int cmd,
			 unsigned long arg, struct file *file)
{
	if (!info || !info->par) {
		pr_info("spi error,%s info is null\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static struct fb_ops first_panel_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var = first_panel_check_var,	/* vinfo check */
	.fb_write	= first_panel_write,
	.fb_blank = first_panel_blank,	/* blank display */
	.fb_pan_display = first_panel_pan_display,	/* pan display */
	.fb_ioctl_v2 = mdss_fb_ioctl_first,	/* perform fb specific ioctl */
	/*
	.fb_fillrect	= st7789v2_first_fillrect,
	.fb_copyarea	= st7789v2_first_copyarea,
	.fb_imageblit	= st7789v2_first_imageblit,
	*/
	.fb_mmap = first_panel_mmap,
};





static void spi_timer_work(struct work_struct *work)
{
	int ret;
	struct delayed_work *dwork = to_delayed_work(work);
	struct first_panel_par *par = container_of(dwork,
				struct first_panel_par, timer_work);

	if (g_par == NULL)
		return;
	mutex_lock(&g_par->spi_tx_mutex);
	if (lcd_inited == 0) {
		mutex_unlock(&g_par->spi_tx_mutex);
		return;
	}

	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_low);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			goto next;
		}
	}
	st7789v2_first_spi_read_data(0x0A, esd_result, 1);
	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			goto next;
		}
	}

	if (esd_check_value != esd_result[0] && lcd_inited == 1) {
		pr_info("spi_timer_work error value 0x%x lcd_inited %d\n", esd_result[0], lcd_inited);
		first_panel_dead_reset();
	}

next:
	mutex_unlock(&g_par->spi_tx_mutex);
	schedule_delayed_work(&par->timer_work, msecs_to_jiffies(SPI_TIMER_PERIOD_MS));
	return;
}



static void first_panel_work(struct work_struct *work)
{
	first_panel_update_display(g_par);
}

#ifdef SPI_USE_TE
static irqreturn_t spi_hw_vsync_handler(int irq, void *data)
{
	queue_work(g_par->workqueue, &g_par->work);
	return IRQ_HANDLED;
}
#endif

static int first_panel_prob(struct spi_device *client)
{
	struct fb_info *info;
	u32 vmem_size = FIRST_PANEL_WIDTH * FIRST_PANEL_HEIGHT * 4;
	struct first_panel_par *par;
	u8 *vmem;
	int ret;

	pr_info("%s\n", __func__);

	if (!client->dev.of_node) {
		dev_err(&client->dev, "No device tree data found!\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct first_panel_par), &client->dev);
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

	info->fbops = &first_panel_ops;
	info->fix = first_panel_fix;
	info->var = first_panel_var;
	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;

	ret = register_framebuffer(info);

	if (ret) {
		dev_err(&client->dev, "Couldn't register the framebuffer\n");
		/*goto probe_error;*/
	}

	par = info->par;
	par->info = info;
	par->client = client;

	dev_info(&client->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n",
		info->node, info->fix.id, vmem_size);

	par->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(par->pinctrl)) {
		pr_err("%s, error devm_pinctrl_get(), par->pinctrl 0x%x\n", __func__, (unsigned int)par->pinctrl);
		goto probe_error;
	} else {
		par->pins_dc_high = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_DC_HIGH);
		if (IS_ERR_OR_NULL(par->pins_dc_high)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_DC_HIGH\n", __func__);
			goto probe_error;
		}

		par->pins_dc_low = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_DC_LOW);
		if (IS_ERR_OR_NULL(par->pins_dc_low)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_DC_LOW\n", __func__);
			goto probe_error;
		}

		par->pins_rst_high = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_RST_HIGH);
		if (IS_ERR_OR_NULL(par->pins_rst_high)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_RST_HIGH\n", __func__);
			goto probe_error;
		}

		par->pins_rst_low = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_RST_LOW);
		if (IS_ERR_OR_NULL(par->pins_rst_low)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_RST_LOW\n", __func__);
			goto probe_error;
		}

		par->pins_backlight_high = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_BACKLIGHT_HIGH);
		if (IS_ERR_OR_NULL(par->pins_backlight_high)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_BACKLIGHT_HIGH\n", __func__);
			goto probe_error;
		}

		par->pins_backlight_low = pinctrl_lookup_state(par->pinctrl, FIRST_PANEL_BACKLIGHT_LOW);
		if (IS_ERR_OR_NULL(par->pins_backlight_low)) {
			pr_err("%s, error pinctrl_lookup_state() for FIRST_PANEL_BACKLIGHT_LOW\n", __func__);
			goto probe_error;
		}
	}
	if (par->pinctrl) {
		ret = pinctrl_select_state(par->pinctrl, par->pins_dc_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			goto probe_error;
		}
		ret = pinctrl_select_state(par->pinctrl, par->pins_backlight_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			goto probe_error;
		}
		ret = pinctrl_select_state(par->pinctrl, par->pins_rst_high);
		if (ret) {
			pr_err("select pins_dc_high failed with %d\n", ret);
			goto probe_error;
		}

	}


	g_par = par;
	g_par->buf_read_index = 0;
	g_par->buf_write_index = 1;
	mutex_init(&g_par->spi_tx_mutex);
	g_par->workqueue = alloc_ordered_workqueue
								("spi_main_workqueue", 0);
	INIT_WORK(&g_par->work, first_panel_work);

	g_par->g_spi_cs_gpio = of_get_named_gpio(client->dev.of_node,
		"zte,spi-cs-gpio", 0);
	if (!gpio_is_valid(g_par->g_spi_cs_gpio)) {
		pr_err("%s:g_spi_cs_gpio not specified\n", __func__);
		g_par->g_spi_cs_gpio = SPI_PANEL_CS_GPIO;
	} else {
		pr_info("%s:g_spi_cs_gpio is gpio %d\n", __func__, g_par->g_spi_cs_gpio);
	}
	g_par->g_spi_te_gpio = of_get_named_gpio(client->dev.of_node,
		"zte,spi-te-gpio", 0);

	if (!gpio_is_valid(g_par->g_spi_te_gpio)) {
		pr_err("%s:%d, TE gpio not specified\n",
						__func__, __LINE__);
		g_par->g_spi_te_gpio = SPI_PANEL_TE_GPIO;
	}
#ifdef SPI_USE_TE
		init_completion(&g_par->te_irq_comp);
		ret = devm_request_irq(&client->dev,
			gpio_to_irq(g_par->g_spi_te_gpio),
			spi_hw_vsync_handler, IRQF_TRIGGER_RISING,
			"SPI_VSYNC_GPIO", g_par);
		if (ret) {
			pr_err("%s: TE request_irq failed for ESD\n", __func__);
		} else {
			pr_info("%s:g_spi_cs_gpio is gpio %d\n", __func__, g_par->g_spi_te_gpio);
		}
#endif

	spi_set_drvdata(client, info);
	first_panel_getid();
	g_par->panel_backlight_level = 60;
	set_first_spi_backlight(1);

	pr_info("spi %s\n", __func__);


	INIT_DELAYED_WORK(&par->timer_work, spi_timer_work);
	schedule_delayed_work(&par->timer_work, msecs_to_jiffies(2000));

	lcd_inited = 1;
	return 0;

probe_error:
	unregister_framebuffer(info);

	framebuffer_release(info);
	return ret;
}

static int st7789v2_first_spi_remove(struct spi_device *client)
{
	struct fb_info *info = spi_get_drvdata(client);

	unregister_framebuffer(info);

	framebuffer_release(info);

	return 0;
}

static void st7789v2_first_spi_shutdown(struct spi_device *client)
{
	int ret;

	ret = pinctrl_select_state(g_par->pinctrl, g_par->pins_rst_low);
	if (ret)
		pr_err("select FIRST_PANEL_RST_LOW failed with %d\n", ret);
	pr_err("st7789v2_first_spi_shutdown\n");

	msleep(120);
}


static const struct of_device_id st7789v2_first_spi_of_match[] = {
	{ .compatible = "qcom,mdss-spi-first-panel" },
	{},
};
MODULE_DEVICE_TABLE(of, st7789v2_first_spi_of_match);

static struct spi_driver first_panel_spi_driver = {
	.probe = first_panel_prob,
	.remove = st7789v2_first_spi_remove,
	.shutdown = st7789v2_first_spi_shutdown,
	.driver = {
		.name = "first_panel_spi",
		.of_match_table = of_match_ptr(st7789v2_first_spi_of_match),
		.owner = THIS_MODULE,
	},
};


static int __init first_panel_spi_init(void)
{
	pr_info("%s\n", __func__);

	return spi_register_driver(&first_panel_spi_driver);
}

static void __exit first_panel_spi_exit(void)
{
	spi_unregister_driver(&first_panel_spi_driver);
}

module_init(first_panel_spi_init);
module_exit(first_panel_spi_exit);



