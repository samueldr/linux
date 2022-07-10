/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/console.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <linux/clk-provider.h>
#include <video/of_display_timing.h>
#include <linux/gpio.h>
#include <linux/omapfb.h>
#include <linux/compiler.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/arch-suniv/cpu.h>
#include <asm/arch-suniv/dma.h>
#include <asm/arch-suniv/gpio.h>
#include <asm/arch-suniv/intc.h>
#include <asm/arch-suniv/lcdc.h>
#include <asm/arch-suniv/debe.h>
#include <asm/arch-suniv/codec.h>
#include <asm/arch-suniv/clock.h>
#include <asm/arch-suniv/common.h>

////////////////////////////////////////////////////////////////////////////////
// ST7789S commands
// {{{

// Table 1
#define ST7789S_CMD_NOP        (0x00)
#define ST7789S_CMD_SWRESET    (0x01) // Software Reset
#define ST7789S_CMD_RDDID      (0x04) // Read Display ID
#define ST7789S_CMD_RDDST      (0x09) // Read Display Status
#define ST7789S_CMD_RDDPM      (0x0A) // Read Display Power Mode
#define ST7789S_CMD_RDDMADCTL  (0x0B) // Read Display MADCTL
#define ST7789S_CMD_RDDCOLMOD  (0x0C) // Read Display Pixel Format
#define ST7789S_CMD_RDDIM      (0x0D) // Read Display Image Mode
#define ST7789S_CMD_RDDSM      (0x0E) // Read Display Signal Mode
#define ST7789S_CMD_RDDSDR     (0x0F) // Read Display Self-Diagnostic Result
#define ST7789S_CMD_SLPIN      (0x10) // Sleep in
#define ST7789S_CMD_SLPOUT     (0x11) // Sleep Out
#define ST7789S_CMD_PTLON      (0x12) // Partial Display Mode On
#define ST7789S_CMD_NORON      (0x13) // Normal Display Mode On
#define ST7789S_CMD_INVOFF     (0x20) // Display Inversion Off
#define ST7789S_CMD_INVON      (0x21) // Display Inversion On
#define ST7789S_CMD_GAMSET     (0x26) // Gamma Set
#define ST7789S_CMD_DISPOFF    (0x28) // Display Off
#define ST7789S_CMD_DISPON     (0x29) // Display On
#define ST7789S_CMD_CASET      (0x2A) // Column Address Set
#define ST7789S_CMD_RASET      (0x2B) // Row Address Set
#define ST7789S_CMD_RAMWR      (0x2C) // Memory Write
#define ST7789S_CMD_RAMRD      (0x2E) // Memory Read
#define ST7789S_CMD_PTLAR      (0x30) // Partial Area
#define ST7789S_CMD_VSCRDEF    (0x33) // Vertical Scrolling Definition
#define ST7789S_CMD_TEOFF      (0x34) // Tearing Effect Line OFF
#define ST7789S_CMD_TEON       (0x35) // Tearing Effect Line On
#define ST7789S_CMD_MADCTL     (0x36) // Memory Data Access Control
#define ST7789S_MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define ST7789S_MADCTL_ML  BIT(4) /* bitmask for vertical refresh order */
#define ST7789S_MADCTL_MV  BIT(5) /* bitmask for page/column order */
#define ST7789S_MADCTL_MX  BIT(6) /* bitmask for column address order */
#define ST7789S_MADCTL_MY  BIT(7) /* bitmask for page address order */
#define ST7789S_CMD_VSCSAD     (0x37) // Vertical Scroll Start Address of RAM
#define ST7789S_CMD_IDMOFF     (0x38) // Idle Mode Off
#define ST7789S_CMD_IDMON      (0x39) // Idle mode on
#define ST7789S_CMD_COLMOD     (0x3A) // Interface Pixel Format
#define ST7789S_CMD_WRMEMC     (0x3C) // Write Memory Continue
#define ST7789S_CMD_RDMEMC     (0x3E) // Read Memory Continue
#define ST7789S_CMD_STE        (0x44) // Set Tear Scanline
#define ST7789S_CMD_GSCAN      (0x45) // Get Scanline
#define ST7789S_CMD_WRDISBV    (0x51) // Write Display Brightness
#define ST7789S_CMD_RDDISBV    (0x52) // Read Display Brightness Value
#define ST7789S_CMD_WRCTRLD    (0x53) // Write CTRL Display
#define ST7789S_CMD_RDCTRLD    (0x54) // Read CTRL Value Display
#define ST7789S_CMD_WRCACE     (0x55) // Write Content Adaptive Brightness Control and Color Enhancement
#define ST7789S_CMD_RDCABC     (0x56) // Read Content Adaptive Brightness Control
#define ST7789S_CMD_WRCABCMB   (0x5E) // Write CABC Minimum Brightness
#define ST7789S_CMD_RDCABCMB   (0x5F) // Read CABC Minimum Brightness
#define ST7789S_CMD_RDID1      (0xDA) // Read ID1
#define ST7789S_CMD_RDID2      (0xDB) // Read ID2
#define ST7789S_CMD_RDID3      (0xDC) // Read ID3

// Table 2
#define ST7789S_CMD_RAMCTRL    (0xB0) // RAM Control
#define ST7789S_RAMCTRL_1_RM_NORMAL       (0)
#define ST7789S_RAMCTRL_1_RM_RGB          BIT(4)
// Choice between those options
#define ST7789S_RAMCTRL_1_DM_MCU          (0)
#define ST7789S_RAMCTRL_1_DM_RGB          (1)
#define ST7789S_RAMCTRL_1_DM_VSYNC        (2)
//                                        (3) is reserved
#define ST7789S_RAMCTRL_ENDIAN_NORMAL   (0 << 3)
#define ST7789S_RAMCTRL_ENDIAN_LITTLE   (1 << 3)
#define ST7789S_RAMCTRL_MAGIC           (3 << 6)
#define ST7789S_RAMCTRL_EPF(n)          (((n) & 3) << 4)

#define ST7789S_CMD_RGBCTRL    (0xB1) // RGB Interface Control
#define ST7789S_CMD_PORCTRL    (0xB2) // Porch Setting
#define ST7789S_CMD_FRCTRL1    (0xB3) // Frame Rate Control 1 (In partial mode/ idle colors)
#define ST7789S_CMD_GCTRL      (0xB7) // Gate Control
#define ST7789S_CMD_DGMEN      (0xBA) // Digital Gamma Enable
#define ST7789S_CMD_VCOMS      (0xBB) // VCOM Setting
#define ST7789S_CMD_LCMCTRL    (0xC0) // LCM Control
#define ST7789S_CMD_IDSET      (0xC1) // ID Code Setting
#define ST7789S_CMD_VDVVRHEN   (0xC2) // VDV and VRH Command Enable
#define ST7789S_CMD_VRHS       (0xC3) // VRH Set
#define ST7789S_CMD_VDVS       (0xC4) // VDV Set
#define ST7789S_CMD_VCMOFSET   (0xC5) // VCOM Offset Set
#define ST7789S_CMD_FRCTRL2    (0xC6) // Frame Rate Control in Normal Mode
#define ST7789S_CMD_CABCCTRL   (0xC7) // CABC Control
#define ST7789S_CMD_REGSEL1    (0xC8) // Register Value Selection 1
#define ST7789S_CMD_REGSEL2    (0xCA) // Register Value Selection 2
#define ST7789S_CMD_PWCTRL1    (0xD0) // Power Control 1
#define ST7789S_CMD_VAPVANEN   (0xD2) // Enable VAP/VAN signal output
#define ST7789S_CMD_PVGAMCTRL  (0xE0) // Positive Voltage Gamma Control
#define ST7789S_CMD_NVGAMCTRL  (0xE1) // Negative Voltage Gamma Control
#define ST7789S_CMD_DGMLUTR    (0xE2) // Digital Gamma Look-up Table for Red
#define ST7789S_CMD_DGMLUTB    (0xE3) // Digital Gamma Look-up Table for Blue
#define ST7789S_CMD_GATECTRL   (0xE4) // Gate Control
#define ST7789S_CMD_SPI2EN     (0xE7) // SPI2 Enable
#define ST7789S_CMD_PWCTRL2    (0xE8) // Power Control 2
#define ST7789S_CMD_EQCTRL     (0xE9) // Equalize time control
#define ST7789S_CMD_PROMCTRL   (0xEC) // Program Mode Control
#define ST7789S_CMD_PROMEN     (0xFA) // Program Mode Enable
#define ST7789S_CMD_NVMSET     (0xFC) // NVM Setting
#define ST7789S_CMD_PROMACT    (0xFE) // Program action

// }}}
////////////////////////////////////////////////////////////////////////////////

#define PALETTE_SIZE 256
#define DRIVER_NAME  "ST7789S-fb"
DECLARE_WAIT_QUEUE_HEAD(wait_vsync_queue);

struct myfb_app{
    uint32_t yoffset;
    uint32_t vsync_count;
};

struct myfb_par {
    struct device *dev;
    struct platform_device *pdev;

    resource_size_t p_palette_base;
    unsigned short *v_palette_base;

    void *vram_virt;
    uint32_t vram_size;
    dma_addr_t vram_phys;
    struct myfb_app *app_virt;

    int bpp;
    int lcdc_irq;
    int gpio_irq;
    int lcdc_ready;
    u32 pseudo_palette[16];
    struct fb_videomode mode;
};

struct suniv_iomm {
    uint8_t *dma;
    uint8_t *ccm;
    uint8_t *gpio;
    uint8_t *lcdc;
    uint8_t *debe;
    uint8_t *intc;
    uint8_t *timer;
};
static int major = -1;
static struct cdev mycdev;
static struct class *myclass = NULL;
struct timer_list mytimer;
static struct suniv_iomm iomm={0};
static struct myfb_par *mypar=NULL;
static struct fb_var_screeninfo myfb_var={0};

static struct fb_fix_screeninfo myfb_fix = {
        .id = DRIVER_NAME,
        .type = FB_TYPE_PACKED_PIXELS,
        .type_aux = 0,
        .visual = FB_VISUAL_TRUECOLOR,
        .xpanstep = 0,
        .ypanstep = 1,
        .ywrapstep = 0,
        .accel = FB_ACCEL_NONE
};

/**
 * Originally named suniv_gpio_init.
 * It does not initialize GPIO, but instead configures the multi-usage
 * GPIO pins to their LCD_XXX mode.
 */
static void miyoo_lcd_pinmux(void)
{
	// Temporary buffer
	uint32_t r=0;

	// Read current configuration
	r = readl(iomm.gpio + PD_CFG0);
	// Resets everything, except bits 0:3 (PD0:btn_select)
	r&= 0x0000000f;
	//     LCD_D11  LDC_D10  LCD_D7  LCD_D6  LCD_D5  LCD_D4  LCD_D3   (PD0)
	// => ["0010",  "0010",  "0010", "0010", "0010", "0010", "0010", "0000"]
	r|= 0x22222220;
	writel(r, iomm.gpio + PD_CFG0);

	// Read current configuration
	r = readl(iomm.gpio + PD_CFG1);
	// Resets everything, except bits 4:7 (PD9:btn_X)
	r&= 0x000000f0;
	//     LCD_D21  LCD_D20  LCD_D19  LCD_D18  LCD_D15  LCD_D14   (PD9)  LCD_D12
	// => ["0010",  "0010",  "0010",  "0010",  "0010",  "0010",  "0000", "0010"]
	r|= 0x22222202;
	writel(r, iomm.gpio + PD_CFG1);

	// Read current configuration
	r = readl(iomm.gpio + PD_CFG2);
	// Resets everything, except bits 24:31 (reserved)
	r&= 0xff000000;
	//    __ reserved ___  LCD_VSYNC  LCD_HSYNC  LCD_DE  LCD_CLK  LCD_D23  LCD_D22
	// => ["0000", "0000", "0010",    "0010",    "0010", "0010",  "0010",  "0010"]
	r|= 0x000222222;
	writel(r, iomm.gpio + PD_CFG2);

	// Read current configuration
	r = readl(iomm.gpio + PD_PUL1);
	// Resets 8:11
	r&= 0xfffff0ff;
	// Pull-Up enable for PD4, PD5 [LCD_D6, LCD_D7]
	// 01 01 00 00 00 00
	r|= 0x00000500;
	writel(r, iomm.gpio + PD_PUL1);

	// Read current configuration
	r = readl(iomm.gpio + PE_CFG1);
	// Resets 12:15
	r&= 0xffff0fff;
	//     ______ reserved ______  PE12    PE11    PE10    PE9     PE8
	//                                     output
	// => ["0000", "0000", "0000", "0000", "0001", "0000", "0000", "0000"]
	// (NOTE: PE11 is supposedly connected to the LCD:RESX line, unverified)
	r|= 0x00001000;
	writel(r, iomm.gpio + PE_CFG1);

	// Sets initial state for reset line.
	writel(0xffffffff, iomm.gpio + PE_DATA);
}

static uint32_t lcdc_wait_busy(void)
{
    uint32_t cnt=0;

    suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 0));
    ndelay(10);
    while(1){
        if(readl(iomm.lcdc + TCON0_CPU_IF_REG) & 0x00c00000){
            if(cnt > 200){
                return -1;
            }
            else{
                cnt+= 1;
            }
        }
        break;
    }
    return 0;
}

static uint32_t extend_16b_to_24b(uint32_t value)
{
    return ((value & 0xfc00) << 8) | ((value & 0x0300) << 6) | ((value & 0x00e0) << 5) | ((value & 0x001f) << 3);
}

static void lcdc_wr(uint8_t is_data, uint32_t data)
{
    while(lcdc_wait_busy());
    if(is_data){
        suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 25));
    }
    else{
        suniv_clrbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 25));
    }
    while(lcdc_wait_busy());
    writel(extend_16b_to_24b(data), iomm.lcdc + TCON0_CPU_WR_REG);
}

static void lcdc_wr_cmd(uint32_t cmd)
{
    lcdc_wr(0, cmd);
}

static void lcdc_wr_dat(uint32_t cmd)
{
    lcdc_wr(1, cmd);
}

static void refresh_lcd(struct myfb_par *par)
{
    suniv_clrbits(iomm.lcdc + TCON_INT_REG0, (1 << 15));
    suniv_clrbits(iomm.lcdc + TCON_CTRL_REG, (1 << 31));

    if(par->lcdc_ready){
        lcdc_wr_cmd(ST7789S_CMD_RAMWR);
        if(par->app_virt->yoffset == 0){
            suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 8));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 9));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 10));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 11));
        }
        else if(par->app_virt->yoffset == 240){
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 8));
            suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 9));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 10));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 11));
        }
        else if(par->app_virt->yoffset == 480){
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 8));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 9));
            suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 10));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 11));
        }
        else if(par->app_virt->yoffset == 720){
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 8));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 9));
            suniv_clrbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 10));
            suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 11));
        }
    }
    suniv_setbits(iomm.debe + DEBE_REGBUFF_CTRL_REG, (1 << 0));
    suniv_setbits(iomm.lcdc + TCON_CTRL_REG, (1 << 31));

    par->app_virt->vsync_count+= 1;
    wake_up_interruptible_all(&wait_vsync_queue);
}

static irqreturn_t gpio_irq_handler(int irq, void *arg)
{
    refresh_lcd(arg);
    return IRQ_HANDLED;
}

static irqreturn_t lcdc_irq_handler(int irq, void *arg)
{
    refresh_lcd(arg);
    suniv_clrbits(iomm.lcdc + TCON_INT_REG0, (1 << 15));
    return IRQ_HANDLED;
}

static void init_lcd(void)
{
	miyoo_lcd_pinmux();

	// Turn the display off and on again
	suniv_clrbits(iomm.lcdc + PE_DATA, (1 << 11)); // Unsets PE11 (RESX)
	mdelay(150);
	suniv_setbits(iomm.lcdc + PE_DATA, (1 << 11)); // Sets PE11 (RESX)
	mdelay(150);

	// Bring out of sleep
	lcdc_wr_cmd(ST7789S_CMD_SLPOUT);
	mdelay(50);

	lcdc_wr_cmd(ST7789S_CMD_MADCTL);
	lcdc_wr_dat(0xb0);

	lcdc_wr_cmd(ST7789S_CMD_COLMOD);
	lcdc_wr_dat(0x05);

	lcdc_wr_cmd(ST7789S_CMD_CASET);
	lcdc_wr_dat(0x00);
	lcdc_wr_dat(0x00);
	lcdc_wr_dat(0x01);
	lcdc_wr_dat(0x3f);

	lcdc_wr_cmd(ST7789S_CMD_RASET);
	lcdc_wr_dat(0x00);
	lcdc_wr_dat(0x00);
	lcdc_wr_dat(0x00);
	lcdc_wr_dat(0xef);

	lcdc_wr_cmd(ST7789S_CMD_PORCTRL); // <- differs
	lcdc_wr_dat(116);
	lcdc_wr_dat(16);
	lcdc_wr_dat(0x01);
	lcdc_wr_dat(0x33);
	lcdc_wr_dat(0x33);

	lcdc_wr_cmd(ST7789S_CMD_GCTRL);
	lcdc_wr_dat(0x35);

	lcdc_wr_cmd(0xb8); // unknown likely ST7789_GTADJ
	lcdc_wr_dat(0x2f);
	lcdc_wr_dat(0x2b);
	lcdc_wr_dat(0x2f);

	lcdc_wr_cmd(ST7789S_CMD_VCOMS);
	lcdc_wr_dat(0x15);

	lcdc_wr_cmd(ST7789S_CMD_LCMCTRL);
	lcdc_wr_dat(0x3c);

	lcdc_wr_cmd(ST7789S_CMD_TEON); // <- configured at the end in my driver
	lcdc_wr_dat(0x00);

	lcdc_wr_cmd(ST7789S_CMD_VDVVRHEN);
	lcdc_wr_dat(0x01);

	lcdc_wr_cmd(ST7789S_CMD_VRHS);
	lcdc_wr_dat(0x13);

	lcdc_wr_cmd(ST7789S_CMD_VDVS);
	lcdc_wr_dat(0x20);

	lcdc_wr_cmd(ST7789S_CMD_FRCTRL2);
	lcdc_wr_dat(0x07); // <- 0x0f on my end

	lcdc_wr_cmd(ST7789S_CMD_PWCTRL1);
	lcdc_wr_dat(0xa4);
	lcdc_wr_dat(0xa1);

	lcdc_wr_cmd(ST7789S_CMD_PWCTRL2);
	lcdc_wr_dat(0x03);

	lcdc_wr_cmd(ST7789S_CMD_EQCTRL);
	lcdc_wr_dat(0x0d);
	lcdc_wr_dat(0x12);
	lcdc_wr_dat(0x00);

	lcdc_wr_cmd(ST7789S_CMD_PVGAMCTRL);
	lcdc_wr_dat(0xd0);
	lcdc_wr_dat(0x08);
	lcdc_wr_dat(0x10);
	lcdc_wr_dat(0x0d);
	lcdc_wr_dat(0x0c);
	lcdc_wr_dat(0x07);
	lcdc_wr_dat(0x37);
	lcdc_wr_dat(0x53);
	lcdc_wr_dat(0x4c);
	lcdc_wr_dat(0x39);
	lcdc_wr_dat(0x15);
	lcdc_wr_dat(0x15);
	lcdc_wr_dat(0x2a);
	lcdc_wr_dat(0x2d);

	lcdc_wr_cmd(ST7789S_CMD_NVGAMCTRL);
	lcdc_wr_dat(0xd0);
	lcdc_wr_dat(0x0d);
	lcdc_wr_dat(0x12);
	lcdc_wr_dat(0x08);
	lcdc_wr_dat(0x08);
	lcdc_wr_dat(0x15);
	lcdc_wr_dat(0x34);
	lcdc_wr_dat(0x34);
	lcdc_wr_dat(0x4a);
	lcdc_wr_dat(0x36);
	lcdc_wr_dat(0x12);
	lcdc_wr_dat(0x13);
	lcdc_wr_dat(0x2b);
	lcdc_wr_dat(0x2f);

	// Missing: CMD_STE (set tear scanline)

	lcdc_wr_cmd(ST7789S_CMD_DISPON);
	lcdc_wr_cmd(ST7789S_CMD_RAMWR);

	mypar->app_virt->yoffset = 0;
	memset(mypar->vram_virt, 0, 320*240*4);
}

static void suniv_lcdc_init(struct myfb_par *par)
{
    uint32_t ret=0, p1=0, p2=0;

    writel(0, iomm.lcdc + TCON_CTRL_REG);
    writel(0, iomm.lcdc + TCON_INT_REG0);
    ret = readl(iomm.lcdc + TCON_CLK_CTRL_REG);
    ret&= ~(0xf << 28);
    writel(ret, iomm.lcdc + TCON_CLK_CTRL_REG);
    writel(0xffffffff, iomm.lcdc + TCON0_IO_CTRL_REG1);
    writel(0xffffffff, iomm.lcdc + TCON1_IO_CTRL_REG1);

    suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 0));
    writel(par->mode.xres << 4, iomm.debe + DEBE_LAY0_LINEWIDTH_REG);
    writel(par->mode.xres << 4, iomm.debe + DEBE_LAY1_LINEWIDTH_REG);
    writel(par->mode.xres << 4, iomm.debe + DEBE_LAY2_LINEWIDTH_REG);
    writel(par->mode.xres << 4, iomm.debe + DEBE_LAY3_LINEWIDTH_REG);
    writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_DISP_SIZE_REG);
    writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_LAY0_SIZE_REG);
    writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_LAY1_SIZE_REG);
    writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_LAY2_SIZE_REG);
    writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_LAY3_SIZE_REG);
    writel((5 << 8), iomm.debe + DEBE_LAY0_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY1_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY2_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY3_ATT_CTRL_REG1);
    suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 8));
    suniv_setbits(iomm.debe + DEBE_REGBUFF_CTRL_REG, (1 << 1));
    suniv_setbits(iomm.debe + DEBE_MODE_CTRL_REG, (1 << 1));

    ret = readl(iomm.lcdc + TCON_CTRL_REG);
    ret&= ~(1 << 0);
    writel(ret, iomm.lcdc + TCON_CTRL_REG);
    ret = (1 + 1 + 1);

    writel((uint32_t)(par->vram_phys + 320*240*2*0) << 3, iomm.debe + DEBE_LAY0_FB_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*1) << 3, iomm.debe + DEBE_LAY1_FB_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*2) << 3, iomm.debe + DEBE_LAY2_FB_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*3) << 3, iomm.debe + DEBE_LAY3_FB_ADDR_REG);

    writel((uint32_t)(par->vram_phys + 320*240*2*0) >> 29, iomm.debe + DEBE_LAY0_FB_HI_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*1) >> 29, iomm.debe + DEBE_LAY1_FB_HI_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*2) >> 29, iomm.debe + DEBE_LAY2_FB_HI_ADDR_REG);
    writel((uint32_t)(par->vram_phys + 320*240*2*3) >> 29, iomm.debe + DEBE_LAY3_FB_HI_ADDR_REG);

    writel((1 << 31) | ((ret & 0x1f) << 4) | (1 << 24), iomm.lcdc + TCON0_CTRL_REG);
    writel((0xf << 28) | (25 << 0), iomm.lcdc + TCON_CLK_CTRL_REG);

    writel((4 << 29) | (1 << 26), iomm.lcdc + TCON0_CPU_IF_REG);
    writel((1 << 28), iomm.lcdc + TCON0_IO_CTRL_REG0);

    p1 = par->mode.yres - 1;
    p2 = par->mode.xres - 1;
    writel((p2 << 16) | (p1 << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG0);

    p1 = 1 + 1;
    p2 = 1 + 1 + par->mode.xres + 2;
    writel((p2 << 16) | (p1 << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG1);

    p1 = 1 + 1;
    p2 = (1 + 1 + par->mode.yres + 1 + 2) << 1;
    writel((p2 << 16) | (p1 << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG2);

    p1 = 1 + 1;
    p2 = 1 + 1;
    writel((p2 << 16) | (p1 << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG3);

    writel(0, iomm.lcdc + TCON0_HV_TIMING_REG);
    writel(0, iomm.lcdc + TCON0_IO_CTRL_REG1);

    suniv_setbits(iomm.lcdc + TCON_CTRL_REG, (1 << 31));
    init_lcd();
    suniv_setbits(iomm.lcdc + TCON_INT_REG0, (1 << 31));
    suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 28));
}

static void suniv_enable_irq(struct myfb_par *par)
{
    int ret=0;

    par->lcdc_irq = platform_get_irq(par->pdev, 0);
    if(par->lcdc_irq < 0){
        printk("%s, failed to get irq number for lcdc irq\n", __func__);
    }
    else{
        ret = request_irq(par->lcdc_irq, lcdc_irq_handler, IRQF_SHARED, "lcdc_irq", par);
        if(ret){
            printk("%s, failed to register lcdc interrupt(%d)\n", __func__, par->lcdc_irq);
        }
    }

        par->gpio_irq = gpio_to_irq(((32 * 4) + 10));
        if(par->gpio_irq < 0){
            printk("%s, failed to get irq number for gpio irq\n", __func__);
        }
        else{
            ret = request_irq(par->gpio_irq, gpio_irq_handler, IRQF_TRIGGER_RISING, "gpio_irq", par);
            if(ret){
                printk("%s, failed to register gpio interrupt(%d)\n", __func__, par->gpio_irq);
            }
        }
}

static void suniv_cpu_init(struct myfb_par *par)
{
    uint32_t ret, i;

    while((readl(iomm.ccm + PLL_VIDEO_CTRL_REG) & (1 << 28)) == 0){
    }
    while((readl(iomm.ccm + PLL_PERIPH_CTRL_REG) & (1 << 28)) == 0){
    }

    ret = readl(iomm.ccm + DRAM_GATING_REG);
    ret|= (1 << 26) | (1 << 24);
    writel(ret, iomm.ccm + DRAM_GATING_REG);

    suniv_setbits(iomm.ccm + FE_CLK_REG, (1 << 31));
    suniv_setbits(iomm.ccm + BE_CLK_REG, (1 << 31));
    suniv_setbits(iomm.ccm + TCON_CLK_REG, (1 << 31) | (1 << 25));
    suniv_setbits(iomm.ccm + BUS_CLK_GATING_REG1, (1 << 14) | (1 << 12) | (1 << 4));
    suniv_setbits(iomm.ccm + BUS_SOFT_RST_REG1, (1 << 14) | (1 << 12) | (1 << 4));
    for(i=0x0800; i<0x1000; i+=4){
        writel(0, iomm.debe + i);
    }
}

static void lcd_delay_init(unsigned long param)
{
    suniv_cpu_init(mypar);
    suniv_lcdc_init(mypar);
    mypar->app_virt->yoffset = 240;
    mypar->lcdc_ready = 1;
    suniv_enable_irq(mypar);
}

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)
static int myfb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info)
{
    red = CNVT_TOHW(red, info->var.red.length);
    blue = CNVT_TOHW(blue, info->var.blue.length);
    green = CNVT_TOHW(green, info->var.green.length);
    ((u32*)(info->pseudo_palette))[regno] = (red << info->var.red.offset) | (green << info->var.green.offset) | (blue << info->var.blue.offset);
    return 0;
}
#undef CNVT_TOHW

static int myfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    int bpp = var->bits_per_pixel >> 3;
    struct myfb_par *par = info->par;
    unsigned long line_size = var->xres_virtual * bpp;

    if((var->xres != 320) || (var->yres != 240) || (var->bits_per_pixel != 16)){
        return -EINVAL;
    }

    var->transp.offset = 0;
    var->transp.length = 0;
    var->red.offset = 11;
    var->red.length = 5;
    var->green.offset = 5;
    var->green.length = 6;
    var->blue.offset = 0;
    var->blue.length = 5;
    var->red.msb_right = 0;
    var->green.msb_right = 0;
    var->blue.msb_right = 0;
    var->transp.msb_right = 0;
    if(line_size * var->yres_virtual > par->vram_size){
        var->yres_virtual = par->vram_size / line_size;
    }
    if(var->yres > var->yres_virtual){
        var->yres = var->yres_virtual;
    }
    if(var->xres > var->xres_virtual){
        var->xres = var->xres_virtual;
    }
    if(var->xres + var->xoffset > var->xres_virtual){
        var->xoffset = var->xres_virtual - var->xres;
    }
    if(var->yres + var->yoffset > var->yres_virtual){
        var->yoffset = var->yres_virtual - var->yres;
    }
    return 0;
}

static int myfb_set_par(struct fb_info *info)
{
    struct myfb_par *par = info->par;

    fb_var_to_videomode(&par->mode, &info->var);
    par->app_virt->yoffset = info->var.yoffset = 0;
    par->bpp = info->var.bits_per_pixel;
    info->fix.visual = FB_VISUAL_TRUECOLOR;
    info->fix.line_length = (par->mode.xres * par->bpp) / 8;
    writel((5 << 8), iomm.debe + DEBE_LAY0_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY1_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY2_ATT_CTRL_REG1);
    writel((5 << 8), iomm.debe + DEBE_LAY3_ATT_CTRL_REG1);
    return 0;
}

static int wait_for_vsync(struct myfb_par *par)
{
    uint32_t count = par->app_virt->vsync_count;
    long t = wait_event_interruptible_timeout(wait_vsync_queue, count != par->app_virt->vsync_count, HZ / 10);
    return t > 0 ? 0 : (t < 0 ? (int)t : -ETIMEDOUT);
}

static int myfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct myfb_par *par = info->par;
    switch(cmd){
        case FBIO_WAITFORVSYNC:
            wait_for_vsync(par);
            break;
    }
    return 0;
}

static int myfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
    const unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
    const unsigned long size = vma->vm_end - vma->vm_start;

    if(offset + size > info->fix.smem_len){
        return -EINVAL;
    }

    if(remap_pfn_range(vma, vma->vm_start, (info->fix.smem_start + offset) >> PAGE_SHIFT, size, vma->vm_page_prot)){
        return -EAGAIN;
    }
    return 0;
}

static int myfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct myfb_par *par = info->par;

    info->var.xoffset = var->xoffset;
    info->var.yoffset = var->yoffset;
    par->app_virt->yoffset = var->yoffset;
    return 0;
}

static struct fb_ops myfb_ops = {
        .owner          = THIS_MODULE,
        .fb_check_var   = myfb_check_var,
        .fb_set_par     = myfb_set_par,
        .fb_setcolreg   = myfb_setcolreg,
        .fb_pan_display = myfb_pan_display,
        .fb_ioctl       = myfb_ioctl,
        .fb_mmap        = myfb_mmap,

        .fb_fillrect  = sys_fillrect,
        .fb_copyarea  = sys_copyarea,
        .fb_imageblit = sys_imageblit,
};

static int myfb_probe(struct platform_device *device)
{
    int ret=0;
    struct fb_info *info=NULL;
    struct myfb_par *par=NULL;
    struct fb_videomode *mode=NULL;

    mode = devm_kzalloc(&device->dev, sizeof(struct fb_videomode), GFP_KERNEL);
    if(mode == NULL){
        return -ENOMEM;
    }
    mode->name = "320x240";
    mode->xres = 320;
    mode->yres = 240;
    mode->vmode = FB_VMODE_NONINTERLACED;
    pm_runtime_enable(&device->dev);
    pm_runtime_get_sync(&device->dev);

    info = framebuffer_alloc(sizeof(struct myfb_par), &device->dev);
    if(!info){
        return -ENOMEM;
    }

    par = info->par;
    par->pdev = device;
    par->dev = &device->dev;
    par->bpp = 16;
    fb_videomode_to_var(&myfb_var, mode);

    par->vram_size = (320 * 240 * 2 * 4) + 4096;
    par->vram_virt = dma_alloc_coherent(NULL, par->vram_size, (resource_size_t*)&par->vram_phys, GFP_KERNEL | GFP_DMA);
    if(!par->vram_virt){
        return -EINVAL;
    }
    info->screen_base = (char __iomem*)par->vram_virt;
    myfb_fix.smem_start = par->vram_phys;
    myfb_fix.smem_len = par->vram_size;
    myfb_fix.line_length = 320 * 2;
    par->app_virt = (struct myfb_app*)((uint8_t*)par->vram_virt + (320 * 240 * 2 * 4));

    par->v_palette_base = dma_alloc_coherent(NULL, PALETTE_SIZE, (resource_size_t*)&par->p_palette_base, GFP_KERNEL | GFP_DMA);
    if(!par->v_palette_base){
        return -EINVAL;
    }
    memset(par->v_palette_base, 0, PALETTE_SIZE);
    myfb_var.grayscale = 0;
    myfb_var.bits_per_pixel = par->bpp;

    info->flags = FBINFO_FLAG_DEFAULT;
    info->fix = myfb_fix;
    info->var = myfb_var;
    info->fbops = &myfb_ops;
    info->pseudo_palette = par->pseudo_palette;
    info->fix.visual = (info->var.bits_per_pixel <= 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;
    ret = fb_alloc_cmap(&info->cmap, PALETTE_SIZE, 0);
    if(ret){
        return -EINVAL;
    }
    info->cmap.len = 32;

    myfb_var.activate = FB_ACTIVATE_FORCE;
    fb_set_var(info, &myfb_var);
    dev_set_drvdata(&device->dev, info);
    if(register_framebuffer(info) < 0){
        return -EINVAL;
    }

    mypar = par;
    mypar->lcdc_ready = 0;
    mypar->app_virt->vsync_count = 0;
    for(ret=0; ret<of_clk_get_parent_count(device->dev.of_node); ret++){
        clk_prepare_enable(of_clk_get(device->dev.of_node, ret));
    }

    setup_timer(&mytimer, lcd_delay_init, 0);
    mod_timer(&mytimer, jiffies + HZ);
    return 0;
}

static int myfb_remove(struct platform_device *dev)
{
    struct fb_info *info = dev_get_drvdata(&dev->dev);
    struct myfb_par *par = info->par;

    if(info){
        del_timer(&mytimer);
        flush_scheduled_work();
        unregister_framebuffer(info);
        fb_dealloc_cmap(&info->cmap);
        dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base, par->p_palette_base);
        dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);
        pm_runtime_put_sync(&dev->dev);
        pm_runtime_disable(&dev->dev);
        framebuffer_release(info);
    }
    return 0;
}

static int myfb_suspend(struct platform_device *dev, pm_message_t state)
{
    struct fb_info *info = platform_get_drvdata(dev);

    console_lock();
    fb_set_suspend(info, 1);
    pm_runtime_put_sync(&dev->dev);
    console_unlock();
    return 0;
}

static int myfb_resume(struct platform_device *dev)
{
    struct fb_info *info = platform_get_drvdata(dev);

    console_lock();
    pm_runtime_get_sync(&dev->dev);
    fb_set_suspend(info, 0);
    console_unlock();
    return 0;
}

static const struct of_device_id fb_of_match[] = {
        {
                .compatible = "allwinner,suniv-f1c500s-tcon0",
        },{}
};
MODULE_DEVICE_TABLE(of, fb_of_match);

static struct platform_driver fb_driver = {
        .probe    = myfb_probe,
        .remove   = myfb_remove,
        .suspend  = myfb_suspend,
        .resume   = myfb_resume,
        .driver = {
                .name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(fb_of_match),
        },
};

static void suniv_ioremap(void)
{
    iomm.ccm = (uint8_t*)ioremap(SUNIV_CCM_BASE, 1024);
    iomm.gpio = (uint8_t*)ioremap(SUNIV_GPIO_BASE, 1024);
    iomm.lcdc = (uint8_t*)ioremap(SUNIV_LCDC_BASE, 1024);
    iomm.debe = (uint8_t*)ioremap(SUNIV_DEBE_BASE, 4096);
}

static void suniv_iounmap(void)
{
    iounmap(iomm.ccm);
    iounmap(iomm.gpio);
    iounmap(iomm.lcdc);
    iounmap(iomm.debe);
    iounmap(iomm.intc);
    iounmap(iomm.timer);
}

static int myopen(struct inode *inode, struct file *file)
{
    return 0;
}

static int myclose(struct inode *inode, struct file *file)
{
    return 0;
}
static long myioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static const struct file_operations myfops = {
        .owner = THIS_MODULE,
        .open = myopen,
        .release = myclose,
        .unlocked_ioctl = myioctl,
};

static int __init fb_init(void)
{
    suniv_ioremap();
    alloc_chrdev_region(&major, 0, 1, "miyoo_fb0");
    myclass = class_create(THIS_MODULE, "miyoo_fb0");
    device_create(myclass, NULL, major, NULL, "miyoo_fb0");
    cdev_init(&mycdev, &myfops);
    cdev_add(&mycdev, major, 1);
    return platform_driver_register(&fb_driver);
}

static void __exit fb_cleanup(void)
{
    suniv_iounmap();
    platform_driver_unregister(&fb_driver);
}

module_init(fb_init);
module_exit(fb_cleanup);

MODULE_DESCRIPTION("Framebuffer driver for ST7789S");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_LICENSE("GPL");

