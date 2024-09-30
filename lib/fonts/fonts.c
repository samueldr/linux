/*
 * `Soft' font definitions
 *
 *    Created 1995 by Geert Uytterhoeven
 *    Rewritten 1998 by Martin Mares <mj@ucw.cz>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#if defined(__mc68000__)
#include <asm/setup.h>
#endif
#include <linux/font.h>

static const struct font_desc *fonts[] = {
#ifdef CONFIG_FONT_8x8
	&font_vga_8x8,
#endif
#ifdef CONFIG_FONT_8x16
	&font_vga_8x16,
#endif
#ifdef CONFIG_FONT_6x11
	&font_vga_6x11,
#endif
#ifdef CONFIG_FONT_7x14
	&font_7x14,
#endif
#ifdef CONFIG_FONT_SUN8x16
	&font_sun_8x16,
#endif
#ifdef CONFIG_FONT_SUN12x22
	&font_sun_12x22,
#endif
#ifdef CONFIG_FONT_10x18
	&font_10x18,
#endif
#ifdef CONFIG_FONT_ACORN_8x8
	&font_acorn_8x8,
#endif
#ifdef CONFIG_FONT_PEARL_8x8
	&font_pearl_8x8,
#endif
#ifdef CONFIG_FONT_MINI_4x6
	&font_mini_4x6,
#endif
#ifdef CONFIG_FONT_6x10
	&font_6x10,
#endif
#ifdef CONFIG_FONT_TER16x32
	&font_ter_16x32,
#endif
#ifdef CONFIG_FONT_6x8
	&font_6x8,
#endif
};

#define num_fonts ARRAY_SIZE(fonts)

#ifdef NO_FONTS
#error No fonts configured.
#endif


/**
 *	find_font - find a font
 *	@name: string name of a font
 *
 *	Find a specified font with string name @name.
 *
 *	Returns %NULL if no font found, or a pointer to the
 *	specified font.
 *
 */
const struct font_desc *find_font(const char *name)
{
	unsigned int i;

	BUILD_BUG_ON(!num_fonts);
	for (i = 0; i < num_fonts; i++)
		if (!strcmp(fonts[i]->name, name))
			return fonts[i];
	return NULL;
}
EXPORT_SYMBOL(find_font);


/**
 *	get_default_font - get default font
 *	@xres: screen size of X
 *	@yres: screen size of Y
 *	@font_w: bit array of supported widths (1 - FB_MAX_BLIT_WIDTH)
 *	@font_h: bit array of supported heights (1 - FB_MAX_BLIT_HEIGHT)
 *
 *	Get the default font for a specified screen size.
 *	Dimensions are in pixels.
 *
 *	font_w or font_h being NULL means all values are supported.
 *
 *	Returns %NULL if no font is found, or a pointer to the
 *	chosen font.
 *
 */
const struct font_desc *get_default_font(int xres, int yres,
					 unsigned long *font_w,
					 unsigned long *font_h)
{
	int i, candidate_score, selected_score, resolution;
	const struct font_desc *candidate_font, *selected_font;

	selected_font = NULL;
	selected_score = -10000;
	for (i = 0; i < num_fonts; i++) {
		candidate_font = fonts[i];
		candidate_score = candidate_font->pref;
#if defined(__mc68000__)
#ifdef CONFIG_FONT_PEARL_8x8
		if (MACH_IS_AMIGA && candidate_font->idx == PEARL8x8_IDX)
			candidate_score = 100;
#endif
#ifdef CONFIG_FONT_6x11
		if (MACH_IS_MAC && xres < 640 && candidate_font->idx == VGA6x11_IDX)
			candidate_score = 100;
#endif
#endif
		if ((yres < 400) == (candidate_font->height <= 8))
			candidate_score += 1000;

		/* prefer a bigger font for high resolution */
		resolution = (xres / candidate_font->width) * (yres / candidate_font->height) / 1000;
		if (resolution > 20)
			candidate_score += 20 - resolution;

		if ((!font_w || test_bit(candidate_font->width - 1, font_w)) &&
		    (!font_h || test_bit(candidate_font->height - 1, font_h)))
			candidate_score += 1000;

		if (candidate_score > selected_score) {
			selected_score = candidate_score;
			selected_font = candidate_font;
		}
	}
	return selected_font;
}
EXPORT_SYMBOL(get_default_font);

MODULE_AUTHOR("James Simmons <jsimmons@users.sf.net>");
MODULE_DESCRIPTION("Console Fonts");
MODULE_LICENSE("GPL");
