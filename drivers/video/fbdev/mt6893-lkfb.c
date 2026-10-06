// SPDX-License-Identifier: GPL-2.0
/*
 * BRING-UP: an fbdev on the framebuffer the LK leaves on the OnePlus Nord 2
 * (MT6893), until the mediatek DRM driver handles this SoC.
 *
 * The LK leaves the display pipeline powered and configured: OVL0_2L and
 * OVL1_2L read the two halves of one linear 1080x2400 buffer at 0x78810000
 * (pitch 4352, ARGB8888), mutex 0 holds every module down to DSI0, and the
 * panel runs in DSI command mode. A frame goes out on two writes: MUTEX0_EN,
 * then DSI_START 0 -> 1.
 *
 * The buffer is linear-mapped RAM (the LK reserves it without no-map), hence
 * memremap(WB) and a cache clean before each frame: neither simpledrm
 * (memremap WC) nor ioremap_wc accept that region.
 */

#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>

#define FB_BASE		0x78810000
#define FB_WIDTH	1080
#define FB_HEIGHT	2400
#define FB_STRIDE	4352
#define FB_SIZE		(FB_STRIDE * FB_HEIGHT)

#define DSI_BASE	0x1400e000
#define DSI_START	0x00
#define DSI_INTSTA	0x0c
#define DSI_BUSY	BIT(31)

/* OVL0 layer 1 carries the LK warning text, over the buffer */
#define OVL0_BASE	0x14000000
#define OVL_SRC_CON	0x2c

#define MUTEX_BASE	0x14117000
#define MUTEX0_EN	0x20

static unsigned int period_ms = 50;
module_param(period_ms, uint, 0644);
MODULE_PARM_DESC(period_ms, "frame period in ms, 0 for none");

static bool pattern;
module_param(pattern, bool, 0444);
MODULE_PARM_DESC(pattern, "draw colour bars at load");

static bool hide_lk_text = true;
module_param(hide_lk_text, bool, 0444);
MODULE_PARM_DESC(hide_lk_text, "hide the LK text layer (OVL0)");

static bool register_fb = true;
module_param(register_fb, bool, 0444);
MODULE_PARM_DESC(register_fb, "register the fbdev (and so fbcon)");

static unsigned long frames, skipped;
module_param(frames, ulong, 0444);
module_param(skipped, ulong, 0444);

static void __iomem *dsi, *mutex, *ovl0;
static void *screen;
static struct fb_info *info;
static struct delayed_work kick_work;
static u32 pseudo_palette[16];
static bool dirty, mmapped;

static void lkfb_clean(void)
{
	unsigned long line = cache_line_size();
	unsigned long p = (unsigned long)screen & ~(line - 1);
	unsigned long end = (unsigned long)screen + FB_SIZE;

	for (; p < end; p += line)
		asm volatile("dc cvac, %0" : : "r" (p) : "memory");
	dsb(sy);
}

static void lkfb_kick(void)
{
	if ((readl(dsi + DSI_INTSTA) & DSI_BUSY) || readl(mutex + MUTEX0_EN)) {
		skipped++;
		return;
	}
	lkfb_clean();
	writel(0, dsi + DSI_INTSTA);
	writel(1, mutex + MUTEX0_EN);
	writel(0, dsi + DSI_START);
	writel(1, dsi + DSI_START);
	frames++;
}

static void lkfb_work(struct work_struct *work)
{
	if (xchg(&dirty, false) || mmapped)
		lkfb_kick();
	if (period_ms)
		schedule_delayed_work(&kick_work, msecs_to_jiffies(period_ms));
}

static void lkfb_pattern(void)
{
	static const u32 bands[] = {
		0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffffff,
		0xff000000, 0xffffff00, 0xff00ffff, 0xffff00ff,
	};
	int x, y;

	for (y = 0; y < FB_HEIGHT; y++) {
		u32 *row = screen + y * FB_STRIDE;
		u32 c = bands[y * ARRAY_SIZE(bands) / FB_HEIGHT];

		for (x = 0; x < FB_WIDTH; x++)
			row[x] = (x < 40 || x >= FB_WIDTH - 40) ? 0xffffffff : c;
	}
}

static ssize_t lkfb_write(struct fb_info *i, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	ssize_t ret = fb_sys_write(i, buf, count, ppos);

	dirty = true;
	return ret;
}

static void lkfb_fillrect(struct fb_info *i, const struct fb_fillrect *r)
{
	sys_fillrect(i, r);
	dirty = true;
}

static void lkfb_copyarea(struct fb_info *i, const struct fb_copyarea *a)
{
	sys_copyarea(i, a);
	dirty = true;
}

static void lkfb_imageblit(struct fb_info *i, const struct fb_image *im)
{
	sys_imageblit(i, im);
	dirty = true;
}

static int lkfb_mmap(struct fb_info *i, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off >= FB_SIZE || size > FB_SIZE - off)
		return -EINVAL;
	/* cacheable, like the linear map: no mismatched alias */
	mmapped = true;
	return remap_pfn_range(vma, vma->vm_start, (FB_BASE + off) >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static int lkfb_setcolreg(unsigned int regno, unsigned int red,
			  unsigned int green, unsigned int blue,
			  unsigned int transp, struct fb_info *i)
{
	if (regno >= 16)
		return -EINVAL;
	pseudo_palette[regno] = 0xff000000 | ((red >> 8) << 16) |
				((green >> 8) << 8) | (blue >> 8);
	return 0;
}

static const struct fb_ops lkfb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= lkfb_write,
	.fb_fillrect	= lkfb_fillrect,
	.fb_copyarea	= lkfb_copyarea,
	.fb_imageblit	= lkfb_imageblit,
	.fb_mmap	= lkfb_mmap,
	.fb_setcolreg	= lkfb_setcolreg,
};

static int lkfb_register(void)
{
	int ret;

	info = framebuffer_alloc(0, NULL);
	if (!info)
		return -ENOMEM;

	strscpy(info->fix.id, "lkfb", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.smem_start = FB_BASE;
	info->fix.smem_len = FB_SIZE;
	info->fix.line_length = FB_STRIDE;

	info->var.xres = info->var.xres_virtual = FB_WIDTH;
	info->var.yres = info->var.yres_virtual = FB_HEIGHT;
	info->var.bits_per_pixel = 32;
	info->var.red = (struct fb_bitfield){ 16, 8, 0 };
	info->var.green = (struct fb_bitfield){ 8, 8, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 8, 0 };
	info->var.transp = (struct fb_bitfield){ 24, 8, 0 };
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.width = 68;	/* mm, 6.43" at 20:9 */
	info->var.height = 151;

	info->fbops = &lkfb_ops;
	info->screen_buffer = screen;
	info->screen_size = FB_SIZE;
	info->pseudo_palette = pseudo_palette;
	info->flags = FBINFO_VIRTFB;

	ret = register_framebuffer(info);
	if (ret) {
		framebuffer_release(info);
		info = NULL;
	}
	return ret;
}

static int __init lkfb_init(void)
{
	int ret = -ENOMEM;

	dsi = ioremap(DSI_BASE, 0x1000);
	mutex = ioremap(MUTEX_BASE, 0x1000);
	ovl0 = ioremap(OVL0_BASE, 0x1000);
	screen = memremap(FB_BASE, FB_SIZE, MEMREMAP_WB);
	if (!dsi || !mutex || !ovl0 || !screen)
		goto err;

	if (hide_lk_text) {
		writel(0, ovl0 + OVL_SRC_CON);
		dirty = true;
	}

	if (pattern) {
		lkfb_pattern();
		dirty = true;
	}

	if (register_fb) {
		ret = lkfb_register();
		if (ret)
			goto err;
	}

	INIT_DELAYED_WORK(&kick_work, lkfb_work);
	schedule_delayed_work(&kick_work, 0);
	pr_info("lkfb: %dx%d at %#x, one frame every %u ms\n",
		FB_WIDTH, FB_HEIGHT, FB_BASE, period_ms);
	return 0;

err:
	if (screen)
		memunmap(screen);
	if (ovl0)
		iounmap(ovl0);
	if (mutex)
		iounmap(mutex);
	if (dsi)
		iounmap(dsi);
	return ret;
}

static void __exit lkfb_exit(void)
{
	period_ms = 0;
	cancel_delayed_work_sync(&kick_work);
	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
	}
	memunmap(screen);
	iounmap(ovl0);
	iounmap(mutex);
	iounmap(dsi);
}

module_init(lkfb_init);
module_exit(lkfb_exit);
MODULE_DESCRIPTION("fbdev on the LK framebuffer, MT6893 in DSI command mode");
MODULE_LICENSE("GPL");
