// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Apple M1 WDT
 *
 * Copyright (C) 2021 Pip Cet <pipcet@gmail.com>
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/slab.h>

#define WDT_FREQ	(24 * 1000 * 1000)
#define WDT_COUNT	0x10
#define WDT_COMPARATOR	0x14
#define WDT_CONTROL	0x1c
#define WDT_CONTROL_TRIGGER	0x04

struct apple_wdt {
	void __iomem *base;
	struct clk *clk;
};

static int apple_wdt_start(struct watchdog_device *w)
{
	struct apple_wdt *wdt = watchdog_get_drvdata(w);
	writel(0, wdt->base + WDT_COUNT);
	writel(U32_MAX, wdt->base + WDT_COMPARATOR);
	writel(WDT_CONTROL_TRIGGER, wdt->base + WDT_CONTROL);
	return 0;
}

static int apple_wdt_stop(struct watchdog_device *w)
{
	struct apple_wdt *wdt = watchdog_get_drvdata(w);
	writel(0, wdt->base + WDT_COUNT);
	writel(U32_MAX, wdt->base + WDT_COMPARATOR);
	writel(0, wdt->base + WDT_CONTROL);
	return 0;
}

static int apple_wdt_ping(struct watchdog_device *w)
{
	struct apple_wdt *wdt = watchdog_get_drvdata(w);
	writel(0, wdt->base + WDT_COUNT);
	return 0;
}

static int apple_wdt_set_timeout(struct watchdog_device *w, unsigned int s)
{
	struct apple_wdt *wdt = watchdog_get_drvdata(w);
	writel(s * WDT_FREQ, wdt->base + WDT_COMPARATOR);
	return 0;
}

static unsigned int apple_wdt_get_timeleft(struct watchdog_device *w)
{
	struct apple_wdt *wdt = watchdog_get_drvdata(w);
	u32 comparator = readl(wdt->base + WDT_COMPARATOR);
	u32 count = readl(wdt->base + WDT_COUNT);
	return (comparator - count) / WDT_FREQ;
}

static struct watchdog_ops apple_wdt_ops = {
	.start = apple_wdt_start,
	.stop = apple_wdt_stop,
	.ping = apple_wdt_ping,
	.set_timeout = apple_wdt_set_timeout,
	.get_timeleft = apple_wdt_get_timeleft,
};

static struct watchdog_info apple_wdt_info = {
	.identity = "Apple WDT",
};

static int apple_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_wdt *wdt = devm_kzalloc(dev, sizeof *wdt, GFP_KERNEL);
	struct watchdog_device *wd = devm_kzalloc(dev, sizeof *wd, GFP_KERNEL);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	void __iomem *base;
	int ret;
	if (!wdt || !wd)
		return -ENOMEM;
	wd->ops = &apple_wdt_ops;
	if (IS_ERR(res))
		return PTR_ERR(res);
	wd->info = &apple_wdt_info;
	wdt->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(wdt->clk))
		wdt->clk = NULL;
	//return PTR_ERR(wdt->clk);
	if (wdt->clk) {
		ret = clk_prepare_enable(wdt->clk);
		if (ret)
			return ret;
	}
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	wdt->base = base;
	if (readl(wdt->base + WDT_CONTROL) & WDT_CONTROL_TRIGGER)
		wd->status |= WDOG_HW_RUNNING;
	ret = devm_watchdog_register_device(dev, wd);
	if (ret < 0)
		return ret;
	watchdog_set_drvdata(wd, wdt);

	printk("successfully registered watchdog\n");
	return 0;
}

static int apple_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wd = platform_get_drvdata(pdev);
	struct apple_wdt *wdt = watchdog_get_drvdata(wd);

	watchdog_unregister_device(wd);
	if (wdt->clk)
		clk_disable_unprepare(wdt->clk);

	return 0;
}

static int apple_wdt_restart(struct watchdog_device *wd, unsigned long mode,
			     void *cmd)
{
	apple_wdt_set_timeout(wd, 0);
	apple_wdt_start(wd);

	return 0;
}

static const struct of_device_id apple_wdt_of_match[] = {
	{ .compatible = "apple,wdt" },
	{ },
};

MODULE_DEVICE_TABLE(of, apple_wdt_of_match);

static struct platform_driver apple_wdt_driver = {
	.driver = {
		.name = "apple-wdt",
		.of_match_table = of_match_ptr(apple_wdt_of_match),
	},
	.probe = apple_wdt_probe,
	.remove = apple_wdt_remove,
};
module_platform_driver(apple_wdt_driver);

MODULE_AUTHOR("Pip Cet <pipcet@gmail.com>");
MODULE_DESCRIPTION("Watchdog Timer driver for Apple M1");
MODULE_ALIAS("platform:apple-m1-wdt");
MODULE_LICENSE("GPL");
