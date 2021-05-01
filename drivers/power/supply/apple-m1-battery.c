// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Pip Cet <pipcet@gmail.com>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static const enum power_supply_property properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
};

extern int apple_m1_smc_read_percentage(struct device *dev);

struct apple_battery {
	struct platform_device *pdev;
	struct power_supply *psy;
};

static int apple_battery_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct apple_battery *batt = power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = apple_m1_smc_read_percentage(&batt->pdev->dev);
		return 0;
	default:
		return -EINVAL;
	}
}

static int apple_battery_property_is_writable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	return 0;
}

static const struct power_supply_desc desc = {
	.name = "Apple M1 battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = properties,
	.num_properties = ARRAY_SIZE(properties),
	.get_property = apple_battery_get_property,
	.property_is_writeable = apple_battery_property_is_writable,
	.use_for_apm = 1,
};

static int apple_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config *cfg =
	       devm_kzalloc(dev, sizeof *cfg, GFP_KERNEL);
	struct apple_battery *batt =
	       devm_kzalloc(dev, sizeof *batt, GFP_KERNEL);

	if (cfg == NULL)
		return -ENOMEM;

	if (batt == NULL)
		return -ENOMEM;

	batt->pdev = pdev;
	batt->psy = power_supply_register(dev, &desc, cfg);

	if (IS_ERR(batt->psy))
		return PTR_ERR(batt->psy);

	return 0;
}

static int apple_battery_remove(struct platform_device *pdev)
{
	struct apple_battery *batt = platform_get_drvdata(pdev);
	power_supply_unregister(batt->psy);

	return 0;
}

static const struct of_device_id apple_battery_of_match[] = {
	{ .compatible = "apple,battery" },
	{ },
};

MODULE_DEVICE_TABLE(of, apple_battery_of_match);

static struct platform_driver apple_battery_driver = {
	.driver = {
		.name = "apple-m1-battery",
		.of_match_table = of_match_ptr(apple_battery_of_match),
	},
	.probe = apple_battery_probe,
	.remove = apple_battery_remove,
};
module_platform_driver(apple_battery_driver);

MODULE_AUTHOR("Pip Cet <pipcet@gmail.com>");
MODULE_DESCRIPTION("Battery status driver for Apple M1");
MODULE_ALIAS("platform:apple-m1-battery");
MODULE_LICENSE("GPL");
