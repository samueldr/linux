// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 Steward Fu <steward.fu@gmail.com>
 * Copyright (C) 2021 Samuel Dionne-Riel <samuel@dionne-riel.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>

#include <asm/io.h>

struct miyoo_battery_info {
	struct device *dev;
	struct power_supply *bat;
	struct power_supply_desc bat_desc;
};
uint8_t *lradc;

static int miyoo_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = (readl(lradc + 0x0c) * 3 * 3 * 10);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static enum power_supply_property miyoo_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int miyoo_battery_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct device *dev = &pdev->dev;

	struct miyoo_battery_info *batttery_info = NULL;
	struct power_supply_config psy_cfg = {0};
	uint32_t reg[2];
	uint32_t ret;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

	batttery_info = devm_kzalloc(dev, sizeof(*batttery_info), GFP_KERNEL);
	if (!batttery_info) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, batttery_info);

	ret = of_property_read_u32_array(node, "reg", reg, 2);
	if (ret) {
		dev_err(dev, "invalid 'reg' property: %d\n", ret);
		return -EINVAL;
	}

	lradc = (uint8_t*)ioremap(reg[0], reg[1]);

	ret = readl(lradc);
	ret|= 0x1;
	writel(ret, lradc);

	batttery_info->dev = dev;
	batttery_info->bat_desc.name = "miyoo-battery";
	batttery_info->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	batttery_info->bat_desc.properties = miyoo_battery_props;
	batttery_info->bat_desc.num_properties = ARRAY_SIZE(miyoo_battery_props);
	batttery_info->bat_desc.get_property = miyoo_battery_get_property;
	batttery_info->bat = power_supply_register(batttery_info->dev, &batttery_info->bat_desc, &psy_cfg);

	psy_cfg.drv_data = batttery_info;

	return 0;
}

static int miyoo_battery_remove(struct platform_device *pdev)
{
	struct miyoo_battery_info *batttery_info = platform_get_drvdata(pdev);

	power_supply_unregister(batttery_info->bat);
	iounmap(lradc);

	return 0;
}

static const struct of_device_id miyoo_battery_of_match[] = {
	{.compatible = "miyoo,battery", },{},
};
MODULE_DEVICE_TABLE(of, miyoo_battery_of_match);

static struct platform_driver miyoo_battery_driver = {
	.probe = miyoo_battery_probe,
	.remove = miyoo_battery_remove,
	.driver = {
		.name = "miyoo-battery",
		.of_match_table = of_match_ptr(miyoo_battery_of_match),
	},
};
module_platform_driver(miyoo_battery_driver);

// Original code
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
// Simplified mainline port
MODULE_AUTHOR("Samuel Dionne-Riel <samuel@dionne-riel.com>");

MODULE_DESCRIPTION("Miyoo battery driver");
MODULE_LICENSE("GPL");
