/*
 * Apple SoC pinctrl+GPIO+external IRQ driver
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: pinctrl-pistachio.c
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2014 Google, Inc.
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "pinctrl-utils.h"
#include "core.h"
#include "devicetree.h"
#include "pinconf.h"
#include "pinmux.h"

struct apple_gpio_pincfg {
	uint8_t irqtype;
	uint8_t stat;
};

#define PINCFG_STAT_OUTVAL	0x01
#define PINCFG_STAT_OUTEN	0x02
#define PINCFG_STAT_PERIPH	0x20
#define PINCFG_STAT_IRQEN	0x80

struct apple_gpio_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctldev;

	unsigned int pin_base;
	const char *pin_prefix;
	unsigned int npins;
	struct pinctrl_pin_desc *pins;
	struct apple_gpio_pincfg *pin_cfgs;
	const char **pin_names;
	unsigned *pin_nums;

	void __iomem *base;
	unsigned int nirqgrps;
	int *irqs;

	struct pinctrl_desc pinctrl_desc;
	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
};

#define REG_GPIO(x)		(4 * (x))
#define  REG_GPIOx_DATA		(1 << 0)
#define  REG_GPIOx_IRQ_MASK	(7 << 1)
#define	REG_GPIOx_IRQ_OUT	(1 << 1)
#define	REG_GPIOx_IRQ_HI	(2 << 1)
#define	REG_GPIOx_IRQ_LO	(3 << 1)
#define	REG_GPIOx_IRQ_UP	(4 << 1)
#define	REG_GPIOx_IRQ_DN	(5 << 1)
#define	REG_GPIOx_IRQ_ANY	(6 << 1)
#define	REG_GPIOx_IRQ_OFF	(7 << 1)
#define  REG_GPIOx_PERIPH	(1 << 5)
#define  REG_GPIOx_CFG_DONE	(1 << 9)
#define  REG_GPIOx_GRP_MASK	(7 << 16)
#define	REG_GPIOx_GRP_SHIFT	16
#define REG_IRQ(g,x)		(0x800 + 0x40 * (g) + 4 * ((x) >> 5))
#define REG_LOCK		0xC50

static void apple_gpio_set_reg(struct apple_gpio_pinctrl *pctl, unsigned pin, uint32_t clr, uint32_t set)
{
	void __iomem *ppin = pctl->base + pin * 4;
	uint32_t prev, cfg;

	prev = readl(ppin);
	cfg = (prev & ~clr) | set;

	if(cfg & REG_GPIOx_CFG_DONE) {
		if(!(prev & REG_GPIOx_CFG_DONE))
			writel(cfg & ~REG_GPIOx_CFG_DONE, ppin);
	} else
		writel(prev & ~REG_GPIOx_CFG_DONE, ppin);
	writel(cfg, ppin);
}

static void apple_gpio_refresh_reg(struct apple_gpio_pinctrl *pctl, unsigned pin)
{
	struct apple_gpio_pincfg *pincfg = &pctl->pin_cfgs[pin];

	if(pincfg->stat & PINCFG_STAT_PERIPH) {
		apple_gpio_set_reg(pctl, pin, REG_GPIOx_IRQ_MASK | REG_GPIOx_DATA, REG_GPIOx_PERIPH | REG_GPIOx_CFG_DONE | (pincfg->stat & PINCFG_STAT_OUTVAL));
		return;
	}

	if(pincfg->stat & PINCFG_STAT_OUTEN) {
		apple_gpio_set_reg(pctl, pin, REG_GPIOx_IRQ_MASK | REG_GPIOx_DATA | REG_GPIOx_PERIPH, REG_GPIOx_CFG_DONE | REG_GPIOx_IRQ_OUT | (pincfg->stat & PINCFG_STAT_OUTVAL));
		return;
	}

	if(pincfg->stat & PINCFG_STAT_IRQEN) {
		apple_gpio_set_reg(pctl, pin, REG_GPIOx_IRQ_MASK | REG_GPIOx_DATA | REG_GPIOx_PERIPH, REG_GPIOx_CFG_DONE | pincfg->irqtype | (pincfg->stat & PINCFG_STAT_OUTVAL));
		return;
	}

	apple_gpio_set_reg(pctl, pin, REG_GPIOx_IRQ_MASK | REG_GPIOx_DATA | REG_GPIOx_PERIPH, REG_GPIOx_CFG_DONE | REG_GPIOx_IRQ_OFF | (pincfg->stat & PINCFG_STAT_OUTVAL));
}

static uint32_t apple_gpio_get_reg(struct apple_gpio_pinctrl *pctl, unsigned pin)
{
	return readl(pctl->base + pin * 4);
}

static void apple_gpio_init_reg(struct apple_gpio_pinctrl *pctl, unsigned pin)
{
	struct apple_gpio_pincfg *pincfg = &pctl->pin_cfgs[pin];
	uint32_t reg = apple_gpio_get_reg(pctl, pin);

	pincfg->irqtype = 0;
	if(reg & REG_GPIOx_PERIPH) {
		pincfg->stat = PINCFG_STAT_PERIPH;
	} else if((reg & REG_GPIOx_IRQ_MASK) == REG_GPIOx_IRQ_OUT) {
		pincfg->stat = PINCFG_STAT_OUTEN | (reg & PINCFG_STAT_OUTVAL);
	} else if((reg & REG_GPIOx_IRQ_MASK) == REG_GPIOx_IRQ_OFF || !(reg & REG_GPIOx_IRQ_MASK)) {
		pincfg->stat = 0;
	} else {
		pincfg->irqtype = reg & REG_GPIOx_IRQ_MASK;
		pincfg->stat = 0;
	}
	apple_gpio_refresh_reg(pctl, pin);
}

/* Pin controller functions */

static int apple_gpio_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct apple_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->npins;
}

static const char *apple_gpio_pinctrl_get_group_name(struct pinctrl_dev *pctldev, unsigned group)
{
	struct apple_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->pins[group].name;
}

static int apple_gpio_pinctrl_get_group_pins(struct pinctrl_dev *pctldev, unsigned group, const unsigned **pins, unsigned *num_pins)
{
	struct apple_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = &pctl->pin_nums[group];
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops apple_gpio_pinctrl_ops = {
	.get_groups_count = apple_gpio_pinctrl_get_groups_count,
	.get_group_name = apple_gpio_pinctrl_get_group_name,
	.get_group_pins = apple_gpio_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

/* Pin multiplexer functions */

static int apple_gpio_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return 2;
}

static const char *apple_gpio_pinmux_get_function_name(struct pinctrl_dev *pctldev, unsigned func)
{
	return func ? "periph" : "gpio";
}

static int apple_gpio_pinmux_get_function_groups(struct pinctrl_dev *pctldev, unsigned func, const char * const **groups, unsigned * const num_groups)
{
	struct apple_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctl->pin_names;
	*num_groups = pctl->npins;
	return 0;
}

static int apple_gpio_pinmux_enable(struct pinctrl_dev *pctldev, unsigned func, unsigned group)
{
	struct apple_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if(func)
		pctl->pin_cfgs[group].stat |= PINCFG_STAT_PERIPH;
	else
		pctl->pin_cfgs[group].stat &= ~PINCFG_STAT_PERIPH;
	apple_gpio_refresh_reg(pctl, group);

	return 0;
}

static const struct pinmux_ops apple_gpio_pinmux_ops = {
	.get_functions_count = apple_gpio_pinmux_get_functions_count,
	.get_function_name = apple_gpio_pinmux_get_function_name,
	.get_function_groups = apple_gpio_pinmux_get_function_groups,
	.set_mux = apple_gpio_pinmux_enable,
};

/* Pin configuration functions */

static int apple_gpio_pinconf_get(struct pinctrl_dev *pctldev, unsigned pin, unsigned long *config)
{
	return -ENOTSUPP;
}

static int apple_gpio_pinconf_set(struct pinctrl_dev *pctldev, unsigned pin, unsigned long *configs, unsigned num_configs)
{
	return -ENOTSUPP;
}

static const struct pinconf_ops apple_gpio_pinconf_ops = {
	.pin_config_get = apple_gpio_pinconf_get,
	.pin_config_set = apple_gpio_pinconf_set,
	.is_generic = true,
};

/* GPIO chip functions */

static int apple_gpio_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	return !(pctl->pin_cfgs[offset].stat & PINCFG_STAT_OUTEN);
}

static int apple_gpio_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	uint32_t reg;

	reg = apple_gpio_get_reg(pctl, offset);
	return !!(reg & REG_GPIOx_DATA);
}

static void apple_gpio_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	if(value)
		pctl->pin_cfgs[offset].stat |= PINCFG_STAT_OUTVAL;
	else
		pctl->pin_cfgs[offset].stat &= ~PINCFG_STAT_OUTVAL;
	apple_gpio_refresh_reg(pctl, offset);
}

static int apple_gpio_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	pctl->pin_cfgs[offset].stat &= ~PINCFG_STAT_OUTEN;
	apple_gpio_refresh_reg(pctl, offset);
	return 0;
}

static int apple_gpio_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	if(value) {
		pctl->pin_cfgs[offset].stat &= ~PINCFG_STAT_PERIPH;
		pctl->pin_cfgs[offset].stat |= PINCFG_STAT_OUTEN | PINCFG_STAT_OUTVAL;
	} else {
		pctl->pin_cfgs[offset].stat &= ~(PINCFG_STAT_OUTVAL | PINCFG_STAT_PERIPH);
		pctl->pin_cfgs[offset].stat |= PINCFG_STAT_OUTEN;
	}
	apple_gpio_refresh_reg(pctl, offset);
	return 0;
}

/* IRQ chip functions */

static void apple_gpio_gpio_irq_ack(struct irq_data *data)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(irq_data_get_irq_chip_data(data));
	unsigned irqgrp = (apple_gpio_get_reg(pctl, data->hwirq) & REG_GPIOx_GRP_MASK) >> REG_GPIOx_GRP_SHIFT;

	writel(1u << (data->hwirq & 31), pctl->base + REG_IRQ(irqgrp, data->hwirq));
}

static void apple_gpio_gpio_irq_mask(struct irq_data *data)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(irq_data_get_irq_chip_data(data));

	pctl->pin_cfgs[data->hwirq].stat &= ~PINCFG_STAT_IRQEN;
	apple_gpio_refresh_reg(pctl, data->hwirq);
}

static void apple_gpio_gpio_irq_unmask(struct irq_data *data)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(irq_data_get_irq_chip_data(data));

	pctl->pin_cfgs[data->hwirq].stat |= PINCFG_STAT_IRQEN;
	apple_gpio_refresh_reg(pctl, data->hwirq);
}

static unsigned int apple_gpio_gpio_irq_startup(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned irqgrp = 0;

	apple_gpio_set_reg(pctl, data->hwirq, REG_GPIOx_GRP_MASK, irqgrp << REG_GPIOx_GRP_SHIFT);

	apple_gpio_gpio_direction_input(chip, data->hwirq);
	apple_gpio_gpio_irq_unmask(data);

	return 0;
}

static int apple_gpio_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(irq_data_get_irq_chip_data(data));

	switch(type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		pctl->pin_cfgs[data->hwirq].irqtype = REG_GPIOx_IRQ_UP;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		pctl->pin_cfgs[data->hwirq].irqtype = REG_GPIOx_IRQ_DN;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		pctl->pin_cfgs[data->hwirq].irqtype = REG_GPIOx_IRQ_ANY;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		pctl->pin_cfgs[data->hwirq].irqtype = REG_GPIOx_IRQ_HI;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		pctl->pin_cfgs[data->hwirq].irqtype = REG_GPIOx_IRQ_LO;
		break;
	default:
		return -EINVAL;
	}

	apple_gpio_refresh_reg(pctl, data->hwirq);

	if(type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(data, handle_level_irq);
	else
		irq_set_handler_locked(data, handle_edge_irq);
	return 0;
}

static void apple_gpio_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct apple_gpio_pinctrl *pctl = gpiochip_get_data(gc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned irqgrp = 0, pinh, pinl;
	unsigned long pending;
	static long count;


	chained_irq_enter(chip, desc);
	for(pinh=0; pinh<pctl->npins; pinh+=32) {
		pending = readl(pctl->base + REG_IRQ(irqgrp, pinh));
		if (pending) {
			if ((count & (count - 1)) == 0) {
				printk("pending: %d/%08lx", pinh, pending);
				for_each_set_bit(pinl, &pending, 32) {
					struct apple_gpio_pincfg *pincfg = &pctl->pin_cfgs[pinh + pinl];
					printk("pin %d stat is %08x\n", pinh + pinl,
					       pincfg->stat);

					apple_gpio_refresh_reg(pctl, pinh + pinl);
					unsigned irqgrp = (apple_gpio_get_reg(pctl, pinh + pinl) & REG_GPIOx_GRP_MASK) >> REG_GPIOx_GRP_SHIFT;

					writel(1u << (pinl), pctl->base + REG_IRQ(irqgrp, pinh + pinl));
				}
			}
			count++;
		}
		for_each_set_bit(pinl, &pending, 32) {
			generic_handle_irq(irq_linear_revmap(gc->irq.domain, pinh + pinl));
		}
	}
	chained_irq_exit(chip, desc);
}

/* Probe & register */

static int apple_gpio_gpio_register(struct apple_gpio_pinctrl *pctl)
{
	struct device_node *node = pctl->dev->of_node;
	struct gpio_irq_chip *girq;
	unsigned int grp;
	int ret = 0;

	if(!of_find_property(node, "gpio-controller", NULL)) {
		dev_err(pctl->dev, "Apple GPIO must have 'gpio-controller' property.\n");
		return -ENODEV;
	}

	pctl->gpio_chip.label = pctl->pin_prefix;
	pctl->gpio_chip.request = gpiochip_generic_request;
	pctl->gpio_chip.free = gpiochip_generic_free;
	pctl->gpio_chip.get_direction = apple_gpio_gpio_get_direction;
	pctl->gpio_chip.direction_input = apple_gpio_gpio_direction_input;
	pctl->gpio_chip.direction_output = apple_gpio_gpio_direction_output;
	pctl->gpio_chip.get = apple_gpio_gpio_get;
	pctl->gpio_chip.set = apple_gpio_gpio_set;
	pctl->gpio_chip.base = pctl->pin_base;
	pctl->gpio_chip.ngpio = pctl->npins;
	pctl->gpio_chip.parent = pctl->dev;
	pctl->gpio_chip.of_node = node;

	pctl->irq_chip.name = pctl->pin_prefix;
	pctl->irq_chip.irq_startup = apple_gpio_gpio_irq_startup;
	pctl->irq_chip.irq_ack = apple_gpio_gpio_irq_ack;
	pctl->irq_chip.irq_mask = apple_gpio_gpio_irq_mask;
	pctl->irq_chip.irq_unmask = apple_gpio_gpio_irq_unmask;
	pctl->irq_chip.irq_set_type = apple_gpio_gpio_irq_set_type;

	girq = &pctl->gpio_chip.irq;
	girq->chip = &pctl->irq_chip;
	girq->parent_handler = apple_gpio_gpio_irq_handler;
	girq->num_parents = pctl->nirqgrps;
	girq->parents = devm_kcalloc(pctl->dev, pctl->nirqgrps, sizeof(*girq->parents), GFP_KERNEL);
	if(!girq->parents)
		return -ENOMEM;

	for(grp=0; grp<pctl->nirqgrps; grp++)
		girq->parents[grp] = pctl->irqs[grp];
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_level_irq;

	ret = gpiochip_add_data(&pctl->gpio_chip, pctl);
	if(ret < 0) {
		dev_err(pctl->dev, "Failed to add GPIO chip (%d).\n", ret);
		return ret;
	}

	ret = gpiochip_add_pin_range(&pctl->gpio_chip, dev_name(pctl->dev), 0, 0, pctl->npins);
	if(ret < 0) {
		dev_err(pctl->dev, "Failed to add GPIO range (%d).\n", ret);
		gpiochip_remove(&pctl->gpio_chip);
		return ret;
	}

	return 0;
}

static const struct of_device_id apple_gpio_pinctrl_of_match[] = {
	{ .compatible = "apple,gpio-v0", },
	{ },
};

static int apple_gpio_pinctrl_probe(struct platform_device *pdev)
{
	struct apple_gpio_pinctrl *pctl;
	struct resource *rsrc;
	int res;
	unsigned i;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if(!pctl)
		return -ENOMEM;
	pctl->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, pctl);

	res = platform_irq_count(pdev); /* may return EPROBE_DEFER */
	if(res < 0)
		return res;
	if(!res) {
		dev_err(&pdev->dev, "Apple GPIO must have at least one IRQ.\n");
		return -EINVAL;
	}
	pctl->nirqgrps = res;

	of_property_read_u32(pdev->dev.of_node, "pin-count", &pctl->npins);
	if(pctl->npins < 0) {
		dev_err(&pdev->dev, "Apple GPIO must have 'pin-count' property.\n");
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "pin-base", &pctl->pin_base);
	if(of_property_read_string(pdev->dev.of_node, "pin-prefix", &pctl->pin_prefix))
		pctl->pin_prefix = "gpio";

	pctl->pins = devm_kzalloc(&pdev->dev, sizeof(pctl->pins[0]) * pctl->npins, GFP_KERNEL);
	if(!pctl->pins)
		return -ENOMEM;
	pctl->pin_names = devm_kzalloc(&pdev->dev, sizeof(pctl->pin_names[0]) * pctl->npins, GFP_KERNEL);
	if(!pctl->pin_names)
		return -ENOMEM;
	pctl->pin_nums = devm_kzalloc(&pdev->dev, sizeof(pctl->pin_nums[0]) * pctl->npins, GFP_KERNEL);
	if(!pctl->pin_nums)
		return -ENOMEM;
	pctl->pin_cfgs = devm_kzalloc(&pdev->dev, sizeof(pctl->pin_cfgs[0]) * pctl->npins, GFP_KERNEL);
	if(!pctl->pin_cfgs)
		return -ENOMEM;
	pctl->irqs = devm_kzalloc(&pdev->dev, sizeof(pctl->irqs[0]) * pctl->nirqgrps, GFP_KERNEL);
	if(!pctl->pins)
		return -ENOMEM;

	for(i=0; i<pctl->nirqgrps; i++) {
		res = platform_get_irq(pdev, i);
		if(res < 0) {
			if(res != -EPROBE_DEFER)
				dev_err(&pdev->dev, "Failed to map IRQ %d (%d).\n", i, res);
			return res;
		}
		pctl->irqs[i] = res;
	}

	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pctl->base = devm_ioremap_resource(&pdev->dev, rsrc);
	if(IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	for(i=0; i<pctl->npins; i++) {
		apple_gpio_init_reg(pctl, i);

		pctl->pins[i].number = i;
		pctl->pins[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s%d", pctl->pin_prefix, i);
		pctl->pins[i].drv_data = pctl;
		pctl->pin_names[i] = pctl->pins[i].name;
		pctl->pin_nums[i] = i;
	}

	pctl->pinctrl_desc.name = pctl->pin_prefix;
	pctl->pinctrl_desc.pins = pctl->pins;
	pctl->pinctrl_desc.npins = pctl->npins;
	pctl->pinctrl_desc.pctlops = &apple_gpio_pinctrl_ops;
	pctl->pinctrl_desc.pmxops = &apple_gpio_pinmux_ops;
	pctl->pinctrl_desc.confops = &apple_gpio_pinconf_ops;

	pctl->pctldev = devm_pinctrl_register(&pdev->dev, &pctl->pinctrl_desc, pctl);
	if (IS_ERR(pctl->pctldev)) {
		dev_err(&pdev->dev, "Failed to register pinctrl device.\n");
		return PTR_ERR(pctl->pctldev);
	}

	writel(0, pctl->base + REG_LOCK);

	return apple_gpio_gpio_register(pctl);
}

static struct platform_driver apple_gpio_pinctrl_driver = {
	.driver = {
		.name = "apple-gpio-pinctrl",
		.of_match_table = apple_gpio_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = apple_gpio_pinctrl_probe,
};

static int __init apple_gpio_pinctrl_register(void)
{
	return platform_driver_register(&apple_gpio_pinctrl_driver);
}
arch_initcall(apple_gpio_pinctrl_register);
