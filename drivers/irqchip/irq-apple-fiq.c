// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright The Asahi Linux Contributors
 *
 * Based on irq-lpc32xx:
 *   Copyright 2015-2016 Vladimir Zapolskiy <vz@mleia.com>
 * Based on irq-bcm2836:
 *   Copyright 2015 Broadcom
 */

/*
 * AIC is a fairly simple interrupt controller with the following features:
 *
 * - 896 level-triggered hardware IRQs
 *   - Single mask bit per IRQ
 *   - Per-IRQ affinity setting
 *   - Automatic masking on event delivery (auto-ack)
 *   - Software triggering (ORed with hw line)
 * - 2 per-CPU IPIs (meant as "self" and "other", but they are
 *   interchangeable if not symmetric)
 * - Automatic prioritization (single event/ack register per CPU, lower IRQs =
 *   higher priority)
 * - Automatic masking on ack
 * - Default "this CPU" register view and explicit per-CPU views
 *
 * In addition, this driver also handles FIQs, as these are routed to the same
 * IRQ vector. These are used for Fast IPIs (TODO), the ARMv8 timer IRQs, and
 * performance counters (TODO).
 *
 * Implementation notes:
 *
 * - This driver creates two IRQ domains, one for HW IRQs and internal FIQs,
 *   and one for IPIs.
 * - Since Linux needs more than 2 IPIs, we implement a software IRQ controller
 *   and funnel all IPIs into one per-CPU IPI (the second "self" IPI is unused).
 * - FIQ hwirq numbers are assigned after true hwirqs, and are per-cpu.
 * - DT bindings use 3-cell form (like GIC):
 *   - <0 nr flags> - hwirq #nr
 *   - <1 nr flags> - FIQ #nr
 *     - nr=0  Physical HV timer
 *     - nr=1  Virtual HV timer
 *     - nr=2  Physical guest timer
 *     - nr=3  Virtual guest timer
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/cpuhotplug.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/limits.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <asm/exception.h>
#include <asm/sysreg.h>
#include <asm/virt.h>

#include <dt-bindings/interrupt-controller/apple-aic.h>

/*
 * IMP-DEF sysregs that control FIQ sources
 * Note: sysreg-based IPIs are not supported yet.
 */

/* Core PMC control register */
#define SYS_IMP_APL_PMCR0_EL1		sys_reg(3, 1, 15, 0, 0)
#define PMCR0_IMODE			GENMASK(10, 8)
#define PMCR0_IMODE_OFF			0
#define PMCR0_IMODE_PMI			1
#define PMCR0_IMODE_AIC			2
#define PMCR0_IMODE_HALT		3
#define PMCR0_IMODE_FIQ			4
#define PMCR0_IACT			BIT(11)

/* IPI request registers */
#define SYS_IMP_APL_IPI_RR_LOCAL_EL1	sys_reg(3, 5, 15, 0, 0)
#define SYS_IMP_APL_IPI_RR_GLOBAL_EL1	sys_reg(3, 5, 15, 0, 1)
#define IPI_RR_CPU			GENMASK(7, 0)
/* Cluster only used for the GLOBAL register */
#define IPI_RR_CLUSTER			GENMASK(23, 16)
#define IPI_RR_TYPE			GENMASK(29, 28)
#define IPI_RR_IMMEDIATE		0
#define IPI_RR_RETRACT			1
#define IPI_RR_DEFERRED			2
#define IPI_RR_NOWAKE			3

/* IPI status register */
#define SYS_IMP_APL_IPI_SR_EL1		sys_reg(3, 5, 15, 1, 1)
#define IPI_SR_PENDING			BIT(0)

/* Guest timer FIQ enable register */
#define SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2	sys_reg(3, 5, 15, 1, 3)
#define VM_TMR_FIQ_ENABLE_V		BIT(0)
#define VM_TMR_FIQ_ENABLE_P		BIT(1)

/* Deferred IPI countdown register */
#define SYS_IMP_APL_IPI_CR_EL1		sys_reg(3, 5, 15, 3, 1)

/* Uncore PMC control register */
#define SYS_IMP_APL_UPMCR0_EL1		sys_reg(3, 7, 15, 0, 4)
#define UPMCR0_IMODE			GENMASK(18, 16)
#define UPMCR0_IMODE_OFF		0
#define UPMCR0_IMODE_AIC		2
#define UPMCR0_IMODE_HALT		3
#define UPMCR0_IMODE_FIQ		4

/* Uncore PMC status register */
#define SYS_IMP_APL_UPMSR_EL1		sys_reg(3, 7, 15, 6, 4)
#define UPMSR_IACT			BIT(0)

#define AIC_NR_FIQ		4
#define AIC_NR_SWIPI		32

/*
 * FIQ hwirq index definitions: FIQ sources use the DT binding defines
 * directly, except that timers are special. At the irqchip level, the
 * two timer types are represented by their access method: _EL0 registers
 * or _EL02 registers. In the DT binding, the timers are represented
 * by their purpose (HV or guest). This mapping is for when the kernel is
 * running at EL2 (with VHE). When the kernel is running at EL1, the
 * mapping differs and aic_irq_domain_translate() performs the remapping.
 */

#define AIC_TMR_EL0_PHYS	AIC_TMR_HV_PHYS
#define AIC_TMR_EL0_VIRT	AIC_TMR_HV_VIRT
#define AIC_TMR_EL02_PHYS	AIC_TMR_GUEST_PHYS
#define AIC_TMR_EL02_VIRT	AIC_TMR_GUEST_VIRT

struct aic_irq_chip {
	void __iomem *base;
	struct irq_domain *hw_domain;
	struct irq_domain *ipi_domain;
	int nr_hw;
	int ipi_hwirq;
};

static DEFINE_PER_CPU(uint32_t, aic_fiq_unmasked);

static DEFINE_PER_CPU(atomic_t, aic_vipi_flag);
static DEFINE_PER_CPU(atomic_t, aic_vipi_enable);

static struct aic_irq_chip *aic_irqc;

static int aic_irq_set_type(struct irq_data *d, unsigned int type)
{
	/*
	 * Some IRQs (e.g. MSIs) implicitly have edge semantics, and we don't
	 * have a way to find out the type of any given IRQ, so just allow both.
	 */
	return (type == IRQ_TYPE_LEVEL_HIGH || type == IRQ_TYPE_EDGE_RISING) ? 0 : 0;
}

/*
 * FIQ irqchip
 */

static unsigned long aic_fiq_get_idx(struct irq_data *d)
{
	struct aic_irq_chip *ic = irq_data_get_irq_chip_data(d);

	return irqd_to_hwirq(d) - ic->nr_hw;
}

static void aic_fiq_set_mask(struct irq_data *d)
{
	/* Only the guest timers have real mask bits, unfortunately. */
	switch (aic_fiq_get_idx(d)) {
	case AIC_TMR_EL02_PHYS:
		sysreg_clear_set_s(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENABLE_P, 0);
		isb();
		break;
	case AIC_TMR_EL02_VIRT:
		sysreg_clear_set_s(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENABLE_V, 0);
		isb();
		break;
	default:
		break;
	}
}

static void aic_fiq_clear_mask(struct irq_data *d)
{
	switch (aic_fiq_get_idx(d)) {
	case AIC_TMR_EL02_PHYS:
		sysreg_clear_set_s(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, 0, VM_TMR_FIQ_ENABLE_P);
		isb();
		break;
	case AIC_TMR_EL02_VIRT:
		sysreg_clear_set_s(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, 0, VM_TMR_FIQ_ENABLE_V);
		isb();
		break;
	default:
		break;
	}
}

static void aic_fiq_mask(struct irq_data *d)
{
	aic_fiq_set_mask(d);
	__this_cpu_and(aic_fiq_unmasked, ~BIT(aic_fiq_get_idx(d)));
}

static void aic_fiq_unmask(struct irq_data *d)
{
	aic_fiq_clear_mask(d);
	__this_cpu_or(aic_fiq_unmasked, BIT(aic_fiq_get_idx(d)));
}

static void aic_fiq_eoi(struct irq_data *d)
{
	/* We mask to ack (where we can), so we need to unmask at EOI. */
	if (__this_cpu_read(aic_fiq_unmasked) & BIT(aic_fiq_get_idx(d)))
		aic_fiq_clear_mask(d);
}

#define TIMER_FIRING(x)                                                        \
	(((x) & (ARCH_TIMER_CTRL_ENABLE | ARCH_TIMER_CTRL_IT_MASK |            \
		 ARCH_TIMER_CTRL_IT_STAT)) ==                                  \
	 (ARCH_TIMER_CTRL_ENABLE | ARCH_TIMER_CTRL_IT_STAT))

static void __exception_irq_entry aic_handle_irq(struct pt_regs *regs)
{
#define AIC_EVENT		0x2004
  u32 event = readl(aic_irqc->base + AIC_EVENT);
  printk("! %08x", event);
  int i;
  for (i = 0; i < 896; i++)
#define AIC_TARGET_CPU		0x3000
    writel(readl(aic_irqc->base + AIC_TARGET_CPU + i * 4) &~ 0x80,
	   aic_irqc->base + AIC_TARGET_CPU + i * 4);
  writel(BIT(event&0x1f),
	 aic_irqc->base + 0x4180 + 4 * ((event & 0xffff)/32));
}
static void __exception_irq_entry aic_handle_fiq(struct pt_regs *regs)
{
	/*
	 * It would be really nice if we had a system register that lets us get
	 * the FIQ source state without having to peek down into sources...
	 * but such a register does not seem to exist.
	 *
	 * So, we have these potential sources to test for:
	 *  - Fast IPIs (not yet used)
	 *  - The 4 timers (CNTP, CNTV for each of HV and guest)
	 *  - Per-core PMCs (not yet supported)
	 *  - Per-cluster uncore PMCs (not yet supported)
	 *
	 * Since not dealing with any of these results in a FIQ storm,
	 * we check for everything here, even things we don't support yet.
	 */

	if (read_sysreg_s(SYS_IMP_APL_IPI_SR_EL1) & IPI_SR_PENDING) {
		pr_err_ratelimited("Fast IPI fired. Acking.\n");
		write_sysreg_s(IPI_SR_PENDING, SYS_IMP_APL_IPI_SR_EL1);
	}

	if (TIMER_FIRING(read_sysreg(cntp_ctl_el0)))
		handle_domain_irq(aic_irqc->hw_domain,
				  aic_irqc->nr_hw, regs);

	if (TIMER_FIRING(read_sysreg(cntv_ctl_el0)))
		handle_domain_irq(aic_irqc->hw_domain,
				  aic_irqc->nr_hw + 1, regs);

	if (is_kernel_in_hyp_mode()) {
		uint64_t enabled = read_sysreg_s(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2);

		if ((enabled & VM_TMR_FIQ_ENABLE_P) &&
		    TIMER_FIRING(read_sysreg_s(SYS_CNTP_CTL_EL02)))
			handle_domain_irq(aic_irqc->hw_domain,
					  aic_irqc->nr_hw, regs);

		if ((enabled & VM_TMR_FIQ_ENABLE_V) &&
		    TIMER_FIRING(read_sysreg_s(SYS_CNTV_CTL_EL02)))
			handle_domain_irq(aic_irqc->hw_domain,
					  aic_irqc->nr_hw + 1, regs);
	}

	if ((read_sysreg_s(SYS_IMP_APL_PMCR0_EL1) & (PMCR0_IMODE | PMCR0_IACT)) ==
			(FIELD_PREP(PMCR0_IMODE, PMCR0_IMODE_FIQ) | PMCR0_IACT)) {
		/*
		 * Not supported yet, let's figure out how to handle this when
		 * we implement these proprietary performance counters. For now,
		 * just mask it and move on.
		 */
		pr_err_ratelimited("PMC FIQ fired. Masking.\n");
		sysreg_clear_set_s(SYS_IMP_APL_PMCR0_EL1, PMCR0_IMODE | PMCR0_IACT,
				   FIELD_PREP(PMCR0_IMODE, PMCR0_IMODE_OFF));
	}

	if (FIELD_GET(UPMCR0_IMODE, read_sysreg_s(SYS_IMP_APL_UPMCR0_EL1)) == UPMCR0_IMODE_FIQ &&
			(read_sysreg_s(SYS_IMP_APL_UPMSR_EL1) & UPMSR_IACT)) {
		/* Same story with uncore PMCs */
		pr_err_ratelimited("Uncore PMC FIQ fired. Masking.\n");
		sysreg_clear_set_s(SYS_IMP_APL_UPMCR0_EL1, UPMCR0_IMODE,
				   FIELD_PREP(UPMCR0_IMODE, UPMCR0_IMODE_OFF));
	}
}

static int aic_fiq_set_type(struct irq_data *d, unsigned int type)
{
	return (type == IRQ_TYPE_LEVEL_HIGH) ? 0 : -EINVAL;
}

static struct irq_chip fiq_chip = {
	.name = "AIC-FIQ",
	.irq_mask = aic_fiq_mask,
	.irq_unmask = aic_fiq_unmask,
	.irq_ack = aic_fiq_set_mask,
	.irq_eoi = aic_fiq_eoi,
	.irq_set_type = aic_fiq_set_type,
};

/*
 * Main IRQ domain
 */

static int aic_irq_domain_map(struct irq_domain *id, unsigned int irq,
			      irq_hw_number_t hw)
{
	struct aic_irq_chip *ic = id->host_data;

	irq_set_percpu_devid(irq);
	irq_domain_set_info(id, irq, hw, &fiq_chip, id->host_data,
			    handle_percpu_devid_irq, NULL, NULL);

	return 0;
}

static int aic_irq_domain_translate(struct irq_domain *id,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	struct aic_irq_chip *ic = id->host_data;

	if (fwspec->param_count != 3 || !is_of_node(fwspec->fwnode))
		return -EINVAL;

	switch (fwspec->param[0]) {
	case AIC_IRQ:
		if (fwspec->param[1] >= ic->nr_hw)
			return -EINVAL;
		*hwirq = fwspec->param[1];
		break;
	case AIC_FIQ:
		if (fwspec->param[1] >= AIC_NR_FIQ)
			return -EINVAL;
		*hwirq = ic->nr_hw + (fwspec->param[1] & 1);

		/*
		 * In EL1 the non-redirected registers are the guest's,
		 * not EL2's, so remap the hwirqs to match.
		 */
		if (!is_kernel_in_hyp_mode()) {
			switch (fwspec->param[1]) {
			case AIC_TMR_GUEST_PHYS:
			  *hwirq = ic->nr_hw;
				break;
			case AIC_TMR_GUEST_VIRT:
			  *hwirq = ic->nr_hw + 1;
				break;
			case AIC_TMR_HV_PHYS:
			case AIC_TMR_HV_VIRT:
				return -ENOENT;
			default:
				break;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static int aic_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	unsigned int type = IRQ_TYPE_NONE;
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	int i, ret;

	ret = aic_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		ret = aic_irq_domain_map(domain, virq + i, hwirq + i);
		if (ret)
			return ret;
	}

	return 0;
}

static void aic_irq_domain_free(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops aic_irq_domain_ops = {
	.translate	= aic_irq_domain_translate,
	.alloc		= aic_irq_domain_alloc,
	.free		= aic_irq_domain_free,
};

void apple_aic_cpu_prepare(unsigned int cpu)
{
}

static int __init aic_of_ic_init(struct device_node *node, struct device_node *parent)
{
	int i;
	void __iomem *regs;
	u32 info;
	struct aic_irq_chip *irqc;

	regs = of_iomap(node, 0);
	if (WARN_ON(!regs))
		return -EIO;

	irqc = kzalloc(sizeof(*irqc), GFP_KERNEL);
	if (!irqc)
		return -ENOMEM;

	aic_irqc = irqc;
	irqc->base = regs;

	irqc->nr_hw = 0;

	irqc->hw_domain = irq_domain_create_linear(of_node_to_fwnode(node),
						   irqc->nr_hw + AIC_NR_FIQ,
						   &aic_irq_domain_ops, irqc);

	for (i = 0; i < irqc->nr_hw; i++)
		irq_set_status_flags(i, IRQ_DISABLE_UNLAZY);
	irq_set_default_host(irqc->hw_domain);
	if (WARN_ON(!irqc->hw_domain)) {
		iounmap(irqc->base);
		kfree(irqc);
		return -ENODEV;
	}

	irq_domain_update_bus_token(irqc->hw_domain, DOMAIN_BUS_WIRED);

	set_handle_irq(aic_handle_irq);
	set_handle_fiq(aic_handle_fiq);

#if 0
	for (i = 0; i < BITS_TO_U32(irqc->nr_hw); i++)
		aic_ic_write(irqc, AIC_MASK_SET + i * 4, U32_MAX);
	for (i = 0; i < BITS_TO_U32(irqc->nr_hw); i++)
		aic_ic_write(irqc, AIC_SW_CLR + i * 4, U32_MAX);
	for (i = 0; i < irqc->nr_hw; i++)
		aic_ic_write(irqc, AIC_TARGET_CPU + i * 4, 1);
#endif

	if (!is_kernel_in_hyp_mode())
		pr_info("Kernel running in EL1, mapping interrupts");

	pr_info("Initialized with %d FIQs", AIC_NR_FIQ);

	return 0;
}

IRQCHIP_DECLARE(apple_m1_aic, "apple,aic", aic_of_ic_init);
