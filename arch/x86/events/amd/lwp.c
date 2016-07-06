/*
 * Performance events x86 architecture code
 *
 *  Copyright (C) 2008 Thomas Gleixner <tglx@linutronix.de>
 *  Copyright (C) 2008-2009 Red Hat, Inc., Ingo Molnar
 *  Copyright (C) 2009 Jaswinder Singh Rajput
 *  Copyright (C) 2009 Advanced Micro Devices, Inc., Robert Richter
 *  Copyright (C) 2008-2009 Red Hat, Inc., Peter Zijlstra
 *  Copyright (C) 2009 Intel Corporation, <markus.t.metzger@intel.com>
 *  Copyright (C) 2009 Google, Inc., Stephane Eranian
 *
 *  For licencing details see kernel-base/COPYING
 */

#include <linux/perf_event.h>
#include <linux/capability.h>
#include <linux/hardirq.h>
#include <linux/kprobes.h>
#include <linux/module.h>
#include <linux/kdebug.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/printk.h>

#include <asm/apic.h>
#include <asm/stacktrace.h>
#include <asm/nmi.h>
#include <asm/smp.h>
#include <asm/alternative.h>
#include <asm/mmu_context.h>
#include <asm/tlbflush.h>
#include <asm/timer.h>
#include <asm/desc.h>
#include <asm/ldt.h>
#include <asm/cpu_device_id.h>

#include "../perf_event.h"

static struct pmu pmu;

struct lwp_hw_events {
        struct perf_event	*event;
        int                     enabled;
};

DEFINE_PER_CPU(struct lwp_hw_events, lwp_hw_events) = {
        .enabled = 1,
};

/*
 * Propagate event elapsed time into the generic event.
 * Can only be executed on the CPU where the event is active.
 * Returns the delta events processed.
 */
static u64 lwp_event_update(struct perf_event *event)
{
        local64_add(event->hw.ptsc, &event->count);

        return 0;
}

static atomic_t active_events;
extern bool request_lwp_interrupt(void (*handler)(struct pt_regs *));

static void lwp_interrupt(struct pt_regs *regs);
static irqreturn_t lwp_irq(int irqno, void *dummy) {
        lwp_interrupt(NULL);

        return IRQ_HANDLED;
}

static bool reserve_lwp_hardware(void)
{
        return request_lwp_interrupt(lwp_interrupt) ||
          request_lwp_interrupt(NULL) && request_lwp_interrupt(lwp_interrupt);
}

static void release_lwp_hardware(void)
{
        request_lwp_interrupt(NULL);
}

static bool check_hw_exists(void)
{
        u64 val, val_fail, val_new= ~0;
        int reg, reg_fail, ret = 0;
        int bios_fail = 0;

        /*
         * Check to see if the BIOS enabled LWP. If so,
         * complain and bail.
         */
        reg = MSR_AMD64_LWP_CFG;
        ret = rdmsrl_safe(reg, &val);

        if (ret)
                goto msr_fail;

        if (0 && val) {
                bios_fail = 1;
                val_fail = val;
                reg_fail = reg;
                goto msr_fail;
        }

        return true;

msr_fail:
        pr_cont("Broken PMU hardware detected, using software events only.\n");
        pr_info("%sFailed to access perfctr msr (MSR %x is %Lx)\n",
                boot_cpu_has(X86_FEATURE_HYPERVISOR) ? KERN_INFO : KERN_ERR,
                reg, val_new);

        return false;
}

static void lwp_release_hardware(void);

static void hw_perf_event_destroy(struct perf_event *event)
{
                lwp_release_hardware();
        atomic_dec(&active_events);
}

static int lwp_reserve_hardware(void)
{
        int err = 0;

			if (!reserve_lwp_hardware())
				err = -EBUSY;

        return err;
}

static void lwp_release_hardware(void)
{
        release_lwp_hardware();
}

static int lwp_setup_perfctr(struct perf_event *event)
{
        struct perf_event_attr *attr = &event->attr;

        if (attr->type != pmu.type)
                return -EINVAL;

        if (attr->exclude_user || !attr->exclude_kernel)
                return -EOPNOTSUPP;

        return 0;
}

static int lwp_hw_config(struct perf_event *event)
{
        return lwp_setup_perfctr(event);
}

/*
 * Setup the hardware configuration for a given attr_type
 */
static int __lwp_event_init(struct perf_event *event)
{
        int err;

        err = lwp_reserve_hardware();
        if (err)
                return err;

        atomic_inc(&active_events);
        event->destroy = hw_perf_event_destroy;

        event->hw.idx = -1;
        event->hw.last_cpu = -1;
        event->hw.last_tag = ~0ULL;

        /* mark unused */
        event->hw.extra_reg.idx = EXTRA_REG_NONE;
        event->hw.branch_reg.idx = EXTRA_REG_NONE;

        return lwp_hw_config(event);
}

static void refresh_lwp_cfg(void *ignored);

static unsigned long lwp_cfg = 0xe000000e;

static void lwp_disable_all(void)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        if (lwp->enabled)
                lwp->enabled = 0;

        //lwp_cfg = 0;
        refresh_lwp_cfg(&lwp_cfg);
}

static void lwp_disable(struct pmu *pmu)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        if (!lwp->enabled)
                return;

        lwp->enabled = 0;
        barrier();

        lwp_disable_all();
}

static void __lwp_enable_event(struct hw_perf_event *hwc,
                               u64 enable_mask)
{
}

static void lwp_enable_all(int added)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        if (lwp->event)
                __lwp_enable_event(NULL, ARCH_PERFMON_EVENTSEL_ENABLE);
}

static struct pmu pmu;

static void lwp_start(struct perf_event *event, int flags);

static void lwp_enable(struct pmu *pmu)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);
        struct perf_event *event;
	struct hw_perf_event *hwc;

        if (lwp->enabled)
                return;

        event = lwp->event;
        //hwc = &event->hw;

        lwp_start(event, PERF_EF_RELOAD);

	lwp->enabled = 1;
        barrier();

        lwp_enable_all(1);
}

void lwp_enable_event(struct perf_event *event)
{
	if (__this_cpu_read(lwp_hw_events.enabled))
        __lwp_enable_event(&event->hw,
                           ARCH_PERFMON_EVENTSEL_ENABLE);
}

/*
 * Add a single event to the PMU.
 *
 * The event is added to the group of enabled events
 * but only if it can be scehduled with existing events.
 */
static int lwp_add(struct perf_event *event, int flags)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        lwp->event = event;

        return 0;
}

static void lwp_start(struct perf_event *event, int flags)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);
	int idx = 1;

        if (!event)
                return;

	//if (WARN_ON_ONCE(!(event->hw.state & PERF_HES_STOPPED)))
	//	return;

	if (WARN_ON_ONCE(idx == -1))
		return;

        if (flags & PERF_EF_RELOAD) {
		WARN_ON_ONCE(!(event->hw.state & PERF_HES_UPTODATE));
        }

        event->hw.state = 0;

        lwp->event = event;
        lwp->enabled = true;
}

void lwp_stop(struct perf_event *event, int flags)
{
        struct hw_perf_event *hwc = &event->hw;
        hwc->state |= PERF_HES_UPTODATE | PERF_HES_STOPPED;
        //lwp_cfg = 0;

        refresh_lwp_cfg(&lwp_cfg);
}

static void lwp_del(struct perf_event *event, int flags)
{
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        lwp->event = NULL;
}

int lwp_handle_irq(struct pt_regs *regs)
{
        struct perf_sample_data data;
        struct lwp_hw_events *lwp;
        struct perf_event *event;
        int handled = 0;

        lwp = this_cpu_ptr(&lwp_hw_events);

        event = lwp->event;

        if (event) {
                lwp_event_update(event);
                handled++;

                rdmsrl_safe(MSR_F15H_PTSC, &event->hw.ptsc);
                perf_sample_data_init(&data, 0, event->hw.ptsc);
                local64_add(10000, &event->count);
                local64_sub(10000, &event->hw.period_left);
                printk(KERN_WARNING "is sampling event: %d\n",
                       is_sampling_event(event));
                perf_event_overflow(event, &data, regs);
        } else {
                printk(KERN_WARNING "no event\n");
        }

        if (handled)
                inc_irq_stat(lwp_irqs);

        return handled;
}

static void
lwp_interrupt(struct pt_regs *regs)
{
        u64 start_clock;
        u64 finish_clock;
        int ret;

        printk(KERN_WARNING "lwp_interrupt\n");

        /*
         * All PMUs/events that share this PMI handler should make sure to
         * increment active_events for their events.
         */
        if (!atomic_read(&active_events)) {
                printk(KERN_WARNING "no active events.\n");
                return;
        }

        start_clock = sched_clock();
        ret = lwp_handle_irq(regs);
        finish_clock = sched_clock();

        //perf_sample_event_took(finish_clock - start_clock);
}

PMU_FORMAT_ATTR(event, "config:0-7");

static struct attribute *formats_attr[] = {
        &format_attr_event.attr,
        NULL,
};

static struct attribute_group lwp_format_group = {
        .name = "format",
        .attrs = formats_attr,
};

EVENT_ATTR(lwp-interrupts,		LWP_INTERRUPTS		);

static struct attribute *events_attr[] = {
        EVENT_PTR(LWP_INTERRUPTS),
        NULL,
};

static struct attribute_group lwp_events_group = {
        .name = "events",
        .attrs = events_attr,
};

ssize_t lwp_event_sysfs_show(char *page, u64 config, u64 event)
{
        ssize_t ret;

        /*
        * We have whole page size to spend and just little data
        * to write, so we can safely use sprintf.
        */
        ret = sprintf(page, "event=0x00\n");

        return ret;
}

static int __init lwp_pmu_init(void)
{
        return 0;
}

static const struct x86_cpu_id cpu_match[] = {
        { .vendor = X86_VENDOR_AMD, .family = 0x15 },
        {},
};

static int __init lwp_init(void)
{
        int ret;
        int err = 0;

        if (boot_cpu_data.x86 < 6)
                return -ENODEV;

        ret = lwp_pmu_init();
        if (ret)
                return ret;

        pr_info("Light-Weight Profiling Events: ");

        switch (boot_cpu_data.x86_vendor) {
        case X86_VENDOR_AMD:
                err = lwp_pmu_init();
                break;
        default:
                err = -ENOTSUPP;
        }
        if (err != 0) {
                pr_cont("no LWP driver.\n");
                return 0;
        }

        /* sanity check that the hardware exists or is emulated */
        if (!check_hw_exists())
                return 0;

        pr_cont("LWP driver.\n");

        perf_pmu_register(&pmu, "lwp", -1);

        return 0;
}

static void __exit lwp_exit(void)
{
        perf_pmu_unregister(&pmu);
}

static inline void lwp_read(struct perf_event *event)
{
        //lwp_event_update(event);
}

static int lwp_event_init(struct perf_event *event)
{
        int err;

        if (event->attr.type != pmu.type)
                return -ENOENT;

        err = __lwp_event_init(event);
        if (err) {
                if (event->destroy)
                        event->destroy(event);
        }

        return err;
}

static void refresh_lwp_cfg(void *cfg_raw)
{
        unsigned long *cfg = cfg_raw;
        wrmsrl(MSR_AMD64_LWP_CFG, (cfg && *cfg) ? (*cfg + ((u64)0xff << 40)) : 0);
}

static void lwp_event_mapped(struct perf_event *event)
{
        on_each_cpu(refresh_lwp_cfg, &lwp_cfg, 1);
}

static void lwp_event_unmapped(struct perf_event *event)
{
}

static int lwp_event_idx(struct perf_event *event)
{
        return 1;
}

static const struct attribute_group *lwp_attr_groups[] = {
        &lwp_format_group,
        &lwp_events_group,
        NULL,
};

static void lwp_sched_task(struct perf_event_context *ctx, bool sched_in)
{
        struct lwp_task_context *lwp_ctx = ctx->task_ctx_data;
        struct lwp_hw_events *lwp = this_cpu_ptr(&lwp_hw_events);

        if (lwp->enabled && sched_in)
                refresh_lwp_cfg(&lwp_cfg);
        else
                refresh_lwp_cfg(&lwp_cfg);
}

struct lwp_task_context {
        unsigned long            lwp_cfg;
};

static struct pmu pmu = {
        .task_ctx_nr            = perf_sw_context,
        .pmu_enable		= lwp_enable,
        .pmu_disable		= lwp_disable,

        .attr_groups		= lwp_attr_groups,

        .event_init		= lwp_event_init,

        .event_mapped		= lwp_event_mapped,
        .event_unmapped		= lwp_event_unmapped,

        .add			= lwp_add,
        .del			= lwp_del,
        .start			= lwp_start,
        .stop			= lwp_stop,
        .read			= lwp_read,

        .event_idx		= lwp_event_idx,
        .sched_task		= lwp_sched_task,
        .task_ctx_size          = sizeof(struct lwp_task_context),
};

module_init(lwp_init);
module_exit(lwp_exit);

MODULE_AUTHOR("Pip Cet <pipcet@gmail.com>");
MODULE_DESCRIPTION("AMD Light-Weight Profiling");
MODULE_LICENSE("GPL v2");
