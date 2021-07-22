// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple DART (Device Address Resolution Table) IOMMU driver
 *
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * Based on arm/arm-smmu/arm-ssmu.c and arm/arm-smmu-v3/arm-smmu-v3.c
 *  Copyright (C) 2013 ARM Limited
 *  Copyright (C) 2015 ARM Limited
 * and on exynos-iommu.c
 *  Copyright (c) 2011,2016 Samsung Electronics Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-direct.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io-pgtable.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/pci.h>

#define DART_MAX_STREAMS 16
#define DART_MAX_TTBR 4

#define DART_STREAM_ALL 0xffff

#define DART_PARAMS1 0x00
#define DART_PARAMS_PAGE_SHIFT GENMASK(27, 24)

#define DART_PARAMS2 0x04
#define DART_PARAMS_BYPASS_SUPPORT BIT(0)

#define DART_STREAM_COMMAND 0x20
#define DART_STREAM_COMMAND_BUSY BIT(2)
#define DART_STREAM_COMMAND_INVALIDATE BIT(20)

#define DART_STREAM_SELECT 0x34

#define DART_ERROR 0x40
#define DART_ERROR_STREAM GENMASK(27, 24)
#define DART_ERROR_CODE GENMASK(23, 0)
#define DART_ERROR_FLAG BIT(31)
#define DART_ERROR_READ_FAULT BIT(4)
#define DART_ERROR_WRITE_FAULT BIT(3)
#define DART_ERROR_NO_PTE BIT(2)
#define DART_ERROR_NO_PMD BIT(1)
#define DART_ERROR_NO_TTBR BIT(0)

#define DART_CONFIG 0x60
#define DART_CONFIG_LOCK BIT(15)

#define DART_STREAM_COMMAND_BUSY_TIMEOUT 100

#define DART_STREAM_REMAP 0x80

#define DART_ERROR_ADDR_HI 0x54
#define DART_ERROR_ADDR_LO 0x50

#define DART_TCR(sid) (0x100 + 4 * (sid))
#define DART_TCR_TRANSLATE_ENABLE BIT(7)
#define DART_TCR_BYPASS0_ENABLE BIT(8)
#define DART_TCR_BYPASS1_ENABLE BIT(12)

#define DART_TTBR(sid, idx) (0x200 + 16 * (sid) + 4 * (idx))
#define DART_TTBR_VALID BIT(31)
#define DART_TTBR_SHIFT 12

/*
 * Private structure associated with each DART device.
 *
 * @dev: device struct
 * @regs: mapped MMIO region
 * @irq: interrupt number, can be shared with other DARTs
 * @clks: clocks associated with this DART
 * @num_clks: number of @clks
 * @lock: lock for @used_sids and hardware operations involving this dart
 * @used_sids: bitmap of streams attached to a domain
 * @pgsize: pagesize supported by this DART
 * @supports_bypass: indicates if this DART supports bypass mode
 * @force_bypass: force bypass mode due to pagesize mismatch?
 * @sw_bypass_cpu_start: offset into cpu address space in software bypass mode
 * @sw_bypass_dma_start: offset into dma address space in software bypass mode
 * @sw_bypass_len: length of iova space in software bypass mode
 * @iommu: iommu core device
 */
struct apple_dart {
	struct device *dev;

	void __iomem *regs;

	int irq;
	struct clk_bulk_data *clks;
	int num_clks;

	spinlock_t lock;

	u32 used_sids;
	u32 pgsize;

	u32 supports_bypass : 1;
	u32 force_bypass : 1;

	u64 sw_bypass_cpu_start;
	u64 sw_bypass_dma_start;
	u64 sw_bypass_len;

	struct iommu_device iommu;
};

/*
 * This structure is used to identify a single stream attached to a domain.
 * It's used as a list inside that domain to be able to attach multiple
 * streams to a single domain. Since multiple devices can use a single stream
 * it additionally keeps track of how many devices are represented by this
 * stream. Once that number reaches zero it is detached from the IOMMU domain
 * and all translations from this stream are disabled.
 *
 * @dart: DART instance to which this stream belongs
 * @sid: stream id within the DART instance
 * @num_devices: count of devices attached to this stream
 * @stream_head: list head for the next stream
 */
struct apple_dart_stream {
	struct apple_dart *dart;
	u32 sid;

	u32 num_devices;

	struct list_head stream_head;
};

/*
 * This structure is attached to each iommu domain handled by a DART.
 * A single domain is used to represent a single virtual address space.
 * It is always allocated together with a page table.
 *
 * Streams are the smallest units the DART hardware can differentiate.
 * These are pointed to the page table of a domain whenever a device is
 * attached to it. A single stream can only be assigned to a single domain.
 *
 * Devices are assigned to at least a single and sometimes multiple individual
 * streams (using the iommus property in the device tree). Multiple devices
 * can theoretically be represented by the same stream, though this is usually
 * not the case.
 *
 * We only keep track of streams here and just count how many devices are
 * represented by each stream. When the last device is removed the whole stream
 * is removed from the domain.
 *
 * @dart: pointer to the DART instance
 * @pgtbl_ops: pagetable ops allocated by io-pgtable
 * @type: domain type IOMMU_DOMAIN_IDENTITY_{IDENTITY,DMA,UNMANAGED,BLOCKED}
 * @sw_bypass_cpu_start: offset into cpu address space in software bypass mode
 * @sw_bypass_dma_start: offset into dma address space in software bypass mode
 * @sw_bypass_len: length of iova space in software bypass mode
 * @streams: list of streams attached to this domain
 * @lock: spinlock for operations involving the list of streams
 * @domain: core iommu domain pointer
 */
struct apple_dart_domain {
	struct apple_dart *dart;
	struct io_pgtable_ops *pgtbl_ops;

	unsigned int type;

	u64 sw_bypass_cpu_start;
	u64 sw_bypass_dma_start;
	u64 sw_bypass_len;

	struct list_head streams;

	spinlock_t lock;

	struct iommu_domain domain;
};

/*
 * This structure is attached to devices with dev_iommu_priv_set() on of_xlate
 * and contains a list of streams bound to this device as defined in the
 * device tree. Multiple DART instances can be attached to a single device
 * and each stream is identified by its stream id.
 * It's usually reference by a pointer called *cfg.
 *
 * A dynamic array instead of a linked list is used here since in almost
 * all cases a device will just be attached to a single stream and streams
 * are never removed after they have been added.
 *
 * @num_streams: number of streams attached
 * @streams: array of structs to identify attached streams and the device link
 *           to the iommu
 */
struct apple_dart_master_cfg {
	int num_streams;
	struct {
		struct apple_dart *dart;
		u32 sid;

		struct device_link *link;
	} streams[];
};

static struct platform_driver apple_dart_driver;
static const struct iommu_ops apple_dart_iommu_ops;
static const struct iommu_flush_ops apple_dart_tlb_ops;

static struct apple_dart_domain *to_dart_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct apple_dart_domain, domain);
}

static void apple_dart_hw_enable_translation(struct apple_dart *dart, u16 sid)
{
	writel(DART_TCR_TRANSLATE_ENABLE, dart->regs + DART_TCR(sid));
}

static void apple_dart_hw_disable_dma(struct apple_dart *dart, u16 sid)
{
	writel(0, dart->regs + DART_TCR(sid));
}

static void apple_dart_hw_enable_bypass(struct apple_dart *dart, u16 sid)
{
	WARN_ON(!dart->supports_bypass);
	writel(DART_TCR_BYPASS0_ENABLE | DART_TCR_BYPASS1_ENABLE,
	       dart->regs + DART_TCR(sid));
}

static void apple_dart_hw_set_ttbr(struct apple_dart *dart, u16 sid, u16 idx,
				   phys_addr_t paddr)
{
	writel(DART_TTBR_VALID | (paddr >> DART_TTBR_SHIFT),
	       dart->regs + DART_TTBR(sid, idx));
}

static void apple_dart_hw_clear_ttbr(struct apple_dart *dart, u16 sid, u16 idx)
{
	writel(0, dart->regs + DART_TTBR(sid, idx));
}

static void apple_dart_hw_clear_all_ttbrs(struct apple_dart *dart, u16 sid)
{
	int i;

	for (i = 0; i < 4; ++i)
		apple_dart_hw_clear_ttbr(dart, sid, i);
}

static int apple_dart_hw_stream_command(struct apple_dart *dart, u16 sid_bitmap,
					u32 command)
{
	unsigned long flags;
	int ret;
	u32 command_reg;

	spin_lock_irqsave(&dart->lock, flags);

	writel(sid_bitmap, dart->regs + DART_STREAM_SELECT);
	writel(command, dart->regs + DART_STREAM_COMMAND);

	ret = readl_poll_timeout_atomic(
		dart->regs + DART_STREAM_COMMAND, command_reg,
		!(command_reg & DART_STREAM_COMMAND_BUSY), 1,
		DART_STREAM_COMMAND_BUSY_TIMEOUT);

	spin_unlock_irqrestore(&dart->lock, flags);

	if (ret) {
		dev_err(dart->dev,
			"busy bit did not clear after command %x for streams %x\n",
			command, sid_bitmap);
		return ret;
	}

	return 0;
}

static int apple_dart_hw_invalidate_tlb_global(struct apple_dart *dart)
{
	return apple_dart_hw_stream_command(dart, DART_STREAM_ALL,
					    DART_STREAM_COMMAND_INVALIDATE);
}

static int apple_dart_hw_invalidate_tlb_stream(struct apple_dart *dart, u16 sid)
{
	return apple_dart_hw_stream_command(dart, 1 << sid,
					    DART_STREAM_COMMAND_INVALIDATE);
}

static int apple_dart_hw_reset(struct apple_dart *dart)
{
	int sid;
	u32 config;

	config = readl(dart->regs + DART_CONFIG);
	if (config & DART_CONFIG_LOCK) {
		dev_err(dart->dev, "DART is locked down until reboot: %08x\n",
			config);
		return -EINVAL;
	}

	for (sid = 0; sid < DART_MAX_STREAMS; ++sid) {
		apple_dart_hw_disable_dma(dart, sid);
		apple_dart_hw_clear_all_ttbrs(dart, sid);
	}

	/* restore stream identity map */
	writel(0x03020100, dart->regs + DART_STREAM_REMAP);
	writel(0x07060504, dart->regs + DART_STREAM_REMAP + 4);
	writel(0x0b0a0908, dart->regs + DART_STREAM_REMAP + 8);
	writel(0x0f0e0d0c, dart->regs + DART_STREAM_REMAP + 12);

	/* clear any pending errors before the interrupt is unmasked */
	writel(readl(dart->regs + DART_ERROR), dart->regs + DART_ERROR);

	return apple_dart_hw_invalidate_tlb_global(dart);
}

static void apple_dart_domain_flush_tlb(struct apple_dart_domain *domain)
{
	unsigned long flags;
	struct apple_dart_stream *stream;
	struct apple_dart *dart = domain->dart;

	if (!dart)
		return;

	spin_lock_irqsave(&domain->lock, flags);
	list_for_each_entry(stream, &domain->streams, stream_head) {
		apple_dart_hw_invalidate_tlb_stream(stream->dart, stream->sid);
	}
	spin_unlock_irqrestore(&domain->lock, flags);
}

static void apple_dart_flush_iotlb_all(struct iommu_domain *domain)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	apple_dart_domain_flush_tlb(dart_domain);
}

static void apple_dart_iotlb_sync(struct iommu_domain *domain,
				  struct iommu_iotlb_gather *gather)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	apple_dart_domain_flush_tlb(dart_domain);
}

static void apple_dart_iotlb_sync_map(struct iommu_domain *domain,
				      unsigned long iova, size_t size)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	apple_dart_domain_flush_tlb(dart_domain);
}

static void apple_dart_tlb_flush_all(void *cookie)
{
	struct apple_dart_domain *domain = cookie;

	apple_dart_domain_flush_tlb(domain);
}

static void apple_dart_tlb_flush_walk(unsigned long iova, size_t size,
				      size_t granule, void *cookie)
{
	struct apple_dart_domain *domain = cookie;

	apple_dart_domain_flush_tlb(domain);
}

static const struct iommu_flush_ops apple_dart_tlb_ops = {
	.tlb_flush_all = apple_dart_tlb_flush_all,
	.tlb_flush_walk = apple_dart_tlb_flush_walk,
	.tlb_add_page = NULL,
};

static phys_addr_t apple_dart_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;

	if (domain->type == IOMMU_DOMAIN_IDENTITY &&
	    dart_domain->dart->supports_bypass)
		return iova;
	if (!ops)
		return -ENODEV;

	return ops->iova_to_phys(ops, iova);
}

static int apple_dart_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t paddr, size_t size, int prot, gfp_t gfp)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;
	if (prot & IOMMU_MMIO)
		return -EINVAL;
	if (prot & IOMMU_NOEXEC)
		return -EINVAL;

	return ops->map(ops, iova, paddr, size, prot, gfp);
}

static size_t apple_dart_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t size, struct iommu_iotlb_gather *gather)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;

	if (!ops)
		return 0;

	return ops->unmap(ops, iova, size, gather);
}

static int apple_dart_prepare_sw_bypass(struct apple_dart *dart,
					struct apple_dart_domain *dart_domain,
					struct device *dev)
{
	lockdep_assert_held(&dart_domain->lock);

	if (dart->supports_bypass)
		return 0;
	if (dart_domain->type != IOMMU_DOMAIN_IDENTITY)
		return 0;

	// use the bus region from the first attached dev for the bypass range
	if (!dart->sw_bypass_len) {
		const struct bus_dma_region *dma_rgn = dev->dma_range_map;

		if (!dma_rgn)
			return -EINVAL;

		dart->sw_bypass_len = dma_rgn->size;
		dart->sw_bypass_cpu_start = dma_rgn->cpu_start;
		dart->sw_bypass_dma_start = dma_rgn->dma_start;
	}

	// ensure that we don't mix different bypass setups
	if (dart_domain->sw_bypass_len) {
		if (dart->sw_bypass_len != dart_domain->sw_bypass_len)
			return -EINVAL;
		if (dart->sw_bypass_cpu_start !=
		    dart_domain->sw_bypass_cpu_start)
			return -EINVAL;
		if (dart->sw_bypass_dma_start !=
		    dart_domain->sw_bypass_dma_start)
			return -EINVAL;
	} else {
		dart_domain->sw_bypass_len = dart->sw_bypass_len;
		dart_domain->sw_bypass_cpu_start = dart->sw_bypass_cpu_start;
		dart_domain->sw_bypass_dma_start = dart->sw_bypass_dma_start;
	}

	return 0;
}

static int apple_dart_domain_needs_pgtbl_ops(struct apple_dart *dart,
					     struct iommu_domain *domain)
{
	if (domain->type == IOMMU_DOMAIN_DMA)
		return 1;
	if (domain->type == IOMMU_DOMAIN_UNMANAGED)
		return 1;
	if (!dart->supports_bypass && domain->type == IOMMU_DOMAIN_IDENTITY)
		return 1;
	return 0;
}

static int apple_dart_finalize_domain(struct iommu_domain *domain)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct apple_dart *dart = dart_domain->dart;
	struct io_pgtable_cfg pgtbl_cfg;

	lockdep_assert_held(&dart_domain->lock);

	if (dart_domain->pgtbl_ops)
		return 0;
	if (!apple_dart_domain_needs_pgtbl_ops(dart, domain))
		return 0;

	pgtbl_cfg = (struct io_pgtable_cfg){
		.pgsize_bitmap = dart->pgsize,
		.ias = 32,
		.oas = 36,
		.coherent_walk = 1,
		.tlb = &apple_dart_tlb_ops,
		.iommu_dev = dart->dev,
	};

	dart_domain->pgtbl_ops =
		alloc_io_pgtable_ops(ARM_APPLE_DART, &pgtbl_cfg, domain);
	if (!dart_domain->pgtbl_ops)
		return -ENOMEM;

	domain->pgsize_bitmap = pgtbl_cfg.pgsize_bitmap;
	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end = DMA_BIT_MASK(32);
	domain->geometry.force_aperture = true;

	/*
	 * Some DARTs come without hardware bypass support but we may still
	 * be forced to use bypass mode (to e.g. allow kernels with 4K pages to
	 * boot). If we reach this point with an identity domain we have to setup
	 * bypass mode in software. This is done by creating a static pagetable
	 * for a linear map specified by dma-ranges in the device tree.
	 */
	if (domain->type == IOMMU_DOMAIN_IDENTITY) {
		u64 offset;
		int ret;

		for (offset = 0; offset < dart_domain->sw_bypass_len;
		     offset += dart->pgsize) {
			ret = dart_domain->pgtbl_ops->map(
				dart_domain->pgtbl_ops,
				dart_domain->sw_bypass_dma_start + offset,
				dart_domain->sw_bypass_cpu_start + offset,
				dart->pgsize, IOMMU_READ | IOMMU_WRITE,
				GFP_ATOMIC);
			if (ret < 0) {
				free_io_pgtable_ops(dart_domain->pgtbl_ops);
				dart_domain->pgtbl_ops = NULL;
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void
apple_dart_stream_setup_translation(struct apple_dart_domain *domain,
				    struct apple_dart *dart, u32 sid)
{
	int i;
	struct io_pgtable_cfg *pgtbl_cfg =
		&io_pgtable_ops_to_pgtable(domain->pgtbl_ops)->cfg;

	for (i = 0; i < pgtbl_cfg->apple_dart_cfg.n_ttbrs; ++i)
		apple_dart_hw_set_ttbr(dart, sid, i,
				       pgtbl_cfg->apple_dart_cfg.ttbr[i]);
	for (; i < DART_MAX_TTBR; ++i)
		apple_dart_hw_clear_ttbr(dart, sid, i);

	apple_dart_hw_enable_translation(dart, sid);
	apple_dart_hw_invalidate_tlb_stream(dart, sid);
}

static int apple_dart_attach_stream(struct apple_dart_domain *domain,
				    struct apple_dart *dart, u32 sid)
{
	unsigned long flags;
	struct apple_dart_stream *stream;
	int ret;

	lockdep_assert_held(&domain->lock);

	if (WARN_ON(dart->force_bypass &&
		    domain->type != IOMMU_DOMAIN_IDENTITY))
		return -EINVAL;

	/*
	 * we can't mix and match DARTs that support bypass mode with those who don't
	 * because the iova space in fake bypass mode generally has an offset
	 */
	if (WARN_ON(domain->type == IOMMU_DOMAIN_IDENTITY &&
		    (domain->dart->supports_bypass != dart->supports_bypass)))
		return -EINVAL;

	list_for_each_entry(stream, &domain->streams, stream_head) {
		if (stream->dart == dart && stream->sid == sid) {
			stream->num_devices++;
			return 0;
		}
	}

	spin_lock_irqsave(&dart->lock, flags);

	if (WARN_ON(dart->used_sids & BIT(sid))) {
		ret = -EINVAL;
		goto error;
	}

	stream = kzalloc(sizeof(*stream), GFP_ATOMIC);
	if (!stream) {
		ret = -ENOMEM;
		goto error;
	}

	stream->dart = dart;
	stream->sid = sid;
	stream->num_devices = 1;
	list_add(&stream->stream_head, &domain->streams);

	dart->used_sids |= BIT(sid);
	spin_unlock_irqrestore(&dart->lock, flags);

	apple_dart_hw_clear_all_ttbrs(stream->dart, stream->sid);

	switch (domain->type) {
	case IOMMU_DOMAIN_IDENTITY:
		if (stream->dart->supports_bypass)
			apple_dart_hw_enable_bypass(stream->dart, stream->sid);
		else
			apple_dart_stream_setup_translation(
				domain, stream->dart, stream->sid);
		break;
	case IOMMU_DOMAIN_BLOCKED:
		apple_dart_hw_disable_dma(stream->dart, stream->sid);
		break;
	case IOMMU_DOMAIN_UNMANAGED:
	case IOMMU_DOMAIN_DMA:
		apple_dart_stream_setup_translation(domain, stream->dart,
						    stream->sid);
		break;
	}

	return 0;

error:
	spin_unlock_irqrestore(&dart->lock, flags);
	return ret;
}

static void apple_dart_disable_stream(struct apple_dart *dart, u32 sid)
{
	unsigned long flags;

	apple_dart_hw_disable_dma(dart, sid);
	apple_dart_hw_clear_all_ttbrs(dart, sid);
	apple_dart_hw_invalidate_tlb_stream(dart, sid);

	spin_lock_irqsave(&dart->lock, flags);
	dart->used_sids &= ~BIT(sid);
	spin_unlock_irqrestore(&dart->lock, flags);
}

static void apple_dart_detach_stream(struct apple_dart_domain *domain,
				     struct apple_dart *dart, u32 sid)
{
	struct apple_dart_stream *stream;

	lockdep_assert_held(&domain->lock);

	list_for_each_entry(stream, &domain->streams, stream_head) {
		if (stream->dart == dart && stream->sid == sid) {
			stream->num_devices--;

			if (stream->num_devices == 0) {
				apple_dart_disable_stream(dart, sid);
				list_del(&stream->stream_head);
				kfree(stream);
			}
			return;
		}
	}
}

static int apple_dart_attach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	int ret;
	int i, j;
	unsigned long flags;
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct apple_dart *dart = cfg->streams[0].dart;

	if (WARN_ON(dart->force_bypass &&
		    dart_domain->type != IOMMU_DOMAIN_IDENTITY)) {
		dev_warn(
			dev,
			"IOMMU must be in bypass mode but trying to attach to translated domain.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&dart_domain->lock, flags);

	ret = apple_dart_prepare_sw_bypass(dart, dart_domain, dev);
	if (ret)
		goto out;

	if (!dart_domain->dart)
		dart_domain->dart = dart;

	ret = apple_dart_finalize_domain(domain);
	if (ret)
		goto out;

	for (i = 0; i < cfg->num_streams; ++i) {
		ret = apple_dart_attach_stream(
			dart_domain, cfg->streams[i].dart, cfg->streams[i].sid);
		if (ret) {
			/* try to undo what we did before returning */
			for (j = 0; j < i; ++j)
				apple_dart_detach_stream(dart_domain,
							 cfg->streams[j].dart,
							 cfg->streams[j].sid);

			goto out;
		}
	}

	ret = 0;

out:
	spin_unlock_irqrestore(&dart_domain->lock, flags);
	return ret;
}

static void apple_dart_detach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	int i;
	unsigned long flags;
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	spin_lock_irqsave(&dart_domain->lock, flags);

	for (i = 0; i < cfg->num_streams; ++i)
		apple_dart_detach_stream(dart_domain, cfg->streams[i].dart,
					 cfg->streams[i].sid);

	spin_unlock_irqrestore(&dart_domain->lock, flags);
}

static struct iommu_device *apple_dart_probe_device(struct device *dev)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	int i;

	if (!cfg)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < cfg->num_streams; ++i) {
		cfg->streams[i].link =
			device_link_add(dev, cfg->streams[i].dart->dev,
					DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	}

	return &cfg->streams[0].dart->iommu;
}

static void apple_dart_release_device(struct device *dev)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	int i;

	if (!cfg)
		return;

	for (i = 0; i < cfg->num_streams; ++i)
		device_link_del(cfg->streams[i].link);

	dev_iommu_priv_set(dev, NULL);
	kfree(cfg);
}

static struct iommu_domain *apple_dart_domain_alloc(unsigned int type)
{
	struct apple_dart_domain *dart_domain;

	if (type != IOMMU_DOMAIN_DMA && type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_IDENTITY && type != IOMMU_DOMAIN_BLOCKED)
		return NULL;

	dart_domain = kzalloc(sizeof(*dart_domain), GFP_KERNEL);
	if (!dart_domain)
		return NULL;

	INIT_LIST_HEAD(&dart_domain->streams);
	spin_lock_init(&dart_domain->lock);
	iommu_get_dma_cookie(&dart_domain->domain);
	dart_domain->type = type;

	return &dart_domain->domain;
}

static void apple_dart_domain_free(struct iommu_domain *domain)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	WARN_ON(!list_empty(&dart_domain->streams));

	kfree(dart_domain);
}

static int apple_dart_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	struct platform_device *iommu_pdev = of_find_device_by_node(args->np);
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	unsigned int num_streams = cfg ? cfg->num_streams : 0;
	struct apple_dart_master_cfg *cfg_new;
	struct apple_dart *dart = platform_get_drvdata(iommu_pdev);

	if (args->args_count != 1)
		return -EINVAL;

	cfg_new = krealloc(cfg, struct_size(cfg, streams, num_streams + 1),
			   GFP_KERNEL);
	if (!cfg_new)
		return -ENOMEM;

	cfg = cfg_new;
	dev_iommu_priv_set(dev, cfg);

	cfg->num_streams = num_streams;
	cfg->streams[cfg->num_streams].dart = dart;
	cfg->streams[cfg->num_streams].sid = args->args[0];
	cfg->num_streams++;

	return 0;
}

static struct iommu_group *apple_dart_device_group(struct device *dev)
{
#ifdef CONFIG_PCI
	struct iommu_group *group;

	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
#else
	return generic_device_group(dev);
#endif
}

static int apple_dart_def_domain_type(struct device *dev)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart *dart = cfg->streams[0].dart;

	if (dart->force_bypass)
		return IOMMU_DOMAIN_IDENTITY;
	if (!dart->supports_bypass)
		return IOMMU_DOMAIN_DMA;

	return 0;
}

static const struct iommu_ops apple_dart_iommu_ops = {
	.domain_alloc = apple_dart_domain_alloc,
	.domain_free = apple_dart_domain_free,
	.attach_dev = apple_dart_attach_dev,
	.detach_dev = apple_dart_detach_dev,
	.map = apple_dart_map,
	.unmap = apple_dart_unmap,
	.flush_iotlb_all = apple_dart_flush_iotlb_all,
	.iotlb_sync = apple_dart_iotlb_sync,
	.iotlb_sync_map = apple_dart_iotlb_sync_map,
	.iova_to_phys = apple_dart_iova_to_phys,
	.probe_device = apple_dart_probe_device,
	.release_device = apple_dart_release_device,
	.device_group = apple_dart_device_group,
	.of_xlate = apple_dart_of_xlate,
	.def_domain_type = apple_dart_def_domain_type,
	.pgsize_bitmap = -1UL, /* Restricted during dart probe */
};

static irqreturn_t apple_dart_irq(int irq, void *dev)
{
	struct apple_dart *dart = dev;
	static DEFINE_RATELIMIT_STATE(rs, DEFAULT_RATELIMIT_INTERVAL,
				      DEFAULT_RATELIMIT_BURST);
	const char *fault_name = NULL;
	u32 error = readl(dart->regs + DART_ERROR);
	u32 error_code = FIELD_GET(DART_ERROR_CODE, error);
	u32 addr_lo = readl(dart->regs + DART_ERROR_ADDR_LO);
	u32 addr_hi = readl(dart->regs + DART_ERROR_ADDR_HI);
	u64 addr = addr_lo | (((u64)addr_hi) << 32);
	u8 stream_idx = FIELD_GET(DART_ERROR_STREAM, error);

	if (!(error & DART_ERROR_FLAG))
		return IRQ_NONE;

	if (error_code & DART_ERROR_READ_FAULT)
		fault_name = "READ FAULT";
	else if (error_code & DART_ERROR_WRITE_FAULT)
		fault_name = "WRITE FAULT";
	else if (error_code & DART_ERROR_NO_PTE)
		fault_name = "NO PTE FOR IOVA";
	else if (error_code & DART_ERROR_NO_PMD)
		fault_name = "NO PMD FOR IOVA";
	else if (error_code & DART_ERROR_NO_TTBR)
		fault_name = "NO TTBR FOR IOVA";

	if (WARN_ON(fault_name == NULL))
		fault_name = "unknown";

	if (__ratelimit(&rs)) {
		dev_err(dart->dev,
			"translation fault: status:0x%x stream:%d code:0x%x (%s) at 0x%llx",
			error, stream_idx, error_code, fault_name, addr);
	}

	writel(error, dart->regs + DART_ERROR);
	return IRQ_HANDLED;
}

static int apple_dart_probe(struct platform_device *pdev)
{
	int ret;
	u32 dart_params[2];
	struct resource *res;
	struct apple_dart *dart;
	struct device *dev = &pdev->dev;

	dart = devm_kzalloc(dev, sizeof(*dart), GFP_KERNEL);
	if (!dart)
		return -ENOMEM;

	dart->dev = dev;
	spin_lock_init(&dart->lock);

	if (pdev->num_resources < 1)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource_size(res) < 0x4000) {
		dev_err(dev, "MMIO region too small (%pr)\n", res);
		return -EINVAL;
	}

	dart->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(dart->regs))
		return PTR_ERR(dart->regs);

	ret = devm_clk_bulk_get_all(dev, &dart->clks);
	if (ret < 0)
		return ret;
	dart->num_clks = ret;

	ret = clk_bulk_prepare_enable(dart->num_clks, dart->clks);
	if (ret)
		return ret;

	ret = apple_dart_hw_reset(dart);
	if (ret)
		goto err_clk_disable;

	dart_params[0] = readl(dart->regs + DART_PARAMS1);
	dart_params[1] = readl(dart->regs + DART_PARAMS2);
	dart->pgsize = 1 << FIELD_GET(DART_PARAMS_PAGE_SHIFT, dart_params[0]);
	dart->supports_bypass = dart_params[1] & DART_PARAMS_BYPASS_SUPPORT;
	dart->force_bypass = dart->pgsize > PAGE_SIZE;

	dart->irq = platform_get_irq(pdev, 0);
	if (dart->irq < 0) {
		ret = -ENODEV;
		goto err_clk_disable;
	}

	ret = devm_request_irq(dart->dev, dart->irq, apple_dart_irq,
			       IRQF_SHARED, "apple-dart fault handler", dart);
	if (ret)
		goto err_clk_disable;

	platform_set_drvdata(pdev, dart);

	ret = iommu_device_sysfs_add(&dart->iommu, dev, NULL, "apple-dart.%s",
				     dev_name(&pdev->dev));
	if (ret)
		goto err_clk_disable;

	ret = iommu_device_register(&dart->iommu, &apple_dart_iommu_ops, dev);
	if (ret)
		goto err_clk_disable;

	if (dev->bus->iommu_ops != &apple_dart_iommu_ops) {
		ret = bus_set_iommu(dev->bus, &apple_dart_iommu_ops);
		if (ret)
			goto err_clk_disable;
	}
#ifdef CONFIG_PCI
	if (dev->bus->iommu_ops != pci_bus_type.iommu_ops) {
		ret = bus_set_iommu(&pci_bus_type, &apple_dart_iommu_ops);
		if (ret)
			goto err_clk_disable;
	}
#endif

	dev_info(
		&pdev->dev,
		"DART [pagesize %x, bypass support: %d, bypass forced: %d] initialized\n",
		dart->pgsize, dart->supports_bypass, dart->force_bypass);
	return 0;

err_clk_disable:
	clk_bulk_disable(dart->num_clks, dart->clks);
	clk_bulk_unprepare(dart->num_clks, dart->clks);

	return ret;
}

static int apple_dart_remove(struct platform_device *pdev)
{
	struct apple_dart *dart = platform_get_drvdata(pdev);

	devm_free_irq(dart->dev, dart->irq, dart);

	iommu_device_unregister(&dart->iommu);
	iommu_device_sysfs_remove(&dart->iommu);

	clk_bulk_disable(dart->num_clks, dart->clks);
	clk_bulk_unprepare(dart->num_clks, dart->clks);

	return 0;
}

static void apple_dart_shutdown(struct platform_device *pdev)
{
	apple_dart_remove(pdev);
}

static const struct of_device_id apple_dart_of_match[] = {
	{ .compatible = "apple,t8103-dart", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, apple_dart_of_match);

static struct platform_driver apple_dart_driver = {
	.driver	= {
		.name			= "apple-dart",
		.of_match_table		= apple_dart_of_match,
	},
	.probe	= apple_dart_probe,
	.remove	= apple_dart_remove,
	.shutdown = apple_dart_shutdown,
};
module_platform_driver(apple_dart_driver);

MODULE_DESCRIPTION("IOMMU API for Apple's DART");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("GPL v2");
