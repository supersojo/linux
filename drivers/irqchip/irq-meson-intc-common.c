/*
 * Atmel AT91 common AIC (Advanced Interrupt Controller) code shared by
 * irq-atmel-aic and irq-atmel-aic5 drivers
 *
 *  Copyright (C) 2004 SAN People
 *  Copyright (C) 2004 ATMEL
 *  Copyright (C) Rick Bronson
 *  Copyright (C) 2014 Free Electrons
 *
 *  Author: Boris BREZILLON <boris.brezillon@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "irq-meson-intc-common.h"

#if 0
#define AT91_AIC_PRIOR			GENMASK(2, 0)
#define AT91_AIC_IRQ_MIN_PRIORITY	0
#define AT91_AIC_IRQ_MAX_PRIORITY	7

#define AT91_AIC_SRCTYPE		GENMASK(6, 5)
#define AT91_AIC_SRCTYPE_LOW		(0 << 5)
#define AT91_AIC_SRCTYPE_FALLING	(1 << 5)
#define AT91_AIC_SRCTYPE_HIGH		(2 << 5)
#define AT91_AIC_SRCTYPE_RISING		(3 << 5)
#endif
#if 0
struct aic_chip_data {
	u32 ext_irqs;
};
#endif

struct meson_intc_chip_data {
	/* meson spec implementation */
};

static void meson_intc_common_shutdown(struct irq_data *d)
{
	struct irq_chip_type *ct = irq_data_get_chip_type(d);

	ct->chip.irq_mask(d);
}

int meson_intc_common_set_type(struct irq_data *d, unsigned type, unsigned *val)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct aic_chip_data *aic = gc->private;
	unsigned meson_intc_type;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		break;
	case IRQ_TYPE_EDGE_RISING:
		break;
	case IRQ_TYPE_LEVEL_LOW:
		break;
	case IRQ_TYPE_EDGE_FALLING:
		break;
	default:
		return -EINVAL;
	}

#if 0
	*val &= ~AT91_AIC_SRCTYPE;
	*val |= aic_type;
#endif
	/* meson spec implementation */
	return 0;
}


int meson_intc_common_irq_domain_xlate(struct irq_domain *d,
				struct device_node *ctrlr,
				const u32 *intspec,
				unsigned int intsize,
				irq_hw_number_t *out_hwirq,
				unsigned int *out_type)
{
	if (WARN_ON(intsize < 2))
		return -EINVAL;

	*out_hwirq = intspec[0];
	*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static void __init meson_intc_common_ext_irq_of_init(struct irq_domain *domain)
{
	struct device_node *node = irq_domain_get_of_node(domain);
	struct irq_chip_generic *gc;
	struct meson_int_chip_data *meson_intc;
	struct property *prop;
	const __be32 *p;
	u32 hwirq;

	gc = irq_get_domain_generic_chip(domain, 0);

	meson_intc = gc->private;

}


struct irq_domain *__init meson_intc_common_of_init(struct device_node *node,
					     const struct irq_domain_ops *ops,
					     const char *name, int nirqs,
					     const struct of_device_id *matches)
{
	struct irq_chip_generic *gc;
	struct irq_domain *domain;
	struct meson_intc_chip_data *meson_intc;
	void __iomem *reg_base;
	int nchips;
	int ret;
	int i;

	nchips = DIV_ROUND_UP(nirqs, 32);

	reg_base = of_iomap(node, 0);
	if (!reg_base)
		return ERR_PTR(-ENOMEM);
	printk("meson_intc reg_base 0x%08x\n",reg_base);

	meson_intc = kcalloc(nchips, sizeof(*meson_intc), GFP_KERNEL);
	if (!meson_intc) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	domain = irq_domain_add_linear(node, nchips * 32, ops, meson_intc);
	if (!domain) {
		ret = -ENOMEM;
		goto err_free_meson_intc;
	}

	ret = irq_alloc_domain_generic_chips(domain, 32, 1, name,
					     handle_fasteoi_irq,
					     IRQ_NOREQUEST | IRQ_NOPROBE |
					     IRQ_NOAUTOEN, 0, 0);
	if (ret)
		goto err_domain_remove;

	for (i = 0; i < nchips; i++) {
		gc = irq_get_domain_generic_chip(domain, i * 32);

		gc->reg_base = reg_base;
		printk("meson_intc reg_base 0x%08x\n", reg_base);

		gc->unused = 0;
		gc->wake_enabled = ~0;
		gc->chip_types[0].type = IRQ_TYPE_SENSE_MASK;
		gc->chip_types[0].chip.irq_set_wake = irq_gc_set_wake;
		gc->chip_types[0].chip.irq_shutdown = meson_intc_common_shutdown;
		gc->private = &meson_intc[i];
	}

	return domain;

err_domain_remove:
	irq_domain_remove(domain);

err_free_meson_intc:
	kfree(meson_intc);

err_iounmap:
	iounmap(reg_base);

	return ERR_PTR(ret);
}
