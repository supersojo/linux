/*
 * Atmel AT91 AIC (Advanced Interrupt Controller) driver
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/bitmap.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <asm/exception.h>
#include <asm/mach/irq.h>

#include "irq-meson-intc-common.h"

/* Number of irq lines managed by MESON INTC */
#define NR_MESON_INTC_IRQS 128

#define MESON_INTC_STAT_REG(n)	((n)*16 + 0)
#define MESON_INTC_CLR_REG(n)	((n)*16 + 4)
#define MESON_INTC_MASK_REG(n)	((n)*16 + 8)
#define MESON_INTC_FIRQ_SEL_REG(n)	((n)*16 + 12)


extern void printascii(const char*);
extern void printhex8(unsigned int v);
static struct irq_domain *meson_intc_domain;


static void dump_intc_regs(void)
{
	u32 val;
	int i;
	struct irq_domain_chip_generic *dgc = meson_intc_domain->gc;
        struct irq_chip_generic *gc = dgc->gc[0];

	for(i=0;i<4;i++) {
		printk("<----------------\n");
		val = irq_reg_readl(gc, MESON_INTC_MASK_REG(i));
		printk("mask reg 0x%08x\n",val);
		val = irq_reg_readl(gc, MESON_INTC_STAT_REG(i));
		printk("stat reg 0x%08x\n",val);
		val = irq_reg_readl(gc, MESON_INTC_CLR_REG(i));
		printk("clr reg 0x%08x\n",val);
		val = irq_reg_readl(gc, MESON_INTC_FIRQ_SEL_REG(i));
		printk("firq reg 0x%08x\n",val);
		printk("---------------->\n");
	}
}

extern void printch(char);
static void meson_intc_irq_ack(struct irq_data *d)
{
	u32 val;
	u32 idx;

	struct irq_domain_chip_generic *dgc = meson_intc_domain->gc;
        struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);

	idx = d->hwirq / dgc->irqs_per_chip;

        irq_gc_lock(gc);

	val = 1<<(d->hwirq % dgc->irqs_per_chip);
        irq_reg_writel(gc, val, MESON_INTC_CLR_REG(idx));

        irq_gc_unlock(gc);

}
static void meson_intc_irq_mask(struct irq_data *d)
{
	u32 val;
	u32 idx;

	struct irq_domain_chip_generic *dgc = meson_intc_domain->gc;
        struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);

	idx = d->hwirq / dgc->irqs_per_chip;

        irq_gc_lock(gc);

	val = irq_reg_readl(gc, MESON_INTC_MASK_REG(idx));
	val &= ~(1<<(d->hwirq % dgc->irqs_per_chip));
        irq_reg_writel(gc, val, MESON_INTC_MASK_REG(idx));

        irq_gc_unlock(gc);


}
static void meson_intc_irq_unmask(struct irq_data *d)
{
	u32 val;
	u32 idx;

	struct irq_domain_chip_generic *dgc = meson_intc_domain->gc;
        struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);

	idx = d->hwirq / dgc->irqs_per_chip;

        irq_gc_lock(gc);

	val = irq_reg_readl(gc, MESON_INTC_MASK_REG(idx));
	val |= 1<<(d->hwirq % dgc->irqs_per_chip);
        irq_reg_writel(gc, val, MESON_INTC_MASK_REG(idx));

        irq_gc_unlock(gc);

}
extern void printch(char);
extern void printhex8(u32);
static asmlinkage void __exception_irq_entry
meson_intc_handle(struct pt_regs *regs)
{
	struct irq_domain_chip_generic *dgc = meson_intc_domain->gc;
	struct irq_chip_generic *gc = dgc->gc[0];
	u32 irqnr;
	u32 irqstat;
	u32 val;
	int chip;
	int i;

	/* meson spec implementation */
	for(chip=0; chip<4; chip++) {
		irqstat = irq_reg_readl(gc, MESON_INTC_STAT_REG(chip));
		/* get unmasked irqs */
		val = ~irq_reg_readl(gc, MESON_INTC_FIRQ_SEL_REG(chip));
		irqstat &= val;

		irqnr = chip*32;

		i = 0;
		do {
			if ((irqstat>>i)&0x1) {

				handle_domain_irq(meson_intc_domain, irqnr+i, regs);
				return;
			}

			i++;

		}while(i<=31);
	}
}

static void __init meson_intc_hw_init(struct irq_domain *domain)
{
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(domain, 0);
	int i;

	/* meson spec implementation */
	for (i=0; i<4; i++) {
		/* disable all irqs */
		irq_reg_writel(gc, 0, MESON_INTC_MASK_REG(i));

		/* clear all irqs */
		irq_reg_writel(gc, 0xffffffff, MESON_INTC_CLR_REG(i));

		/* set all interrupts to irq */
		irq_reg_writel(gc, 0, MESON_INTC_FIRQ_SEL_REG(i));
	}
}

static int meson_intc_irq_domain_xlate(struct irq_domain *d,
				struct device_node *ctrlr,
				const u32 *intspec, unsigned int intsize,
				irq_hw_number_t *out_hwirq,
				unsigned int *out_type)
{
	struct irq_domain_chip_generic *dgc = d->gc;
	struct irq_chip_generic *gc;
	unsigned long flags;
	unsigned mask;
	int idx;
	int ret;

	if (!dgc)
		return -EINVAL;

	ret = meson_intc_common_irq_domain_xlate(d, ctrlr, intspec, intsize,
					  out_hwirq, out_type);
	if (ret)
		return ret;

	printhex8(dgc->irqs_per_chip);
	idx = intspec[0] / dgc->irqs_per_chip;
	if (idx >= dgc->num_chips)
		return -EINVAL;

	gc = dgc->gc[idx];


	/* set irq priority and sensitivity if controller support */
#if 0
	irq_gc_lock_irqsave(gc, flags);

	/* enable irq */
	//mask = irq_reg_readl(gc, MESON_INTC_MASK_REG(idx));
	//mask |= 1<<(*out_hwirq % dgc->irqs_per_chip);
	//irq_reg_writel(gc, mask, MESON_INTC_MASK_REG(idx));

	irq_gc_unlock_irqrestore(gc, flags);
#endif
	return ret;
}

static const struct irq_domain_ops meson_intc_irq_ops = {
	.map	= irq_map_generic_chip,
	.xlate	= meson_intc_irq_domain_xlate,
};

static const struct of_device_id meson_intc_ids[] __initconst = {
	{ .compatible = "amlogic,meson-intc", .data = NULL },
	{ /* sentinel */ },
};

static int __init meson_intc_of_init(struct device_node *node,
			      struct device_node *parent)
{
	int i;
	struct irq_chip_generic *gc;
	struct irq_domain *domain;

	if (meson_intc_domain)
		return -EEXIST;

	domain = meson_intc_common_of_init(node, &meson_intc_irq_ops, "meson-intc",
				    NR_MESON_INTC_IRQS, meson_intc_ids);
	if (IS_ERR(domain))
		return PTR_ERR(domain);

	meson_intc_domain = domain;

	for (i = 0; i < 4; i++) {
                gc = irq_get_domain_generic_chip(domain, i * 32);

		gc->chip_types[0].chip.irq_eoi = meson_intc_irq_ack;
		gc->chip_types[0].chip.irq_ack = meson_intc_irq_ack;
		gc->chip_types[0].chip.irq_mask = meson_intc_irq_mask;
		gc->chip_types[0].chip.irq_unmask = meson_intc_irq_unmask;
	}

	meson_intc_hw_init(domain);
	set_handle_irq(meson_intc_handle);

	return 0;
}
IRQCHIP_DECLARE(meson_intc, "amlogic,meson-intc", meson_intc_of_init);
