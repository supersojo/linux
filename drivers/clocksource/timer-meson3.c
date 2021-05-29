// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic Meson 8726M3 SoCs timer handling.
 *
 * Copyright (C) 2014 Carlo Caione <carlo@caione.org>
 *
 * Based on code from Amlogic, Inc
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sched_clock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#ifdef CONFIG_ARM
#include <linux/delay.h>
#endif

#define MESON_ISA_TIMER_MUX					0x00
#define MESON_ISA_TIMER_MUX_TIMERD_EN				BIT(19)
#define MESON_ISA_TIMER_MUX_TIMERC_EN				BIT(18)
#define MESON_ISA_TIMER_MUX_TIMERB_EN				BIT(17)
#define MESON_ISA_TIMER_MUX_TIMERA_EN				BIT(16)
#define MESON_ISA_TIMER_MUX_TIMERD_MODE				BIT(15)
#define MESON_ISA_TIMER_MUX_TIMERC_MODE				BIT(14)
#define MESON_ISA_TIMER_MUX_TIMERB_MODE				BIT(13)
#define MESON_ISA_TIMER_MUX_TIMERA_MODE				BIT(12)
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK		GENMASK(10, 8)
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_SYSTEM_CLOCK	0x0
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_1US		0x1
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_10US		0x2
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_100US		0x3
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_1MS		0x4
#define MESON_ISA_TIMER_MUX_TIMERD_INPUT_CLOCK_MASK		GENMASK(7, 6)
#define MESON_ISA_TIMER_MUX_TIMERC_INPUT_CLOCK_MASK		GENMASK(5, 4)
#define MESON_ISA_TIMER_MUX_TIMERB_INPUT_CLOCK_MASK		GENMASK(3, 2)
#define MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK		GENMASK(1, 0)
#define MESON_ISA_TIMER_MUX_TIMERABCD_INPUT_CLOCK_1US		0x0
#define MESON_ISA_TIMER_MUX_TIMERABCD_INPUT_CLOCK_10US		0x1
#define MESON_ISA_TIMER_MUX_TIMERABCD_INPUT_CLOCK_100US		0x0
#define MESON_ISA_TIMER_MUX_TIMERABCD_INPUT_CLOCK_1MS		0x3

#define MESON_ISA_TIMERA					0x04
#define MESON_ISA_TIMERB					0x08
#define MESON_ISA_TIMERC					0x0c
#define MESON_ISA_TIMERD					0x10
#define MESON_ISA_TIMERE					0x14

static void __iomem *timer_base;

static void dump_timer_regs(void)
{
	u32 val;
	
	printk("<----------\n");
	val = readl(timer_base + MESON_ISA_TIMER_MUX);
	printk("mux reg 0x%08x\n",val);
	val = readl(timer_base + MESON_ISA_TIMERA);
	printk("A reg 0x%08x\n",val);
	val = readl(timer_base + MESON_ISA_TIMERB);
	printk("B reg 0x%08x\n",val);
	val = readl(timer_base + MESON_ISA_TIMERC);
	printk("C reg 0x%08x\n",val);
	val = readl(timer_base + MESON_ISA_TIMERD);
	printk("D reg 0x%08x\n",val);
	val = readl(timer_base + MESON_ISA_TIMERE);
	printk("E reg 0x%08x\n",val);
	printk("---------->\n");
}

#ifdef CONFIG_ARM
static unsigned long meson3_read_current_timer(void)
{
	return readl_relaxed(timer_base + MESON_ISA_TIMERE);
}

static struct delay_timer meson3_delay_timer = {
	.read_current_timer = meson3_read_current_timer,
	.freq = 1000 * 1000,
};
#endif

static u64 notrace meson3_timer_sched_read(void)
{
	return (u64)readl(timer_base + MESON_ISA_TIMERE);
}

static void meson3_clkevt_time_stop(void)
{
	u32 val = readl(timer_base + MESON_ISA_TIMER_MUX);

	writel(val & ~MESON_ISA_TIMER_MUX_TIMERA_EN,
	       timer_base + MESON_ISA_TIMER_MUX);
}

static void meson3_clkevt_time_setup(unsigned long delay)
{
	writel(delay, timer_base + MESON_ISA_TIMERA);
}

static void meson3_clkevt_time_start(bool periodic)
{
	u32 val = readl(timer_base + MESON_ISA_TIMER_MUX);

	if (periodic)
		val |= MESON_ISA_TIMER_MUX_TIMERA_MODE;
	else
		val &= ~MESON_ISA_TIMER_MUX_TIMERA_MODE;

	writel(val | MESON_ISA_TIMER_MUX_TIMERA_EN,
	       timer_base + MESON_ISA_TIMER_MUX);
}

static int meson3_shutdown(struct clock_event_device *evt)
{
	meson3_clkevt_time_stop();
	return 0;
}

static int meson3_set_oneshot(struct clock_event_device *evt)
{
	meson3_clkevt_time_stop();
	meson3_clkevt_time_start(false);
	return 0;
}

static int meson3_set_periodic(struct clock_event_device *evt)
{
	meson3_clkevt_time_stop();
	meson3_clkevt_time_setup(USEC_PER_SEC / HZ - 1);
	meson3_clkevt_time_start(true);
	return 0;
}

static int meson3_clkevt_next_event(unsigned long evt,
				    struct clock_event_device *unused)
{
	meson3_clkevt_time_stop();
	meson3_clkevt_time_setup(evt);
	meson3_clkevt_time_start(false);

	return 0;
}

static struct clock_event_device meson3_clockevent = {
	.name			= "meson3_tick",
	.rating			= 400,
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown	= meson3_shutdown,
	.set_state_periodic	= meson3_set_periodic,
	.set_state_oneshot	= meson3_set_oneshot,
	.tick_resume		= meson3_shutdown,
	.set_next_event		= meson3_clkevt_next_event,
};

extern void printch(char);
static irqreturn_t meson3_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

extern void printascii(const char*);

static int __init meson3_timer_init(struct device_node *node)
{
	u32 val;
	int ret, irq;
	printascii("meson3_timer_init 000\r\n");
	timer_base = of_io_request_and_map(node, 0, "meson3-timer");
	if (IS_ERR(timer_base)) {
		pr_err("Can't map registers\n");
		return -ENXIO;
	}

	printascii("meson3_timer_init 111\r\n");
	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0) {
		pr_err("Can't parse IRQ\n");
		return -EINVAL;
	}

	/* Set 1us for timer E */
	val = readl(timer_base + MESON_ISA_TIMER_MUX);
	val &= ~MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK;
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK,
			  MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_1US);
	writel(val, timer_base + MESON_ISA_TIMER_MUX);

	printascii("meson3_timer_init 222\r\n");
	sched_clock_register(meson3_timer_sched_read, 32, USEC_PER_SEC);
	clocksource_mmio_init(timer_base + MESON_ISA_TIMERE, node->name,
			      1000 * 1000, 300, 32, clocksource_mmio_readl_up);

	/* Timer A base 1us */
	val &= ~MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK;
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK,
			  MESON_ISA_TIMER_MUX_TIMERABCD_INPUT_CLOCK_1US);
	writel(val, timer_base + MESON_ISA_TIMER_MUX);

	/* Stop the timer A */
	meson3_clkevt_time_stop();

	printascii("meson3_timer_init 333\r\n");
	ret = request_irq(irq, meson3_timer_interrupt,
			  IRQF_TIMER | IRQF_IRQPOLL, "meson3_timer",
			  &meson3_clockevent);
	if (ret) {
		pr_warn("failed to setup irq %d\n", irq);
		return ret;
	}

	printascii("meson3_timer_init 444\r\n");
	meson3_clockevent.cpumask = cpu_possible_mask;
	meson3_clockevent.irq = irq;

	clockevents_config_and_register(&meson3_clockevent, USEC_PER_SEC,
					1, 0xfffe);

#ifdef CONFIG_ARM
	/* Also use MESON_ISA_TIMERE for delays */
	register_current_timer_delay(&meson3_delay_timer);
#endif

	printascii("meson3_timer_init 555\r\n");
	printk("meson3_timer_init called\n");
	dump_timer_regs();
	printascii("meson3_timer_init 666\r\n");

	return 0;
}
TIMER_OF_DECLARE(meson3, "amlogic,meson3-timer",
		       meson3_timer_init);
