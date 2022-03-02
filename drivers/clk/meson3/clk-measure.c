#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>

/* default regmap config template */
static const struct regmap_config syscon_regmap_config = {
    .reg_bits = 32,
    .val_bits = 32,
    .reg_stride = 4,
};

static struct regmap *clock_measure_regmap;

static int meson_clock_measure_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct resource res;
    void __iomem *base;
    struct regmap *regmap;
    int ret = 0;

    struct regmap_config syscon_config = syscon_regmap_config;

    if (of_address_to_resource(dev->of_node, 0, &res)) {
        ret = -ENOMEM;
        goto err_map;
    }

    base = of_iomap(dev->of_node, 0);
    if (!base) {
        ret = -ENOMEM;
        goto err_map;
    }
    syscon_config.name = kasprintf(GFP_KERNEL, "%pOFn@%llx", dev->of_node,
                       (u64)res.start);/* kmalloc */
    syscon_config.reg_stride = 4;
    syscon_config.val_bits = 4 * 8;
    syscon_config.max_register = resource_size(&res) - 4;
    regmap = regmap_init_mmio(NULL, base, &syscon_config);
    kfree(syscon_config.name);
    if (IS_ERR(regmap)) {
        pr_err("regmap init failed\n");
        ret = PTR_ERR(regmap);
        goto err_regmap;
    }

    clock_measure_regmap = regmap;

err_regmap:
err_map:
    return ret;
}
enum {
    MSR_CLK_DUTY = 0,
    MSR_CLK_REG0 = 4,
    MSR_CLK_REG1 = 8,
    MSR_CLK_REG2 = 12,
};
u32 clk_util_clk_msr(u32 clk_mux)
{
    u32 v;

    regmap_write(clock_measure_regmap, MSR_CLK_REG0, 0);

    regmap_clear_bits(clock_measure_regmap, MSR_CLK_REG0, 0xffff);

    regmap_update_bits(clock_measure_regmap, MSR_CLK_REG0, 0xffff, (64 - 1));/* 64 us */

    regmap_clear_bits(clock_measure_regmap, MSR_CLK_REG0, 0x3<<17);

    regmap_clear_bits(clock_measure_regmap, MSR_CLK_REG0, 0x1f<<20);

    regmap_set_bits(clock_measure_regmap, MSR_CLK_REG0, clk_mux<<20|1<<16|1<<19);

    regmap_read_poll_timeout(clock_measure_regmap, MSR_CLK_REG0, v, !(v&(1<<31)), 100/* 100us */, 1000000/* 1s */);

    regmap_clear_bits(clock_measure_regmap, MSR_CLK_REG0, 1<<16);

    regmap_read(clock_measure_regmap, MSR_CLK_REG2, &v);
    v = (v+31)&0xffff;
    v>>=6;

    return v;
}

static const struct of_device_id meson_clock_measure_id[] = {
    { .compatible = "amlogic, meson-clock-measure" },
    { }
};
MODULE_DEVICE_TABLE(of, meson_clock_measure_id);

static struct platform_driver meson_clock_measure_driver = {
    .driver = {
        .name = "meson-clock-measure",
        .of_match_table = meson_clock_measure_id,
    },
    .probe = meson_clock_measure_probe,
};
module_platform_driver(meson_clock_measure_driver);

