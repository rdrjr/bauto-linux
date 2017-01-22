/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 * Copyright 2014 PHYTEC Technologie Holding AG.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

// #include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/wl12xx.h>
#include <linux/smsc911x.h>
#include <linux/delay.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

/* This is for a gpr init quirk for ID pin, doesn't exist in 3.14 kernel */
#define IMX_OTG_ID_QUIRK 1

#ifdef CONFIG_SMSC911X
# define EIM_CS0_BASE_ADDR 0x8000000
# define EIM_CS1_BASE_ADDR EIM_CS0_BASE_ADDR + 0x02000000
static struct resource smsc911x_cs0_resources[] = {
	{
        /* TODO pull this out of the dt */
        .start        = EIM_CS0_BASE_ADDR,
        .end          = EIM_CS0_BASE_ADDR + SZ_32M - 1,
        .flags        = IORESOURCE_MEM,
    }, {
        /* irq number is run-time assigned */
        .flags        = IORESOURCE_IRQ,
    },
};
static struct resource smsc911x_cs1_resources[] = {
	{
        /* TODO pull this out of the dt during init */
        .start        = EIM_CS1_BASE_ADDR,
        .end          = EIM_CS1_BASE_ADDR + SZ_32M - 1,
        .flags        = IORESOURCE_MEM,
    }, {
        /* irq number is run-time assigned */
        .flags        = IORESOURCE_IRQ,
    },
};

/* NOTE: These settings should be derrived from dt entry as well, but here for completeness */
/* Entry duplicated because mac address is pushed into this structure potentially */
static struct smsc911x_platform_config smsc911x_cs0_info = {
	.flags		    = SMSC911X_USE_16BIT 
                      | SMSC911X_FORCE_INTERNAL_PHY 
                      | SMSC911X_SAVE_MAC_ADDRESS,
    .irq_polarity    = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
    .irq_type        = SMSC911X_IRQ_TYPE_PUSH_PULL,
    .phy_interface    = PHY_INTERFACE_MODE_MII,
    .shift = 1, /* FIFO Select is bit 0 */
};
static struct smsc911x_platform_config smsc911x_cs1_info = {
	.flags		    = SMSC911X_USE_16BIT 
                      | SMSC911X_FORCE_INTERNAL_PHY 
                      | SMSC911X_SAVE_MAC_ADDRESS,
    .irq_polarity    = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
    .irq_type        = SMSC911X_IRQ_TYPE_PUSH_PULL,
    .phy_interface    = PHY_INTERFACE_MODE_MII,
    .shift = 1, /* FIFO Select is bit 0 */
};

static struct platform_device smsc_lan9221_weimcs0_device = {
	.name		    = "smsc911x_cs0",
	.id		        = 1,
	.num_resources	= ARRAY_SIZE(smsc911x_cs0_resources),
	.resource	    = smsc911x_cs0_resources,
	.dev		    = {
		.platform_data = &smsc911x_cs0_info,
	},
};

static struct platform_device smsc_lan9221_weimcs1_device = {
	.name		    = "smsc911x_cs1",
	.id		        = 0,
	.num_resources	= ARRAY_SIZE(smsc911x_cs1_resources),
	.resource	    = smsc911x_cs1_resources,
	.dev		    = {
		.platform_data = &smsc911x_cs1_info,
	},
};

/* These are defined in the dts */
/* static struct regulator_consumer_supply dummy_supplies[] = {
	REGULATOR_SUPPLY("vdd33a", "smsc911x"),
	REGULATOR_SUPPLY("vddvario", "smsc911x"),
};
*/
#endif // CONFIG_SMSC911X

#if defined (CONFIG_WL12XX)|| defined (CONFIG_WL12XX_MODULE)
static struct wl12xx_platform_data wl12xx_wlan_data __initdata = {};
#endif //CONFIG_WL12XX

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

/* PFL-A02 SOM-based PHY is KSZ9031RNX */
static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

static void __init ksp5013_enet_phy_init(void)
{
    phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK, ksz9031rn_phy_fixup);
}

static void __init ksp5013_1588_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}


#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init ksp5013_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low, macaddr_high;
	u8 *macaddr;

	enet_np = of_find_compatible_node(NULL, NULL, compatible);
	if (!enet_np)
		return;

	if (of_get_mac_address(enet_np))
		goto put_enet_node;

	ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!ocotp_np) {
		pr_warn("failed to find ocotp node\n");
		goto put_enet_node;
	}

	base = of_iomap(ocotp_np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_ocotp_node;
	}

	macaddr_high = readl_relaxed(base + OCOTP_MACn(0));
	macaddr_low = readl_relaxed(base + OCOTP_MACn(1));

	newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
	if (!newmac)
		goto put_ocotp_node;

	newmac->value = newmac + 1;
	newmac->length = 6;
	newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
	if (!newmac->name) {
		kfree(newmac);
		goto put_ocotp_node;
	}

	macaddr = newmac->value;
	macaddr[5] = macaddr_high & 0xff;
	macaddr[4] = (macaddr_high >> 8) & 0xff;
	macaddr[3] = (macaddr_high >> 16) & 0xff;
	macaddr[2] = (macaddr_high >> 24) & 0xff;
	macaddr[1] = macaddr_low & 0xff;
	macaddr[0] = (macaddr_low >> 8) & 0xff;

	of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
}

static inline void ksp5013_enet_init(void)
{
    struct regmap *gpr;

    ksp5013_enet_mac_init("fsl,imx6q-fec");
    ksp5013_enet_phy_init();
    //ksp5013_1588_init();

    /* TODO make this a dt request */
    gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
    if (!IS_ERR(gpr))
    {
        /* Set 32MB for WEIM CS0-3 and CS0 & CS1 on */
        regmap_update_bits(gpr, IOMUXC_GPR1, 0x3F, 0x09);
    }
    else
        pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

#ifdef CONFIG_SMSC911X
    smsc911x_cs0_resources[1].start = gpio_to_irq(132); /* EIM_D22__GPIO5_IO04 */
    smsc911x_cs0_resources[1].end = gpio_to_irq(132); /* EIM_D22__GPIO5_IO04 */
    smsc911x_cs1_resources[1].start = gpio_to_irq(86); /* EIM_D22__GPIO3_IO22 */
    smsc911x_cs1_resources[1].end = gpio_to_irq(86); /* EIM_D22__GPIO3_IO22 */
#endif //CONFIG_SMSC911X
}

//TODO change when moving to fslc as the gpr pinctrl hack isn't there

#ifndef IMX_OTG_ID_QUIRK
static void __init ksp5013_usbotg_init(void)
{
    struct regmap *gpr;

    gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
    {
        /* Set ID pin for OTG */
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_USB_OTG_ID_SEL_MASK,
                IMX6Q_GPR1_USB_OTG_ID_SEL_GPIO_1);
    }
}
#else
static void __init ksp5013_usbotg_init(void) {;}
#endif // IMX_OTG_ID_QUIRK

#if defined (CONFIG_WL12XX)|| defined (CONFIG_WL12XX_MODULE)

#define IMX_GPIO_NR(bank, nr)        (((bank) - 1) * 32 + (nr))
#define WLAN_IRQ    IMX_GPIO_NR(7, 0)
#define WLAN_ENABLE    IMX_GPIO_NR(6, 18)

static void __init ksp5013_wl1271_init(void)
{
	int ret = 0;
	pr_info("Enabling wl1271\n");
	wl12xx_wlan_data.irq = gpio_to_irq(WLAN_IRQ);
	wl12xx_wlan_data.board_ref_clock = WL12XX_REFCLOCK_38; /* 38.4 MHz */
	ret = wl12xx_set_platform_data(&wl12xx_wlan_data);
	if (ret) {
		pr_warn("failed to set wl12xx data!\n");
	}
}
#else
static void __init ksp5013_wl1271_init(void) {;}
#endif // (CONFIG_WL12XX || CONFIG_WL12XX_MODULE)

/* Add auxdata to pass platform data */
/* This may not be needed as what is the platform data and not the device data
 * is successfully pulled from the dt by the driver */
static const struct of_dev_auxdata ksp5013_auxdata_lookup[] __initconst = {
//#if defined (CONFIG_WL12XX || CONFIG_WL12XX_MODULE)
#if 0 
    OF_DEV_AUXDATA("smsc,lan9115", SMSC_START, NULL, &smsc911x_info),
#endif // CONFIG_WL12XX
    { /* sentinel */ }
};

#define WDT_ENABLE    IMX_GPIO_NR(2, 29)
static bool wdt_device_present = false;
static void __init ksp5013_arm_disarm_wdt(bool arm_wdt)
{
    if (wdt_device_present == true)
    {
        gpio_direction_output(WDT_ENABLE, (arm_wdt)?0:1);
        pr_info("MAX6031 watchdog %s\n",(arm_wdt)?"Armed":"Disarmed");
    }
};
static void __init ksp5013_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();
	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

    of_platform_populate(NULL, of_default_bus_match_table,
                    ksp5013_auxdata_lookup, parent);

    imx_anatop_init();
    imx6q_pm_init();
    ksp5013_enet_init();
    ksp5013_usbotg_init();
    ksp5013_wl1271_init();

    /* TODO: add hook for wdt enable from dt */
    if (gpio_request(WDT_ENABLE, "wdt-oe-b")) {
		pr_warn("failed to init watchdog enable!\n");
        wdt_device_present = false;
    }
    else
        wdt_device_present = true;
    
    /* Disarm MAX6031 Watchdog */
    ksp5013_arm_disarm_wdt(false);
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_1GHZ		0x2
#define OCOTP_CFG3_SPEED_850MHZ		0x1
#define OCOTP_CFG3_SPEED_800MHZ		0x0

static void __init ksp5013_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz; -- i.MX6Q only.
	 * 2b'10: 1000000000Hz;
	 * 2b'01: 850000000Hz; -- i.MX6Q Only, exclusive with 1GHz.
	 * 2b'00: 800000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_1P2GHZ)
			if (dev_pm_opp_disable(cpu_dev, 1200000000))
				pr_warn("failed to disable 1.2 GHz OPP\n");
	}
	if ((val & 0x3) < OCOTP_CFG3_SPEED_1GHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 1 GHz OPP\n");
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_850MHZ ||
			(val & 0x3) == OCOTP_CFG3_SPEED_1GHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 850 MHz OPP\n");
	}

put_node:
	of_node_put(np);
}

static void __init ksp5013_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	ksp5013_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device ksp5013_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init ksp5013_init_late(void)
{
	struct regmap *gpr;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
    {
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT/*MASK*/,
			IMX6Q_GPR1_GINT/*ASSERT*/);
    }
    else
    {
        pr_err("cannot find gpr dt node!\n");
    }

	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		ksp5013_opp_init(&ksp5013_cpufreq_pdev.dev);
		platform_device_register(&ksp5013_cpufreq_pdev);
	}
    /* Arm MAX6031 Watchdog */
    gpio_direction_output(WDT_ENABLE, 0);
    pr_info("MAX6031 watchdog armed\n");

#ifdef CONFIG_SMSC911X
    /* Placed in init_late to avoid duplicated probe calls */
    platform_device_register(&smsc_lan9221_weimcs0_device);
    platform_device_register(&smsc_lan9221_weimcs1_device);
#endif //CONFIG_SMSC911X
}

static void __init ksp5013_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
}

static void __init ksp5013_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init ksp5013_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());
}

static const char *ksp5013_dt_compat[] __initdata = {
	"phytec,ksp5013",
	NULL,
};

DT_MACHINE_START(KSP5013, "PHYTEC i.MX6 KSP5013 (Device Tree Support)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.smp		    = smp_ops(imx_smp_ops),
	.map_io		    = ksp5013_map_io,
	.init_irq	    = ksp5013_init_irq,
	.init_time	    = ksp5013_timer_init,
	.init_machine	= ksp5013_init_machine,
	.init_late      = ksp5013_init_late,
	.dt_compat	    = ksp5013_dt_compat,
	.restart	    = mxc_restart,
MACHINE_END
