/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <serial.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk_synthesizer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/omap_sec_common.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include <libfdt.h>
#include <fdt_support.h>

#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

/* GPIO that controls power to DDR on EVM-SK */
static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

/*
 * Read header information from EEPROM into global structure.
 */
#ifdef CONFIG_TI_I2C_BOARD_DETECT
void do_board_detect(void)
{
	enable_i2c0_pin_mux();
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

	if (ti_i2c_eeprom_am_get(-1, CONFIG_SYS_I2C_EEPROM_ADDR))
		printf("ti_i2c_eeprom_init failed\n");
}
#endif

#ifndef CONFIG_DM_SERIAL
struct serial_device *default_serial_console(void)
{
	return &eserial4_device;
}
#endif

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

const struct dpll_params *get_dpll_ddr_params(void)
{
	int ind = get_sys_clk_index();

	return &dpll_ddr3_400MHz[ind];
}

const struct dpll_params *get_dpll_mpu_params(void)
{
	int ind = get_sys_clk_index();
	int freq = am335x_get_efuse_mpu_max_freq(cdev);

	switch (freq) {
	case MPUPLL_M_1000:
		return &dpll_mpu_opp[ind][5];
	case MPUPLL_M_800:
		return &dpll_mpu_opp[ind][4];
	case MPUPLL_M_720:
		return &dpll_mpu_opp[ind][3];
	case MPUPLL_M_600:
		return &dpll_mpu_opp[ind][2];
	case MPUPLL_M_500:
		return &dpll_mpu_opp100;
	case MPUPLL_M_300:
		return &dpll_mpu_opp[ind][0];
	}

	return &dpll_mpu_opp[ind][0];
}


void scale_vcores(void)
{
	
}

void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#elif CONFIG_CONS_INDEX == 3
	enable_uart2_pin_mux();
#elif CONFIG_CONS_INDEX == 4
	enable_uart3_pin_mux();
#elif CONFIG_CONS_INDEX == 5
	enable_uart4_pin_mux();
#elif CONFIG_CONS_INDEX == 6
	enable_uart5_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

#define SBC8600B_EMIF_READ_LATENCY 	0x10B
#define SBC8600B_EMIF_TIM1     		0x0CCCE524
#define SBC8600B_EMIF_TIM2     		0x30308023
#define SBC8600B_EMIF_TIM3     		0x009F82CF
#define SBC8600B_EMIF_SDCFG        	0x62A45032
#define SBC8600B_EMIF_SDREF        	0x10000C30
#define SBC8600B_ZQ_CFG            	0x50074BE4
#define SBC8600B_RATIO         		0x100
#define SBC8600B_INVERT_CLKOUT     	0x01
#define SBC8600B_RD_DQS            	0x3B
#define SBC8600B_WR_DQS            	0xA6
#define SBC8600B_PHY_FIFO_WE       	0x12A
#define SBC8600B_PHY_WR_DATA       	0xE6
#define SBC8600B_IOCTRL_VALUE      	0x18B

static const struct ddr_data ddr3_sbc8600b_data = {
   .datardsratio0 = SBC8600B_RD_DQS,
   .datawdsratio0 = SBC8600B_WR_DQS,
   .datafwsratio0 = SBC8600B_PHY_FIFO_WE,
   .datawrsratio0 = SBC8600B_PHY_WR_DATA,
};
static const struct cmd_control ddr3_cmd_ctrl_sbc8600b_data = {
   .cmd0csratio = SBC8600B_RATIO,
   .cmd0iclkout = SBC8600B_INVERT_CLKOUT,

   .cmd1csratio = SBC8600B_RATIO,
   .cmd1iclkout = SBC8600B_INVERT_CLKOUT,

   .cmd2csratio = SBC8600B_RATIO,
   .cmd2iclkout = SBC8600B_INVERT_CLKOUT,
};
static struct emif_regs ddr3_emif_reg_sbc8600b_data = {
   .sdram_config = 	SBC8600B_EMIF_SDCFG,
   .ref_ctrl = 		SBC8600B_EMIF_SDREF,
   .sdram_tim1 = 	SBC8600B_EMIF_TIM1,
   .sdram_tim2 = 	SBC8600B_EMIF_TIM2,
   .sdram_tim3 = 	SBC8600B_EMIF_TIM3,
   .zq_config = 	SBC8600B_ZQ_CFG,
   .emif_ddr_phy_ctlr_1 = 	SBC8600B_EMIF_READ_LATENCY |
               				PHY_EN_DYN_PWRDN,
};
const struct ctrl_ioregs ioregs_sbc8600b = {
   .cm0ioctl       = SBC8600B_IOCTRL_VALUE,
   .cm1ioctl       = SBC8600B_IOCTRL_VALUE,
   .cm2ioctl       = SBC8600B_IOCTRL_VALUE,
   .dt0ioctl       = SBC8600B_IOCTRL_VALUE,
   .dt1ioctl       = SBC8600B_IOCTRL_VALUE,
};


void sdram_init(void)
{
		config_ddr(400, &ioregs_sbc8600b,
			   &ddr3_sbc8600b_data,
			   &ddr3_cmd_ctrl_sbc8600b_data,
			   &ddr3_emif_reg_sbc8600b_data, 0);
}
#endif

#if !defined(CONFIG_SPL_BUILD) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP))

static bool eth0_is_mii;
static bool eth1_is_mii;
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif

#if !defined(CONFIG_SPL_BUILD) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#if !defined(CONFIG_SPL_BUILD)
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

	/*
	 * Default FIT boot on HS devices. Non FIT images are not allowed
	 * on HS devices.
	 */
	if (get_device_type() == HS_DEVICE)
		setenv("boot_fit", "1");
#endif

#if !defined(CONFIG_SPL_BUILD)
	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}
#endif

	return 0;
}
#endif

#ifndef CONFIG_DM_ETH

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 4,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) &&\
	defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/*
	 * use efuse mac address for USB ethernet as we know that
	 * both CPSW and USB ethernet will never be active at the same time
	 */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;


#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		puts("<ethaddr> not set. Validating first E-fuse MAC\n");
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW
		writel((RGMII_MODE_ENABLE | RGMII_INT_DELAY), &cdev->miisel);
		cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if =
				PHY_INTERFACE_MODE_RGMII;

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif

	/*
	 *
	 * CPSW RGMII Internal Delay Mode is not supported in all PVT
	 * operating points.  So we must set the TX clock delay feature
	 * in the AR8051 PHY.  Since we only support a single ethernet
	 * device in U-Boot, we only do this for the first instance.
	 */
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		0x100

		const char *devname;
		devname = miiphy_get_current_dev();

		miiphy_write(devname, 0x4, AR8051_PHY_DEBUG_ADDR_REG,
				AR8051_DEBUG_RGMII_CLK_DLY_REG);
		miiphy_write(devname, 0x4, AR8051_PHY_DEBUG_DATA_REG,
				AR8051_RGMII_TX_CLK_DLY);
#endif
#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ethaddr(mac_addr))
		eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;
}
#endif

#endif /* CONFIG_DM_ETH */

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, "am335x-sbc8600b"))
		return 0;
	return -1;
}
#endif

#ifdef CONFIG_TI_SECURE_DEVICE
void board_fit_image_post_process(void **p_image, size_t *p_size)
{
	secure_boot_verify_image(p_image, p_size);
}
#endif
#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *fdt, bd_t *bd)
{
	return 0;
}
#endif
