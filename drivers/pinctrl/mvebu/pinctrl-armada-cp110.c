/*
 * Marvell Armada CP110 pinctrl driver based on mvebu pinctrl core
 *
 * Copyright (C) 2017 Marvell
 *
 * Hanna Hawa <hannah@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

#include "pinctrl-mvebu.h"

/*
 * Even if the pin controller is the same the MMP available depend on the SoC
 * integration.
 *  - In Armada7K (single CP) almost all the MPPs are available (except the
 *    MMP 39 to 43)
 *  - In Armada8K (dual CP) the MPPs are split into 2 parts, MPPs 0-31 from
 *    CPS, and MPPs 32-62 from CPM, the below flags (V_ARMADA_8K_CPM,
 *    V_ARMADA_8K_CPS) set which MPP is available to the CPx.
 * The x_PLUS enum mean that the MPP available for CPx and for Armada70x0
 */
enum {
	V_ARMADA_7K = BIT(0),
	V_ARMADA_8K_CPM = BIT(1),
	V_ARMADA_8K_CPS = BIT(2),
	V_ARMADA_7K_8K_CPM = (V_ARMADA_7K | V_ARMADA_8K_CPM),
	V_ARMADA_7K_8K_CPS = (V_ARMADA_7K | V_ARMADA_8K_CPS),
};

static struct mvebu_mpp_mode armada_cp110_mpp_modes[] = {
	MPP_MODE(0,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ale1"),
		 MPP_FUNCTION(2,	"au",		"i2smclk"),
		 MPP_FUNCTION(3,	"ge0",		"rxd3"),
		 MPP_FUNCTION(4,	"tdm",		"pclk"),
		 MPP_FUNCTION(6,	"ptp",		"pulse"),
		 MPP_FUNCTION(7,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(8,	"uart0",	"rxd"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"ge",		"mdio")),
	MPP_MODE(1,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ale0"),
		 MPP_FUNCTION(2,	"au",		"i2sdo_spdifo"),
		 MPP_FUNCTION(3,	"ge0",		"rxd2"),
		 MPP_FUNCTION(4,	"tdm",		"drx"),
		 MPP_FUNCTION(6,	"ptp",		"clk"),
		 MPP_FUNCTION(7,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(8,	"uart0",	"txd"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"ge",		"mdc")),
	MPP_MODE(2,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad15"),
		 MPP_FUNCTION(2,	"au",		"i2sextclk"),
		 MPP_FUNCTION(3,	"ge0",		"rxd1"),
		 MPP_FUNCTION(4,	"tdm",		"dtx"),
		 MPP_FUNCTION(5,	"mss_uart",	"rxd"),
		 MPP_FUNCTION(6,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(7,	"i2c1",		"sck"),
		 MPP_FUNCTION(8,	"uart1",	"rxd"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"xg",		"mdc")),
	MPP_MODE(3,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad14"),
		 MPP_FUNCTION(2,	"au",		"i2slrclk"),
		 MPP_FUNCTION(3,	"ge0",		"rxd0"),
		 MPP_FUNCTION(4,	"tdm",		"fsync"),
		 MPP_FUNCTION(5,	"mss_uart",	"txd"),
		 MPP_FUNCTION(6,	"pcie",		"rstoutn"),
		 MPP_FUNCTION(7,	"i2c1",		"sda"),
		 MPP_FUNCTION(8,	"uart1",	"txd"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"xg",		"mdio")),
	MPP_MODE(4,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad13"),
		 MPP_FUNCTION(2,	"au",		"i2sbclk"),
		 MPP_FUNCTION(3,	"ge0",		"rxctl"),
		 MPP_FUNCTION(4,	"tdm",		"rstn"),
		 MPP_FUNCTION(5,	"mss_uart",	"rxd"),
		 MPP_FUNCTION(6,	"uart1",	"cts"),
		 MPP_FUNCTION(7,	"pcie0",	"clkreq"),
		 MPP_FUNCTION(8,	"uart3",	"rxd"),
		 MPP_FUNCTION(10,	"ge",		"mdc")),
	MPP_MODE(5,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad12"),
		 MPP_FUNCTION(2,	"au",		"i2sdi"),
		 MPP_FUNCTION(3,	"ge0",		"rxclk"),
		 MPP_FUNCTION(4,	"tdm",		"intn"),
		 MPP_FUNCTION(5,	"mss_uart",	"txd"),
		 MPP_FUNCTION(6,	"uart1",	"rts"),
		 MPP_FUNCTION(7,	"pcie1",	"clkreq"),
		 MPP_FUNCTION(8,	"uart3",	"txd"),
		 MPP_FUNCTION(10,	"ge",		"mdio")),
	MPP_MODE(6,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad11"),
		 MPP_FUNCTION(3,	"ge0",		"txd3"),
		 MPP_FUNCTION(4,	"spi0",		"csn2"),
		 MPP_FUNCTION(5,	"au",		"i2sextclk"),
		 MPP_FUNCTION(6,	"sata1",	"present_act"),
		 MPP_FUNCTION(7,	"pcie2",	"clkreq"),
		 MPP_FUNCTION(8,	"uart0",	"rxd"),
		 MPP_FUNCTION(9,	"ptp",		"pulse")),
	MPP_MODE(7,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad10"),
		 MPP_FUNCTION(3,	"ge0",		"txd2"),
		 MPP_FUNCTION(4,	"spi0",		"csn1"),
		 MPP_FUNCTION(5,	"spi1",		"csn1"),
		 MPP_FUNCTION(6,	"sata0",	"present_act"),
		 MPP_FUNCTION(7,	"led",		"data"),
		 MPP_FUNCTION(8,	"uart0",	"txd"),
		 MPP_FUNCTION(9,	"ptp",		"clk")),
	MPP_MODE(8,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad9"),
		 MPP_FUNCTION(3,	"ge0",		"txd1"),
		 MPP_FUNCTION(4,	"spi0",		"csn0"),
		 MPP_FUNCTION(5,	"spi1",		"csn0"),
		 MPP_FUNCTION(6,	"uart0",	"cts"),
		 MPP_FUNCTION(7,	"led",		"stb"),
		 MPP_FUNCTION(8,	"uart2",	"rxd"),
		 MPP_FUNCTION(9,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(10,	"synce1",	"clk")),
	MPP_MODE(9,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad8"),
		 MPP_FUNCTION(3,	"ge0",		"txd0"),
		 MPP_FUNCTION(4,	"spi0",		"mosi"),
		 MPP_FUNCTION(5,	"spi1",		"mosi"),
		 MPP_FUNCTION(7,	"pcie",		"rstoutn"),
		 MPP_FUNCTION(10,	"synce2",	"clk")),
	MPP_MODE(10,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"readyn"),
		 MPP_FUNCTION(3,	"ge0",		"txctl"),
		 MPP_FUNCTION(4,	"spi0",		"miso"),
		 MPP_FUNCTION(5,	"spi1",		"miso"),
		 MPP_FUNCTION(6,	"uart0",	"cts"),
		 MPP_FUNCTION(7,	"sata1",	"present_act")),
	MPP_MODE(11,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"wen1"),
		 MPP_FUNCTION(3,	"ge0",		"txclkout"),
		 MPP_FUNCTION(4,	"spi0",		"clk"),
		 MPP_FUNCTION(5,	"spi1",		"clk"),
		 MPP_FUNCTION(6,	"uart0",	"rts"),
		 MPP_FUNCTION(7,	"led",		"clk"),
		 MPP_FUNCTION(8,	"uart2",	"txd"),
		 MPP_FUNCTION(9,	"sata0",	"present_act")),
	MPP_MODE(12,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"clk_out"),
		 MPP_FUNCTION(2,	"nf",		"rbn1"),
		 MPP_FUNCTION(3,	"spi1",		"csn1"),
		 MPP_FUNCTION(4,	"ge0",		"rxclk")),
	MPP_MODE(13,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"burstn"),
		 MPP_FUNCTION(2,	"nf",		"rbn0"),
		 MPP_FUNCTION(3,	"spi1",		"miso"),
		 MPP_FUNCTION(4,	"ge0",		"rxctl"),
		 MPP_FUNCTION(8,	"mss_spi",	"miso")),
	MPP_MODE(14,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"bootcsn"),
		 MPP_FUNCTION(2,	"dev",		"csn0"),
		 MPP_FUNCTION(3,	"spi1",		"csn0"),
		 MPP_FUNCTION(4,	"spi0",		"csn3"),
		 MPP_FUNCTION(5,	"au",		"i2sextclk"),
		 MPP_FUNCTION(6,	"spi0",		"miso"),
		 MPP_FUNCTION(7,	"sata0",	"present_act"),
		 MPP_FUNCTION(8,	"mss_spi",	"csn")),
	MPP_MODE(15,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad7"),
		 MPP_FUNCTION(3,	"spi1",		"mosi"),
		 MPP_FUNCTION(6,	"spi0",		"mosi"),
		 MPP_FUNCTION(8,	"mss_spi",	"mosi"),
		 MPP_FUNCTION(11,	"ptp",		"pulse_cp2cp")),
	MPP_MODE(16,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad6"),
		 MPP_FUNCTION(3,	"spi1",		"clk"),
		 MPP_FUNCTION(8,	"mss_spi",	"clk")),
	MPP_MODE(17,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad5"),
		 MPP_FUNCTION(4,	"ge0",		"txd3")),
	MPP_MODE(18,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad4"),
		 MPP_FUNCTION(4,	"ge0",		"txd2"),
		 MPP_FUNCTION(11,	"ptp",		"clk_cp2cp")),
	MPP_MODE(19,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad3"),
		 MPP_FUNCTION(4,	"ge0",		"txd1"),
		 MPP_FUNCTION(11,	"wakeup",	"out_cp2cp")),
	MPP_MODE(20,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad2"),
		 MPP_FUNCTION(4,	"ge0",		"txd0")),
	MPP_MODE(21,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad1"),
		 MPP_FUNCTION(4,	"ge0",		"txctl"),
		 MPP_FUNCTION(11,	"sei",		"in_cp2cp")),
	MPP_MODE(22,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"ad0"),
		 MPP_FUNCTION(4,	"ge0",		"txclkout"),
		 MPP_FUNCTION(11,	"wakeup",	"in_cp2cp")),
	MPP_MODE(23,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"a1"),
		 MPP_FUNCTION(5,	"au",		"i2smclk"),
		 MPP_FUNCTION(11,	"link",		"rd_in_cp2cp")),
	MPP_MODE(24,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"a0"),
		 MPP_FUNCTION(5,	"au",		"i2slrclk")),
	MPP_MODE(25,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"oen"),
		 MPP_FUNCTION(5,	"au",		"i2sdo_spdifo")),
	MPP_MODE(26,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"wen0"),
		 MPP_FUNCTION(5,	"au",		"i2sbclk")),
	MPP_MODE(27,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"csn0"),
		 MPP_FUNCTION(2,	"spi1",		"miso"),
		 MPP_FUNCTION(3,	"mss_gpio4",	NULL),
		 MPP_FUNCTION(4,	"ge0",		"rxd3"),
		 MPP_FUNCTION(5,	"spi0",		"csn4"),
		 MPP_FUNCTION(8,	"ge",		"mdio"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"uart0",	"rts"),
		 MPP_FUNCTION(11,	"rei",		"in_cp2cp")),
	MPP_MODE(28,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"csn1"),
		 MPP_FUNCTION(2,	"spi1",		"csn0"),
		 MPP_FUNCTION(3,	"mss_gpio5",	NULL),
		 MPP_FUNCTION(4,	"ge0",		"rxd2"),
		 MPP_FUNCTION(5,	"spi0",		"csn5"),
		 MPP_FUNCTION(6,	"pcie2",	"clkreq"),
		 MPP_FUNCTION(7,	"ptp",		"pulse"),
		 MPP_FUNCTION(8,	"ge",		"mdc"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"uart0",	"cts"),
		 MPP_FUNCTION(11,	"led",		"data")),
	MPP_MODE(29,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"csn2"),
		 MPP_FUNCTION(2,	"spi1",		"mosi"),
		 MPP_FUNCTION(3,	"mss_gpio6",	NULL),
		 MPP_FUNCTION(4,	"ge0",		"rxd1"),
		 MPP_FUNCTION(5,	"spi0",		"csn6"),
		 MPP_FUNCTION(6,	"pcie1",	"clkreq"),
		 MPP_FUNCTION(7,	"ptp",		"clk"),
		 MPP_FUNCTION(8,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"uart0",	"rxd"),
		 MPP_FUNCTION(11,	"led",		"stb")),
	MPP_MODE(30,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"csn3"),
		 MPP_FUNCTION(2,	"spi1",		"clk"),
		 MPP_FUNCTION(3,	"mss_gpio7",	NULL),
		 MPP_FUNCTION(4,	"ge0",		"rxd0"),
		 MPP_FUNCTION(5,	"spi0",		"csn7"),
		 MPP_FUNCTION(6,	"pcie0",	"clkreq"),
		 MPP_FUNCTION(7,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(8,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"uart0",	"txd"),
		 MPP_FUNCTION(11,	"led",		"clk")),
	MPP_MODE(31,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"dev",		"a2"),
		 MPP_FUNCTION(3,	"mss_gpio4",	NULL),
		 MPP_FUNCTION(6,	"pcie",		"rstoutn"),
		 MPP_FUNCTION(8,	"ge",		"mdc")),
	MPP_MODE(32,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mii",		"col"),
		 MPP_FUNCTION(2,	"mii",		"txerr"),
		 MPP_FUNCTION(3,	"mss_spi",	"miso"),
		 MPP_FUNCTION(4,	"tdm",		"drx"),
		 MPP_FUNCTION(5,	"au",		"i2sextclk"),
		 MPP_FUNCTION(6,	"au",		"i2sdi"),
		 MPP_FUNCTION(7,	"ge",		"mdio"),
		 MPP_FUNCTION(8,	"sdio",		"v18_en"),
		 MPP_FUNCTION(9,	"pcie1",	"clkreq"),
		 MPP_FUNCTION(10,	"mss_gpio0",	NULL)),
	MPP_MODE(33,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mii",		"txclk"),
		 MPP_FUNCTION(2,	"sdio",		"pwr10"),
		 MPP_FUNCTION(3,	"mss_spi",	"csn"),
		 MPP_FUNCTION(4,	"tdm",		"fsync"),
		 MPP_FUNCTION(5,	"au",		"i2smclk"),
		 MPP_FUNCTION(6,	"sdio",		"bus_pwr"),
		 MPP_FUNCTION(8,	"xg",		"mdio"),
		 MPP_FUNCTION(9,	"pcie2",	"clkreq"),
		 MPP_FUNCTION(10,	"mss_gpio1",	NULL)),
	MPP_MODE(34,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mii",		"rxerr"),
		 MPP_FUNCTION(2,	"sdio",		"pwr11"),
		 MPP_FUNCTION(3,	"mss_spi",	"mosi"),
		 MPP_FUNCTION(4,	"tdm",		"dtx"),
		 MPP_FUNCTION(5,	"au",		"i2slrclk"),
		 MPP_FUNCTION(6,	"sdio",		"wr_protect"),
		 MPP_FUNCTION(7,	"ge",		"mdc"),
		 MPP_FUNCTION(9,	"pcie0",	"clkreq"),
		 MPP_FUNCTION(10,	"mss_gpio2",	NULL)),
	MPP_MODE(35,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sata1",	"present_act"),
		 MPP_FUNCTION(2,	"i2c1",		"sda"),
		 MPP_FUNCTION(3,	"mss_spi",	"clk"),
		 MPP_FUNCTION(4,	"tdm",		"pclk"),
		 MPP_FUNCTION(5,	"au",		"i2sdo_spdifo"),
		 MPP_FUNCTION(6,	"sdio",		"card_detect"),
		 MPP_FUNCTION(7,	"xg",		"mdio"),
		 MPP_FUNCTION(8,	"ge",		"mdio"),
		 MPP_FUNCTION(9,	"pcie",		"rstoutn"),
		 MPP_FUNCTION(10,	"mss_gpio3",	NULL)),
	MPP_MODE(36,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"synce2",	"clk"),
		 MPP_FUNCTION(2,	"i2c1",		"sck"),
		 MPP_FUNCTION(3,	"ptp",		"clk"),
		 MPP_FUNCTION(4,	"synce1",	"clk"),
		 MPP_FUNCTION(5,	"au",		"i2sbclk"),
		 MPP_FUNCTION(6,	"sata0",	"present_act"),
		 MPP_FUNCTION(7,	"xg",		"mdc"),
		 MPP_FUNCTION(8,	"ge",		"mdc"),
		 MPP_FUNCTION(9,	"pcie2",	"clkreq"),
		 MPP_FUNCTION(10,	"mss_gpio5",	NULL)),
	MPP_MODE(37,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"uart2",	"rxd"),
		 MPP_FUNCTION(2,	"i2c0",		"sck"),
		 MPP_FUNCTION(3,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(4,	"tdm",		"intn"),
		 MPP_FUNCTION(5,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(6,	"sata1",	"present_act"),
		 MPP_FUNCTION(7,	"ge",		"mdc"),
		 MPP_FUNCTION(8,	"xg",		"mdc"),
		 MPP_FUNCTION(9,	"pcie1",	"clkreq"),
		 MPP_FUNCTION(10,	"mss_gpio6",	NULL),
		 MPP_FUNCTION(11,	"link",		"rd_out_cp2cp")),
	MPP_MODE(38,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"uart2",	"txd"),
		 MPP_FUNCTION(2,	"i2c0",		"sda"),
		 MPP_FUNCTION(3,	"ptp",		"pulse"),
		 MPP_FUNCTION(4,	"tdm",		"rstn"),
		 MPP_FUNCTION(5,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(6,	"sata0",	"present_act"),
		 MPP_FUNCTION(7,	"ge",		"mdio"),
		 MPP_FUNCTION(8,	"xg",		"mdio"),
		 MPP_FUNCTION(9,	"au",		"i2sextclk"),
		 MPP_FUNCTION(10,	"mss_gpio7",	NULL),
		 MPP_FUNCTION(11,	"ptp",		"pulse_cp2cp")),
	MPP_MODE(39,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sdio",		"wr_protect"),
		 MPP_FUNCTION(4,	"au",		"i2sbclk"),
		 MPP_FUNCTION(5,	"ptp",		"clk"),
		 MPP_FUNCTION(6,	"spi0",		"csn1"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"mss_gpio0",	NULL)),
	MPP_MODE(40,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sdio",		"pwr11"),
		 MPP_FUNCTION(2,	"synce1",	"clk"),
		 MPP_FUNCTION(3,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(4,	"au",		"i2sdo_spdifo"),
		 MPP_FUNCTION(5,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(6,	"spi0",		"clk"),
		 MPP_FUNCTION(7,	"uart1",	"txd"),
		 MPP_FUNCTION(8,	"ge",		"mdio"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"mss_gpio1",	NULL)),
	MPP_MODE(41,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sdio",		"pwr10"),
		 MPP_FUNCTION(2,	"sdio",		"bus_pwr"),
		 MPP_FUNCTION(3,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(4,	"au",		"i2slrclk"),
		 MPP_FUNCTION(5,	"ptp",		"pulse"),
		 MPP_FUNCTION(6,	"spi0",		"mosi"),
		 MPP_FUNCTION(7,	"uart1",	"rxd"),
		 MPP_FUNCTION(8,	"ge",		"mdc"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"mss_gpio2",	NULL),
		 MPP_FUNCTION(11,	"rei",		"out_cp2cp")),
	MPP_MODE(42,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sdio",		"v18_en"),
		 MPP_FUNCTION(2,	"sdio",		"wr_protect"),
		 MPP_FUNCTION(3,	"synce2",	"clk"),
		 MPP_FUNCTION(4,	"au",		"i2smclk"),
		 MPP_FUNCTION(5,	"mss_uart",	"txd"),
		 MPP_FUNCTION(6,	"spi0",		"miso"),
		 MPP_FUNCTION(7,	"uart1",	"cts"),
		 MPP_FUNCTION(8,	"xg",		"mdc"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"mss_gpio4",	NULL)),
	MPP_MODE(43,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"sdio",		"card_detect"),
		 MPP_FUNCTION(3,	"synce1",	"clk"),
		 MPP_FUNCTION(4,	"au",		"i2sextclk"),
		 MPP_FUNCTION(5,	"mss_uart",	"rxd"),
		 MPP_FUNCTION(6,	"spi0",		"csn0"),
		 MPP_FUNCTION(7,	"uart1",	"rts"),
		 MPP_FUNCTION(8,	"xg",		"mdio"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"mss_gpio5",	NULL),
		 MPP_FUNCTION(11,	"wakeup",	"out_cp2cp")),
	MPP_MODE(44,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txd2"),
		 MPP_FUNCTION(7,	"uart0",	"rts"),
		 MPP_FUNCTION(11,	"ptp",		"clk_cp2cp")),
	MPP_MODE(45,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txd3"),
		 MPP_FUNCTION(7,	"uart0",	"txd"),
		 MPP_FUNCTION(9,	"pcie",		"rstoutn")),
	MPP_MODE(46,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txd1"),
		 MPP_FUNCTION(7,	"uart1",	"rts")),
	MPP_MODE(47,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txd0"),
		 MPP_FUNCTION(5,	"spi1",		"clk"),
		 MPP_FUNCTION(7,	"uart1",	"txd"),
		 MPP_FUNCTION(8,	"ge",		"mdc")),
	MPP_MODE(48,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txctl_txen"),
		 MPP_FUNCTION(5,	"spi1",		"mosi"),
		 MPP_FUNCTION(8,	"xg",		"mdc"),
		 MPP_FUNCTION(11,	"wakeup",	"in_cp2cp")),
	MPP_MODE(49,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"txclkout"),
		 MPP_FUNCTION(2,	"mii",		"crs"),
		 MPP_FUNCTION(5,	"spi1",		"miso"),
		 MPP_FUNCTION(7,	"uart1",	"rxd"),
		 MPP_FUNCTION(8,	"ge",		"mdio"),
		 MPP_FUNCTION(9,	"pcie0",	"clkreq"),
		 MPP_FUNCTION(10,	"sdio",		"v18_en"),
		 MPP_FUNCTION(11,	"sei",		"out_cp2cp")),
	MPP_MODE(50,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxclk"),
		 MPP_FUNCTION(2,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(5,	"spi1",		"csn0"),
		 MPP_FUNCTION(6,	"uart2",	"txd"),
		 MPP_FUNCTION(7,	"uart0",	"rxd"),
		 MPP_FUNCTION(8,	"xg",		"mdio"),
		 MPP_FUNCTION(10,	"sdio",		"pwr11")),
	MPP_MODE(51,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxd0"),
		 MPP_FUNCTION(2,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(5,	"spi1",		"csn1"),
		 MPP_FUNCTION(6,	"uart2",	"rxd"),
		 MPP_FUNCTION(7,	"uart0",	"cts"),
		 MPP_FUNCTION(10,	"sdio",		"pwr10")),
	MPP_MODE(52,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxd1"),
		 MPP_FUNCTION(2,	"synce1",	"clk"),
		 MPP_FUNCTION(4,	"synce2",	"clk"),
		 MPP_FUNCTION(5,	"spi1",		"csn2"),
		 MPP_FUNCTION(7,	"uart1",	"cts"),
		 MPP_FUNCTION(8,	"led",		"clk"),
		 MPP_FUNCTION(9,	"pcie",		"rstoutn"),
		 MPP_FUNCTION(10,	"pcie0",	"clkreq")),
	MPP_MODE(53,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxd2"),
		 MPP_FUNCTION(3,	"ptp",		"clk"),
		 MPP_FUNCTION(5,	"spi1",		"csn3"),
		 MPP_FUNCTION(7,	"uart1",	"rxd"),
		 MPP_FUNCTION(8,	"led",		"stb"),
		 MPP_FUNCTION(11,	"sdio",		"led")),
	MPP_MODE(54,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxd3"),
		 MPP_FUNCTION(2,	"synce2",	"clk"),
		 MPP_FUNCTION(3,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(4,	"synce1",	"clk"),
		 MPP_FUNCTION(8,	"led",		"data"),
		 MPP_FUNCTION(10,	"sdio",		"hw_rst"),
		 MPP_FUNCTION(11,	"sdio",		"wr_protect")),
	MPP_MODE(55,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"ge1",		"rxctl_rxdv"),
		 MPP_FUNCTION(3,	"ptp",		"pulse"),
		 MPP_FUNCTION(10,	"sdio",		"led"),
		 MPP_FUNCTION(11,	"sdio",		"card_detect")),
	MPP_MODE(56,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(4,	"tdm",		"drx"),
		 MPP_FUNCTION(5,	"au",		"i2sdo_spdifo"),
		 MPP_FUNCTION(6,	"spi0",		"clk"),
		 MPP_FUNCTION(7,	"uart1",	"rxd"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(14,	"sdio",		"clk")),
	MPP_MODE(57,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(2,	"mss_i2c",	"sda"),
		 MPP_FUNCTION(3,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(4,	"tdm",		"intn"),
		 MPP_FUNCTION(5,	"au",		"i2sbclk"),
		 MPP_FUNCTION(6,	"spi0",		"mosi"),
		 MPP_FUNCTION(7,	"uart1",	"txd"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(14,	"sdio",		"cmd")),
	MPP_MODE(58,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(2,	"mss_i2c",	"sck"),
		 MPP_FUNCTION(3,	"ptp",		"clk"),
		 MPP_FUNCTION(4,	"tdm",		"rstn"),
		 MPP_FUNCTION(5,	"au",		"i2sdi"),
		 MPP_FUNCTION(6,	"spi0",		"miso"),
		 MPP_FUNCTION(7,	"uart1",	"cts"),
		 MPP_FUNCTION(8,	"led",		"clk"),
		 MPP_FUNCTION(14,	"sdio",		"d0")),
	MPP_MODE(59,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mss_gpio7",	NULL),
		 MPP_FUNCTION(2,	"synce2",	"clk"),
		 MPP_FUNCTION(4,	"tdm",		"fsync"),
		 MPP_FUNCTION(5,	"au",		"i2slrclk"),
		 MPP_FUNCTION(6,	"spi0",		"csn0"),
		 MPP_FUNCTION(7,	"uart0",	"cts"),
		 MPP_FUNCTION(8,	"led",		"stb"),
		 MPP_FUNCTION(9,	"uart1",	"txd"),
		 MPP_FUNCTION(14,	"sdio",		"d1")),
	MPP_MODE(60,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mss_gpio6",	NULL),
		 MPP_FUNCTION(3,	"ptp",		"pulse"),
		 MPP_FUNCTION(4,	"tdm",		"dtx"),
		 MPP_FUNCTION(5,	"au",		"i2smclk"),
		 MPP_FUNCTION(6,	"spi0",		"csn1"),
		 MPP_FUNCTION(7,	"uart0",	"rts"),
		 MPP_FUNCTION(8,	"led",		"data"),
		 MPP_FUNCTION(9,	"uart1",	"rxd"),
		 MPP_FUNCTION(14,	"sdio",		"d2")),
	MPP_MODE(61,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mss_gpio5",	NULL),
		 MPP_FUNCTION(3,	"ptp",		"clk"),
		 MPP_FUNCTION(4,	"tdm",		"pclk"),
		 MPP_FUNCTION(5,	"au",		"i2sextclk"),
		 MPP_FUNCTION(6,	"spi0",		"csn2"),
		 MPP_FUNCTION(7,	"uart0",	"txd"),
		 MPP_FUNCTION(8,	"uart2",	"txd"),
		 MPP_FUNCTION(9,	"sata1",	"present_act"),
		 MPP_FUNCTION(10,	"ge",		"mdio"),
		 MPP_FUNCTION(14,	"sdio",		"d3")),
	MPP_MODE(62,
		 MPP_FUNCTION(0,	"gpio",		NULL),
		 MPP_FUNCTION(1,	"mss_gpio4",	NULL),
		 MPP_FUNCTION(2,	"synce1",	"clk"),
		 MPP_FUNCTION(3,	"ptp",		"pclk_out"),
		 MPP_FUNCTION(5,	"sata1",	"present_act"),
		 MPP_FUNCTION(6,	"spi0",		"csn3"),
		 MPP_FUNCTION(7,	"uart0",	"rxd"),
		 MPP_FUNCTION(8,	"uart2",	"rxd"),
		 MPP_FUNCTION(9,	"sata0",	"present_act"),
		 MPP_FUNCTION(10,	"ge",		"mdc")),
};

#if 0
static const struct of_device_id armada_cp110_pinctrl_of_match[] = {
	{
		.compatible	= "marvell,armada-7k-pinctrl",
		.data		= (void *) V_ARMADA_7K,
	},
	{
		.compatible	= "marvell,armada-8k-cpm-pinctrl",
		.data		= (void *) V_ARMADA_8K_CPM,
	},
	{
		.compatible	= "marvell,armada-8k-cps-pinctrl",
		.data		= (void *) V_ARMADA_8K_CPS,
	},
	{ },
};
#endif

static const struct mvebu_mpp_ctrl armada_cp110_mpp_controls[] = {
	MPP_FUNC_CTRL(0, 62, NULL, mvebu_regmap_mpp_ctrl),
};

static void mvebu_pinctrl_assign_variant(struct mvebu_mpp_mode *m,
					 u8 variant)
{
	struct mvebu_mpp_ctrl_setting *s;

	for (s = m->settings ; s->name ; s++)
		s->variant = variant;
}

#if 0
static int armada_cp110_pinctrl_probe(struct platform_device *pdev)
{
	struct mvebu_pinctrl_soc_info *soc;
	const struct of_device_id *match =
		of_match_device(armada_cp110_pinctrl_of_match, &pdev->dev);
	int i;

	if (!pdev->dev.parent)
		return -ENODEV;

	soc = devm_kzalloc(&pdev->dev,
			   sizeof(struct mvebu_pinctrl_soc_info), GFP_KERNEL);
	if (!soc)
		return -ENOMEM;

	soc->variant = (unsigned long) match->data & 0xff;
	soc->controls = armada_cp110_mpp_controls;
	soc->ncontrols = ARRAY_SIZE(armada_cp110_mpp_controls);
	soc->modes = armada_cp110_mpp_modes;
	soc->nmodes = ARRAY_SIZE(armada_cp110_mpp_modes);
	for (i = 0; i < ARRAY_SIZE(armada_cp110_mpp_modes); i++) {
		struct mvebu_mpp_mode *m = &armada_cp110_mpp_modes[i];

		switch (i) {
		case 0 ... 31:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPS);
			break;
		case 32 ... 38:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPM);
			break;
		case 39 ... 43:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_8K_CPM);
			break;
		case 44 ... 62:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPM);
			break;
		}
	}
	pdev->dev.platform_data = soc;

	return mvebu_pinctrl_simple_regmap_probe(pdev, pdev->dev.parent, 0);
}

static struct platform_driver armada_cp110_pinctrl_driver = {
	.driver = {
		.name = "armada-cp110-pinctrl",
		.of_match_table = of_match_ptr(armada_cp110_pinctrl_of_match),
	},
	.probe = armada_cp110_pinctrl_probe,
};

builtin_platform_driver(armada_cp110_pinctrl_driver);
#else

static int armada_cp110_pinctrl_probe(struct vmm_device *dev,
				  const struct vmm_devtree_nodeid *devid)
{
	struct mvebu_pinctrl_soc_info *soc;
	int i;

	if (!dev->parent)
		return -ENODEV;

	soc = devm_kzalloc(dev,
			   sizeof(struct mvebu_pinctrl_soc_info), GFP_KERNEL);
	if (!soc)
		return -ENOMEM;

	soc->variant = (unsigned long)devid->data & 0xff;
	soc->controls = armada_cp110_mpp_controls;
	soc->ncontrols = ARRAY_SIZE(armada_cp110_mpp_controls);
	soc->modes = armada_cp110_mpp_modes;
	soc->nmodes = ARRAY_SIZE(armada_cp110_mpp_modes);
	for (i = 0; i < ARRAY_SIZE(armada_cp110_mpp_modes); i++) {
		struct mvebu_mpp_mode *m = &armada_cp110_mpp_modes[i];

		switch (i) {
		case 0 ... 31:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPS);
			break;
		case 32 ... 38:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPM);
			break;
		case 39 ... 43:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_8K_CPM);
			break;
		case 44 ... 62:
			mvebu_pinctrl_assign_variant(m, V_ARMADA_7K_8K_CPM);
			break;
		}
	}
	dev_set_platdata(dev, soc);

	return mvebu_pinctrl_simple_regmap_probe(dev, dev->parent, 0);
}

static struct vmm_devtree_nodeid armada_cp110_pinctrl_of_match[] = {
	{
		.compatible	= "marvell,armada-7k-pinctrl",
		.data		= (void *) V_ARMADA_7K,
	},
	{
		.compatible	= "marvell,armada-8k-cpm-pinctrl",
		.data		= (void *) V_ARMADA_8K_CPM,
	},
	{
		.compatible	= "marvell,armada-8k-cps-pinctrl",
		.data		= (void *) V_ARMADA_8K_CPS,
	},
	{ /* end of list */ },
};

static struct vmm_driver armada_cp110_pinctrl_driver = {
	.name = "armada-cp110-pinctrl",
	.match_table = armada_cp110_pinctrl_of_match,
	.probe = armada_cp110_pinctrl_probe,
};

static int __init armada_cp110_pinctrl_init(void)
{
	return vmm_devdrv_register_driver(&armada_cp110_pinctrl_driver);
}

static void __exit armada_cp110_pinctrl_exit(void)
{
	vmm_devdrv_unregister_driver(&armada_cp110_pinctrl_driver);
}

#define MODULE_DESC			"Marvell Armada CP110 Pin Controller"
#define MODULE_AUTHOR			"Anup Patel"
#define MODULE_LICENSE			"GPL"
#define MODULE_IPRIORITY		1
#define	MODULE_INIT			armada_cp110_pinctrl_init
#define	MODULE_EXIT			armada_cp110_pinctrl_exit

VMM_DECLARE_MODULE(MODULE_DESC,
			MODULE_AUTHOR,
			MODULE_LICENSE,
			MODULE_IPRIORITY,
			MODULE_INIT,
			MODULE_EXIT);
#endif
