/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/opp.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#include "common.h"

/* iNGERSOL LED GPIO CONFIG */
#define INGERSOL_LED1	GPIO_TO_PIN(1, 13)
#define INGERSOL_LED2	GPIO_TO_PIN(0, 27)
#define INGERSOL_LED3	GPIO_TO_PIN(0, 23)
#define INGERSOL_LED4	GPIO_TO_PIN(1, 14)
#define INGERSOL_LED5	GPIO_TO_PIN(0, 26)
#define INGERSOL_LED6	GPIO_TO_PIN(1, 15)
#define INGERSOL_LED7	GPIO_TO_PIN(1, 12)
#define INGERSOL_LED8	GPIO_TO_PIN(2, 13)

/*Ingersol gpio init */
static int gpios[] = { INGERSOL_LED1, INGERSOL_LED2, INGERSOL_LED3, INGERSOL_LED4,
                       INGERSOL_LED5, INGERSOL_LED6, INGERSOL_LED7, INGERSOL_LED8 };

/* Platform structure for gpio driver for Ingersol board */
static struct platform_device gpio_device = {
       .name              = "gpio_driver",
       .id                = -1,
       .dev.platform_data = &gpios,
};

static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(3, 14),
		.gpio_wp        = GPIO_TO_PIN(0, 7),
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
  const char *string_name; /* signal name format */
  int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
  void (*device_init)(int evm_id, int profile);

#define DEV_ON_BASEBOARD  0
  u32 device_on;
  u32 profile;
};

static bool daughter_brd_detected;
static int am33xx_evmid = -EINVAL;

void am335x_evm_set_id(unsigned int evmid)
{
   am33xx_evmid = evmid;
	return;
}

int am335x_evm_get_id(void)
{
   return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);


/* Module pin mux for SPI fash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mii2 */
static struct pinmux_config mii2_pin_mux[] = {
	{"gpmc_wpn.mii2_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a0.mii2_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a1.mii2_rxdv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.mii2_txd3", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a3.mii2_txd2", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.mii2_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.mii2_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.mii2_txclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a7.mii2_rxclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.mii2_rxd3", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.mii2_rxd2", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.mii2_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.mii2_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
        {"mmc0_dat3.mmc0_dat3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat2.mmc0_dat2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat1.mmc0_dat1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat0.mmc0_dat0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_clk.mmc0_clk",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_cmd.mmc0_cmd",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mcasp0_aclkx.mmc0_sdcd",  OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLUP},
        {NULL, 0},
};

/* Module Pin mux for CAN0 */
static struct pinmux_config can_pin_mux[] = {
        {"uart1_ctsn.dcan0_tx",OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
        {"uart1_rtsn.dcan0_rx",OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
        {NULL, 0},
};

/* Module pin mux for uart3 */
static struct pinmux_config uart3_pin_mux[] = {
        {"spi0_cs1.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"ecap0_in_pwm0_out.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
        {NULL, 0},
};


/* Module pin mux for uart5 */
static struct pinmux_config uart5_pin_mux[] = {
        {"lcd_data9.uart5_rxd", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLUP},
        {"lcd_data8.uart5_txd", OMAP_MUX_MODE4 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin mux for uart1 Telet GPRS Modem */
static struct pinmux_config uart1_pin_mux[] = {
        {"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT},
        {"uart1_rxd.uart1_rxd",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"uart1_txd.uart1_txd",   OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
        {"gpmc_ad8.gpio0_22",     OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},        
        {NULL, 0},
};

/* Module pin mux for Ingersol LEDs */
static struct pinmux_config gpio_pin_mux[] = {
        {"gpmc_ad13.gpio1_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED1 GPIO */
        {"gpmc_ad11.gpio0_27", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED2 GPIO */
        {"gpmc_ad9.gpio0_23",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED3 GPIO */
        {"gpmc_ad14.gpio1_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED4 GPIO */
        {"gpmc_ad10.gpio0_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED5 GPIO */
        {"gpmc_ad15.gpio1_15", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED6 GPIO */
        {"gpmc_ad12.gpio1_12", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED7 GPIO */
        {"lcd_data7.gpio2_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /*LED8 GPIO */
        {NULL, 0},
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}
/* EVM Board Configuration */
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}

/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct pinmux_config i2c0_pin_mux[] = {
        {"i2c0_sda.i2c0_sda", OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
                                        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
        {"i2c0_scl.i2c0_scl", OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
                                        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
        {NULL, 0},
};

static void mii2_init(int evm_id, int profile)
{
	setup_pin_mux(mii2_pin_mux);
	return;
}

static void rmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rmii1_pin_mux);
	return;
}

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}

static void uart3_init(int evm_id, int profile)
{
        setup_pin_mux(uart3_pin_mux);
        return;
}

static void uart5_init(int evm_id, int profile)
{
        setup_pin_mux(uart5_pin_mux);
        return;
}

static void uart1_init(int evm_id, int profile)
{
        setup_pin_mux(uart1_pin_mux);
        return;
}

/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = SZ_512K + SZ_256K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = MTDPART_SIZ_FULL,
	}
};

static const struct flash_platform_data am335x_spi_flash = {
	.type      = "sst25vf016b",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi0_slave_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &am335x_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 10000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
};

/* Calixto EVM MAC Address initialization */
void calixto_evm_phy_int(void){

#define EEPROM_NO_OF_MAC_ADDR         3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

  am335x_mac_addr[0][0] = 0xd4;
  am335x_mac_addr[0][1] = 0x94;
  am335x_mac_addr[0][2] = 0xa1;
  am335x_mac_addr[0][3] = 0x38;
  am335x_mac_addr[0][4] = 0xed;
  am335x_mac_addr[0][5] = 0x8b;
  am335x_mac_addr[1][0] = 0xd4;
  am335x_mac_addr[1][1] = 0x94;
  am335x_mac_addr[1][2] = 0xa1;
  am335x_mac_addr[1][3] = 0x38;
  am335x_mac_addr[1][4] = 0xed;
  am335x_mac_addr[1][5] = 0x8c;

  am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
                          &am335x_mac_addr[1][0]);

}

/* Calixto EVM CAN Interface */
void can0_init(int evm_id, int profile)
{
     setup_pin_mux(can_pin_mux);
     /* Instance Zero */
     am33xx_d_can_init(0);
}

/*Calixto EVM MMC0 Interface */
static void mmc0_init(int evm_id, int profile)
{
        setup_pin_mux(mmc0_pin_mux);
        omap2_hsmmc_init(am335x_mmc);
        return;
}

/* Calixto EVM SPI Flash Interface */
static void spi0_init(int evm_id, int profile)
{
        setup_pin_mux(spi0_pin_mux);
        spi_register_board_info(am335x_spi0_slave_info,
                        ARRAY_SIZE(am335x_spi0_slave_info));
        return;
}

/* Ingersol EVM GPIO_Driver */
static void gpio_init(int evm_id, int profile)
{
   setup_pin_mux(gpio_pin_mux);
   (void) platform_device_register(&gpio_device);     
        return;
}

/* Calixto EVM Interface Configuration */
static struct evm_dev_cfg calixto_dev_cfg[] = {
        {spi0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mii2_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{rmii1_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {mmc0_init,     DEV_ON_BASEBOARD, PROFILE_NONE},  
//        {can0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart3_init,  DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart5_init,  DEV_ON_BASEBOARD, PROFILE_NONE},
        {gpio_init,     DEV_ON_BASEBOARD, PROFILE_NONE},  /* Ingersol Led GPIO */
        {NULL, 0, 0},
};

static void setup_calixto_evm_board(void)
{
        
    pr_info("The board is a AM335x Calixto board.\n");
    
    calixto_evm_phy_int();
    /* EVM has Micro-SD slot which doesn't have Write Protect pin */
    am335x_mmc[0].gpio_wp = -EINVAL;

    _configure_device(CALIXTO_EVM, calixto_dev_cfg, PROFILE_NONE);

     am33xx_cpsw_init(CALIXTO_EVM_ETHERNET_INTERFACE, NULL, "0:03");
}

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

/* I2C Bus Call */
static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
        {
                I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
                .platform_data  = &am335x_tps65910_info,
        },
};

/* Calixto BP USB Host Mode */
static struct omap_musb_board_data evm_musb_pdata_data = {
        .interface_type = MUSB_INTERFACE_ULPI,
        .mode           = (MUSB_HOST << 4) | MUSB_HOST,
        .power          = 500,
        .instances      = 1,
};

static void __init am335x_evm_i2c_init(void)
{
    setup_pin_mux(i2c0_pin_mux); 
    omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo,
            		ARRAY_SIZE(am335x_i2c0_boardinfo));
}

static struct resource am335x_rtc_resources[] = {
	{
		.start		= AM33XX_RTC_BASE,
		.end		= AM33XX_RTC_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{ /* timer irq */
		.start		= AM33XX_IRQ_RTC_TIMER,
		.end		= AM33XX_IRQ_RTC_TIMER,
		.flags		= IORESOURCE_IRQ,
	},
	{ /* alarm irq */
		.start		= AM33XX_IRQ_RTC_ALARM,
		.end		= AM33XX_IRQ_RTC_ALARM,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device am335x_rtc_device = {
	.name           = "omap_rtc",
	.id             = -1,
	.num_resources	= ARRAY_SIZE(am335x_rtc_resources),
	.resource	= am335x_rtc_resources,
};

static int am335x_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return -1;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return -1;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return -ENOMEM;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	return  platform_device_register(&am335x_rtc_device);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout2_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
        am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

        return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_rtc_init();
	clkout2_enable();
	am335x_evm_i2c_init();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&evm_musb_pdata_data);
	/* Create an alias for icss clock */
///	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
//		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
//	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
//		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");

        setup_calixto_evm_board();
}

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END

MACHINE_START(AM335XIAEVM, "am335xiaevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
