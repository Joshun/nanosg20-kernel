/*
 *  Copyright (C) 2010 Christian Glindkamp <christian.glindkamp@taskit.de>
 *                     taskit GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/input.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/w1-gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/at91sam9_smc.h>
#include <mach/board.h>

#include "generic.h"
#include "sam9_smc.h"


static void __init nanosg20_init_early(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
	at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
						| ATMEL_UART_DTR | ATMEL_UART_DSR
						| ATMEL_UART_DCD | ATMEL_UART_RI);

	/* USART1 on ttyS2 as RS485. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_RTS);

	/* USART2 on ttyS3. (Rx, Tx, CTS, RTS) */
	at91_register_uart(AT91SAM9260_ID_US2, 3, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* added by josh :P see http://armbedded.eu/node/472 */
	/* USART3 on ttyS4. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US3, 4, 0);

	/* set serial console to ttyS1 (ie, USART0) */
	at91_set_serial_console(1);

	/* enable RS232 drivers for USART0 and USART2 */
	at91_set_GPIO_periph(AT91_PIN_PA25, 0);
	at91_set_gpio_output(AT91_PIN_PA25, 0);
	at91_set_GPIO_periph(AT91_PIN_PC3, 0);
	at91_set_gpio_output(AT91_PIN_PC3, 0);
}

/*
 * NAND flash
 */
static struct atmel_nand_data __initdata nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.bus_width_16	= 0,
	.ecc_mode	= NAND_ECC_SOFT_BCH,
	.det_pin	= -EINVAL,
};

static struct sam9_smc_config __initdata nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_8,
	.tdf_cycles		= 3,
};

static void __init nanosg20_add_device_nand(void)
{
	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(0, 3, &nand_smc_config);

	at91_add_device_nand(&nand_data);
}

/*
 * MCI (SD/MMC)
 * det_pin, wp_pin and vcc_pin are not connected
 */
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
static struct mci_platform_data __initdata mmc_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= -1,
		.wp_pin		= -1,
	},
};
#else
static struct at91_mmc_data __initdata mmc_data = {
	.slot_b		= 0,
	.wire4		= 1,
	.det_pin	= -EINVAL,
	.wp_pin		= -EINVAL,
	.vcc_pin	= -EINVAL,
};
#endif

/*
 * USB Host port
 */
static struct at91_usbh_data __initdata usbh_data = {
	.ports		= 2,
	.vbus_pin	= {-EINVAL, -EINVAL},
	.overcurrent_pin= {-EINVAL, -EINVAL},
};


/*
 * USB Device port
 */
static struct at91_udc_data __initdata nanosg20_udc_data = {
	.vbus_pin	= AT91_PIN_PC7,
	.pullup_pin	= -EINVAL,		/* pull-up driven by UDC */
};

/*
 * MACB Ethernet device
 */
static struct macb_platform_data __initdata macb_data = {
	.phy_irq_pin	= AT91_PIN_PA28,
	.is_rmii	= 1,
};

/*
 * LEDs
 */
static struct gpio_led nanosg20_leds[] = {
	{
		.name			= "LED1",
		.gpio			= AT91_PIN_PB20,
		.active_low		= 1,
		.default_trigger	= "mmc0",
	}, {
		.name			= "LED2",
		.gpio			= AT91_PIN_PB21,
		.active_low		= 1,
		.default_trigger	= "nand-disk",
	}, {
		.name			= "LED3",
		.gpio			= AT91_PIN_PB30,
		.active_low		= 1,
		.default_trigger	= "default-on",
	}, {
		.name			= "LED4",
		.gpio			= AT91_PIN_PB31,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	}
};

/*
 * GPIO Buttons
 */
static struct gpio_keys_button nanosg20_buttons[] = {
	{
		.gpio		= AT91_PIN_PC9,
		.code		= KEY_POWER,
		.wakeup		= 1,
		.debounce_interval = 100,
	}
};

static struct gpio_keys_platform_data nanosg20_button_data = {
	.buttons	= nanosg20_buttons,
	.nbuttons	= ARRAY_SIZE(nanosg20_buttons),
};

static struct platform_device nanosg20_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &nanosg20_button_data,
	}
};

static void __init nanosg20_add_device_buttons(void)
{
	at91_set_gpio_input(AT91_PIN_PC9, 1);
	at91_set_deglitch(AT91_PIN_PC9, 1);

	platform_device_register(&nanosg20_button_device);
}

/*
 * SPI devices
 */
static struct spi_board_info nanosg20_spi_devices[] = {
	{
		.modalias	= "spidev",
		.chip_select	= 0,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 0,
	}, {
		.modalias	= "spidev",
		.chip_select	= 0,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
};

/*
 * Dallas 1-Wire
 */
static struct w1_gpio_platform_data nanosg20_w1_gpio_pdata = {
	.pin		= AT91_PIN_PA29,
	.is_open_drain	= 1,
};

static struct platform_device nanosg20_w1_device = {
	.name			= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &nanosg20_w1_gpio_pdata,
};

void nanosg20_add_w1(void)
{
	at91_set_GPIO_periph(nanosg20_w1_gpio_pdata.pin, 1);
	at91_set_multi_drive(nanosg20_w1_gpio_pdata.pin, 1);
	platform_device_register(&nanosg20_w1_device);
}

static void __init nanosg20_board_init(void)
{
	at91_add_device_serial();
	at91_add_device_usbh(&usbh_data);
	at91_add_device_udc(&nanosg20_udc_data);
	at91_add_device_eth(&macb_data);
	at91_add_device_i2c(NULL, 0);
	at91_add_device_spi(nanosg20_spi_devices, ARRAY_SIZE(nanosg20_spi_devices));
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
	at91_add_device_mci(0, &mmc_data);
#else
	at91_add_device_mmc(0, &mmc_data);
#endif

	at91_gpio_leds(nanosg20_leds, ARRAY_SIZE(nanosg20_leds));
	nanosg20_add_device_nand();
	nanosg20_add_device_buttons();
	nanosg20_add_w1();

	/* enable USB host regulator */
	at91_set_GPIO_periph(AT91_PIN_PA27, 0);
	at91_set_gpio_output(AT91_PIN_PA27, 0);
}

MACHINE_START(NANOS, "NanosG20")
	/* Maintainer: taskit GmbH */
	.timer		= &at91sam926x_timer,
	.map_io		= at91_map_io,
	.init_early	= nanosg20_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= nanosg20_board_init,
MACHINE_END

