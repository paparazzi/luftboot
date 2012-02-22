/*
 * This file is part of the Paparazzi UAV project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>

/* Set STM32 to 72 MHz. */
void clock_setup(void)
{
	rcc_clock_setup_in_hse_12mhz_out_72mhz();

	/* Enable GPIOB, GPIOC, and AFIO clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
}

void gpio_setup(void)
{
	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);

	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

	/* preconfigure the led's */
	gpio_clear(GPIOA, GPIO8); /* switch on led */
	gpio_set(GPIOB, GPIO4); /* switch off led */
	gpio_clear(GPIOC, GPIO15); /* switch on led */
}

int main(void)
{
	int i;
	unsigned char counter = 0;

	clock_setup();
	gpio_setup();

	counter = 0;

	/* Full bank blink to indicate reset */

	for (counter=0; counter < 2; counter++) {
		for (i = 0; i < 800000; i++)	/* Wait a bit. */
			__asm__("nop");

		gpio_clear(GPIOA, GPIO8);	/* LED on/off */
		gpio_clear(GPIOB, GPIO4);	/* LED on/off */
		gpio_clear(GPIOC, GPIO15);	/* LED on/off */
		gpio_clear(GPIOC, GPIO2);	/* LED on/off */
		gpio_clear(GPIOC, GPIO5);	/* LED on/off */

		for (i = 0; i < 800000; i++)	/* Wait a bit. */
			__asm__("nop");

		gpio_set(GPIOA, GPIO8);	        /* LED on/off */
		gpio_set(GPIOB, GPIO4);	        /* LED on/off */
		gpio_set(GPIOC, GPIO15);        /* LED on/off */
		gpio_set(GPIOC, GPIO2);	        /* LED on/off */
		gpio_set(GPIOC, GPIO5);         /* LED on/off */
	}

	counter = 0;

	/* Blink the LED (PC12) on the board. */
	while (1) {

		counter++;
		if (counter & (1 << 0)) {
			gpio_clear(GPIOA, GPIO8);	/* LED on/off */
		} else {
			gpio_set(GPIOA, GPIO8);	        /* LED on/off */
		}

		if (counter & (1 << 1)) {
			gpio_clear(GPIOB, GPIO4);	/* LED on/off */
		} else {
			gpio_set(GPIOB, GPIO4);	        /* LED on/off */
		}

		if (counter & (1 << 2)) {
			gpio_clear(GPIOC, GPIO15);	/* LED on/off */
		} else {
			gpio_set(GPIOC, GPIO15);	/* LED on/off */
		}

		if (counter & (1 << 3)) {
			gpio_clear(GPIOC, GPIO2);	/* LED on/off */
		} else {
			gpio_set(GPIOC, GPIO2);	        /* LED on/off */
		}

		if (counter & (1 << 4)) {
			gpio_clear(GPIOC, GPIO5);	/* LED on/off */
		} else {
			gpio_set(GPIOC, GPIO5);	        /* LED on/off */
		}

		for (i = 0; i < 800000; i++)	/* Wait a bit. */
			__asm__("nop");

		if (counter == 32) {
			/*
			 * Set the bootloader force pin to be input pull-down and low.
			 * After the cortex core reset this setting is being carried
			 * over to the bootloader indicating that we want to stay in
			 * the bootloader.
			 */
			gpio_clear(GPIOC, GPIO0);
			gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
				      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
			scb_reset_core(); /* reset the cortex core */
		}
	}


	return 0;
}
