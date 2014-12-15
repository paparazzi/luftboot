/*
 * This file is part of the Paparazzi UAV project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011-2012 Piotr Esden-Tempski <piotr@esden.net>
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

#include <stdint.h>
#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/stm32/desig.h>

#ifndef VERSION
#define VERSION         ""
#endif

#ifndef DEV_SERIAL
#define DEV_SERIAL      "NSERIAL"
#endif

#define APP_ADDRESS	0x08002000
#define SECTOR_SIZE	2048

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

#define FLASH_OBP_RDP 0x1FFFF800
#define FLASH_OBP_WRP10 0x1FFFF808
/* Defines user option register as per Table 5 on Page 55 of RM0008 (STM32
 * Reference Manual)
 */
#define FLASH_OBP_DATA0 0x1FFFF804

#define FLASH_OBP_RDP_KEY 0x5aa5

static const char
dev_serial[] __attribute__((section (".devserial"))) = DEV_SERIAL;

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[SECTOR_SIZE];

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1D50,
	.idProduct = 0x600F,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = SECTOR_SIZE,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

/** contains DEV_SERIAL and unique chip ID.
 * chars: 7 (serial) + 1 (space) + 24 (id) + '\0'
 */
static char serial_no[7+1+24+1];

static inline char *get_serial_string(char *s);

static const char *usb_strings[] = {
	"Transition Robotics Inc.",
	"Lisa/M (Upgrade) " VERSION,
	serial_no,
	/* This string is used by ST Microelectronics' DfuSe utility */
	"@Internal Flash   /0x08000000/4*002Ka,124*002Kg"
};

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	switch(usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;

	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;

	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *device,
					  struct usb_setup_data *req)
{
	int i;
	(void)req;

	switch(usbdfu_state) {
	case STATE_DFU_DNBUSY:

		flash_unlock();
		if(prog.blocknum == 0) {
			if ((*(uint32_t*)(prog.buf+1) < 0x8002000) ||
				(*(uint32_t*)(prog.buf+1) >= 0x8040000)) {
				usbd_ep_stall_set(device, 0, 1);
				return;
			}
			switch(prog.buf[0]) {
			case CMD_ERASE:
				flash_erase_page(*(uint32_t*)(prog.buf+1));
			case CMD_SETADDR:
				prog.addr = *(uint32_t*)(prog.buf+1);
			}
		} else {
			uint32_t baseaddr = prog.addr +
				((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			for(i = 0; i < prog.len; i += 2)
				flash_program_half_word(baseaddr + i,
						*(uint16_t*)(prog.buf+i));
		}
		flash_lock();

		/* We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
		 */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;

	case STATE_DFU_MANIFEST:
		/* Mark DATA0 register that we have just downloaded the code */
		if((FLASH_OBR & 0x3FC00) != 0x00) {
		  flash_unlock();
		  FLASH_CR = 0;
		  flash_erase_option_bytes();
		  flash_program_option_bytes(FLASH_OBP_RDP, 0x5AA5);
		  flash_program_option_bytes(FLASH_OBP_WRP10, 0x03FC);
		  flash_program_option_bytes(FLASH_OBP_DATA0, 0xFF00);
		  flash_lock();
		}
		/* USB device must detach, we just reset... */
		scb_reset_system();
		return; /* Will never return */
	default:
		return;
	}
}

static int usbdfu_control_request(usbd_device *device,
				  struct usb_setup_data *req, uint8_t **buf,
				  uint16_t *len,
				  void (**complete)(usbd_device *device,
						struct usb_setup_data *req))
{

	if((req->bmRequestType & 0x7F) != 0x21)
		return 0; /* Only accept class request */

	switch(req->bRequest) {
	case DFU_DNLOAD:
		if((len == NULL) || (*len == 0)) {
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return 1;
		} else {
			/* Copy download data for use on GET_STATUS */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return 1;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE */
		if(usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state */
		usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_UPLOAD:
		/* Upload not supported for now */
		return 0;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec
						 */

		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0;	/* iString not used here */
		*len = 6;

		*complete = usbdfu_getstatus_complete;

		return 1;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision */
		*buf[0] = usbdfu_state;
		*len = 1;
		return 1;
	}

	return 0;
}

static inline void gpio_init(void)
{
	/* Enable GPIOA, GPIOB, GPIOC, and AFIO clocks. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

	/* LED1 */
	/* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	/* JTAG_TRST */
	/* Set GPIO4 (in GPIO port B) to 'output push-pull'. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

	/* LED2, ADC4, ADC6 */
	/* Set GPIO15, GPIO5, GPIO2 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO15 | GPIO5 | GPIO2);

	/* Preconfigure the LEDs. */
	gpio_set(GPIOA, GPIO8);
	gpio_set(GPIOB, GPIO4);
	gpio_set(GPIOC, GPIO15 | GPIO5 | GPIO2);
}

void led_set(int id, int on)
{
	if (on) {
		switch (id) {
			case 0:
				gpio_clear(GPIOA, GPIO8); /* LED1 On */
				break;
			case 1:
				gpio_clear(GPIOB, GPIO4); /* JTAG_TRST On */
				break;
			case 2:
				gpio_clear(GPIOC, GPIO2); /* ADC6 On */
				break;
			case 3:
				gpio_clear(GPIOC, GPIO5); /* ADC4 On */
				break;
			case 4:
				gpio_clear(GPIOC, GPIO15); /* LED2 On */
				break;
		}
	} else {
		switch (id) {
			case 0:
				gpio_set(GPIOA, GPIO8); /* LED1 On */
				break;
			case 1:
				gpio_set(GPIOB, GPIO4); /* JTAG_TRST On */
				break;
			case 2:
				gpio_set(GPIOC, GPIO2); /* ADC6 On */
				break;
			case 3:
				gpio_set(GPIOC, GPIO5); /* ADC4 On */
				break;
			case 4:
				gpio_set(GPIOC, GPIO15); /* LED2 On */
				break;
		}
	}
}

static inline void led_advance(void)
{
	static int state = 0;

	if (state < 5) {
		led_set(state, 1);
	} else if (state < 10) {
		led_set(state - 5, 0);
	} else if (state < 15) {
		led_set(14 - state, 1);
	} else if (state < 20) {
		led_set(19 - state, 0);
	}

	state++;
	if(state == 20) state = 0;

}

bool gpio_force_bootloader()
{
	/* Force the bootloader if the GPIO state was changed to indicate this
		  in the application (state remains after a core-only reset)
	   Skip bootloader if the "skip bootloader" pin is grounded
	   Force bootloader if the USB vbus is powered
	   Skip bootloader otherwise */

	/* Check if we are being forced by the payload. */
	if (((GPIO_CRL(GPIOC) & 0x3) == 0x0) &&
		((GPIO_CRL(GPIOC) & 0xC) == 0x8) &&
		((GPIO_IDR(GPIOC) & 0x1) == 0x0)){
		return true;
	} else {
		/* Enable clock for the "skip bootloader" pin bank and check
		 * for it
		 */
		rcc_periph_clock_enable(RCC_GPIOC);
		gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
					  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
		gpio_set(GPIOC, GPIO0);

		if (!gpio_get(GPIOC, GPIO0)) {
			/* If pin grounded, disable the pin bank and return */
			gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
						  GPIO_CNF_INPUT_FLOAT, GPIO0);
			rcc_periph_clock_disable(RCC_GPIOC);
			return false;
		}
		/* Disable the pin bank */
		gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
					  GPIO0);
		rcc_periph_clock_disable(RCC_GPIOC);

		/* Enable clock for the "USB vbus" pin bank and check for it */
		rcc_periph_clock_enable(RCC_GPIOA);
		gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
					  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO9);
		gpio_clear(GPIOA, GPIO9);

		if (gpio_get(GPIOA, GPIO9)) {
			/* If vbus pin high, disable the pin bank and return */
			gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						  GPIO_CNF_INPUT_FLOAT, GPIO9);
			rcc_periph_clock_disable(RCC_GPIOA);
			return true;
		}
		/* Disable the pin bank */
		gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
					  GPIO9);
		rcc_periph_clock_disable(RCC_GPIOA);
	}

	return false;
}

int main(void)
{
	/* Check if the application is valid. */
	if ((*(volatile uint32_t *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
		/* Check if we have just downloaded the new code by looking at
		 * the DATA0 option register or that we do NOT want to force
		 * the bootloader
		 */
		if (((FLASH_OBR & 0x3FC00) == 0x00) ||
			(!gpio_force_bootloader() && 1)) {
			/* If we DID just download new code, reset that data
			 * register
			 */
			if((FLASH_OBR & 0x3FC00) == 0x00) {
				flash_unlock();
				FLASH_CR = 0;
				flash_erase_option_bytes();
				/* Flash read unprotect */
				flash_program_option_bytes(FLASH_OBP_RDP, 0x5AA5);
				/* Write protect first 4 flash pages */
				flash_program_option_bytes(FLASH_OBP_WRP10, 0x03FC);
				/* Write data register that we downloaded the code
				 * and want to jump the app
				 */
				flash_program_option_bytes(FLASH_OBP_DATA0,
							   0x00FF);
				flash_lock();
			}
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;
			/* Initialise master stack pointer. */
			asm volatile("msr msp, %0"::"g"
					 (*(volatile uint32_t *)APP_ADDRESS));
			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
		}
	}

	if ((FLASH_WRPR & 0x03) != 0x00) {
		flash_unlock();
		FLASH_CR = 0;
		flash_erase_option_bytes();
		flash_program_option_bytes(FLASH_OBP_RDP, FLASH_OBP_RDP_KEY);
		flash_program_option_bytes(FLASH_OBP_WRP10, 0x03FC);
	}

#if LUFTBOOT_USE_48MHZ_INTERNAL_OSC
#pragma message "Luftboot using 8MHz internal RC oscillator to PLL it to 48MHz."
	rcc_clock_setup_in_hsi_out_48mhz();
#else
#pragma message "Luftboot using 12MHz external clock to PLL it to 72MHz."
	rcc_clock_setup_in_hse_12mhz_out_72mhz();
#endif

	rcc_periph_clock_enable(RCC_OTGFS);

	gpio_init();

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);
	systick_interrupt_enable();
	systick_counter_enable();

	/* Get serial number */
	get_serial_string(serial_no);

	usbd_device *device = usbd_init(&stm32f107_usb_driver, &dev, &config,
					usb_strings, 4, usbd_control_buffer,
					sizeof(usbd_control_buffer));
	usbd_register_control_callback( device,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);

	while (1)
		usbd_poll(device);
}

/** get serial number as combination of DEV_SERIAL and unique chip ID.
 * first 7 chars are DEV_SERIAL, space, then 24 chars unique chip ID.
 */
static inline char *get_serial_string(char *s)
{
	int i;
	for(i = 0; i < 7; i++) {
		s[i] = dev_serial[i];
	}
	s[i] = ' ';
	desig_get_unique_id_as_string(&s[8], 25);

	return s;
}

void sys_tick_handler()
{
	led_advance();
}
