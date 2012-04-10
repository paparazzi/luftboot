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

#include <string.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

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

static const char dev_serial[] __attribute__ ((section (".devserial"))) = DEV_SERIAL;

/* We need a special large control buffer for this device: */
u8 usbd_control_buffer[SECTOR_SIZE];

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

inline char *get_dev_unique_id(char *serial_no);

static struct {
	u8 buf[sizeof(usbd_control_buffer)];
	u16 len;
	u32 addr;
	u16 blocknum;
} prog;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0xDF11,
	.bcdDevice = 0x0200,
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

static char serial_no[24+8];

static const char *usb_strings[] = {
	"x",
	"Transition Robotics Inc.",
	"Lisa/M (Upgrade) " VERSION,
	serial_no,
	/* This string is used by ST Microelectronics' DfuSe utility */
	"@Internal Flash   /0x08000000/4*002Ka,124*002Kg"
};

static u8 usbdfu_getstatus(u32 *bwPollTimeout)
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

static void usbdfu_getstatus_complete(struct usb_setup_data *req)
{
	int i;
	(void)req;

	switch(usbdfu_state) {
	case STATE_DFU_DNBUSY:

		flash_unlock();
		if(prog.blocknum == 0) {
			switch(prog.buf[0]) {
			case CMD_ERASE:
				flash_erase_page(*(u32*)(prog.buf+1));
			case CMD_SETADDR:
				prog.addr = *(u32*)(prog.buf+1);
			}
		} else {
			u32 baseaddr = prog.addr +
				((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			for(i = 0; i < prog.len; i += 2)
				flash_program_half_word(baseaddr + i,
						*(u16*)(prog.buf+i));
		}
		flash_lock();

		/* We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
		 */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;

	case STATE_DFU_MANIFEST:
		/* USB device must detach, we just reset... */
		scb_reset_system();
		return; /* Will never return */
	default:
		return;
	}
}

static int usbdfu_control_request(struct usb_setup_data *req, u8 **buf,
		u16 *len, void (**complete)(struct usb_setup_data *req))
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
		u32 bwPollTimeout = 0; /* 24-bit integer in DFU class spec */

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
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
						  RCC_APB2ENR_IOPBEN |
						  RCC_APB2ENR_IOPCEN |
						  RCC_APB2ENR_AFIOEN);
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
	/* Check if we are being forced by the payload. */
	if (((GPIO_CRL(GPIOC) & 0x3) == 0x0) &&
	    ((GPIO_CRL(GPIOC) & 0xC) == 0x8) &&
	    ((GPIO_IDR(GPIOC) & 0x1) == 0x0)){
		return true;
	} else {
		/* Enable clock for the "force bootloader" pin bank and check for it */
		rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
		gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
		gpio_set(GPIOC, GPIO0);

		if(!gpio_get(GPIOC, GPIO0)) {
			return true;
		}
	}

	return false;
}

int main(void)
{
	if(!gpio_force_bootloader() && 1) {
		/* Boot the application if it's valid */
		if((*(volatile u32*)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
			/* Set vector table base address */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;
			/* Initialise master stack pointer */
			asm volatile ("msr msp, %0"::"g"
					(*(volatile u32*)APP_ADDRESS));
			/* Jump to application */
			(*(void(**)())(APP_ADDRESS + 4))();
		}
	}


	rcc_clock_setup_in_hse_12mhz_out_72mhz();

	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);

	gpio_init();

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);
	systick_interrupt_enable();
	systick_counter_enable();

	get_dev_unique_id(serial_no);

	usbd_init(&stm32f107_usb_driver, &dev, &config, usb_strings);
	usbd_set_control_buffer_size(sizeof(usbd_control_buffer));
	usbd_register_control_callback(
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);

	while (1)
		usbd_poll();
}

inline char *get_dev_unique_id(char *s)
{
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	int i;

	for(i = 0; i < 7; i++) {
		s[i] = dev_serial[i];
	}
	s[i] = ' ';

	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 24; i+=2) {
		s[i+8] = ((*unique_id >> 4) & 0xF) + '0';
		s[i+8+1] = (*unique_id++ & 0xF) + '0';
	}
	for(i = 0; i < 24; i++)
		if(s[i+8] > '9')
			s[i+8] += 'A' - '9' - 1;

	return s;
}

void sys_tick_handler()
{
	led_advance();
}
