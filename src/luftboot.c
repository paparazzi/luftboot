/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2012 Sergey Krukowski <softsr@yahoo.de>
 * Copyright (C) 2014 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file luftboot.c
 * Bootloader for Lisa/Lia MX boards with STM32F4 chip.
 * Based on KroozSD bootloader
 * see https://github.com/softsr/libopencm3/tree/master/examples/stm32/f4/krooz/usb_dfu
 * for original file
 */

#include <stdint.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

#include <libopencm3/cm3/systick.h>


#define APP_ADDRESS 0x08004000
#define SECTOR_SIZE 2048

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR 0x21
#define CMD_ERASE 0x41

#define USER_AP_WP 0x20000

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[SECTOR_SIZE];

uint32_t sector_addr[12] = {0x08000000, 0x08004000, 0x08008000, 0x0800C000,
    0x08010000, 0x08020000, 0x08040000,
    0x08060000, 0x08080000, 0x080A0000,
    0x080C0000, 0x080E0000};
uint16_t sector_erase_time[12]= {500, 1000, 500, 500,
    1000, 1500, 1500,
    1500, 1500, 1500,
    1500, 1500};
uint8_t sector_num = 1;

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

inline char *get_dev_unique_id(char *serial_no);
void led_set(int id, int on);
static inline void led_advance(void);

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

static char serial_no[25];

static const char *usb_strings[] = {
    "Paparazzi UAV",
    "Lisa/Lia MX (Upgrade) ",
    serial_no,
    /* This string is used by ST Microelectronics' DfuSe utility. */
    "@Internal Flash   /0x08000000/8*001Ka,56*001Kg",
};

/**
 * Serial is 96bit so 12bytes so 12 hexa numbers, or 24 decimal + termination character
 */
inline char *get_dev_unique_id(char *s)
{
  volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFF7A10;

  int i;

  // Fetch serial number from chip's unique ID
  for(i = 0; i < 24; i+=2) {
    s[i] = ((*unique_id >> 4) & 0xF) + '0';
    s[i+1] = (*unique_id++ & 0xF) + '0';
  }
  for(i = 0; i < 24; i++)
    if(s[i] > '9') {
      s[i] += 'A' - '9' - 1;
    }
  // add termination character
  s[24] = '\0';

  return s;
}

static uint8_t usbdfu_getstatus(usbd_device *usbd_dev, uint32_t *bwPollTimeout)
{
  (void)usbd_dev;

  switch (usbdfu_state) {
  case STATE_DFU_DNLOAD_SYNC:
    usbdfu_state = STATE_DFU_DNBUSY;
    *bwPollTimeout = 70;

    if (prog.blocknum == 0 && prog.buf[0] == CMD_ERASE)
      if(*(uint32_t *)(prog.buf + 1) == sector_addr[sector_num])
        *bwPollTimeout = sector_erase_time[sector_num];

    return DFU_STATUS_OK;
  case STATE_DFU_MANIFEST_SYNC:
    /* Device will reset when read is complete. */
    usbdfu_state = STATE_DFU_MANIFEST;
    return DFU_STATUS_OK;
  default:
    return DFU_STATUS_OK;
  }
}

static void usbdfu_getstatus_complete(usbd_device *usbd_dev, struct usb_setup_data *req)
{
  int i;
  uint32_t addr;
  (void)req;
  (void)usbd_dev;

  switch (usbdfu_state) {
  case STATE_DFU_DNBUSY:
    flash_unlock();
    if (prog.blocknum == 0) {
      switch (prog.buf[0]) {
      case CMD_ERASE:
        addr = *(uint32_t *)(prog.buf + 1);
        /* Unprotect user application area */
        if(addr == APP_ADDRESS) {
          flash_program_option_bytes(FLASH_OPTCR | USER_AP_WP);
        }
        if(addr == sector_addr[sector_num] && (addr >= APP_ADDRESS)) {
          flash_erase_sector((sector_num&0x0F) << 3, 2<<8);
          sector_num++;
        }
      case CMD_SETADDR:
        prog.addr = *(uint32_t *)(prog.buf + 1);
      }
    } else {

      uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) *
          dfu_function.wTransferSize);
      if(baseaddr >= APP_ADDRESS) {

        for (i = 0; i < prog.len; i += 4)
          flash_program_word(baseaddr + i,
              *(uint32_t *)(prog.buf + i));
      }
    }
    flash_lock();

    /* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
    usbdfu_state = STATE_DFU_DNLOAD_IDLE;
    return;
  case STATE_DFU_MANIFEST:
    /* USB device must detach, we just reset... */
    // add flash read protect
    // add flash write protect
    // save data to BACKUP regisyter
    scb_reset_system();
    return; /* Will never return. */
  default:
    return;
  }
}

static int usbdfu_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
  if ((req->bmRequestType & 0x7F) != 0x21)
    return 0; /* Only accept class request. */
  switch (req->bRequest) {
  case DFU_DNLOAD:
    if ((len == NULL) || (*len == 0)) {
      usbdfu_state = STATE_DFU_MANIFEST_SYNC;
      return 1;
    } else {
      /* Copy download data for use on GET_STATUS. */
      prog.blocknum = req->wValue;
      prog.len = *len;
      memcpy(prog.buf, *buf, *len);
      usbdfu_state = STATE_DFU_DNLOAD_SYNC;
      return 1;
    }
  case DFU_CLRSTATUS:
    /* Clear error and return to dfuIDLE. */
    if (usbdfu_state == STATE_DFU_ERROR)
      usbdfu_state = STATE_DFU_IDLE;
    return 1;
  case DFU_ABORT:
    /* Abort returns to dfuIDLE state. */
    usbdfu_state = STATE_DFU_IDLE;
    return 1;
  case DFU_UPLOAD:
    /* Upload not supported for now. */
    return 0;
  case DFU_GETSTATUS:
  {
    uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
    (*buf)[0] = usbdfu_getstatus(usbd_dev, &bwPollTimeout);
    (*buf)[1] = bwPollTimeout & 0xFF;
    (*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
    (*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
    (*buf)[4] = usbdfu_state;
    (*buf)[5] = 0; /* iString not used here */
    *len = 6;
    *complete = usbdfu_getstatus_complete;
    return 1;
  }
  case DFU_GETSTATE:
    /* Return state with no state transision. */
    *buf[0] = usbdfu_state;
    *len = 1;
    return 1;
  }

  return 0;
}

bool gpio_force_bootloader()
{
  /* Enable clock for the "force bootloader" pin bank and check for it */
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9);
  gpio_clear(GPIOA, GPIO9);

  if(gpio_get(GPIOA, GPIO9)) {
    rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    return true;
  }
  rcc_peripheral_disable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
  return false;
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

void sys_tick_handler()
{
  led_advance();
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

int main(void)
{
  if (!(gpio_force_bootloader() && 1)
      || (FLASH_OPTCR & USER_AP_WP)
  )
  {
    /* Boot the application if it's valid. */
    {
      /* Write protect user application area */
      flash_program_option_bytes(FLASH_OPTCR & ~USER_AP_WP);
      /* Set vector table base address. */
      SCB_VTOR = APP_ADDRESS & 0xFFFF;
      /* Initialise master stack pointer. */
      asm volatile("msr msp, %0"::"g"
          (*(volatile uint32_t *)APP_ADDRESS));
      /* Jump to application. */
      (*(void (**)())(APP_ADDRESS + 4))();
    }
  }

  usbd_device *usbd_dev;

  rcc_clock_setup_hse_3v3(&hse_12mhz_3v3[CLOCK_3V3_48MHZ]);

  //rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

  /* Enable GPIOA, GPIOB, GPIOC, and AFIO clocks. */
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN |
      RCC_AHB1ENR_IOPBEN |
      RCC_AHB1ENR_IOPCEN);

  //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
  //gpio_clear(GPIOA, GPIO13);
  //gpio_set(GPIOA, GPIO14 | GPIO15);

  /* Write protect bootloader sector if not yet */
  if((FLASH_OPTCR & 0x10000)) {
    flash_program_option_bytes(FLASH_OPTCR & ~0x10000);
  }

  rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
      GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

  /* LED1 */
  /* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
  /* Preconfigure the LEDs. */
  gpio_clear(GPIOA, GPIO8); /* LED1 On */

  /* LED 2*/
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
        GPIO4);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO4);
  gpio_clear(GPIOB, GPIO4); /* JTAG_TRST On */

  /* LED3-5 */
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
      GPIO2 | GPIO5 | GPIO15);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO2 | GPIO5 | GPIO15);
  gpio_clear(GPIOC, GPIO2);
  gpio_clear(GPIOC, GPIO5);
  gpio_clear(GPIOC, GPIO15);

  /* Sys tick*/
  rcc_periph_clock_enable(RCC_OTGFS);

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

  systick_set_reload(700000);
  systick_interrupt_enable();
  systick_counter_enable();


  /* USB */
  get_dev_unique_id(serial_no);

  usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 4, usbd_control_buffer,
      sizeof(usbd_control_buffer));
  usbd_register_control_callback(
      usbd_dev,
      USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
      usbdfu_control_request);

  while (1)
    usbd_poll(usbd_dev);
}


