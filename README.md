Luftboot
========

This software is part of the Paparazzi UAV Project.

Luftboot is the bootloader for stm32 based autopilots to enable upgrading the
autopilot firmware using usb. It implements the dfu standard.

Luftboot is based on the usbdfu bootloader implementation by Gareth McMullin
for the Black Magic Probe project (http://www.blacksphere.co.nz/main/blackmagic).

Building
--------

Call 'make' inside the src directory.

Luftboot needs libopencm3 and an appropriate arm-none-eabi gcc cross compiler.
If you don't have these in your PATH, specify them with e.g.

    make LIBOPENCM3=path/to/libopencm3 PREFIX=path/to/bin/arm-none-eabi-

By default, Luftboot uses a 12MHz external clock to PLL it to 72MHz. If no external
clock is available, or for any other reason, one can build Luftboot to use the 8MHz
internal RC oscillator to PLL it to 48MHz. Call make as follows from inside ./src:

    make LUFTBOOT_USE_48MHZ_INTERNAL_OSC=1

Startup sequence
----------------

At bootup, Luftboot uses several criteria (in this order) to decide whether to jump
to the payload or to initiate the bootloader:
 * On boot, if the payload is NOT valid, start the bootloader
 * If the flag is set indicating we just downloaded a payload, jump to payload
 * If the payload is forcing the bootloader (using GPIO state after core-only reset)
   start the bootloader
 * If the ADC2 pin on Lisa/M is grounded, jump to payload (i.e. "skip bootloader" jumper)
 * If voltage is present on the USB vbus, start the bootloader
 * Otherwise, jump to payload
 
Notes
-----

Big bootloader 

If the bootloader ever gets bigger than 0xFFFF in the main() of sourcecode there is a set vector table base address.
 '''SCB_VTOR = APP_ADDRESS & 0xFFFF;'''

This only will work if the bootloader reserved space is less than 0xFFFF, i.e. the app address is less than 0xFFFF. This utilizes the aliased memory location of the application vector table, but it seems to work just fine using the unaliased address, i.e. just APP_ADDRESS directly.
