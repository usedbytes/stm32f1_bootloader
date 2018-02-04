/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Modified 2018 Brian Starkey <stark3y@gmail.com> to use libopencm3 constructs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

/**
 *  @file hardware.c
 *
 *  @brief init routines to setup clocks, interrupts, also destructor functions.
 *  does not include USB stuff. EEPROM read/write functions.
 *
 */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware.h"

void systemReset(void) {
	RCC_CR |= 0x00000001;
	RCC_CFGR &= 0xF8FF0000;
	RCC_CR &= 0xFEF6FFFF;
	RCC_CR &= 0xFFFBFFFF;
	RCC_CFGR &= 0xFF80FFFF;

	RCC_CIR = 0x00000000;  /* disable all RCC interrupts */
}

bool checkUserCode(uint32_t usrAddr) {
	uint32_t sp = *(volatile uint32_t *) usrAddr;

	if ((sp & 0x2FFE0000) == 0x20000000) {
		return true;
	} else {
		return false;
	}
}

static void setMspAndJump(uint32_t usrAddr) {
	// Dedicated function with no call to any function (appart the last call)
	// This way, there is no manipulation of the stack here, ensuring that GGC
	// didn't insert any pop from the SP after having set the MSP.
	typedef void (*funcPtr)(void);
	uint32_t jumpAddr = *(volatile uint32_t *)(usrAddr + 0x04); /* reset ptr in vector table */

	funcPtr usrMain = (funcPtr) jumpAddr;

	SCB_VTOR = (volatile uint32_t) (usrAddr);

	asm volatile("msr msp, %0"::"g"
			(*(volatile uint32_t *)usrAddr));

	usrMain();                                /* go! */
}

static void usbPowerOff(void) {
	SET_REG(USB_CNTR_REG, USB_CNTR_FRES);
	SET_REG(USB_ISTR_REG, 0);
	SET_REG(USB_CNTR_REG, USB_CNTR_FRES | USB_CNTR_PWDN);
}

void jumpToUser(uint32_t usrAddr) {

	/* tear down all the dfu related setup */
	// disable usb interrupts, clear them, turn off usb, set the disc pin
	// todo pick exactly what we want to do here, now its just a conservative
	nvicDisableInterrupts();

	usbPowerOff();

	// Does nothing, as PC12 is not connected on teh Maple mini according to the schemmatic     setPin(GPIOC, 12); // disconnect usb from host. todo, macroize pin
	systemReset(); // resets clocks and periphs, not core regs

	setMspAndJump(usrAddr);
}

void nvicDisableInterrupts() {
	NVIC_ICER(0) = 0xFFFFFFFF;
	NVIC_ICER(1) = 0xFFFFFFFF;
	NVIC_ICPR(0) = 0xFFFFFFFF;
	NVIC_ICPR(1) = 0xFFFFFFFF;

	STK_CSR = 0x04; /* disable the systick, which operates separately from nvic */
}
