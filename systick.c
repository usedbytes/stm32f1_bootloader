/*
 * Copyright (C) 2017 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "systick.h"

volatile uint32_t msTicks;

void sys_tick_handler(void)
{
	msTicks++;
}

void systick_init(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(8999);
	systick_interrupt_enable();
	systick_counter_enable();
}

void delay_ms(uint32_t ms)
{
	uint32_t end = msTicks + ms;
	if (end < msTicks) {
		while (msTicks <= 0xffffffff);
	}
	while (msTicks < end);
}

void delay_us(uint32_t us)
{
	while (us--) {
		int i = 72;
		while (i--) {
			asm("nop");
		}
	}
}
