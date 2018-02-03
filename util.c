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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "util.h"
#include "systick.h"

void led_on(void)
{
	gpio_clear(GPIOC, GPIO13);
}

void led_off(void)
{
	gpio_set(GPIOC, GPIO13);
}

void blink_us(uint32_t ontime)
{
	led_on();
	delay_us(ontime);
	led_off();
	delay_us(2);
}

void panic(void)
{
	while (1) {
		delay_ms(500);
		gpio_toggle(GPIOC, GPIO13);
	}
}

void hard_fault_handler(void)
{
	panic();
}

void bus_fault_handler(void)
{
	panic();
}

void usage_fault_handler(void)
{
	panic();
}

