/*
 * Portions based on the libopencm3 example: part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2013 Joshua Harlan Lifton <joshua.harlan.lifton@gmail.com>
 * Copyright (C) 2016 Brian Starkey <stark3y@gmail.com>
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

#ifndef __USB_CDC_H__
#define __USB_CDC_H__
#include <stdbool.h>
#include <stdlib.h>

void usb_cdc_init(void);

int usb_usart_recv(char *buf, size_t len, int timeout);
void usb_usart_send(const char *buf, size_t len);
void usb_usart_print(const char *str);
void usb_usart_flush_rx(void);
bool usb_usart_dtr(void);

#endif /* __USB_CDC_H__ */
