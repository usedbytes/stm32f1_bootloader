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

#ifndef __SPI_H__
#define __SPI_H__

#include "queue.h"

#define SPI_PACKET_DATA_LEN 32
struct spi_pl_packet {
	struct queue_node *next;
	uint8_t id;
	uint8_t type;
	uint8_t nparts;
#define SPI_FLAG_CRCERR (1 << 0)
#define SPI_FLAG_ERROR SPI_FLAG_CRCERR
	uint8_t flags;
	uint8_t data[SPI_PACKET_DATA_LEN];
	uint8_t crc;
};

void spi_init(void);
void spi_slave_enable(uint32_t spidev);
void spi_slave_disable(uint32_t spidev);

void spi_free_packet(struct spi_pl_packet *pkt);
struct spi_pl_packet *spi_alloc_packet(void);
struct spi_pl_packet *spi_receive_packet(void);
void spi_send_packet(struct spi_pl_packet *pkt);

void spi_dump_packet(const char *indent, struct spi_pl_packet *pkt);
void spi_dump_lists(void);
void spi_dump_trace(void);
#endif /* __SPI_H__ */
