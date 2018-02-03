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
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <string.h>

#include "queue.h"
#include "util.h"
#include "spi.h"

#define SPI1_RX_DMA 2
#define SPI1_TX_DMA 3

#define SPI_N_PACKETS 32

#define DEBUG
#ifdef DEBUG
volatile char spi_trace[100];
volatile unsigned int spi_trace_idx;
void spi_dump_trace(void)
{
	unsigned int i;
	if (!spi_trace_idx)
		return;

	printf("S: %s\r\n", spi_trace);
	for (i = 0; i < sizeof(spi_trace); i++) {
		spi_trace[i] = '\0';
	}
	spi_trace_idx = 0;
	fflush(stdout);
}

static inline void __spi_trace(char c)
{
	if (spi_trace_idx < sizeof(spi_trace))
		spi_trace[spi_trace_idx++] = c;
}

static inline void __spi_trace_hex(char c)
{
	static const char *hex = "0123456789abcdef";
	__spi_trace(hex[((c & 0xF0) >> 4)]);
	__spi_trace(hex[(c & 0xF)]);
}
#else
void spi_dump_trace(void) { }
static inline void __spi_trace(char c) { (void)(c); }
static inline void __spi_trace_hex(char c) { (void)(c); }
#endif

struct spi_pl_packet_head {
	struct queue queue;
	struct spi_pl_packet *current;
	struct spi_pl_packet zero;
};

volatile bool spi_busy;
struct spi_pl_packet packet_pool[SPI_N_PACKETS];
struct spi_pl_packet_head packet_free = {
	.queue = { .last = (struct queue_node *)&packet_free },
};
struct spi_pl_packet_head packet_inbox = {
	.queue = { .last = (struct queue_node *)&packet_inbox },
};
struct spi_pl_packet_head packet_outbox = {
	.queue = { .last = (struct queue_node *)&packet_outbox },
};

#define SPI_PACKET_DMA_SIZE (offsetof(struct spi_pl_packet, crc) - offsetof(struct spi_pl_packet, id))

static inline uint32_t spi_pl_packet_dma_addr(struct spi_pl_packet *pkt)
{
	return (uint32_t)&(pkt->id);
}

static struct spi_pl_packet *spi_dequeue_packet(struct spi_pl_packet_head *list)
{
	return (struct spi_pl_packet *)queue_dequeue(&(list->queue));
}

static void spi_add_last(struct spi_pl_packet_head *list, struct spi_pl_packet *pkt)
{
	queue_enqueue(&(list->queue), (struct queue_node *)pkt);
}

static void spi_slave_init(uint32_t spidev)
{
	spi_reset(spidev);

	spi_set_dff_8bit(spidev);

	spi_set_clock_phase_0(spidev);
	spi_set_clock_polarity_0(spidev);

	spi_send_msb_first(spidev);

	spi_disable_software_slave_management(spidev);
	spi_disable_ss_output(spidev);

	spi_set_slave_mode(spidev);
}

static void prepare_tx(void)
{
	static uint8_t id = 0;

	/*
	 * Preload the data register, so we transmit the ID while setting
	 * up the DMA
	 */
	SPI_DR(SPI1) = id++;
}

static void start_tx(void)
{
	/* If we aren't re-transmitting, we need to set up the new transfer */
	struct spi_pl_packet *pkt = packet_outbox.current;
	if (!pkt) {
		pkt = spi_dequeue_packet(&packet_outbox);
		if (!pkt) {
			pkt = &packet_outbox.zero;
		}
		packet_outbox.current = pkt;
	}

	/* Plus one because DMA skips the ID */
	dma_set_memory_address(DMA1, SPI1_TX_DMA, spi_pl_packet_dma_addr(pkt) + 1);

	dma_enable_channel(DMA1, SPI1_TX_DMA);
	spi_enable_tx_dma(SPI1);
}

static void finish_tx(void)
{
	/* Disable the channel so we can modify it */
	dma_disable_channel(DMA1, SPI1_TX_DMA);
	/* Reset the counter, minus one because we don't DMA the ID */
	dma_set_number_of_data(DMA1, SPI1_TX_DMA, SPI_PACKET_DMA_SIZE - 1);

	/* If the previous transfer completed, free it */
	if (dma_get_interrupt_flag(DMA1, SPI1_TX_DMA, DMA_TCIF)) {
		struct spi_pl_packet *pkt = packet_outbox.current;
		if (pkt != &packet_outbox.zero) {
			spi_free_packet(pkt);
		}
		packet_outbox.current = NULL;
	}

	dma_clear_interrupt_flags(DMA1, SPI1_TX_DMA, DMA_TEIF | DMA_HTIF | DMA_TCIF | DMA_GIF);
}

static void prepare_rx(void)
{
	/*
	 * We can set up the new packet for receive up-front.
	 * The only downside is we might not have anything in the free-list,
	 * whereas something might have been processed and freed by the time
	 * the next transfer starts.
	 * It's not worth worrying about, this moves lots of work off the
	 * critical path.
	 */
	struct spi_pl_packet *pkt = packet_free.current;
	if (!pkt) {
		pkt = spi_alloc_packet();
		if (!pkt) {
			pkt = &packet_free.zero;
		}
		packet_free.current = pkt;
	}

	dma_set_memory_address(DMA1, SPI1_RX_DMA, spi_pl_packet_dma_addr(pkt));
}

static void start_rx(void)
{
	dma_enable_channel(DMA1, SPI1_RX_DMA);
	spi_enable_rx_dma(SPI1);
}

static void receive_packet(struct spi_pl_packet *pkt)
{
	uint8_t status = SPI_SR(SPI1);
	SPI_SR(SPI1) = 0;
	if (status & SPI_SR_CRCERR) {
		pkt->flags |= SPI_FLAG_CRCERR;
	}
	spi_add_last(&packet_inbox, pkt);
}

static void finish_rx(void)
{
	/* Disable the channel so we can modify it */
	dma_disable_channel(DMA1, SPI1_RX_DMA);
	/* Reset the counter, minus one because we don't DMA the ID */
	dma_set_number_of_data(DMA1, SPI1_RX_DMA, SPI_PACKET_DMA_SIZE);

	/* If the previous transfer completed, receive it */
	if (dma_get_interrupt_flag(DMA1, SPI1_RX_DMA, DMA_TCIF)) {
		struct spi_pl_packet *pkt = packet_free.current;
		if (pkt != &packet_free.zero) {
			receive_packet(pkt);
		}
		packet_free.current = NULL;
	}

	dma_clear_interrupt_flags(DMA1, SPI1_RX_DMA, DMA_TEIF | DMA_HTIF | DMA_TCIF | DMA_GIF);
}

static void start_transaction(void)
{
	/* Do RX first, because we've got a whole byte of time to sort out TX */
	start_rx();
	start_tx();
}

static void finish_transaction(void)
{
	/*
	 * Discard the final byte. Seems like peripheral reset doesn't clear
	 * it?
	 */
	SPI_DR(SPI1);

	finish_rx();
	finish_tx();

	/* Reset the peripheral to discard the TX DR */
	spi_slave_init(SPI1);
	spi_enable_crc(SPI1);
	spi_slave_enable(SPI1);

	prepare_rx();
	prepare_tx();
}

void exti4_isr(void)
{
	//spi_busy = !gpio_get(GPIOA, GPIO4);
#ifdef DEBUG
	led_on();
	led_off();
#endif

	EXTI_PR |= 1 << 4;

	if (!spi_busy) {
		start_transaction();
		exti_set_trigger(GPIO4, EXTI_TRIGGER_RISING);
		spi_busy = true;
	} else {
		finish_transaction();
		exti_set_trigger(GPIO4, EXTI_TRIGGER_FALLING);
		spi_busy = false;
	}
}

void spi_slave_enable(uint32_t spidev)
{
	spi_enable(spidev);
}

void spi_slave_disable(uint32_t spidev)
{
	/* Wait until not busy */
	while (SPI_SR(spidev) & SPI_SR_BSY);
	spi_disable(spidev);
}

void spi_free_packet(struct spi_pl_packet *pkt)
{
	if (!pkt)
		return;

	memset(pkt, 0, sizeof(*pkt));

	spi_add_last(&packet_free, pkt);
}

struct spi_pl_packet *spi_alloc_packet(void)
{
	/*
	 * Interesting macro - disables interrupts whilst in this function.
	 * All packet allocation in the SPI state machine is off the fast path
	 * so this shouldn't cause any troubles.
	 */
	CM_ATOMIC_CONTEXT();
	return spi_dequeue_packet(&packet_free);
}

struct spi_pl_packet *spi_receive_packet(void)
{
	return spi_dequeue_packet(&packet_inbox);
}

void spi_send_packet(struct spi_pl_packet *pkt)
{
	spi_add_last(&packet_outbox, pkt);
}

static void spi_init_dma(void)
{
	dma_channel_reset(DMA1, SPI1_RX_DMA);
	dma_disable_channel(DMA1, SPI1_RX_DMA);
	dma_set_read_from_peripheral(DMA1, SPI1_RX_DMA);
	dma_set_memory_size(DMA1, SPI1_RX_DMA, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, SPI1_RX_DMA, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, SPI1_RX_DMA);
	dma_disable_peripheral_increment_mode(DMA1, SPI1_RX_DMA);
	dma_set_peripheral_address(DMA1, SPI1_RX_DMA, (uint32_t)&(SPI_DR(SPI1)));
	dma_set_number_of_data(DMA1, SPI1_RX_DMA, SPI_PACKET_DMA_SIZE);
	dma_enable_transfer_complete_interrupt(DMA1, SPI1_RX_DMA);
	dma_enable_transfer_error_interrupt(DMA1, SPI1_RX_DMA);

	dma_channel_reset(DMA1, SPI1_TX_DMA);
	dma_disable_channel(DMA1, SPI1_TX_DMA);
	dma_set_read_from_memory(DMA1, SPI1_TX_DMA);
	dma_set_memory_size(DMA1, SPI1_TX_DMA, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, SPI1_TX_DMA, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, SPI1_TX_DMA);
	dma_disable_peripheral_increment_mode(DMA1, SPI1_TX_DMA);
	dma_set_peripheral_address(DMA1, SPI1_TX_DMA, (uint32_t)&(SPI_DR(SPI1)));
	dma_set_number_of_data(DMA1, SPI1_TX_DMA, SPI_PACKET_DMA_SIZE - 1);
	dma_enable_transfer_complete_interrupt(DMA1, SPI1_TX_DMA);
	dma_enable_transfer_error_interrupt(DMA1, SPI1_TX_DMA);
}

static void spi_init_packet_pool(void)
{
	unsigned int i;

	for (i = 0; i < (sizeof(packet_pool) / sizeof(*packet_pool)); i++) {
		struct spi_pl_packet *pkt = &packet_pool[i];
		spi_free_packet(pkt);
	}
}

void spi_dump_packet(const char *indent, struct spi_pl_packet *pkt)
{
	uint8_t *c = pkt->data;
	if (pkt) {
		printf("%s%p %d %d %d %02x\r\n", indent, pkt, pkt->id, pkt->type,
				pkt->nparts, pkt->flags);
		printf("%s  ", indent);
		while (*c && (c < pkt->data + sizeof(pkt->data))) {
			printf("%02x ", *c);
			c++;
		}
		printf("\r\n");
		printf("%s crc: %02x\r\n", indent, pkt->crc);
		printf("%s next: %p\r\n", indent, pkt->next);
	} else {
		printf("(nil)\r\n");
	}
}

void spi_dump_lists(void)
{
	printf("Free:\r\n");
	dump_queue(&packet_free.queue);
	printf("Outbox:\r\n");
	dump_queue(&packet_outbox.queue);
	printf("Inbox:\r\n");
	dump_queue(&packet_inbox.queue);
}

void spi_init(void)
{
	spi_init_dma();
	spi_init_packet_pool();

	spi_slave_init(SPI1);
	spi_enable_crc(SPI1);

	exti_select_source(GPIO4, GPIOA);
	exti_set_trigger(GPIO4, EXTI_TRIGGER_FALLING);
	exti_enable_request(GPIO4);
	nvic_enable_irq(NVIC_EXTI4_IRQ);

	/* SPI1 GPIOs in slave mode */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
	              GPIO4 | GPIO5 | GPIO7);
	//gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
	//              GPIO4);
	//gpio_clear(GPIOA, GPIO4);

	/* Set up the first transfer */
	prepare_rx();
	prepare_tx();
}
