#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <string.h>
#include <stdio.h>

#include "spi.h"
#include "usb_cdc.h"

#include "systick.h"

#define DEBUG(...) printf(__VA_ARGS__)

#define ERROR_PKT_TYPE 0xff
struct error_pkt {
	uint8_t id;
	uint8_t pad[3];
	char str[0];
};

#define ACK_PKT_TYPE 0x1
struct ack_pkt {
	uint8_t id;
	uint8_t pad[3];
};

#define SYNC_PKT_TYPE 0x2
struct sync_pkt {
	uint8_t id;
	uint8_t pad[3];
	uint32_t cookie;
};

#define ERASE_PKT_TYPE 0x3
struct erase_pkt {
	uint32_t address;
};

#define WRITE_PKT_TYPE 0x4
struct write_pkt {
	uint32_t address;
	uint32_t len;
	uint32_t crc;
	uint8_t data[0];
};

#define READREQ_PKT_TYPE 0x5
struct readreq_pkt {
	uint32_t address;
	uint32_t len;
	uint8_t data[0];
};

#define READRESP_PKT_TYPE 0x6
struct readresp_pkt {
	uint32_t address;
	uint32_t len;
	uint32_t crc;
	uint8_t data[0];
};

static void setup_irq_priorities(void)
{
	struct map_entry {
		uint32_t irqn;
		uint8_t  prio;
	} map[] = {
		{ NVIC_EXTI4_IRQ,           (0 << 6) | (0 << 4) },
		{ NVIC_TIM4_IRQ,            (1 << 6) | (0 << 4) },
		{ NVIC_USB_LP_CAN_RX0_IRQ,  (2 << 6) | (0 << 4) },
		{ NVIC_USB_WAKEUP_IRQ,      (2 << 6) | (1 << 4) },
		{ NVIC_TIM3_IRQ,            (3 << 6) | (0 << 4) },
		{ 0, 0 }
	}, *p = map;

	/*
	 * Priority ordering to try and make SPI reliable...
	 * stm32f103 only implements 4 bits of priority!
	 * Interrupt priority grouping (2 bits of pre-empt):
	 *   7:6 - pre-emption
	 *   5:4 - priority
	 *   3:0 - unused
	 */
	scb_set_priority_grouping(5 << 8);
	while (p->irqn) {
		nvic_set_priority(p->irqn, p->prio);
		p++;
	};
}

static void setup_gpio(void) {
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

static void report_error(uint8_t id, const char *str)
{
	// len(str) + NUL + header, aligned up to packet size
	unsigned npkts = (strlen(str) + 1 + 4 + (SPI_PACKET_DATA_LEN - 1)) / SPI_PACKET_DATA_LEN;
	bool first = true;

	DEBUG("Report error: %d %s\r\n", id, str);

	while (npkts--) {
		uint8_t *p;
		unsigned int ndata = SPI_PACKET_DATA_LEN;
		struct spi_pl_packet *pkt = spi_alloc_packet();
		if (!pkt) {
			// panic?
			return;
		}

		pkt->type = ERROR_PKT_TYPE;
		pkt->nparts = npkts;
		p = pkt->data;

		if (first) {
			struct error_pkt *err = (struct error_pkt *)pkt->data;
			first = false;
			err->id = id;
			ndata -= 4;
			p += 4;
		}

		while (*str && ndata) {
			*p = *str;
			p++; str++;
			ndata--;
		}

		if (ndata) {
			*p = *str;
		}

		spi_send_packet(pkt);
	}
}

static void process_sync_pkt(struct spi_pl_packet *pkt)
{
	struct sync_pkt *payload = (struct sync_pkt *)pkt->data;

	if (pkt->nparts) {
		report_error(pkt->id, "Unexpected nparts on sync pkt");
	}

	pkt->id = 0;
	pkt->type = SYNC_PKT_TYPE;
	pkt->nparts = 0;
	pkt->flags = 0;
	pkt->crc = 0;

	payload->id = pkt->id;

	spi_send_packet(pkt);
}

static void ep0xfe_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 0xfe) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	scb_reset_system();
	spi_free_packet(pkt);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_DMA1);

	systick_init();
	setup_gpio();

	usb_cdc_init();

	spi_init();
	spi_slave_enable(SPI1);

	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO14 | GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO14);
	gpio_set(GPIOC, GPIO14);
	gpio_clear(GPIOC, GPIO15);

	gpio_set(GPIOC, GPIO13);

	setup_irq_priorities();

	struct spi_pl_packet *pkt;
	uint32_t time = msTicks;
	while (1) {
		while ((pkt = spi_receive_packet())) {
			if (pkt->flags & SPI_FLAG_CRCERR) {
				report_error(pkt->id, "CRC Error.");
				spi_free_packet(pkt);
				continue;
			}
			switch (pkt->type) {
				case 0:
					spi_free_packet(pkt);
					break;
				case SYNC_PKT_TYPE:
					process_sync_pkt(pkt);
					break;
				case 0xfe:
					ep0xfe_process_packet(pkt);
					break;
				default:
					report_error(pkt->id, "Unknown type. But lets make this error.");
					spi_free_packet(pkt);
			}
		}
	}

	return 0;
}
