#include <libopencm3/stm32/crc.h>
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

/*
 * Helper to stream data into a series of packets.
 *
 * into:     should point to the first packet - which may alredy have been set
 *           up with whatever header type is required.
 * offset:   is the offset into the first packet where the data should start
 *           (i.e. the size of the header for this packet type).
 * type:     is used to set the packet type for all the packets.
 *
 * NOTE: There must be enough packets in the free-list to take all the data!
 */
static void packetise_stream(struct spi_pl_packet *into, uint8_t offset, uint8_t type, const char *data, uint32_t len)
{
	unsigned npkts = (len + offset + (SPI_PACKET_DATA_LEN - 1)) / SPI_PACKET_DATA_LEN;
	unsigned int ndata = SPI_PACKET_DATA_LEN - offset;
	uint8_t *p = into->data + offset;

	while (npkts--) {
		into->type = type;
		into->nparts = npkts;

		while (len && ndata) {
			*p = *data;
			p++; data++;
			len--; ndata--;
		}

		spi_send_packet(into);

		into = spi_alloc_packet();
		if (!into) {
			// Panic?
			return;
		}
		p = into->data;
		ndata = SPI_PACKET_DATA_LEN;
	}
}

static void report_error(uint8_t id, const char *str)
{

	struct error_pkt *err;
	struct spi_pl_packet *pkt = spi_alloc_packet();

	DEBUG("Report error: %d %s\r\n", id, str);

	if (!pkt) {
		// Panic?
		return;
	}

	err = (struct error_pkt *)pkt->data;
	err->id = id;

	packetise_stream(pkt, 4, ERROR_PKT_TYPE, str, strlen(str) + 1);
}

static void process_sync_pkt(struct spi_pl_packet *pkt)
{
	struct sync_pkt *payload = (struct sync_pkt *)pkt->data;

	if (pkt->nparts) {
		report_error(pkt->id, "Unexpected nparts on sync pkt");
		return;
	}

	pkt->id = 0;
	pkt->type = SYNC_PKT_TYPE;
	pkt->nparts = 0;
	pkt->flags = 0;
	pkt->crc = 0;

	payload->id = pkt->id;

	spi_send_packet(pkt);
}

static void process_readreq_pkt(struct spi_pl_packet *pkt)
{
	struct spi_pl_packet *resp;
	struct readresp_pkt *resp_pl;
	struct readreq_pkt *payload = (struct readreq_pkt *)pkt->data;
	if (pkt->nparts) {
		report_error(pkt->id, "Unexpected nparts on readreq pkt");
		return;
	}

	DEBUG("Read %ld bytes from %08lx\r\n", payload->len, payload->address);

	if (payload->address & 0x3) {
		report_error(pkt->id, "Read address must be word-aligned");
		return;
	}

	if (payload->len & 0x3) {
		report_error(pkt->id, "Read length must be word-aligned");
		return;
	}

	// XXX: We could sanitise address and length

	resp = spi_alloc_packet();
	if (!resp) {
		DEBUG("No packet for response\r\n");
		// Panic?
		return;
	}

	resp_pl = (struct readresp_pkt *)resp->data;
	resp_pl->address = payload->address;
	resp_pl->len = payload->len;

	crc_reset();
	resp_pl->crc = crc_calculate_block((uint32_t *)payload->address, payload->len);

	DEBUG("CRC: %08lx\r\n", resp_pl->crc);
	payload = NULL;
	spi_free_packet(pkt);

	packetise_stream(resp, offsetof(struct readresp_pkt, data), READRESP_PKT_TYPE, (char *)resp_pl->address, resp_pl->len);
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
	rcc_periph_clock_enable(RCC_CRC);

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
				case READREQ_PKT_TYPE:
					process_readreq_pkt(pkt);
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
