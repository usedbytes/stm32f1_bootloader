#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/flash.h>
#include <string.h>

#include "hardware.h"
#include "spi.h"
#ifdef DEBUG
#include <stdio.h>
#include "usb_cdc.h"
#endif

#include "systick.h"

#define MAX_TRANSFER 512
#define DEFAULT_USER_ADDR 0x08002000

#ifdef DEBUG
#define DBG_PRINT(...) printf(__VA_ARGS__)
#else
#define DBG_PRINT(...) {}
#endif

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

#define GO_PKT_TYPE 0x7
struct go_pkt {
	uint32_t address;
};

#define QUERY_PKT_TYPE 0x8
#define QUERY_PARAM_MAX_TRANSFER 0x1
#define QUERY_PARAM_DEFAULT_USER_ADDR 0x2
struct query_pkt {
	uint32_t parameter;
};

#define QUERYRESP_PKT_TYPE 0x9
struct queryresp_pkt {
	uint32_t parameter;
	uint32_t value;
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

		if (npkts) {
			into = spi_alloc_packet();
			if (!into) {
				DBG_PRINT("Panic (packetise)\r\n");
				return;
			}
			p = into->data;
			ndata = SPI_PACKET_DATA_LEN;
		}
	}
}

static void report_error(uint8_t id, const char *str)
{

	struct error_pkt *err;
	struct spi_pl_packet *pkt = spi_alloc_packet();

	DBG_PRINT("Report error: %d %s\r\n", id, str);

	if (!pkt) {
		DBG_PRINT("Panic (error)\r\n");
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
		spi_free_packet(pkt);
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
		spi_free_packet(pkt);
		return;
	}

	DBG_PRINT("Read %ld bytes from %08lx\r\n", payload->len, payload->address);

	if (payload->address & 0x3) {
		report_error(pkt->id, "Read address must be word-aligned");
		spi_free_packet(pkt);
		return;
	}

	if (payload->len & 0x3) {
		report_error(pkt->id, "Read length must be word-aligned");
		spi_free_packet(pkt);
		return;
	}

	// XXX: We could sanitise address and length

	resp = spi_alloc_packet();
	if (!resp) {
		DBG_PRINT("No packet for response\r\n");
		spi_free_packet(pkt);
		return;
	}

	resp_pl = (struct readresp_pkt *)resp->data;
	resp_pl->address = payload->address;
	resp_pl->len = payload->len;

	crc_reset();
	resp_pl->crc = crc_calculate_block((uint32_t *)payload->address, payload->len / 4);

	DBG_PRINT("CRC: %08lx\r\n", resp_pl->crc);
	payload = NULL;
	spi_free_packet(pkt);

	packetise_stream(resp, offsetof(struct readresp_pkt, data), READRESP_PKT_TYPE, (char *)resp_pl->address, resp_pl->len);
}

static void process_erase_pkt(struct spi_pl_packet *pkt)
{
	struct erase_pkt *payload = (struct erase_pkt *)pkt->data;
	uint32_t flags, flash_end;
	if (pkt->nparts) {
		report_error(pkt->id, "Unexpected nparts on erase pkt");
		spi_free_packet(pkt);
		return;
	}

	DBG_PRINT("Erase page at %08lx\r\n", payload->address);

	if (payload->address & (1024 - 1)) {
		report_error(pkt->id, "Erase address must be 1 kB aligned.");
		spi_free_packet(pkt);
		return;
	}

	flash_end = 0x08000000 + ((DESIG_FLASH_SIZE - 1) << 10);
	if (payload->address > flash_end) {
		report_error(pkt->id, "Erase address outside flash!");
		spi_free_packet(pkt);
		return;
	}

	flash_unlock();
	flash_erase_page(payload->address);
	flags = flash_get_status_flags();
	flash_lock();

	if (flags & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
		report_error(pkt->id, "Flash erase error.");
		spi_free_packet(pkt);
		return;
	}

	pkt->type = ACK_PKT_TYPE;
	spi_send_packet(pkt);
}

static inline uint32_t min(uint32_t a, uint32_t b) {
	return a < b ? a : b;
}

static void process_write_pkt(struct spi_pl_packet *pkt)
{
	static uint8_t *dst;
	static uint32_t data_words[MAX_TRANSFER / 4];
	static struct spi_pl_packet *start = NULL;
	static struct write_pkt *payload;
	static uint32_t len;
	uint8_t *src;
	uint32_t tocopy;

	if (!start) {
		payload = (struct write_pkt *)pkt->data;

		uint32_t flash_end;
		unsigned nparts = (payload->len + 12 - 1) / SPI_PACKET_DATA_LEN;
		if (nparts != pkt->nparts) {
			DBG_PRINT("Expected nparts %d, got %d\r\n", nparts, pkt->nparts);
			report_error(pkt->id, "Unexpected nparts on write pkt");
			goto cleanup;
		}

		if (payload->len > MAX_TRANSFER) {
			report_error(pkt->id, "Write request too long.");
			goto cleanup;
		}

		flash_end = 0x08000000 + ((DESIG_FLASH_SIZE) << 10);
		if (payload->address + payload->len > flash_end) {
			report_error(pkt->id, "Write address outside flash!");
			goto cleanup;
		}

		start = pkt;
		pkt = NULL; /* Make sure we don't double-free pkt in the error path */
		dst = (uint8_t *)data_words;
		len = payload->len;

		tocopy = min(len, SPI_PACKET_DATA_LEN - 12);
		src = start->data + 12;
	} else {
		if (pkt->nparts != start->nparts) {
			report_error(pkt->id, "Unexpected nparts.");
			goto cleanup;

		}
		src = pkt->data;
		tocopy = min(len, SPI_PACKET_DATA_LEN);
	}

	DBG_PRINT("Copy %ld bytes from %p to %p\r\n", tocopy, src, dst);
	memcpy(dst, src, tocopy);
	len -= tocopy;
	dst += tocopy;

	if (!start->nparts) {
		if (len != 0) {
			report_error(pkt->id, "Expected to be finished.");
			goto cleanup;
		}

		crc_reset();
		uint32_t flags, crc = crc_calculate_block(data_words, payload->len / 4);
		DBG_PRINT("Calculated CRC %08lx\r\n", crc);
		if (crc != payload->crc) {
			report_error(pkt->id, "Write integrity error.");
			goto cleanup;
		}

		flash_unlock();
		for (len = 0; len < payload->len / 4; len++, payload->address += 4) {
			flash_program_word(payload->address, data_words[len]);
		}
		flags = flash_get_status_flags();
		flash_lock();

		if (flags & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
			report_error(start->id, "Flash program error.");
			goto cleanup;
		}

		if (!pkt) {
			pkt = spi_alloc_packet();
			if (!pkt) {
				DBG_PRINT("Panic (Write ack)\r\n");
			}
		}
		memset(pkt, 0, sizeof(*pkt));
		pkt->type = ACK_PKT_TYPE;
		spi_send_packet(pkt);
		pkt = NULL;
		goto cleanup;
	}

	start->nparts--;
	spi_free_packet(pkt);
	return;

cleanup:
	if (start) {
		spi_free_packet(start);
		start = NULL;
	}
	if (pkt) {
		spi_free_packet(pkt);
	}
}

static void process_go_pkt(struct spi_pl_packet *pkt)
{
	struct go_pkt *payload = (struct go_pkt *)pkt->data;
	if (pkt->nparts) {
		DBG_PRINT("bad nparts\r\n");
		report_error(pkt->id, "Unexpected nparts on erase pkt.");
		spi_free_packet(pkt);
		return;
	}

	DBG_PRINT("Jump to %08lx.\r\n", payload->address);

	if (!checkUserCode(payload->address)) {
		report_error(pkt->id, "Jump target looks dubious.");
		spi_free_packet(pkt);
		scb_reset_system();
		return;
	}


	DBG_PRINT("Validated, jumping.\r\n");

	jumpToUser(payload->address);

	return;
}

static void process_query_pkt(struct spi_pl_packet *pkt)
{
	struct query_pkt *payload = (struct query_pkt *)pkt->data;
	struct queryresp_pkt *resp = (struct queryresp_pkt *)pkt->data;
	uint32_t parameter, value;
	if (pkt->nparts) {
		report_error(pkt->id, "Unexpected nparts on query pkt");
		spi_free_packet(pkt);
		return;
	}

	parameter = payload->parameter;
	DBG_PRINT("Query %ld.\r\n", parameter);

	switch (parameter) {
		case QUERY_PARAM_MAX_TRANSFER:
			value = MAX_TRANSFER;
			break;
		case QUERY_PARAM_DEFAULT_USER_ADDR:
			value = DEFAULT_USER_ADDR;
			break;
		default:
			report_error(pkt->id, "Unknown query.");
			spi_free_packet(pkt);
			return;
	}

	DBG_PRINT("Response %ld : %ld.\r\n", parameter, value);

	memset(pkt, 0, sizeof(*pkt));
	pkt->type = QUERYRESP_PKT_TYPE;
	resp->parameter = parameter;
	resp->value = value;
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
	rcc_periph_clock_enable(RCC_CRC);

	systick_init();
	setup_gpio();

#ifdef DEBUG
	usb_cdc_init();
#endif

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

	bool booting = true;
	int countdown = 20;
	while (1) {
		while ((pkt = spi_receive_packet())) {

			booting = false;

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
				case ERASE_PKT_TYPE:
					process_erase_pkt(pkt);
					break;
				case WRITE_PKT_TYPE:
					process_write_pkt(pkt);
					break;
				case GO_PKT_TYPE:
					process_go_pkt(pkt);
					break;
				case QUERY_PKT_TYPE:
					process_query_pkt(pkt);
					break;
				case 0xfe:
					ep0xfe_process_packet(pkt);
					break;
				default:
					DBG_PRINT("Unknown type %d\n", pkt->type);
					report_error(pkt->id, "Unknown type. But lets make this error.");
					spi_free_packet(pkt);
			}
		}

		if (msTicks > time + 100) {
			gpio_toggle(GPIOC, GPIO13);
			time = msTicks + 100;
			if (booting) {
				countdown--;
				if (!countdown && checkUserCode(DEFAULT_USER_ADDR)) {
					jumpToUser(DEFAULT_USER_ADDR);
				}
			}
		}
	}

	return 0;
}
