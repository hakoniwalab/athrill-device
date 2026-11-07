/*
 * The STM32F7/H7 DMA controllers (see stm32_dma.h). Offsets and bits from
 * RM0410 8.5 and RM0433 16.5.
 */
#include "stm32_dma.h"
#include "athrill_device_spi_dma.h"
#include "athrill_device_stm32_bridge.h"
#include "athrill_device_host.h"
#include "athrill_protocol_dma.h"

#include <string.h>

#define F7_DMA1_BASE	0x40026000U
#define F7_DMA2_BASE	0x40026400U
#define H7_DMA1_BASE	0x40020000U
#define H7_DMA2_BASE	0x40020400U
#define DMA_SIZE	0x400U

#define DMA_LISR	0x00U
#define DMA_HISR	0x04U
#define DMA_LIFCR	0x08U
#define DMA_HIFCR	0x0CU
#define DMA_STREAM0	0x10U
#define DMA_STREAM_SIZE	0x18U
#define STREAMS		8U

#define SxCR		0U
#define SxNDTR		1U
#define SxPAR		2U
#define SxM0AR		3U
#define SxM1AR		4U
#define SxFCR		5U

#define CR_EN		(1U << 0)
#define CR_HTIE		(1U << 3)
#define CR_TCIE		(1U << 4)
#define CR_DIR_SHIFT	6U
#define CR_CIRC		(1U << 8)
#define CR_PINC		(1U << 9)
#define CR_MINC		(1U << 10)
#define CR_PSIZE_SHIFT	11U
#define CR_MSIZE_SHIFT	13U
#define CR_CHSEL_SHIFT	25U
#define DIR_P2M		0U
#define DIR_M2P		1U

#define FLAG_HT		(1U << 4)
#define FLAG_TC		(1U << 5)


typedef struct DmaType DmaType;

typedef struct {
	DmaType *controller;
	uint32 stream;
	uint64 peripheral_value;
	std_bool peripheral_valid;
} DmaEndpoint;

struct DmaType {
	uint32 isr[2];		/* LISR, HISR */
	uint32 stream[STREAMS][6];
	uint32 ndtr_start[STREAMS];	/* NDTR when the stream was enabled (circular reload) */
	uint32 irq[STREAMS];
	uint32 transferred[STREAMS];
	std_bool block[STREAMS];
	AthrillChannel *channel[STREAMS];
	AthrillDmaProtocol *protocol[STREAMS];
	DmaEndpoint endpoint[STREAMS];
};

static DmaType dmas[2] = {
	{ .irq = { 11U, 12U, 13U, 14U, 15U, 16U, 17U, 47U } },
	{ .irq = { 56U, 57U, 58U, 59U, 60U, 68U, 69U, 70U } },
};

static uint32 dma1_base = F7_DMA1_BASE;
static uint32 dma2_base = F7_DMA2_BASE;

void stm32_dma_set_h7(std_bool h7)
{
	dma1_base = h7 ? H7_DMA1_BASE : F7_DMA1_BASE;
	dma2_base = h7 ? H7_DMA2_BASE : F7_DMA2_BASE;
}

std_bool stm32_dma_owns(uint32 addr)
{
	return (addr >= dma1_base) && (addr < (dma2_base + DMA_SIZE));
}

/* The bit position of stream s's flags in LISR/HISR. */
static uint32 flag_shift(uint32 s)
{
	static const uint32 shifts[4] = { 0U, 6U, 16U, 22U };

	return shifts[s % 4U];
}

uint32 stm32_dma_read(uint32 addr)
{
	DmaType *d = &dmas[(addr >= dma2_base) ? 1 : 0];
	uint32 offset = (addr - ((addr >= dma2_base) ? dma2_base : dma1_base)) & ~3U;
	uint32 shift = 8U * (addr & 3U);

	if (offset <= DMA_HISR) {
		return d->isr[offset / 4U] >> shift;
	}
	if ((offset >= DMA_STREAM0) && (offset < (DMA_STREAM0 + (STREAMS * DMA_STREAM_SIZE)))) {
		uint32 s = (offset - DMA_STREAM0) / DMA_STREAM_SIZE;
		uint32 r = ((offset - DMA_STREAM0) % DMA_STREAM_SIZE) / 4U;

		return d->stream[s][r] >> shift;
	}
	return 0U;
}

static uint32 unit(uint32 cr, uint32 shift)
{
	return 1U << ((cr >> shift) & 3U);	/* bytes: 1, 2, 4 */
}

static uint32 mem_read(uint32 addr, uint32 size)
{
	uint8 v8 = 0U;
	uint16 v16 = 0U;
	uint32 v32 = 0U;

	/* one access of the item width (host memory_read) */
	if (size == 1U) {
		(void)athrill_device_host()->memory_read(NULL, addr, &v8, 1U);
		return v8;
	}
	if (size == 2U) {
		(void)athrill_device_host()->memory_read(NULL, addr, &v16, 2U);
		return v16;
	}
	(void)athrill_device_host()->memory_read(NULL, addr, &v32, 4U);
	return v32;
}

static void mem_write(uint32 addr, uint32 size, uint32 value)
{
	uint8 v8 = (uint8)value;
	uint16 v16 = (uint16)value;

	/* one access of the item width (host memory_write) */
	if (size == 1U) {
		(void)athrill_device_host()->memory_write(NULL, addr, &v8, 1U);
	}
	else if (size == 2U) {
		(void)athrill_device_host()->memory_write(NULL, addr, &v16, 2U);
	}
	else {
		(void)athrill_device_host()->memory_write(NULL, addr, &value, 4U);
	}
}

static sint32 endpoint_memory_read(void *context, uint64 address, uint8 width,
	uint64 *value)
{
	(void)context;
	if (value == NULL || address > 0xFFFFFFFFU) {
		return -1;
	}
	*value = mem_read((uint32)address, width);
	return 0;
}

static sint32 endpoint_memory_write(void *context, uint64 address, uint8 width,
	uint64 value)
{
	(void)context;
	if (address > 0xFFFFFFFFU) {
		return -1;
	}
	mem_write((uint32)address, width, (uint32)value);
	return 0;
}

static sint32 endpoint_peripheral_read(void *context, uint32 request,
	uint8 width, uint64 *value)
{
	DmaEndpoint *endpoint = context;
	uint32 cr = endpoint->controller->stream[endpoint->stream][SxCR];

	(void)width;
	if (value == NULL || !endpoint->peripheral_valid ||
	    request != ((cr >> CR_CHSEL_SHIFT) & 7U)) {
		return -1;
	}
	*value = endpoint->peripheral_value;
	endpoint->peripheral_valid = FALSE;
	return 0;
}

static sint32 endpoint_peripheral_write(void *context, uint32 request,
	uint8 width, uint64 value)
{
	DmaEndpoint *endpoint = context;
	uint32 cr = endpoint->controller->stream[endpoint->stream][SxCR];

	(void)width;
	if (request != ((cr >> CR_CHSEL_SHIFT) & 7U)) {
		return -1;
	}
	endpoint->peripheral_value = value;
	endpoint->peripheral_valid = TRUE;
	return 0;
}

static const AthrillDmaEndpointOps endpoint_ops = {
	.memory_read = endpoint_memory_read,
	.memory_write = endpoint_memory_write,
	.peripheral_read = endpoint_peripheral_read,
	.peripheral_write = endpoint_peripheral_write,
};

static void stream_destroy(DmaType *d, uint32 s)
{
	if (d->protocol[s] != NULL) {
		athrill_dma_protocol_destroy(d->protocol[s]);
		d->protocol[s] = NULL;
	}
	if (d->channel[s] != NULL) {
		athrill_channel_destroy(d->channel[s]);
		d->channel[s] = NULL;
	}
	d->endpoint[s].peripheral_valid = FALSE;
}

static void stream_notify(void *context, AthrillChannelNotification notification,
	std_bool asserted, uint64 time)
{
	DmaEndpoint *endpoint = context;
	DmaType *d = endpoint->controller;
	uint32 s = endpoint->stream;
	uint32 *st = d->stream[s];
	uint32 flags;

	(void)time;
	if (!asserted || notification != ATHRILL_CHANNEL_NOTIFY_COMPLETE) {
		return;
	}
	flags = FLAG_TC | (d->block[s] ? FLAG_HT : 0U);
	d->isr[s / 4U] |= flags << flag_shift(s);
	if ((st[SxCR] & CR_CIRC) != 0U) {
		st[SxNDTR] = d->ndtr_start[s];
		d->transferred[s] = 0U;
	}
	else {
		st[SxNDTR] = 0U;
		st[SxCR] &= ~CR_EN;
	}
	if ((((flags & FLAG_HT) != 0U) && ((st[SxCR] & CR_HTIE) != 0U)) ||
	    ((st[SxCR] & CR_TCIE) != 0U)) {
		athrill_device_host_raise(d->irq[s]);
	}
}

static std_bool stream_create(DmaType *d, uint32 s)
{
	uint32 *st = d->stream[s];
	AthrillChannelConfig config;

	stream_destroy(d, s);
	memset(&config, 0, sizeof(config));
	config.peripheral_width = (uint8)unit(st[SxCR], CR_PSIZE_SHIFT);
	config.memory_width = (uint8)unit(st[SxCR], CR_MSIZE_SHIFT);
	if (config.peripheral_width > 4U || config.memory_width > 4U ||
	    st[SxNDTR] == 0U) {
		return FALSE;
	}
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_REQUEST;
	config.selection.value = (st[SxCR] >> CR_CHSEL_SHIFT) & 7U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = st[SxNDTR];
	config.circular = (st[SxCR] & CR_CIRC) != 0U;
	config.directions = (((st[SxCR] >> CR_DIR_SHIFT) & 3U) == DIR_M2P) ?
		ATHRILL_CHANNEL_ALLOW_TO_CHILD : ATHRILL_CHANNEL_ALLOW_TO_OWNER;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = stream_notify;
	config.notify_context = &d->endpoint[s];
	d->endpoint[s].controller = d;
	d->endpoint[s].stream = s;
	d->endpoint[s].peripheral_valid = FALSE;
	if (athrill_channel_create(&config, &d->channel[s]) != 0 ||
	    athrill_dma_protocol_create(d->channel[s], &endpoint_ops,
		&d->endpoint[s], &d->protocol[s]) != 0 ||
	    athrill_dma_start(d->protocol[s], st[SxM0AR],
		(st[SxCR] & CR_MINC) != 0U, athrill_device_host_clock()) != 0) {
		stream_destroy(d, s);
		return FALSE;
	}
	d->transferred[s] = 0U;
	d->block[s] = FALSE;
	return TRUE;
}

static void stream_half(DmaType *d, uint32 s)
{
	uint32 *st = d->stream[s];

	if (d->transferred[s] == (d->ndtr_start[s] / 2U) &&
	    d->transferred[s] != 0U) {
		d->isr[s / 4U] |= FLAG_HT << flag_shift(s);
		if ((st[SxCR] & CR_HTIE) != 0U) {
			athrill_device_host_raise(d->irq[s]);
		}
	}
}

static std_bool stream_request_rx(DmaType *d, uint32 s, uint64 value)
{
	uint32 request = (d->stream[s][SxCR] >> CR_CHSEL_SHIFT) & 7U;

	if (d->protocol[s] == NULL) {
		return FALSE;
	}
	d->endpoint[s].peripheral_value = value;
	d->endpoint[s].peripheral_valid = TRUE;
	d->transferred[s]++;
	if (athrill_dma_request(d->protocol[s], request, athrill_device_host_clock()) != 1) {
		d->transferred[s]--;
		d->endpoint[s].peripheral_valid = FALSE;
		return FALSE;
	}
	d->stream[s][SxNDTR] = (uint32)athrill_dma_remaining(d->protocol[s]);
	stream_half(d, s);
	return TRUE;
}

static std_bool stream_request_tx(DmaType *d, uint32 s, uint64 *value)
{
	uint32 request = (d->stream[s][SxCR] >> CR_CHSEL_SHIFT) & 7U;

	if (value == NULL || d->protocol[s] == NULL) {
		return FALSE;
	}
	d->endpoint[s].peripheral_valid = FALSE;
	d->transferred[s]++;
	if (athrill_dma_request(d->protocol[s], request, athrill_device_host_clock()) != 1 ||
	    !d->endpoint[s].peripheral_valid) {
		d->transferred[s]--;
		return FALSE;
	}
	*value = d->endpoint[s].peripheral_value;
	d->endpoint[s].peripheral_valid = FALSE;
	d->stream[s][SxNDTR] = (uint32)athrill_dma_remaining(d->protocol[s]);
	stream_half(d, s);
	return TRUE;
}

/* The enabled stream that reads from peripheral address par, if any. */
static std_bool find_rx(uint32 par, DmaType **rd, uint32 *rs)
{
	uint32 c;
	uint32 s;

	for (c = 0U; c < 2U; c++) {
		for (s = 0U; s < STREAMS; s++) {
			uint32 *st = dmas[c].stream[s];

			if (((st[SxCR] & CR_EN) != 0U) && (((st[SxCR] >> CR_DIR_SHIFT) & 3U) == DIR_P2M)
				&& (st[SxPAR] == par)) {
				*rd = &dmas[c];
				*rs = s;
				return TRUE;
			}
		}
	}
	return FALSE;
}

/* The enabled stream that writes to peripheral address par, if any. */
static std_bool find_tx(uint32 par, DmaType **td, uint32 *ts)
{
	uint32 c;
	uint32 s;

	for (c = 0U; c < 2U; c++) {
		for (s = 0U; s < STREAMS; s++) {
			uint32 *st = dmas[c].stream[s];

			if (((st[SxCR] & CR_EN) != 0U) && (((st[SxCR] >> CR_DIR_SHIFT) & 3U) == DIR_M2P)
				&& (st[SxPAR] == par)) {
				*td = &dmas[c];
				*ts = s;
				return TRUE;
			}
		}
	}
	return FALSE;
}

/* TX stream s enabled: an SPI exchange of NDTR frames, with the RX stream of the same DR. */
static void start(DmaType *d, uint32 s)
{
	uint32 *tx = d->stream[s];
	uint32 par = tx[SxPAR];
	DmaType *rd = NULL;
	uint32 rs = 0U;
	uint32 *rx = NULL;
	uint32 n = tx[SxNDTR];
	uint32 i;
	uint32 rx_par = par;
	uint32 response = 0U;

	if (((tx[SxCR] >> CR_DIR_SHIFT) & 3U) != DIR_M2P) {
		return;
	}
	/* TO BE REMOVED (step 4-5): the SPI controllers are athrill-device
	 * models; the stream reaches them through athrill_device_spi_dma.h. */
	if (!athrill_device_stm32_spi_dma(par, 0U, 0U, 0U, &rx_par, &response)) {
		return;
	}
	if (find_rx(rx_par, &rd, &rs)) {
		rx = rd->stream[rs];
	}
	d->block[s] = TRUE;
	if (rx != NULL) {
		rd->block[rs] = TRUE;
	}
	for (i = 0U; i < n; i++) {
		uint64 value;

		if (!stream_request_tx(d, s, &value)) {
			break;
		}

		(void)athrill_device_stm32_spi_dma(par, unit(tx[SxCR], CR_PSIZE_SHIFT),
			(uint32)value, (rx != NULL) ? unit(rx[SxCR], CR_PSIZE_SHIFT) : 1U,
			&rx_par, &response);
		if (rx != NULL && !stream_request_rx(rd, rs, response)) {
			break;
		}
	}
}

void stm32_dma_write(uint32 addr, uint32 size, uint32 value)
{
	DmaType *d = &dmas[(addr >= dma2_base) ? 1 : 0];
	uint32 offset = (addr - ((addr >= dma2_base) ? dma2_base : dma1_base)) & ~3U;
	uint32 shift = 8U * (addr & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	value = (value << shift) & mask;
	if ((offset == DMA_LIFCR) || (offset == DMA_HIFCR)) {
		d->isr[(offset - DMA_LIFCR) / 4U] &= ~value;
		return;
	}
	if ((offset >= DMA_STREAM0) && (offset < (DMA_STREAM0 + (STREAMS * DMA_STREAM_SIZE)))) {
		uint32 s = (offset - DMA_STREAM0) / DMA_STREAM_SIZE;
		uint32 r = ((offset - DMA_STREAM0) % DMA_STREAM_SIZE) / 4U;
		uint32 was = d->stream[s][SxCR];

		d->stream[s][r] = (d->stream[s][r] & ~mask) | value;
		if (r == SxNDTR) {
			d->stream[s][r] &= 0xFFFFU;
		}
		if ((r == SxCR) && ((was & CR_EN) != 0U) &&
		    ((d->stream[s][SxCR] & CR_EN) == 0U)) {
			if (d->protocol[s] != NULL) {
				(void)athrill_dma_stop(d->protocol[s], athrill_device_host_clock());
			}
			stream_destroy(d, s);
		}
		if ((r == SxCR) && ((was & CR_EN) == 0U) && ((d->stream[s][SxCR] & CR_EN) != 0U)) {
			d->ndtr_start[s] = d->stream[s][SxNDTR];
			if (!stream_create(d, s)) {
				d->stream[s][SxCR] &= ~CR_EN;
			}
			else {
				start(d, s);
			}
		}
	}
}

std_bool stm32_dma_peripheral_rx(uint32 par, uint32 data)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	if (!find_rx(par, &d, &s)) {
		return FALSE;
	}
	return stream_request_rx(d, s, data);
}

std_bool stm32_dma_peripheral_tx(uint32 par, uint32 *data)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	uint64 value;

	if (!find_tx(par, &d, &s)) {
		return FALSE;
	}
	if (!stream_request_tx(d, s, &value)) {
		return FALSE;
	}
	*data = (uint32)value;
	return TRUE;
}

std_bool stm32_dma_peripheral_ready(uint32 par, std_bool to_memory)
{
	DmaType *d = NULL;
	uint32 s = 0U;

	return to_memory ? find_rx(par, &d, &s) : find_tx(par, &d, &s);
}

std_bool stm32_dma_peripheral_block_rx(uint32 par, const uint8 *data, uint32 len)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	uint32 i;

	if (!find_rx(par, &d, &s)) {
		return FALSE;
	}
	d->block[s] = TRUE;
	for (i = 0U; i < len;) {
		uint32 width = unit(d->stream[s][SxCR], CR_PSIZE_SHIFT);
		uint64 value = 0U;
		uint32 amount = (len - i < width) ? len - i : width;

		memcpy(&value, &data[i], amount);
		if (!stream_request_rx(d, s, value)) {
			return FALSE;
		}
		i += amount;
	}
	return TRUE;
}

std_bool stm32_dma_peripheral_block_tx(uint32 par, uint8 *data, uint32 len)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	uint32 i;

	if (!find_tx(par, &d, &s)) {
		return FALSE;
	}
	d->block[s] = TRUE;
	for (i = 0U; i < len;) {
		uint32 width = unit(d->stream[s][SxCR], CR_PSIZE_SHIFT);
		uint64 value;
		uint32 amount = (len - i < width) ? len - i : width;

		if (!stream_request_tx(d, s, &value)) {
			return FALSE;
		}
		memcpy(&data[i], &value, amount);
		i += amount;
	}
	return TRUE;
}

void stm32_dma_reset(void)
{
	uint32 c;
	uint32 s;
	uint32 r;

	/* TO BE REMOVED (step 4-5): the UART models reach these streams through
	 * athrill_device_stm32_bridge.h. */
	athrill_device_stm32_dma_bridge(stm32_dma_peripheral_tx, stm32_dma_peripheral_rx);

	for (c = 0U; c < 2U; c++) {
		dmas[c].isr[0] = 0U;
		dmas[c].isr[1] = 0U;
		for (s = 0U; s < STREAMS; s++) {
			stream_destroy(&dmas[c], s);
			for (r = 0U; r < 6U; r++) {
				dmas[c].stream[s][r] = 0U;
			}
			dmas[c].stream[s][SxFCR] = 0x21U;
			dmas[c].ndtr_start[s] = 0U;
			dmas[c].transferred[s] = 0U;
			dmas[c].block[s] = FALSE;
		}
	}
}
