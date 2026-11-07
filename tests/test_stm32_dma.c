/*
 * The STM32 DMA model (src/models/stm32_dma.c) driven by request lines
 * (ABI 1.14), with RAM and fake peripheral registers on a fake bus.
 *
 * Expected values come from the reference manual, not from the code:
 * RM0410 Rev 5 8.3.4 (the stream moves an item per request of its selected
 * channel), 8.3.9 (circular: NDTR reloads, the stream stays enabled),
 * 8.3.14 (end of transfer: the stream disables itself), 8.5.1/8.5.3
 * (LISR/HISR flag positions: stream 0 bit 0, 1 bit 6, 2 bit 16, 3 bit 22;
 * HTIF +4, TCIF +5), 8.5.5 (SxCR: EN 0, HTIE 3, TCIE 4, DIR 7:6, CIRC 8,
 * MINC 10, PSIZE 12:11, MSIZE 14:13, CHSEL 27:25); and the Devicetree's
 * interrupts in stream order (DMA1: 11-17, 47).
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define DMA1		0x40026000U
#define RAM		0x20000000U
#define TDR		0x40004828U	/* a transmit register (USART3 TDR) */
#define RDR		0x40004824U	/* a receive register (USART3 RDR) */
#define SPI_DR		0x4001300CU	/* an SPI DR: a write receives a frame */

#define S(n, r)		(0x10U + 0x18U * (n) + 4U * (r))	/* SxCR..SxFCR */
#define CR_EN		(1U << 0)
#define CR_HTIE		(1U << 3)
#define CR_TCIE		(1U << 4)
#define CR_M2P		(1U << 6)
#define CR_CIRC		(1U << 8)
#define CR_MINC		(1U << 10)
#define CHSEL(c)	((uint32)(c) << 25)

static uint8 ram[0x100];
static char tdr_out[32];
static uint32 tdr_len;
static const char *rdr_in;
static uint32 rdr_pos;
static uint32 raised[128];
static uint64 now;

/* SPI-like: a DR write receives a frame (MOSI ^ 0xFF), raising "rx";
 * a DR read takes it, lowering "rx". */
static const AthrillDeviceDmaProviderOps *provider;
static void *dev;
static void line(uint32 stream, uint32 channel, std_bool asserted);
static uint8 spi_rx;
static uint32 spi_pending, spi_pending_max;

static void line(uint32 stream, uint32 channel, std_bool asserted)
{
	AthrillDeviceDmaRequestLine l;
	memset(&l, 0, sizeof(l));
	l.cell_count = 4U;
	l.cells[0] = stream;
	l.cells[1] = channel;
	TEST_CHECK(provider->request(dev, &l, asserted) == 0);
}

static sint32 memory_read(void *t, uint64 address, void *data, uint32 size)
{
	(void)t;
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy(data, &ram[address - RAM], size);
		return 0;
	}
	if (address == RDR) {
		uint32 v = (uint8)rdr_in[rdr_pos++];
		memcpy(data, &v, size);
		line(1U, 4U, FALSE);		/* the RDR read lowers RXNE */
		return 0;
	}
	if (address == SPI_DR) {
		uint32 v = spi_rx;
		memcpy(data, &v, size);
		spi_pending--;
		line(0U, 3U, FALSE);		/* RXNE low */
		return 0;
	}
	return -1;
}

static sint32 memory_write(void *t, uint64 address, const void *data, uint32 size)
{
	uint32 v = 0U;
	(void)t;
	memcpy(&v, data, size);
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy(&ram[address - RAM], data, size);
		return 0;
	}
	if (address == TDR) {
		tdr_out[tdr_len++] = (char)v;
		return 0;
	}
	if (address == SPI_DR) {
		spi_rx = (uint8)(v ^ 0xFFU);
		spi_pending++;
		if (spi_pending > spi_pending_max) {
			spi_pending_max = spi_pending;
		}
		line(0U, 3U, TRUE);		/* RXNE: "rx" */
		return 0;
	}
	return -1;
}

static uint64 time_now(void *t) { (void)t; return now; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine l, std_bool on)
{
	(void)t;
	if (on && l.value < 128U) {
		raised[l.value]++;
	}
	return 0;
}

static const uint32 stream_irqs[8] = { 11U, 12U, 13U, 14U, 15U, 16U, 17U, 47U };
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *l)
{
	(void)c; (void)name;
	if (index >= 8U) {
		return -1;
	}
	l->value = stream_irqs[index];
	return 0;
}
static sint32 dma_provide(void *c, const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	(void)c;
	provider = ops;
	TEST_CHECK(instance == dev);
	return 0;
}
static uint32 reg_cells[2] = { DMA1, 0x400U };
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = reg_cells[index];
	return 0;
}

static const AthrillDeviceModelOps *m = &athrill_device_stm32_dma_model;
static uint32 rd(uint32 offset)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, DMA1 + offset, 4U, &value) == 0);
	return (uint32)value;
}
static void wr(uint32 offset, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, DMA1 + offset, 4U, value) == 0);
}

/* stream n: PAR, M0AR, NDTR, then CR (EN last, as the drivers do) */
static void program(uint32 n, uint32 par, uint32 m0ar, uint32 ndtr, uint32 cr)
{
	wr(S(n, 2), par);
	wr(S(n, 3), m0ar);
	wr(S(n, 1), ndtr);
	wr(S(n, 0), cr & ~CR_EN);
	wr(S(n, 0), cr);
}

static void reset_all(void)
{
	TEST_CHECK(m->reset(dev) == 0);
	memset(ram, 0, sizeof(ram));
	memset(raised, 0, sizeof(raised));
	memset(tdr_out, 0, sizeof(tdr_out));
	tdr_len = 0U;
	rdr_pos = 0U;
	spi_pending = 0U;
	spi_pending_max = 0U;
}

int main(void)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNodeOps node_ops;
	static AthrillDeviceResourceOps resources;
	AthrillDeviceNode node = { 0U, "/soc/dma@40026000", "st,stm32-dma-v1", &node_ops, NULL };
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	uint32 i;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.memory_read = memory_read;
	target_ops.memory_write = memory_write;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.dma_provide = dma_provide;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;
	TEST_CHECK(m->validate(&node, NULL, 0U) == 0);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0 && provider != NULL);
	reset_all();

	/* memory to TDR: stream 3, channel 4 (USART3 TX), 4 bytes */
	memcpy(ram, "ABCD", 4U);
	program(3U, TDR, RAM, 4U, CHSEL(4) | CR_MINC | CR_M2P | CR_HTIE | CR_TCIE | CR_EN);
	TEST_CHECK(tdr_len == 0U && rd(S(3, 1)) == 4U);	/* no request: nothing */
	line(3U, 5U, TRUE);					/* another channel */
	TEST_CHECK(tdr_len == 0U);
	line(3U, 5U, FALSE);
	/* the request raised (and kept): every item moves, in order */
	line(3U, 4U, TRUE);
	TEST_CHECK(tdr_len == 4U && memcmp(tdr_out, "ABCD", 4U) == 0);
	TEST_CHECK(rd(S(3, 1)) == 0U);				/* NDTR */
	TEST_CHECK((rd(S(3, 0)) & CR_EN) == 0U);		/* disabled itself */
	TEST_CHECK(rd(0x00U) == (0x30U << 22));			/* LISR: HTIF3, TCIF3 */
	TEST_CHECK(raised[14] == 2U);				/* half, then complete */
	/* LIFCR clears the written flags */
	wr(0x08U, 0x20U << 22);
	TEST_CHECK(rd(0x00U) == (0x10U << 22));
	/* disabled: a request moves nothing */
	line(3U, 4U, FALSE);
	line(3U, 4U, TRUE);
	TEST_CHECK(tdr_len == 4U);
	line(3U, 4U, FALSE);

	/* a request raised before the stream is enabled: served at enable */
	reset_all();
	memcpy(ram, "XY", 2U);
	line(3U, 4U, TRUE);
	program(3U, TDR, RAM, 2U, CHSEL(4) | CR_MINC | CR_M2P | CR_EN);
	TEST_CHECK(tdr_len == 2U && memcmp(tdr_out, "XY", 2U) == 0);
	TEST_CHECK(raised[14] == 0U);				/* no HTIE/TCIE */
	line(3U, 4U, FALSE);

	/* RDR to memory, circular: stream 1, channel 4 (USART3 RX), NDTR 3;
	 * the peripheral raises its request per character and the RDR read
	 * lowers it (a request it kept raised would move items on and on) */
	reset_all();
	rdr_in = "xyz12";
	program(1U, RDR, RAM, 3U, CHSEL(4) | CR_MINC | CR_CIRC | CR_HTIE | CR_TCIE | CR_EN);
	for (i = 0U; i < 3U; i++) {
		line(1U, 4U, TRUE);
		line(1U, 4U, FALSE);
	}
	TEST_CHECK(memcmp(ram, "xyz", 3U) == 0);
	TEST_CHECK(rd(S(1, 1)) == 3U);				/* NDTR reloaded */
	TEST_CHECK((rd(S(1, 0)) & CR_EN) != 0U);		/* still enabled */
	TEST_CHECK(rd(0x00U) == (0x30U << 6));			/* HTIF1 (after 1 of 3), TCIF1 */
	TEST_CHECK(raised[12] == 2U);
	for (i = 0U; i < 2U; i++) {
		line(1U, 4U, TRUE);
		line(1U, 4U, FALSE);
	}
	TEST_CHECK(memcmp(ram, "12z", 3U) == 0);		/* from the start again */
	TEST_CHECK(rd(S(1, 1)) == 1U);
	/* EN cleared mid-transfer: the stream stops */
	wr(S(1, 0), CHSEL(4) | CR_MINC | CR_CIRC);
	rdr_in = "q";
	rdr_pos = 0U;
	line(1U, 4U, TRUE);
	TEST_CHECK(rdr_pos == 0U);
	line(1U, 4U, FALSE);

	/* SPI-like exchange on one controller: TX stream 3 and RX stream 0,
	 * channel 3 (SPI1 on DMA2); each DR write raises "rx" during the TX
	 * stream's move. The RX stream is served by the same loop, in stream
	 * order, so at most one frame waits (no recursion, no overrun). */
	reset_all();
	memcpy(ram, "\x01\x02\x03\x04\x05\x06", 6U);
	program(0U, SPI_DR, RAM + 0x80U, 6U, CHSEL(3) | CR_MINC | CR_TCIE | CR_EN);
	program(3U, SPI_DR, RAM, 6U, CHSEL(3) | CR_MINC | CR_M2P | CR_TCIE | CR_EN);
	line(3U, 3U, TRUE);					/* TXE */
	for (i = 0U; i < 6U; i++) {
		TEST_CHECK(ram[0x80U + i] == (uint8)((i + 1U) ^ 0xFFU));
	}
	TEST_CHECK(spi_pending == 0U && spi_pending_max == 1U);
	TEST_CHECK((rd(S(0, 0)) & CR_EN) == 0U && (rd(S(3, 0)) & CR_EN) == 0U);
	TEST_CHECK(raised[11] == 1U && raised[14] == 1U);
	TEST_CHECK(rd(0x00U) == ((0x30U << 0) | (0x30U << 22)));

	/* a line outside the streams or channels */
	{
		AthrillDeviceDmaRequestLine l;
		memset(&l, 0, sizeof(l));
		l.cell_count = 2U;
		l.cells[0] = 8U;
		TEST_CHECK(provider->request(dev, &l, TRUE) != 0);
		l.cells[0] = 0U;
		l.cells[1] = 16U;
		TEST_CHECK(provider->request(dev, &l, TRUE) != 0);
		l.cell_count = 1U;
		l.cells[1] = 0U;
		TEST_CHECK(provider->request(dev, &l, TRUE) != 0);
	}

	m->destroy(dev);
	printf("stm32-dma: ok\n");
	return 0;
}
