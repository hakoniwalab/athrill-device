/*
 * The STM32 SPI controller models (src/models/stm32_spi.c, stm32h7_spi.c)
 * with a chip selected through watch_gpio, ported from athrill-target-stm32's
 * test_spi_ms5611.c (test_spi), test_stm32h7_spi.c and, run with the argument
 * "spec-gaps" (WILL_FAIL), test_stm32h7_spi_frontend.c. Plus the DMA
 * request lines (ABI 1.14 dma_request): F7 "tx" = TXDMAEN and TXE, "rx" =
 * RXDMAEN and RXNE (RM0410 35.5.8); H7 "tx" = TXDMAEN and TXP, "rx" =
 * RXDMAEN and RXP (RM0433 52.4.14).
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

/* ---- a fake host and resources ---- */
static uint32 raised[160];
static uint64 clock_now;
static uint32 batches;
static uint64 time_now(void *t) { (void)t; return clock_now++; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool on)
{
	(void)t;
	if (on && line.value < 160U) {
		raised[line.value]++;
	}
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine line) { (void)t; (void)line; return FALSE; }
static void end_batch(void *t) { (void)t; batches++; }

/* The DMA request levels the model last told (and how often). */
static std_bool tx_level, rx_level;
static uint32 request_calls;
static sint32 dma_request(void *c, const char *name, std_bool asserted)
{
	(void)c;
	request_calls++;
	if (strcmp(name, "tx") == 0) {
		tx_level = asserted;
	} else if (strcmp(name, "rx") == 0) {
		rx_level = asserted;
	} else {
		return -1;
	}
	return 0;
}

/* The chip: answers MOSI ^ 0xFF; the FRAM-like command 0x9F returns an ID. */
typedef struct {
	std_bool selected;
	uint32 selects;
	uint8 command;
	uint32 position;
} FakeChip;
static const uint8 fram_id[9] = { 0x7FU, 0x7FU, 0x7FU, 0x7FU, 0x7FU, 0x7FU, 0xC2U, 0x22U, 0x08U };
static void chip_select(void *instance, std_bool asserted)
{
	FakeChip *chip = instance;
	chip->selected = asserted;
	chip->selects++;
	chip->position = 0U;
}
static uint8 chip_exchange(void *instance, uint8 mosi)
{
	FakeChip *chip = instance;
	if (chip->position == 0U) {
		chip->command = mosi;
	}
	if (chip->command == 0x9FU && chip->position >= 1U && chip->position <= 9U) {
		return fram_id[chip->position++ - 1U];
	}
	chip->position++;
	return (uint8)(mosi ^ 0xFFU);
}
static const AthrillDeviceSpiPeripheralOps chip_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceSpiPeripheralOps),
	chip_select, chip_exchange
};
static FakeChip chip;

static AthrillDeviceGpioEdge cs_callback;
static void *cs_instance;
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)c; (void)name; (void)index;
	line->value = 36U;	/* SPI2 global */
	return 0;
}
static sint32 get_gpio(void *c, const char *property, uint32 index, AthrillDeviceGpioLine *line)
{
	(void)c;
	if (strcmp(property, "cs-gpios") != 0 || index != 0U) {
		return -1;
	}
	line->value = 0x44U;
	return 0;
}
static sint32 get_spi_peripheral(void *c, uint32 chip_select_index,
	const AthrillDeviceSpiPeripheralOps **ops, void **instance)
{
	(void)c;
	if (chip_select_index != 0U) {
		return -1;
	}
	*ops = &chip_ops;
	*instance = &chip;
	return 0;
}
static sint32 watch_gpio(void *c, AthrillDeviceGpioLine line, AthrillDeviceGpioEdge cb, void *instance)
{
	(void)c;
	TEST_CHECK(line.value == 0x44U);
	cs_callback = cb;
	cs_instance = instance;
	return 0;
}

static uint32 reg_cells[2];
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = reg_cells[index];
	return 0;
}

static const AthrillDeviceModelOps *m;
static void *dev;
static uint32 base;

static void make(const AthrillDeviceModelOps *model, uint32 at)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNodeOps node_ops;
	static AthrillDeviceResourceOps resources;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.end_batch = end_batch;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.get_gpio = get_gpio;
	resources.get_spi_peripheral = get_spi_peripheral;
	resources.watch_gpio = watch_gpio;
	resources.dma_request = dma_request;
	tx_level = FALSE;
	rx_level = FALSE;
	request_calls = 0U;
	node.ops = &node_ops;
	node.path = "/soc/spi";
	reg_cells[0] = at;
	reg_cells[1] = 0x400U;
	base = at;
	m = model;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;
	memset(&chip, 0, sizeof(chip));
	memset(raised, 0, sizeof(raised));
	cs_callback = NULL;
	TEST_CHECK(m->validate(&node, NULL, 0U) == 0);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0 && cs_callback != NULL);
	TEST_CHECK(m->reset(dev) == 0);
}

static uint32 rd(uint32 offset, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, base + offset, size, &value) == 0);
	return (uint32)value;
}
static void wr(uint32 offset, uint32 size, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, base + offset, size, value) == 0);
}
static void cs(std_bool level) { cs_callback(cs_instance, level); }

/* ---- F7 (test_spi_ms5611.c test_spi) ---- */
#define F7_CR1 0x00U
#define F7_CR2 0x04U
#define F7_SR  0x08U
#define F7_DR  0x0CU

static uint8 f7_send(uint8 b)
{
	wr(F7_DR, 1U, b);
	return (uint8)rd(F7_DR, 1U);
}

static void test_f7(void)
{
	make(&athrill_device_stm32_spi_model, 0x40013400U);
	wr(F7_CR2, 4U, (7U << 8) | (1U << 12));	/* 8-bit, FRXTH */
	wr(F7_CR1, 4U, (1U << 6) | (1U << 2));	/* SPE, MSTR */
	TEST_CHECK((rd(F7_SR, 4U) & 3U) == 2U);	/* TXE */
	TEST_CHECK(f7_send(0x12U) == 0xFFU);	/* no chip selected: MISO high */
	/* the chip selected by its cs-gpios line (low), deselected (high) */
	cs(TRUE);
	cs(FALSE);
	TEST_CHECK(chip.selected && chip.selects == 1U);
	TEST_CHECK(f7_send(0x12U) == 0xEDU);
	cs(TRUE);
	TEST_CHECK(!chip.selected && f7_send(0x12U) == 0xFFU);
	/* FRXTH = 0: RXNE needs two bytes */
	wr(F7_CR2, 4U, 7U << 8);
	wr(F7_DR, 1U, 0x00U);
	TEST_CHECK((rd(F7_SR, 4U) & 1U) == 0U);
	TEST_CHECK(((rd(F7_SR, 4U) >> 9) & 3U) == 1U);	/* FRLVL quarter */
	wr(F7_DR, 1U, 0x00U);
	TEST_CHECK((rd(F7_SR, 4U) & 1U) == 1U);
	TEST_CHECK(rd(F7_DR, 2U) == 0xFFFFU);	/* 16-bit read takes both */
	TEST_CHECK(((rd(F7_SR, 4U) >> 9) & 3U) == 0U);
	/* SPE cleared: nothing sent */
	wr(F7_CR1, 4U, 1U << 2);
	wr(F7_DR, 1U, 0x00U);
	TEST_CHECK((rd(F7_SR, 4U) & 1U) == 0U);
	/* every register read ends the CPU batch */
	batches = 0U;
	(void)rd(F7_SR, 4U);
	TEST_CHECK(batches == 1U);
	/* DMA requests: none without TXDMAEN/RXDMAEN */
	wr(F7_CR1, 4U, (1U << 6) | (1U << 2));
	wr(F7_CR2, 4U, (7U << 8) | (1U << 12));
	cs(FALSE);
	TEST_CHECK(!tx_level && !rx_level);
	/* TXDMAEN (bit 1) with TXE: "tx"; RXDMAEN (bit 0) and RX empty: no "rx" */
	wr(F7_CR2, 4U, (7U << 8) | (1U << 12) | 3U);
	TEST_CHECK(tx_level && !rx_level);
	/* a DR write (as the DMA does): the frame received, RXNE: "rx" */
	wr(F7_DR, 1U, 0x0FU);
	TEST_CHECK(tx_level && rx_level);
	/* a DR read (as the DMA does): RX empty again, "rx" low */
	TEST_CHECK(rd(F7_DR, 1U) == 0xF0U);
	TEST_CHECK(tx_level && !rx_level);
	/* changes only: another SR read tells nothing */
	request_calls = 0U;
	(void)rd(F7_SR, 4U);
	TEST_CHECK(request_calls == 0U);
	/* TXDMAEN cleared: "tx" low */
	wr(F7_CR2, 4U, (7U << 8) | (1U << 12) | 1U);
	TEST_CHECK(!tx_level);
	m->destroy(dev);
}

/* ---- H7 (test_stm32h7_spi.c) ---- */
#define H7_CR1  0x00U
#define H7_IER  0x10U
#define H7_SR   0x14U
#define H7_TXDR 0x20U
#define H7_RXDR 0x30U

static void test_h7(void)
{
	uint8 id[9];
	uint32 i;
	make(&athrill_device_stm32h7_spi_model, 0x40003800U);
	cs(TRUE);
	cs(FALSE);					/* chip select low */
	wr(H7_CR1, 4U, 1U);				/* SPE */
	wr(H7_TXDR, 1U, 0x9FU);
	(void)rd(H7_RXDR, 1U);
	for (i = 0U; i < sizeof(id); i++) {
		wr(H7_TXDR, 1U, 0U);
		id[i] = (uint8)rd(H7_RXDR, 1U);
	}
	cs(TRUE);
	TEST_CHECK(id[0] == 0x7FU && id[6] == 0xC2U && id[7] == 0x22U && id[8] == 0x08U);
	TEST_CHECK((rd(H7_SR, 4U) & ((1U << 1) | (1U << 12))) == ((1U << 1) | (1U << 12)));
	wr(H7_IER, 4U, 1U << 3);			/* EOTIE after DMA completion */
	TEST_CHECK(raised[36] == 1U);			/* the global IRQ */
	/* DMA requests (CFG1 TXDMAEN bit 15, RXDMAEN bit 14): the received
	 * byte passes through the RX FIFO */
	cs(FALSE);
	TEST_CHECK(!tx_level && !rx_level);
	wr(0x08U, 4U, 0x00070007U | (1U << 15) | (1U << 14));
	TEST_CHECK(tx_level && !rx_level);		/* TXP */
	wr(H7_TXDR, 1U, 0x5AU);
	TEST_CHECK(rx_level && (rd(H7_SR, 4U) & 1U) == 1U);	/* RXP */
	TEST_CHECK(rd(H7_RXDR, 1U) == 0xA5U && !rx_level);
	wr(0x08U, 4U, 0x00070007U);
	TEST_CHECK(!tx_level);
	m->destroy(dev);
}

/* ---- H7 spec gaps (test_stm32h7_spi_frontend.c), WILL_FAIL ---- */
static int checks;
static int failures;
#define CHECK_EQ(what, got, want) check_eq(__LINE__, what, (unsigned long long)(got), (unsigned long long)(want))
static void check_eq(int line, const char *what, unsigned long long got, unsigned long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = 0x%llx, want 0x%llx\n", line, what, got, want);
	}
}
#define CR2 0x04U
#define CFG1 0x08U
#define CFG2 0x0CU
#define IFCR 0x18U
#define SPE (1U << 0)
#define CSTART (1U << 9)
#define RXDMAEN (1U << 14)
#define TXDMAEN (1U << 15)
#define MASTER (1U << 22)
#define CPHA (1U << 24)
#define CPOL (1U << 25)
#define SSM (1U << 26)
#define RXP (1U << 0)
#define TXP (1U << 1)
#define EOT (1U << 3)
#define SUSP (1U << 11)
#define TXC (1U << 12)
#define EOTC (1U << 3)
#define CFG1_RESET 0x00070007U

static uint32 r4(uint32 offset) { return rd(offset, 4U); }
/* reset, with the peer selected again (reset deselects the chips) */
static void gaps_reset(void)
{
	TEST_CHECK(m->reset(dev) == 0);
	tx_level = FALSE;	/* a reset resets the DMA too: every line low */
	rx_level = FALSE;
	cs(TRUE);
	cs(FALSE);
}
static void w4(uint32 offset, uint32 value) { wr(offset, 4U, value); }

static void gaps_register_file(void)
{
	const uint32 cfg1 = (3U << 28) | (3U << 5) | 7U;
	const uint32 cfg2 = MASTER | SSM | CPOL | CPHA;
	gaps_reset();
	CHECK_EQ("CR1 reset", r4(H7_CR1), 0U);
	CHECK_EQ("CFG1 reset", r4(CFG1), CFG1_RESET);
	CHECK_EQ("CFG2 reset", r4(CFG2), 0U);
	CHECK_EQ("SR reset", r4(H7_SR), TXC | TXP);
	w4(CR2, 2U);
	w4(CFG1, cfg1);
	w4(CFG2, cfg2);
	CHECK_EQ("TSIZE readback while disabled", r4(CR2) & 0xFFFFU, 2U);
	CHECK_EQ("CFG1 readback while disabled", r4(CFG1), cfg1);
	CHECK_EQ("CFG2 readback while disabled", r4(CFG2), cfg2);
	w4(H7_CR1, SPE);
	w4(CFG2, 0U);
	CHECK_EQ("CFG2 write protected while enabled", r4(CFG2), cfg2);
	w4(CFG1, TXDMAEN | RXDMAEN | 7U);
	CHECK_EQ("only CFG1 DMA enables change while enabled", r4(CFG1), cfg1 | TXDMAEN | RXDMAEN);
}

static void gaps_tsize_and_flags(void)
{
	uint32 status;
	gaps_reset();
	w4(CR2, 2U);
	w4(H7_CR1, SPE | CSTART);
	status = r4(H7_SR);
	CHECK_EQ("start: CTSIZE", status >> 16, 2U);
	CHECK_EQ("start: TXP only", status & (TXP | RXP | EOT | SUSP | TXC), TXP);
	wr(H7_TXDR, 1U, 0x11U);
	status = r4(H7_SR);
	CHECK_EQ("one frame: CTSIZE", status >> 16, 1U);
	CHECK_EQ("one frame: RXP without EOT", status & (RXP | EOT), RXP);
	CHECK_EQ("first RX byte", rd(H7_RXDR, 1U), 0xEEU);
	CHECK_EQ("read drains RXP", r4(H7_SR) & RXP, 0U);
	wr(H7_TXDR, 1U, 0x22U);
	status = r4(H7_SR);
	CHECK_EQ("complete: CTSIZE", status >> 16, 0U);
	CHECK_EQ("complete: RXP EOT TXC", status & (RXP | EOT | TXC), RXP | EOT | TXC);
	CHECK_EQ("EOT clears CSTART", r4(H7_CR1) & CSTART, 0U);
	w4(IFCR, EOTC);
	CHECK_EQ("EOTC clears EOT and TSIZE-mode TXC", r4(H7_SR) & (EOT | TXC), 0U);
}

static void gaps_fifo_threshold(void)
{
	uint32 packed;
	gaps_reset();
	w4(CFG1, CFG1_RESET | (1U << 5));
	w4(CR2, 2U);
	w4(H7_CR1, SPE | CSTART);
	wr(H7_TXDR, 1U, 0x31U);
	CHECK_EQ("one frame is below two-frame RXP threshold", r4(H7_SR) & RXP, 0U);
	wr(H7_TXDR, 1U, 0x42U);
	CHECK_EQ("two frames assert RXP", r4(H7_SR) & RXP, RXP);
	packed = rd(H7_RXDR, 2U);
	CHECK_EQ("packed RX order", packed, 0xBDCEU);
	CHECK_EQ("packed read clears RXP", r4(H7_SR) & RXP, 0U);
}

static void gaps_dma_requests(void)
{
	gaps_reset();
	w4(H7_CR1, SPE);
	CHECK_EQ("TX DMA disabled: no request", tx_level, FALSE);
	gaps_reset();
	w4(CFG1, CFG1_RESET | TXDMAEN | RXDMAEN);
	w4(H7_CR1, SPE);
	CHECK_EQ("TXP with TXDMAEN produces request", tx_level, TRUE);
	w4(H7_TXDR, 0x5AU);
	CHECK_EQ("RXP with RXDMAEN produces request", rx_level, TRUE);
	CHECK_EQ("the RX stream's datum", rd(H7_RXDR, 1U) & 0xFFU, 0xA5U);
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "spec-gaps") == 0) {
		make(&athrill_device_stm32h7_spi_model, 0x40003800U);
		cs(TRUE);
		cs(FALSE);	/* the peer answers MOSI ^ 0xFF */
		gaps_register_file();
		gaps_tsize_and_flags();
		gaps_fifo_threshold();
		gaps_dma_requests();
		m->destroy(dev);
		printf("%d checks, %d failures\n", checks, failures);
		return failures == 0 ? 0 : 1;
	}
	test_f7();
	test_h7();
	printf("stm32-spi: ok\n");
	return 0;
}
