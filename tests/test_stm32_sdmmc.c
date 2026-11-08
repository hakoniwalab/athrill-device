/*
 * The STM32 SDMMC model (src/models/stm32_sdmmc.c, stm32h7_sdmmc.c) with the
 * SD card model (src/models/sdcard.c) and, for the F7, the DMA model
 * (src/models/stm32_dma.c) on request lines. Ported from
 * athrill-target-stm32's test_sdmmc_sdcard.c (F7 SDMMC1: identification,
 * the SCR through the FIFO, block reads and writes by DMA2 stream 6, the
 * time a transfer takes) and test_stm32h7_sdmmc.c (H7 SDMMC2: identification
 * and IDMA block transfers): their checks unchanged, the models created as
 * the Devicetree runtime creates them, on a fake host and bus.
 * Expectations: RM0410 35.8 / RM0433 55.9 registers, the SD Physical Layer
 * Simplified Specification (commands and responses), and the bus time
 * (len * 8 / width + 100 card clocks).
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_protocol_sd.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* ---- the fake host: clock, interrupts, RAM and the bus ---- */
static uint64 now;
static uint64 cpu_hz;
static uint32 raised[160];
static uint8 ram[0x2000];
static uint32 ram_base;
static const char *image;
static uint32 gpio_high;		/* the last line driven high + 1 */

static void *sdmmc;			/* the controller under test */
static uint32 sd_base;
static void *dma2;			/* the F7 DMA2 model */
static const AthrillDeviceDmaProviderOps *dma2_provider;

static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency_hz(void *t) { (void)t; return cpu_hz; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool on)
{
	(void)t;
	if (on && line.value < 160U) {
		raised[line.value]++;
	}
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine line) { (void)t; (void)line; return FALSE; }
static void end_batch(void *t) { (void)t; }
static sint32 gpio_drive(void *t, AthrillDeviceGpioLine line, std_bool level)
{
	(void)t;	/* resources (ABI 1.16) */
	if (level) {
		gpio_high = (uint32)line.value + 1U;
	}
	return 0;
}
static sint32 host_value(void *t, const char *key, const char **value)
{
	(void)t;
	if (strcmp(key, "DEVICE_CONFIG_STM32_SDCARD_IMAGE") != 0 || image == NULL) {
		return -1;
	}
	*value = image;
	return 0;
}
/* RAM, and the controller's registers (the DMA reads and writes its FIFO) */
static sint32 memory_read(void *t, uint64 address, void *data, uint32 size)
{
	uint64 value = 0U;
	(void)t;
	if (address >= ram_base && address + size <= ram_base + sizeof(ram)) {
		memcpy(data, &ram[address - ram_base], size);
		return 0;
	}
	if (address >= sd_base && address < sd_base + 0x400U) {
		TEST_CHECK(athrill_device_stm32_sdmmc_model.mmio->read(sdmmc, address, size, &value) == 0);
		memcpy(data, &value, size);
		return 0;
	}
	return -1;
}
static sint32 memory_write(void *t, uint64 address, const void *data, uint32 size)
{
	uint32 value = 0U;
	(void)t;
	if (address >= ram_base && address + size <= ram_base + sizeof(ram)) {
		memcpy(&ram[address - ram_base], data, size);
		return 0;
	}
	if (address >= sd_base && address < sd_base + 0x400U) {
		memcpy(&value, data, size);
		return athrill_device_stm32_sdmmc_model.mmio->write(sdmmc, address, size, value);
	}
	return -1;
}

static AthrillDeviceTargetOps target_ops;

/* ---- the node and its resources ---- */
static uint32 node_reg[2];
static std_bool node_h7;
static uint32 node_irq;
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = node_reg[index];
	return 0;
}
static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c;
	if (!node_h7 || strcmp(name, "athrill,stm32h7-sdmmc") != 0) {
		return -1;
	}
	*data = NULL;
	*size = 0U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, NULL
};

static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	static const uint32 dma2_irqs[8] = { 56U, 57U, 58U, 59U, 60U, 68U, 69U, 70U };
	(void)name;
	if (c == &dma2) {
		if (index >= 8U) {
			return -1;
		}
		line->value = dma2_irqs[index];
		return 0;
	}
	line->value = node_irq;
	return 0;
}
static sint32 get_gpio(void *c, const char *property, uint32 index, AthrillDeviceGpioLine *line)
{
	(void)c;
	if (node_h7 || strcmp(property, "athrill,d0-gpios") != 0 || index != 0U) {
		return -1;
	}
	line->value = 0x28U;	/* PC8 */
	return 0;
}
static const void *card_ops;
static void *card_instance;
static sint32 sd_card_provide(void *c, const void *ops, void *instance)
{
	(void)c;
	card_ops = ops;
	card_instance = instance;
	return 0;
}
static sint32 get_sd_card(void *c, const void **ops, void **instance)
{
	(void)c;
	if (card_ops == NULL) {
		return -1;
	}
	*ops = card_ops;
	*instance = card_instance;
	return 0;
}
static sint32 dma_provide(void *c, const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	(void)c;
	(void)instance;
	dma2_provider = ops;
	return 0;
}
/* SDMMC1's dmas: "rx" and "tx" on DMA2 stream 6, channel 4 */
static sint32 dma_request(void *c, const char *name, std_bool asserted)
{
	AthrillDeviceDmaRequestLine line;
	(void)c;
	if (strcmp(name, "rx") != 0 && strcmp(name, "tx") != 0) {
		return -1;
	}
	if (dma2_provider == NULL) {
		return 0;
	}
	memset(&line, 0, sizeof(line));
	line.cell_count = 4U;
	line.cells[0] = 6U;
	line.cells[1] = 4U;
	return dma2_provider->request(dma2, &line, asserted);
}

static AthrillDeviceResourceOps resources;

static void *create(const AthrillDeviceModelOps *m, const char *path, const char *compatible,
	void *context)
{
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	void *instance = NULL;

	node.path = path;
	node.matched_compatible = compatible;
	node.ops = &node_ops;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = context;
	TEST_CHECK(m->validate == NULL || m->validate(&node, NULL, 0U) == 0);
	TEST_CHECK(m->create(&args, &instance) == 0);
	TEST_CHECK(m->connect(instance, &connect) == 0);
	return instance;
}

/* ---- checks ---- */
static int failures;
static int checks;
#define CHECK_EQ(what, got, want) check_eq(__LINE__, what, (long long)(got), (long long)(want))
static void check_eq(int line, const char *what, long long got, long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = %lld (0x%llx), want %lld (0x%llx)\n", line, what, got, got, want, want);
	}
}

#define CLKCR	0x04U
#define ARG	0x08U
#define CMD	0x0CU
#define RESPCMD	0x10U
#define RESP1	0x14U
#define RESP2	0x18U
#define DLEN	0x28U
#define DCTRL	0x2CU
#define STA	0x34U
#define ICR	0x38U
#define MASK	0x3CU
#define FIFO	0x80U
#define IDMACTRL	0x50U
#define IDMABASE0	0x58U
#define DMA2	0x40026400U
#define S6	(0x10U + (6U * 0x18U))	/* DMA2 stream 6 */

#define CTIMEOUT	(1U << 2)
#define CMDREND		(1U << 6)
#define CMDSENT		(1U << 7)
#define DATAEND		(1U << 8)
#define DPSMACT		(1U << 12)
#define RXDAVL		(1U << 21)

static void wr(uint32 offset, uint32 value)
{
	TEST_CHECK(athrill_device_stm32_sdmmc_model.mmio->write(sdmmc, sd_base + offset, 4U, value) == 0);
}
static uint32 rd(uint32 offset)
{
	uint64 value = 0U;
	TEST_CHECK(athrill_device_stm32_sdmmc_model.mmio->read(sdmmc, sd_base + offset, 4U, &value) == 0);
	return (uint32)value;
}
static void tick(void)
{
	AthrillDeviceTickResult result;
	result.next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result.can_skip = TRUE;
	TEST_CHECK(athrill_device_stm32_sdmmc_model.tick(sdmmc, now, &result) == 0);
}
static void dma_wr(uint32 addr, uint32 value)
{
	TEST_CHECK(athrill_device_stm32_dma_model.mmio->write(dma2, addr, 4U, value) == 0);
}
static uint32 dma_rd(uint32 addr)
{
	uint64 value = 0U;
	TEST_CHECK(athrill_device_stm32_dma_model.mmio->read(dma2, addr, 4U, &value) == 0);
	return (uint32)value;
}

static void write_image(const char *path)
{
	FILE *f = fopen(path, "wb");
	uint8 block[512];
	uint32 i;

	/* 1 MiB image, block n starts with n */
	for (i = 0U; i < 2048U; i++) {
		memset(block, (int)(i & 0xFFU), sizeof(block));
		(void)fwrite(block, 1U, sizeof(block), f);
	}
	(void)fclose(f);
}

static void *card;

/* the controller, its card (the node's child) and, F7, DMA2 */
static void setup(std_bool h7)
{
	if (sdmmc != NULL) {
		athrill_device_stm32_sdmmc_model.destroy(sdmmc);
		athrill_device_sdcard_model.destroy(card);
		sdmmc = NULL;
	}
	if (dma2 != NULL) {
		athrill_device_stm32_dma_model.destroy(dma2);
		dma2 = NULL;
		dma2_provider = NULL;
	}
	node_h7 = h7;
	cpu_hz = h7 ? 480000000U : 216000000U;
	ram_base = h7 ? 0x24000000U : 0x20000000U;
	sd_base = h7 ? 0x48022400U : 0x40012C00U;
	node_irq = h7 ? 124U : 49U;
	image = h7 ? "test_stm32h7_sdmmc.img" : "test_sdmmc_sdcard.img";
	write_image(image);
	now = 0U;
	gpio_high = 0U;
	memset(ram, 0, sizeof(ram));
	memset(raised, 0, sizeof(raised));
	card_ops = NULL;
	if (!h7) {
		node_reg[0] = DMA2;
		node_reg[1] = 0x400U;
		dma2 = create(&athrill_device_stm32_dma_model, "/soc/dma@40026400", "st,stm32-dma-v1", &dma2);
		TEST_CHECK(athrill_device_stm32_dma_model.reset(dma2) == 0);
	}
	node_reg[0] = sd_base;
	node_reg[1] = 0x400U;
	sdmmc = create(&athrill_device_stm32_sdmmc_model, "/soc/sdmmc", "st,stm32-sdmmc", NULL);
	card = create(&athrill_device_sdcard_model, "/soc/sdmmc/sd-card", "zephyr,sdmmc-disk", NULL);
	TEST_CHECK(athrill_device_sdcard_model.reset(card) == 0);
	TEST_CHECK(athrill_device_stm32_sdmmc_model.reset(sdmmc) == 0);
}

/* ---- F7 (test_sdmmc_sdcard.c) ---- */

/* NuttX stm32_sendcmd: WAITRESP 0 none, 1 short, 3 long; returns STA. */
static uint32 command(uint32 index, uint32 arg, uint32 wait)
{
	wr(ICR, 0x004005FFU);
	wr(ARG, arg);
	wr(CMD, index | (wait << 6) | (1U << 10));
	return rd(STA);
}

static void test_identify(void)
{
	uint32 rca;

	setup(FALSE);
	CHECK_EQ("D0 driven high (PC8)", gpio_high, 0x28U + 1U);
	CHECK_EQ("CMD0: sent", command(0U, 0U, 0U) & CMDSENT, CMDSENT);
	CHECK_EQ("CMD5 (SDIO probe): timeout", command(5U, 0U, 1U) & CTIMEOUT, CTIMEOUT);
	CHECK_EQ("CMD8: response", command(8U, 0x1AAU, 1U) & CMDREND, CMDREND);
	CHECK_EQ("CMD8: echo", rd(RESP1), 0x1AAU);
	CHECK_EQ("CMD8: RESPCMD", rd(RESPCMD), 8U);
	(void)command(55U, 0U, 1U);
	CHECK_EQ("CMD55: APP_CMD", rd(RESP1) & (1U << 5), 1U << 5);
	(void)command(41U, 0x40100000U, 1U);
	CHECK_EQ("ACMD41: ready, high capacity", rd(RESP1) & 0xC0000000U, 0xC0000000U);
	CHECK_EQ("ACMD41: RESPCMD 0x3F (R3)", rd(RESPCMD), 0x3FU);
	CHECK_EQ("CMD2: long response", command(2U, 0U, 3U) & CMDREND, CMDREND);
	(void)command(3U, 0U, 1U);
	rca = rd(RESP1) >> 16;
	CHECK_EQ("CMD3: RCA", rca, 0x1234U);
	(void)command(9U, rca << 16, 3U);
	CHECK_EQ("CSD 2.0", rd(RESP1) >> 30, 1U);
	/* C_SIZE bits 69..48: RESP2 [5:0] and RESP3 [31:16]; 1 MiB = 2 x 512 KiB */
	CHECK_EQ("C_SIZE", ((rd(RESP2) & 0x3FU) << 16) | (rd(0x1CU) >> 16), 1U);
	(void)command(7U, rca << 16, 1U);
	(void)command(13U, rca << 16, 1U);
	CHECK_EQ("CMD13: transfer state", (rd(RESP1) >> 9) & 15U, 4U);
}

static void test_scr_fifo(void)
{
	uint32 w0;

	test_identify();
	/* NuttX recvsetup: DLEN 8, 8-byte blocks, card to controller, no DMA; then CMD55 + ACMD51 */
	wr(DLEN, 8U);
	wr(DCTRL, (3U << 4) | 3U);
	wr(MASK, (1U << 8) | (1U << 15));	/* DATAEND, RXFIFOHF */
	(void)command(55U, 0x1234U << 16, 1U);
	(void)command(51U, 0U, 1U);
	CHECK_EQ("ACMD51: no data yet", rd(STA) & DATAEND, 0U);
	now = 100000U;
	tick();
	CHECK_EQ("DATAEND, data in the FIFO", rd(STA) & (DATAEND | RXDAVL), DATAEND | RXDAVL);
	CHECK_EQ("interrupt", raised[49] != 0U, 1);
	w0 = rd(FIFO);
	CHECK_EQ("SCR: SD 2.0 (byte 0 low), 4-bit bus", w0 & 0x0F0FU, 0x0502U);
	(void)rd(FIFO);
	CHECK_EQ("FIFO empty", rd(STA) & RXDAVL, 0U);
}

/* NuttX dmarecvsetup / dmasendsetup: the stream of the FIFO, 32-bit, PFCTRL, MINC, TCIE */
static void dma_stream(uint32 memory, std_bool to_memory)
{
	uint32 cr = (1U << 5) | (1U << 10) | (2U << 11) | (2U << 13) | (1U << 4) | (4U << 25);

	cr |= to_memory ? 0U : (1U << 6);
	dma_wr(DMA2 + S6 + 0x08U, sd_base + FIFO);
	dma_wr(DMA2 + S6 + 0x0CU, memory);
	dma_wr(DMA2 + S6 + 0x04U, 128U);
	dma_wr(DMA2 + S6, cr | 1U);
}

static void test_dma_blocks(void)
{
	uint64 start;

	test_identify();
	wr(CLKCR, (1U << 11) | 1U);	/* 4-bit bus, 48 MHz / 3 = 16 MHz */
	wr(MASK, 1U << 8);
	/* read block 3: data path and stream before CMD17 */
	wr(DLEN, 512U);
	wr(DCTRL, (9U << 4) | 3U | (1U << 3));
	dma_stream(ram_base + 0x100U, TRUE);
	start = now;
	(void)command(17U, 3U, 1U);
	now = start + 13000U;
	tick();
	CHECK_EQ("read: not before the bus time", rd(STA) & DATAEND, 0U);
	/* 512 bytes on 4 bits = 1024 cycles + 100 at 16 MHz = 70.3 us = 15187 CPU clocks */
	now = start + 15188U;
	tick();
	CHECK_EQ("read: DATAEND", rd(STA) & DATAEND, DATAEND);
	CHECK_EQ("read: block 3 in memory", ram[0x100] | (ram[0x2FF] << 8), 0x0303U);
	CHECK_EQ("read: DMA TCIF6 (HISR bit 21)", dma_rd(DMA2 + 0x04U) & (1U << 21), 1U << 21);
	CHECK_EQ("read: the FIFO emptied by the DMA", rd(STA) & RXDAVL, 0U);
	/* write block 5 from memory: CMD24, then the data path and the stream */
	memset(&ram[0x400], 0xA5, 512U);
	wr(ICR, 0x004005FFU);
	dma_wr(DMA2 + 0x0CU, 0xFFFFFFFFU);
	(void)command(24U, 5U, 1U);
	wr(DLEN, 512U);
	wr(DCTRL, (9U << 4) | 1U | (1U << 3));
	dma_stream(ram_base + 0x400U, FALSE);
	/* the stream fills the FIFO when it starts: it completes before DATAEND */
	CHECK_EQ("write: DMA TCIF6 when the data are in", dma_rd(DMA2 + 0x04U) & (1U << 21), 1U << 21);
	start = now;
	wr(MASK, 1U << 8);	/* NuttX configxfrints after dmastart: the transfer starts */
	now = start + 15000U;
	tick();
	CHECK_EQ("write: not before the bus time", rd(STA) & DATAEND, 0U);
	now = start + 15188U;
	tick();
	CHECK_EQ("write: DATAEND", rd(STA) & DATAEND, DATAEND);
	/* read it back */
	wr(ICR, 0x004005FFU);
	wr(DLEN, 512U);
	wr(DCTRL, (9U << 4) | 3U | (1U << 3));
	dma_stream(ram_base + 0x800U, TRUE);
	(void)command(17U, 5U, 1U);
	now += 20000U;
	tick();
	CHECK_EQ("written block read back", ram[0x800] | (ram[0x9FF] << 8), 0xA5A5U);
}

/* ---- H7 (test_stm32h7_sdmmc.c) ---- */

static uint32 h7_command(uint32 index, uint32 arg, uint32 wait, std_bool data)
{
	wr(ICR, 0x1FE00FFFU);
	wr(ARG, arg);
	wr(CMD, index | (data ? (1U << 6) : 0U) | (wait << 8) | (1U << 12));
	return rd(STA);
}

static void h7_identify(void)
{
	uint32 rca;

	CHECK_EQ("no D0 line on the H7 node", gpio_high, 0U);
	CHECK_EQ("CMD0 sent", h7_command(0U, 0U, 0U, FALSE) & CMDSENT, CMDSENT);
	CHECK_EQ("CMD5 timeout", h7_command(5U, 0U, 1U, FALSE) & CTIMEOUT, CTIMEOUT);
	CHECK_EQ("CMD8 response", h7_command(8U, 0x1AAU, 1U, FALSE) & CMDREND, CMDREND);
	CHECK_EQ("CMD8 echo", rd(RESP1), 0x1AAU);
	CHECK_EQ("CMD8 index", rd(RESPCMD), 8U);
	(void)h7_command(55U, 0U, 1U, FALSE);
	(void)h7_command(41U, 0x40100000U, 1U, FALSE);
	CHECK_EQ("ACMD41 ready", rd(RESP1) & 0xC0000000U, 0xC0000000U);
	(void)h7_command(2U, 0U, 3U, FALSE);
	(void)h7_command(3U, 0U, 1U, FALSE);
	rca = rd(RESP1) >> 16;
	CHECK_EQ("RCA", rca, 0x1234U);
	(void)h7_command(7U, rca << 16, 1U, FALSE);
}

static void idma(uint32 address, std_bool read)
{
	wr(IDMACTRL, 0U);
	wr(DLEN, 512U);
	wr(DCTRL, (9U << 4) | (read ? (1U << 1) : 0U));
	wr(IDMABASE0, address);
	wr(IDMACTRL, 1U);
	wr(MASK, DATAEND);
}

static void test_idma_blocks(void)
{
	uint64 start;

	setup(TRUE);
	h7_identify();
	wr(CLKCR, (1U << 14) | 5U); /* four bits, PLL1Q 240 MHz / (2 * 5) = 24 MHz */
	idma(ram_base + 0x100U, TRUE);
	start = now;
	(void)h7_command(17U, 3U, 1U, TRUE);
	CHECK_EQ("read active", rd(STA) & DPSMACT, DPSMACT);
	now = start + 22000U;
	tick();
	CHECK_EQ("not before bus time", rd(STA) & DATAEND, 0U);
	now = start + 22500U;
	tick();
	CHECK_EQ("read DATAEND", rd(STA) & DATAEND, DATAEND);
	CHECK_EQ("read block 3", ram[0x100U] | (ram[0x2FFU] << 8), 0x0303U);
	CHECK_EQ("SDMMC2 IRQ", raised[124] != 0U, TRUE);

	memset(&ram[0x400U], 0xA5, 512U);
	idma(ram_base + 0x400U, FALSE);
	start = now;
	(void)h7_command(24U, 5U, 1U, TRUE);
	now = start + 22500U;
	tick();
	CHECK_EQ("write DATAEND", rd(STA) & DATAEND, DATAEND);

	idma(ram_base + 0x800U, TRUE);
	(void)h7_command(17U, 5U, 1U, TRUE);
	now += 22500U;
	tick();
	CHECK_EQ("written block read back", ram[0x800U] | (ram[0x9FFU] << 8), 0xA5A5U);
}

/* no image: the card answers nothing (as no card did before) */
static void test_no_image(void)
{
	const char *keep = NULL;

	setup(FALSE);
	keep = image;
	image = NULL;
	TEST_CHECK(athrill_device_sdcard_model.reset(card) == 0);
	CHECK_EQ("no image: CMD8 times out", command(8U, 0x1AAU, 1U) & CTIMEOUT, CTIMEOUT);
	image = keep;
}

int main(void)
{
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.time_frequency_hz = time_frequency_hz;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.end_batch = end_batch;
	target_ops.memory_read = memory_read;
	target_ops.memory_write = memory_write;
	target_ops.host_value = host_value;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.get_gpio = get_gpio;
	resources.sd_card_provide = sd_card_provide;
	resources.get_sd_card = get_sd_card;
	resources.dma_provide = dma_provide;
	resources.dma_request = dma_request;
	resources.gpio_drive = gpio_drive;

	test_identify();
	test_scr_fifo();
	test_dma_blocks();
	test_idma_blocks();
	test_no_image();
	athrill_device_stm32_sdmmc_model.destroy(sdmmc);
	athrill_device_sdcard_model.destroy(card);
	athrill_device_stm32_dma_model.destroy(dma2);
	(void)unlink("test_sdmmc_sdcard.img");
	(void)unlink("test_stm32h7_sdmmc.img");
	printf("%d checks, %d failures\n", checks, failures);
	return (failures == 0) ? 0 : 1;
}
