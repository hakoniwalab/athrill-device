/*
 * STM32 DMA-v1 error behaviour for issue #113.
 *
 * Sources for the expectations below:
 * - RM0410 Rev 5 sections 8.3.19 (error management) and 8.5.1-.4: an AHB access
 *   error sets TEIF, hardware clears EN, TCIF is not a substitute for the
 *   failed transfer, TEIE gates the stream interrupt, DMEIF reports a
 *   direct-mode overrun, and LIFCR/HIFCR clear the written flags.
 * - RM0410 8.3.19 also says a DMEIF alone does not automatically disable a
 *   stream because no data is lost.
 * - NuttX stm32f7/stm32_dma.c:799-824 always adds TEIE when starting a
 *   normal stream; ChibiOS SPIv3/hal_spi_lld.c:155-160 and 196-202 treats
 *   TEIF and DMEIF as DMA errors.
 * DMEIF is not checked: it needs the memory bus to be busy while a
 * direct-mode datum waits, and Athrill grants every DMA access at once (no
 * bus arbitration), so the model never has the condition.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define DMA1       0x40026000U
#define RAM        0x20000000U
#define PERIPH     0x40000000U

#define LISR       0x00U
#define HISR       0x04U
#define LIFCR      0x08U
#define HIFCR      0x0CU
#define S(n, r)    (0x10U + 0x18U * (n) + 4U * (r))

#define CR_EN      (1U << 0)
#define CR_DMEIE   (1U << 1)
#define CR_TEIE    (1U << 2)
#define CR_TCIE    (1U << 4)
#define CR_M2P     (1U << 6)
#define CR_MINC    (1U << 10)
#define FCR_DMDIS  (1U << 2)

#define FEIF       (1U << 0)
#define DMEIF      (1U << 2)
#define TEIF       (1U << 3)
#define HTIF       (1U << 4)
#define TCIF       (1U << 5)

enum bus_fault {
	BUS_OK,
	BUS_MEMORY_ERROR,
	BUS_PERIPHERAL_ERROR
};

static enum bus_fault fault;
static uint8 memory[16];
static uint8 peripheral[16];
static uint32 peripheral_writes;
static uint32 memory_attempts;
static uint32 raised[8];
static uint64 now;
static const AthrillDeviceDmaProviderOps *provider;
static const AthrillDeviceModelOps *model = &athrill_device_stm32_dma_model;
static void *device;
static int checks;
static int failures;

static void check_eq(int line, const char *what, uint32 got, uint32 want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = 0x%x, want 0x%x\n", line, what, got, want);
	}
}

#define CHECK_EQ(what, got, want) check_eq(__LINE__, (what), (uint32)(got), (uint32)(want))

static uint64 time_now(void *context)
{
	(void)context;
	return now++;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line, std_bool asserted)
{
	(void)context;
	if (asserted && line.value >= 11U && line.value <= 18U) {
		raised[line.value - 11U]++;
	}
	return 0;
}

static sint32 memory_read(void *context, uint64 address, void *data, uint32 size)
{
	(void)context;
	if (address >= RAM && address + size <= RAM + sizeof(memory)) {
		if (fault == BUS_MEMORY_ERROR) {
			return -1;
		}
		memcpy(data, &memory[address - RAM], size);
		return 0;
	}
	if (address == PERIPH) {
		if (fault == BUS_PERIPHERAL_ERROR) {
			return -1;
		}
		memcpy(data, peripheral, size);
		return 0;
	}
	return -1;
}

static sint32 memory_write(void *context, uint64 address, const void *data,
	uint32 size)
{
	(void)context;
	if (address >= RAM && address + size <= RAM + sizeof(memory)) {
		memory_attempts++;
		if (fault == BUS_MEMORY_ERROR) {
			return -1;
		}
		memcpy(&memory[address - RAM], data, size);
		return 0;
	}
	if (address == PERIPH) {
		if (fault == BUS_PERIPHERAL_ERROR) {
			return -1;
		}
		memcpy(&peripheral[peripheral_writes++], data, size);
		return 0;
	}
	return -1;
}

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)name;
	if (index >= 8U) {
		return -1;
	}
	line->value = 11U + index;
	return 0;
}

static sint32 dma_provide(void *context,
	const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	(void)context;
	TEST_CHECK(instance == device);
	provider = ops;
	return 0;
}

static uint32 reg_cells[2] = { DMA1, 0x400U };
static sint32 get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	(void)context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = reg_cells[index];
	return 0;
}

static uint32 read_reg(uint32 offset)
{
	uint64 value = 0U;
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->read(device, DMA1 + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void write_reg(uint32 offset, uint32 value)
{
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->write(device, DMA1 + offset, 4U, value) == 0);
}

static void request(uint32 stream, std_bool asserted)
{
	AthrillDeviceDmaRequestLine line;
	memset(&line, 0, sizeof(line));
	line.cell_count = 2U;
	line.cells[0] = stream;
	line.cells[1] = 0U;
	/* Athrill DMA provider ABI: zero means the request line was accepted. */
	TEST_CHECK(provider->request(device, &line, asserted) == 0);
}

static void reset_fixture(void)
{
	TEST_CHECK(model->reset(device) == 0); /* model reset ABI: zero is success */
	memset(memory, 0, sizeof(memory));
	memset(peripheral, 0, sizeof(peripheral));
	memset(raised, 0, sizeof(raised));
	peripheral_writes = 0U;
	memory_attempts = 0U;
	fault = BUS_OK;
}

static void program(uint32 stream, uint32 par, uint32 m0ar, uint32 count,
	uint32 cr, std_bool fifo_mode)
{
	write_reg(S(stream, 2), par);
	write_reg(S(stream, 3), m0ar);
	write_reg(S(stream, 1), count);
	write_reg(S(stream, 5), fifo_mode ? FCR_DMDIS : 0U);
	write_reg(S(stream, 0), cr | CR_EN);
}

static void test_normal_transfer(void)
{
	reset_fixture();
	memory[0] = 0x5AU;
	memory[1] = 0xA5U;
	program(7U, PERIPH, RAM, 2U, CR_M2P | CR_MINC | CR_TCIE, TRUE);
	request(7U, TRUE);
	request(7U, FALSE);
	/* RM0410 8.3.14: two successful requested units reach the peripheral. */
	CHECK_EQ("normal peripheral writes", peripheral_writes, 2U);
	/* RM0410 8.3.14: a normal non-circular transfer clears EN. */
	CHECK_EQ("normal stream disabled", read_reg(S(7U, 0)) & CR_EN, 0U);
	/* RM0410 8.5.2: stream 7 TCIF is HISR bit 27 (shift 22 + bit 5). */
	CHECK_EQ("normal TCIF7", read_reg(HISR) & (TCIF << 22), TCIF << 22);
	/* RM0410 8.4 (Table 35): TCIE produces the stream 7 interrupt. */
	CHECK_EQ("normal TC interrupt", raised[7], 1U);
	write_reg(HIFCR, TCIF << 22);
	/* RM0410 8.5.4: writing CTCIF7 clears TCIF7. */
	CHECK_EQ("normal HIFCR clear", read_reg(HISR) & (TCIF << 22), 0U);
}

static void test_memory_access_error(void)
{
	reset_fixture();
	fault = BUS_MEMORY_ERROR;
	program(0U, PERIPH, RAM, 1U,
		CR_M2P | CR_DMEIE | CR_TEIE, TRUE);
	request(0U, TRUE);
	request(0U, FALSE);
	/* RM0410 8.3.19/8.5.1: a memory AHB access failure sets TEIF0. */
	CHECK_EQ("memory error TEIF0", read_reg(LISR) & TEIF, TEIF);
	/* RM0410 8.5.1: a failed item must not be reported as TCIF0. */
	CHECK_EQ("memory error no TCIF0", read_reg(LISR) & TCIF, 0U);
	/* RM0410 8.5.5 EN: hardware clears EN on an AHB transfer error. */
	CHECK_EQ("memory error disables stream", read_reg(S(0U, 0)) & CR_EN, 0U);
	/* RM0410 Table 35: TEIF with TEIE raises stream 0's interrupt. */
	CHECK_EQ("memory error TE interrupt", raised[0], 1U);
	write_reg(LIFCR, TEIF);
	/* RM0410 8.5.3: CTEIF0 in LIFCR clears TEIF0. */
	CHECK_EQ("memory error LIFCR clear", read_reg(LISR) & TEIF, 0U);
}

static void test_peripheral_access_error(void)
{
	reset_fixture();
	memory[0] = 0xC3U;
	fault = BUS_PERIPHERAL_ERROR;
	program(4U, PERIPH, RAM, 1U,
		CR_M2P | CR_DMEIE | CR_TEIE, TRUE);
	request(4U, TRUE);
	request(4U, FALSE);
	/* RM0410 8.3.19/8.5.2: a peripheral AHB failure sets TEIF4. */
	CHECK_EQ("peripheral error TEIF4", read_reg(HISR) & TEIF, TEIF);
	/* RM0410 8.5.2: a failed item must not be reported as TCIF4. */
	CHECK_EQ("peripheral error no TCIF4", read_reg(HISR) & TCIF, 0U);
	/* RM0410 8.5.5 EN: a transfer error disables stream 4. */
	CHECK_EQ("peripheral error disables stream", read_reg(S(4U, 0)) & CR_EN, 0U);
	/* RM0410 Table 35: TEIE turns TEIF4 into stream 4's interrupt. */
	CHECK_EQ("peripheral error TE interrupt", raised[4], 1U);
	write_reg(HIFCR, TEIF);
	/* RM0410 8.5.4: CTEIF4 in HIFCR clears TEIF4. */
	CHECK_EQ("peripheral error HIFCR clear", read_reg(HISR) & TEIF, 0U);
}

int main(void)
{
	AthrillDeviceTargetOps target_ops;
	AthrillDeviceNodeOps node_ops = {
		ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
		sizeof(AthrillDeviceNodeOps), NULL, get_u32, NULL, NULL
	};
	AthrillDeviceResourceOps resources;
	AthrillDeviceNode node = {
		0U, "/soc/dma@40026000", "st,stm32-dma-v1", &node_ops, NULL
	};
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.abi_minor = ATHRILL_DEVICE_ABI_MINOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.memory_read = memory_read;
	target_ops.memory_write = memory_write;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.dma_provide = dma_provide;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;

	TEST_CHECK(model->validate(&node, NULL, 0U) == 0); /* model ABI success */
	TEST_CHECK(model->create(&args, &device) == 0); /* model ABI success */
	TEST_CHECK(model->connect(device, &connect) == 0); /* model ABI success */
	TEST_CHECK(provider != NULL);
	test_normal_transfer();
	test_memory_access_error();
	test_peripheral_access_error();
	model->destroy(device);
	printf("dma-errors: %d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
