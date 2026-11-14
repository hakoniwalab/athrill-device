/*
 * STM32H7 SPI TSIZE/end-of-transfer behaviour for issue #112.
 *
 * Sources for the expectations below:
 * - RM0433 Rev 9 sections 50.4.11, 50.4.12 and 50.11.2/.6/.7: CSTART begins a TSIZE session, CTSIZE reports the
 *   remaining frames, TXTF is set when TSIZE frames have entered TxFIFO,
 *   EOT is set when those frames have finished, TXC mirrors EOT for
 *   TSIZE!=0, and EOTC/TXTFC clear the corresponding flags.  TSIZE=0 is an
 *   endless transaction and has no EOT event.
 * - NuttX stm32h7/stm32_spi.c:2181-2200 starts DMA and SPI before CSTART;
 *   lines 1836-1872 clear suspend, set CSTART, exchange, then set CSUSP.
 * - ChibiOS SPIv3/hal_spi_lld.c:858-873 starts both DMA streams then sets
 *   CSTART; lines 1058-1063 and 1101-1142 use CSUSP to abort/end a session.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define SPI2       0x40003800U
#define CR1        0x00U
#define CR2        0x04U
#define SR         0x14U
#define IFCR       0x18U
#define TXDR       0x20U

#define SPE        (1U << 0)
#define CSTART     (1U << 9)
#define CSUSP      (1U << 10)
#define EOT        (1U << 3)
#define TXTF       (1U << 4)
#define SUSP       (1U << 11)
#define TXC        (1U << 12)
#define EOTC       (1U << 3)
#define TXTFC      (1U << 4)
#define CTSIZE(s)  ((s) >> 16)

#define FRAMES     3U

static uint64 now;
static int checks;
static int failures;
static const AthrillDeviceModelOps *model = &athrill_device_stm32h7_spi_model;
static void *device;

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
	(void)line;
	(void)asserted;
	return 0;
}

static std_bool irq_is_active(void *context, AthrillDeviceIrqLine line)
{
	(void)context;
	(void)line;
	return FALSE;
}

static void end_batch(void *context)
{
	(void)context;
}

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)name;
	(void)index;
	line->value = 36U;
	return 0;
}

static sint32 no_spi_peripheral(void *context, uint32 chip_select,
	const AthrillDeviceSpiPeripheralOps **ops, void **instance)
{
	(void)context;
	(void)chip_select;
	(void)ops;
	(void)instance;
	return -1;
}

static sint32 unused_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	(void)context;
	(void)property;
	(void)index;
	(void)line;
	return -1;
}

static sint32 unused_watch(void *context, AthrillDeviceGpioLine line,
	AthrillDeviceGpioEdge callback, void *instance)
{
	(void)context;
	(void)line;
	(void)callback;
	(void)instance;
	return -1;
}

static sint32 dma_request(void *context, const char *name, std_bool asserted)
{
	(void)context;
	(void)name;
	(void)asserted;
	return 0;
}

static uint32 reg_cells[2] = { SPI2, 0x400U };
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
	TEST_CHECK(model->mmio->read(device, SPI2 + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void write_reg(uint32 offset, uint32 size, uint32 value)
{
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->write(device, SPI2 + offset, size, value) == 0);
}

static void reset_controller(void)
{
	TEST_CHECK(model->reset(device) == 0); /* model reset ABI: zero is success */
}

static void test_sized_transfer(void)
{
	uint32 status;

	reset_controller();
	write_reg(CR2, 4U, FRAMES);
	/* NuttX lines 2181-2186 and ChibiOS lines 868-873: SPE then CSTART. */
	write_reg(CR1, 4U, SPE | CSTART);
	status = read_reg(SR);
	/* RM0433 50.11.6 CTSIZE: all three frames remain at session start. */
	CHECK_EQ("start CTSIZE", CTSIZE(status), FRAMES);
	/* RM0433 50.11.6 EOT/TXC/TXTF: none is complete before frame 3. */
	CHECK_EQ("start completion flags", status & (EOT | TXTF | TXC), 0U);

	write_reg(TXDR, 1U, 0x11U);
	status = read_reg(SR);
	/* RM0433 50.11.6 CTSIZE counts one submitted/transmitted frame down. */
	CHECK_EQ("after frame 1 CTSIZE", CTSIZE(status), 2U);
	/* RM0433 50.11.6: frame 1 of 3 cannot set EOT, TXTF, or TXC. */
	CHECK_EQ("after frame 1 completion flags", status & (EOT | TXTF | TXC), 0U);

	write_reg(TXDR, 1U, 0x22U);
	status = read_reg(SR);
	/* RM0433 50.11.6 CTSIZE: one frame remains after the second unit. */
	CHECK_EQ("after frame 2 CTSIZE", CTSIZE(status), 1U);
	/* RM0433 50.11.6: frame 2 of 3 still cannot complete the session. */
	CHECK_EQ("after frame 2 completion flags", status & (EOT | TXTF | TXC), 0U);

	write_reg(TXDR, 1U, 0x33U);
	status = read_reg(SR);
	/* RM0433 50.11.6 CTSIZE reaches zero after TSIZE=3 data units. */
	CHECK_EQ("after frame 3 CTSIZE", CTSIZE(status), 0U);
	/* RM0433 50.11.6 TXTF: TSIZE units have now been written to TxFIFO. */
	CHECK_EQ("TXTF at TSIZE", status & TXTF, TXTF);
	/* RM0433 50.11.6 EOT/TXC: the third unit completes the sized transfer. */
	CHECK_EQ("EOT and TXC at TSIZE", status & (EOT | TXC), EOT | TXC);

	write_reg(IFCR, 4U, EOTC | TXTFC);
	status = read_reg(SR);
	/* RM0433 50.11.7: EOTC clears EOT and its TSIZE-mode TXC copy;
	 * TXTFC independently clears TXTF. */
	CHECK_EQ("IFCR clears completion flags", status & (EOT | TXTF | TXC), 0U);
}

static void test_endless_and_suspend(void)
{
	uint32 status;
	uint32 i;

	reset_controller();
	write_reg(CR2, 4U, 0U);
	/* RM0433 50.11.2: CSTART with TSIZE=0 starts an endless transaction. */
	write_reg(CR1, 4U, SPE | CSTART);
	/* RM0433 50.11.6: an endless transaction never produces EOT. */
	CHECK_EQ("endless start has no EOT", read_reg(SR) & EOT, 0U);
	for (i = 0U; i < 5U; i++) {
		write_reg(TXDR, 1U, 0x40U + i);
		/* RM0433 50.4.11: TSIZE=0 continues until software suspends it. */
		CHECK_EQ("endless data has no EOT", read_reg(SR) & EOT, 0U);
	}

	/* NuttX lines 1869-1872 and ChibiOS lines 1058-1063: CSUSP ends the
	 * active master session at a frame boundary. */
	write_reg(CR1, 4U, SPE | CSTART | CSUSP);
	status = read_reg(SR);
	/* RM0433 50.11.6 SUSP: hardware acknowledges the CSUSP request. */
	CHECK_EQ("CSUSP sets SUSP", status & SUSP, SUSP);
	/* RM0433 50.4.12: a completed suspend clears the active CSTART state. */
	CHECK_EQ("CSUSP clears CSTART", read_reg(CR1) & CSTART, 0U);
	/* RM0433 50.11.6: suspension of an endless session is not an EOT event. */
	CHECK_EQ("suspend has no EOT", read_reg(SR) & EOT, 0U);
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
		0U, "/soc/spi@40003800", "st,stm32h7-spi", &node_ops, NULL
	};
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.abi_minor = ATHRILL_DEVICE_ABI_MINOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.end_batch = end_batch;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.get_gpio = unused_gpio;
	resources.get_spi_peripheral = no_spi_peripheral;
	resources.watch_gpio = unused_watch;
	resources.dma_request = dma_request;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;

	TEST_CHECK(model->validate(&node, NULL, 0U) == 0); /* model ABI success */
	TEST_CHECK(model->create(&args, &device) == 0); /* model ABI success */
	TEST_CHECK(model->connect(device, &connect) == 0); /* model ABI success */
	test_sized_transfer();
	test_endless_and_suspend();
	model->destroy(device);
	printf("h7-spi-eot: %d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
