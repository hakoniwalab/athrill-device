/*
 * STM32H7 SPI frame byte order (ADV-SPI-H7-016BIT, #152 stage 3: ADIS16470 uses 16-bit frames).
 * RM0433 Rev 9 50.4.7 (CFG2 LSBFRST = 0 at reset: MSB first) and 50.4.13 (data packing: a data-register
 * access holds whole frames, the first frame in the least significant bits). Expected:
 * - DSIZE 16: one 16-bit TXDR write of 0x7200 puts 0x72 then 0x00 on MOSI; the bytes received
 *   (0xAB then 0xCD) read back from RXDR as 0xABCD.
 * - DSIZE 8 with a 16-bit access (two packed frames): frame 1 = low byte first, so 0x00 then 0x72;
 *   the received 0xAB, 0xCD read back as 0xCDAB (unchanged behaviour).
 * The older header of the copied harness: STM32H7 SPI TSIZE/end-of-transfer behaviour for issue #112.
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
#define RXDR       0x30U
#define CFG1       0x08U

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

static uint8 mosi_seen[8];
static uint32 mosi_count;
static const uint8 miso_script[2] = { 0xABU, 0xCDU };

static void peripheral_select(void *instance, std_bool selected)
{
	(void)instance;
	if (selected) {
		mosi_count = 0U;
	}
}

static uint8 peripheral_exchange(void *instance, uint8 mosi)
{
	uint8 miso = miso_script[mosi_count % 2U];
	(void)instance;
	if (mosi_count < sizeof(mosi_seen)) {
		mosi_seen[mosi_count] = mosi;
	}
	mosi_count++;
	return miso;
}

static const AthrillDeviceSpiPeripheralOps peripheral_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), peripheral_select, peripheral_exchange
};

static sint32 one_spi_peripheral(void *context, uint32 chip_select,
	const AthrillDeviceSpiPeripheralOps **ops, void **instance)
{
	(void)context;
	if (chip_select != 0U) {
		return -1;
	}
	*ops = &peripheral_ops;
	*instance = NULL;
	return 0;
}

static sint32 cs_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	(void)context;
	(void)property;
	(void)index;
	line->value = 1U;
	return 0;
}

static AthrillDeviceGpioEdge cs_edge;
static void *cs_instance;

static sint32 cs_watch(void *context, AthrillDeviceGpioLine line,
	AthrillDeviceGpioEdge callback, void *instance)
{
	(void)context;
	(void)line;
	cs_edge = callback;
	cs_instance = instance;
	return 0;
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

static uint32 read_reg(uint32 offset, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(model->mmio->read(device, SPI2 + offset, size, &value) == 0);
	return (uint32)value;
}

static void write_reg(uint32 offset, uint32 size, uint32 value)
{
	TEST_CHECK(model->mmio->write(device, SPI2 + offset, size, value) == 0);
}

static void transfer(uint32 dsize, uint32 frames, uint32 value)
{
	TEST_CHECK(model->reset(device) == 0);
	write_reg(CFG1, 4U, dsize);
	write_reg(CR2, 4U, frames);
	write_reg(CR1, 4U, SPE | CSTART);
	cs_edge(cs_instance, FALSE);	/* chip select low */
	write_reg(TXDR, 2U, value);
}

static void test_16bit_frame(void)
{
	transfer(15U, 1U, 0x7200U);
	CHECK_EQ("16-bit frame: bytes on MOSI", mosi_count, 2U);
	CHECK_EQ("16-bit frame: first MOSI byte is the high byte", mosi_seen[0], 0x72U);
	CHECK_EQ("16-bit frame: second MOSI byte", mosi_seen[1], 0x00U);
	CHECK_EQ("16-bit frame: RXDR", read_reg(RXDR, 2U), 0xABCDU);
	cs_edge(cs_instance, TRUE);
}

static void test_8bit_packed(void)
{
	transfer(7U, 2U, 0x7200U);
	CHECK_EQ("8-bit frames: bytes on MOSI", mosi_count, 2U);
	CHECK_EQ("8-bit frames: frame 1 is the low byte", mosi_seen[0], 0x00U);
	CHECK_EQ("8-bit frames: frame 2", mosi_seen[1], 0x72U);
	CHECK_EQ("8-bit frames: RXDR", read_reg(RXDR, 2U), 0xCDABU);
	cs_edge(cs_instance, TRUE);
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
	resources.struct_size = sizeof(resources);
	resources.get_interrupt = get_interrupt;
	resources.get_gpio = cs_gpio;
	resources.get_spi_peripheral = one_spi_peripheral;
	resources.watch_gpio = cs_watch;
	resources.dma_request = dma_request;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;

	TEST_CHECK(model->validate(&node, NULL, 0U) == 0);
	TEST_CHECK(model->create(&args, &device) == 0);
	TEST_CHECK(model->connect(device, &connect) == 0);
	TEST_CHECK(cs_edge != NULL);
	test_16bit_frame();
	test_8bit_packed();
	model->destroy(device);
	printf("h7-spi-frames: %d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
