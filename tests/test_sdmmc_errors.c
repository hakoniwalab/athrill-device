/*
 * STM32F7/H7 SDMMC data-error and CMD12 behaviour for issue #114.
 *
 * Sources for the expectations below:
 * - RM0410 Rev 5 sections 39.3, 39.7.7-.9 (35.8 in the revision used by
 *   the existing model): DCRCFAIL/DTIMEOUT are data failures, DATAEND means
 *   the programmed data count ended, CMD12 stops an open-ended read, and
 *   ICR clears the corresponding static flags.
 * - RM0433 Rev 8 sections 55.6.2 and 55.10.11-.12: CMD12 with CMDSTOP may
 *   abort any active read, DABORT records that abort, DATAEND is only a
 *   correct data end, and ICR clears DABORT/data-error flags.
 * - NuttX stm32h7/stm32_sdmmc.c:2264-2283 emits CMDSTOP for MMCSD_STOPXFR;
 *   lines 1535-1557 stop the DPSM on transfer errors.
 * - ChibiOS SDMMCv1/hal_sdc_lld.c:306-330,343-388 and SDMMCv2 equivalents
 *   at 275-287,301-345 distinguish DATAEND from data errors, clear STA via
 *   ICR, and send CMD12 for multi-block operations.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_protocol_sd.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define F7_BASE     0x40012C00U
#define H7_BASE     0x48022400U
#define RAM_F7      0x20000000U
#define RAM_H7      0x24000000U

#define ARG         0x08U
#define CMD         0x0CU
#define DTIMER      0x24U
#define DLEN        0x28U
#define DCTRL       0x2CU
#define STA         0x34U
#define ICR         0x38U
#define IDMACTRL    0x50U
#define IDMABASE0   0x58U

#define DCRCFAIL    (1U << 1)
#define DTIMEOUT    (1U << 3)
#define DATAEND     (1U << 8)
#define DBCKEND     (1U << 10)
#define DABORT      (1U << 11)
#define DPSMACT     (1U << 12)

#define F7_DTDIR    (1U << 1)
#define F7_DTEN     (1U << 0)
#define F7_WAITRESP (1U << 6)
#define F7_CPSMEN   (1U << 10)

#define H7_CMDTRANS (1U << 6)
#define H7_CMDSTOP  (1U << 7)
#define H7_WAITRESP (1U << 8)
#define H7_CPSMEN   (1U << 12)
#define H7_DTDIR    (1U << 1)
#define H7_IDMAEN   (1U << 0)

typedef struct {
	std_bool h7;
	uint32 base;
	uint32 ram_base;
	uint64 now;
	uint8 memory[2048];
	uint32 command_count[64];
	uint32 read_count;
	uint32 read_length;
	uint32 done_count;
	std_bool fail_read;
	void *device;
} Fixture;

static int checks;
static int failures;
static const AthrillDeviceModelOps *model = &athrill_device_stm32_sdmmc_model;

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
	return ((Fixture *)context)->now;
}

static uint64 time_frequency_hz(void *context)
{
	/* PX4 board clocks used by the existing fixtures: F7 216 MHz, H7 480 MHz. */
	return ((Fixture *)context)->h7 ? 480000000U : 216000000U;
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

static sint32 memory_read(void *context, uint64 address, void *data, uint32 size)
{
	Fixture *fixture = context;
	if (address < fixture->ram_base ||
	    address + size > fixture->ram_base + sizeof(fixture->memory)) {
		return -1;
	}
	memcpy(data, &fixture->memory[address - fixture->ram_base], size);
	return 0;
}

static sint32 memory_write(void *context, uint64 address, const void *data,
	uint32 size)
{
	Fixture *fixture = context;
	if (address < fixture->ram_base ||
	    address + size > fixture->ram_base + sizeof(fixture->memory)) {
		return -1;
	}
	memcpy(&fixture->memory[address - fixture->ram_base], data, size);
	return 0;
}

static sint32 get_property(void *context, const char *name, const void **data,
	uint32 *size)
{
	Fixture *fixture = context;
	if (!fixture->h7 || strcmp(name, "athrill,stm32h7-sdmmc") != 0) {
		return -1;
	}
	*data = NULL;
	*size = 0U;
	return 0;
}

static sint32 get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	Fixture *fixture = context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = index == 0U ? fixture->base : 0x400U;
	return 0;
}

static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceNodeOps), get_property, get_u32, NULL, NULL
};

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	Fixture *fixture = context;
	(void)name;
	(void)index;
	line->value = fixture->h7 ? 124U : 49U;
	return 0;
}

static sint32 get_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	Fixture *fixture = context;
	if (fixture->h7 || strcmp(property, "athrill,d0-gpios") != 0 || index != 0U) {
		return -1;
	}
	line->value = 0x28U;
	return 0;
}

static sint32 gpio_drive(void *context, AthrillDeviceGpioLine line,
	std_bool level)
{
	(void)context;
	(void)line;
	(void)level;
	return 0;
}

static sint32 card_command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	Fixture *fixture = context;
	(void)argument;
	fixture->command_count[index]++;
	memset(response, 0, sizeof(*response));
	response->kind = ATHRILL_SD_RESPONSE_SHORT;
	response->has_index = TRUE;
	if (index == 18U) {
		response->data = ATHRILL_SD_DATA_READ;
	}
	return 0;
}

static sint32 card_read(void *context, uint8 *data, uint32 length)
{
	Fixture *fixture = context;
	fixture->read_count++;
	fixture->read_length = length;
	memset(data, 0x5AU, length);
	return fixture->fail_read ? -1 : 0;
}

static sint32 card_write(void *context, const uint8 *data, uint32 length)
{
	(void)context;
	(void)data;
	(void)length;
	return 0;
}

static sint32 card_transfer_done(void *context)
{
	((Fixture *)context)->done_count++;
	return 0;
}

static const AthrillSdCardOps card_ops = {
	card_command, card_read, card_write, card_transfer_done
};

static sint32 get_sd_card(void *context, const void **ops, void **instance)
{
	*ops = &card_ops;
	*instance = context;
	return 0;
}

static uint32 read_reg(Fixture *fixture, uint32 offset)
{
	uint64 value = 0U;
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->read(fixture->device, fixture->base + offset,
		4U, &value) == 0);
	return (uint32)value;
}

static void write_reg(Fixture *fixture, uint32 offset, uint32 value)
{
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->write(fixture->device, fixture->base + offset,
		4U, value) == 0);
}

static void tick(Fixture *fixture)
{
	AthrillDeviceTickResult result = { ATHRILL_DEVICE_NO_EVENT, TRUE, 0U };
	TEST_CHECK(model->tick(fixture->device, fixture->now, &result) == 0);
}

static void issue_command(Fixture *fixture, uint32 index, std_bool data,
	std_bool stop)
{
	write_reg(fixture, ARG, 0U);
	if (fixture->h7) {
		write_reg(fixture, CMD, index | H7_WAITRESP | H7_CPSMEN |
			(data ? H7_CMDTRANS : 0U) | (stop ? H7_CMDSTOP : 0U));
	}
	else {
		write_reg(fixture, CMD, index | F7_WAITRESP | F7_CPSMEN);
	}
}

/* ChibiOS SDMMCv1 arms DCTRL before CMD18; SDMMCv2 arms DLEN/IDMA,
 * issues CMD18 with CMDTRANS, then enables the data path. */
static void start_read(Fixture *fixture, uint32 length)
{
	write_reg(fixture, DTIMER, 0xFFFFFFFFU);
	write_reg(fixture, DLEN, length);
	if (fixture->h7) {
		write_reg(fixture, IDMABASE0, fixture->ram_base);
		write_reg(fixture, IDMACTRL, H7_IDMAEN);
		issue_command(fixture, 18U, TRUE, FALSE);
		write_reg(fixture, DCTRL, (9U << 4) | H7_DTDIR);
	}
	else {
		write_reg(fixture, DCTRL, (9U << 4) | F7_DTDIR | F7_DTEN);
		issue_command(fixture, 18U, TRUE, FALSE);
	}
}

static void reset_fixture(Fixture *fixture)
{
	fixture->now = 0U;
	fixture->fail_read = FALSE;
	fixture->read_count = 0U;
	fixture->read_length = 0U;
	fixture->done_count = 0U;
	memset(fixture->command_count, 0, sizeof(fixture->command_count));
	memset(fixture->memory, 0, sizeof(fixture->memory));
	TEST_CHECK(model->reset(fixture->device) == 0); /* model reset ABI success */
}

static void test_card_read_failure(Fixture *fixture)
{
	uint32 status;
	const char *kind = fixture->h7 ? "H7" : "F7";

	reset_fixture(fixture);
	fixture->fail_read = TRUE;
	start_read(fixture, 512U);
	fixture->now = 1000000U;
	tick(fixture);
	status = read_reg(fixture, STA);
	/* Test fake-card contract: CMD18 asks its read callback for one block. */
	CHECK_EQ(kind, fixture->read_count, 1U);
	/* SD Physical Layer block length and both drivers: one block is 512 bytes. */
	CHECK_EQ("failed read length", fixture->read_length, 512U);
	/* RM0410 39.7.8 / RM0433 55.10.11: a card data failure terminates
	 * with DCRCFAIL or DTIMEOUT, never silent success. */
	CHECK_EQ("failed read has data error",
		(status & (DCRCFAIL | DTIMEOUT)) != 0U, 1U);
	/* Those same STA definitions reserve DATAEND for a correctly ended path. */
	CHECK_EQ("failed read has no DATAEND", status & DATAEND, 0U);
	/* A failed block cannot report the CRC-passed DBCKEND condition. */
	CHECK_EQ("failed read has no DBCKEND", status & DBCKEND, 0U);
	write_reg(fixture, ICR, DCRCFAIL | DTIMEOUT);
	/* RM0410 39.7.9 / RM0433 55.10.12: ICR clears data-error flags. */
	CHECK_EQ("ICR clears data errors",
		read_reg(fixture, STA) & (DCRCFAIL | DTIMEOUT), 0U);
}

static void test_cmd12_abort(Fixture *fixture)
{
	uint32 status;

	reset_fixture(fixture);
	start_read(fixture, 1024U);
	if (fixture->h7) {
		/* RM0433 55.10.11: the H7 DPSM is active before its scheduled end. */
		CHECK_EQ("H7 read starts DPSM", read_reg(fixture, STA) & DPSMACT, DPSMACT);
	}
	/* NuttX H7 lines 2280-2283 and RM0433 55.6.2 require CMDSTOP for CMD12. */
	issue_command(fixture, 12U, FALSE, fixture->h7);
	/* SD Physical Layer CMD12: the Stop Transmission command reached the card. */
	CHECK_EQ("CMD12 reached card", fixture->command_count[12], 1U);
	fixture->now = 1000000U;
	tick(fixture);
	status = read_reg(fixture, STA);
	/* RM0410 39.3 and RM0433 55.6.2: CMD12 stops the active multi-block
	 * data path, so it cannot later finish as a normal DATAEND. */
	CHECK_EQ("CMD12 prevents DATAEND", status & DATAEND, 0U);
	/* Stopping before the scheduled read means the fake card supplies no block. */
	CHECK_EQ("CMD12 prevents card read", fixture->read_count, 0U);
	if (fixture->h7) {
		/* RM0433 55.10.11: a CMD12-aborted transfer sets DABORT. */
		CHECK_EQ("H7 CMD12 DABORT", status & DABORT, DABORT);
		write_reg(fixture, ICR, DABORT);
		/* RM0433 55.10.12: DABORTC clears the static DABORT flag. */
		CHECK_EQ("H7 ICR clears DABORT", read_reg(fixture, STA) & DABORT, 0U);
	}
}

static void create_fixture(Fixture *fixture, std_bool h7)
{
	static AthrillDeviceTargetOps target_ops[2];
	static AthrillDeviceResourceOps resources[2];
	static AthrillDeviceNode nodes[2];
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	uint32 slot = h7 ? 1U : 0U;

	memset(fixture, 0, sizeof(*fixture));
	fixture->h7 = h7;
	fixture->base = h7 ? H7_BASE : F7_BASE;
	fixture->ram_base = h7 ? RAM_H7 : RAM_F7;
	memset(&target_ops[slot], 0, sizeof(target_ops[slot]));
	target_ops[slot].abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops[slot].abi_minor = ATHRILL_DEVICE_ABI_MINOR;
	target_ops[slot].struct_size = sizeof(target_ops[slot]);
	target_ops[slot].time_now = time_now;
	target_ops[slot].time_frequency_hz = time_frequency_hz;
	target_ops[slot].irq_set = irq_set;
	target_ops[slot].irq_is_active = irq_is_active;
	target_ops[slot].end_batch = end_batch;
	target_ops[slot].memory_read = memory_read;
	target_ops[slot].memory_write = memory_write;
	memset(&resources[slot], 0, sizeof(resources[slot]));
	resources[slot].get_interrupt = get_interrupt;
	resources[slot].get_gpio = get_gpio;
	resources[slot].gpio_drive = gpio_drive;
	resources[slot].get_sd_card = get_sd_card;
	nodes[slot].path = h7 ? "/soc/sdmmc@48022400" : "/soc/sdmmc@40012c00";
	nodes[slot].matched_compatible = "st,stm32-sdmmc";
	nodes[slot].ops = &node_ops;
	nodes[slot].context = fixture;
	args.target_ops = &target_ops[slot];
	args.target = fixture;
	args.node = &nodes[slot];
	connect.resources = &resources[slot];
	connect.context = fixture;

	TEST_CHECK(model->validate(&nodes[slot], NULL, 0U) == 0); /* model ABI */
	TEST_CHECK(model->create(&args, &fixture->device) == 0); /* model ABI */
	TEST_CHECK(model->connect(fixture->device, &connect) == 0); /* model ABI */
	reset_fixture(fixture);
}

int main(void)
{
	Fixture f7;
	Fixture h7;

	create_fixture(&f7, FALSE);
	test_card_read_failure(&f7);
	test_cmd12_abort(&f7);
	model->destroy(f7.device);

	create_fixture(&h7, TRUE);
	test_card_read_failure(&h7);
	test_cmd12_abort(&h7);
	model->destroy(h7.device);

	printf("sdmmc-errors: %d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
