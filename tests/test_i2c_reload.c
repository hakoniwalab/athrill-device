/*
 * STM32 I2C-v2 reload behaviour for issue #111.
 *
 * Sources for the expectations below:
 * - RM0410 Rev 5, Figure 365/366 and sections 33.9.2 and 33.9.7: NBYTES is
 *   eight bits, RELOAD makes completion of NBYTES set TCR, writing a nonzero
 *   NBYTES clears TCR, and the last chunk with RELOAD=AUTOEND=0 sets TC.
 * - NuttX stm32f7/stm32_i2c.c:2028-2070: payloads over 255 bytes are sent as
 *   repeated 255-byte RELOAD chunks followed by a final short chunk.
 * - ChibiOS I2Cv3/hal_i2c_lld.c:232-251 and 261-280: both receive and
 *   transmit rewrite CR2 with n=255 and RELOAD while more than 255 remain.
 *
 * In a 600-byte transfer the first two CR2 reload values are therefore
 * intentionally identical.  A register-change detector must not stall it.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define I2C3       0x40005C00U
#define CR1        0x00U
#define CR2        0x04U
#define ISR        0x18U
#define RXDR       0x24U
#define TXDR       0x28U

#define PE         (1U << 0)
#define TXIS       (1U << 1)
#define RXNE       (1U << 2)
#define TC         (1U << 6)
#define TCR        (1U << 7)
#define RD_WRN     (1U << 10)
#define START      (1U << 13)
#define RELOAD     (1U << 24)

#define ADDRESS    0x50U
#define CHUNK      255U
#define LAST       90U
#define TOTAL      600U

typedef struct {
	uint32 writes;
	uint32 reads;
} CountingChip;

static CountingChip chip;
static uint64 now;
static int checks;
static int failures;

static void check_eq(int line, const char *what, uint32 got, uint32 want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = %u (0x%x), want %u (0x%x)\n",
			line, what, got, got, want, want);
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

static std_bool chip_start(void *instance, std_bool read)
{
	(void)instance;
	(void)read;
	return TRUE;
}

static void chip_write(void *instance, uint8 value)
{
	CountingChip *counting = instance;
	(void)value;
	counting->writes++;
}

static uint8 chip_read(void *instance)
{
	CountingChip *counting = instance;
	return (uint8)counting->reads++;
}

static void chip_stop(void *instance)
{
	(void)instance;
}

static const AthrillDeviceI2cPeripheralOps chip_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceI2cPeripheralOps),
	NULL, chip_start, chip_write, chip_read, chip_stop
};

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)name;
	line->value = 72U + index;
	return 0;
}

static sint32 get_i2c_peripheral(void *context, uint16 address,
	const AthrillDeviceI2cPeripheralOps **ops, void **instance)
{
	(void)context;
	if (address != ADDRESS) {
		return -1;
	}
	*ops = &chip_ops;
	*instance = &chip;
	return 0;
}

static uint32 reg_cells[2] = { I2C3, 0x400U };
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

static const AthrillDeviceModelOps *model = &athrill_device_stm32_i2c_model;
static void *device;

static uint32 read_reg(uint32 offset)
{
	uint64 value = 0U;
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->read(device, I2C3 + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void write_reg(uint32 offset, uint32 value)
{
	/* AthrillDeviceMmioOps: zero is a successful MMIO access. */
	TEST_CHECK(model->mmio->write(device, I2C3 + offset, 4U, value) == 0);
}

static uint32 cr2_value(std_bool read, uint32 nbytes, std_bool reload)
{
	return (ADDRESS << 1) | (read ? RD_WRN : 0U) | (nbytes << 16) |
		(reload ? RELOAD : 0U);
}

static void reset_controller(void)
{
	memset(&chip, 0, sizeof(chip));
	TEST_CHECK(model->reset(device) == 0); /* model reset ABI: zero is success */
	write_reg(CR1, PE);
}

static void write_bytes(uint32 count, uint32 first)
{
	uint32 i;
	for (i = 0U; i < count; i++) {
		/* RM0410 33.9.7 TXIS: TXDR is ready for the next transmitted byte. */
		CHECK_EQ("TXIS before byte", read_reg(ISR) & TXIS, TXIS);
		write_reg(TXDR, first + i);
	}
}

static void read_bytes(uint32 count)
{
	uint32 i;
	for (i = 0U; i < count; i++) {
		/* RM0410 33.9.7 RXNE: RXDR contains the next received byte. */
		CHECK_EQ("RXNE before byte", read_reg(ISR) & RXNE, RXNE);
		(void)read_reg(RXDR);
	}
}

static void test_write_reload(void)
{
	uint32 reload = cr2_value(FALSE, CHUNK, TRUE);
	std_bool second_started;

	reset_controller();
	write_reg(CR2, reload | START);
	write_bytes(CHUNK, 0U);
	/* RM0410 33.9.7: RELOAD=1 after 255 NBYTES sets TCR, not TC. */
	CHECK_EQ("write first chunk TCR", read_reg(ISR) & (TCR | TC), TCR);
	/* ChibiOS I2Cv3 lines 266-280 and NuttX lines 2033-2051 rewrite the
	 * exact same NBYTES=255|RELOAD value for bytes 256..510. */
	write_reg(CR2, reload);
	/* RM0410 33.9.7: a nonzero NBYTES write clears TCR and resumes with TXIS. */
	CHECK_EQ("write identical reload resumes", read_reg(ISR) & (TCR | TXIS), TXIS);
	second_started = (read_reg(ISR) & TXIS) != 0U;
	if (second_started) {
		write_bytes(CHUNK, CHUNK);
	}
	/* RM0410 Figure 365: the second full chunk ends in TCR again. */
	CHECK_EQ("write second chunk TCR", read_reg(ISR) & TCR, TCR);
	write_reg(CR2, cr2_value(FALSE, LAST, FALSE));
	write_bytes(LAST, 2U * CHUNK);
	/* RM0410 33.9.7 TC: RELOAD=AUTOEND=0 after the last 90 bytes sets TC. */
	CHECK_EQ("write final TC", read_reg(ISR) & (TCR | TC), TC);
	/* The two driver chunk sequences above total 255 + 255 + 90 = 600. */
	CHECK_EQ("bytes delivered to target", chip.writes, TOTAL);
}

static void test_read_reload(void)
{
	uint32 reload = cr2_value(TRUE, CHUNK, TRUE);
	std_bool second_started;

	reset_controller();
	write_reg(CR2, reload | START);
	read_bytes(CHUNK);
	/* RM0410 33.9.7: RELOAD=1 after 255 received bytes sets TCR, not TC. */
	CHECK_EQ("read first chunk TCR", read_reg(ISR) & (TCR | TC), TCR);
	/* ChibiOS I2Cv3 lines 237-251 uses the same 255-byte reload for receive. */
	write_reg(CR2, reload);
	/* RM0410 33.9.7: the nonzero NBYTES write clears TCR and produces RXNE. */
	CHECK_EQ("read identical reload resumes", read_reg(ISR) & (TCR | RXNE), RXNE);
	second_started = (read_reg(ISR) & RXNE) != 0U;
	if (second_started) {
		read_bytes(CHUNK);
	}
	/* RM0410 Figure 366: the second full receive chunk ends in TCR. */
	CHECK_EQ("read second chunk TCR", read_reg(ISR) & TCR, TCR);
	write_reg(CR2, cr2_value(TRUE, LAST, FALSE));
	read_bytes(LAST);
	/* RM0410 33.9.7 TC: the last 90-byte receive chunk completes with TC. */
	CHECK_EQ("read final TC", read_reg(ISR) & (TCR | TC), TC);
	/* The card-side read callback is invoked once per one of the 600 bytes. */
	CHECK_EQ("bytes read from target", chip.reads, TOTAL);
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
		0U, "/soc/i2c@40005c00", "st,stm32-i2c-v2", &node_ops, NULL
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
	resources.get_i2c_peripheral = get_i2c_peripheral;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;

	TEST_CHECK(model->validate(&node, NULL, 0U) == 0); /* model ABI success */
	TEST_CHECK(model->create(&args, &device) == 0); /* model ABI success */
	TEST_CHECK(model->connect(device, &connect) == 0); /* model ABI success */
	test_write_reload();
	test_read_reload();
	model->destroy(device);
	printf("i2c-reload: %d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
