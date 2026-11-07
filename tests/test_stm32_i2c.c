/*
 * The I2C controller model (src/models/stm32_i2c.c), driven as NuttX's
 * interrupt-driven STM32 I2C driver does: a register address written, then a
 * repeated START to read. Ported from athrill-target-stm32's
 * test_i2c_ist8310.c (test_flags) with a small chip in place of the IST8310
 * (WHO_AM_I 0x10 at register 0), plus a chip that has only transfer().
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <string.h>

#define I2C3	0x40005C00U
#define CR1	0x00U
#define CR2	0x04U
#define ISR	0x18U
#define ICR	0x1CU
#define RXDR	0x24U
#define TXDR	0x28U
#define IRQ_EV	72U
#define IRQ_ER	73U

#define TXIS	(1U << 1)
#define RXNE	(1U << 2)
#define NACKF	(1U << 4)
#define STOPF	(1U << 5)
#define TC	(1U << 6)
#define BUSY	(1U << 15)

static uint32 raised[128];
static std_bool active[128];
static uint64 clock_now;
static uint32 batches;

static uint64 time_now(void *t) { (void)t; return clock_now++; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool on)
{
	(void)t;
	if (on) {
		raised[line.value]++;
	}
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine line) { (void)t; return active[line.value]; }
static void end_batch(void *t) { (void)t; batches++; }

/* A chip with byte operations: a register pointer, WHO_AM_I 0x10 at 0. */
typedef struct {
	uint8 regs[16];
	uint8 pointer;
	std_bool have_pointer;
} ByteChip;

static std_bool byte_start(void *i, std_bool read) { ByteChip *c = i; if (!read) c->have_pointer = FALSE; return TRUE; }
static void byte_write(void *i, uint8 d)
{
	ByteChip *c = i;
	if (!c->have_pointer) {
		c->pointer = d & 0x0FU;
		c->have_pointer = TRUE;
	} else {
		c->regs[c->pointer++ & 0x0FU] = d;
	}
}
static uint8 byte_read(void *i) { ByteChip *c = i; return c->regs[c->pointer++ & 0x0FU]; }
static void byte_stop(void *i) { (void)i; }
static const AthrillDeviceI2cPeripheralOps byte_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceI2cPeripheralOps),
	NULL, byte_start, byte_write, byte_read, byte_stop
};

/* A chip with only transfer(): records the messages. */
typedef struct {
	uint8 written[8];
	uint32 written_count;
	uint32 messages;
	std_bool last_stop;
} MessageChip;

static sint32 message_transfer(void *i, AthrillDeviceI2cMessage *m, uint32 n)
{
	MessageChip *c = i;
	uint32 k, b;
	for (k = 0U; k < n; k++) {
		c->messages++;
		c->last_stop = m[k].stop;
		if (m[k].read) {
			for (b = 0U; b < m[k].size; b++) {
				m[k].data[b] = (uint8)(0xA0U + b);
			}
		} else {
			for (b = 0U; b < m[k].size && c->written_count < sizeof(c->written); b++) {
				c->written[c->written_count++] = m[k].data[b];
			}
		}
	}
	return 0;
}
static const AthrillDeviceI2cPeripheralOps message_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceI2cPeripheralOps),
	message_transfer, NULL, NULL, NULL, NULL
};

static ByteChip byte_chip;
static MessageChip message_chip;

static sint32 get_interrupt(void *context, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)index;
	line->value = (strcmp(name, "event") == 0) ? IRQ_EV : IRQ_ER;
	return 0;
}

static sint32 get_i2c_peripheral(void *context, uint16 address,
	const AthrillDeviceI2cPeripheralOps **ops, void **instance)
{
	(void)context;
	if (address == 0x0EU) {
		*ops = &byte_ops;
		*instance = &byte_chip;
		return 0;
	}
	if (address == 0x50U) {
		*ops = &message_ops;
		*instance = &message_chip;
		return 0;
	}
	return -1;
}

static uint32 reg_cells[2] = { I2C3, 0x400U };
static sint32 get_u32(void *context, const char *name, uint32 index, uint32 *value)
{
	(void)context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = reg_cells[index];
	return 0;
}

static const AthrillDeviceModelOps *m = &athrill_device_stm32_i2c_model;
static void *dev;

static uint32 rd(uint32 offset)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, I2C3 + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void wr(uint32 offset, uint32 size, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, I2C3 + offset, size, value) == 0);
}

static void cr2(uint32 address, std_bool read, uint32 n)
{
	wr(CR2, 4U, (address << 1) | (read ? (1U << 10) : 0U) | (n << 16) | (1U << 13));
}

static void tick(void)
{
	AthrillDeviceTickResult result = { ATHRILL_DEVICE_NO_EVENT, TRUE };
	TEST_CHECK(m->tick(dev, clock_now, &result) == 0);
}

static void setup(void)
{
	memset(raised, 0, sizeof(raised));
	memset(active, 0, sizeof(active));
	memset(&byte_chip, 0, sizeof(byte_chip));
	byte_chip.regs[0] = 0x10U;
	memset(&message_chip, 0, sizeof(message_chip));
	TEST_CHECK(m->reset(dev) == 0);
	wr(CR1, 4U, 0xD7U);	/* PE, TXIE, RXIE, NACKIE, STOPIE, TCIE, ERRIE */
}

static void test_flags(void)
{
	setup();
	cr2(0x0EU, FALSE, 1U);
	TEST_CHECK((rd(ISR) & (TXIS | BUSY)) == (TXIS | BUSY));	/* address ACK */
	TEST_CHECK(raised[IRQ_EV] == 1U);			/* TXIS interrupts */
	wr(TXDR, 1U, 0x00U);
	TEST_CHECK((rd(ISR) & (TC | TXIS)) == TC);		/* NBYTES sent, no AUTOEND */
	cr2(0x0EU, TRUE, 1U);
	TEST_CHECK((rd(ISR) & (RXNE | TC)) == (RXNE | TC));	/* repeated START read */
	TEST_CHECK(rd(RXDR) == 0x10U);				/* WHO_AM_I */
	/* TC stays: the level interrupt is pending again once the handler ends */
	raised[IRQ_EV] = 0U;
	active[IRQ_EV] = TRUE;
	tick();
	TEST_CHECK(raised[IRQ_EV] == 0U);
	active[IRQ_EV] = FALSE;
	tick();
	TEST_CHECK(raised[IRQ_EV] == 1U);
	wr(CR2, 4U, rd(CR2) | (1U << 14));
	TEST_CHECK((rd(ISR) & (STOPF | BUSY | TC)) == STOPF);	/* STOP */
	wr(ICR, 4U, 1U << 5);
	TEST_CHECK((rd(ISR) & STOPF) == 0U);			/* ICR STOPCF */
	/* no chip at the address: NACKF and an automatic STOP */
	cr2(0x1EU, FALSE, 1U);
	TEST_CHECK((rd(ISR) & (NACKF | STOPF | BUSY | TXIS)) == (NACKF | STOPF));
	/* AUTOEND: a STOP after NBYTES */
	wr(ICR, 4U, 0x3F38U);
	wr(CR2, 4U, (0x0EU << 1) | (1U << 10) | (2U << 16) | (1U << 25) | (1U << 13));
	(void)rd(RXDR);
	(void)rd(RXDR);
	TEST_CHECK((rd(ISR) & (STOPF | BUSY)) == STOPF);
	/* PE = 0 resets the flags */
	wr(CR1, 4U, 0U);
	TEST_CHECK(rd(ISR) == 1U);
	/* every register read ends the CPU batch, on every board */
	batches = 0U;
	(void)rd(ISR);
	(void)rd(RXDR);
	TEST_CHECK(batches == 2U);
}

/* A chip without byte operations: written bytes as one message, reads one at a time. */
static void test_transfer_only_chip(void)
{
	setup();
	cr2(0x50U, FALSE, 2U);
	wr(TXDR, 1U, 0x12U);
	wr(TXDR, 1U, 0x34U);
	TEST_CHECK((rd(ISR) & TC) == TC);
	TEST_CHECK(message_chip.messages == 0U);	/* held until START or STOP */
	cr2(0x50U, TRUE, 2U);
	TEST_CHECK(message_chip.written_count == 2U && message_chip.written[0] == 0x12U &&
		message_chip.written[1] == 0x34U);
	TEST_CHECK(rd(RXDR) == 0xA0U);
	TEST_CHECK(rd(RXDR) == 0xA0U);
	wr(CR2, 4U, rd(CR2) | (1U << 14));
	TEST_CHECK(message_chip.last_stop);
}

int main(void)
{
	AthrillDeviceTargetOps target_ops;
	AthrillDeviceNodeOps node_ops = {
		ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
		NULL, get_u32, NULL, NULL
	};
	AthrillDeviceResourceOps resources;
	AthrillDeviceNode node = { 0U, "/soc/i2c@40005c00", "st,stm32-i2c-v2", &node_ops, NULL };
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

	TEST_CHECK(m->validate(&node, NULL, 0U) == 0);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	test_flags();
	test_transfer_only_chip();
	m->destroy(dev);
	printf("stm32-i2c: ok\n");
	return 0;
}
