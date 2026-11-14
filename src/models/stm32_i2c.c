/*
 * STM32F7/H7 I2C controller (RM0410 33.7, RM0433 47.4), master with 7-bit
 * addresses, one model per Devicetree node (st,stm32-i2c-v2), with the chips
 * of its bus as its Devicetree children.
 *
 * Ported unchanged from athrill-target-stm32's stm32_i2c.c (now
 * hakoniwalab/athrill-device#13 step 4-2): the registers are those of the
 * hardware and the transfer follows its flags. START sends the address (ACK
 * from a chip at it, else NACKF and an automatic STOP), TXIS asks for each
 * byte to send, RXNE offers each byte received, and after NBYTES comes TCR
 * (RELOAD), a STOP (AUTOEND) or TC; the event interrupt follows the enabled
 * flags and, being a level, is made pending again by the tick while an
 * enabled flag stays set. Lightweight: no bit timing (each step happens at
 * the register access that allows it), no slave mode, no SMBus, no DMA.
 *
 * A register read ends the CPU batch on every board. The target's H7 path
 * did so, its F7 path did not; the board does not decide that.
 *
 * What changed with the move: the bus and base come from the node, the
 * event interrupt from its interrupts ("event"; "error" is connected and,
 * as before, never raised), and the chips are reached
 * through get_i2c_peripheral instead of the target's bridge. A chip without
 * the byte-level operations is driven through its transfer operation as the
 * target's bridge did.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_protocol_i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I2C_CR1		0x00U
#define I2C_CR2		0x04U
#define I2C_ISR		0x18U
#define I2C_ICR		0x1CU
#define I2C_RXDR	0x24U
#define I2C_TXDR	0x28U
#define I2C_REGS	0x2CU
#define I2C_BLOCK	0x400U

#define CR1_PE		(1U << 0)
#define CR1_TXIE	(1U << 1)
#define CR1_RXIE	(1U << 2)
#define CR1_NACKIE	(1U << 4)
#define CR1_STOPIE	(1U << 5)
#define CR1_TCIE	(1U << 6)

#define CR2_SADD_SHIFT	1U	/* 7-bit address in SADD[7:1] */
#define CR2_RD_WRN	(1U << 10)
#define CR2_START	(1U << 13)
#define CR2_STOP	(1U << 14)
#define CR2_NBYTES_SHIFT 16U
#define CR2_RELOAD	(1U << 24)
#define CR2_AUTOEND	(1U << 25)

#define ISR_TXE		(1U << 0)
#define ISR_TXIS	(1U << 1)
#define ISR_RXNE	(1U << 2)
#define ISR_NACKF	(1U << 4)
#define ISR_STOPF	(1U << 5)
#define ISR_TC		(1U << 6)
#define ISR_TCR		(1U << 7)
#define ISR_BUSY	(1U << 15)
#define ISR_DIR		(1U << 16)
#define ICR_MASK	0x3F38U	/* ADDRCF NACKCF STOPCF BERRCF ARLOCF OVRCF PECCF TIMOUTCF ALERTCF */

#define CHIPS_MAX	8U
#define PENDING_MAX	256U

/* A chip on the bus. */
typedef struct {
	uint32 address;
	const AthrillDeviceI2cPeripheralOps *ops;
	void *instance;
	AthrillChannel *channel;
	AthrillI2cProtocol *protocol;
	/* a chip with only transfer(): the bytes written so far, as the bridge */
	uint8 pending[PENDING_MAX];
	uint32 pending_count;
	std_bool reading;
	std_bool failed;
} I2cChip;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint32 base;
	AthrillDeviceIrqLine irq_event;
	AthrillDeviceIrqLine irq_error;	/* connected, never raised: no bus errors */
	uint32 regs[I2C_REGS / 4U];
	std_bool active;
	std_bool reading;
	uint32 remaining;	/* bytes left of NBYTES */
	I2cChip *chip;
	I2cChip chips[CHIPS_MAX];
	uint32 chip_count;
} Stm32I2c;

static uint64 now(const Stm32I2c *b)
{
	return b->target_ops->time_now(b->target);
}

/* A chip with only transfer(): its written bytes as one message. */
static sint32 chip_flush(I2cChip *chip, std_bool stop)
{
	AthrillDeviceI2cMessage message;
	if (chip->pending_count == 0U) {
		return 0;
	}
	message.data = chip->pending;
	message.size = chip->pending_count;
	message.read = FALSE;
	message.stop = stop;
	chip->pending_count = 0U;
	return chip->ops->transfer(chip->instance, &message, 1U);
}

static std_bool channel_start(void *context, std_bool read,
	std_bool repeated, uint64 time)
{
	I2cChip *chip = context;
	(void)repeated;
	(void)time;
	if (chip->ops->start != NULL) {
		return chip->ops->start(chip->instance, read);
	}
	if (chip_flush(chip, FALSE) != 0) {
		chip->failed = TRUE;
		return FALSE;
	}
	chip->reading = read;
	return TRUE;
}

static std_bool channel_write(void *context, uint8 data, uint64 time)
{
	I2cChip *chip = context;
	(void)time;
	if (chip->ops->write_byte != NULL) {
		chip->ops->write_byte(chip->instance, data);
		return TRUE;
	}
	if (chip->pending_count >= PENDING_MAX) {
		chip->failed = TRUE;
		return TRUE;
	}
	chip->pending[chip->pending_count++] = data;
	return TRUE;
}

static sint32 channel_read(void *context, uint8 *data,
	std_bool master_ack, uint64 time)
{
	I2cChip *chip = context;
	AthrillDeviceI2cMessage message;
	(void)master_ack;
	(void)time;
	if (chip->ops->read_byte != NULL) {
		*data = chip->ops->read_byte(chip->instance);
		return 0;
	}
	*data = 0xffU;
	message.data = data;
	message.size = 1U;
	message.read = TRUE;
	message.stop = FALSE;
	if (chip->ops->transfer(chip->instance, &message, 1U) != 0) {
		chip->failed = TRUE;
	}
	return 0;
}

static void channel_stop(void *context, uint64 time)
{
	I2cChip *chip = context;
	AthrillDeviceI2cMessage message;
	(void)time;
	if (chip->ops->stop != NULL) {
		chip->ops->stop(chip->instance);
		return;
	}
	if (chip->reading) {
		message.data = NULL;
		message.size = 0U;
		message.read = TRUE;
		message.stop = TRUE;
		if (chip->ops->transfer(chip->instance, &message, 1U) != 0) {
			chip->failed = TRUE;
		}
	} else if (chip_flush(chip, TRUE) != 0) {
		chip->failed = TRUE;
	}
	chip->reading = FALSE;
}

static const AthrillI2cChildOps channel_child_ops = {
	channel_start, channel_write, channel_read, channel_stop
};

static uint32 *reg(Stm32I2c *b, uint32 offset)
{
	return &b->regs[offset / 4U];
}

/* The flags whose event interrupts are enabled. */
static uint32 enabled_flags(Stm32I2c *b)
{
	uint32 cr1 = *reg(b, I2C_CR1);
	uint32 enabled = 0U;

	enabled |= ((cr1 & CR1_TXIE) != 0U) ? ISR_TXIS : 0U;
	enabled |= ((cr1 & CR1_RXIE) != 0U) ? ISR_RXNE : 0U;
	enabled |= ((cr1 & CR1_NACKIE) != 0U) ? ISR_NACKF : 0U;
	enabled |= ((cr1 & CR1_STOPIE) != 0U) ? ISR_STOPF : 0U;
	enabled |= ((cr1 & CR1_TCIE) != 0U) ? (ISR_TC | ISR_TCR) : 0U;
	return enabled;
}

/* The event interrupt for the flags just set, if their interrupts are enabled. */
static void raise(Stm32I2c *b, uint32 flags)
{
	if ((flags & enabled_flags(b)) != 0U) {
		(void)b->target_ops->irq_set(b->target, b->irq_event, TRUE);
	}
}

static void set_flags(Stm32I2c *b, uint32 flags)
{
	*reg(b, I2C_ISR) |= flags;
	raise(b, flags);
}

static void stop(Stm32I2c *b)
{
	if (b->active && (b->chip != NULL)) {
		(void)athrill_i2c_stop(b->chip->protocol, now(b));
	}
	b->active = FALSE;
	b->chip = NULL;
	*reg(b, I2C_ISR) &= ~(ISR_BUSY | ISR_TC | ISR_TCR | ISR_TXIS | ISR_DIR);
	set_flags(b, ISR_STOPF);
}

/* NBYTES done: TCR (RELOAD), a STOP (AUTOEND) or TC. */
static void end_of_bytes(Stm32I2c *b)
{
	uint32 cr2 = *reg(b, I2C_CR2);

	if ((cr2 & CR2_RELOAD) != 0U) {
		set_flags(b, ISR_TCR);
	} else if ((cr2 & CR2_AUTOEND) != 0U) {
		stop(b);
	} else {
		set_flags(b, ISR_TC);
	}
}

/* The next byte from the chip into RXDR. */
static void receive(Stm32I2c *b)
{
	uint8 data = 0xffU;
	(void)athrill_i2c_read(b->chip->protocol, &data, b->remaining > 1U, now(b));
	*reg(b, I2C_RXDR) = data;
	b->remaining--;
	*reg(b, I2C_ISR) |= ISR_RXNE;
	if (b->remaining == 0U) {
		end_of_bytes(b);
	}
	raise(b, ISR_RXNE);
}

/* The bytes of NBYTES start (or continue after TCR). */
static void next_bytes(Stm32I2c *b)
{
	if (b->remaining == 0U) {
		end_of_bytes(b);
	} else if (b->reading) {
		receive(b);
	} else {
		set_flags(b, ISR_TXIS);
	}
}

static void start(Stm32I2c *b)
{
	uint32 cr2 = *reg(b, I2C_CR2);
	uint32 address = (cr2 >> CR2_SADD_SHIFT) & 0x7FU;
	uint32 i;

	*reg(b, I2C_CR2) &= ~CR2_START;
	*reg(b, I2C_ISR) &= ~(ISR_TC | ISR_TCR | ISR_TXIS | ISR_DIR);
	*reg(b, I2C_ISR) |= ISR_BUSY;
	b->reading = ((cr2 & CR2_RD_WRN) != 0U);
	b->remaining = (cr2 >> CR2_NBYTES_SHIFT) & 0xFFU;
	b->chip = NULL;
	for (i = 0U; i < b->chip_count; i++) {
		if (b->chips[i].address == address) {
			b->chip = &b->chips[i];
		}
	}
	if ((b->chip == NULL) ||
	    athrill_i2c_start(b->chip->protocol, (uint16)address,
		b->reading, now(b)) != 1) {
		/* no acknowledge: NACKF, then a STOP of its own */
		b->active = FALSE;
		set_flags(b, ISR_NACKF);
		stop(b);
		return;
	}
	b->active = TRUE;
	if (b->reading) {
		*reg(b, I2C_ISR) |= ISR_DIR;
	}
	next_bytes(b);
}

static sint32 i2c_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	Stm32I2c *b = instance;
	uint32 offset, shift, value;
	if (address < b->base || address >= (uint64)b->base + I2C_BLOCK ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	/* RXDR consumes the received stream and ISR follows the transfer: a
	 * register read ends the CPU batch so that the next instruction sees the
	 * effect, on every board (the register's meaning decides, not the SoC). */
	b->target_ops->end_batch(b->target);
	offset = ((uint32)address - b->base) & ~3U;
	shift = 8U * (((uint32)address - b->base) & 3U);
	switch (offset) {
	case I2C_RXDR:
		value = *reg(b, I2C_RXDR);
		*reg(b, I2C_ISR) &= ~ISR_RXNE;
		if (b->active && b->reading && (b->remaining != 0U) &&
		    ((*reg(b, I2C_ISR) & ISR_TCR) == 0U)) {
			receive(b);
		}
		*result = value >> shift;
		return 0;
	case I2C_ICR:
		*result = 0U;
		return 0;
	default:
		*result = (offset < I2C_REGS) ? (*reg(b, offset) >> shift) : 0U;
		return 0;
	}
}

static sint32 i2c_write(void *instance, uint64 address, uint32 size, uint64 input)
{
	Stm32I2c *b = instance;
	uint32 offset, shift, mask, value, old;
	if (address < b->base || address >= (uint64)b->base + I2C_BLOCK ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = ((uint32)address - b->base) & ~3U;
	shift = 8U * (((uint32)address - b->base) & 3U);
	mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	switch (offset) {
	case I2C_TXDR:
		if (b->active && !b->reading && ((*reg(b, I2C_ISR) & ISR_TXIS) != 0U)) {
			std_bool ack = TRUE;
			*reg(b, I2C_ISR) &= ~ISR_TXIS;
			(void)athrill_i2c_write(b->chip->protocol, (uint8)value, &ack, now(b));
			b->remaining--;
			next_bytes(b);
		}
		return 0;
	case I2C_ICR:
		*reg(b, I2C_ISR) &= ~(value & ICR_MASK);
		return 0;
	case I2C_ISR:
	case I2C_RXDR:
		return 0;
	case I2C_CR1:
		*reg(b, I2C_CR1) = (*reg(b, I2C_CR1) & ~mask) | value;
		if ((*reg(b, I2C_CR1) & CR1_PE) == 0U) {
			/* PE = 0: a software reset of the state and the flags */
			b->active = FALSE;
			b->chip = NULL;
			*reg(b, I2C_ISR) = ISR_TXE;
		} else {
			raise(b, *reg(b, I2C_ISR));
		}
		return 0;
	case I2C_CR2:
		old = *reg(b, I2C_CR2);
		*reg(b, I2C_CR2) = (old & ~mask) | value;
		if ((*reg(b, I2C_CR1) & CR1_PE) == 0U) {
			return 0;
		}
		if ((*reg(b, I2C_CR2) & CR2_STOP) != 0U) {
			*reg(b, I2C_CR2) &= ~CR2_STOP;
			stop(b);
		} else if ((*reg(b, I2C_CR2) & CR2_START) != 0U) {
			start(b);
		} else if (((*reg(b, I2C_ISR) & ISR_TCR) != 0U) &&
			   (((mask >> CR2_NBYTES_SHIFT) & 0xFFU) != 0U) &&
			   (((*reg(b, I2C_CR2) >> CR2_NBYTES_SHIFT) & 0xFFU) != 0U)) {
			/* NBYTES written (non-zero) after TCR: the transfer goes on
			 * (RM0410 33.7.7: TCR is cleared by writing a non-zero NBYTES;
			 * NuttX and ChibiOS write 255|RELOAD again, the same value) */
			*reg(b, I2C_ISR) &= ~ISR_TCR;
			b->remaining = (*reg(b, I2C_CR2) >> CR2_NBYTES_SHIFT) & 0xFFU;
			next_bytes(b);
		}
		return 0;
	default:
		if (offset < I2C_REGS) {
			*reg(b, offset) = (*reg(b, offset) & ~mask) | value;
		}
		return 0;
	}
}

static const AthrillDeviceMmioOps i2c_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), i2c_read, i2c_write
};

static sint32 i2c_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	uint32 size;
	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != I2C_BLOCK) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size,
				"I2C register window must be 0x%x bytes", I2C_BLOCK);
		}
		return -1;
	}
	return 0;
}

static sint32 i2c_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Stm32I2c *b = calloc(1U, sizeof(*b));
	if (b == NULL || args->target_ops == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &b->base) != 0) {
		free(b);
		return -1;
	}
	b->target_ops = args->target_ops;
	b->target = args->target;
	*instance = b;
	return 0;
}

static void destroy_chips(Stm32I2c *b)
{
	uint32 i;
	for (i = 0U; i < b->chip_count; i++) {
		athrill_i2c_protocol_destroy(b->chips[i].protocol);
		athrill_channel_destroy(b->chips[i].channel);
	}
	b->chip_count = 0U;
}

static void i2c_destroy(void *instance)
{
	Stm32I2c *b = instance;
	destroy_chips(b);
	free(b);
}

/* The chip at an address, on a channel and protocol as the target attached it. */
static sint32 attach(Stm32I2c *b, uint32 address,
	const AthrillDeviceI2cPeripheralOps *ops, void *chip_instance)
{
	AthrillChannelConfig config = {0};
	I2cChip *chip;
	if (b->chip_count >= CHIPS_MAX || ops == NULL ||
	    (ops->start == NULL && ops->transfer == NULL)) {
		return -1;
	}
	chip = &b->chips[b->chip_count];
	memset(chip, 0, sizeof(*chip));
	chip->address = address;
	chip->ops = ops;
	chip->instance = chip_instance;
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_ADDRESS;
	config.selection.value = address;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	if (athrill_channel_create(&config, &chip->channel) != 0 ||
	    athrill_i2c_protocol_create(chip->channel, &channel_child_ops,
		chip, &chip->protocol) != 0) {
		athrill_i2c_protocol_destroy(chip->protocol);
		athrill_channel_destroy(chip->channel);
		return -1;
	}
	b->chip_count++;
	return 0;
}

static sint32 i2c_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Stm32I2c *b = instance;
	uint32 address;
	if (b->target_ops->time_now == NULL || b->target_ops->irq_set == NULL ||
	    b->target_ops->irq_is_active == NULL || b->target_ops->end_batch == NULL ||
	    args->resources->get_interrupt(args->context, "event", 0U, &b->irq_event) != 0 ||
	    args->resources->get_interrupt(args->context, "error", 1U, &b->irq_error) != 0) {
		return -1;
	}
	for (address = 0U; address <= 0x7FU; address++) {
		const AthrillDeviceI2cPeripheralOps *ops;
		void *chip;
		if (args->resources->get_i2c_peripheral(args->context, (uint16)address,
			&ops, &chip) == 0 && attach(b, address, ops, chip) != 0) {
			return -1;
		}
	}
	return 0;
}

static sint32 i2c_reset(void *instance)
{
	Stm32I2c *b = instance;
	memset(b->regs, 0, sizeof(b->regs));
	b->regs[I2C_ISR / 4U] = ISR_TXE;
	b->active = FALSE;
	b->chip = NULL;
	return 0;
}

/*
 * The event interrupt is a level: while an enabled flag stays set (two
 * events at once, TC after the last RXNE), it is made pending again once
 * its handler is no longer active, as the NVIC does at exception return.
 */
static sint32 i2c_tick(void *instance, uint64 time, AthrillDeviceTickResult *result)
{
	Stm32I2c *b = instance;
	(void)time;
	(void)result;
	if (((*reg(b, I2C_ISR) & enabled_flags(b)) != 0U) &&
	    !b->target_ops->irq_is_active(b->target, b->irq_event)) {
		(void)b->target_ops->irq_set(b->target, b->irq_event, TRUE);
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_i2c_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-i2c", "1", i2c_validate,
	i2c_create, i2c_destroy, &i2c_mmio, NULL, NULL, NULL, i2c_connect,
	i2c_reset, i2c_tick
};
