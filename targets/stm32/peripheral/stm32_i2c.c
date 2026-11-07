/*
 * The STM32F7/H7 I2C controllers (see stm32_i2c.h). Offsets and bits from
 * RM0410 33.7 and RM0433 47.4. The register layout is shared; only the
 * board-selected bases differ.
 */
#include "stm32_i2c.h"
#include "athrill_protocol_i2c.h"
#include "athrill_device_host.h"

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

typedef struct {
	uint32 base;
	uint32 irq_event;
	uint32 regs[I2C_REGS / 4U];
	std_bool active;
	std_bool reading;
	uint32 remaining;	/* bytes left of NBYTES */
	const struct I2cChip *chip;
} I2cType;

typedef struct I2cChip {
	uint32 bus;
	uint32 address;
	const Stm32I2cChipOps *ops;
	void *chip;
	AthrillChannel *channel;
	AthrillI2cProtocol *protocol;
} I2cChipType;

static I2cType i2cs[] = {
	{ .base = 0x40005400U, .irq_event = 31U },	/* I2C1 */
	{ .base = 0x40005800U, .irq_event = 33U },	/* I2C2 */
	{ .base = 0x40005C00U, .irq_event = 72U },	/* I2C3 */
	{ .base = 0x40006000U, .irq_event = 95U },	/* I2C4 */
};

#define I2C_NUM	(sizeof(i2cs) / sizeof(i2cs[0]))

static const uint32 f7_bases[I2C_NUM] = {
	0x40005400U, 0x40005800U, 0x40005C00U, 0x40006000U,
};

static const uint32 h7_bases[I2C_NUM] = {
	0x40005400U, 0x40005800U, 0x40005C00U, 0x58001C00U,
};

static I2cChipType chips[CHIPS_MAX];
static uint32 chip_count;

static std_bool channel_start(void *context, std_bool read,
	std_bool repeated, uint64 time)
{
	I2cChipType *chip = context;
	(void)repeated;
	(void)time;
	return chip->ops->start(chip->chip, read);
}

static std_bool channel_write(void *context, uint8 data, uint64 time)
{
	I2cChipType *chip = context;
	(void)time;
	chip->ops->write(chip->chip, data);
	return TRUE;
}

static sint32 channel_read(void *context, uint8 *data,
	std_bool master_ack, uint64 time)
{
	I2cChipType *chip = context;
	(void)master_ack;
	(void)time;
	*data = chip->ops->read(chip->chip);
	return 0;
}

static void channel_stop(void *context, uint64 time)
{
	I2cChipType *chip = context;
	(void)time;
	chip->ops->stop(chip->chip);
}

static const AthrillI2cChildOps channel_child_ops = {
	channel_start, channel_write, channel_read, channel_stop
};

void stm32_i2c_set_h7(std_bool h7)
{
	const uint32 *bases = h7 ? h7_bases : f7_bases;
	uint32 i;

	for (i = 0U; i < I2C_NUM; i++) {
		i2cs[i].base = bases[i];
	}
}

static I2cType *i2c_of(uint32 addr)
{
	uint32 i;

	for (i = 0U; i < I2C_NUM; i++) {
		if ((addr >= i2cs[i].base) && (addr < (i2cs[i].base + I2C_BLOCK))) {
			return &i2cs[i];
		}
	}
	return NULL;
}

std_bool stm32_i2c_owns(uint32 addr)
{
	return i2c_of(addr) != NULL;
}

static uint32 *reg(I2cType *b, uint32 offset)
{
	return &b->regs[offset / 4U];
}

/* The flags whose event interrupts are enabled. */
static uint32 enabled_flags(I2cType *b)
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
static void raise(I2cType *b, uint32 flags)
{
	if ((flags & enabled_flags(b)) != 0U) {
		athrill_device_host_raise(b->irq_event);
	}
}

static void set_flags(I2cType *b, uint32 flags)
{
	*reg(b, I2C_ISR) |= flags;
	raise(b, flags);
}

static void stop(I2cType *b)
{
	if (b->active && (b->chip != NULL)) {
		(void)athrill_i2c_stop(b->chip->protocol, athrill_device_host_clock());
	}
	b->active = FALSE;
	b->chip = NULL;
	*reg(b, I2C_ISR) &= ~(ISR_BUSY | ISR_TC | ISR_TCR | ISR_TXIS | ISR_DIR);
	set_flags(b, ISR_STOPF);
}

/* NBYTES done: TCR (RELOAD), a STOP (AUTOEND) or TC. */
static void end_of_bytes(I2cType *b)
{
	uint32 cr2 = *reg(b, I2C_CR2);

	if ((cr2 & CR2_RELOAD) != 0U) {
		set_flags(b, ISR_TCR);
	}
	else if ((cr2 & CR2_AUTOEND) != 0U) {
		stop(b);
	}
	else {
		set_flags(b, ISR_TC);
	}
}

/* The next byte from the chip into RXDR. */
static void receive(I2cType *b)
{
	uint8 data = 0xffU;
	(void)athrill_i2c_read(b->chip->protocol, &data,
		b->remaining > 1U, athrill_device_host_clock());
	*reg(b, I2C_RXDR) = data;
	b->remaining--;
	*reg(b, I2C_ISR) |= ISR_RXNE;
	if (b->remaining == 0U) {
		end_of_bytes(b);
	}
	raise(b, ISR_RXNE);
}

/* The bytes of NBYTES start (or continue after TCR). */
static void next_bytes(I2cType *b)
{
	if (b->remaining == 0U) {
		end_of_bytes(b);
	}
	else if (b->reading) {
		receive(b);
	}
	else {
		set_flags(b, ISR_TXIS);
	}
}

static void start(I2cType *b)
{
	uint32 cr2 = *reg(b, I2C_CR2);
	uint32 bus = (uint32)(b - i2cs) + 1U;
	uint32 address = (cr2 >> CR2_SADD_SHIFT) & 0x7FU;
	uint32 i;

	*reg(b, I2C_CR2) &= ~CR2_START;
	*reg(b, I2C_ISR) &= ~(ISR_TC | ISR_TCR | ISR_TXIS | ISR_DIR);
	*reg(b, I2C_ISR) |= ISR_BUSY;
	b->reading = ((cr2 & CR2_RD_WRN) != 0U);
	b->remaining = (cr2 >> CR2_NBYTES_SHIFT) & 0xFFU;
	b->chip = NULL;
	for (i = 0U; i < chip_count; i++) {
		if ((chips[i].bus == bus) && (chips[i].address == address)) {
			b->chip = &chips[i];
		}
	}
	if ((b->chip == NULL) ||
	    athrill_i2c_start(b->chip->protocol, (uint16)address,
		b->reading, athrill_device_host_clock()) != 1) {
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

uint32 stm32_i2c_read(uint32 addr)
{
	I2cType *b = i2c_of(addr);
	uint32 offset = (addr - b->base) & ~3U;
	uint32 shift = 8U * ((addr - b->base) & 3U);
	uint32 value;

	switch (offset) {
	case I2C_RXDR:
		value = *reg(b, I2C_RXDR);
		*reg(b, I2C_ISR) &= ~ISR_RXNE;
		if (b->active && b->reading && (b->remaining != 0U)
			&& ((*reg(b, I2C_ISR) & ISR_TCR) == 0U)) {
			receive(b);
		}
		return value >> shift;
	case I2C_ICR:
		return 0U;
	default:
		return (offset < I2C_REGS) ? (*reg(b, offset) >> shift) : 0U;
	}
}

void stm32_i2c_write(uint32 addr, uint32 size, uint32 value)
{
	I2cType *b = i2c_of(addr);
	uint32 offset = (addr - b->base) & ~3U;
	uint32 shift = 8U * ((addr - b->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	uint32 old;

	value = (value << shift) & mask;
	switch (offset) {
	case I2C_TXDR:
		if (b->active && !b->reading && ((*reg(b, I2C_ISR) & ISR_TXIS) != 0U)) {
			std_bool ack = TRUE;
			*reg(b, I2C_ISR) &= ~ISR_TXIS;
			(void)athrill_i2c_write(b->chip->protocol, (uint8)value,
				&ack, athrill_device_host_clock());
			b->remaining--;
			next_bytes(b);
		}
		return;
	case I2C_ICR:
		*reg(b, I2C_ISR) &= ~(value & ICR_MASK);
		return;
	case I2C_ISR:
	case I2C_RXDR:
		return;
	case I2C_CR1:
		*reg(b, I2C_CR1) = (*reg(b, I2C_CR1) & ~mask) | value;
		if ((*reg(b, I2C_CR1) & CR1_PE) == 0U) {
			/* PE = 0: a software reset of the state and the flags */
			b->active = FALSE;
			b->chip = NULL;
			*reg(b, I2C_ISR) = ISR_TXE;
		}
		else {
			raise(b, *reg(b, I2C_ISR));
		}
		return;
	case I2C_CR2:
		old = *reg(b, I2C_CR2);
		*reg(b, I2C_CR2) = (old & ~mask) | value;
		if ((*reg(b, I2C_CR1) & CR1_PE) == 0U) {
			return;
		}
		if ((*reg(b, I2C_CR2) & CR2_STOP) != 0U) {
			*reg(b, I2C_CR2) &= ~CR2_STOP;
			stop(b);
		}
		else if ((*reg(b, I2C_CR2) & CR2_START) != 0U) {
			start(b);
		}
		else if (((*reg(b, I2C_ISR) & ISR_TCR) != 0U)
			&& (((old ^ *reg(b, I2C_CR2)) >> CR2_NBYTES_SHIFT) & 0xFFU) != 0U) {
			/* NBYTES written after TCR: the transfer goes on */
			*reg(b, I2C_ISR) &= ~ISR_TCR;
			b->remaining = (*reg(b, I2C_CR2) >> CR2_NBYTES_SHIFT) & 0xFFU;
			next_bytes(b);
		}
		return;
	default:
		if (offset < I2C_REGS) {
			*reg(b, offset) = (*reg(b, offset) & ~mask) | value;
		}
		return;
	}
}

void stm32_i2c_reset(void)
{
	uint32 i;
	uint32 k;
	for (i = 0U; i < chip_count; i++) {
		athrill_i2c_protocol_destroy(chips[i].protocol);
		athrill_channel_destroy(chips[i].channel);
	}

	for (i = 0U; i < I2C_NUM; i++) {
		for (k = 0U; k < (I2C_REGS / 4U); k++) {
			i2cs[i].regs[k] = 0U;
		}
		i2cs[i].regs[I2C_ISR / 4U] = ISR_TXE;
		i2cs[i].active = FALSE;
		i2cs[i].chip = NULL;
	}
	chip_count = 0U;	/* the board attaches its chips again */
}

void stm32_i2c_attach(uint32 bus, uint32 address, const Stm32I2cChipOps *ops, void *chip)
{
	AthrillChannelConfig config = {0};
	I2cChipType *attached;
	if (chip_count >= CHIPS_MAX || bus == 0U || bus > I2C_NUM ||
	    address > 0x7fU || ops == NULL || ops->start == NULL ||
	    ops->write == NULL || ops->read == NULL || ops->stop == NULL) {
		return;
	}
	attached = &chips[chip_count];
	attached->bus = bus;
	attached->address = address;
	attached->ops = ops;
	attached->chip = chip;
	attached->channel = NULL;
	attached->protocol = NULL;
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_ADDRESS;
	config.selection.value = address;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	if (athrill_channel_create(&config, &attached->channel) != 0 ||
	    athrill_i2c_protocol_create(attached->channel, &channel_child_ops,
		attached, &attached->protocol) != 0) {
		athrill_i2c_protocol_destroy(attached->protocol);
		athrill_channel_destroy(attached->channel);
		attached->protocol = NULL;
		attached->channel = NULL;
		return;
	}
	chip_count++;
}

/*
 * The event interrupt is a level: while an enabled flag stays set (two
 * events at once, TC after the last RXNE), it is made pending again once
 * its handler is no longer active, as the NVIC does at exception return.
 */
void stm32_i2c_tick(void)
{
	uint32 i;

	for (i = 0U; i < I2C_NUM; i++) {
		I2cType *b = &i2cs[i];

		if (((*reg(b, I2C_ISR) & enabled_flags(b)) != 0U) && !athrill_device_host_irq_pending(b->irq_event)) {
			athrill_device_host_raise(b->irq_event);
		}
	}
}
