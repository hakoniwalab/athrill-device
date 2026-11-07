/*
 * The STM32F7 SPI controllers (see stm32_spi.h). Register offsets and bits
 * from RM0410 35.9.
 */
#include "stm32_spi.h"
#include "stm32_spi_bindings.h"
#include "stm32_spi_config.h"
#include "athrill_protocol_spi.h"
#include "athrill_device_host.h"

#define SPI_BLOCK	0x400U
#define SPI_WORDS	(SPI_BLOCK / 4U)

/* Register interpretation: offsets and bits come from the bindings
 * (spec/bindings/st-stm32-spi-v1__f7_gen.yaml), resolved once at reset. */
static uint32 CR1_OFFSET, CR2_OFFSET, SR_OFFSET, DR_OFFSET, CRCPR_OFFSET;
static uint32 spi_regs;				/* register window (bytes) */
static uint32 reset_values[SPI_WORDS];	/* by word */
static uint32 CR1_SPE, CR2_DS_SHIFT, CR2_FRXTH, SR_RXNE, SR_TXE, SR_FRLVL_SHIFT;

static void bind_registers(void)
{
	static const Stm32SpiBinding table[] = {
		BIND_OFFSET(CR1),
		BIND_OFFSET(CR2),
		BIND_OFFSET(SR),
		BIND_OFFSET(DR),
		BIND_OFFSET(CRCPR),
		BIND_MASK(CR1, SPE),
		BIND_SHIFT(CR2, DS),
		BIND_MASK(CR2, FRXTH),
		BIND_MASK(SR, RXNE),
		BIND_MASK(SR, TXE),
		BIND_SHIFT(SR, FRLVL),
	};
	stm32_spi_bind("st,stm32-spi", "st,stm32f7", table, sizeof(table) / sizeof(table[0]));
	spi_regs = stm32_spi_bind_resets("st,stm32-spi", "st,stm32f7", reset_values, SPI_WORDS);
}

#define RX_FIFO_SIZE	STM32F7_SPI_RX_FIFO_BYTES
#define CHIPS_MAX	16U

typedef struct {
	uint32 base;
	uint32 regs[SPI_WORDS];
	uint8 rx[RX_FIFO_SIZE];
	uint32 rx_head;
	uint32 rx_count;
} SpiType;

typedef struct {
	uint32 bus;
	uint32 port;
	uint32 pin;
	const Stm32SpiChipOps *ops;
	void *chip;
	AthrillChannel *channel;
	AthrillSpiProtocol *protocol;
	uint32 selection;
	std_bool selected;
} SpiChipType;

static SpiType spis[] = {
	{ .base = 0x40013000U },	/* SPI1 */
	{ .base = 0x40003800U },	/* SPI2 */
	{ .base = 0x40003C00U },	/* SPI3 */
	{ .base = 0x40013400U },	/* SPI4 */
	{ .base = 0x40015000U },	/* SPI5 */
	{ .base = 0x40015400U },	/* SPI6 */
};

#define SPI_NUM	(sizeof(spis) / sizeof(spis[0]))

static SpiChipType chips[CHIPS_MAX];
static uint32 chip_count;

static SpiType *spi_of(uint32 addr)
{
	uint32 i;

	for (i = 0U; i < SPI_NUM; i++) {
		if ((addr >= spis[i].base) && (addr < (spis[i].base + SPI_BLOCK))) {
			return &spis[i];
		}
	}
	return NULL;
}

std_bool stm32_spi_owns(uint32 addr)
{
	return spi_of(addr) != NULL;
}

static uint32 reg(const SpiType *s, uint32 offset)
{
	return s->regs[offset / 4U];
}

/* Bits per frame (DS + 1). */
static uint32 frame_bits(const SpiType *s)
{
	uint32 ds = (reg(s, CR2_OFFSET) >> CR2_DS_SHIFT) & 0xFU;

	return (ds < 3U) ? 8U : (ds + 1U);
}

static void channel_select(void *context, uint64 time)
{
	SpiChipType *chip = context;
	(void)time;
	chip->ops->select(chip->chip, TRUE);
}

static sint32 channel_exchange(void *context, uint8 transmitted,
	uint8 *received, uint64 time)
{
	SpiChipType *chip = context;
	(void)time;
	*received = chip->ops->exchange(chip->chip, transmitted);
	return 0;
}

static void channel_deselect(void *context, uint64 time)
{
	SpiChipType *chip = context;
	(void)time;
	chip->ops->select(chip->chip, FALSE);
}

static const AthrillSpiChildOps channel_child_ops = {
	channel_select, channel_exchange, channel_deselect
};

/* The selected child channel answers MOSI (0xFF: no child drives MISO). */
uint8 stm32_spi_chip_exchange(uint32 bus, uint8 mosi)
{
	uint32 i;
	uint8 miso;

	for (i = 0U; i < chip_count; i++) {
		if ((chips[i].bus == bus) && chips[i].selected) {
			if (athrill_spi_exchange(chips[i].protocol, mosi, &miso,
				athrill_device_host()->time_now(NULL)) == 0) {
				return miso;
			}
			return 0xFFU;
		}
	}
	return 0xFFU;
}

static void rx_push(SpiType *s, uint8 data)
{
	if (s->rx_count < RX_FIFO_SIZE) {
		s->rx[(s->rx_head + s->rx_count) % RX_FIFO_SIZE] = data;
		s->rx_count++;
	}
}

static uint8 rx_pop(SpiType *s)
{
	uint8 data = 0U;

	if (s->rx_count != 0U) {
		data = s->rx[s->rx_head];
		s->rx_head = (s->rx_head + 1U) % RX_FIFO_SIZE;
		s->rx_count--;
	}
	return data;
}

/* A write to DR: frames of up to 8 bits are packed two per halfword access; a wider frame goes MSB first. */
static void transmit(SpiType *s, uint32 size, uint32 value)
{
	uint32 bus = (uint32)(s - spis) + 1U;
	uint8 hi;
	uint8 lo;

	if ((reg(s, CR1_OFFSET) & CR1_SPE) == 0U) {
		return;
	}
	if (frame_bits(s) > 8U) {
		hi = stm32_spi_chip_exchange(bus, (uint8)(value >> 8));
		lo = stm32_spi_chip_exchange(bus, (uint8)value);
		rx_push(s, lo);
		rx_push(s, hi);
		return;
	}
	rx_push(s, stm32_spi_chip_exchange(bus, (uint8)value));
	if (size >= 2U) {
		rx_push(s, stm32_spi_chip_exchange(bus, (uint8)(value >> 8)));
	}
}

static uint32 status(const SpiType *s)
{
	uint32 sr = SR_TXE;
	uint32 needed = ((frame_bits(s) <= 8U) && ((reg(s, CR2_OFFSET) & CR2_FRXTH) != 0U)) ? 1U : 2U;
	uint32 level = (s->rx_count >= 3U) ? 3U : s->rx_count;

	if (s->rx_count >= needed) {
		sr |= SR_RXNE;
	}
	return sr | (level << SR_FRLVL_SHIFT);
}

uint32 stm32_spi_read(uint32 addr, uint32 size)
{
	SpiType *s = spi_of(addr);
	uint32 offset = (addr - s->base) & ~3U;
	uint32 value;

	if (offset == SR_OFFSET) {
		return status(s);
	}
	if (offset == DR_OFFSET) {
		value = rx_pop(s);
		if ((size >= 2U) || (frame_bits(s) > 8U)) {
			value |= (uint32)rx_pop(s) << 8;
		}
		return value;
	}
	return (offset < spi_regs) ? (reg(s, offset) >> (8U * ((addr - s->base) & 3U))) : 0U;
}

void stm32_spi_write(uint32 addr, uint32 size, uint32 value)
{
	SpiType *s = spi_of(addr);
	uint32 offset = (addr - s->base) & ~3U;
	uint32 shift = 8U * ((addr - s->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	if (offset == DR_OFFSET) {
		transmit(s, size, value);
		return;
	}
	if (offset == SR_OFFSET) {
		return;
	}
	if (offset < spi_regs) {
		s->regs[offset / 4U] = (s->regs[offset / 4U] & ~mask) | ((value << shift) & mask);
	}
	/* SPE cleared: the FIFOs empty */
	if ((offset == CR1_OFFSET) && ((reg(s, CR1_OFFSET) & CR1_SPE) == 0U)) {
		s->rx_count = 0U;
	}
}

void stm32_spi_reset(void)
{
	uint32 i;
	uint32 k;

	bind_registers();
	for (i = 0U; i < chip_count; i++) {
		athrill_spi_protocol_destroy(chips[i].protocol);
		athrill_channel_destroy(chips[i].channel);
	}

	for (i = 0U; i < SPI_NUM; i++) {
		for (k = 0U; k < SPI_WORDS; k++) {
			spis[i].regs[k] = reset_values[k];
		}
		spis[i].rx_head = 0U;
		spis[i].rx_count = 0U;
	}
	chip_count = 0U;	/* the board attaches its chips again */
}

void stm32_spi_attach(uint32 bus, uint32 port, uint32 pin, const Stm32SpiChipOps *ops, void *chip)
{
	AthrillChannelConfig config = {0};
	SpiChipType *attached;
	if (chip_count >= CHIPS_MAX) {
		return;
	}
	if (bus == 0U || bus > SPI_NUM || pin >= 16U || ops == NULL ||
	    ops->select == NULL || ops->exchange == NULL) {
		return;
	}
	attached = &chips[chip_count];
	attached->bus = bus;
	attached->port = port;
	attached->pin = pin;
	attached->ops = ops;
	attached->chip = chip;
	attached->selection = chip_count;
	attached->selected = FALSE;
	attached->channel = NULL;
	attached->protocol = NULL;
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_CHIP;
	config.selection.value = attached->selection;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	if (athrill_channel_create(&config, &attached->channel) != 0 ||
	    athrill_spi_protocol_create(attached->channel, &channel_child_ops,
		attached, &attached->protocol) != 0) {
		athrill_spi_protocol_destroy(attached->protocol);
		athrill_channel_destroy(attached->channel);
		attached->protocol = NULL;
		attached->channel = NULL;
		return;
	}
	chip_count++;
}

void stm32_spi_pins_changed(uint32 port, uint32 old, uint32 now)
{
	uint32 i;

	for (i = 0U; i < chip_count; i++) {
		SpiChipType *c = &chips[i];
		uint32 bit = 1U << c->pin;
		std_bool selected;

		if ((c->port != port) || (((old ^ now) & bit) == 0U)) {
			continue;
		}
		selected = ((now & bit) == 0U);
		if (selected != c->selected) {
			if (selected) {
				if (athrill_spi_select(c->protocol, c->selection,
					athrill_device_host()->time_now(NULL)) == 1) {
					c->selected = TRUE;
				}
			} else {
				(void)athrill_spi_deselect(c->protocol,
					athrill_device_host()->time_now(NULL));
				c->selected = FALSE;
			}
		}
	}
}
