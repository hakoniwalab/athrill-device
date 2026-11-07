/*
 * STM32H743 SPI-v2 controllers (RM0433 chapter 52).  This is only the H7
 * register/FIFO front end; the bus, chip-select and chip implementations are
 * shared with the F7 front end in stm32_spi.c.
 */
#include "stm32h7_spi.h"
#include "stm32_spi.h"
#include "stm32_spi_bindings.h"
#include "stm32_spi_config.h"
#include "athrill_device_host.h"

#include <stdio.h>
#include <stdlib.h>

#define SPI_BLOCK	0x400U
#define SPI_WORDS	(SPI_BLOCK / 4U)

/* Register interpretation: offsets and bits come from the bindings
 * (spec/bindings/st-stm32-spi-v2__h7_gen.yaml), resolved once at reset. */
static uint32 CR1_OFFSET, CFG1_OFFSET, IER_OFFSET, SR_OFFSET, IFCR_OFFSET, TXDR_OFFSET, RXDR_OFFSET;
static uint32 CR1_SPE, CR1_CSTART, CR1_CSUSP, SR_RXP, SR_TXP, SR_EOT, SR_SUSP, SR_TXC, IER_EOTIE;
static uint32 spi_regs;				/* register window (bytes) */
static uint32 reset_values[SPI_WORDS];	/* by word */

static void bind_registers(void)
{
	static const Stm32SpiBinding table[] = {
		BIND_OFFSET(CR1),
		BIND_OFFSET(CFG1),
		BIND_OFFSET(IER),
		BIND_OFFSET(SR),
		BIND_OFFSET(IFCR),
		BIND_OFFSET(TXDR),
		BIND_OFFSET(RXDR),
		BIND_MASK(CR1, SPE),
		BIND_MASK(CR1, CSTART),
		BIND_MASK(CR1, CSUSP),
		BIND_MASK(SR, RXP),
		BIND_MASK(SR, TXP),
		BIND_MASK(SR, EOT),
		BIND_MASK(SR, SUSP),
		BIND_MASK(SR, TXC),
		BIND_MASK(IER, EOTIE),
	};
	stm32_spi_bind("st,stm32h7-spi", "st,stm32h7", table, sizeof(table) / sizeof(table[0]));
	spi_regs = stm32_spi_bind_resets("st,stm32h7-spi", "st,stm32h7", reset_values, SPI_WORDS);
}

typedef struct {
	uint32 base;
	uint32 bus;
	uint32 irq;
	uint32 regs[SPI_WORDS];
	uint8 rx[STM32H7_SPI_RX_FIFO_MAX];
	uint32 fifo_bytes;	/* RX FIFO depth of this SPI (stm32_spi_config.h) */
	uint32 head;
	uint32 count;
} H7SpiType;

static H7SpiType spis[] = {
	{ .base = 0x40013000U, .bus = 1U, .irq = 35U },
	{ .base = 0x40003800U, .bus = 2U, .irq = 36U },
	{ .base = 0x40003C00U, .bus = 3U, .irq = 51U },
	{ .base = 0x40013400U, .bus = 4U, .irq = 84U },
	{ .base = 0x40015000U, .bus = 5U, .irq = 85U },
	{ .base = 0x58001400U, .bus = 6U, .irq = 86U },
};

#define SPI_NUM (sizeof(spis) / sizeof(spis[0]))

static H7SpiType *spi_of(uint32 addr)
{
	uint32 i;

	for (i = 0U; i < SPI_NUM; i++) {
		if ((addr >= spis[i].base) && (addr < (spis[i].base + SPI_BLOCK))) {
			return &spis[i];
		}
	}
	return NULL;
}

std_bool stm32h7_spi_owns(uint32 addr)
{
	return spi_of(addr) != NULL;
}

static void push(H7SpiType *s, uint8 value)
{
	if (s->count < s->fifo_bytes) {
		s->rx[(s->head + s->count) % s->fifo_bytes] = value;
		s->count++;
	}
}

static uint8 pop(H7SpiType *s)
{
	uint8 value = 0xFFU;

	if (s->count != 0U) {
		value = s->rx[s->head];
		s->head = (s->head + 1U) % s->fifo_bytes;
		s->count--;
	}
	return value;
}

static uint32 exchange(H7SpiType *s, uint32 size, uint32 value, std_bool queue)
{
	uint32 response = 0U;
	uint32 i;

	for (i = 0U; i < size; i++) {
		uint8 byte = stm32_spi_chip_exchange(s->bus, (uint8)(value >> (8U * i)));

		response |= (uint32)byte << (8U * i);
		if (queue) {
			push(s, byte);
		}
	}
	return response;
}

uint32 stm32h7_spi_read(uint32 addr, uint32 size)
{
	H7SpiType *s = spi_of(addr);
	uint32 offset = (addr - s->base) & ~3U;
	uint32 value = 0U;
	uint32 i;

	if (offset == SR_OFFSET) {
		value = SR_TXP | SR_EOT | SR_SUSP | SR_TXC;
		if (s->count != 0U) {
			value |= SR_RXP;
		}
		return value;
	}
	if (offset == RXDR_OFFSET) {
		for (i = 0U; i < size; i++) {
			value |= (uint32)pop(s) << (8U * i);
		}
		return value;
	}
	value = (offset < spi_regs) ? s->regs[offset / 4U] : 0U;
	if ((offset == CR1_OFFSET) && ((value & CR1_CSUSP) != 0U)) {
		value &= ~CR1_CSTART;
		s->regs[offset / 4U] = value;
	}
	return value >> (8U * ((addr - s->base) & 3U));
}

void stm32h7_spi_write(uint32 addr, uint32 size, uint32 value)
{
	H7SpiType *s = spi_of(addr);
	uint32 offset = (addr - s->base) & ~3U;
	uint32 shift = 8U * ((addr - s->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	if (offset == TXDR_OFFSET) {
		(void)exchange(s, size, value, TRUE);
		return;
	}
	if ((offset == SR_OFFSET) || (offset == IFCR_OFFSET)) {
		return;
	}
	if (offset < spi_regs) {
		s->regs[offset / 4U] = (s->regs[offset / 4U] & ~mask) | ((value << shift) & mask);
		if ((offset == CR1_OFFSET) && ((s->regs[offset / 4U] & CR1_SPE) == 0U)) {
			s->head = 0U;
			s->count = 0U;
		}
		/* EOT/TXC are already true for the instantaneous exchange.  NuttX
		 * enables EOTIE after the DMA completes and waits for this IRQ. */
		if (((offset == IER_OFFSET) || (offset == CR1_OFFSET))
			&& ((s->regs[IER_OFFSET / 4U] & IER_EOTIE) != 0U)) {
			AthrillDeviceIrqLine line;

			line.value = s->irq;
			(void)athrill_device_host()->irq_set(NULL, line, TRUE);
		}
	}
}

std_bool stm32h7_spi_dma_exchange(uint32 txdr, uint32 size, uint32 value,
	uint32 *rxdr, uint32 *response)
{
	H7SpiType *s = spi_of(txdr);

	if ((s == NULL) || (((txdr - s->base) & ~3U) != TXDR_OFFSET)) {
		return FALSE;
	}
	*rxdr = s->base + RXDR_OFFSET;
	*response = exchange(s, size, value, FALSE);
	return TRUE;
}

void stm32h7_spi_reset(void)
{
	static const uint32 fifo_bytes[] = STM32H7_SPI_RX_FIFO_BYTES;
	uint32 i;
	uint32 k;

	bind_registers();
	if ((sizeof(fifo_bytes) / sizeof(fifo_bytes[0])) != SPI_NUM) {
		fprintf(stderr, "STM32H7_SPI_RX_FIFO_BYTES: %u values for %u SPIs\n",
			(unsigned)(sizeof(fifo_bytes) / sizeof(fifo_bytes[0])), (unsigned)SPI_NUM);
		exit(1);
	}

	for (i = 0U; i < SPI_NUM; i++) {
		if ((fifo_bytes[i] == 0U) || (fifo_bytes[i] > STM32H7_SPI_RX_FIFO_MAX)) {
			fprintf(stderr, "STM32H7_SPI_RX_FIFO_BYTES: SPI%u %u bytes, 1..%u allowed\n",
				(unsigned)(i + 1U), (unsigned)fifo_bytes[i], (unsigned)STM32H7_SPI_RX_FIFO_MAX);
			exit(1);
		}
		for (k = 0U; k < SPI_WORDS; k++) {
			spis[i].regs[k] = reset_values[k];
		}
		spis[i].fifo_bytes = fifo_bytes[i];
		spis[i].head = 0U;
		spis[i].count = 0U;
	}
}
