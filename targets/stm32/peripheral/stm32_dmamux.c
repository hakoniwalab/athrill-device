/* DMAMUX1/2: DMA request selectors plus status/clear registers. */
#include "stm32_dmamux.h"
#include <string.h>

#define DMAMUX1_BASE 0x40020800U
#define DMAMUX2_BASE 0x58025800U
#define DMAMUX_SIZE 0x400U
#define DMAMUX_CSR  0x80U
#define DMAMUX_CFR  0x84U

static uint32 regs[2][DMAMUX_SIZE / 4U];

static uint32 base_of(uint32 addr)
{
	return (addr >= DMAMUX2_BASE) ? DMAMUX2_BASE : DMAMUX1_BASE;
}

static uint32 *regs_of(uint32 addr)
{
	return regs[(addr >= DMAMUX2_BASE) ? 1U : 0U];
}

std_bool stm32_dmamux_owns(uint32 addr)
{
	return ((addr >= DMAMUX1_BASE) && (addr < (DMAMUX1_BASE + DMAMUX_SIZE)))
		|| ((addr >= DMAMUX2_BASE) && (addr < (DMAMUX2_BASE + DMAMUX_SIZE)));
}

uint32 stm32_dmamux_read(uint32 addr)
{
	uint32 offset = (addr - base_of(addr)) & ~3U;
	uint32 *r = regs_of(addr);

	return r[offset / 4U] >> (8U * (addr & 3U));
}

void stm32_dmamux_write(uint32 addr, uint32 size, uint32 value)
{
	uint32 offset = (addr - base_of(addr)) & ~3U;
	uint32 *r = regs_of(addr);
	uint32 shift = 8U * (addr & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	value = (value << shift) & mask;
	if (offset == DMAMUX_CFR) {
		r[DMAMUX_CSR / 4U] &= ~value;
	}
	else {
		r[offset / 4U] = (r[offset / 4U] & ~mask) | value;
	}
}

void stm32_dmamux_reset(void)
{
	memset(regs, 0, sizeof(regs));
}
