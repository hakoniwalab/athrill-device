/*
 * Register interpretation of the STM32 SPI front ends from the athrill-device
 * bindings (offset, field mask or field shift by SVD register/field name),
 * resolved once at reset into plain variables used like the former #defines.
 * SPDX-License-Identifier: MIT
 */
#ifndef STM32_SPI_BINDINGS_H
#define STM32_SPI_BINDINGS_H

#include "std_types.h"

typedef enum { STM32_BIND_OFFSET, STM32_BIND_MASK, STM32_BIND_SHIFT } Stm32BindKind;

typedef struct {
	uint32 *target;
	const char *reg;
	const char *field;
	Stm32BindKind kind;
} Stm32SpiBinding;

/* Table rows. Naming rule: the variable is spelt from the SVD names,
 *   <REG>_OFFSET         register offset   BIND_OFFSET(SR)       -> SR_OFFSET      <- "SR"
 *   <REG>_<FIELD>        field mask        BIND_MASK(SR, RXNE)   -> SR_RXNE        <- "SR", "RXNE"
 *   <REG>_<FIELD>_SHIFT  field lsb         BIND_SHIFT(SR, FRLVL) -> SR_FRLVL_SHIFT <- "SR", "FRLVL" */
#define BIND_OFFSET(r)    { &r##_OFFSET, #r, NULL, STM32_BIND_OFFSET }
#define BIND_MASK(r, f)   { &r##_##f, #r, #f, STM32_BIND_MASK }
#define BIND_SHIFT(r, f)  { &r##_##f##_SHIFT, #r, #f, STM32_BIND_SHIFT }

void stm32_spi_bind(const char *compatible, const char *soc, const Stm32SpiBinding *table, uint32 count);

/* Reset values of the SPI's own registers by word (resets[offset / 4], 0
 * where no register is) and the register window: the end of the last
 * register in bytes. Stops the emulator when the window exceeds max_words. */
uint32 stm32_spi_bind_resets(const char *compatible, const char *soc, uint32 *resets, uint32 max_words);

#endif
