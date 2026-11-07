/*
 * See stm32_spi_bindings.h. A missing binding stops the emulator: the
 * interpretation has a single source.
 * SPDX-License-Identifier: MIT
 */
#include "stm32_spi_bindings.h"
#include "athrill_bindings.h"

#include <stdio.h>
#include <stdlib.h>

static const AthrillBindings *bindings_of(const char *compatible, const char *soc)
{
	char error[256] = "";
	const AthrillBindings *b = athrill_bindings_for(compatible, soc, error, sizeof(error));

	if (b == NULL) {
		fprintf(stderr, "SPI bindings: %s\n", error);
		exit(1);
	}
	return b;
}

uint32 stm32_spi_bind_resets(const char *compatible, const char *soc, uint32 *resets, uint32 max_words)
{
	const AthrillBindings *b = bindings_of(compatible, soc);
	uint32 window = 0U;
	uint32 i;

	for (i = 0U; i < max_words; i++) {
		resets[i] = 0U;
	}
	for (i = 0U; i < athrill_bindings_register_count(b); i++) {
		const AthrillBindRegister *r = athrill_bindings_register(b, i);
		uint32 end = r->offset + (uint32)r->size / 8U;

		if (r->peripheral != 0U) {
			continue;	/* registers of other peripherals */
		}
		if ((r->size != 32U) || ((r->offset % 4U) != 0U) || (end > (max_words * 4U))) {
			fprintf(stderr, "SPI bindings %s: register %s (offset 0x%x, %u bits) outside the %u-byte model\n",
				compatible, r->name, r->offset, (unsigned)r->size, max_words * 4U);
			exit(1);
		}
		resets[r->offset / 4U] = (uint32)r->reset;
		if (end > window) {
			window = end;
		}
	}
	return window;
}

void stm32_spi_bind(const char *compatible, const char *soc, const Stm32SpiBinding *table, uint32 count)
{
	const AthrillBindings *b = bindings_of(compatible, soc);
	uint32 i;
	for (i = 0U; i < count; i++) {
		uint32 offset = 0U, lsb = 0U, width = 0U;
		if (athrill_bindings_find(b, NULL, table[i].reg, table[i].field, &offset, &lsb, &width) != 0) {
			fprintf(stderr, "SPI bindings %s: no %s%s%s\n", compatible, table[i].reg,
				table[i].field != NULL ? "." : "", table[i].field != NULL ? table[i].field : "");
			exit(1);
		}
		switch (table[i].kind) {
		case STM32_BIND_OFFSET:
			*table[i].target = offset;
			break;
		case STM32_BIND_SHIFT:
			*table[i].target = lsb;
			break;
		default:
			*table[i].target = ((width >= 32U) ? 0xFFFFFFFFU : ((1U << width) - 1U)) << lsb;
			break;
		}
	}
}
