/*
 * STM32H7 DMAMUX (RM0433 section 17.6): the request selector registers of
 * one DMAMUX instance (DMAMUX1, DMAMUX2), one model per Devicetree node.
 *
 * Ported unchanged from athrill-target-stm32's stm32_dmamux.c: every
 * register keeps what is written to it, and a write to CFR clears the
 * written bits of CSR; its interrupt is connected and never raised. The
 * request routing itself is not modelled here: the
 * DMA model serves the requests (spec/semantics/st-stm32-dmamux.yaml).
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DMAMUX_SIZE	0x400U
#define DMAMUX_CSR	0x80U	/* request line multiplexer interrupt status */
#define DMAMUX_CFR	0x84U	/* its clear flag register */

typedef struct {
	uint32 base;
	uint32 regs[DMAMUX_SIZE / 4U];
	/* The overrun interrupt (interrupts, index 0): connected, never raised;
	 * the synchronisation overrun and request generator are not modelled. */
	AthrillDeviceIrqLine irq;
} Stm32Dmamux;

static sint32 dmamux_validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 size;
	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 ||
	    size != DMAMUX_SIZE) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size,
				"DMAMUX register window must be 0x%x bytes", DMAMUX_SIZE);
		}
		return -1;
	}
	return 0;
}

static sint32 dmamux_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Stm32Dmamux *dmamux = calloc(1U, sizeof(*dmamux));
	if (dmamux == NULL || args->node->ops->get_u32(args->node->context,
		"reg", 0U, &dmamux->base) != 0) {
		free(dmamux);
		return -1;
	}
	*instance = dmamux;
	return 0;
}

static void dmamux_destroy(void *instance)
{
	free(instance);
}

static sint32 dmamux_read(void *instance, uint64 address, uint32 size,
	uint64 *result)
{
	Stm32Dmamux *dmamux = instance;
	uint32 offset;
	if (address < dmamux->base || address >= (uint64)dmamux->base + DMAMUX_SIZE ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	offset = (uint32)address - dmamux->base;
	*result = dmamux->regs[(offset & ~3U) / 4U] >> (8U * (offset & 3U));
	return 0;
}

static sint32 dmamux_write(void *instance, uint64 address, uint32 size,
	uint64 input)
{
	Stm32Dmamux *dmamux = instance;
	uint32 offset, shift, mask, value;
	if (address < dmamux->base || address >= (uint64)dmamux->base + DMAMUX_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = (uint32)address - dmamux->base;
	shift = 8U * (offset & 3U);
	mask = ((size == 4U) ? 0xffffffffU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	offset &= ~3U;
	if (offset == DMAMUX_CFR) {
		dmamux->regs[DMAMUX_CSR / 4U] &= ~value;
	} else {
		dmamux->regs[offset / 4U] = (dmamux->regs[offset / 4U] & ~mask) | value;
	}
	return 0;
}

static sint32 dmamux_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Stm32Dmamux *dmamux = instance;
	return args->resources->get_interrupt(args->context, NULL, 0U,
		&dmamux->irq);
}

static const AthrillDeviceMmioOps dmamux_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), dmamux_read, dmamux_write
};

static sint32 dmamux_reset(void *instance)
{
	Stm32Dmamux *dmamux = instance;
	memset(dmamux->regs, 0, sizeof(dmamux->regs));
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_dmamux_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-dmamux", "1", dmamux_validate,
	dmamux_create, dmamux_destroy, &dmamux_mmio, NULL, NULL, NULL,
	dmamux_connect, dmamux_reset, NULL
};
