/*
 * STM32F7/H7 EXTI and SYSCFG_EXTICR model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F7_EXTI_BASE       0x40013c00U
#define F7_EXTICR_BASE     0x40013808U
#define F7_GPIO_BASE       0x40020000U
#define H7_EXTI_BASE       0x58000000U
#define H7_EXTICR_BASE     0x58000408U
#define H7_GPIO_BASE       0x58020000U
#define GPIO_STRIDE        0x400U
#define GPIO_PORTS         11U
#define GPIO_PINS          16U
#define EXTI_REGS          0xacU
#define EXTICR_SIZE        0x10U
#define LINES_MASK         0x01ffffffU
#define EXTI_IRQS          7U

typedef struct Stm32Exti Stm32Exti;
typedef struct {
	Stm32Exti *exti;
	uint32 port;
	uint32 pin;
} Stm32ExtiInput;

struct Stm32Exti {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint32 regs[EXTI_REGS / 4U];
	uint32 exticr[EXTICR_SIZE / 4U];
	AthrillDeviceIrqLine irqs[EXTI_IRQS];
	AthrillDeviceMmioToken exticr_token;
	Stm32ExtiInput inputs[GPIO_PORTS][GPIO_PINS];
	uint32 exti_base;
	uint32 exticr_base;
	uint32 gpio_base;
	uint32 imr;
	uint32 emr;
	uint32 rtsr;
	uint32 ftsr;
	uint32 swier;
	uint32 pr;
	std_bool exticr_registered;
};

static sint32 exti_validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 base;
	if (node->ops->get_u32(node->context, "reg", 0U, &base) != 0 ||
	    (base != F7_EXTI_BASE && base != H7_EXTI_BASE)) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "unsupported EXTI register base");
		}
		return -1;
	}
	return 0;
}

static sint32 exti_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Stm32Exti *exti = calloc(1U, sizeof(*exti));
	uint32 base, port, pin;
	if (exti == NULL || args->node->ops->get_u32(args->node->context,
		"reg", 0U, &base) != 0) {
		free(exti);
		return -1;
	}
	exti->target_ops = args->target_ops;
	exti->target = args->target;
	exti->exti_base = base;
	if (base == F7_EXTI_BASE) {
		exti->exticr_base = F7_EXTICR_BASE;
		exti->gpio_base = F7_GPIO_BASE;
		exti->imr = 0x00U;
		exti->emr = 0x04U;
		exti->rtsr = 0x08U;
		exti->ftsr = 0x0cU;
		exti->swier = 0x10U;
		exti->pr = 0x14U;
	} else {
		exti->exticr_base = H7_EXTICR_BASE;
		exti->gpio_base = H7_GPIO_BASE;
		exti->rtsr = 0x00U;
		exti->ftsr = 0x04U;
		exti->swier = 0x08U;
		exti->imr = 0x80U;
		exti->emr = 0x84U;
		exti->pr = 0x88U;
	}
	for (port = 0U; port < GPIO_PORTS; port++) {
		for (pin = 0U; pin < GPIO_PINS; pin++) {
			exti->inputs[port][pin].exti = exti;
			exti->inputs[port][pin].port = port;
			exti->inputs[port][pin].pin = pin;
		}
	}
	*instance = exti;
	return 0;
}

static uint32 irq_index(uint32 line)
{
	if (line <= 4U) {
		return line;
	}
	return line <= 9U ? 5U : 6U;
}

static void pend(Stm32Exti *exti, uint32 lines)
{
	uint32 line;
	exti->regs[exti->pr / 4U] |= lines;
	for (line = 0U; line < GPIO_PINS; line++) {
		if ((lines & exti->regs[exti->imr / 4U] & (1U << line)) != 0U) {
			(void)exti->target_ops->irq_set(exti->target,
				exti->irqs[irq_index(line)], TRUE);
		}
	}
}

static sint32 exti_read(void *instance, uint64 address, uint32 size,
	uint64 *result)
{
	Stm32Exti *exti = instance;
	uint32 offset, shift, value;
	if ((size != 1U && size != 2U && size != 4U) || result == NULL ||
	    address > 0xffffffffULL) {
		return -1;
	}
	if (address >= exti->exticr_base &&
	    address < (uint64)exti->exticr_base + EXTICR_SIZE) {
		offset = (uint32)address - exti->exticr_base;
		shift = 8U * (offset & 3U);
		*result = exti->exticr[(offset & ~3U) / 4U] >> shift;
		return 0;
	}
	if (address < exti->exti_base ||
	    address >= (uint64)exti->exti_base + EXTI_REGS) {
		return -1;
	}
	offset = (uint32)address - exti->exti_base;
	shift = 8U * (offset & 3U);
	value = exti->regs[(offset & ~3U) / 4U];
	*result = value >> shift;
	return 0;
}

static sint32 exti_write(void *instance, uint64 address, uint32 size,
	uint64 input)
{
	Stm32Exti *exti = instance;
	uint32 offset, shift, mask, value;
	if ((size != 1U && size != 2U && size != 4U) ||
	    address > 0xffffffffULL) {
		return -1;
	}
	if (address >= exti->exticr_base &&
	    address < (uint64)exti->exticr_base + EXTICR_SIZE) {
		offset = (uint32)address - exti->exticr_base;
		shift = 8U * (offset & 3U);
		mask = ((size == 4U) ? 0xffffffffU :
			((1U << (8U * size)) - 1U)) << shift;
		value = ((uint32)input << shift) & mask;
		offset &= ~3U;
		exti->exticr[offset / 4U] =
			((exti->exticr[offset / 4U] & ~mask) | value) & 0xffffU;
		return 0;
	}
	if (address < exti->exti_base ||
	    address >= (uint64)exti->exti_base + EXTI_REGS) {
		return -1;
	}
	offset = (uint32)address - exti->exti_base;
	shift = 8U * (offset & 3U);
	mask = ((size == 4U) ? 0xffffffffU :
		((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	offset &= ~3U;
	if (offset == exti->pr) {
		exti->regs[offset / 4U] &= ~value;
	} else if (offset == exti->swier) {
		pend(exti, value & LINES_MASK);
	} else {
		exti->regs[offset / 4U] =
			((exti->regs[offset / 4U] & ~mask) | value) & LINES_MASK;
	}
	return 0;
}

static const AthrillDeviceMmioOps exti_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), exti_read, exti_write
};

static void gpio_edge(void *context, std_bool level)
{
	Stm32ExtiInput *input = context;
	Stm32Exti *exti = input->exti;
	uint32 selected = (exti->exticr[input->pin / 4U] >>
		(4U * (input->pin % 4U))) & 0xfU;
	uint32 edges = exti->regs[(level ? exti->rtsr : exti->ftsr) / 4U];
	if (selected == input->port && (edges & (1U << input->pin)) != 0U) {
		pend(exti, 1U << input->pin);
	}
}

static sint32 exti_connect(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	static const char *const names[EXTI_IRQS] = {
		"line0", "line1", "line2", "line3", "line4", "line5-9", "line10-15"
	};
	Stm32Exti *exti = instance;
	uint32 port, pin;
	if (exti->target_ops->irq_set == NULL ||
	    exti->target_ops->mmio_register == NULL ||
	    exti->target_ops->mmio_unregister == NULL ||
	    exti->target_ops->gpio_resolve == NULL ||
	    exti->target_ops->gpio_subscribe == NULL) {
		return -1;
	}
	for (pin = 0U; pin < EXTI_IRQS; pin++) {
		if (args->resources->get_interrupt(args->context, names[pin], 0U,
			&exti->irqs[pin]) != 0) {
			return -1;
		}
	}
	if (exti->target_ops->mmio_register(exti->target, exti->exticr_base,
		EXTICR_SIZE, &exti_mmio, exti, &exti->exticr_token) != 0) {
		return -1;
	}
	exti->exticr_registered = TRUE;
	for (port = 0U; port < GPIO_PORTS; port++) {
		char provider_path[32];
		(void)snprintf(provider_path, sizeof(provider_path), "/gpio@%x",
			exti->gpio_base + port * GPIO_STRIDE);
		for (pin = 0U; pin < GPIO_PINS; pin++) {
			AthrillDeviceSpecifier spec;
			AthrillDeviceGpioLine line;
			memset(&spec, 0, sizeof(spec));
			spec.provider_path = provider_path;
			spec.provider_compatible = "st,stm32-gpio";
			spec.cell_count = 2U;
			spec.cells[0] = pin;
			if (exti->target_ops->gpio_resolve(exti->target, &spec, &line) != 0 ||
			    exti->target_ops->gpio_subscribe(exti->target, line, gpio_edge,
				&exti->inputs[port][pin]) != 0) {
				return -1;
			}
		}
	}
	return 0;
}

static sint32 exti_reset(void *instance)
{
	Stm32Exti *exti = instance;
	memset(exti->regs, 0, sizeof(exti->regs));
	memset(exti->exticr, 0, sizeof(exti->exticr));
	return 0;
}

static sint32 exti_tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	return 0;
}

static void exti_destroy(void *instance)
{
	Stm32Exti *exti = instance;
	if (exti->exticr_registered && exti->target_ops->mmio_unregister != NULL) {
		exti->target_ops->mmio_unregister(exti->target, exti->exticr_token);
	}
	free(exti);
}

const AthrillDeviceModelOps athrill_device_stm32_exti_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-exti", "1", exti_validate,
	exti_create, exti_destroy, &exti_mmio, NULL, NULL, NULL, exti_connect,
	exti_reset, exti_tick
};
