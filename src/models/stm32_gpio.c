/*
 * STM32F7 GPIO provider. Register offsets and reset values are from RM0410
 * section 6.4.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F7_GPIO_BASE 0x40020000U
#define H7_GPIO_BASE 0x58020000U
#define GPIO_STRIDE  0x400U
#define GPIO_PORTS   11U

#define GPIO_MODER   0x00U
#define GPIO_OSPEEDR 0x08U
#define GPIO_PUPDR   0x0cU
#define GPIO_IDR     0x10U
#define GPIO_ODR     0x14U
#define GPIO_BSRR    0x18U
#define GPIO_REGS    0x28U
#define GPIO_PINS    16U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceGpioLine lines[GPIO_PINS];
	uint32 regs[GPIO_REGS / 4U];
	uint32 inputs;
	uint32 base;
	/* output changes go through the runtime (model watchers, then the target) */
	const AthrillDeviceResourceOps *resources;
	void *resource_context;
} Stm32Gpio;

typedef struct {
	Stm32Gpio *gpio;
	uint32 pin;
} Stm32GpioInput;

typedef struct {
	Stm32Gpio gpio;
	Stm32GpioInput inputs[GPIO_PINS];
} Stm32GpioInstance;

static sint32 gpio_validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 base;
	if (node->ops->get_u32(node->context, "reg", 0U, &base) != 0 ||
	    (!((base >= F7_GPIO_BASE && base < F7_GPIO_BASE + GPIO_PORTS * GPIO_STRIDE) ||
	       (base >= H7_GPIO_BASE && base < H7_GPIO_BASE + GPIO_PORTS * GPIO_STRIDE))) ||
	    ((base - (base >= H7_GPIO_BASE ? H7_GPIO_BASE : F7_GPIO_BASE)) % GPIO_STRIDE) != 0U) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "unsupported GPIO register base");
		}
		return -1;
	}
	return 0;
}

static sint32 gpio_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Stm32GpioInstance *holder = calloc(1U, sizeof(*holder));
	uint32 pin;
	if (holder == NULL || args->node->ops->get_u32(args->node->context,
		"reg", 0U, &holder->gpio.base) != 0) {
		free(holder);
		return -1;
	}
	holder->gpio.target_ops = args->target_ops;
	holder->gpio.target = args->target;
	holder->gpio.node = args->node;
	for (pin = 0U; pin < GPIO_PINS; pin++) {
		holder->inputs[pin].gpio = &holder->gpio;
		holder->inputs[pin].pin = pin;
	}
	*instance = holder;
	return 0;
}

static uint32 output_pins(const Stm32Gpio *gpio)
{
	uint32 pins = 0U, pin;
	for (pin = 0U; pin < GPIO_PINS; pin++) {
		if (((gpio->regs[GPIO_MODER / 4U] >> (2U * pin)) & 3U) == 1U) {
			pins |= 1U << pin;
		}
	}
	return pins;
}

static sint32 gpio_read(void *instance, uint64 address, uint32 size,
	uint64 *result)
{
	Stm32Gpio *gpio = &((Stm32GpioInstance *)instance)->gpio;
	uint32 offset, shift, value;
	if (address < gpio->base || address >= (uint64)gpio->base + GPIO_STRIDE ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	offset = (uint32)(address - gpio->base);
	shift = 8U * (offset & 3U);
	offset &= ~3U;
	if (offset == GPIO_IDR) {
		uint32 outputs = output_pins(gpio);
		value = (gpio->regs[GPIO_ODR / 4U] & outputs) |
			(gpio->inputs & ~outputs);
	} else if (offset < GPIO_REGS && offset != GPIO_BSRR) {
		value = gpio->regs[offset / 4U];
	} else {
		value = 0U;
	}
	*result = value >> shift;
	return 0;
}

static void set_odr(Stm32Gpio *gpio, uint32 value)
{
	uint32 old = gpio->regs[GPIO_ODR / 4U];
	gpio->regs[GPIO_ODR / 4U] = value & 0xffffU;
	if (old != gpio->regs[GPIO_ODR / 4U]) {
		(void)gpio->resources->publish_gpio_outputs(gpio->resource_context,
			gpio->lines, GPIO_PINS, gpio->regs[GPIO_ODR / 4U]);
	}
}

static sint32 gpio_write(void *instance, uint64 address, uint32 size,
	uint64 input)
{
	Stm32Gpio *gpio = &((Stm32GpioInstance *)instance)->gpio;
	uint32 offset, shift, mask, value, odr;
	if (address < gpio->base || address >= (uint64)gpio->base + GPIO_STRIDE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = (uint32)(address - gpio->base);
	shift = 8U * (offset & 3U);
	mask = ((size == 4U) ? 0xffffffffU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	offset &= ~3U;
	switch (offset) {
	case GPIO_BSRR:
		odr = gpio->regs[GPIO_ODR / 4U];
		odr &= ~(value >> 16);
		odr |= value & 0xffffU;
		set_odr(gpio, odr);
		break;
	case GPIO_ODR:
		set_odr(gpio, (gpio->regs[GPIO_ODR / 4U] & ~mask) | value);
		break;
	case GPIO_IDR:
		break;
	default:
		if (offset < GPIO_REGS) {
			gpio->regs[offset / 4U] =
				(gpio->regs[offset / 4U] & ~mask) | value;
		}
		break;
	}
	return 0;
}

static const AthrillDeviceMmioOps gpio_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), gpio_read, gpio_write
};

static void gpio_input(void *context, std_bool level)
{
	Stm32GpioInput *input = context;
	if (level) {
		input->gpio->inputs |= 1U << input->pin;
	} else {
		input->gpio->inputs &= ~(1U << input->pin);
	}
}

static sint32 gpio_connect(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Stm32GpioInstance *holder = instance;
	Stm32Gpio *gpio = &holder->gpio;
	uint32 pin;
	gpio->resources = args->resources;
	gpio->resource_context = args->context;
	if (args->resources->publish_gpio_outputs == NULL) {
		return -1;
	}
	for (pin = 0U; pin < GPIO_PINS; pin++) {
		AthrillDeviceSpecifier spec;
		memset(&spec, 0, sizeof(spec));
		spec.provider = gpio->node->id;
		spec.provider_path = gpio->node->path;
		spec.provider_compatible = gpio->node->matched_compatible;
		spec.cell_count = 2U;
		spec.cells[0] = pin;
		if (args->resources->gpio_resolve(args->context, &spec,
			&gpio->lines[pin]) != 0 ||
		    args->resources->gpio_subscribe(args->context,
			gpio->lines[pin], gpio_input, &holder->inputs[pin]) != 0) {
			return -1;
		}
	}
	return 0;
}

static sint32 gpio_reset(void *instance)
{
	Stm32Gpio *gpio = &((Stm32GpioInstance *)instance)->gpio;
	uint32 port_base = gpio->base >= H7_GPIO_BASE ? H7_GPIO_BASE : F7_GPIO_BASE;
	uint32 port = (gpio->base - port_base) / GPIO_STRIDE;
	memset(gpio->regs, 0, sizeof(gpio->regs));
	/* Preserve levels driven by board models during the connect phase. */
	if (port == 0U) {
		gpio->regs[GPIO_MODER / 4U] = 0xa8000000U;
		gpio->regs[GPIO_PUPDR / 4U] = 0x64000000U;
	} else if (port == 1U) {
		gpio->regs[GPIO_MODER / 4U] = 0x00000280U;
		gpio->regs[GPIO_OSPEEDR / 4U] = 0x000000c0U;
		gpio->regs[GPIO_PUPDR / 4U] = 0x00000100U;
	}
	return 0;
}

static sint32 gpio_tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	result->contract = ATHRILL_DEVICE_TICK_NONE;	/* ABI 1.19: this tick does nothing */
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_gpio_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-gpio", "1", gpio_validate,
	gpio_create, free, &gpio_mmio, NULL, NULL, NULL, gpio_connect,
	gpio_reset, gpio_tick
};
