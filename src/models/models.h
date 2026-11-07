/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_MODELS_H
#define ATHRILL_DEVICE_MODELS_H

#include "athrill_device_if.h"

#include <stddef.h>

extern const AthrillDeviceModelOps athrill_device_fm25v02_model;
extern const AthrillDeviceModelOps athrill_device_bmi055_model;
extern const AthrillDeviceModelOps athrill_device_ms5611_model;
extern const AthrillDeviceModelOps athrill_device_ist8310_model;
extern const AthrillDeviceModelOps athrill_device_ublox_m8_model;
extern const AthrillDeviceModelOps athrill_device_icm20689_model;
extern const AthrillDeviceModelOps athrill_device_icm42688p_model;
extern const AthrillDeviceModelOps athrill_device_stm32_gpio_model;
extern const AthrillDeviceModelOps athrill_device_stm32_exti_model;
extern const AthrillDeviceModelOps athrill_device_stm32_syscfg_model;
extern const AthrillDeviceModelOps athrill_device_stm32_dmamux_model;
extern const AthrillDeviceModelOps athrill_device_stm32_dma_model;
extern const AthrillDeviceModelOps athrill_device_stm32_sdmmc_model;
extern const AthrillDeviceModelOps athrill_device_sdcard_model;
extern const AthrillDeviceModelOps athrill_device_stm32_i2c_model;
extern const AthrillDeviceModelOps athrill_device_stm32_spi_model;
extern const AthrillDeviceModelOps athrill_device_stm32h7_spi_model;
extern const AthrillDeviceModelOps athrill_device_stm32_uart_model;
extern const AthrillDeviceModelOps athrill_device_stm32_tim_model;
extern const AthrillDeviceModelOps athrill_device_pwm_actuator_model;

static inline sint32 athrill_device_connect_value_source(
	const AthrillDeviceNode *node, const AthrillDeviceConnectArgs *args,
	AthrillDeviceValueSource *source)
{
	const void *property;
	uint32 size;
	if (node->ops->get_property(node->context, "value-source", &property,
		&size) != 0) {
		return 0;
	}
	if (size != sizeof(uint32) || args->resources->get_value_source == NULL ||
	    args->resources->get_value_source(args->context, "value-source",
		    source) != 0) {
		return -1;
	}
	return 0;
}

#endif
