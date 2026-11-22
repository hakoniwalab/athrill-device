/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_MODELS_H
#define ATHRILL_DEVICE_MODELS_H

#include "athrill_device_if.h"

#include <stddef.h>

extern const AthrillDeviceModelOps athrill_device_fm25v02_model;
extern const AthrillDeviceModelOps athrill_device_bmi055_model;
extern const AthrillDeviceModelOps athrill_device_adis16470_model;
extern const AthrillDeviceModelOps athrill_device_icp201xx_model;
extern const AthrillDeviceModelOps athrill_device_ms5611_model;
extern const AthrillDeviceModelOps athrill_device_ist8310_model;
extern const AthrillDeviceModelOps athrill_device_bmm150_model;
extern const AthrillDeviceModelOps athrill_device_ublox_m8_model;
extern const AthrillDeviceModelOps athrill_device_icm20689_model;
extern const AthrillDeviceModelOps athrill_device_icm42688p_model;
extern const AthrillDeviceModelOps athrill_device_icm45686_model;
extern const AthrillDeviceModelOps athrill_device_iim42652_model;
extern const AthrillDeviceModelOps athrill_device_stm32_gpio_model;
extern const AthrillDeviceModelOps athrill_device_stm32_exti_model;
extern const AthrillDeviceModelOps athrill_device_stm32_syscfg_model;
extern const AthrillDeviceModelOps athrill_device_stm32_dmamux_model;
extern const AthrillDeviceModelOps athrill_device_stm32_dma_model;
extern const AthrillDeviceModelOps athrill_device_stm32_sdmmc_model;
extern const AthrillDeviceModelOps athrill_device_sdcard_model;
extern const AthrillDeviceModelOps athrill_device_register_block_model;
extern const AthrillDeviceModelOps athrill_device_riscv_mtimer_model;
extern const AthrillDeviceModelOps athrill_device_test_watchdog_model;
extern const AthrillDeviceModelOps athrill_device_stm32_iwdg_model;
extern const AthrillDeviceModelOps athrill_device_shakti_uart_model;
extern const AthrillDeviceModelOps athrill_device_bench_sensor_model;
extern const AthrillDeviceModelOps athrill_device_esc_fault_bypass_model;
extern const AthrillDeviceModelOps athrill_device_dshot_esc_model;
extern const AthrillDeviceModelOps athrill_device_esc_fault_schedule_model;
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

/* ABI 1.24: use a time-resolved sample when the source offers one, otherwise
 * retain the pre-1.24 latest-sample behaviour. */
static inline sint32 athrill_device_sensor_sample_at(
	const AthrillDeviceValueSource *source, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	const AthrillDeviceValueProviderOps *ops = source->ops;
	if (ops != NULL && ops->abi_major == ATHRILL_DEVICE_ABI_MAJOR &&
	    ops->abi_minor >= 24U &&
	    ops->struct_size >= offsetof(AthrillDeviceValueProviderOps,
	    sensor_sample_at) + sizeof(ops->sensor_sample_at) &&
	    ops->sensor_sample_at != NULL &&
	    ops->sensor_sample_at(source->instance, time, sample) == 0) {
		return 0;
	}
	if (ops != NULL && ops->sensor_sample != NULL) {
		return ops->sensor_sample(source->instance, sample);
	}
	return -1;
}

#endif
