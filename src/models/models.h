/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_MODELS_H
#define ATHRILL_DEVICE_MODELS_H

#include "athrill_device_if.h"

extern const AthrillDeviceModelOps athrill_device_fm25v02_model;
extern const AthrillDeviceModelOps athrill_device_ms5611_model;
extern const AthrillDeviceModelOps athrill_device_ist8310_model;
extern const AthrillDeviceModelOps athrill_device_ublox_m8_model;
extern const AthrillDeviceModelOps athrill_device_icm20689_model;
extern const AthrillDeviceModelOps athrill_device_stm32_gpio_model;
extern const AthrillDeviceModelOps athrill_device_stm32_exti_model;
extern const AthrillDeviceModelOps athrill_device_stm32_tim_model;
extern const AthrillDeviceModelOps athrill_device_pwm_actuator_model;

#endif
