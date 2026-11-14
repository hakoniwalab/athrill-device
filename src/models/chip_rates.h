/*
 * Output data rates and full scales of the IMU chips, from the values the firmware writes
 * to their registers (spec/chips.yaml ADV-CHIP-BMI055A-002, ADV-CHIP-BMI055G-002,
 * ADV-CHIP-ICM42688-002, ADV-CHIP-ICM42688-003, ADV-CHIP-ICM20689-002). Rates are in mHz so that the slow
 * settings (7.8125 Hz bandwidth, 12.5 Hz) are exact; 0 means a reserved code.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_CHIP_RATES_H
#define ATHRILL_CHIP_RATES_H

#include "athrill_device_if.h"

/* BMI055 accelerometer: PMU_BW (0x10) and ACCD_HBW (0x13). */
uint64 chip_bmi055_accel_rate_mhz(uint8 pmu_bw, uint8 accd_hbw);
/* BMI055 gyroscope: BW (0x10) and RATE_HBW (0x13). */
uint64 chip_bmi055_gyro_rate_mhz(uint8 bw, uint8 rate_hbw);
/* ICM-42688-P: GYRO_CONFIG0 (0x4F) and ACCEL_CONFIG0 (0x50), bank 0. */
uint64 chip_icm42688_gyro_rate_mhz(uint8 gyro_config0);
uint64 chip_icm42688_accel_rate_mhz(uint8 accel_config0);
/* ICM-42688-P 16-bit data: counts per dps and per g for the configured full scale. */
float32 chip_icm42688_gyro_counts_per_dps(uint8 gyro_config0);
float32 chip_icm42688_accel_counts_per_g(uint8 accel_config0);

/* ICM-20689: CONFIG (0x1A), GYRO_CONFIG (0x1B), SMPLRT_DIV (0x19), ACCEL_CONFIG2 (0x1D). The
 * gyroscope's rate is the sample rate (the FIFO's); the accelerometer's its own. */
uint64 chip_icm20689_gyro_rate_mhz(uint8 config, uint8 gyro_config, uint8 smplrt_div);
uint64 chip_icm20689_accel_rate_mhz(uint8 accel_config2, uint8 smplrt_div);

/* The period of a rate in target clocks (frequency_hz: the target's time base) and in ns. */
uint64 chip_rate_clocks(uint64 frequency_hz, uint64 rate_mhz);
uint64 chip_rate_period_ns(uint64 rate_mhz);

#endif
