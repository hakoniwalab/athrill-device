/*
 * Output data rates and full scales of the IMU chips (chip_rates.h).
 *
 * Sources, by register name: the BMI055 datasheet (accelerometer PMU_BW and ACCD_HBW,
 * gyroscope BW and RATE_HBW); the ICM-42688-P datasheet register map (GYRO_CONFIG0 and
 * ACCEL_CONFIG0: GYRO_ODR/ACCEL_ODR in bits 3:0, the full scale in bits 7:5). The codes the
 * firmwares write are cross-checked with their drivers: PX4 v1.17.0 sets data_high_bw on
 * both BMI055 dies (2 kHz) and GYRO_ODR/ACCEL_ODR 8 kHz with 2000 dps and 16 g; ArduPilot
 * writes PMU_BW 0x0F, ACCD_HBW 0x80, BW 0x81 (2 kHz each) and ODR codes 0x03-0x06
 * (8, 4, 2, 1 kHz). ICM-20689: the datasheet register map (CONFIG DLPF_CFG, GYRO_CONFIG
 * FCHOICE_B, SMPLRT_DIV, ACCEL_CONFIG2 ACCEL_FCHOICE_B), with PX4's settings.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "chip_rates.h"

#define DATA_HIGH_BW 0x80U

/* [ADV-CHIP-BMI055A-002] Unfiltered data (data_high_bw) comes at 2 kHz; filtered data at
 * twice the bandwidth of PMU_BW bits 4:0: codes 0x08..0x0F are 7.81, 15.63, 31.25, 62.5,
 * 125, 250, 500 and 1000 Hz; codes below 0x08 act as 0x08 and above 0x0F as 0x0F. */
uint64 chip_bmi055_accel_rate_mhz(uint8 pmu_bw, uint8 accd_hbw)
{
	uint8 code = (uint8)(pmu_bw & 0x1FU);
	if ((accd_hbw & DATA_HIGH_BW) != 0U) {
		return 2000000U;
	}
	if (code < 0x08U) {
		code = 0x08U;
	}
	if (code > 0x0FU) {
		code = 0x0FU;
	}
	/* 0x08: 7.8125 Hz bandwidth, doubling per code; the data rate is twice the bandwidth */
	return 15625U << (code - 0x08U);
}

/* [ADV-CHIP-BMI055G-002] Unfiltered data (data_high_bw) comes at 2 kHz; otherwise BW bits
 * 2:0 give the output data rate: 0 and 1: 2000 Hz, 2: 1000, 3: 400, 4: 200, 5: 100, 6: 200,
 * 7: 100 Hz. */
uint64 chip_bmi055_gyro_rate_mhz(uint8 bw, uint8 rate_hbw)
{
	static const uint64 rate_hz[8] = { 2000U, 2000U, 1000U, 400U, 200U, 100U, 200U, 100U };
	if ((rate_hbw & DATA_HIGH_BW) != 0U) {
		return 2000000U;
	}
	return rate_hz[bw & 7U] * 1000U;
}

/* [ADV-CHIP-ICM42688-002] GYRO_ODR / ACCEL_ODR (bits 3:0): 1: 32 kHz, 2: 16 kHz, 3: 8 kHz,
 * 4: 4 kHz, 5: 2 kHz, 6: 1 kHz, 7: 200 Hz, 8: 100 Hz, 9: 50 Hz, 10: 25 Hz, 11: 12.5 Hz,
 * 15: 500 Hz; the accelerometer also 12: 6.25 Hz, 13: 3.125 Hz, 14: 1.5625 Hz (low power).
 * 0 and the gyroscope's 12-14 are reserved. */
static const uint64 icm42688_odr_mhz[16] = {
	0U, 32000000U, 16000000U, 8000000U, 4000000U, 2000000U, 1000000U, 200000U,
	100000U, 50000U, 25000U, 12500U, 6250U, 3125U, 1562U, 500000U
};

uint64 chip_icm42688_gyro_rate_mhz(uint8 gyro_config0)
{
	uint8 code = (uint8)(gyro_config0 & 0x0FU);
	return (code >= 12U && code <= 14U) ? 0U : icm42688_odr_mhz[code];
}

uint64 chip_icm42688_accel_rate_mhz(uint8 accel_config0)
{
	return icm42688_odr_mhz[accel_config0 & 0x0FU];
}

/* [ADV-CHIP-ICM42688-003] GYRO_FS_SEL (bits 7:5): 0: ±2000 dps (16.4 counts/dps), 1: ±1000
 * (32.8), 2: ±500 (65.5), 3: ±250 (131), 4: ±125 (262), 5: ±62.5 (524.3), 6: ±31.25
 * (1048.6), 7: ±15.625 (2097.2). ACCEL_FS_SEL (bits 7:5): 0: ±16 g (2048 counts/g), 1: ±8 g
 * (4096), 2: ±4 g (8192), 3: ±2 g (16384); 4-7 are reserved and read as ±16 g. */
float32 chip_icm42688_gyro_counts_per_dps(uint8 gyro_config0)
{
	static const float32 counts[8] = { 16.4F, 32.8F, 65.5F, 131.0F, 262.0F, 524.3F, 1048.6F, 2097.2F };
	return counts[(gyro_config0 >> 5) & 7U];
}

float32 chip_icm42688_accel_counts_per_g(uint8 accel_config0)
{
	static const float32 counts[8] = { 2048.0F, 4096.0F, 8192.0F, 16384.0F, 2048.0F, 2048.0F, 2048.0F, 2048.0F };
	return counts[(accel_config0 >> 5) & 7U];
}

/* [ADV-CHIP-ICM20689-002] Gyroscope (the sample rate, the FIFO's): FCHOICE_B (GYRO_CONFIG bits
 * 1:0) not 0 bypasses the DLPF at 32 kHz; otherwise DLPF_CFG (CONFIG bits 2:0) 0 or 7 runs at
 * 8 kHz, 1-6 at 1 kHz / (1 + SMPLRT_DIV). Accelerometer: ACCEL_FCHOICE_B (ACCEL_CONFIG2 bit 3)
 * bypasses its DLPF at 4 kHz; otherwise 1 kHz / (1 + SMPLRT_DIV). PX4 v1.17.0 sets DLPF_CFG 7,
 * FCHOICE_B 0 and ACCEL_FCHOICE_B: 8 kHz and 4 kHz. */
uint64 chip_icm20689_gyro_rate_mhz(uint8 config, uint8 gyro_config, uint8 smplrt_div)
{
	uint8 dlpf = (uint8)(config & 7U);
	if ((gyro_config & 3U) != 0U) {
		return 32000000U;
	}
	if (dlpf == 0U || dlpf == 7U) {
		return 8000000U;
	}
	return 1000000U / (1U + (uint64)smplrt_div);
}

uint64 chip_icm20689_accel_rate_mhz(uint8 accel_config2, uint8 smplrt_div)
{
	if ((accel_config2 & 0x08U) != 0U) {
		return 4000000U;
	}
	return 1000000U / (1U + (uint64)smplrt_div);
}

uint64 chip_rate_clocks(uint64 frequency_hz, uint64 rate_mhz)
{
	return rate_mhz == 0U ? 0U : (frequency_hz * 1000U) / rate_mhz;
}

uint64 chip_rate_period_ns(uint64 rate_mhz)
{
	return rate_mhz == 0U ? 0U : 1000000000000ULL / rate_mhz;
}
