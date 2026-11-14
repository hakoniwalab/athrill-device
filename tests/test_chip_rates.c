/* SPDX-License-Identifier: MIT */
/*
 * The IMU chips' output data rates and full scales from their register values
 * (src/models/chip_rates.c), every code against the datasheet values written out here by
 * hand (not from the table under test).
 */
#include "chip_rates.h"
#include "test_check.h"

int main(void)
{
	uint32 i;

	/* [ADV-CHIP-BMI055A-002.T1] BMI055 accelerometer: bandwidths 7.81..1000 Hz, data at twice
	 * the bandwidth, so 15.625..2000 Hz (in mHz); codes below 0x08 as 0x08, above 0x0F as 0x0F */
	{
		static const uint64 expected[32] = {
			15625U, 15625U, 15625U, 15625U, 15625U, 15625U, 15625U, 15625U, /* 0x00-0x07 */
			15625U, 31250U, 62500U, 125000U, 250000U, 500000U, 1000000U, 2000000U, /* 0x08-0x0F */
			2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U,
			2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U, 2000000U };
		for (i = 0U; i < 32U; i++) {
			TEST_CHECK(chip_bmi055_accel_rate_mhz((uint8)i, 0x00U) == expected[i]);
		}
		/* data_high_bw: unfiltered 2 kHz, whatever the bandwidth (PX4) */
		TEST_CHECK(chip_bmi055_accel_rate_mhz(0x08U, 0x80U) == 2000000U);
		/* ArduPilot: PMU_BW 0x0F (1000 Hz bandwidth) and data_high_bw */
		TEST_CHECK(chip_bmi055_accel_rate_mhz(0x0FU, 0x80U) == 2000000U);
		/* the upper bits of PMU_BW are not part of the code */
		TEST_CHECK(chip_bmi055_accel_rate_mhz(0xEAU, 0x00U) == 62500U);
	}

	/* [ADV-CHIP-BMI055G-002.T1] BMI055 gyroscope: BW 0..7 -> 2000, 2000, 1000, 400, 200, 100, 200, 100 Hz */
	{
		static const uint64 expected[8] = { 2000000U, 2000000U, 1000000U, 400000U, 200000U, 100000U, 200000U, 100000U };
		for (i = 0U; i < 8U; i++) {
			TEST_CHECK(chip_bmi055_gyro_rate_mhz((uint8)i, 0x00U) == expected[i]);
			TEST_CHECK(chip_bmi055_gyro_rate_mhz((uint8)(0x80U | i), 0x00U) == expected[i]); /* bit 7 reads 1 */
		}
		TEST_CHECK(chip_bmi055_gyro_rate_mhz(0x05U, 0x80U) == 2000000U); /* data_high_bw (PX4) */
		TEST_CHECK(chip_bmi055_gyro_rate_mhz(0x81U, 0x00U) == 2000000U); /* ArduPilot BW 0x81 */
	}

	/* [ADV-CHIP-ICM42688-002.T1] ICM-42688-P ODR codes */
	{
		static const uint64 gyro[16] = { 0U, 32000000U, 16000000U, 8000000U, 4000000U, 2000000U, 1000000U, 200000U,
			100000U, 50000U, 25000U, 12500U, 0U, 0U, 0U, 500000U };
		static const uint64 accel[16] = { 0U, 32000000U, 16000000U, 8000000U, 4000000U, 2000000U, 1000000U, 200000U,
			100000U, 50000U, 25000U, 12500U, 6250U, 3125U, 1562U, 500000U };
		for (i = 0U; i < 16U; i++) {
			TEST_CHECK(chip_icm42688_gyro_rate_mhz((uint8)i) == gyro[i]);
			TEST_CHECK(chip_icm42688_accel_rate_mhz((uint8)i) == accel[i]);
			/* the full-scale bits do not change the rate */
			TEST_CHECK(chip_icm42688_gyro_rate_mhz((uint8)(0xE0U | i)) == gyro[i]);
		}
		/* PX4: GYRO_FS_SEL_2000_DPS | GYRO_ODR_8KHZ (0x03); ArduPilot by default 0x06 (1 kHz) */
		TEST_CHECK(chip_icm42688_gyro_rate_mhz(0x03U) == 8000000U);
		TEST_CHECK(chip_icm42688_accel_rate_mhz(0x06U) == 1000000U);
	}

	/* [ADV-CHIP-ICM42688-003.T1] ICM-42688-P full scales (16-bit data) */
	{
		static const float32 dps[8] = { 16.4F, 32.8F, 65.5F, 131.0F, 262.0F, 524.3F, 1048.6F, 2097.2F };
		static const float32 g[8] = { 2048.0F, 4096.0F, 8192.0F, 16384.0F, 2048.0F, 2048.0F, 2048.0F, 2048.0F };
		for (i = 0U; i < 8U; i++) {
			TEST_CHECK(chip_icm42688_gyro_counts_per_dps((uint8)((i << 5) | 0x06U)) == dps[i]);
			TEST_CHECK(chip_icm42688_accel_counts_per_g((uint8)((i << 5) | 0x06U)) == g[i]);
		}
	}

	/* [ADV-CHIP-ICM20689-002.T1] ICM-20689: gyroscope (sample rate) and accelerometer */
	{
		/* PX4: DLPF_CFG 7, FCHOICE_B 0, ACCEL_FCHOICE_B: 8 kHz and 4 kHz */
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x47U, 0x18U, 0U) == 8000000U);
		TEST_CHECK(chip_icm20689_accel_rate_mhz(0x08U, 0U) == 4000000U);
		/* DLPF_CFG 0 also bypasses the 1 kHz path: 8 kHz */
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x00U, 0x00U, 9U) == 8000000U);
		/* FCHOICE_B 01, 10, 11: 32 kHz whatever DLPF_CFG */
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x01U, 0U) == 32000000U);
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x02U, 0U) == 32000000U);
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x03U, 0U) == 32000000U);
		/* DLPF_CFG 1..6: 1 kHz / (1 + SMPLRT_DIV): 1000, 500, 200, 100 Hz for 0, 1, 4, 9 */
		for (i = 1U; i <= 6U; i++) {
			TEST_CHECK(chip_icm20689_gyro_rate_mhz((uint8)i, 0x00U, 0U) == 1000000U);
		}
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x00U, 1U) == 500000U);
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x00U, 4U) == 200000U);
		TEST_CHECK(chip_icm20689_gyro_rate_mhz(0x03U, 0x00U, 9U) == 100000U);
		/* the accelerometer without ACCEL_FCHOICE_B: 1 kHz / (1 + SMPLRT_DIV) */
		TEST_CHECK(chip_icm20689_accel_rate_mhz(0x00U, 0U) == 1000000U);
		TEST_CHECK(chip_icm20689_accel_rate_mhz(0x07U, 4U) == 200000U);
	}

	/* periods: 2 kHz at a 400 MHz time base is 200000 clocks and 500000 ns; 15.625 Hz is 64 ms */
	TEST_CHECK(chip_rate_clocks(400000000U, 2000000U) == 200000U);
	TEST_CHECK(chip_rate_period_ns(2000000U) == 500000U);
	TEST_CHECK(chip_rate_period_ns(15625U) == 64000000U);
	TEST_CHECK(chip_rate_clocks(400000000U, 0U) == 0U && chip_rate_period_ns(0U) == 0U);
	return EXIT_SUCCESS;
}
