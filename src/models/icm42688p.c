/*
 * TDK InvenSense ICM-42688-P SPI model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"
#include "imu_filter.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_DEVICE_CONFIG	0x11U
#define REG_INT_CONFIG		0x14U
#define REG_FIFO_CONFIG		0x16U
#define REG_TEMP_DATA1		0x1DU
#define REG_ACCEL_DATA_X1	0x1FU
#define REG_GYRO_DATA_X1	0x25U
#define REG_INT_STATUS		0x2DU
#define REG_FIFO_COUNTH		0x2EU
#define REG_FIFO_COUNTL		0x2FU
#define REG_FIFO_DATA		0x30U
#define REG_SIGNAL_PATH_RESET	0x4BU
#define REG_INTF_CONFIG0	0x4CU
#define REG_PWR_MGMT0		0x4EU
#define REG_GYRO_CONFIG0	0x4FU
#define REG_ACCEL_CONFIG0	0x50U
#define REG_GYRO_CONFIG1	0x51U
#define REG_GYRO_ACCEL_CONFIG0 0x52U
#define REG_ACCEL_CONFIG1	0x53U
#define REG_FIFO_CONFIG1	0x5FU
#define REG_FIFO_CONFIG2	0x60U
#define REG_FIFO_CONFIG3	0x61U
#define REG_INT_SOURCE0		0x65U
#define REG_WHO_AM_I		0x75U
#define REG_BANK_SEL		0x76U

#define WHO_AM_I_VALUE		0x47U
#define DEVICE_SOFT_RESET	0x01U
#define RESET_DONE_INT		0x10U
#define FIFO_THS_INT		0x04U
#define FIFO_FLUSH		0x02U
#define FIFO_COUNT_REC		0x40U
#define FIFO_COUNT_ENDIAN	0x20U
#define SENSOR_DATA_ENDIAN	0x10U
#define FIFO_HIRES_EN		0x10U
#define FIFO_GYRO_EN		0x02U
#define FIFO_ACCEL_EN		0x01U
#define FIFO_THS_INT1_EN	0x04U
#define GRAVITY			9.80665F
#define RAD_TO_DEG		57.29577951F
#define FIFO_BYTES		2048U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	/* the data-ready line (ABI 1.16: GPIO lines in the runtime) */
	const AthrillDeviceResourceOps *gpio_resources;
	void *gpio_context;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint8 regs[5][128];
	std_bool yaw_270;
	uint64 clocks_per_sample;
	uint64 frequency_hz;
	/* the sample periods the value source was told last (ABI 1.22), 0: none yet */
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 reported_gyro_ns;
	uint64 reported_accel_ns;
	uint8 bank;
	std_bool started;
	std_bool reading;
	uint8 address;
	uint64 fifo_start;
	uint64 fifo_read_bytes;
	uint8 *fifo_materialized;
	uint32 fifo_materialized_bytes;
	uint32 fifo_materialized_read;
	uint8 record[20];
	uint64 next_data_ready;
	AthrillDeviceGpioLine data_ready;
	std_bool data_ready_resolved;
	std_bool ideal_filters;
	ImuFilterStream accel_filter;
	ImuFilterStream gyro_filter;
} Icm42688p;

typedef struct {
	sint32 acc[3];
	sint32 gyro[3];
	int16_t temp;
} Raw20;

static sint32 saturate(float32 v, sint32 minimum, sint32 maximum)
{
	if (v >= (float32)maximum) {
		return maximum;
	}
	if (v <= (float32)minimum) {
		return minimum;
	}
	return (sint32)((v < 0.0F) ? (v - 0.5F) : (v + 0.5F));
}

/* Firmware flips sensor y/z; undo that and the board's mounting rotation. */
static void to_chip_axes(const Icm42688p *chip, const float32 *frd, float32 *out)
{
	float32 v[3];

	if (chip->yaw_270) {
		/* yaw 270: body = (v_y, -v_x, v_z) */
		v[0] = -frd[1];
		v[1] = frd[0];
		v[2] = frd[2];
	}
	else {
		v[0] = frd[0];
		v[1] = frd[1];
		v[2] = frd[2];
	}
	out[0] = v[0];
	out[1] = -v[1];
	out[2] = -v[2];
}

static void raw_from_sample(const Icm42688p *chip,
	const AthrillDeviceSensorSample *sample, std_bool hires, Raw20 *raw)
{
	float32 acc[3];
	float32 gyro[3];
	/* [ADV-CHIP-ICM42688-003] 16-bit data at the configured full scales; the 20-bit
	 * (high resolution) FIFO data has its fixed ±16 g and ±2000 dps */
	float32 acc_scale = (hires ? 8192.0F : chip_icm42688_accel_counts_per_g(chip->regs[0][REG_ACCEL_CONFIG0])) / GRAVITY;
	float32 gyro_scale = (hires ? 131.0F : chip_icm42688_gyro_counts_per_dps(chip->regs[0][REG_GYRO_CONFIG0])) * RAD_TO_DEG;
	uint32 k;

	to_chip_axes(chip, sample->acceleration, acc);
	to_chip_axes(chip, sample->angular_velocity, gyro);
	for (k = 0U; k < 3U; k++) {
		raw->acc[k] = saturate(acc[k] * acc_scale, hires ? -131072 : -32768, hires ? 131071 : 32767);
		raw->gyro[k] = saturate(gyro[k] * gyro_scale, hires ? -262144 : -32768, hires ? 262143 : 32767);
	}
	raw->temp = (int16_t)saturate((sample->temperature_c - 25.0F) *
		(hires ? 132.48F : 2.0F), -32768, 32767);
}

static void filtered_sample(Icm42688p *chip, uint64 time,
	std_bool hires, Raw20 *raw)
{
	/* [ADV-IMU-FILTER-001] The DT property bypasses every modeled stage. */
	AthrillDeviceSensorSample sample = {0};
	float32 value[3];
	imu_filter_source_sample(&chip->source, time, &sample);
	if (!chip->ideal_filters) {
		(void)imu_filter_sample(&chip->accel_filter, &chip->source,
			time, IMU_FILTER_ACCEL, value);
		memcpy(sample.acceleration, value, sizeof(value));
		(void)imu_filter_sample(&chip->gyro_filter, &chip->source,
			time, IMU_FILTER_GYRO, value);
		memcpy(sample.angular_velocity, value, sizeof(value));
	}
	raw_from_sample(chip, &sample, hires, raw);
}

static std_bool sampling(const Icm42688p *chip)
{
	return (chip->regs[0][REG_PWR_MGMT0] & 0x0FU) != 0U;
}

static std_bool fifo_on(const Icm42688p *chip)
{
	return sampling(chip)
		&& ((chip->regs[0][REG_FIFO_CONFIG1] & (FIFO_ACCEL_EN | FIFO_GYRO_EN))
			== (FIFO_ACCEL_EN | FIFO_GYRO_EN));
}

static uint32 record_bytes(const Icm42688p *chip)
{
	return (chip->regs[0][REG_FIFO_CONFIG1] & FIFO_HIRES_EN) != 0U ? 20U : 16U;
}

static uint64 time_now(const Icm42688p *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

static void fifo_restart(Icm42688p *chip)
{
	chip->fifo_start = time_now(chip);
	chip->fifo_read_bytes = 0U;
	chip->fifo_materialized_bytes = 0U;
	chip->fifo_materialized_read = 0U;
	chip->next_data_ready = 0U;
}

static uint64 lazy_bytes(const Icm42688p *chip)
{
	uint64 made_records;
	uint32 size;

	if (!fifo_on(chip)) {
		return 0U;
	}
	size = record_bytes(chip);
	made_records = (time_now(chip) - chip->fifo_start) /
		chip->clocks_per_sample;
	return made_records * size > chip->fifo_read_bytes ?
		made_records * size - chip->fifo_read_bytes : 0U;
}

/* Stream-to-FIFO mode overwrites the oldest packet first. Materialized bytes
 * represent the oldest packets and are therefore discarded before lazy ones. */
static void discard_overflow(Icm42688p *chip)
{
	uint32 size = record_bytes(chip);
	uint64 saved = chip->fifo_materialized_bytes -
		chip->fifo_materialized_read;
	uint64 lazy = lazy_bytes(chip);
	uint64 capacity = (FIFO_BYTES / size) * size;
	uint64 drop = saved + lazy > capacity ? saved + lazy - capacity : 0U;
	uint64 from_saved = drop < saved ? drop : saved;
	chip->fifo_materialized_read += (uint32)from_saved;
	drop -= from_saved;
	chip->fifo_read_bytes += drop;
}

static uint32 fifo_bytes(Icm42688p *chip)
{
	uint64 available;
	if (!fifo_on(chip)) {
		return 0U;
	}
	discard_overflow(chip);
	available = chip->fifo_materialized_bytes -
		chip->fifo_materialized_read + lazy_bytes(chip);
	return (uint32)available;
}

static uint16 fifo_count(Icm42688p *chip)
{
	uint32 bytes = fifo_bytes(chip);

	return (chip->regs[0][REG_INTF_CONFIG0] & FIFO_COUNT_REC) != 0U
		? (uint16)(bytes / record_bytes(chip)) : (uint16)bytes;
}

static void put16(uint8 *out, uint32 offset, sint32 value, std_bool big_endian)
{
	uint16 v = (uint16)value;

	out[offset + (big_endian ? 0U : 1U)] = (uint8)(v >> 8);
	out[offset + (big_endian ? 1U : 0U)] = (uint8)v;
}

/* [ADV-FIFO-001] [ADV-FIFO-003] [ADV-FIFO-004] frame at its own time. */
static void make_record(Icm42688p *chip, uint64 time)
{
	std_bool hires = (chip->regs[0][REG_FIFO_CONFIG1] & FIFO_HIRES_EN) != 0U;
	std_bool big_endian = (chip->regs[0][REG_INTF_CONFIG0] & SENSOR_DATA_ENDIAN) != 0U;
	Raw20 raw;
	uint32 k;

	filtered_sample(chip, time, hires, &raw);
	if (hires) {
		chip->record[0] = 0x78U; /* accel + gyro + 20-bit + ODR timestamp */
		for (k = 0U; k < 3U; k++) {
			uint32 acc = (uint32)(raw.acc[k] * 4) & 0xFFFFFU;
			uint32 gyro = (uint32)(raw.gyro[k] * 2) & 0xFFFFFU;

			chip->record[1U + (2U * k)] = (uint8)(acc >> 12);
			chip->record[2U + (2U * k)] = (uint8)(acc >> 4);
			chip->record[7U + (2U * k)] = (uint8)(gyro >> 12);
			chip->record[8U + (2U * k)] = (uint8)(gyro >> 4);
			/* PX4 consumes the low nibble for both paths; gyro is the denser signal. */
			chip->record[17U + k] = (uint8)((acc & 0x0FU) << 4) | (uint8)(gyro & 0x0FU);
		}
		chip->record[13] = (uint8)((uint16)raw.temp >> 8);
		chip->record[14] = (uint8)raw.temp;
		/* the ODR timestamp: 7 for one 8 kHz period, in proportion to the period */
		{
			uint64 period_ns = chip_rate_period_ns(chip_icm42688_gyro_rate_mhz(chip->regs[0][REG_GYRO_CONFIG0]));
			uint64 stamp = (period_ns * 7U + 62500U) / 125000U;
			chip->record[15] = (uint8)(stamp >> 8);
			chip->record[16] = (uint8)stamp;
		}
	}
	else {
		chip->record[0] = 0x68U; /* accel + gyro + ODR timestamp */
		for (k = 0U; k < 3U; k++) {
			put16(chip->record, 1U + (2U * k), raw.acc[k], big_endian);
			put16(chip->record, 7U + (2U * k), raw.gyro[k], big_endian);
		}
		chip->record[13] = (uint8)raw.temp;
		/* the ODR timestamp in us: 125 for one 8 kHz period */
		put16(chip->record, 14U, (sint32)(chip_rate_period_ns(
			chip_icm42688_gyro_rate_mhz(chip->regs[0][REG_GYRO_CONFIG0])) / 1000U), big_endian);
	}
}

/* [ADV-FIFO-008] Freeze every unread packet before its filter or ODR changes. */
static void materialize_fifo(Icm42688p *chip)
{
	uint32 size = record_bytes(chip);
	uint32 saved;
	uint64 lazy;
	uint64 record;
	uint32 i;
	if (!fifo_on(chip) || fifo_bytes(chip) == 0U) {
		return;
	}
	saved = chip->fifo_materialized_bytes -
		chip->fifo_materialized_read;
	if (saved != 0U && chip->fifo_materialized_read != 0U) {
		memmove(chip->fifo_materialized,
			&chip->fifo_materialized[chip->fifo_materialized_read], saved);
	}
	lazy = lazy_bytes(chip);
	for (i = 0U; i < (uint32)(lazy / size); i++) {
		record = chip->fifo_read_bytes / size + i;
		make_record(chip, chip->fifo_start +
			(record + 1U) * chip->clocks_per_sample);
		memcpy(&chip->fifo_materialized[saved + i * size],
			chip->record, size);
	}
	chip->fifo_materialized_bytes = saved + (uint32)lazy;
	chip->fifo_materialized_read = 0U;
	chip->fifo_start = time_now(chip);
	chip->fifo_read_bytes = 0U;
	chip->next_data_ready = 0U;
}

static uint8 fifo_byte(Icm42688p *chip)
{
	uint32 size = record_bytes(chip);
	uint32 pos;

	if (fifo_bytes(chip) == 0U) {
		return 0U;
	}
	discard_overflow(chip);
	if (chip->fifo_materialized_read < chip->fifo_materialized_bytes) {
		pos = chip->fifo_materialized_read % size;
		return chip->fifo_materialized[chip->fifo_materialized_read++];
	}
	pos = (uint32)(chip->fifo_read_bytes % size);
	if (pos == 0U) {
		uint64 record = chip->fifo_read_bytes / size;
		make_record(chip, chip->fifo_start +
			(record + 1U) * chip->clocks_per_sample);
	}
	chip->fifo_read_bytes++;
	return chip->record[pos];
}

static uint8 direct_data(Icm42688p *chip, uint8 address)
{
	Raw20 raw;
	std_bool big_endian = (chip->regs[0][REG_INTF_CONFIG0] & SENSOR_DATA_ENDIAN) != 0U;
	sint32 value;
	uint32 offset;

	filtered_sample(chip, time_now(chip), FALSE, &raw);
	if (address < REG_ACCEL_DATA_X1) {
		value = raw.temp;
		offset = address - REG_TEMP_DATA1;
	}
	else if (address < REG_GYRO_DATA_X1) {
		offset = address - REG_ACCEL_DATA_X1;
		value = raw.acc[offset / 2U];
	}
	else {
		offset = address - REG_GYRO_DATA_X1;
		value = raw.gyro[offset / 2U];
	}
	return (((offset & 1U) == 0U) == big_endian) ? (uint8)((uint16)value >> 8) : (uint8)value;
}

/* [ADV-CHIP-ICM42688-001] expose WHO_AM_I value 0x47 at register 0x75 */
static uint8 read_register(Icm42688p *chip, uint8 address)
{
	uint16 count;
	std_bool count_big;

	if (address == REG_BANK_SEL) {
		return chip->bank;
	}
	if (chip->bank != 0U) {
		return chip->regs[chip->bank][address];
	}
	if (address == REG_WHO_AM_I) {
		return WHO_AM_I_VALUE;
	}
	if ((address >= REG_TEMP_DATA1) && (address <= (REG_GYRO_DATA_X1 + 5U))) {
		return direct_data(chip, address);
	}
	if ((address == REG_FIFO_COUNTH) || (address == REG_FIFO_COUNTL)) {
		count = fifo_count(chip);
		count_big = (chip->regs[0][REG_INTF_CONFIG0] & FIFO_COUNT_ENDIAN) != 0U;
		return ((address == REG_FIFO_COUNTH) == count_big) ? (uint8)(count >> 8) : (uint8)count;
	}
	if (address == REG_FIFO_DATA) {
		return fifo_byte(chip);
	}
	if (address == REG_INT_STATUS) {
		uint8 status = chip->regs[0][REG_INT_STATUS];
		uint32 threshold = chip->regs[0][REG_FIFO_CONFIG2]
			| ((uint32)(chip->regs[0][REG_FIFO_CONFIG3] & 0x0FU) << 8);

		if ((threshold != 0U) && (fifo_bytes(chip) >= threshold)) {
			status |= FIFO_THS_INT;
		}
		return status;
	}
	return chip->regs[0][address];
}

static void reset_registers(Icm42688p *chip)
{
	uint32 bank;
	uint32 reg;

	for (bank = 0U; bank < 5U; bank++) {
		for (reg = 0U; reg < 128U; reg++) {
			chip->regs[bank][reg] = 0U;
		}
	}
	chip->bank = 0U;
	chip->regs[0][REG_WHO_AM_I] = WHO_AM_I_VALUE;
	chip->regs[0][REG_INT_STATUS] = RESET_DONE_INT;
	/* reset values: ±2000 dps and ±16 g at 1 kHz */
	chip->regs[0][REG_GYRO_CONFIG0] = 0x06U;
	chip->regs[0][REG_ACCEL_CONFIG0] = 0x06U;
	chip->regs[0][REG_GYRO_CONFIG1] = 0x16U;
	chip->regs[0][REG_GYRO_ACCEL_CONFIG0] = 0x11U;
	chip->regs[0][REG_ACCEL_CONFIG1] = 0x0DU;
	chip->regs[1][0x0BU] = 0xA0U;
	chip->regs[1][0x0CU] = 0x0DU;
	chip->regs[1][0x0DU] = 0xAAU;
	chip->regs[1][0x0EU] = 0x80U;
	chip->regs[2][0x03U] = 0x30U;
	chip->regs[2][0x04U] = 0x40U;
	chip->regs[2][0x05U] = 0x62U;
}

/* [ADV-CHIP-ICM42688-002] FIFO records come at the gyroscope's output data rate (both
 * parts are in each record). With the FIFO running, the bytes already made stay. */
static void apply_rate(Icm42688p *chip)
{
	uint64 clocks = chip_rate_clocks(chip->frequency_hz,
		chip_icm42688_gyro_rate_mhz(chip->regs[0][REG_GYRO_CONFIG0]));
	if (clocks == 0U || clocks == chip->clocks_per_sample) {
		return;
	}
	if (fifo_on(chip)) {
		chip->fifo_start = time_now(chip);
		chip->fifo_read_bytes = 0U;
		chip->next_data_ready = 0U;
	}
	chip->clocks_per_sample = clocks;
}

static float32 ui_bandwidth(uint8 code, float32 rate)
{
	static const uint8 divisor[8] = { 2U, 4U, 5U, 8U, 10U, 16U, 20U, 40U };
	float32 base;
	if (code > 7U) {
		return 0.0F;
	}
	if (code == 0U) {
		return rate * 0.5F;
	}
	base = rate > 400.0F ? rate : 400.0F;
	return base / (float32)divisor[code];
}

static float32 aaf_bandwidth(uint8 delt, uint16 deltsqr,
	uint8 bitshift)
{
	static const uint16 bandwidth[63] = {
		42U, 84U, 126U, 170U, 213U, 258U, 303U, 348U, 394U,
		441U, 488U, 536U, 585U, 634U, 684U, 734U, 785U, 837U,
		890U, 943U, 997U, 1051U, 1107U, 1163U, 1220U, 1277U,
		1336U, 1395U, 1454U, 1515U, 1577U, 1639U, 1702U, 1766U,
		1830U, 1896U, 1962U, 2029U, 2097U, 2166U, 2235U, 2306U,
		2377U, 2449U, 2522U, 2596U, 2671U, 2746U, 2823U, 2900U,
		2978U, 3057U, 3137U, 3217U, 3299U, 3381U, 3464U, 3548U,
		3633U, 3718U, 3805U, 3892U, 3979U
	};
	static const uint16 square[63] = {
		1U, 4U, 9U, 16U, 25U, 36U, 49U, 64U, 81U, 100U, 122U,
		144U, 170U, 196U, 224U, 256U, 288U, 324U, 360U, 400U,
		440U, 488U, 528U, 576U, 624U, 680U, 736U, 784U, 848U,
		896U, 960U, 1024U, 1088U, 1152U, 1232U, 1296U, 1376U,
		1440U, 1536U, 1600U, 1696U, 1760U, 1856U, 1952U, 2016U,
		2112U, 2208U, 2304U, 2400U, 2496U, 2592U, 2720U, 2816U,
		2944U, 3008U, 3136U, 3264U, 3392U, 3456U, 3584U, 3712U,
		3840U, 3968U
	};
	static const uint8 shift[63] = {
		15U, 13U, 12U, 11U, 10U, 10U, 9U, 9U, 9U, 8U, 8U, 8U,
		8U, 7U, 7U, 7U, 7U, 7U, 6U, 6U, 6U, 6U, 6U, 6U, 6U,
		6U, 5U, 5U, 5U, 5U, 5U, 5U, 5U, 5U, 5U, 5U, 4U, 4U,
		4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U,
		4U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U
	};
	uint32 index;
	if (delt == 0U || delt > 63U) {
		return 0.0F;
	}
	index = (uint32)delt - 1U;
	if (square[index] != deltsqr || shift[index] != bitshift) {
		return 0.0F;
	}
	return (float32)bandwidth[index];
}

/* [ADV-CHIP-ICM42688-004] AAF and UI stages selected by their register fields. */
static void configure_filters(Icm42688p *chip)
{
	uint64 gyro_rate_mhz = chip_icm42688_gyro_rate_mhz(
		chip->regs[0][REG_GYRO_CONFIG0]);
	uint64 accel_rate_mhz = chip_icm42688_accel_rate_mhz(
		chip->regs[0][REG_ACCEL_CONFIG0]);
	float32 gyro_rate = (float32)gyro_rate_mhz / 1000.0F;
	float32 accel_rate = (float32)accel_rate_mhz / 1000.0F;
	uint8 gyro_orders[2];
	uint8 accel_orders[2];
	float32 gyro_bandwidths[2];
	float32 accel_bandwidths[2];
	uint16 square;
	uint64 now = time_now(chip);
	uint8 code;

	square = (uint16)chip->regs[1][0x0DU] |
		((uint16)(chip->regs[1][0x0EU] & 0x0FU) << 8);
	gyro_bandwidths[0] = aaf_bandwidth(chip->regs[1][0x0CU] & 0x3FU,
		square, chip->regs[1][0x0EU] >> 4);
	gyro_orders[0] = (chip->regs[1][0x0BU] & 0x02U) == 0U &&
		gyro_bandwidths[0] > 0.0F ? 2U : 0U;
	code = (chip->regs[0][REG_GYRO_CONFIG1] >> 2) & 3U;
	gyro_orders[1] = code < 3U ? (uint8)(code + 1U) : 0U;
	code = chip->regs[0][REG_GYRO_ACCEL_CONFIG0] & 0x0FU;
	gyro_bandwidths[1] = ui_bandwidth(code, gyro_rate);
	if (code > 7U) {
		gyro_orders[1] = 0U;
	}

	square = (uint16)chip->regs[2][0x04U] |
		((uint16)(chip->regs[2][0x05U] & 0x0FU) << 8);
	accel_bandwidths[0] = aaf_bandwidth(
		(chip->regs[2][0x03U] >> 1) & 0x3FU,
		square, chip->regs[2][0x05U] >> 4);
	accel_orders[0] = (chip->regs[2][0x03U] & 0x01U) == 0U &&
		accel_bandwidths[0] > 0.0F ? 2U : 0U;
	code = (chip->regs[0][REG_ACCEL_CONFIG1] >> 3) & 3U;
	accel_orders[1] = code < 3U ? (uint8)(code + 1U) : 0U;
	code = chip->regs[0][REG_GYRO_ACCEL_CONFIG0] >> 4;
	accel_bandwidths[1] = ui_bandwidth(code, accel_rate);
	if (code > 7U) {
		accel_orders[1] = 0U;
	}

	imu_filter_configure(&chip->gyro_filter, now,
		chip_rate_clocks(chip->frequency_hz, gyro_rate_mhz), gyro_rate,
		gyro_orders, gyro_bandwidths, 2U);
	imu_filter_configure(&chip->accel_filter, now,
		chip_rate_clocks(chip->frequency_hz, accel_rate_mhz), accel_rate,
		accel_orders, accel_bandwidths, 2U);
}

/* The value source learns each part's period once the FIFO runs, and each change. */
static void report_periods(Icm42688p *chip)
{
	uint64 gyro = chip_rate_period_ns(chip_icm42688_gyro_rate_mhz(chip->regs[0][REG_GYRO_CONFIG0]));
	uint64 accel = chip_rate_period_ns(chip_icm42688_accel_rate_mhz(chip->regs[0][REG_ACCEL_CONFIG0]));
	if (chip->period_ops == NULL) {
		return;
	}
	if (gyro != chip->reported_gyro_ns) {
		chip->reported_gyro_ns = gyro;
		(void)chip->period_ops->sample_period(chip->period_instance, ATHRILL_DEVICE_SAMPLE_GYRO, gyro);
	}
	if (accel != chip->reported_accel_ns) {
		chip->reported_accel_ns = accel;
		(void)chip->period_ops->sample_period(chip->period_instance, ATHRILL_DEVICE_SAMPLE_ACCEL, accel);
	}
}

static void write_register(Icm42688p *chip, uint8 address, uint8 value)
{
	std_bool was_on = fifo_on(chip);

	if (address == REG_BANK_SEL) {
		chip->bank = value & 7U;
		if (chip->bank > 4U) {
			chip->bank = 0U;
		}
		return;
	}
	if (chip->bank != 0U) {
		if (((chip->bank == 1U && address >= 0x0BU && address <= 0x0EU) ||
		     (chip->bank == 2U && address >= 0x03U && address <= 0x05U)) &&
		    chip->regs[chip->bank][address] != value) {
			materialize_fifo(chip);
		}
		chip->regs[chip->bank][address] = value;
		if ((chip->bank == 1U && address >= 0x0BU && address <= 0x0EU) ||
		    (chip->bank == 2U && address >= 0x03U && address <= 0x05U)) {
			configure_filters(chip);
		}
		return;
	}
	if (address == REG_DEVICE_CONFIG && (value & DEVICE_SOFT_RESET) != 0U) {
		reset_registers(chip);
		apply_rate(chip);
		configure_filters(chip);
		fifo_restart(chip);
		return;
	}
	if (address == REG_SIGNAL_PATH_RESET) {
		if ((value & FIFO_FLUSH) != 0U) {
			fifo_restart(chip);
		}
		chip->regs[0][address] = 0U;
		return;
	}
	if ((address == REG_WHO_AM_I) || (address == REG_INT_STATUS)
		|| (address == REG_FIFO_COUNTH) || (address == REG_FIFO_COUNTL)
		|| (address == REG_FIFO_DATA)) {
		return;
	}
	if ((address == REG_GYRO_CONFIG0 || address == REG_ACCEL_CONFIG0 ||
	     address == REG_GYRO_CONFIG1 ||
	     address == REG_GYRO_ACCEL_CONFIG0 ||
	     address == REG_ACCEL_CONFIG1) &&
	    chip->regs[0][address] != value) {
		materialize_fifo(chip);
	}
	chip->regs[0][address] = value;
	if (address == REG_GYRO_CONFIG0) {
		apply_rate(chip);
	}
	if (address == REG_GYRO_CONFIG0 || address == REG_ACCEL_CONFIG0 ||
	    address == REG_GYRO_CONFIG1 ||
	    address == REG_GYRO_ACCEL_CONFIG0 ||
	    address == REG_ACCEL_CONFIG1) {
		configure_filters(chip);
	}
	if (!was_on && fifo_on(chip)) {
		fifo_restart(chip);
	}
	/* the chip produces samples once its FIFO runs, with or without its interrupt (ArduPilot
	 * polls the FIFO): from then on, the value source learns the periods and each change */
	if (fifo_on(chip)) {
		report_periods(chip);
	}
}

static void chip_select(void *p, std_bool selected)
{
	Icm42688p *chip = (Icm42688p *)p;

	(void)selected;
	chip->started = FALSE;
}

static uint8 chip_exchange(void *p, uint8 mosi)
{
	Icm42688p *chip = (Icm42688p *)p;
	uint8 miso = 0U;

	if (!chip->started) {
		chip->started = TRUE;
		chip->reading = (mosi & 0x80U) != 0U;
		chip->address = mosi & 0x7FU;
		return 0U;
	}
	if (chip->reading) {
		miso = read_register(chip, chip->address);
	}
	else {
		write_register(chip, chip->address, mosi);
	}
	if (chip->address != REG_FIFO_DATA) {
		chip->address = (chip->address + 1U) & 0x7FU;
	}
	return miso;
}

static sint32 reset(void *instance)
{
	Icm42688p *chip = instance;
	chip->started = FALSE;
	chip->reading = FALSE;
	chip->address = 0U;
	reset_registers(chip);
	apply_rate(chip);
	configure_filters(chip);
	fifo_restart(chip);
	if (chip->data_ready_resolved) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, TRUE);
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Icm42688p *chip = calloc(1U, sizeof(*chip) + FIFO_BYTES);
	const char *rotation;
	const void *ideal;
	uint32 ideal_size;
	uint64 frequency;
	if (chip == NULL || args->target_ops->time_frequency_hz == NULL ||
	    (frequency = args->target_ops->time_frequency_hz(args->target)) == 0U) {
		free(chip);
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	chip->fifo_materialized = (uint8 *)(chip + 1);
	chip->frequency_hz = frequency;
	reset_registers(chip);
	chip->clocks_per_sample = chip_rate_clocks(frequency,
		chip_icm42688_gyro_rate_mhz(chip->regs[0][REG_GYRO_CONFIG0]));
	chip->ideal_filters = args->node->ops->get_property(
		args->node->context, "athrill,ideal-filters", &ideal,
		&ideal_size) == 0;
	configure_filters(chip);
	if (args->node->ops->get_string(args->node->context,
		"athrill,mount-rotation", 0U, &rotation) == 0 &&
	    strcmp(rotation, "yaw-270") == 0) {
		chip->yaw_270 = TRUE;
	}
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Icm42688p *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0 ||
	    args->resources->get_gpio(args->context, "int-gpios", 0U,
		&chip->data_ready) != 0) {
		return -1;
	}
	chip->gpio_resources = args->resources;
	chip->gpio_context = args->context;
	chip->data_ready_resolved = TRUE;
	/* ABI 1.22: a value source that takes sample periods (optional) */
	if (args->resources->struct_size >= offsetof(AthrillDeviceResourceOps, get_sample_period) +
	    sizeof(args->resources->get_sample_period) && args->resources->get_sample_period != NULL &&
	    args->resources->get_sample_period(args->context, "value-source", &chip->period_ops,
		&chip->period_instance) != 0) {
		chip->period_ops = NULL;
	}
	(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready, TRUE);
	return 0;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	Icm42688p *chip = instance;
	uint32 threshold = chip->regs[0][REG_FIFO_CONFIG2]
		| ((uint32)(chip->regs[0][REG_FIFO_CONFIG3] & 0x0FU) << 8);
	uint32 samples;
	uint64 period;
	std_bool active_low = (chip->regs[0][REG_INT_CONFIG] & 1U) == 0U;

	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
		chip->data_ready, active_low);
	if (!fifo_on(chip) || ((chip->regs[0][REG_INT_SOURCE0] & FIFO_THS_INT1_EN) == 0U)) {
		return 0;
	}
	report_periods(chip);
	if (threshold == 0U) {
		threshold = record_bytes(chip);
	}
	samples = (threshold + record_bytes(chip) - 1U) / record_bytes(chip);
	period = chip->clocks_per_sample * samples;
	if (chip->next_data_ready == 0U) {
		chip->next_data_ready = (now / period + 1U) * period;
	}
	if (now >= chip->next_data_ready) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, !active_low);
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, active_low);
		chip->next_data_ready += period *
			(1U + (now - chip->next_data_ready) / period);
	}
	result->next_event_interval = chip->next_data_ready - now;
	return 0;
}

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), chip_select, chip_exchange
};

const AthrillDeviceModelOps athrill_device_icm42688p_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "icm42688p", "1", NULL,
	create, free, NULL, &spi_ops, NULL, NULL, connect_model, reset, tick
};
