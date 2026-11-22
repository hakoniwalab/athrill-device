/*
 * TDK InvenSense ICM-45686 SPI model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include "chip_rates.h"
#include "imu_filter.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_ACCEL_DATA_X1         0x00U
#define REG_GYRO_DATA_X1          0x06U
#define REG_TEMP_DATA1            0x0cU
#define REG_PWR_MGMT0             0x10U
#define REG_FIFO_COUNT_0          0x12U
#define REG_FIFO_COUNT_1          0x13U
#define REG_FIFO_DATA             0x14U
#define REG_INT1_CONFIG0          0x16U
#define REG_INT1_CONFIG1          0x17U
#define REG_INT1_CONFIG2          0x18U
#define REG_INT1_STATUS0          0x19U
#define REG_INT1_STATUS1          0x1aU
#define REG_ACCEL_CONFIG0         0x1bU
#define REG_GYRO_CONFIG0          0x1cU
#define REG_FIFO_CONFIG0          0x1dU
#define REG_FIFO_CONFIG1_0        0x1eU
#define REG_FIFO_CONFIG1_1        0x1fU
#define REG_FIFO_CONFIG2          0x20U
#define REG_FIFO_CONFIG3          0x21U
#define REG_FIFO_CONFIG4          0x22U
#define REG_TMST_WOM_CONFIG       0x23U
#define REG_RTC_CONFIG            0x26U
#define REG_INTF_CONFIG0          0x2cU
#define REG_IOC_PAD_SCENARIO_OVRD 0x31U
#define REG_WHO_AM_I              0x72U
#define REG_MISC2                 0x7fU

#define WHO_AM_I_VALUE            0xe9U
#define SOFT_RESET                0x02U
#define IREG_DONE                 0x01U
#define PWR_GYRO_MODE_MASK        0x0cU
#define PWR_ACCEL_MODE_MASK       0x03U
#define FIFO_MODE_MASK            0xc0U
#define FIFO_MODE_STREAM          0x40U
#define FIFO_MODE_STOP_ON_FULL    0x80U
#define FIFO_DEPTH_MASK           0x3fU
#define FIFO_DEPTH_2K             0x07U
#define FIFO_DEPTH_8K             0x1fU
#define FIFO_FLUSH                0x80U
#define FIFO_WR_WM_GT_TH          0x08U
#define FIFO_HIRES_EN             0x08U
#define FIFO_GYRO_EN              0x04U
#define FIFO_ACCEL_EN             0x02U
#define FIFO_IF_EN                0x01U
#define FIFO_COMP_EN              0x04U
#define INT_RESET_DONE            0x80U
#define INT_DRDY                  0x04U
#define INT_FIFO_THS              0x02U
#define INT_FIFO_FULL             0x01U
#define FIFO_PACKET_BYTES         20U
#define FIFO_BYTES_MAX            8192U
#define FIFO_PACKETS_MAX          (FIFO_BYTES_MAX / FIFO_PACKET_BYTES)
#define GRAVITY                   9.80665F
#define RAD_TO_DEG                57.29577951F
#define ACCEL_HIRES_COUNTS_PER_G  16384.0F
#define GYRO_HIRES_COUNTS_PER_DPS 131.1F
#define TEMP_HIRES_COUNTS_PER_C   132.48F

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	const AthrillDeviceResourceOps *gpio_resources;
	void *gpio_context;
	AthrillDeviceGpioLine data_ready;
	std_bool data_ready_resolved;
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 frequency_hz;
	uint64 clocks_per_sample;
	uint64 next_sample_time;
	uint64 reported_gyro_ns;
	uint64 reported_accel_ns;
	uint8 registers[128];
	uint8 fifo[FIFO_PACKETS_MAX][FIFO_PACKET_BYTES];
	uint16 fifo_head;
	uint16 fifo_count;
	uint8 fifo_offset;
	uint8 interrupt_pending;
	std_bool yaw_270;
	std_bool ideal_filters;
	ImuFilterStream accel_filter;
	ImuFilterStream gyro_filter;
	std_bool started;
	std_bool reading;
	uint8 address;
} Icm45686;

typedef struct {
	sint32 accel[3];
	sint32 gyro[3];
	int16_t temperature;
} RawSample;

static uint64 time_now(const Icm45686 *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

static uint64 odr_mhz(uint8 config)
{
	static const uint64 rates[16] = {
		0U, 0U, 0U, 6400000U, 3200000U, 1600000U, 800000U,
		400000U, 200000U, 100000U, 50000U, 25000U, 12500U,
		6250U, 3125U, 1562U
	};
	return rates[config & 0x0fU];
}

static sint32 saturate(float32 value, sint32 minimum, sint32 maximum)
{
	if (value >= (float32)maximum) {
		return maximum;
	}
	if (value <= (float32)minimum) {
		return minimum;
	}
	return (sint32)(value < 0.0F ? value - 0.5F : value + 0.5F);
}

/* [ADV-CHIP-ICM45686-008] Firmware converts the chip's FLU axes to FRD;
 * undo that conversion and the one mount rotation currently supported by
 * the ICM-42688-P model. */
static void to_chip_axes(const Icm45686 *chip, const float32 *frd,
	float32 *out)
{
	float32 mounted[3];
	if (chip->yaw_270) {
		mounted[0] = -frd[1];
		mounted[1] = frd[0];
		mounted[2] = frd[2];
	}
	else {
		mounted[0] = frd[0];
		mounted[1] = frd[1];
		mounted[2] = frd[2];
	}
	out[0] = mounted[0];
	out[1] = -mounted[1];
	out[2] = -mounted[2];
}

/* [ADV-CHIP-ICM45686-008] DS-000577 sections 21.7, 21.11, 22.7 and
 * 22.10 reset both FIR source controls off and both UI LPFs to bypass. */
static void configure_filters(Icm45686 *chip)
{
	uint8 orders[1] = { 0U };
	float32 bandwidths[1] = { 0.0F };
	uint64 gyro_rate = odr_mhz(chip->registers[REG_GYRO_CONFIG0]);
	uint64 accel_rate = odr_mhz(chip->registers[REG_ACCEL_CONFIG0]);
	uint64 now = time_now(chip);
	imu_filter_configure(&chip->gyro_filter, now,
		chip_rate_clocks(chip->frequency_hz, gyro_rate),
		(float32)gyro_rate / 1000.0F, orders, bandwidths, 1U);
	imu_filter_configure(&chip->accel_filter, now,
		chip_rate_clocks(chip->frequency_hz, accel_rate),
		(float32)accel_rate / 1000.0F, orders, bandwidths, 1U);
}

static std_bool filter_active(const ImuFilterStream *stream)
{
	uint32 stage;
	for (stage = 0U; stage < stream->stage_count; stage++) {
		if (stream->stages[stage].count != 0U) {
			return TRUE;
		}
	}
	return FALSE;
}

static void filtered_sample(Icm45686 *chip, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	float32 values[3];
	imu_filter_source_sample(&chip->source, time, sample);
	if (chip->ideal_filters) {
		return;
	}
	if (filter_active(&chip->accel_filter)) {
		(void)imu_filter_sample(&chip->accel_filter, &chip->source, time,
			IMU_FILTER_ACCEL, values);
		memcpy(sample->acceleration, values, sizeof(values));
	}
	if (filter_active(&chip->gyro_filter)) {
		(void)imu_filter_sample(&chip->gyro_filter, &chip->source, time,
			IMU_FILTER_GYRO, values);
		memcpy(sample->angular_velocity, values, sizeof(values));
	}
}

static void raw_from_sample(const Icm45686 *chip,
	const AthrillDeviceSensorSample *sample, RawSample *raw)
{
	float32 acceleration[3];
	float32 angular_velocity[3];
	uint32 axis;
	to_chip_axes(chip, sample->acceleration, acceleration);
	to_chip_axes(chip, sample->angular_velocity, angular_velocity);
	for (axis = 0U; axis < 3U; axis++) {
		raw->accel[axis] = 2 * saturate(acceleration[axis] *
			(ACCEL_HIRES_COUNTS_PER_G * 0.5F) / GRAVITY,
			-262144, 262143);
		raw->gyro[axis] = saturate(angular_velocity[axis] * RAD_TO_DEG *
			GYRO_HIRES_COUNTS_PER_DPS, -524288, 524287);
	}
	raw->temperature = (int16_t)saturate((sample->temperature_c - 25.0F) *
		TEMP_HIRES_COUNTS_PER_C, -32768, 32767);
}

/* [ADV-CHIP-ICM45686-004] DS-000577 sections 6.1, 6.2 and 15: a
 * little-endian 20-byte packet with datasheet-correct extension nibbles. */
static void make_packet(Icm45686 *chip, uint8 *packet, uint64 time)
{
	AthrillDeviceSensorSample sample = {0};
	RawSample raw;
	uint32 axis;
	filtered_sample(chip, time, &sample);
	raw_from_sample(chip, &sample, &raw);
	memset(packet, 0, FIFO_PACKET_BYTES);
	packet[0] = 0x78U;
	for (axis = 0U; axis < 3U; axis++) {
		uint32 accel = (uint32)raw.accel[axis] & 0xfffffU;
		uint32 gyro = (uint32)raw.gyro[axis] & 0xfffffU;
		packet[1U + 2U * axis] = (uint8)(accel >> 4);
		packet[2U + 2U * axis] = (uint8)(accel >> 12);
		packet[7U + 2U * axis] = (uint8)(gyro >> 4);
		packet[8U + 2U * axis] = (uint8)(gyro >> 12);
		packet[17U + axis] = (uint8)((accel & 0x0fU) << 4) |
			(uint8)(gyro & 0x0fU);
	}
	packet[13] = (uint8)raw.temperature;
	packet[14] = (uint8)((uint16)raw.temperature >> 8);
	/* PX4 leaves FIFO_TMST_FSYNC_EN clear. DS-000577 section 17.33 then
	 * requires the structurally present timestamp field to be 0x0000. */
}

static std_bool sensors_on(const Icm45686 *chip)
{
	return (chip->registers[REG_PWR_MGMT0] & PWR_GYRO_MODE_MASK) != 0U &&
		(chip->registers[REG_PWR_MGMT0] & PWR_ACCEL_MODE_MASK) != 0U;
}

static std_bool fifo_on(const Icm45686 *chip)
{
	uint8 mode = chip->registers[REG_FIFO_CONFIG0] & FIFO_MODE_MASK;
	uint8 enabled = FIFO_HIRES_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN |
		FIFO_IF_EN;
	return sensors_on(chip) &&
		(mode == FIFO_MODE_STREAM || mode == FIFO_MODE_STOP_ON_FULL) &&
		(chip->registers[REG_FIFO_CONFIG3] & enabled) == enabled &&
		(chip->registers[REG_FIFO_CONFIG4] & FIFO_COMP_EN) == 0U &&
		chip->clocks_per_sample != 0U;
}

static uint16 fifo_capacity(const Icm45686 *chip)
{
	uint8 depth = chip->registers[REG_FIFO_CONFIG0] & FIFO_DEPTH_MASK;
	return depth == FIFO_DEPTH_8K ? FIFO_PACKETS_MAX :
		(uint16)(2048U / FIFO_PACKET_BYTES);
}

static void clear_fifo(Icm45686 *chip)
{
	chip->fifo_head = 0U;
	chip->fifo_count = 0U;
	chip->fifo_offset = 0U;
	chip->registers[REG_INT1_STATUS0] &=
		(uint8)~(INT_FIFO_THS | INT_FIFO_FULL);
	chip->interrupt_pending &=
		(uint8)~(INT_FIFO_THS | INT_FIFO_FULL);
	chip->next_sample_time = fifo_on(chip) ?
		time_now(chip) + chip->clocks_per_sample : 0U;
}

static void record_events(Icm45686 *chip, uint8 events)
{
	chip->registers[REG_INT1_STATUS0] |= events;
	chip->interrupt_pending |= events & chip->registers[REG_INT1_CONFIG0];
}

static void drop_oldest(Icm45686 *chip, uint16 packets)
{
	if (packets > chip->fifo_count) {
		packets = chip->fifo_count;
	}
	chip->fifo_head = (uint16)((chip->fifo_head + packets) %
		FIFO_PACKETS_MAX);
	chip->fifo_count = (uint16)(chip->fifo_count - packets);
	chip->fifo_offset = 0U;
}

static void append_packet(Icm45686 *chip, uint64 sample_time)
{
	uint16 tail = (uint16)((chip->fifo_head + chip->fifo_count) %
		FIFO_PACKETS_MAX);
	make_packet(chip, chip->fifo[tail], sample_time);
	chip->fifo_count++;
}

static uint16 watermark_bytes(const Icm45686 *chip)
{
	return (uint16)chip->registers[REG_FIFO_CONFIG1_0] |
		((uint16)chip->registers[REG_FIFO_CONFIG1_1] << 8);
}

/* [ADV-CHIP-ICM45686-003] [ADV-CHIP-ICM45686-006] Materialize complete
 * packets at 6.4 kHz and implement stream or stop-on-full at packet depth. */
static void update_fifo(Icm45686 *chip)
{
	uint64 now;
	uint64 due;
	uint64 first;
	uint64 accepted;
	uint64 index;
	uint16 capacity;
	uint16 before;
	uint16 threshold_bytes;
	uint32 before_bytes;
	uint32 after_bytes;
	uint8 events = 0U;
	std_bool stream;
	if (!fifo_on(chip)) {
		return;
	}
	now = time_now(chip);
	if (chip->next_sample_time == 0U) {
		chip->next_sample_time = now + chip->clocks_per_sample;
		return;
	}
	if (now < chip->next_sample_time) {
		return;
	}
	due = (now - chip->next_sample_time) / chip->clocks_per_sample + 1U;
	capacity = fifo_capacity(chip);
	before = chip->fifo_count;
	stream = (chip->registers[REG_FIFO_CONFIG0] & FIFO_MODE_MASK) ==
		FIFO_MODE_STREAM;
	first = 0U;
	if (stream) {
		if (due >= capacity) {
			drop_oldest(chip, chip->fifo_count);
			first = due - capacity;
		}
		else if ((uint64)chip->fifo_count + due > capacity) {
			drop_oldest(chip, (uint16)((uint64)chip->fifo_count +
				due - capacity));
		}
		accepted = due - first;
	}
	else {
		accepted = chip->fifo_count < capacity ?
			capacity - chip->fifo_count : 0U;
		if (accepted > due) {
			accepted = due;
		}
	}
	for (index = 0U; index < accepted; index++) {
		append_packet(chip, chip->next_sample_time +
			(first + index) * chip->clocks_per_sample);
	}
	chip->next_sample_time += due * chip->clocks_per_sample;
	events |= INT_DRDY;
	threshold_bytes = watermark_bytes(chip);
	before_bytes = (uint32)before * FIFO_PACKET_BYTES;
	after_bytes = (uint32)chip->fifo_count * FIFO_PACKET_BYTES;
	if (threshold_bytes != 0U) {
		if ((chip->registers[REG_FIFO_CONFIG2] & FIFO_WR_WM_GT_TH) != 0U) {
			if (after_bytes >= threshold_bytes) {
				events |= INT_FIFO_THS;
			}
		}
		else if (before_bytes < threshold_bytes &&
		    after_bytes == threshold_bytes) {
			events |= INT_FIFO_THS;
		}
	}
	if (chip->fifo_count == capacity || accepted < due) {
		events |= INT_FIFO_FULL;
	}
	record_events(chip, events);
}

static void report_periods(Icm45686 *chip)
{
	uint64 gyro = chip_rate_period_ns(
		odr_mhz(chip->registers[REG_GYRO_CONFIG0]));
	uint64 accel = chip_rate_period_ns(
		odr_mhz(chip->registers[REG_ACCEL_CONFIG0]));
	if (chip->period_ops == NULL) {
		return;
	}
	if (gyro != chip->reported_gyro_ns) {
		chip->reported_gyro_ns = gyro;
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_GYRO, gyro);
	}
	if (accel != chip->reported_accel_ns) {
		chip->reported_accel_ns = accel;
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_ACCEL, accel);
	}
}

static void apply_rate(Icm45686 *chip)
{
	uint64 clocks = chip_rate_clocks(chip->frequency_hz,
		odr_mhz(chip->registers[REG_GYRO_CONFIG0]));
	configure_filters(chip);
	if (clocks == chip->clocks_per_sample) {
		return;
	}
	chip->clocks_per_sample = clocks;
	chip->next_sample_time = fifo_on(chip) && clocks != 0U ?
		time_now(chip) + clocks : 0U;
	if (fifo_on(chip)) {
		report_periods(chip);
	}
}

static void reset_registers(Icm45686 *chip)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_INT1_CONFIG0] = 0x80U;
	chip->registers[REG_INT1_CONFIG2] = 0x04U;
	chip->registers[REG_ACCEL_CONFIG0] = 0x06U;
	chip->registers[REG_GYRO_CONFIG0] = 0x06U;
	chip->registers[REG_FIFO_CONFIG2] = 0x20U;
	chip->registers[REG_RTC_CONFIG] = 0x03U;
	chip->registers[REG_INTF_CONFIG0] = 0x9aU;
	chip->registers[REG_WHO_AM_I] = WHO_AM_I_VALUE;
	chip->registers[REG_MISC2] = IREG_DONE;
	chip->registers[REG_INT1_STATUS0] = INT_RESET_DONE;
	chip->interrupt_pending = INT_RESET_DONE;
	chip->clocks_per_sample = chip_rate_clocks(chip->frequency_hz,
		odr_mhz(chip->registers[REG_GYRO_CONFIG0]));
	configure_filters(chip);
	clear_fifo(chip);
}

static sint32 direct_raw(Icm45686 *chip, uint8 address)
{
	AthrillDeviceSensorSample sample = {0};
	float32 acceleration[3];
	float32 angular_velocity[3];
	uint32 axis;
	filtered_sample(chip, time_now(chip), &sample);
	to_chip_axes(chip, sample.acceleration, acceleration);
	to_chip_axes(chip, sample.angular_velocity, angular_velocity);
	if (address < REG_GYRO_DATA_X1) {
		static const float32 scale[8] = {
			1024.0F, 2048.0F, 4096.0F, 8192.0F,
			16384.0F, 1024.0F, 1024.0F, 1024.0F
		};
		axis = (address - REG_ACCEL_DATA_X1) / 2U;
		return saturate(acceleration[axis] *
			scale[(chip->registers[REG_ACCEL_CONFIG0] >> 4) & 7U] /
			GRAVITY, -32768, 32767);
	}
	if (address < REG_TEMP_DATA1) {
		static const float32 scale[16] = {
			8.192F, 16.384F, 32.768F, 65.536F,
			131.072F, 262.144F, 524.288F, 1048.576F,
			2097.152F, 8.192F, 8.192F, 8.192F,
			8.192F, 8.192F, 8.192F, 8.192F
		};
		axis = (address - REG_GYRO_DATA_X1) / 2U;
		return saturate(angular_velocity[axis] * RAD_TO_DEG *
			scale[chip->registers[REG_GYRO_CONFIG0] >> 4],
			-32768, 32767);
	}
	return saturate((sample.temperature_c - 25.0F) * 128.0F,
		-32768, 32767);
}

static uint8 direct_data(Icm45686 *chip, uint8 address)
{
	sint32 value = direct_raw(chip, address);
	uint32 offset;
	if (address < REG_GYRO_DATA_X1) {
		offset = address - REG_ACCEL_DATA_X1;
	}
	else if (address < REG_TEMP_DATA1) {
		offset = address - REG_GYRO_DATA_X1;
	}
	else {
		offset = address - REG_TEMP_DATA1;
	}
	return (offset & 1U) == 0U ? (uint8)value :
		(uint8)((uint16)value >> 8);
}

static uint8 fifo_byte(Icm45686 *chip)
{
	uint8 value;
	update_fifo(chip);
	if (chip->fifo_count == 0U) {
		return 0x7fU;
	}
	value = chip->fifo[chip->fifo_head][chip->fifo_offset++];
	if (chip->fifo_offset == FIFO_PACKET_BYTES) {
		chip->fifo_offset = 0U;
		chip->fifo_head = (uint16)((chip->fifo_head + 1U) %
			FIFO_PACKETS_MAX);
		chip->fifo_count--;
	}
	return value;
}

/* [ADV-CHIP-ICM45686-001] [ADV-CHIP-ICM45686-003] Bank-0 reads use
 * default little-endian count/data and read-to-clear interrupt status. */
static uint8 read_register(Icm45686 *chip, uint8 address)
{
	uint8 value;
	if (address <= REG_TEMP_DATA1 + 1U) {
		return direct_data(chip, address);
	}
	if (address == REG_FIFO_COUNT_0 || address == REG_FIFO_COUNT_1) {
		update_fifo(chip);
		return address == REG_FIFO_COUNT_0 ? (uint8)chip->fifo_count :
			(uint8)(chip->fifo_count >> 8);
	}
	if (address == REG_FIFO_DATA) {
		return fifo_byte(chip);
	}
	if (address == REG_INT1_STATUS0 || address == REG_INT1_STATUS1) {
		update_fifo(chip);
		value = chip->registers[address];
		chip->registers[address] = 0U;
		chip->interrupt_pending &= (uint8)~value;
		if (address == REG_INT1_STATUS0 &&
		    (chip->registers[REG_INT1_CONFIG2] & 0x02U) != 0U &&
		    chip->interrupt_pending == 0U && chip->data_ready_resolved) {
			std_bool active_low =
				(chip->registers[REG_INT1_CONFIG2] & 0x01U) == 0U;
			(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
				chip->data_ready, active_low);
		}
		return value;
	}
	return chip->registers[address];
}

static std_bool affects_fifo(uint8 address)
{
	return address == REG_PWR_MGMT0 || address == REG_ACCEL_CONFIG0 ||
		address == REG_GYRO_CONFIG0 || address == REG_FIFO_CONFIG0 ||
		address == REG_FIFO_CONFIG1_0 ||
		address == REG_FIFO_CONFIG1_1 ||
		address == REG_FIFO_CONFIG2 || address == REG_FIFO_CONFIG3 ||
		address == REG_FIFO_CONFIG4;
}

/* [ADV-CHIP-ICM45686-002] [ADV-CHIP-ICM45686-005] Accept the PX4
 * configuration sequence, explicit flush, and compatibility reset by bypass. */
static void write_register(Icm45686 *chip, uint8 address, uint8 value)
{
	std_bool was_on = fifo_on(chip);
	std_bool now_on;
	if (was_on && affects_fifo(address)) {
		update_fifo(chip);
	}
	if (address == REG_MISC2) {
		if ((value & SOFT_RESET) != 0U) {
			reset_registers(chip);
		}
		return;
	}
	if (address == REG_FIFO_CONFIG2) {
		uint64 next_sample_time = chip->next_sample_time;
		chip->registers[address] = value & (uint8)~FIFO_FLUSH;
		if ((value & FIFO_FLUSH) != 0U) {
			clear_fifo(chip);
			if (fifo_on(chip) && next_sample_time != 0U) {
				chip->next_sample_time = next_sample_time;
			}
		}
		return;
	}
	if (address == REG_WHO_AM_I || address == REG_FIFO_COUNT_0 ||
		address == REG_FIFO_COUNT_1 || address == REG_FIFO_DATA ||
		address == REG_INT1_STATUS0 || address == REG_INT1_STATUS1 ||
		address <= REG_TEMP_DATA1 + 1U) {
		return;
	}
	chip->registers[address] = value;
	if (address == REG_INT1_CONFIG0) {
		chip->interrupt_pending &= value;
	}
	if (address == REG_FIFO_CONFIG0 &&
	    (value & FIFO_MODE_MASK) == 0U) {
		/* DS-000577 only guarantees FIFO_FLUSH. PX4 v1.16/v1.17 uses
		 * bypass as FIFOReset, so entering bypass also clears it. */
		clear_fifo(chip);
	}
	if (address == REG_GYRO_CONFIG0 || address == REG_ACCEL_CONFIG0) {
		apply_rate(chip);
	}
	now_on = fifo_on(chip);
	if (!was_on && now_on) {
		chip->next_sample_time = time_now(chip) +
			chip->clocks_per_sample;
		report_periods(chip);
	}
	else if (was_on && !now_on) {
		chip->next_sample_time = 0U;
	}
}

static void chip_select(void *instance, std_bool selected)
{
	Icm45686 *chip = instance;
	(void)selected;
	chip->started = FALSE;
}

/* SPI framing only decodes the command; register execution stays above. */
static uint8 chip_exchange(void *instance, uint8 mosi)
{
	Icm45686 *chip = instance;
	uint8 miso = 0U;
	if (!chip->started) {
		chip->started = TRUE;
		chip->reading = (mosi & 0x80U) != 0U;
		chip->address = mosi & 0x7fU;
		return 0U;
	}
	if (chip->reading) {
		miso = read_register(chip, chip->address);
	}
	else {
		write_register(chip, chip->address, mosi);
	}
	if (chip->address != REG_FIFO_DATA) {
		chip->address = (chip->address + 1U) & 0x7fU;
	}
	return miso;
}

static void drive_inactive(Icm45686 *chip)
{
	std_bool active_low =
		(chip->registers[REG_INT1_CONFIG2] & 0x01U) == 0U;
	if (chip->data_ready_resolved) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, active_low);
	}
}

static sint32 reset(void *instance)
{
	Icm45686 *chip = instance;
	chip->started = FALSE;
	chip->reading = FALSE;
	chip->address = 0U;
	chip->reported_gyro_ns = 0U;
	chip->reported_accel_ns = 0U;
	reset_registers(chip);
	drive_inactive(chip);
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Icm45686 *chip;
	const char *rotation;
	const void *ideal;
	uint32 ideal_size;
	uint64 frequency;
	if (args->target_ops->time_frequency_hz == NULL) {
		return -1;
	}
	frequency = args->target_ops->time_frequency_hz(args->target);
	if (frequency == 0U) {
		return -1;
	}
	chip = calloc(1U, sizeof(*chip));
	if (chip == NULL) {
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	chip->frequency_hz = frequency;
	chip->ideal_filters = args->node->ops->get_property(
		args->node->context, "athrill,ideal-filters", &ideal,
		&ideal_size) == 0;
	if (args->node->ops->get_string != NULL &&
	    args->node->ops->get_string(args->node->context,
		"athrill,mount-rotation", 0U, &rotation) == 0 &&
	    strcmp(rotation, "yaw-270") == 0) {
		chip->yaw_270 = TRUE;
	}
	reset_registers(chip);
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Icm45686 *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0 ||
	    args->resources->get_gpio(args->context, "int-gpios", 0U,
		&chip->data_ready) != 0) {
		return -1;
	}
	chip->gpio_resources = args->resources;
	chip->gpio_context = args->context;
	chip->data_ready_resolved = TRUE;
	if (args->resources->struct_size >=
	    offsetof(AthrillDeviceResourceOps, get_sample_period) +
	    sizeof(args->resources->get_sample_period) &&
	    args->resources->get_sample_period != NULL &&
	    args->resources->get_sample_period(args->context, "value-source",
		&chip->period_ops, &chip->period_instance) != 0) {
		chip->period_ops = NULL;
	}
	drive_inactive(chip);
	return 0;
}

/* [ADV-CHIP-ICM45686-007] INT1 status enables gate active-low/high pulse
 * delivery; status remains read-to-clear independently of the pin. */
static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	Icm45686 *chip = instance;
	std_bool active_low;
	std_bool latch;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	update_fifo(chip);
	active_low = (chip->registers[REG_INT1_CONFIG2] & 0x01U) == 0U;
	latch = (chip->registers[REG_INT1_CONFIG2] & 0x02U) != 0U;
	if (chip->interrupt_pending != 0U) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, !active_low);
		if (!latch) {
			(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
				chip->data_ready, active_low);
			chip->interrupt_pending = 0U;
		}
	}
	else {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, active_low);
	}
	if (fifo_on(chip)) {
		report_periods(chip);
		result->next_event_interval = chip->next_sample_time > now ?
			chip->next_sample_time - now : 0U;
	}
	return 0;
}

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), chip_select, chip_exchange
};

const AthrillDeviceModelOps athrill_device_icm45686_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "icm45686", "1", NULL,
	create, free, NULL, &spi_ops, NULL, NULL, connect_model, reset, tick
};
