/*
 * TDK InvenSense ICM-20689 SPI IMU model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"
#include "imu_filter.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_SMPLRT_DIV       0x19U
#define REG_CONFIG           0x1aU
#define REG_GYRO_CONFIG      0x1bU
#define REG_ACCEL_CONFIG     0x1cU
#define REG_ACCEL_CONFIG2    0x1dU
#define REG_FIFO_EN          0x23U
#define REG_INT_PIN_CFG      0x37U
#define REG_INT_ENABLE       0x38U
#define REG_INT_STATUS       0x3aU
#define REG_ACCEL_XOUT_H     0x3bU
#define REG_TEMP_OUT_H       0x41U
#define REG_GYRO_XOUT_H      0x43U
#define REG_SIGNAL_PATH_RESET 0x68U
#define REG_USER_CTRL        0x6aU
#define REG_PWR_MGMT_1       0x6bU
#define REG_FIFO_COUNTH      0x72U
#define REG_FIFO_COUNTL      0x73U
#define REG_FIFO_R_W         0x74U
#define REG_WHO_AM_I         0x75U
#define WHO_AM_I_VALUE       0x98U
#define PWR_MGMT_1_RESET     0x40U
#define FIFO_EN_TEMP         (1U << 7)
#define FIFO_EN_XG           (1U << 6)
#define FIFO_EN_YG           (1U << 5)
#define FIFO_EN_ZG           (1U << 4)
#define FIFO_EN_ACCEL        (1U << 3)
#define USER_CTRL_FIFO_EN    (1U << 6)
#define USER_CTRL_FIFO_RST   (1U << 2)
#define USER_CTRL_SIG_COND_RST (1U << 0)
#define PWR_MGMT_1_RESET_BIT (1U << 7)
#define PWR_MGMT_1_SLEEP     (1U << 6)
#define INT_PIN_CFG_INT_LEVEL (1U << 7)
#define INT_ENABLE_DATA_RDY  (1U << 0)
#define GRAVITY              9.80665F
#define RAD_TO_DEG           57.29577951F
#define TEMP_SENSITIVITY     326.8F
#define TEMP_OFFSET          25.0F
#define FIFO_BYTES           512U

typedef struct {
	int16_t acceleration[3];
	int16_t temperature;
	int16_t angular_velocity[3];
} RawSample;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	/* the data-ready line (ABI 1.16: GPIO lines in the runtime) */
	const AthrillDeviceResourceOps *gpio_resources;
	void *gpio_context;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint8 registers[128];
	uint64 frequency_hz;
	/* the sample periods the value source was told last (ABI 1.22), 0: none yet */
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 reported_gyro_ns;
	uint64 reported_accel_ns;
	std_bool rotated;
	std_bool started;
	std_bool reading;
	uint8 address;
	uint64 fifo_start;
	uint64 fifo_taken;
	uint64 fifo_read_bytes;
	std_bool fifo_overflow;
	uint8 record[14];
	int16_t acceleration_held[3];
	uint64 next_sample;
	AthrillDeviceGpioLine data_ready;
	std_bool data_ready_resolved;
	std_bool ideal_filters;
	ImuFilterStream accel_filter;
	ImuFilterStream gyro_filter;
} Icm20689;

static uint64 time_now(const Icm20689 *chip);

static int16_t saturate(float32 value)
{
	if (value >= 32767.0F) {
		return 32767;
	}
	if (value <= -32768.0F) {
		return -32768;
	}
	return (int16_t)(value < 0.0F ? value - 0.5F : value + 0.5F);
}

static void to_chip_axes(const Icm20689 *chip, const float32 *frd,
	float32 *output)
{
	float32 value[3];
	if (chip->rotated) {
		value[0] = frd[1];
		value[1] = -frd[0];
		value[2] = frd[2];
	} else {
		memcpy(value, frd, sizeof(value));
	}
	output[0] = value[0];
	output[1] = -value[1];
	output[2] = -value[2];
}

static void raw_from_sample(const Icm20689 *chip,
	const AthrillDeviceSensorSample *sample, RawSample *raw)
{
	static const float32 acceleration_scale[4] = {
		16384.0F, 8192.0F, 4096.0F, 2048.0F
	};
	static const float32 gyro_scale[4] = {
		131.0F, 65.5F, 32.8F, 16.4F
	};
	float32 acceleration[3], angular_velocity[3];
	float32 acc_factor = acceleration_scale[
		(chip->registers[REG_ACCEL_CONFIG] >> 3) & 3U] / GRAVITY;
	float32 gyro_factor = gyro_scale[
		(chip->registers[REG_GYRO_CONFIG] >> 3) & 3U] * RAD_TO_DEG;
	uint32 i;
	to_chip_axes(chip, sample->acceleration, acceleration);
	to_chip_axes(chip, sample->angular_velocity, angular_velocity);
	for (i = 0U; i < 3U; i++) {
		raw->acceleration[i] = saturate(acceleration[i] * acc_factor);
		raw->angular_velocity[i] = saturate(angular_velocity[i] * gyro_factor);
	}
	raw->temperature = saturate((sample->temperature_c - TEMP_OFFSET) *
		TEMP_SENSITIVITY);
}

static void filtered_sample(Icm20689 *chip, uint64 time, RawSample *raw)
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
	raw_from_sample(chip, &sample, raw);
}

/* [ADV-CHIP-ICM20689-002] The sample rate (the FIFO's and data-ready's) the registers set. */
static uint64 gyro_rate_mhz(const Icm20689 *chip)
{
	return chip_icm20689_gyro_rate_mhz(chip->registers[REG_CONFIG], chip->registers[REG_GYRO_CONFIG],
		chip->registers[REG_SMPLRT_DIV]);
}

static uint64 sample_clocks(const Icm20689 *chip)
{
	return chip_rate_clocks(chip->frequency_hz, gyro_rate_mhz(chip));
}

/* [ADV-CHIP-ICM20689-003] DLPF_CFG/A_DLPF_CFG bandwidths and bypass bits. */
static void configure_filters(Icm20689 *chip)
{
	static const float32 gyro_bandwidth[8] = {
		250.0F, 176.0F, 92.0F, 41.0F, 20.0F, 10.0F, 5.0F, 3281.0F
	};
	static const float32 accel_bandwidth[8] = {
		218.1F, 218.1F, 99.0F, 44.8F, 21.2F, 10.2F, 5.1F, 420.0F
	};
	uint64 gyro_rate = gyro_rate_mhz(chip);
	uint64 accel_rate = chip_icm20689_accel_rate_mhz(
		chip->registers[REG_ACCEL_CONFIG2],
		chip->registers[REG_SMPLRT_DIV]);
	uint8 gyro_order = (chip->registers[REG_GYRO_CONFIG] & 3U) == 0U ? 2U : 0U;
	uint8 accel_order = (chip->registers[REG_ACCEL_CONFIG2] & 0x08U) == 0U ? 2U : 0U;
	float32 gyro_bw = gyro_bandwidth[chip->registers[REG_CONFIG] & 7U];
	float32 accel_bw = accel_bandwidth[chip->registers[REG_ACCEL_CONFIG2] & 7U];
	uint64 now = time_now(chip);
	imu_filter_configure(&chip->gyro_filter, now,
		chip_rate_clocks(chip->frequency_hz, gyro_rate),
		(float32)gyro_rate / 1000.0F, &gyro_order, &gyro_bw, 1U);
	imu_filter_configure(&chip->accel_filter, now,
		chip_rate_clocks(chip->frequency_hz, accel_rate),
		(float32)accel_rate / 1000.0F, &accel_order, &accel_bw, 1U);
}

/* The value source learns each part's period once the chip samples into its FIFO or raises
 * data-ready, and each change. */
static void report_periods(Icm20689 *chip)
{
	uint64 gyro = chip_rate_period_ns(gyro_rate_mhz(chip));
	uint64 accel = chip_rate_period_ns(chip_icm20689_accel_rate_mhz(chip->registers[REG_ACCEL_CONFIG2],
		chip->registers[REG_SMPLRT_DIV]));
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

static std_bool sampling(const Icm20689 *chip)
{
	return (chip->registers[REG_PWR_MGMT_1] & PWR_MGMT_1_SLEEP) == 0U;
}

static uint32 record_bytes(const Icm20689 *chip)
{
	uint8 enabled = chip->registers[REG_FIFO_EN];
	uint32 count = 0U;
	count += (enabled & FIFO_EN_ACCEL) != 0U ? 6U : 0U;
	count += (enabled & FIFO_EN_TEMP) != 0U ? 2U : 0U;
	count += (enabled & FIFO_EN_XG) != 0U ? 2U : 0U;
	count += (enabled & FIFO_EN_YG) != 0U ? 2U : 0U;
	count += (enabled & FIFO_EN_ZG) != 0U ? 2U : 0U;
	return count;
}

static std_bool fifo_on(const Icm20689 *chip)
{
	return (chip->registers[REG_USER_CTRL] & USER_CTRL_FIFO_EN) != 0U &&
		record_bytes(chip) != 0U && sampling(chip);
}

static uint64 time_now(const Icm20689 *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

static void fifo_restart(Icm20689 *chip)
{
	chip->fifo_start = time_now(chip);
	chip->fifo_taken = 0U;
	chip->fifo_read_bytes = 0U;
	chip->fifo_overflow = FALSE;
}

static uint32 fifo_count(Icm20689 *chip)
{
	uint64 made;
	uint64 oldest;
	uint64 read_bytes;
	uint64 bytes;
	uint32 size = record_bytes(chip);
	uint64 capacity;
	if (size == 0U) {
		return 0U;
	}
	if (!fifo_on(chip)) {
		made = chip->fifo_taken;
	}
	else {
		chip->fifo_taken = (time_now(chip) - chip->fifo_start) /
			sample_clocks(chip);
		made = chip->fifo_taken;
	}
	capacity = FIFO_BYTES / size;
	oldest = made > capacity ? made - capacity : 0U;
	read_bytes = chip->fifo_read_bytes;
	if (read_bytes < oldest * size) {
		read_bytes = oldest * size;
	}
	bytes = made * size > read_bytes ? made * size - read_bytes : 0U;
	if (made > capacity && chip->fifo_read_bytes < oldest * size) {
		chip->fifo_overflow = TRUE;
	}
	return (uint32)bytes;
}

static uint8 fifo_byte(Icm20689 *chip)
{
	uint32 bytes = record_bytes(chip), position, at = 0U, i;
	uint8 enabled = chip->registers[REG_FIFO_EN];
	RawSample raw;
	if (bytes == 0U || fifo_count(chip) == 0U) {
		return 0U;
	}
	{
		uint64 capacity = FIFO_BYTES / bytes;
		uint64 oldest = chip->fifo_taken > capacity ?
			chip->fifo_taken - capacity : 0U;
		/* [ADV-FIFO-002] start with the newest capacity records. */
		if (chip->fifo_read_bytes < oldest * bytes) {
			chip->fifo_read_bytes = oldest * bytes;
		}
	}
	position = (uint32)(chip->fifo_read_bytes % bytes);
	if (position == 0U) {
		uint64 record = chip->fifo_read_bytes / bytes;
		/* [ADV-FIFO-001] [ADV-FIFO-003] [ADV-FIFO-004] */
		filtered_sample(chip, chip->fifo_start +
			(record + 1U) * sample_clocks(chip), &raw);
		if (record % 2U == 0U) {
			memcpy(chip->acceleration_held, raw.acceleration,
				sizeof(chip->acceleration_held));
		}
		if ((enabled & FIFO_EN_ACCEL) != 0U) {
			for (i = 0U; i < 3U; i++) {
				chip->record[at++] = (uint8)((uint16)chip->acceleration_held[i] >> 8);
				chip->record[at++] = (uint8)chip->acceleration_held[i];
			}
		}
		if ((enabled & FIFO_EN_TEMP) != 0U) {
			chip->record[at++] = (uint8)((uint16)raw.temperature >> 8);
			chip->record[at++] = (uint8)raw.temperature;
		}
		for (i = 0U; i < 3U; i++) {
			if ((enabled & (FIFO_EN_XG >> i)) != 0U) {
				chip->record[at++] = (uint8)((uint16)raw.angular_velocity[i] >> 8);
				chip->record[at++] = (uint8)raw.angular_velocity[i];
			}
		}
	}
	chip->fifo_read_bytes++;
	return chip->record[position];
}

static uint8 data_register(Icm20689 *chip, uint8 address)
{
	RawSample raw;
	int16_t value;
	filtered_sample(chip, time_now(chip), &raw);
	if (address < REG_TEMP_OUT_H) {
		value = raw.acceleration[(address - REG_ACCEL_XOUT_H) / 2U];
	} else if (address < REG_GYRO_XOUT_H) {
		value = raw.temperature;
	} else {
		value = raw.angular_velocity[(address - REG_GYRO_XOUT_H) / 2U];
	}
	return (address & 1U) == (REG_ACCEL_XOUT_H & 1U) ?
		(uint8)((uint16)value >> 8) : (uint8)value;
}

/* [ADV-CHIP-ICM20689-001] expose WHO_AM_I value 0x98 at register 0x75 */
static uint8 read_register(Icm20689 *chip, uint8 address)
{
	switch (address) {
	case REG_FIFO_COUNTH: {
		uint32 count = fifo_count(chip);
		chip->registers[REG_FIFO_COUNTL] = (uint8)count;
		return (uint8)(count >> 8);
	}
	case REG_FIFO_COUNTL: return chip->registers[REG_FIFO_COUNTL];
	case REG_FIFO_R_W: return fifo_byte(chip);
	case REG_INT_STATUS: return sampling(chip) ? 1U : 0U;
	case REG_WHO_AM_I: return WHO_AM_I_VALUE;
	default:
		if (address >= REG_ACCEL_XOUT_H && address <= REG_GYRO_XOUT_H + 5U) {
			return data_register(chip, address);
		}
		return chip->registers[address & 0x7fU];
	}
}

static void reset_registers(Icm20689 *chip)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_PWR_MGMT_1] = PWR_MGMT_1_RESET;
	chip->registers[REG_WHO_AM_I] = WHO_AM_I_VALUE;
}

static void write_register(Icm20689 *chip, uint8 address, uint8 value)
{
	std_bool was_on = fifo_on(chip);
	(void)fifo_count(chip);
	switch (address) {
	case REG_PWR_MGMT_1:
		if ((value & PWR_MGMT_1_RESET_BIT) != 0U) {
			reset_registers(chip);
			configure_filters(chip);
			fifo_restart(chip);
			return;
		}
		break;
	case REG_USER_CTRL:
		if ((value & USER_CTRL_FIFO_RST) != 0U) {
			fifo_restart(chip);
		}
		value &= (uint8)~(USER_CTRL_FIFO_RST | USER_CTRL_SIG_COND_RST);
		break;
	case REG_SIGNAL_PATH_RESET: value = 0U; break;
	case REG_FIFO_COUNTH:
	case REG_FIFO_COUNTL:
	case REG_INT_STATUS:
	case REG_WHO_AM_I:
	case REG_FIFO_R_W:
		return;
	default:
		break;
	}
	chip->registers[address & 0x7fU] = value;
	if (address == REG_SMPLRT_DIV || address == REG_CONFIG ||
	    address == REG_GYRO_CONFIG || address == REG_ACCEL_CONFIG2) {
		configure_filters(chip);
	}
	if (!was_on && fifo_on(chip)) {
		fifo_restart(chip);
	}
	if (fifo_on(chip) || (sampling(chip) && (chip->registers[REG_INT_ENABLE] & INT_ENABLE_DATA_RDY) != 0U)) {
		report_periods(chip);
	}
}

static void select_chip(void *instance, std_bool selected)
{
	Icm20689 *chip = instance;
	(void)selected;
	chip->started = FALSE;
}

static uint8 exchange(void *instance, uint8 mosi)
{
	Icm20689 *chip = instance;
	uint8 miso = 0U;
	if (!chip->started) {
		chip->started = TRUE;
		chip->reading = (mosi & 0x80U) != 0U;
		chip->address = mosi & 0x7fU;
		return 0U;
	}
	if (chip->reading) {
		miso = read_register(chip, chip->address);
	} else {
		write_register(chip, chip->address, mosi);
	}
	if (chip->address != REG_FIFO_R_W) {
		chip->address = (chip->address + 1U) & 0x7fU;
	}
	return miso;
}

static sint32 reset(void *instance)
{
	Icm20689 *chip = instance;
	reset_registers(chip);
	configure_filters(chip);
	chip->started = FALSE;
	chip->reading = FALSE;
	chip->address = 0U;
	chip->fifo_start = 0U;
	chip->fifo_taken = 0U;
	chip->fifo_read_bytes = 0U;
	chip->fifo_overflow = FALSE;
	chip->next_sample = 0U;
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Icm20689 *chip = calloc(1U, sizeof(*chip));
	const void *alignment;
	const void *ideal;
	uint32 size;
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
	chip->frequency_hz = frequency;
	chip->rotated = args->node->ops->get_property(args->node->context,
		"axis-align-x", &alignment, &size) == 0;
	chip->ideal_filters = args->node->ops->get_property(
		args->node->context, "athrill,ideal-filters", &ideal,
		&ideal_size) == 0;
	reset_registers(chip);
	configure_filters(chip);
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Icm20689 *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0) {
		return -1;
	}
	if (args->resources->get_gpio(args->context, "int-gpios", 0U,
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
	return 0;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	Icm20689 *chip = instance;
	std_bool active_low = (chip->registers[REG_INT_PIN_CFG] &
		INT_PIN_CFG_INT_LEVEL) != 0U;
	uint64 period;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	if ((chip->registers[REG_INT_ENABLE] & INT_ENABLE_DATA_RDY) == 0U ||
	    !sampling(chip)) {
		return 0;
	}
	period = sample_clocks(chip);
	(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready,
		active_low);
	if (chip->next_sample == 0U) {
		chip->next_sample = (now / period + 1U) * period;
	}
	if (now >= chip->next_sample) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready,
			!active_low);
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready,
			active_low);
		chip->next_sample += period *
			(1U + (now - chip->next_sample) / period);
	}
	result->next_event_interval = chip->next_sample - now;
	return 0;
}

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), select_chip, exchange
};

const AthrillDeviceModelOps athrill_device_icm20689_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "icm20689", "1", NULL,
	create, free, NULL, &spi_ops, NULL, NULL, connect_model, reset, tick
};
