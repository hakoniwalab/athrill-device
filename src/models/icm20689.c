/*
 * TDK InvenSense ICM-20689 SPI IMU model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_SMPLRT_DIV       0x19U
#define REG_CONFIG           0x1aU
#define REG_GYRO_CONFIG      0x1bU
#define REG_ACCEL_CONFIG     0x1cU
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
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint8 registers[128];
	uint64 clocks_per_sample;
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
} Icm20689;

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

static void raw_sample(const Icm20689 *chip, RawSample *raw)
{
	static const float32 acceleration_scale[4] = {
		16384.0F, 8192.0F, 4096.0F, 2048.0F
	};
	static const float32 gyro_scale[4] = {
		131.0F, 65.5F, 32.8F, 16.4F
	};
	AthrillDeviceSensorSample sample = {0};
	float32 acceleration[3], angular_velocity[3];
	float32 acc_factor = acceleration_scale[
		(chip->registers[REG_ACCEL_CONFIG] >> 3) & 3U] / GRAVITY;
	float32 gyro_factor = gyro_scale[
		(chip->registers[REG_GYRO_CONFIG] >> 3) & 3U] * RAD_TO_DEG;
	uint32 i;
	if (chip->source.ops != NULL) {
		(void)chip->source.ops->sensor_sample(chip->source.instance, &sample);
	} else if (chip->target_ops->sensor_sample != NULL) {
		(void)chip->target_ops->sensor_sample(chip->target, &sample);
	}
	to_chip_axes(chip, sample.acceleration, acceleration);
	to_chip_axes(chip, sample.angular_velocity, angular_velocity);
	for (i = 0U; i < 3U; i++) {
		raw->acceleration[i] = saturate(acceleration[i] * acc_factor);
		raw->angular_velocity[i] = saturate(angular_velocity[i] * gyro_factor);
	}
	raw->temperature = saturate((sample.temperature_c - TEMP_OFFSET) *
		TEMP_SENSITIVITY);
}

static uint64 sample_clocks(const Icm20689 *chip)
{
	uint32 dlpf = chip->registers[REG_CONFIG] & 7U;
	return (dlpf == 0U || dlpf == 7U) ? chip->clocks_per_sample :
		chip->clocks_per_sample * 8U *
		(1U + chip->registers[REG_SMPLRT_DIV]);
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
	uint64 bytes;
	if (!fifo_on(chip)) {
		bytes = chip->fifo_taken * record_bytes(chip);
		return bytes > chip->fifo_read_bytes ?
			(uint32)(bytes - chip->fifo_read_bytes) : 0U;
	}
	chip->fifo_taken = (time_now(chip) - chip->fifo_start) /
		sample_clocks(chip);
	bytes = chip->fifo_taken * record_bytes(chip) - chip->fifo_read_bytes;
	if (bytes > FIFO_BYTES) {
		chip->fifo_overflow = TRUE;
		bytes = FIFO_BYTES;
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
	position = (uint32)(chip->fifo_read_bytes % bytes);
	if (position == 0U) {
		uint64 record = chip->fifo_read_bytes / bytes;
		raw_sample(chip, &raw);
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

static uint8 data_register(const Icm20689 *chip, uint8 address)
{
	RawSample raw;
	int16_t value;
	raw_sample(chip, &raw);
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
	if (!was_on && fifo_on(chip)) {
		fifo_restart(chip);
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
	uint32 size;
	uint64 frequency;
	if (chip == NULL || args->target_ops->time_frequency_hz == NULL ||
	    (frequency = args->target_ops->time_frequency_hz(args->target)) == 0U) {
		free(chip);
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	chip->clocks_per_sample = frequency / 8000U;
	chip->rotated = args->node->ops->get_property(args->node->context,
		"axis-align-x", &alignment, &size) == 0;
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
	chip->data_ready_resolved = TRUE;
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
	(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready,
		active_low);
	if (chip->next_sample == 0U) {
		chip->next_sample = (now / period + 1U) * period;
	}
	if (now >= chip->next_sample) {
		(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready,
			!active_low);
		(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready,
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
