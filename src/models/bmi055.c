/*
 * Bosch BMI055 accelerometer and gyroscope SPI models.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_CHIP_ID             0x00U
#define REG_DATA_X_L            0x02U
#define REG_TEMP                0x08U
#define REG_INT_STATUS_1        0x0aU
#define REG_FIFO_STATUS         0x0eU
#define REG_RANGE               0x0fU
#define REG_SOFTRESET           0x14U
#define REG_ACC_INT_EN_1        0x17U
#define REG_ACC_INT_MAP_1       0x1aU
#define REG_ACC_FIFO_CONFIG_0   0x30U
#define REG_GYR_INT_EN_0        0x15U
#define REG_GYR_INT_MAP_1       0x18U
#define REG_GYR_FIFO_WM_ENABLE  0x1eU
#define REG_GYR_FIFO_CONFIG_0   0x3dU
#define REG_FIFO_CONFIG_1       0x3eU
#define REG_FIFO_DATA           0x3fU

#define ACC_ID                  0xfaU
#define GYRO_ID                 0x0fU
#define SOFTRESET_VALUE         0xb6U
#define FIFO_MODE_MASK          0xc0U
#define GRAVITY                 9.80665F
#define RAD_TO_DEG              57.29577951F

typedef enum {
	BMI055_ACCEL = 0,
	BMI055_GYRO = 1,
} Bmi055Kind;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint8 registers[64];
	Bmi055Kind kind;
	std_bool yaw_180;
	uint64 clocks_per_sample;
	std_bool started;
	std_bool reading;
	uint8 address;
	uint64 fifo_start;
	uint64 fifo_read_frames;
	uint8 record[6];
	uint64 next_data_ready;
	AthrillDeviceGpioLine data_ready;
	std_bool data_ready_resolved;
} Bmi055;

static int16_t saturate(float32 value, float32 minimum, float32 maximum)
{
	if (value >= maximum) {
		return (int16_t)maximum;
	}
	if (value <= minimum) {
		return (int16_t)minimum;
	}
	return (int16_t)(value < 0.0F ? value - 0.5F : value + 0.5F);
}

static void to_chip_axes(const Bmi055 *chip, const float32 *frd,
	float32 *output)
{
	float32 x = frd[0];
	float32 y = frd[1];
	if (chip->yaw_180) {
		x = -x;
		y = -y;
	}
	output[0] = x;
	output[1] = -y;
	output[2] = -frd[2];
}

static void raw_sample(const Bmi055 *chip, int16_t *raw)
{
	static const float32 acceleration_lsb_per_g[16] = {
		0.0F, 0.0F, 0.0F, 1024.0F, 0.0F, 512.0F, 0.0F, 0.0F,
		256.0F, 0.0F, 0.0F, 0.0F, 128.0F, 0.0F, 0.0F, 0.0F
	};
	static const float32 gyro_lsb_per_dps[8] = {
		16.384F, 32.768F, 65.536F, 0.0F, 131.072F, 262.144F, 0.0F, 0.0F
	};
	AthrillDeviceSensorSample sample = {0};
	float32 value[3];
	float32 scale;
	uint32 i;
	if (chip->source.ops != NULL) {
		(void)chip->source.ops->sensor_sample(chip->source.instance, &sample);
	} else if (chip->target_ops->sensor_sample != NULL) {
		(void)chip->target_ops->sensor_sample(chip->target, &sample);
	}
	to_chip_axes(chip, chip->kind == BMI055_ACCEL ? sample.acceleration :
		sample.angular_velocity, value);
	if (chip->kind == BMI055_ACCEL) {
		scale = acceleration_lsb_per_g[chip->registers[REG_RANGE] & 0x0fU];
		if (scale == 0.0F) {
			scale = 128.0F;
		}
		scale /= GRAVITY;
		for (i = 0U; i < 3U; i++) {
			raw[i] = saturate(value[i] * scale, -2048.0F, 2047.0F);
		}
	} else {
		scale = gyro_lsb_per_dps[chip->registers[REG_RANGE] & 7U];
		if (scale == 0.0F) {
			scale = 16.384F;
		}
		scale *= RAD_TO_DEG;
		for (i = 0U; i < 3U; i++) {
			raw[i] = saturate(value[i] * scale, -32768.0F, 32767.0F);
		}
	}
}

static std_bool fifo_on(const Bmi055 *chip)
{
	return (chip->registers[REG_FIFO_CONFIG_1] & FIFO_MODE_MASK) != 0U;
}

static uint32 fifo_capacity(const Bmi055 *chip)
{
	return chip->kind == BMI055_ACCEL ? 32U : 100U;
}

static uint64 time_now(const Bmi055 *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

static void fifo_restart(Bmi055 *chip)
{
	chip->fifo_start = time_now(chip);
	chip->fifo_read_frames = 0U;
	chip->next_data_ready = 0U;
}

static uint32 fifo_frames(const Bmi055 *chip)
{
	uint64 made;
	uint64 available;
	if (!fifo_on(chip)) {
		return 0U;
	}
	made = (time_now(chip) - chip->fifo_start) / chip->clocks_per_sample;
	available = made > chip->fifo_read_frames ? made - chip->fifo_read_frames : 0U;
	return available > fifo_capacity(chip) ? fifo_capacity(chip) : (uint32)available;
}

static uint8 fifo_status(const Bmi055 *chip)
{
	uint64 made = fifo_on(chip) ?
		(time_now(chip) - chip->fifo_start) / chip->clocks_per_sample : 0U;
	uint64 available = made > chip->fifo_read_frames ?
		made - chip->fifo_read_frames : 0U;
	uint8 overrun = available > fifo_capacity(chip) ? 0x80U : 0U;
	return overrun | (uint8)fifo_frames(chip);
}

static void make_record(Bmi055 *chip)
{
	int16_t raw[3];
	uint32 i;
	raw_sample(chip, raw);
	for (i = 0U; i < 3U; i++) {
		uint16 value = chip->kind == BMI055_ACCEL ?
			(uint16)((sint32)raw[i] * 16) : (uint16)raw[i];
		chip->record[2U * i] = (uint8)value;
		chip->record[2U * i + 1U] = (uint8)(value >> 8);
	}
}

static uint8 fifo_byte(Bmi055 *chip)
{
	uint32 position;
	if (fifo_frames(chip) == 0U) {
		return 0U;
	}
	position = chip->registers[REG_FIFO_DATA] % 6U;
	if (position == 0U) {
		make_record(chip);
	}
	chip->registers[REG_FIFO_DATA] = (uint8)((position + 1U) % 6U);
	if (position == 5U) {
		chip->fifo_read_frames++;
	}
	return chip->record[position];
}

static uint8 data_register(Bmi055 *chip, uint8 address)
{
	int16_t raw[3];
	uint32 offset = address - REG_DATA_X_L;
	uint16 value;
	raw_sample(chip, raw);
	value = chip->kind == BMI055_ACCEL ?
		(uint16)((sint32)raw[offset / 2U] * 16) : (uint16)raw[offset / 2U];
	return (offset & 1U) == 0U ? (uint8)value : (uint8)(value >> 8);
}

static uint8 read_register(Bmi055 *chip, uint8 address)
{
	if (address == REG_CHIP_ID) {
		return chip->kind == BMI055_ACCEL ? ACC_ID : GYRO_ID;
	}
	if (address >= REG_DATA_X_L && address < REG_DATA_X_L + 6U) {
		return data_register(chip, address);
	}
	if (address == REG_TEMP && chip->kind == BMI055_ACCEL) {
		return (uint8)-6;
	}
	if (address == REG_FIFO_STATUS) {
		return fifo_status(chip);
	}
	if (address == REG_FIFO_DATA) {
		return fifo_byte(chip);
	}
	if (address == REG_INT_STATUS_1 && chip->kind == BMI055_ACCEL) {
		return fifo_frames(chip) >=
			(chip->registers[REG_ACC_FIFO_CONFIG_0] & 0x3fU) ? 0x40U : 0U;
	}
	return chip->registers[address & 0x3fU];
}

static void reset_registers(Bmi055 *chip)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_CHIP_ID] = chip->kind == BMI055_ACCEL ? ACC_ID : GYRO_ID;
	chip->registers[REG_RANGE] = chip->kind == BMI055_ACCEL ? 0x03U : 0U;
}

static void write_register(Bmi055 *chip, uint8 address, uint8 value)
{
	std_bool was_on = fifo_on(chip);
	uint8 fifo_config_0 = chip->kind == BMI055_ACCEL ?
		REG_ACC_FIFO_CONFIG_0 : REG_GYR_FIFO_CONFIG_0;
	if (address == REG_SOFTRESET && value == SOFTRESET_VALUE) {
		reset_registers(chip);
		fifo_restart(chip);
		return;
	}
	if (address == REG_CHIP_ID || address == REG_FIFO_STATUS ||
	    address == REG_FIFO_DATA) {
		return;
	}
	chip->registers[address & 0x3fU] = value;
	if (address == fifo_config_0 || address == REG_FIFO_CONFIG_1 ||
	    (!was_on && fifo_on(chip))) {
		chip->registers[REG_FIFO_DATA] = 0U;
		fifo_restart(chip);
	}
}

static void select_chip(void *instance, std_bool selected)
{
	Bmi055 *chip = instance;
	(void)selected;
	chip->started = FALSE;
	chip->registers[REG_FIFO_DATA] = 0U;
}

static uint8 exchange(void *instance, uint8 mosi)
{
	Bmi055 *chip = instance;
	uint8 miso = 0U;
	if (!chip->started) {
		chip->started = TRUE;
		chip->reading = (mosi & 0x80U) != 0U;
		chip->address = mosi & 0x3fU;
		return 0U;
	}
	if (chip->reading) {
		miso = read_register(chip, chip->address);
	} else {
		write_register(chip, chip->address, mosi);
	}
	if (chip->address != REG_FIFO_DATA) {
		chip->address = (chip->address + 1U) & 0x3fU;
	}
	return miso;
}

static std_bool interrupt_enabled(const Bmi055 *chip)
{
	if (chip->kind == BMI055_ACCEL) {
		return (chip->registers[REG_ACC_INT_EN_1] & 0x40U) != 0U &&
			(chip->registers[REG_ACC_INT_MAP_1] & 0x02U) != 0U;
	}
	return (chip->registers[REG_GYR_INT_EN_0] & 0x40U) != 0U &&
		(chip->registers[REG_GYR_INT_MAP_1] & 0x04U) != 0U &&
		(chip->registers[REG_GYR_FIFO_WM_ENABLE] & 0x80U) != 0U;
}

static sint32 reset(void *instance)
{
	Bmi055 *chip = instance;
	reset_registers(chip);
	chip->started = FALSE;
	chip->reading = FALSE;
	chip->address = 0U;
	fifo_restart(chip);
	if (chip->data_ready_resolved) {
		(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready, TRUE);
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Bmi055 *chip = calloc(1U, sizeof(*chip));
	const char *rotation;
	uint64 frequency;
	if (chip == NULL || args->target_ops->time_frequency_hz == NULL ||
	    (frequency = args->target_ops->time_frequency_hz(args->target)) == 0U) {
		free(chip);
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	chip->kind = strcmp(args->node->matched_compatible, "bosch,bma280") == 0 ?
		BMI055_ACCEL : BMI055_GYRO;
	chip->clocks_per_sample = frequency / 2000U;
	if (args->node->ops->get_string(args->node->context,
		"athrill,mount-rotation", 0U, &rotation) == 0 &&
	    strcmp(rotation, "yaw-180") == 0) {
		chip->yaw_180 = TRUE;
	}
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Bmi055 *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0 ||
	    args->resources->get_gpio(args->context, "int-gpios", 0U,
		&chip->data_ready) != 0) {
		return -1;
	}
	chip->data_ready_resolved = TRUE;
	(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready, TRUE);
	return 0;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	Bmi055 *chip = instance;
	uint8 config = chip->registers[chip->kind == BMI055_ACCEL ?
		REG_ACC_FIFO_CONFIG_0 : REG_GYR_FIFO_CONFIG_0];
	uint32 watermark = config & (chip->kind == BMI055_ACCEL ? 0x3fU : 0x7fU);
	uint64 period;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready, TRUE);
	if (!fifo_on(chip) || !interrupt_enabled(chip)) {
		return 0;
	}
	if (watermark == 0U) {
		watermark = 1U;
	}
	period = chip->clocks_per_sample * watermark;
	if (chip->next_data_ready == 0U) {
		chip->next_data_ready = (now / period + 1U) * period;
	}
	if (now >= chip->next_data_ready) {
		(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready, FALSE);
		(void)chip->target_ops->gpio_drive(chip->target, chip->data_ready, TRUE);
		chip->next_data_ready += period *
			(1U + (now - chip->next_data_ready) / period);
	}
	result->next_event_interval = chip->next_data_ready - now;
	return 0;
}

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), select_chip, exchange
};

const AthrillDeviceModelOps athrill_device_bmi055_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "bmi055", "1", NULL,
	create, free, NULL, &spi_ops, NULL, NULL, connect_model, reset, tick
};
