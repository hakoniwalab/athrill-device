/*
 * iSentek IST8310 magnetometer model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_WAI        0x00U
#define REG_STAT1      0x02U
#define REG_DATAXL     0x03U
#define REG_CNTL1      0x0aU
#define REG_CNTL2      0x0bU
#define WAI_VALUE      0x10U
#define STAT1_DRDY     (1U << 0)
#define CNTL1_SINGLE   0x01U
#define CNTL2_SRST     (1U << 0)
#define LSB_PER_GAUSS  1320.0F

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint8 registers[256];
	uint8 pointer;
	std_bool address_next;
	std_bool rotated;
} Ist8310;

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

static void reset_registers(Ist8310 *chip)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_WAI] = WAI_VALUE;
}

static sint32 reset(void *instance)
{
	Ist8310 *chip = instance;
	reset_registers(chip);
	chip->pointer = 0U;
	chip->address_next = FALSE;
	return 0;
}

static void measure(Ist8310 *chip)
{
	AthrillDeviceSensorSample sample = {0};
	float32 raw[3];
	uint32 i;
	if (chip->target_ops->sensor_sample != NULL) {
		(void)chip->target_ops->sensor_sample(chip->target, &sample);
	}
	if (chip->rotated) {
		raw[0] = sample.magnetic_field[1];
		raw[1] = sample.magnetic_field[0];
		raw[2] = sample.magnetic_field[2];
	} else {
		raw[0] = sample.magnetic_field[0];
		raw[1] = sample.magnetic_field[1];
		raw[2] = -sample.magnetic_field[2];
	}
	for (i = 0U; i < 3U; i++) {
		int16_t value = saturate(raw[i] * LSB_PER_GAUSS);
		chip->registers[REG_DATAXL + 2U * i] = (uint8)value;
		chip->registers[REG_DATAXL + 2U * i + 1U] =
			(uint8)((uint16)value >> 8);
	}
	chip->registers[REG_STAT1] |= STAT1_DRDY;
	chip->registers[REG_CNTL1] = 0U;
}

static void write_register(Ist8310 *chip, uint8 value)
{
	uint8 address = chip->pointer++;
	switch (address) {
	case REG_CNTL1:
		chip->registers[address] = value;
		if ((value & 0x0fU) == CNTL1_SINGLE) {
			measure(chip);
		}
		break;
	case REG_CNTL2:
		if ((value & CNTL2_SRST) != 0U) {
			reset_registers(chip);
		} else {
			chip->registers[address] = value;
		}
		break;
	case REG_WAI:
	case REG_STAT1:
		break;
	default:
		chip->registers[address] = value;
		break;
	}
}

static uint8 read_register(Ist8310 *chip)
{
	uint8 address = chip->pointer++;
	uint8 value = chip->registers[address];
	if (address >= REG_DATAXL && address <= REG_DATAXL + 5U) {
		chip->registers[REG_STAT1] &= (uint8)~STAT1_DRDY;
	}
	return value;
}

static std_bool start(void *instance, std_bool read)
{
	Ist8310 *chip = instance;
	chip->address_next = !read;
	return TRUE;
}

static void write_byte(void *instance, uint8 data)
{
	Ist8310 *chip = instance;
	if (chip->address_next) {
		chip->pointer = data;
		chip->address_next = FALSE;
		return;
	}
	write_register(chip, data);
}

static uint8 read_byte(void *instance)
{
	return read_register(instance);
}

static void stop(void *instance)
{
	(void)instance;
}

static sint32 transfer(void *instance, AthrillDeviceI2cMessage *messages,
	uint32 message_count)
{
	Ist8310 *chip = instance;
	uint32 i, byte;
	if (messages == NULL && message_count != 0U) {
		return -1;
	}
	for (i = 0U; i < message_count; i++) {
		AthrillDeviceI2cMessage *message = &messages[i];
		if (message->size != 0U && message->data == NULL) {
			return -1;
		}
		if (!start(chip, message->read)) {
			return -1;
		}
		if (message->read) {
			for (byte = 0U; byte < message->size; byte++) {
				message->data[byte] = read_byte(chip);
			}
		} else {
			for (byte = 0U; byte < message->size; byte++) {
				write_byte(chip, message->data[byte]);
			}
		}
		if (message->stop) {
			stop(chip);
		}
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Ist8310 *chip = calloc(1U, sizeof(*chip));
	const char *rotation;
	if (chip == NULL) {
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	if (args->node->ops->get_string(args->node->context,
		"athrill,mount-rotation", 0U, &rotation) == 0 &&
	    strcmp(rotation, "roll-180-yaw-90") == 0) {
		chip->rotated = TRUE;
	}
	*instance = chip;
	return 0;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	return 0;
}

static const AthrillDeviceI2cPeripheralOps i2c_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceI2cPeripheralOps), transfer,
	start, write_byte, read_byte, stop
};

const AthrillDeviceModelOps athrill_device_ist8310_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "ist8310", "1", NULL,
	create, free, NULL, NULL, &i2c_ops, NULL, NULL, reset, tick
};
