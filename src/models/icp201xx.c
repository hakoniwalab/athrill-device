/*
 * TDK InvenSense ICP-201xx I2C pressure sensor model.
 * DS-000416 revision 1.1; PX4 v1.17.0 icp201xx driver.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include "chip_rates.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_TRIM1_MSB        0x05U
#define REG_TRIM2_LSB        0x06U
#define REG_TRIM2_MSB        0x07U
#define REG_DEVICE_ID        0x0cU
#define REG_OTP_CONFIG1      0xacU
#define REG_OTP_MR_LSB       0xadU
#define REG_OTP_MR_MSB       0xaeU
#define REG_OTP_MRA_LSB      0xafU
#define REG_OTP_MRA_MSB      0xb0U
#define REG_OTP_MRB_LSB      0xb1U
#define REG_OTP_MRB_MSB      0xb2U
#define REG_OTP_ADDRESS      0xb5U
#define REG_OTP_COMMAND      0xb6U
#define REG_OTP_RDATA        0xb8U
#define REG_OTP_STATUS       0xb9U
#define REG_OTP_DEBUG2       0xbcU
#define REG_MASTER_LOCK      0xbeU
#define REG_OTP_STATUS2      0xbfU
#define REG_MODE_SELECT      0xc0U
#define REG_INTERRUPT_STATUS 0xc1U
#define REG_INTERRUPT_MASK   0xc2U
#define REG_FIFO_CONFIG      0xc3U
#define REG_FIFO_FILL        0xc4U
#define REG_DEVICE_STATUS    0xcdU
#define REG_VERSION          0xd3U
#define REG_FIFO_BASE        0xfaU

#define DEVICE_ID_VALUE      0x63U
#define VERSION_B2           0xb2U
#define MODE_MEASUREMENT     0x08U
#define MODE_CONFIG_SHIFT    5U
#define MODE_CONFIG_MASK     7U
#define FIFO_FLUSH           0x80U
#define FIFO_EMPTY           0x40U
#define FIFO_FULL            0x20U
#define FIFO_LEVEL_MASK      0x1fU
#define FIFO_CAPACITY        16U
#define FIFO_FRAME_SIZE      6U
#define INT_FIFO_UNDERFLOW   0x02U
#define INT_FIFO_OVERFLOW    0x01U
#define RAW_MIN              (-524288)
#define RAW_MAX              524287

typedef struct {
	sint32 pressure;
	sint32 temperature;
} Icp201xxFrame;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint8 registers[256];
	Icp201xxFrame fifo[FIFO_CAPACITY];
	uint64 next_sample_time;
	uint64 period_clocks;
	uint8 fifo_head;
	uint8 fifo_count;
	uint8 pointer;
	std_bool address_next;
	std_bool measuring;
} Icp201xx;

static uint64 now(const Icp201xx *chip)
{
	if (chip->target_ops == NULL || chip->target_ops->time_now == NULL) {
		return 0U;
	}
	return chip->target_ops->time_now(chip->target);
}

static sint32 clamp_raw(double value)
{
	if (value <= (double)RAW_MIN) {
		return RAW_MIN;
	}
	if (value >= (double)RAW_MAX) {
		return RAW_MAX;
	}
	return (sint32)lround(value);
}

/* [ADV-CHIP-ICP201XX-005] Invert DS-000416 section 6.7 conversions. */
static void make_frame(Icp201xx *chip, Icp201xxFrame *frame, uint64 time)
{
	AthrillDeviceSensorSample sample = {
		.pressure_hpa = 1013.25F,
		.temperature_c = 25.0F
	};
	(void)athrill_device_sensor_sample_at(&chip->source, time, &sample);
	frame->pressure = clamp_raw((((double)sample.pressure_hpa * 100.0) -
		70000.0) * 131072.0 / 40000.0);
	frame->temperature = clamp_raw(((double)sample.temperature_c - 25.0) *
		262144.0 / 65.0);
}

/* [ADV-CHIP-ICP201XX-004] Materialize only when firmware observes the FIFO. */
static void update_fifo(Icp201xx *chip)
{
	uint64 current;
	uint64 due;
	uint64 accepted;
	uint64 i;
	if (!chip->measuring || chip->period_clocks == 0U) {
		return;
	}
	current = now(chip);
	if (current < chip->next_sample_time) {
		return;
	}
	due = (current - chip->next_sample_time) / chip->period_clocks + 1U;
	accepted = FIFO_CAPACITY - chip->fifo_count;
	if (accepted > due) {
		accepted = due;
	}
	for (i = 0U; i < accepted; i++) {
		uint8 tail = (uint8)((chip->fifo_head + chip->fifo_count) %
			FIFO_CAPACITY);
		make_frame(chip, &chip->fifo[tail], chip->next_sample_time +
			i * chip->period_clocks);
		chip->fifo_count++;
	}
	if (due > accepted) {
		chip->registers[REG_INTERRUPT_STATUS] |= INT_FIFO_OVERFLOW;
	}
	chip->next_sample_time += due * chip->period_clocks;
}

static void report_period(Icp201xx *chip, uint64 rate_mhz)
{
	uint64 period_ns;
	uint64 frequency;
	period_ns = chip_rate_period_ns(rate_mhz);
	frequency = chip->target_ops != NULL &&
		chip->target_ops->time_frequency_hz != NULL ?
		chip->target_ops->time_frequency_hz(chip->target) : 0U;
	chip->period_clocks = chip_rate_clocks(frequency, rate_mhz);
	if (chip->period_ops != NULL && chip->period_ops->sample_period != NULL) {
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_PRESSURE, period_ns);
	}
}

static void select_mode(Icp201xx *chip, uint8 value)
{
	uint8 mode = (uint8)((value >> MODE_CONFIG_SHIFT) & MODE_CONFIG_MASK);
	uint64 rate = chip_icp201xx_rate_mhz(mode);
	update_fifo(chip);
	chip->registers[REG_MODE_SELECT] = value;
	chip->measuring = (value & MODE_MEASUREMENT) != 0U && rate != 0U;
	if (chip->measuring) {
		report_period(chip, rate);
		chip->next_sample_time = now(chip) + chip->period_clocks;
	}
}

static void flush_fifo(Icp201xx *chip)
{
	chip->fifo_head = 0U;
	chip->fifo_count = 0U;
}

static sint32 reset(void *instance)
{
	Icp201xx *chip = instance;
	uint8 version = chip->registers[REG_VERSION];
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_DEVICE_ID] = DEVICE_ID_VALUE;
	chip->registers[REG_VERSION] = version;
	chip->registers[REG_DEVICE_STATUS] = 0x01U;
	chip->pointer = 0U;
	chip->address_next = FALSE;
	chip->measuring = FALSE;
	chip->period_clocks = 0U;
	chip->next_sample_time = 0U;
	flush_fifo(chip);
	return 0;
}

static uint8 otp_value(uint8 address)
{
	switch (address) {
	case 0xf8U:
		return 0x12U;
	case 0xf9U:
		return 0x03U;
	case 0xfaU:
		return 0x45U;
	default:
		return 0U;
	}
}

/* [ADV-CHIP-ICP201XX-002] Registers used by the PX4 v1.17.0 boot sequence. */
static void write_register(Icp201xx *chip, uint8 address, uint8 value)
{
	switch (address) {
	case REG_DEVICE_ID:
	case REG_DEVICE_STATUS:
	case REG_VERSION:
	case REG_OTP_RDATA:
	case REG_OTP_STATUS:
		break;
	case REG_MODE_SELECT:
		select_mode(chip, value);
		break;
	case REG_INTERRUPT_STATUS:
		chip->registers[address] &= (uint8)~value;
		break;
	case REG_FIFO_FILL:
		if ((value & FIFO_FLUSH) != 0U) {
			flush_fifo(chip);
		}
		break;
	case REG_OTP_COMMAND:
		chip->registers[address] = value;
		if ((value & 0x70U) == 0x10U) {
			chip->registers[REG_OTP_STATUS] = 0U;
			chip->registers[REG_OTP_RDATA] = otp_value(
				chip->registers[REG_OTP_ADDRESS]);
		}
		break;
	default:
		chip->registers[address] = value;
		break;
	}
}

static uint8 fifo_byte(Icp201xx *chip, uint8 offset)
{
	uint32 raw;
	std_bool last = offset == FIFO_FRAME_SIZE - 1U;
	if (chip->fifo_count == 0U) {
		if (last) {
			chip->registers[REG_INTERRUPT_STATUS] |= INT_FIFO_UNDERFLOW;
		}
		return 0U;
	}
	if (offset < 3U) {
		raw = (uint32)chip->fifo[chip->fifo_head].pressure & 0xfffffU;
	} else {
		raw = (uint32)chip->fifo[chip->fifo_head].temperature & 0xfffffU;
		offset = (uint8)(offset - 3U);
	}
	if (last) {
		chip->fifo_head = (uint8)((chip->fifo_head + 1U) % FIFO_CAPACITY);
		chip->fifo_count--;
	}
	return (uint8)(raw >> (8U * offset));
}

static uint8 read_register(Icp201xx *chip)
{
	uint8 address = chip->pointer;
	uint8 value;
	if (address == REG_FIFO_FILL) {
		update_fifo(chip);
		value = chip->fifo_count & FIFO_LEVEL_MASK;
		if (chip->fifo_count == 0U) {
			value |= FIFO_EMPTY;
		}
		if (chip->fifo_count == FIFO_CAPACITY) {
			value |= FIFO_FULL;
		}
		chip->pointer++;
		return value;
	}
	if (address >= REG_FIFO_BASE) {
		uint8 offset = (uint8)(address - REG_FIFO_BASE);
		update_fifo(chip);
		value = fifo_byte(chip, offset);
		chip->pointer = address == 0xffU ? REG_FIFO_BASE :
			(uint8)(address + 1U);
		return value;
	}
	value = chip->registers[address];
	chip->pointer++;
	return value;
}

static std_bool start(void *instance, std_bool read)
{
	Icp201xx *chip = instance;
	chip->address_next = !read;
	return TRUE;
}

static void write_byte(void *instance, uint8 data)
{
	Icp201xx *chip = instance;
	if (chip->address_next) {
		chip->pointer = data;
		chip->address_next = FALSE;
		return;
	}
	write_register(chip, chip->pointer++, data);
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
	Icp201xx *chip = instance;
	uint32 message;
	uint32 byte;
	if (messages == NULL && message_count != 0U) {
		return -1;
	}
	for (message = 0U; message < message_count; message++) {
		AthrillDeviceI2cMessage *current = &messages[message];
		if (current->data == NULL && current->size != 0U) {
			return -1;
		}
		if (!start(chip, current->read)) {
			return -1;
		}
		if (current->read) {
			for (byte = 0U; byte < current->size; byte++) {
				current->data[byte] = read_byte(chip);
			}
		} else {
			for (byte = 0U; byte < current->size; byte++) {
				write_byte(chip, current->data[byte]);
			}
		}
		if (current->stop) {
			stop(chip);
		}
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Icp201xx *chip = calloc(1U, sizeof(*chip));
	uint32 version;
	if (chip == NULL) {
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	if (args->node->ops->get_u32 != NULL &&
		args->node->ops->get_u32(args->node->context,
		"athrill,icp201xx-version", 0U, &version) == 0 &&
		version == VERSION_B2) {
		chip->registers[REG_VERSION] = VERSION_B2;
	}
	*instance = chip;
	return reset(chip);
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Icp201xx *chip = instance;
	sint32 result = athrill_device_connect_value_source(chip->node, args,
		&chip->source);
	if (result != 0) {
		return result;
	}
	if (args->resources->struct_size >=
		offsetof(AthrillDeviceResourceOps, get_sample_period) +
		sizeof(args->resources->get_sample_period) &&
		args->resources->get_sample_period != NULL &&
		args->resources->get_sample_period(args->context, "value-source",
		&chip->period_ops, &chip->period_instance) != 0) {
		chip->period_ops = NULL;
		chip->period_instance = NULL;
	}
	return 0;
}

static sint32 tick(void *instance, uint64 time,
	AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)time;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	return 0;
}

static const AthrillDeviceI2cPeripheralOps i2c_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceI2cPeripheralOps), transfer,
	start, write_byte, read_byte, stop
};

const AthrillDeviceModelOps athrill_device_icp201xx_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "icp201xx", "1", NULL,
	create, free, NULL, NULL, &i2c_ops, NULL, connect_model, reset, tick
};
