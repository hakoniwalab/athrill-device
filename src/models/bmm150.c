/*
 * Bosch BMM150 magnetometer model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_CHIP_ID       0x40U
#define REG_DATAX_LSB     0x42U
#define REG_DATAY_LSB     0x44U
#define REG_DATAZ_LSB     0x46U
#define REG_RHALL_LSB     0x48U
#define REG_STATUS        0x4aU
#define REG_POWER_CONTROL 0x4bU
#define REG_OP_MODE       0x4cU
#define REG_REPXY         0x51U
#define REG_REPZ          0x52U
#define REG_DIG_X1        0x5dU
#define REG_DIG_Z4_LSB    0x62U
#define REG_DIG_Z2_LSB    0x68U

#define CHIP_ID_VALUE     0x32U
#define POWER_CONTROL     (1U << 0)
#define SOFT_RESET        ((1U << 7) | (1U << 1))
#define SELF_TEST         (1U << 0)
#define OP_MODE_MASK      (3U << 1)
#define OP_MODE_NORMAL    (0U << 1)
#define OP_MODE_SLEEP     (3U << 1)
#define STATUS_OVERFLOW   (1U << 6)
#define DATA_READY        (1U << 0)
#define OVERFLOW_XY       (-4096)
#define OVERFLOW_Z        (-16384)
#define RHALL_VALUE       8192U
#define RAW_PER_GAUSS     1600.0F

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 frequency_hz;
	uint64 sample_anchor;
	uint64 sampled_at;
	uint64 reported_period_ns;
	uint8 registers[256];
	uint8 pointer;
	std_bool address_next;
	std_bool sample_valid;
} Bmm150;

static uint64 time_now(const Bmm150 *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

static uint64 rate_mhz(const Bmm150 *chip)
{
	return chip_bmm150_rate_mhz(chip->registers[REG_OP_MODE]);
}

static uint64 sample_clocks(const Bmm150 *chip)
{
	return chip_rate_clocks(chip->frequency_hz, rate_mhz(chip));
}

static void set_trim_registers(Bmm150 *chip)
{
	/* These fixed trims reduce the PX4 v1.17.0 compensation equations to
	 * compensated_uT = raw / 16 when RHALL is 8192. */
	chip->registers[REG_DIG_X1] = 0U;
	chip->registers[REG_DIG_X1 + 1U] = 0U;
	chip->registers[REG_DIG_Z4_LSB] = 0U;
	chip->registers[REG_DIG_Z4_LSB + 1U] = 0U;
	chip->registers[REG_DIG_Z4_LSB + 2U] = 0x80U;
	chip->registers[REG_DIG_Z4_LSB + 3U] = 0x80U;
	chip->registers[REG_DIG_Z2_LSB] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 1U] = 0x60U;
	chip->registers[REG_DIG_Z2_LSB + 2U] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 3U] = 0x80U;
	chip->registers[REG_DIG_Z2_LSB + 4U] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 5U] = 0x20U;
	chip->registers[REG_DIG_Z2_LSB + 6U] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 7U] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 8U] = 0U;
	chip->registers[REG_DIG_Z2_LSB + 9U] = 0U;
}

static void reset_registers(Bmm150 *chip, std_bool powered)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_CHIP_ID] = CHIP_ID_VALUE;
	chip->registers[REG_DATAX_LSB] = 1U;
	chip->registers[REG_DATAY_LSB] = 1U;
	chip->registers[REG_DATAZ_LSB] = 1U;
	chip->registers[REG_POWER_CONTROL] = powered ? POWER_CONTROL : 0U;
	chip->registers[REG_OP_MODE] = OP_MODE_SLEEP;
	set_trim_registers(chip);
	chip->sample_anchor = time_now(chip);
	chip->sample_valid = FALSE;
}

static void report_period(Bmm150 *chip)
{
	uint64 period_ns = 0U;
	if ((chip->registers[REG_POWER_CONTROL] & POWER_CONTROL) != 0U &&
	    (chip->registers[REG_OP_MODE] & OP_MODE_MASK) == OP_MODE_NORMAL) {
		period_ns = chip_rate_period_ns(rate_mhz(chip));
	}
	if (chip->period_ops != NULL && period_ns != chip->reported_period_ns) {
		chip->reported_period_ns = period_ns;
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_MAG, period_ns);
	}
}

static sint32 reset(void *instance)
{
	Bmm150 *chip = instance;
	reset_registers(chip, FALSE);
	chip->pointer = 0U;
	chip->address_next = FALSE;
	chip->sampled_at = 0U;
	chip->reported_period_ns = 0U;
	return 0;
}

static int32_t rounded_raw(float32 gauss)
{
	float32 raw = gauss * RAW_PER_GAUSS;
	return (int32_t)(raw < 0.0F ? raw - 0.5F : raw + 0.5F);
}

static int16_t raw_axis(float32 gauss, std_bool z_axis,
	std_bool *overflow)
{
	/* [ADV-CHIP-BMM150-005] [ADV-CHIP-BMM150-006] Fixed trims make
	 * raw = field_gauss * 1600; reserved most-negative codes mean overflow. */
	int32_t raw = rounded_raw(gauss);
	int32_t minimum = z_axis ? -16383 : -4095;
	int32_t maximum = z_axis ? 16383 : 4095;
	if (raw < minimum || raw > maximum) {
		*overflow = TRUE;
		return z_axis ? OVERFLOW_Z : OVERFLOW_XY;
	}
	return (int16_t)raw;
}

static void encode_xy(Bmm150 *chip, uint8 address, int16_t raw)
{
	uint16 bits = (uint16)raw & 0x1fffU;
	chip->registers[address] = (uint8)((bits & 0x1fU) << 3) | 1U;
	chip->registers[address + 1U] = (uint8)(bits >> 5);
}

static void encode_z(Bmm150 *chip, int16_t raw)
{
	uint16 bits = (uint16)raw & 0x7fffU;
	chip->registers[REG_DATAZ_LSB] = (uint8)((bits & 0x7fU) << 1) | 1U;
	chip->registers[REG_DATAZ_LSB + 1U] = (uint8)(bits >> 7);
}

/* [ADV-CHIP-BMM150-004] One coherent sample at its BMM150 output time. */
static void measure(Bmm150 *chip, uint64 sample_time)
{
	AthrillDeviceSensorSample sample = {0};
	std_bool overflow = FALSE;
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	(void)athrill_device_sensor_sample_at(&chip->source, sample_time, &sample);
	raw_x = raw_axis(sample.magnetic_field[0], FALSE, &overflow);
	raw_y = raw_axis(sample.magnetic_field[1], FALSE, &overflow);
	raw_z = raw_axis(sample.magnetic_field[2], TRUE, &overflow);
	encode_xy(chip, REG_DATAX_LSB, raw_x);
	encode_xy(chip, REG_DATAY_LSB, raw_y);
	encode_z(chip, raw_z);
	chip->registers[REG_RHALL_LSB] =
		(uint8)((RHALL_VALUE & 0x3fU) << 2) | DATA_READY;
	chip->registers[REG_RHALL_LSB + 1U] = (uint8)(RHALL_VALUE >> 6);
	chip->registers[REG_STATUS] = overflow ? STATUS_OVERFLOW : 0U;
	chip->sampled_at = sample_time;
	chip->sample_valid = TRUE;
}

static void update_measurement(Bmm150 *chip)
{
	uint64 now;
	uint64 period;
	uint64 sample_time;
	if ((chip->registers[REG_POWER_CONTROL] & POWER_CONTROL) == 0U ||
	    (chip->registers[REG_OP_MODE] & OP_MODE_MASK) != OP_MODE_NORMAL) {
		return;
	}
	now = time_now(chip);
	period = sample_clocks(chip);
	if (period == 0U || now < chip->sample_anchor + period) {
		return;
	}
	sample_time = chip->sample_anchor +
		((now - chip->sample_anchor) / period) * period;
	if (!chip->sample_valid || sample_time != chip->sampled_at) {
		measure(chip, sample_time);
	}
}

static void write_register(Bmm150 *chip, uint8 value)
{
	/* [ADV-CHIP-BMM150-001] [ADV-CHIP-BMM150-002]
	 * Model the reset and successful normal self-test used by PX4. */
	uint8 address = chip->pointer++;
	if (address <= REG_STATUS || address >= REG_DIG_X1) {
		return;
	}
	switch (address) {
	case REG_POWER_CONTROL:
		if ((value & POWER_CONTROL) == 0U) {
			reset_registers(chip, FALSE);
		}
		else if ((value & SOFT_RESET) == SOFT_RESET) {
			reset_registers(chip, TRUE);
		}
		else {
			chip->registers[address] = value & (POWER_CONTROL | (1U << 2));
		}
		report_period(chip);
		break;
	case REG_OP_MODE:
		chip->registers[address] = value;
		if ((value & SELF_TEST) != 0U) {
			chip->registers[address] &= (uint8)~SELF_TEST;
			chip->registers[REG_DATAX_LSB] |= 1U;
			chip->registers[REG_DATAY_LSB] |= 1U;
			chip->registers[REG_DATAZ_LSB] |= 1U;
		}
		chip->sample_anchor = time_now(chip);
		chip->sample_valid = FALSE;
		report_period(chip);
		break;
	case REG_REPXY:
	case REG_REPZ:
		chip->registers[address] = value;
		break;
	default:
		chip->registers[address] = value;
		break;
	}
}

static uint8 read_register(Bmm150 *chip)
{
	uint8 address = chip->pointer++;
	if ((chip->registers[REG_POWER_CONTROL] & POWER_CONTROL) == 0U &&
	    address != REG_POWER_CONTROL) {
		return 0U;
	}
	if (address >= REG_DATAX_LSB && address <= REG_STATUS) {
		update_measurement(chip);
	}
	return chip->registers[address];
}

static std_bool start(void *instance, std_bool read)
{
	Bmm150 *chip = instance;
	chip->address_next = !read;
	return TRUE;
}

static void write_byte(void *instance, uint8 data)
{
	Bmm150 *chip = instance;
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
	Bmm150 *chip = instance;
	uint32 i;
	uint32 byte;
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
		for (byte = 0U; byte < message->size; byte++) {
			if (message->read) {
				message->data[byte] = read_byte(chip);
			}
			else {
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
	Bmm150 *chip;
	uint64 frequency;
	if (args->target_ops == NULL ||
	    args->target_ops->time_frequency_hz == NULL ||
	    (frequency = args->target_ops->time_frequency_hz(args->target)) == 0U) {
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
	reset_registers(chip, FALSE);
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Bmm150 *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0) {
		return -1;
	}
	if (args->resources->struct_size >=
	    offsetof(AthrillDeviceResourceOps, get_sample_period) +
	    sizeof(args->resources->get_sample_period) &&
	    args->resources->get_sample_period != NULL &&
	    args->resources->get_sample_period(args->context, "value-source",
		&chip->period_ops, &chip->period_instance) != 0) {
		chip->period_ops = NULL;
	}
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

const AthrillDeviceModelOps athrill_device_bmm150_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "bmm150", "1", NULL,
	create, free, NULL, NULL, &i2c_ops, NULL, connect_model, reset, tick
};
