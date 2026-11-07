/*
 * MS5611-01BA03 pressure sensor model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define CMD_RESET       0x1EU
#define CMD_CONVERT_D1  0x40U
#define CMD_CONVERT_D2  0x50U
#define CMD_ADC_READ    0x00U
#define CMD_PROM_READ   0xA0U
#define RAW_MAX         0xFFFFFFU

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint16 prom[8];
	uint8 command;
	uint8 out[3];
	uint32 out_len;
	uint32 out_pos;
	uint32 adc;
	uint32 d1;
	uint32 d2;
	std_bool has_d2;
	std_bool noise;
	uint64 noise_state;
} Ms5611;

static const uint16 calibration[6] = {
	40127U, 36924U, 23317U, 23282U, 33464U, 28312U
};
static const float32 pressure_noise_mbar[5] = {
	0.065F, 0.042F, 0.027F, 0.018F, 0.012F
};
static const float32 temperature_noise_c[5] = {
	0.012F, 0.008F, 0.005F, 0.003F, 0.002F
};

static uint16 prom_crc4(const uint16 *prom)
{
	uint16 rem = 0U;
	uint32 count, bit;
	for (count = 0U; count < 16U; count++) {
		uint16 word = (count == 14U || count == 15U) ?
			(uint16)(prom[7] & 0xff00U) : prom[count >> 1];
		rem ^= (count & 1U) != 0U ?
			(uint16)(word & 0xffU) : (uint16)(word >> 8);
		for (bit = 0U; bit < 8U; bit++) {
			rem = (rem & 0x8000U) != 0U ?
				(uint16)((rem << 1) ^ 0x3000U) : (uint16)(rem << 1);
		}
	}
	return (uint16)((rem >> 12) & 0xfU);
}

static sint32 reset(void *instance)
{
	Ms5611 *chip = instance;
	uint32 i;
	chip->prom[0] = 0U;
	for (i = 0U; i < 6U; i++) {
		chip->prom[i + 1U] = calibration[i];
	}
	chip->prom[7] = prom_crc4(chip->prom);
	chip->command = 0xffU;
	chip->out_len = 0U;
	chip->out_pos = 0U;
	chip->adc = 0U;
	chip->noise = TRUE;
	chip->has_d2 = FALSE;
	chip->noise_state = 0x9e3779b97f4a7c15ULL;
	return 0;
}

static float32 gaussian(Ms5611 *chip)
{
	double uniform[2];
	uint32 i;
	for (i = 0U; i < 2U; i++) {
		chip->noise_state ^= chip->noise_state << 13;
		chip->noise_state ^= chip->noise_state >> 7;
		chip->noise_state ^= chip->noise_state << 17;
		uniform[i] = ((double)(chip->noise_state >> 11) + 0.5) /
			9007199254740992.0;
	}
	return (float32)(sqrt(-2.0 * log(uniform[0])) *
		cos(6.283185307179586 * uniform[1]));
}

static void compensate(const Ms5611 *chip, uint32 d1, uint32 d2,
	sint32 *pressure, sint32 *temperature)
{
	int64_t dt = (int64_t)d2 - ((int64_t)chip->prom[5] << 8);
	int64_t temp = 2000 + ((dt * chip->prom[6]) >> 23);
	int64_t off = ((int64_t)chip->prom[2] << 16) +
		(((int64_t)chip->prom[4] * dt) >> 7);
	int64_t sens = ((int64_t)chip->prom[1] << 15) +
		(((int64_t)chip->prom[3] * dt) >> 8);
	if (temp < 2000) {
		int64_t t2 = (dt * dt) >> 31;
		int64_t factor = (temp - 2000) * (temp - 2000);
		int64_t off2 = (5 * factor) >> 1;
		int64_t sens2 = (5 * factor) >> 2;
		if (temp < -1500) {
			int64_t cold = (temp + 1500) * (temp + 1500);
			off2 += 7 * cold;
			sens2 += (11 * cold) >> 1;
		}
		temp -= t2;
		off -= off2;
		sens -= sens2;
	}
	*temperature = (sint32)temp;
	*pressure = (sint32)(((((int64_t)d1 * sens) >> 21) - off) >> 15);
}

static uint32 search(const Ms5611 *chip, std_bool for_pressure,
	uint32 fixed, sint32 wanted)
{
	uint32 low = 0U, high = RAW_MAX;
	while (low < high) {
		uint32 middle = low + (high - low) / 2U;
		sint32 pressure, temperature;
		if (for_pressure) {
			compensate(chip, middle, fixed, &pressure, &temperature);
		} else {
			compensate(chip, 0U, middle, &pressure, &temperature);
		}
		if ((for_pressure ? pressure : temperature) < wanted) {
			low = middle + 1U;
		} else {
			high = middle;
		}
	}
	return low;
}

static sint32 hundredths(float32 value)
{
	return (sint32)lround((double)value * 100.0);
}

static void raw(const Ms5611 *chip, float32 pressure, float32 temperature,
	uint32 *d1, uint32 *d2)
{
	*d2 = search(chip, FALSE, 0U, hundredths(temperature));
	*d1 = search(chip, TRUE, *d2, hundredths(pressure));
}

static void command(Ms5611 *chip, uint8 value)
{
	AthrillDeviceSensorSample sample = {
		.pressure_hpa = 1013.25F, .temperature_c = 20.0F
	};
	chip->command = value;
	if (value == CMD_RESET) {
		chip->adc = 0U;
	} else if ((value & 0xf0U) == CMD_CONVERT_D1 ||
	           (value & 0xf0U) == CMD_CONVERT_D2) {
		uint32 osr = ((uint32)value >> 1) & 7U;
		std_bool d1 = (value & 0xf0U) == CMD_CONVERT_D1;
		if (chip->source.ops != NULL) {
			(void)chip->source.ops->sensor_sample(chip->source.instance,
				&sample);
		} else if (chip->target_ops->sensor_sample != NULL) {
			(void)chip->target_ops->sensor_sample(chip->target, &sample);
		}
		if (chip->noise && osr < 5U) {
			if (d1) {
				sample.pressure_hpa += pressure_noise_mbar[osr] * gaussian(chip);
			} else {
				sample.temperature_c += temperature_noise_c[osr] * gaussian(chip);
			}
		}
		if (d1 && chip->has_d2) {
			chip->d1 = search(chip, TRUE, chip->d2,
				hundredths(sample.pressure_hpa));
		} else {
			raw(chip, sample.pressure_hpa, sample.temperature_c,
				&chip->d1, &chip->d2);
			chip->has_d2 = chip->has_d2 || !d1;
		}
		chip->adc = d1 ? chip->d1 : chip->d2;
	} else if (value == CMD_ADC_READ) {
		chip->out[0] = (uint8)(chip->adc >> 16);
		chip->out[1] = (uint8)(chip->adc >> 8);
		chip->out[2] = (uint8)chip->adc;
		chip->out_len = 3U;
	} else if ((value & 0xf0U) == CMD_PROM_READ) {
		uint16 word = chip->prom[(value >> 1) & 7U];
		chip->out[0] = (uint8)(word >> 8);
		chip->out[1] = (uint8)word;
		chip->out_len = 2U;
	}
}

static void select_chip(void *instance, std_bool selected)
{
	Ms5611 *chip = instance;
	(void)selected;
	chip->command = 0xffU;
	chip->out_len = 0U;
	chip->out_pos = 0U;
}

static uint8 exchange(void *instance, uint8 mosi)
{
	Ms5611 *chip = instance;
	if (chip->command == 0xffU) {
		command(chip, mosi);
		return 0U;
	}
	return chip->out_pos < chip->out_len ? chip->out[chip->out_pos++] : 0U;
}

static std_bool i2c_start(void *instance, std_bool read)
{
	Ms5611 *chip = instance;
	if (read) {
		chip->out_pos = 0U;
	} else {
		chip->command = 0xffU;
		chip->out_len = 0U;
		chip->out_pos = 0U;
	}
	return TRUE;
}

static void i2c_write_byte(void *instance, uint8 data)
{
	Ms5611 *chip = instance;
	if (chip->command == 0xffU) {
		command(chip, data);
	}
}

static uint8 i2c_read_byte(void *instance)
{
	Ms5611 *chip = instance;
	return chip->out_pos < chip->out_len ? chip->out[chip->out_pos++] : 0U;
}

static void i2c_stop(void *instance)
{
	(void)instance;
}

static sint32 i2c_transfer(void *instance,
	AthrillDeviceI2cMessage *messages, uint32 message_count)
{
	Ms5611 *chip = instance;
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
		if (!i2c_start(chip, current->read)) {
			return -1;
		}
		if (current->read) {
			for (byte = 0U; byte < current->size; byte++) {
				current->data[byte] = i2c_read_byte(chip);
			}
		} else {
			for (byte = 0U; byte < current->size; byte++) {
				i2c_write_byte(chip, current->data[byte]);
			}
		}
		if (current->stop) {
			i2c_stop(chip);
		}
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Ms5611 *chip = calloc(1U, sizeof(*chip));
	if (chip == NULL) {
		return -1;
	}
	chip->target_ops = args->target_ops;
	chip->target = args->target;
	chip->node = args->node;
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Ms5611 *chip = instance;
	return athrill_device_connect_value_source(chip->node, args,
		&chip->source);
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

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), select_chip, exchange
};

static const AthrillDeviceI2cPeripheralOps i2c_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceI2cPeripheralOps), i2c_transfer,
	i2c_start, i2c_write_byte, i2c_read_byte, i2c_stop
};

const AthrillDeviceModelOps athrill_device_ms5611_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "ms5611", "1", NULL,
	create, free, NULL, &spi_ops, &i2c_ops, NULL, connect_model, reset, tick
};
