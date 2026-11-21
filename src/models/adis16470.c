/*
 * Analog Devices ADIS16470 SPI IMU model (data sheet revision C).
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"
#include "imu_filter.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_DIAG_STAT       0x02U
#define REG_FILT_CTRL       0x5cU
#define REG_MSC_CTRL        0x60U
#define REG_DEC_RATE        0x64U
#define REG_GLOB_CMD        0x68U
#define REG_FIRM_REV        0x6cU
#define REG_FIRM_DM         0x6eU
#define REG_FIRM_Y          0x70U
#define REG_PROD_ID         0x72U
#define REG_SERIAL_NUM      0x74U
#define PROD_ID_VALUE       0x4056U
#define MSC_CTRL_DEFAULT    0x00c1U
#define DIAG_SPI_ERROR      (1U << 3)
#define GLOB_SOFTWARE_RESET (1U << 7)
#define GLOB_SENSOR_TEST    (1U << 2)
#define BURST_WORDS         10U
#define GRAVITY             9.80665F
#define RAD_TO_DEG          57.29577951F
#define GYRO_BANDWIDTH_HZ   550.0F
#define ACCEL_BANDWIDTH_HZ  600.0F

typedef struct {
	int16_t gyro[3];
	int16_t accel[3];
	int16_t temperature;
} RawSample;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceResourceOps *gpio_resources;
	void *gpio_context;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 frequency_hz;
	uint64 reported_period_ns;
	uint64 sample_anchor;
	uint64 next_sample;
	uint16 registers[64];
	uint16 pending;
	uint16 command;
	uint16 burst[BURST_WORDS];
	uint8 byte_index;
	uint8 burst_index;
	std_bool burst_active;
	std_bool selected;
	AthrillDeviceGpioLine data_ready;
	ImuFilterStream accel_filter;
	ImuFilterStream gyro_filter;
} Adis16470;

static uint64 time_now(const Adis16470 *chip)
{
	return chip->target_ops->time_now != NULL ?
		chip->target_ops->time_now(chip->target) : 0U;
}

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

static uint64 rate_mhz(const Adis16470 *chip)
{
	return chip_adis16470_rate_mhz(chip->registers[REG_DEC_RATE / 2U]);
}

static uint64 sample_clocks(const Adis16470 *chip)
{
	return chip_rate_clocks(chip->frequency_hz, rate_mhz(chip));
}

static uint64 current_sample_time(const Adis16470 *chip)
{
	uint64 now = time_now(chip);
	uint64 period = sample_clocks(chip);
	if (period == 0U || now <= chip->sample_anchor) {
		return chip->sample_anchor;
	}
	return chip->sample_anchor +
		((now - chip->sample_anchor) / period) * period;
}

static void configure_filters(Adis16470 *chip)
{
	/* [ADV-CHIP-ADIS16470-007] The data sheet gives only the -3 dB points. */
	uint8 order = 1U;
	float32 accel_bandwidth = ACCEL_BANDWIDTH_HZ;
	float32 gyro_bandwidth = GYRO_BANDWIDTH_HZ;
	uint64 period = sample_clocks(chip);
	float32 rate = (float32)rate_mhz(chip) / 1000.0F;
	uint64 now = time_now(chip);
	imu_filter_configure(&chip->accel_filter, now, period, rate,
		&order, &accel_bandwidth, 1U);
	imu_filter_configure(&chip->gyro_filter, now, period, rate,
		&order, &gyro_bandwidth, 1U);
}

static void report_period(Adis16470 *chip)
{
	/* [ADV-CHIP-ADIS16470-005] Report the register-selected ODR. */
	uint64 period = chip_rate_period_ns(rate_mhz(chip));
	if (chip->period_ops != NULL && period != chip->reported_period_ns) {
		chip->reported_period_ns = period;
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_GYRO, period);
		(void)chip->period_ops->sample_period(chip->period_instance,
			ATHRILL_DEVICE_SAMPLE_ACCEL, period);
	}
}

static void raw_sample(Adis16470 *chip, uint64 time, RawSample *raw)
{
	/* [ADV-CHIP-ADIS16470-006] [ADV-CHIP-ADIS16470-007] */
	AthrillDeviceSensorSample sample = {0};
	float32 accel[3] = {0};
	float32 gyro[3] = {0};
	uint32 axis;
	(void)imu_filter_sample(&chip->accel_filter, &chip->source, time,
		IMU_FILTER_ACCEL, accel);
	(void)imu_filter_sample(&chip->gyro_filter, &chip->source, time,
		IMU_FILTER_GYRO, gyro);
	imu_filter_source_sample(&chip->source, time, &sample);
	/* Value sources are forward/right/down.  ADIS is forward/left/up. */
	accel[1] = -accel[1];
	accel[2] = -accel[2];
	gyro[1] = -gyro[1];
	gyro[2] = -gyro[2];
	for (axis = 0U; axis < 3U; axis++) {
		raw->accel[axis] = saturate(accel[axis] * 800.0F / GRAVITY);
		raw->gyro[axis] = saturate(gyro[axis] * RAD_TO_DEG * 10.0F);
	}
	raw->temperature = saturate(sample.temperature_c * 10.0F);
}

static void reset_registers(Adis16470 *chip)
{
	memset(chip->registers, 0, sizeof(chip->registers));
	chip->registers[REG_MSC_CTRL / 2U] = MSC_CTRL_DEFAULT;
	chip->registers[REG_PROD_ID / 2U] = PROD_ID_VALUE;
	chip->sample_anchor = time_now(chip);
	chip->next_sample = 0U;
	configure_filters(chip);
	report_period(chip);
}

/* [ADV-CHIP-ADIS16470-001] The product identity used by the PX4 probe. */
static uint16 read_register(Adis16470 *chip, uint8 address)
{
	uint8 even = address & 0x7eU;
	uint16 value = chip->registers[even / 2U];
	if (even == REG_DIAG_STAT) {
		chip->registers[REG_DIAG_STAT / 2U] = 0U;
	}
	return value;
}

/* [ADV-CHIP-ADIS16470-002] Low byte then high byte updates a 16-bit register. */
static void write_byte(Adis16470 *chip, uint8 address, uint8 value)
{
	uint8 even = address & 0x7eU;
	uint16 *reg = &chip->registers[even / 2U];
	if (even == REG_DIAG_STAT || even == REG_PROD_ID ||
	    even == REG_FIRM_REV || even == REG_FIRM_DM ||
	    even == REG_FIRM_Y || even == REG_SERIAL_NUM) {
		return;
	}
	if (even == REG_GLOB_CMD) {
		/* [ADV-CHIP-ADIS16470-003] PX4 reset and successful self-test. */
		if ((value & GLOB_SOFTWARE_RESET) != 0U) {
			reset_registers(chip);
		}
		if ((value & GLOB_SENSOR_TEST) != 0U) {
			chip->registers[REG_DIAG_STAT / 2U] &=
				(uint16)~(1U << 5);
		}
		return;
	}
	if ((address & 1U) == 0U) {
		*reg = (uint16)((*reg & 0xff00U) | value);
	}
	else {
		*reg = (uint16)((*reg & 0x00ffU) | ((uint16)value << 8));
	}
	if (even == REG_FILT_CTRL || even == REG_DEC_RATE) {
		configure_filters(chip);
		chip->sample_anchor = time_now(chip);
		chip->next_sample = 0U;
		report_period(chip);
	}
}

/* [ADV-CHIP-ADIS16470-004] One 176-bit burst is one time-resolved snapshot. */
static void prepare_burst(Adis16470 *chip)
{
	RawSample raw;
	uint64 sample_time = current_sample_time(chip);
	uint64 period = sample_clocks(chip);
	uint16 checksum = 0U;
	uint32 i;
	raw_sample(chip, sample_time, &raw);
	chip->burst[0] = chip->registers[REG_DIAG_STAT / 2U];
	chip->registers[REG_DIAG_STAT / 2U] = 0U;
	for (i = 0U; i < 3U; i++) {
		chip->burst[1U + i] = (uint16)raw.gyro[i];
		chip->burst[4U + i] = (uint16)raw.accel[i];
	}
	chip->burst[7] = (uint16)raw.temperature;
	chip->burst[8] = period == 0U ? 0U :
		(uint16)((sample_time - chip->sample_anchor) / period);
	for (i = 0U; i < 9U; i++) {
		checksum = (uint16)(checksum + (chip->burst[i] >> 8) +
			(chip->burst[i] & 0xffU));
	}
	chip->burst[9] = checksum;
	chip->burst_index = 0U;
	chip->burst_active = TRUE;
	chip->pending = chip->burst[0];
}

/* [ADV-CHIP-ADIS16470-002] Reads are pipelined by one 16-bit word. */
static void complete_word(Adis16470 *chip)
{
	uint8 address;
	if (chip->burst_active) {
		chip->burst_index++;
		if (chip->burst_index < BURST_WORDS) {
			chip->pending = chip->burst[chip->burst_index];
		}
		else {
			chip->pending = 0U;
			chip->burst_active = FALSE;
		}
		return;
	}
	address = (uint8)(chip->command >> 8);
	if (chip->command == 0x6800U) {
		prepare_burst(chip);
	}
	else if ((address & 0x80U) != 0U) {
		write_byte(chip, address & 0x7fU, (uint8)chip->command);
		chip->pending = 0U;
	}
	else {
		chip->pending = read_register(chip, address);
	}
}

static void select_chip(void *instance, std_bool selected)
{
	Adis16470 *chip = instance;
	if (selected) {
		chip->selected = TRUE;
		chip->byte_index = 0U;
		chip->command = 0U;
	}
	else {
		/* An incomplete word raises the sticky SPI communication flag. */
		if (chip->selected && chip->byte_index != 0U) {
			chip->registers[REG_DIAG_STAT / 2U] |= DIAG_SPI_ERROR;
		}
		chip->selected = FALSE;
		chip->byte_index = 0U;
		chip->burst_active = FALSE;
	}
}

static uint8 exchange(void *instance, uint8 mosi)
{
	Adis16470 *chip = instance;
	uint8 miso;
	if (chip->byte_index == 0U) {
		miso = (uint8)(chip->pending >> 8);
		chip->command = (uint16)mosi << 8;
		chip->byte_index = 1U;
	}
	else {
		miso = (uint8)chip->pending;
		chip->command |= mosi;
		chip->byte_index = 0U;
		complete_word(chip);
	}
	return miso;
}

static sint32 reset(void *instance)
{
	Adis16470 *chip = instance;
	chip->pending = 0U;
	chip->command = 0U;
	chip->byte_index = 0U;
	chip->burst_index = 0U;
	chip->burst_active = FALSE;
	chip->selected = FALSE;
	reset_registers(chip);
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Adis16470 *chip = calloc(1U, sizeof(*chip));
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
	reset_registers(chip);
	*instance = chip;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	Adis16470 *chip = instance;
	if (athrill_device_connect_value_source(chip->node, args,
		&chip->source) != 0 ||
	    args->resources->get_gpio(args->context, "int-gpios", 0U,
		&chip->data_ready) != 0) {
		return -1;
	}
	chip->gpio_resources = args->resources;
	chip->gpio_context = args->context;
	if (args->resources->struct_size >=
	    offsetof(AthrillDeviceResourceOps, get_sample_period) +
	    sizeof(args->resources->get_sample_period) &&
	    args->resources->get_sample_period != NULL &&
	    args->resources->get_sample_period(args->context, "value-source",
		&chip->period_ops, &chip->period_instance) != 0) {
		chip->period_ops = NULL;
	}
	report_period(chip);
	return 0;
}

/* [ADV-CHIP-ADIS16470-005] DR pulses at the configured output rate. */
static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	Adis16470 *chip = instance;
	uint64 period = sample_clocks(chip);
	std_bool active_low =
		(chip->registers[REG_MSC_CTRL / 2U] & 1U) == 0U;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	if (period == 0U) {
		return 0;
	}
	(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
		chip->data_ready, active_low);
	if (chip->next_sample == 0U) {
		chip->next_sample = chip->sample_anchor + period;
	}
	if (now >= chip->next_sample) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, !active_low);
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context,
			chip->data_ready, active_low);
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

const AthrillDeviceModelOps athrill_device_adis16470_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "adis16470", "1", NULL,
	create, free, NULL, &spi_ops, NULL, NULL, connect_model, reset, tick
};
