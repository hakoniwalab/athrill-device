/*
 * Bosch BMI055 accelerometer and gyroscope SPI models.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"
#include "imu_filter.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_CHIP_ID             0x00U
#define REG_DATA_X_L            0x02U
#define REG_TEMP                0x08U
#define REG_INT_STATUS_1        0x0aU
#define REG_FIFO_STATUS         0x0eU
#define REG_RANGE               0x0fU
#define REG_BW                  0x10U   /* accelerometer PMU_BW, gyroscope BW */
#define REG_HBW                 0x13U   /* accelerometer ACCD_HBW, gyroscope RATE_HBW */
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
	/* the data-ready line (ABI 1.16: GPIO lines in the runtime) */
	const AthrillDeviceResourceOps *gpio_resources;
	void *gpio_context;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource source;
	uint8 registers[64];
	Bmi055Kind kind;
	uint32 yaw_quarter_turns; /* athrill,mount-rotation yaw-90/180/270 = 1/2/3 */
	uint64 clocks_per_sample;
	uint64 frequency_hz;
	/* the sample period the value source was told last (ABI 1.22), 0: none yet */
	const AthrillDeviceSamplePeriodOps *period_ops;
	void *period_instance;
	uint64 reported_period_ns;
	std_bool started;
	std_bool reading;
	uint8 address;
	uint64 fifo_start;
	uint64 fifo_read_frames;
	uint8 *fifo_materialized;
	uint32 fifo_materialized_frames;
	uint32 fifo_materialized_read;
	uint8 record[6];
	uint64 next_data_ready;
	AthrillDeviceGpioLine data_ready;
	std_bool data_ready_resolved;
	std_bool ideal_filters;
	ImuFilterStream filter;
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
	/* Undo the board's yaw mount into the chip frame (PX4 rotation convention). */
	if (chip->yaw_quarter_turns == 1U) {
		/* yaw 90: board = (-chip_y, chip_x) */
		x = frd[1];
		y = -frd[0];
	}
	else if (chip->yaw_quarter_turns == 2U) {
		x = -frd[0];
		y = -frd[1];
	}
	else if (chip->yaw_quarter_turns == 3U) {
		/* yaw 270: board = (chip_y, -chip_x) */
		x = -frd[1];
		y = frd[0];
	}
	output[0] = x;
	output[1] = -y;
	output[2] = -frd[2];
}

static void raw_from_sample(const Bmi055 *chip,
	const AthrillDeviceSensorSample *sample, int16_t *raw)
{
	static const float32 acceleration_lsb_per_g[16] = {
		0.0F, 0.0F, 0.0F, 1024.0F, 0.0F, 512.0F, 0.0F, 0.0F,
		256.0F, 0.0F, 0.0F, 0.0F, 128.0F, 0.0F, 0.0F, 0.0F
	};
	static const float32 gyro_lsb_per_dps[8] = {
		16.384F, 32.768F, 65.536F, 0.0F, 131.072F, 262.144F, 0.0F, 0.0F
	};
	float32 value[3];
	float32 scale;
	uint32 i;
	to_chip_axes(chip, chip->kind == BMI055_ACCEL ? sample->acceleration :
		sample->angular_velocity, value);
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

static void filtered_sample(Bmi055 *chip, uint64 time, int16_t *raw)
{
	/* [ADV-IMU-FILTER-001] The DT property bypasses every modeled stage. */
	AthrillDeviceSensorSample sample = {0};
	float32 value[3];
	imu_filter_source_sample(&chip->source, time, &sample);
	if (!chip->ideal_filters) {
		(void)imu_filter_sample(&chip->filter, &chip->source, time,
			chip->kind == BMI055_ACCEL ? IMU_FILTER_ACCEL :
			IMU_FILTER_GYRO, value);
		if (chip->kind == BMI055_ACCEL) {
			memcpy(sample.acceleration, value, sizeof(value));
		}
		else {
			memcpy(sample.angular_velocity, value, sizeof(value));
		}
	}
	raw_from_sample(chip, &sample, raw);
}

static std_bool fifo_on(const Bmi055 *chip)
{
	return (chip->registers[REG_FIFO_CONFIG_1] & FIFO_MODE_MASK) != 0U;
}

static uint32 fifo_capacity(const Bmi055 *chip)
{
	if ((chip->registers[REG_FIFO_CONFIG_1] & FIFO_MODE_MASK) == 0x80U) {
		return chip->kind == BMI055_ACCEL ? 31U : 99U;
	}
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
	chip->fifo_materialized_frames = 0U;
	chip->fifo_materialized_read = 0U;
	chip->next_data_ready = 0U;
}

static uint64 lazy_frames(const Bmi055 *chip)
{
	uint64 made;
	if (!fifo_on(chip)) {
		return 0U;
	}
	made = (time_now(chip) - chip->fifo_start) / chip->clocks_per_sample;
	return made > chip->fifo_read_frames ? made - chip->fifo_read_frames : 0U;
}

/* The modeled FIFO paths use the datasheet stream behaviour: newest frames replace
 * the oldest unread frames, including frames saved across a configuration change. */
static void discard_overflow(Bmi055 *chip)
{
	uint64 saved = chip->fifo_materialized_frames -
		chip->fifo_materialized_read;
	uint64 lazy = lazy_frames(chip);
	uint64 capacity = fifo_capacity(chip);
	uint64 drop = saved + lazy > capacity ? saved + lazy - capacity : 0U;
	uint64 from_saved = drop < saved ? drop : saved;
	chip->fifo_materialized_read += (uint32)from_saved;
	drop -= from_saved;
	chip->fifo_read_frames += drop;
}

static uint32 fifo_frames(Bmi055 *chip)
{
	uint64 available;
	if (!fifo_on(chip)) {
		return 0U;
	}
	discard_overflow(chip);
	available = chip->fifo_materialized_frames -
		chip->fifo_materialized_read + lazy_frames(chip);
	return (uint32)available;
}

static uint8 fifo_status(Bmi055 *chip)
{
	uint64 available = chip->fifo_materialized_frames -
		chip->fifo_materialized_read + lazy_frames(chip);
	uint8 overrun = available > fifo_capacity(chip) ? 0x80U : 0U;
	return overrun | (uint8)fifo_frames(chip);
}

/* [ADV-FIFO-001] [ADV-FIFO-003] [ADV-FIFO-004] frame at its own time. */
static void make_record(Bmi055 *chip, uint64 time)
{
	int16_t raw[3];
	uint32 i;
	filtered_sample(chip, time, raw);
	for (i = 0U; i < 3U; i++) {
		uint16 value = chip->kind == BMI055_ACCEL ?
			(uint16)((sint32)raw[i] * 16) : (uint16)raw[i];
		chip->record[2U * i] = (uint8)value;
		chip->record[2U * i + 1U] = (uint8)(value >> 8);
	}
}

/* [ADV-FIFO-008] Freeze unread frames before the filter or period changes. */
static void materialize_fifo(Bmi055 *chip)
{
	uint32 saved;
	uint64 lazy;
	uint64 record;
	uint32 i;
	if (!fifo_on(chip) || fifo_frames(chip) == 0U) {
		return;
	}
	saved = chip->fifo_materialized_frames -
		chip->fifo_materialized_read;
	if (saved != 0U && chip->fifo_materialized_read != 0U) {
		memmove(chip->fifo_materialized,
			&chip->fifo_materialized[chip->fifo_materialized_read * 6U],
			saved * 6U);
	}
	lazy = lazy_frames(chip);
	for (i = 0U; i < (uint32)lazy; i++) {
		record = chip->fifo_read_frames + i;
		make_record(chip, chip->fifo_start +
			(record + 1U) * chip->clocks_per_sample);
		memcpy(&chip->fifo_materialized[(saved + i) * 6U],
			chip->record, 6U);
	}
	chip->fifo_materialized_frames = saved + (uint32)lazy;
	chip->fifo_materialized_read = 0U;
	chip->fifo_start = time_now(chip);
	chip->fifo_read_frames = 0U;
	chip->next_data_ready = 0U;
}

static uint8 fifo_byte(Bmi055 *chip)
{
	uint32 position;
	if (fifo_frames(chip) == 0U) {
		return 0U;
	}
	position = chip->registers[REG_FIFO_DATA] % 6U;
	if (position == 0U) {
		discard_overflow(chip);
		if (chip->fifo_materialized_read <
		    chip->fifo_materialized_frames) {
			memcpy(chip->record, &chip->fifo_materialized[
				chip->fifo_materialized_read * 6U], 6U);
		}
		else {
			make_record(chip, chip->fifo_start +
				(chip->fifo_read_frames + 1U) *
				chip->clocks_per_sample);
		}
	}
	chip->registers[REG_FIFO_DATA] = (uint8)((position + 1U) % 6U);
	if (position == 5U) {
		if (chip->fifo_materialized_read <
		    chip->fifo_materialized_frames) {
			chip->fifo_materialized_read++;
		}
		else {
			chip->fifo_read_frames++;
		}
	}
	return chip->record[position];
}

static uint8 data_register(Bmi055 *chip, uint8 address)
{
	int16_t raw[3];
	uint32 offset = address - REG_DATA_X_L;
	uint16 value;
	filtered_sample(chip, time_now(chip), raw);
	value = chip->kind == BMI055_ACCEL ?
		(uint16)((sint32)raw[offset / 2U] * 16) : (uint16)raw[offset / 2U];
	return (offset & 1U) == 0U ? (uint8)value : (uint8)(value >> 8);
}

/* [ADV-CHIP-BMI055A-001] expose the accelerometer 0xFA chip identity */
static uint8 accelerometer_chip_id(void)
{
	return ACC_ID;
}

/* [ADV-CHIP-BMI055G-001] expose the gyroscope 0x0F chip identity */
static uint8 gyroscope_chip_id(void)
{
	return GYRO_ID;
}

static uint8 read_register(Bmi055 *chip, uint8 address)
{
	if (address == REG_CHIP_ID) {
		return chip->kind == BMI055_ACCEL ? accelerometer_chip_id() :
			gyroscope_chip_id();
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
	/* reset values: accelerometer PMU_BW 0x0F (1000 Hz bandwidth), gyroscope BW 0x80 (2 kHz) */
	chip->registers[REG_BW] = chip->kind == BMI055_ACCEL ? 0x0FU : 0x80U;
}

/* [ADV-CHIP-BMI055A-002] [ADV-CHIP-BMI055G-002] The output data rate the registers set. */
static uint64 rate_mhz(const Bmi055 *chip)
{
	return chip->kind == BMI055_ACCEL ?
		chip_bmi055_accel_rate_mhz(chip->registers[REG_BW], chip->registers[REG_HBW]) :
		chip_bmi055_gyro_rate_mhz(chip->registers[REG_BW], chip->registers[REG_HBW]);
}

/* [ADV-CHIP-BMI055A-003] [ADV-CHIP-BMI055G-003] Datasheet bandwidth filters. */
static void configure_filter(Bmi055 *chip)
{
	static const float32 gyro_bandwidth[8] = {
		0.0F, 230.0F, 116.0F, 47.0F, 23.0F, 12.0F, 64.0F, 32.0F
	};
	uint8 order = 2U;
	float32 bandwidth;
	uint64 rate = rate_mhz(chip);
	if ((chip->registers[REG_HBW] & 0x80U) != 0U) {
		order = 0U;
	}
	if (chip->kind == BMI055_ACCEL) {
		uint8 code = chip->registers[REG_BW] & 0x1FU;
		if (code < 0x08U) {
			code = 0x08U;
		}
		if (code > 0x0FU) {
			code = 0x0FU;
		}
		bandwidth = 7.8125F * (float32)(1U << (code - 0x08U));
	}
	else {
		uint8 code = chip->registers[REG_BW] & 7U;
		bandwidth = gyro_bandwidth[code];
		if (code == 0U) {
			order = 0U;
		}
	}
	imu_filter_configure(&chip->filter, time_now(chip),
		chip_rate_clocks(chip->frequency_hz, rate),
		(float32)rate / 1000.0F, &order, &bandwidth, 1U);
}

/* Samples come at the rate the registers set. With the FIFO running, the frames already
 * made stay; the next ones follow the new period. */
static void apply_rate(Bmi055 *chip)
{
	uint64 clocks = chip_rate_clocks(chip->frequency_hz, rate_mhz(chip));
	if (clocks == 0U || clocks == chip->clocks_per_sample) {
		return;
	}
	if (fifo_on(chip)) {
		chip->fifo_start = time_now(chip);
		chip->fifo_read_frames = 0U;
		chip->next_data_ready = 0U;
	}
	chip->clocks_per_sample = clocks;
}

/* The value source learns the period once the chip produces samples, and each change. */
static void report_period(Bmi055 *chip)
{
	uint64 period_ns = chip_rate_period_ns(rate_mhz(chip));
	if (chip->period_ops == NULL || period_ns == chip->reported_period_ns) {
		return;
	}
	chip->reported_period_ns = period_ns;
	(void)chip->period_ops->sample_period(chip->period_instance,
		chip->kind == BMI055_ACCEL ? ATHRILL_DEVICE_SAMPLE_ACCEL : ATHRILL_DEVICE_SAMPLE_GYRO,
		period_ns);
}

static void write_register(Bmi055 *chip, uint8 address, uint8 value)
{
	std_bool was_on = fifo_on(chip);
	uint8 fifo_config_0 = chip->kind == BMI055_ACCEL ?
		REG_ACC_FIFO_CONFIG_0 : REG_GYR_FIFO_CONFIG_0;
	if (address == REG_SOFTRESET && value == SOFTRESET_VALUE) {
		reset_registers(chip);
		apply_rate(chip);
		configure_filter(chip);
		fifo_restart(chip);
		return;
	}
	if (address == REG_CHIP_ID || address == REG_FIFO_STATUS ||
	    address == REG_FIFO_DATA) {
		return;
	}
	if ((address == REG_BW || address == REG_HBW) &&
	    chip->registers[address & 0x3fU] != value) {
		materialize_fifo(chip);
	}
	chip->registers[address & 0x3fU] = value;
	if (address == REG_BW || address == REG_HBW) {
		apply_rate(chip);
		configure_filter(chip);
	}
	if (address == fifo_config_0 || address == REG_FIFO_CONFIG_1 ||
	    (!was_on && fifo_on(chip))) {
		chip->registers[REG_FIFO_DATA] = 0U;
		fifo_restart(chip);
	}
	/* the chip produces samples once its FIFO runs, with or without its interrupt (ArduPilot
	 * polls the FIFO): from then on, the value source learns the period and each change */
	if (fifo_on(chip)) {
		report_period(chip);
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
	apply_rate(chip);
	configure_filter(chip);
	chip->started = FALSE;
	chip->reading = FALSE;
	chip->address = 0U;
	fifo_restart(chip);
	if (chip->data_ready_resolved) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready, TRUE);
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Bmi055Kind kind = strcmp(args->node->matched_compatible,
		"bosch,bma280") == 0 ? BMI055_ACCEL : BMI055_GYRO;
	uint32 capacity = kind == BMI055_ACCEL ? 32U : 100U;
	Bmi055 *chip = calloc(1U, sizeof(*chip) + capacity * 6U);
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
	chip->kind = kind;
	chip->fifo_materialized = (uint8 *)(chip + 1);
	chip->frequency_hz = frequency;
	reset_registers(chip);
	chip->clocks_per_sample = chip_rate_clocks(frequency, rate_mhz(chip));
	chip->ideal_filters = args->node->ops->get_property(
		args->node->context, "athrill,ideal-filters", &ideal,
		&ideal_size) == 0;
	configure_filter(chip);
	if (args->node->ops->get_string(args->node->context,
		"athrill,mount-rotation", 0U, &rotation) == 0 &&
	    rotation != NULL) {
		if (strcmp(rotation, "yaw-90") == 0) {
			chip->yaw_quarter_turns = 1U;
		}
		else if (strcmp(rotation, "yaw-180") == 0) {
			chip->yaw_quarter_turns = 2U;
		}
		else if (strcmp(rotation, "yaw-270") == 0) {
			chip->yaw_quarter_turns = 3U;
		}
		else {
			free(chip);
			return -1;
		}
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
	Bmi055 *chip = instance;
	uint8 config = chip->registers[chip->kind == BMI055_ACCEL ?
		REG_ACC_FIFO_CONFIG_0 : REG_GYR_FIFO_CONFIG_0];
	uint32 watermark = config & (chip->kind == BMI055_ACCEL ? 0x3fU : 0x7fU);
	uint64 period;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready, TRUE);
	if (!fifo_on(chip) || !interrupt_enabled(chip)) {
		return 0;
	}
	report_period(chip);
	if (watermark == 0U) {
		watermark = 1U;
	}
	period = chip->clocks_per_sample * watermark;
	if (chip->next_data_ready == 0U) {
		chip->next_data_ready = (now / period + 1U) * period;
	}
	if (now >= chip->next_data_ready) {
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready, FALSE);
		(void)chip->gpio_resources->gpio_drive(chip->gpio_context, chip->data_ready, TRUE);
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
