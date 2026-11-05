#include "hako_sensor_device.h"

#include <string.h>

#define REG(offset) ((offset) / 4U)

static uint32_t float_bits(float value)
{
	uint32_t bits;

	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

static float bits_float(uint32_t bits)
{
	float value;

	memcpy(&value, &bits, sizeof(value));
	return value;
}

static void set_time(HakoSensorDevice *device, uint32_t lo_offset, uint32_t hi_offset, uint64_t time_usec)
{
	device->regs[REG(lo_offset)] = (uint32_t)time_usec;
	device->regs[REG(hi_offset)] = (uint32_t)(time_usec >> 32U);
}

/*
 * Set STATUS bits; returns those that are enabled. Every new sample raises
 * the interrupt again, also when the driver has not cleared its bit yet, so
 * that a late driver is woken up anyway.
 */
static uint32_t raise_status(HakoSensorDevice *device, uint32_t bits)
{
	device->regs[REG(HAKO_REG_STATUS)] |= bits;
	return bits & device->regs[REG(HAKO_REG_IRQ_ENABLE)];
}

void hako_sensor_device_reset(HakoSensorDevice *device)
{
	memset(device, 0, sizeof(*device));
	device->regs[REG(HAKO_REG_ID)] = HAKO_SENSOR_DEVICE_ID_VALUE;
	device->regs[REG(HAKO_REG_VERSION)] = HAKO_SENSOR_DEVICE_VERSION_VALUE;
}

uint32_t hako_sensor_device_read(const HakoSensorDevice *device, uint32_t offset)
{
	if ((offset >= HAKO_SENSOR_DEVICE_SIZE) || ((offset & 3U) != 0U)) {
		return 0U;
	}
	return device->regs[REG(offset)];
}

void hako_sensor_device_write(HakoSensorDevice *device, uint32_t offset, uint32_t value)
{
	uint32_t n;

	if ((offset >= HAKO_SENSOR_DEVICE_SIZE) || ((offset & 3U) != 0U)) {
		return;
	}
	if (offset == HAKO_REG_STATUS) {
		device->regs[REG(offset)] &= ~value;
	}
	else if (offset == HAKO_REG_IRQ_ENABLE) {
		device->regs[REG(offset)] = value & HAKO_STATUS_ALL;
	}
	else if ((offset >= HAKO_REG_CONTROL(0)) && (offset <= HAKO_REG_FLAGS_HI)) {
		device->regs[REG(offset)] = value;
	}
	else if (offset == HAKO_REG_COMMIT) {
		for (n = 0U; n < HAKO_ACTUATOR_CHANNELS; n++) {
			device->committed.controls[n] = bits_float(device->regs[REG(HAKO_REG_CONTROL(n))]);
		}
		device->committed.mode = (uint8_t)device->regs[REG(HAKO_REG_MODE)];
		device->committed.flags = (uint64_t)device->regs[REG(HAKO_REG_FLAGS_LO)] |
			((uint64_t)device->regs[REG(HAKO_REG_FLAGS_HI)] << 32U);
		device->has_committed = true;
	}
	/* Other registers are read-only. */
}

uint32_t hako_sensor_device_latch_sensor(HakoSensorDevice *device, const HakoSensorSample *sample)
{
	uint32_t status = 0U;
	uint32_t i;

	set_time(device, HAKO_REG_SENSOR_TIME_LO, HAKO_REG_SENSOR_TIME_HI, sample->time_usec);
	device->regs[REG(HAKO_REG_FIELDS_UPDATED)] = sample->fields_updated;
	for (i = 0U; i < 3U; i++) {
		device->regs[REG(HAKO_REG_ACC_X) + i] = float_bits(sample->acc[i]);
		device->regs[REG(HAKO_REG_GYRO_X) + i] = float_bits(sample->gyro[i]);
		device->regs[REG(HAKO_REG_MAG_X) + i] = float_bits(sample->mag[i]);
	}
	device->regs[REG(HAKO_REG_ABS_PRESSURE)] = float_bits(sample->abs_pressure);
	device->regs[REG(HAKO_REG_DIFF_PRESSURE)] = float_bits(sample->diff_pressure);
	device->regs[REG(HAKO_REG_PRESSURE_ALT)] = float_bits(sample->pressure_alt);
	device->regs[REG(HAKO_REG_TEMPERATURE)] = float_bits(sample->temperature);
	if ((sample->fields_updated & HAKO_FIELDS_IMU) != 0U) {
		status |= HAKO_STATUS_IMU;
	}
	if ((sample->fields_updated & HAKO_FIELDS_MAG) != 0U) {
		status |= HAKO_STATUS_MAG;
	}
	if ((sample->fields_updated & HAKO_FIELDS_BARO) != 0U) {
		status |= HAKO_STATUS_BARO;
	}
	return raise_status(device, status);
}

uint32_t hako_sensor_device_latch_gps(HakoSensorDevice *device, const HakoGpsSample *sample)
{
	set_time(device, HAKO_REG_GPS_TIME_LO, HAKO_REG_GPS_TIME_HI, sample->time_usec);
	device->regs[REG(HAKO_REG_GPS_LAT)] = (uint32_t)sample->lat;
	device->regs[REG(HAKO_REG_GPS_LON)] = (uint32_t)sample->lon;
	device->regs[REG(HAKO_REG_GPS_ALT)] = (uint32_t)sample->alt;
	device->regs[REG(HAKO_REG_GPS_EPH)] = sample->eph;
	device->regs[REG(HAKO_REG_GPS_EPV)] = sample->epv;
	device->regs[REG(HAKO_REG_GPS_VEL)] = sample->vel;
	device->regs[REG(HAKO_REG_GPS_VN)] = (uint32_t)(int32_t)sample->vn;
	device->regs[REG(HAKO_REG_GPS_VE)] = (uint32_t)(int32_t)sample->ve;
	device->regs[REG(HAKO_REG_GPS_VD)] = (uint32_t)(int32_t)sample->vd;
	device->regs[REG(HAKO_REG_GPS_COG)] = sample->cog;
	device->regs[REG(HAKO_REG_GPS_YAW)] = sample->yaw;
	device->regs[REG(HAKO_REG_GPS_FIX_TYPE)] = sample->fix_type;
	device->regs[REG(HAKO_REG_GPS_SATELLITES)] = sample->satellites_visible;
	device->regs[REG(HAKO_REG_GPS_ID)] = sample->id;
	return raise_status(device, HAKO_STATUS_GPS);
}

bool hako_sensor_device_committed(const HakoSensorDevice *device, HakoActuatorSample *sample)
{
	if (!device->has_committed) {
		return false;
	}
	*sample = device->committed;
	return true;
}
