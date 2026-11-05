#include "HakoSensorDevice.hpp"

#include <arch/irq.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <nuttx/irq.h>
#include <px4_platform_common/log.h>

#include <math.h>
#include <string.h>

#include "hako_sensor_device_regs.h"

ModuleBase::Descriptor HakoSensorDevice::desc{task_spawn, custom_command, print_usage};

namespace
{

constexpr int DEVICE_IRQ = RISCV_IRQ_MEXT + HAKO_SENSOR_DEVICE_PLIC_SOURCE;

/* The same MAVLink HIL_ACTUATOR_CONTROLS mode bits as PX4 SITL (SimulatorMavlink). */
constexpr uint32_t MODE_FLAG_CUSTOM = 1;
constexpr uint32_t MODE_FLAG_ARMED = 128;
constexpr uint32_t FLAGS_LOCKSTEP = 1;

volatile uint32_t &reg(uint32_t offset)
{
	return *reinterpret_cast<volatile uint32_t *>(HAKO_SENSOR_DEVICE_BASE + offset);
}

float reg_float(uint32_t offset)
{
	const uint32_t bits = reg(offset);
	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

void write_float(uint32_t offset, float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	reg(offset) = bits;
}

uint32_t device_id(uint8_t devtype)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.bus = 1;	/* as PX4 SITL simulated sensors: bus 1, address 1 */
	id.devid_s.address = 1;
	id.devid_s.devtype = devtype;
	return id.devid;
}

} // namespace

HakoSensorDevice::HakoSensorDevice() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_accel(device_id(DRV_IMU_DEVTYPE_SIM)),
	_px4_gyro(device_id(DRV_IMU_DEVTYPE_SIM)),
	_px4_mag(device_id(DRV_MAG_DEVTYPE_MAGSIM)),
	_px4_baro(device_id(DRV_BARO_DEVTYPE_BAROSIM))
{
}

HakoSensorDevice::~HakoSensorDevice()
{
	up_disable_irq(DEVICE_IRQ);
	irq_detach(DEVICE_IRQ);
	perf_free(_interrupt_perf);
	perf_free(_imu_perf);
	perf_free(_gps_perf);
	perf_free(_output_perf);
}

bool HakoSensorDevice::init()
{
	if ((reg(HAKO_REG_ID) != HAKO_SENSOR_DEVICE_ID_VALUE) || (reg(HAKO_REG_VERSION) != HAKO_SENSOR_DEVICE_VERSION_VALUE)) {
		PX4_ERR("no Hakoniwa sensor device at 0x%08lx", (unsigned long)HAKO_SENSOR_DEVICE_BASE);
		return false;
	}

	if (!_actuator_outputs_sub.registerCallback()) {
		PX4_ERR("actuator_outputs_sim callback registration failed");
		return false;
	}

	reg(HAKO_REG_STATUS) = HAKO_STATUS_ALL;
	irq_attach(DEVICE_IRQ, &HakoSensorDevice::DataReadyInterrupt, this);
	up_enable_irq(DEVICE_IRQ);
	reg(HAKO_REG_IRQ_ENABLE) = HAKO_STATUS_ALL;
	return true;
}

int HakoSensorDevice::DataReadyInterrupt(int irq, void *context, void *arg)
{
	HakoSensorDevice *self = static_cast<HakoSensorDevice *>(arg);
	perf_count(self->_interrupt_perf);
	self->ScheduleNow();
	return 0;
}

void HakoSensorDevice::Run()
{
	if (should_exit()) {
		_actuator_outputs_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	/*
	 * Athrill latched the sample at its simulated time and raised the
	 * interrupt there, so PX4's time now is the sample time (the device's
	 * time_usec is the same clock).
	 */
	const hrt_abstime now = hrt_absolute_time();
	const uint32_t status = reg(HAKO_REG_STATUS);

	if (status != 0) {
		reg(HAKO_REG_STATUS) = status;
		publishSensors(status, now);

		if (status & HAKO_STATUS_GPS) {
			publishGps(now);
		}
	}

	if (_actuator_outputs_sub.updated()) {
		writeOutputs();
	}
}

void HakoSensorDevice::publishSensors(uint32_t status, hrt_abstime timestamp)
{
	const float temperature = reg_float(HAKO_REG_TEMPERATURE);

	if (status & HAKO_STATUS_IMU) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
		_px4_accel.update(timestamp, reg_float(HAKO_REG_ACC_X), reg_float(HAKO_REG_ACC_Y), reg_float(HAKO_REG_ACC_Z));
		_px4_gyro.update(timestamp, reg_float(HAKO_REG_GYRO_X), reg_float(HAKO_REG_GYRO_Y), reg_float(HAKO_REG_GYRO_Z));
		perf_count(_imu_perf);
	}

	if (status & HAKO_STATUS_MAG) {
		_px4_mag.set_temperature(temperature);
		_px4_mag.update(timestamp, reg_float(HAKO_REG_MAG_X), reg_float(HAKO_REG_MAG_Y), reg_float(HAKO_REG_MAG_Z));
	}

	if (status & HAKO_STATUS_BARO) {
		_px4_baro.set_temperature(temperature);
		_px4_baro.update(timestamp, reg_float(HAKO_REG_ABS_PRESSURE) * 100.f);	/* hPa -> Pa */
	}
}

void HakoSensorDevice::publishGps(hrt_abstime timestamp)
{
	sensor_gnss_s gnss{};
	const float vn = (float)(int32_t)reg(HAKO_REG_GPS_VN) * 0.01f;
	const float ve = (float)(int32_t)reg(HAKO_REG_GPS_VE) * 0.01f;
	const float vd = (float)(int32_t)reg(HAKO_REG_GPS_VD) * 0.01f;

	gnss.timestamp_sample = timestamp;
	gnss.device_id = device_id(DRV_GPS_DEVTYPE_SIM);
	gnss.latitude = (double)(int32_t)reg(HAKO_REG_GPS_LAT) * 1e-7;
	gnss.longitude = (double)(int32_t)reg(HAKO_REG_GPS_LON) * 1e-7;
	gnss.altitude_msl = (double)(int32_t)reg(HAKO_REG_GPS_ALT) * 1e-3;
	gnss.altitude_ellipsoid = gnss.altitude_msl;
	gnss.eph = (float)reg(HAKO_REG_GPS_EPH) * 0.01f;
	gnss.epv = (float)reg(HAKO_REG_GPS_EPV) * 0.01f;
	gnss.hdop = gnss.eph;
	gnss.vdop = gnss.epv;
	gnss.fix_type = (uint8_t)reg(HAKO_REG_GPS_FIX_TYPE);
	gnss.satellites_used = (uint8_t)reg(HAKO_REG_GPS_SATELLITES);
	gnss.vel_north = vn;
	gnss.vel_east = ve;
	gnss.vel_down = vd;
	gnss.ground_speed = sqrtf(vn * vn + ve * ve);
	gnss.course = atan2f(ve, vn);
	gnss.vel_ned_valid = true;
	gnss.speed_accuracy = 0.4f;
	gnss.course_accuracy = 0.1f;
	gnss.timestamp = hrt_absolute_time();
	_sensor_gnss_pub.publish(gnss);
	perf_count(_gps_perf);
}

void HakoSensorDevice::writeOutputs()
{
	actuator_outputs_s outputs;
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	if (!_actuator_outputs_sub.copy(&outputs)) {
		return;
	}

	/* As PX4 SITL builds HIL_ACTUATOR_CONTROLS: the outputs while armed, else zero. */
	for (unsigned i = 0; i < HAKO_ACTUATOR_CHANNELS; i++) {
		write_float(HAKO_REG_CONTROL(i), (_armed && i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) ? outputs.output[i] : 0.f);
	}

	reg(HAKO_REG_MODE) = MODE_FLAG_CUSTOM | (_armed ? MODE_FLAG_ARMED : 0);
	reg(HAKO_REG_FLAGS_LO) = FLAGS_LOCKSTEP;
	reg(HAKO_REG_FLAGS_HI) = 0;
	reg(HAKO_REG_COMMIT) = 1;
	perf_count(_output_perf);
}

int HakoSensorDevice::print_status()
{
	perf_print_counter(_interrupt_perf);
	perf_print_counter(_imu_perf);
	perf_print_counter(_gps_perf);
	perf_print_counter(_output_perf);
	return 0;
}

int HakoSensorDevice::task_spawn(int argc, char *argv[])
{
	HakoSensorDevice *instance = new HakoSensorDevice();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int HakoSensorDevice::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HakoSensorDevice::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver of the Hakoniwa sensor device of Athrill: IMU, magnetometer, barometer
and GPS from the device registers, PX4's simulation outputs to its motor registers.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hako_sensor_device", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int hako_sensor_device_main(int argc, char *argv[])
{
	return ModuleBase::main(HakoSensorDevice::desc, argc, argv);
}
