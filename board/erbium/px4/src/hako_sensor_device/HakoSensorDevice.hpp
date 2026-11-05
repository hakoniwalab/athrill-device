/*
 * PX4 driver of the Hakoniwa sensor device (Athrill only), see
 * README.md and
 * board/common/include/hako_sensor_device_regs.h.
 *
 * On the data-ready interrupt it reads the registers and passes the samples
 * to PX4 like a sensor chip driver does (PX4Accelerometer, PX4Gyroscope,
 * PX4Magnetometer, PX4Barometer, sensor_gnss). It also writes PX4's
 * simulation outputs (actuator_outputs_sim, from pwm_out_sim) to the motor
 * registers, as PX4 SITL builds HIL_ACTUATOR_CONTROLS.
 */
#pragma once

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/vehicle_status.h>

class HakoSensorDevice : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	HakoSensorDevice();
	~HakoSensorDevice() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	static int DataReadyInterrupt(int irq, void *context, void *arg);

	void publishSensors(uint32_t status, hrt_abstime timestamp);
	void publishGps(hrt_abstime timestamp);
	void writeOutputs();

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
	PX4Magnetometer _px4_mag;
	PX4Barometer _px4_baro;
	uORB::PublicationMulti<sensor_gnss_s> _sensor_gnss_pub{ORB_ID(sensor_gnss)};

	uORB::SubscriptionCallbackWorkItem _actuator_outputs_sub{this, ORB_ID(actuator_outputs_sim)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};

	perf_counter_t _interrupt_perf{perf_alloc(PC_COUNT, MODULE_NAME": interrupt")};
	perf_counter_t _imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": imu")};
	perf_counter_t _gps_perf{perf_alloc(PC_COUNT, MODULE_NAME": gps")};
	perf_counter_t _output_perf{perf_alloc(PC_COUNT, MODULE_NAME": output")};
};
