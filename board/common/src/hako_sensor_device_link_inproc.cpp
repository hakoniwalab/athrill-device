#include "hako_sensor_device_link.h"
#include "hako_sensor_device_link_mavlink.h"

#include "service.hpp"
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
#include "hakoniwa/sensor_adapter/host/sensor_host.hpp"
#endif

#include <cmath>
#include <condition_variable>
#include <array>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using hako::aircraft::IAirCraftContainer;
using hako::config::DroneConfig;
using hako::config::DroneConfigManager;
using hako::mavlink::IMavLinkService;
using hako::mavlink::IMavLinkServiceCallback;
using hako::mavlink::MavlinkHakoMessage;
using hako::mavlink::MavlinkMsgType;
using hako::mavlink::MavLinkServiceDesitinationType;
using hako::mavlink::MavLinkServiceContainer;
using hako::mavlink::MavLinkUserCustomDecoderType;
using hako::mavlink::MavlinkServiceIoType;
using hako::mavlink::IMavlinkCommEndpointType;
using hako::service::IAircraftServiceContainer;
using hako::service::IServicePduSyncher;
using hako::service::ServicePduDataType;

class InProcessMavLinkService final : public IMavLinkService {
public:
	bool addMavProxyClient(MavlinkServiceIoType, const IMavlinkCommEndpointType&) override
	{
		return false;
	}

	bool setReceiverCallback(IMavLinkServiceCallback& callback) override
	{
		callback_ = &callback;
		return true;
	}

	bool sendMessage(MavLinkServiceDesitinationType, MavlinkHakoMessage& message) override
	{
		return storeMessage(message);
	}

	bool sendMessage(MavlinkHakoMessage& message) override
	{
		return storeMessage(message);
	}

	bool readMessage(MavlinkHakoMessage& message, bool& is_dirty) override
	{
		if (message.type != MavlinkMsgType::HIL_ACTUATOR_CONTROLS || !has_actuator_) {
			is_dirty = false;
			return false;
		}
		message = actuator_;
		is_dirty = actuator_dirty_;
		actuator_dirty_ = false;
		return true;
	}

	bool startService() override
	{
		started_ = true;
		return true;
	}

	void stopService() override
	{
		started_ = false;
	}

	bool setUserCustomDecoder(MavLinkUserCustomDecoderType) override
	{
		return true;
	}

	bool deliverActuator(const HakoActuatorSample& actuator)
	{
		if (!started_ || callback_ == nullptr) {
			return false;
		}
		actuator_ = {};
		actuator_.type = MavlinkMsgType::HIL_ACTUATOR_CONTROLS;
		hako_sensor_link_encode_actuator(&actuator, &actuator_.data.hil_actuator_controls);
		has_actuator_ = true;
		actuator_dirty_ = true;
		callback_->onReceiveMessage(0);
		return true;
	}

	bool readSensor(Hako_HakoHilSensor& sensor, Hako_HakoHilGps& gps, bool& has_gps) const
	{
		if (!has_sensor_) {
			return false;
		}
		sensor = sensor_;
		has_gps = has_gps_;
		if (has_gps) {
			gps = gps_;
		}
		return true;
	}

private:
	bool storeMessage(const MavlinkHakoMessage& message)
	{
		switch (message.type) {
		case MavlinkMsgType::HIL_SENSOR:
			sensor_ = message.data.hil_sensor;
			has_sensor_ = true;
			return true;
		case MavlinkMsgType::HIL_GPS:
			gps_ = message.data.hil_gps;
			has_gps_ = true;
			return true;
		default:
			return true;
		}
	}

	IMavLinkServiceCallback *callback_{nullptr};
	MavlinkHakoMessage actuator_{};
	Hako_HakoHilSensor sensor_{};
	Hako_HakoHilGps gps_{};
	bool started_{false};
	bool has_actuator_{false};
	bool actuator_dirty_{false};
	bool has_sensor_{false};
	bool has_gps_{false};
};

class NoopPduSyncher final : public IServicePduSyncher {
public:
	bool flush(uint32_t, ServicePduDataType&) override { return true; }
	bool load(uint32_t, ServicePduDataType&) override { return true; }
	bool reset_data() override { return true; }
	void sync_time(uint64_t) override {}
};

struct InProcessLink {
	DroneConfigManager config_manager;
	std::shared_ptr<IAirCraftContainer> aircraft_container;
	std::shared_ptr<InProcessMavLinkService> mavlink_service;
	std::shared_ptr<MavLinkServiceContainer> mavlink_container;
	std::shared_ptr<IAircraftServiceContainer> aircraft_service;
	std::shared_ptr<NoopPduSyncher> pdu_syncher;
	HakoActuatorSample actuator{};
	uint64_t physics_step_usec{0};
	uint64_t sensor_period_usec{0};
	uint64_t gps_time_usec{0};
	bool has_actuator{false};
	bool worker_enabled{true};
	std::mutex worker_mutex;
	std::condition_variable worker_condition;
	std::thread worker_thread;
	uint64_t worker_after_usec{0};
	bool worker_pending{false};
	bool worker_sample_ready{false};
	bool worker_stop{false};
	bool worker_failed{false};
	/* the rotors as the aircraft left them with the sample being prepared,
	 * and with the sample the device read last (hako_sensor_link_read) */
	std::vector<HakoMotorStatusSample> worker_motors;
	std::vector<HakoMotorStatusSample> read_motors;
	HakoVehiclePoseSample worker_pose{};
	HakoVehiclePoseSample read_pose{};
	bool has_pose{false};
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	/* the vehicle's true motion after the last physics step (athrill-device#129) */
	hakoniwa::sensor_adapter::host::RigidBodyMotion motion{};
	hakoniwa::sensor_adapter::host::ImuVibration vibration{};
	bool has_motion{false};
	FILE *truth_log{nullptr};
#endif
};

std::unique_ptr<InProcessLink> link;

uint64_t usecFromMsec(const std::optional<uint64_t>& msec)
{
	return msec ? *msec * 1000U : 0U;
}

bool configureSensorPeriods(InProcessLink& state, DroneConfig& config)
{
	const auto acc_msec = config.getCompSensorUpdatePeriodMsec("acc");
	const auto gyro_msec = config.getCompSensorUpdatePeriodMsec("gyro");
	if (acc_msec.has_value() != gyro_msec.has_value()
		|| (acc_msec && *acc_msec != *gyro_msec)) {
		std::cerr << "ERROR: hako_sensor_device: acc and gyro update periods must match" << std::endl;
		return false;
	}

	const auto sitl = config.getSimSitlConfig();
	state.sensor_period_usec = acc_msec ? *acc_msec * 1000U : sitl.sensor_period_msec * 1000U;
	if (state.sensor_period_usec != 0U
		&& !state.aircraft_service->setSitlSensorPeriodUsec(0U, state.sensor_period_usec)) {
		return false;
	}
	return state.aircraft_service->setSitlSensorUpdatePeriodsUsec(
		0U,
		usecFromMsec(config.getCompSensorUpdatePeriodMsec("baro")),
		usecFromMsec(config.getCompSensorUpdatePeriodMsec("mag")),
		usecFromMsec(config.getCompSensorUpdatePeriodMsec("gps")));
}

bool loadAircraftConfig(InProcessLink& state, const std::string& path)
{
	if (std::filesystem::is_directory(path)) {
		state.config_manager.loadConfigsFromDirectory(path);
	}
	else if (!state.config_manager.loadConfigFromFile(path)) {
		return false;
	}
	if (state.config_manager.getConfigCount() != 1) {
		std::cerr << "ERROR: hako_sensor_device: in-process mode needs exactly one aircraft config" << std::endl;
		return false;
	}
	return true;
}

#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
namespace host = hakoniwa::sensor_adapter::host;

/* Drone PRO's world NED and body FRD to the sensor adapters' world NWU and body FLU. */
hakoniwa::sensor_adapter::Vector3 nedToNwu(double x, double y, double z)
{
	return {x, -y, -z};
}

/* The vehicle's true motion now, as the host helpers take it (sensor spec 2.4:
 * the body origin's motion; MuJoCo's free-joint body). The world acceleration
 * is used, not get_acc_body_frame (that is d(v_body)/dt, without w x v). */
void captureTruth(InProcessLink& state)
{
	auto aircraft = state.aircraft_container->getAirCraft(0U);
	if (aircraft == nullptr) {
		return;
	}
	auto& dynamics = aircraft->get_drone_dynamics();
	const auto pos = dynamics.get_pos();
	const auto vel = dynamics.get_vel();
	const auto acc = dynamics.get_acc();
	const auto angle = dynamics.get_angle();
	const auto rate = dynamics.get_angular_vel_body_frame();
	const auto alpha = dynamics.get_angular_acc_body_frame();
	constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
	state.motion.position_nwu = nedToNwu(pos.data.x, pos.data.y, pos.data.z);
	state.motion.velocity_nwu = nedToNwu(vel.data.x, vel.data.y, vel.data.z);
	state.motion.linear_acceleration_nwu = nedToNwu(acc.data.x, acc.data.y, acc.data.z);
	state.motion.angular_velocity_body = nedToNwu(rate.data.x, rate.data.y, rate.data.z);
	state.motion.angular_acceleration_body = nedToNwu(alpha.data.x, alpha.data.y, alpha.data.z);
	/* The body's attitude in NWU/FLU (roll kept, pitch and yaw negated, as capturePose);
	 * mount_from_rpy_deg's quaternion is the coordinate rotation from the outer frame
	 * (here the world) into the rotated one (here the body). */
	state.motion.orientation_world_to_body = host::mount_from_rpy_deg({}, angle.data.x * kRadToDeg,
		-angle.data.y * kRadToDeg, -angle.data.z * kRadToDeg).orientation_body_to_sensor;
	std::array<double, 3> vib_acc{};
	std::array<double, 3> vib_gyro{};
	if (aircraft->get_rotor_vibration(vib_acc, vib_gyro)) {
		state.vibration.acceleration_body = nedToNwu(vib_acc[0], vib_acc[1], vib_acc[2]);
		state.vibration.angular_velocity_body = nedToNwu(vib_gyro[0], vib_gyro[1], vib_gyro[2]);
	}
	state.has_motion = true;
}

/* The truth check: Drone PRO's accelerometer and gyro (its sensor models, FRD) beside
 * the host helpers' ideal IMU at the body origin from the captured truth (FRD). With an
 * error-free airframe the two must agree. */
void logTruth(InProcessLink& state, const Hako_HakoHilSensor& sensor)
{
	if (state.truth_log == nullptr || !state.has_motion) {
		return;
	}
	const host::World world{};
	const auto imu = host::imu_truth(0U, state.motion, host::MountPose{}, world, state.vibration);
	std::fprintf(state.truth_log,
		"%llu %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
		static_cast<unsigned long long>(sensor.time_usec),
		sensor.xacc, sensor.yacc, sensor.zacc, sensor.xgyro, sensor.ygyro, sensor.zgyro,
		imu.specific_force.x, -imu.specific_force.y, -imu.specific_force.z,
		imu.angular_velocity.x, -imu.angular_velocity.y, -imu.angular_velocity.z);
}
#endif

bool stepUntilSensor(InProcessLink& state, uint64_t last_time_usec)
{
	Hako_HakoHilSensor sensor{};
	Hako_HakoHilGps gps{};
	bool has_gps = false;
	const uint64_t period = state.sensor_period_usec != 0U
		? state.sensor_period_usec
		: state.physics_step_usec * 3U;
	const uint64_t max_steps = (period + state.physics_step_usec - 1U) / state.physics_step_usec + 1U;

	for (uint64_t step = 0U; step < max_steps; step++) {
		if (state.mavlink_service->readSensor(sensor, gps, has_gps)
			&& sensor.time_usec > last_time_usec) {
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
			logTruth(state, sensor);
#endif
			return true;
		}
		if (state.has_actuator) {
			if (!state.mavlink_service->deliverActuator(state.actuator)) {
				return false;
			}
		}
		else if (!state.aircraft_service->flushSensorData(0U)) {
			return false;
		}
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
		captureTruth(state);
#endif
	}
	return state.mavlink_service->readSensor(sensor, gps, has_gps)
		&& sensor.time_usec > last_time_usec;
}

/* The vehicle's pose now, in Hakoniwa's world frame (as the aircraft
 * service's pos PDU: NED to ROS, y and z and their angles negated). */
HakoVehiclePoseSample capturePose(InProcessLink& state)
{
	HakoVehiclePoseSample pose{};
	auto aircraft = state.aircraft_container->getAirCraft(0U);
	if (aircraft != nullptr) {
		const auto pos = aircraft->get_drone_dynamics().get_pos();
		const auto angle = aircraft->get_drone_dynamics().get_angle();
		pose.x = pos.data.x;
		pose.y = -pos.data.y;
		pose.z = -pos.data.z;
		pose.roll = angle.data.x;
		pose.pitch = -angle.data.y;
		pose.yaw = -angle.data.z;
	}
	return pose;
}

/* The aircraft's rotors now, in the thread that steps it. */
std::vector<HakoMotorStatusSample> captureMotors(InProcessLink& state)
{
	std::vector<HakoMotorStatusSample> motors;
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
	auto aircraft = state.aircraft_container->getAirCraft(0U);
	if (aircraft == nullptr) {
		return motors;
	}
	for (std::size_t i = 0U; i < aircraft->get_rotor_count(); ++i) {
		hako::aircraft::RotorStatusType rotor{};
		HakoMotorStatusSample motor{};
		if (!aircraft->get_rotor_status(i, rotor)) {
			break;
		}
		motor.rpm = static_cast<float>(rotor.rpm);
		motor.current_a = static_cast<float>(rotor.current);
		motor.voltage_v = static_cast<float>(rotor.voltage);
		motor.duty = static_cast<float>(rotor.duty);
		motor.has_current = rotor.has_current;
		motors.push_back(motor);
	}
#else
	(void)state;
#endif
	return motors;
}

void runAircraftWorker(InProcessLink *state)
{
	std::unique_lock<std::mutex> lock(state->worker_mutex);
	for (;;) {
		state->worker_condition.wait(lock, [state] {
			return state->worker_stop || state->worker_pending;
		});
		if (state->worker_stop) {
			return;
		}

		const uint64_t last_time_usec = state->worker_after_usec;
		lock.unlock();
		const bool success = stepUntilSensor(*state, last_time_usec);
		std::vector<HakoMotorStatusSample> motors = captureMotors(*state);
		const HakoVehiclePoseSample pose = capturePose(*state);
		lock.lock();
		state->worker_motors = std::move(motors);
		state->worker_pose = pose;

		state->worker_pending = false;
		state->worker_sample_ready = success;
		state->worker_failed = !success;
		state->worker_condition.notify_all();
	}
}

bool readWorkerSample(
	InProcessLink& state, uint64_t last_time_usec,
	HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps)
{
	std::unique_lock<std::mutex> lock(state.worker_mutex);
	state.worker_condition.wait(lock, [&state] {
		return state.worker_stop || state.worker_failed || state.worker_sample_ready;
	});
	if (state.worker_stop || state.worker_failed) {
		return false;
	}

	Hako_HakoHilSensor source_sensor{};
	Hako_HakoHilGps source_gps{};
	bool source_has_gps = false;
	if (!state.mavlink_service->readSensor(source_sensor, source_gps, source_has_gps)
		|| source_sensor.time_usec <= last_time_usec) {
		return false;
	}
	hako_sensor_link_decode_sensor(&source_sensor, sensor);
	*has_gps = source_has_gps && source_gps.time_usec != state.gps_time_usec;
	if (*has_gps) {
		state.gps_time_usec = source_gps.time_usec;
		hako_sensor_link_decode_gps(&source_gps, gps);
	}
	state.read_motors = state.worker_motors;	/* with this sample */
	state.read_pose = state.worker_pose;
	state.has_pose = true;
	state.worker_sample_ready = false;
	return true;
}

void stopAircraftWorker(InProcessLink& state)
{
	if (!state.worker_enabled || !state.worker_thread.joinable()) {
		return;
	}
	{
		std::lock_guard<std::mutex> lock(state.worker_mutex);
		state.worker_stop = true;
		state.worker_condition.notify_all();
	}
	state.worker_thread.join();
}

}  // namespace

extern "C" bool hako_sensor_link_initialize(const HakoSensorLinkConfig *config)
{
	if (config == nullptr || config->aircraft_config_path == nullptr || link) {
		return false;
	}
	try {
		auto state = std::make_unique<InProcessLink>();
		state->worker_enabled = config->aircraft_worker;
		if (!loadAircraftConfig(*state, config->aircraft_config_path)) {
			return false;
		}
		DroneConfig& aircraft_config = state->config_manager.getConfig(0U);
		if (config->physics_step_usec != 0U) {
			/* before the aircraft is made: every part of it takes its step from here */
			aircraft_config.setSimTimeStep(static_cast<double>(config->physics_step_usec) / 1000000.0);
		}
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
		if (config->truth_log_path != nullptr) {
			std::cout << "INFO: hako_sensor_device: truth check written to " << config->truth_log_path << std::endl;
		}
		if (config->truth_log_path != nullptr) {
			state->truth_log = std::fopen(config->truth_log_path, "w");
			if (state->truth_log == nullptr) {
				std::cerr << "ERROR: hako_sensor_device: can not write " << config->truth_log_path << std::endl;
				return false;
			}
			std::fprintf(state->truth_log,
				"# time_usec hil_acc_x y z hil_gyro_x y z host_acc_x y z host_gyro_x y z (FRD)\n");
		}
#endif
		const double step_usec = aircraft_config.getSimTimeStep() * 1000000.0;
		state->physics_step_usec = static_cast<uint64_t>(std::llround(step_usec));
		if (state->physics_step_usec == 0U
			|| std::fabs(step_usec - static_cast<double>(state->physics_step_usec)) > 1e-6) {
			std::cerr << "ERROR: hako_sensor_device: simulation.timeStep must be whole microseconds" << std::endl;
			return false;
		}

		state->aircraft_container = IAirCraftContainer::create();
		state->aircraft_container->createAirCrafts(state->config_manager);
		state->mavlink_service = std::make_shared<InProcessMavLinkService>();
		state->mavlink_container = std::make_shared<MavLinkServiceContainer>();
		state->mavlink_container->addService(state->mavlink_service);
		state->aircraft_service = IAircraftServiceContainer::create(
			state->mavlink_container, state->aircraft_container);
		state->pdu_syncher = std::make_shared<NoopPduSyncher>();
		state->aircraft_service->setPduSyncher(state->pdu_syncher);
		state->aircraft_service->setSitlConfig(aircraft_config.getSimSitlConfig());
		state->aircraft_service->setRealTimeStepUsec(0U);
		if (!configureSensorPeriods(*state, aircraft_config)) {
			std::cerr << "ERROR: hako_sensor_device: invalid sensor update periods" << std::endl;
			return false;
		}
		state->aircraft_service->enableReceiveEvent(0U);
		if (!state->aircraft_service->startService(true, state->physics_step_usec)) {
			return false;
		}
		if (state->worker_enabled) {
			/* The worker also prepares the initial sample, before any actuator. */
			state->worker_pending = true;
			InProcessLink *worker_state = state.get();
			state->worker_thread = std::thread([worker_state] {
				runAircraftWorker(worker_state);
			});
		}
		link = std::move(state);
		return true;
	}
	catch (const std::exception& error) {
		std::cerr << "ERROR: hako_sensor_device: can not create in-process aircraft: "
			<< error.what() << std::endl;
		return false;
	}
}

extern "C" bool hako_sensor_link_poll_events(void)
{
	return link != nullptr;
}

extern "C" bool hako_sensor_link_read(
	uint64_t last_time_usec, HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps)
{
	if (!link || sensor == nullptr || gps == nullptr || has_gps == nullptr) {
		return false;
	}
	if (link->worker_enabled) {
		return readWorkerSample(*link, last_time_usec, sensor, gps, has_gps);
	}
	if (!stepUntilSensor(*link, last_time_usec)) {
		return false;
	}
	link->read_motors = captureMotors(*link);
	link->read_pose = capturePose(*link);
	link->has_pose = true;
	Hako_HakoHilSensor source_sensor{};
	Hako_HakoHilGps source_gps{};
	bool source_has_gps = false;
	if (!link->mavlink_service->readSensor(source_sensor, source_gps, source_has_gps)
		|| source_sensor.time_usec <= last_time_usec) {
		return false;
	}
	hako_sensor_link_decode_sensor(&source_sensor, sensor);
	*has_gps = source_has_gps && source_gps.time_usec != link->gps_time_usec;
	if (*has_gps) {
		link->gps_time_usec = source_gps.time_usec;
		hako_sensor_link_decode_gps(&source_gps, gps);
	}
	return true;
}

extern "C" bool hako_sensor_link_write_actuator(const HakoActuatorSample *actuator)
{
	if (!link || actuator == nullptr) {
		return false;
	}
	if (link->worker_enabled) {
		std::lock_guard<std::mutex> lock(link->worker_mutex);
		if (link->worker_stop || link->worker_failed || link->worker_pending
			|| link->worker_sample_ready) {
			return false;
		}
		link->actuator = *actuator;
		link->has_actuator = true;
		link->worker_after_usec = actuator->time_usec;
		link->worker_pending = true;
		link->worker_condition.notify_one();
		return true;
	}
	link->actuator = *actuator;
	link->has_actuator = true;
	return link->mavlink_service->deliverActuator(link->actuator);
}

extern "C" uint64_t hako_sensor_link_async_sample_period_usec(void)
{
	return link && link->worker_enabled ? link->sensor_period_usec : 0U;
}

extern "C" void hako_sensor_link_notify_time(uint64_t)
{
}

extern "C" bool hako_sensor_link_read_pose(HakoVehiclePoseSample *pose)
{
	if (!link || pose == nullptr || !link->has_pose) {
		return false;
	}
	*pose = link->read_pose;
	return true;
}

extern "C" bool hako_sensor_link_read_motor_status(uint32_t index, HakoMotorStatusSample *status)
{
	if (!link || status == nullptr || index >= link->read_motors.size()) {
		return false;
	}
	*status = link->read_motors[index];	/* read in the device's thread */
	return true;
}

extern "C" void hako_sensor_link_cleanup(void)
{
	if (!link) {
		return;
	}
	stopAircraftWorker(*link);
	link->aircraft_service->stopService();
	link->mavlink_service->stopService();
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	if (link->truth_log != nullptr) {
		std::fclose(link->truth_log);
		link->truth_log = nullptr;
	}
#endif
	link.reset();
}
