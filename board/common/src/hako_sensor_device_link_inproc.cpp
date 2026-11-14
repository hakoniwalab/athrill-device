#include "hako_sensor_device_link.h"
#include "hako_sensor_device_link_mavlink.h"

#include "service.hpp"
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
#include "hakoniwa/sensor_adapter/host/sensor_host.hpp"
#include "hakoniwa/sensor_adapter/host/sensor_log.hpp"
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <array>
#include <cstdio>
#include <cstdlib>
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
	/* one sensor instance of the configuration (HAKO_SENSOR_DEVICE_SENSORS) */
	struct SensorInstance {
		std::string name;
		std::string type;		/* imu, mag, baro, gps */
		std::string implementation;	/* the adapters' implementation name */
		std::string config_path;	/* the adapter's configuration, absolute */
		hakoniwa::sensor_adapter::host::MountPose mount;	/* FLU, from the FRD mount */
		std::unique_ptr<hakoniwa::sensor_adapter::IImuSensorAdapter> imu;
		std::unique_ptr<hakoniwa::sensor_adapter::IMagSensorAdapter> mag;
		std::unique_ptr<hakoniwa::sensor_adapter::IBaroSensorAdapter> baro;
		std::unique_ptr<hakoniwa::sensor_adapter::IGpsSensorAdapter> gps;
		/* the adapter's output after the last physics step */
		hakoniwa::sensor_adapter::ImuSample imu_out{};
		hakoniwa::sensor_adapter::MagSample mag_out{};
		hakoniwa::sensor_adapter::BaroSample baro_out{};
		hakoniwa::sensor_adapter::GpsSample gps_out{};
	};
	/* one instance's value as the device takes it (hako_sensor_link_read_instance) */
	struct InstanceValue {
		HakoSensorSample sensor{};
		HakoGpsSample gps{};
		bool has_gps{false};
	};
	std::vector<SensorInstance> sensors;
	hakoniwa::sensor_adapter::host::World world{};
	uint64_t host_step_count{0};	/* physics steps the adapters have taken */
	/* the instances' values with the sample prepared last (stepUntilSensor) and with
	 * the sample the device read last (hako_sensor_link_read) */
	std::vector<InstanceValue> prepared_values;
	std::vector<InstanceValue> read_values;
	/* chips' sample periods, from the CPU's thread, checked before the next physics step */
	struct PeriodReport {
		uint32_t index;
		uint32_t kind;
		uint64_t period_ns;
	};
	std::mutex period_mutex;
	std::vector<PeriodReport> period_reports;
	/* the vehicle's true motion after the last physics step */
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

/* Hakoniwa Drone's world NED and body FRD to the sensor adapters' world NWU and body FLU. */
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

/* [Chip rates] A chip's output data rate as its firmware programmed it, against the
 * instance's adapter (docs/sensor-host.md). The physics step is fixed for the run: a
 * chip faster than it is an error. Otherwise the adapter follows the chip where it can
 * (hakoniwa-sensor-adapter update-period); where it can not, that is an error. */
void checkChipPeriods(InProcessLink& state)
{
	std::vector<InProcessLink::PeriodReport> reports;
	{
		std::lock_guard<std::mutex> lock(state.period_mutex);
		reports.swap(state.period_reports);
	}
	const uint64_t step_ns = state.physics_step_usec * 1000U;
	for (const auto& report : reports) {
		auto& sensor = state.sensors[report.index];
		const bool gyro = report.kind == HAKO_SENSOR_SAMPLE_GYRO;
		const char *part = gyro ? "gyro" : "accelerometer";
		const auto us = [](uint64_t ns) { return static_cast<double>(ns) / 1000.0; };
		if (!sensor.imu) {
			std::cerr << "WARNING: hako_sensor_device: sensor " << sensor.name << " (" << sensor.type
				<< ") got an IMU chip's sample period; not checked" << std::endl;
			continue;
		}
		if (report.period_ns == 0U) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << " " << part
				<< ": the chip's output data rate register holds a reserved setting" << std::endl;
			continue;
		}
		if (report.period_ns < step_ns) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << " " << part << ": the chip samples every "
				<< us(report.period_ns) << " us, finer than the physics step of " << us(step_ns)
				<< " us, which can not change during the run; the chip repeats values. Run with a physics step of at most "
				<< us(report.period_ns) << " us (HAKO_AIRCRAFT_PHYSICS_STEP_USEC)" << std::endl;
			continue;
		}
		const auto imu_part = gyro ? hakoniwa::sensor_adapter::ImuPart::gyro : hakoniwa::sensor_adapter::ImuPart::accel;
		uint64_t adapter_ns = 0;
		if (!sensor.imu->part_update_period_ns(imu_part, adapter_ns)) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << " " << part << ": the chip samples every "
				<< us(report.period_ns) << " us; the adapter does not say its period, so it can not be checked" << std::endl;
			continue;
		}
		if (adapter_ns == report.period_ns) {
			std::cout << "INFO: hako_sensor_device: sensor " << sensor.name << " " << part << ": chip and adapter every "
				<< us(report.period_ns) << " us" << std::endl;
			continue;
		}
		std::string error;
		if (sensor.imu->set_part_update_period_ns(imu_part, report.period_ns, error)) {
			std::cout << "INFO: hako_sensor_device: sensor " << sensor.name << " " << part << ": the adapter follows the chip, every "
				<< us(adapter_ns) << " us -> " << us(report.period_ns) << " us" << std::endl;
		} else {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << " " << part << ": the chip samples every "
				<< us(report.period_ns) << " us, the adapter every " << us(adapter_ns) << " us, and it can not follow: "
				<< error << std::endl;
		}
	}
}

/* Every instance's adapter, one physics step: the truth at its mount from the motion
 * just captured, then its own errors and timing (docs/sensor-host.md, The pipeline). */
bool stepSensorInstances(InProcessLink& state)
{
	if (state.sensors.empty()) {
		return true;
	}
	checkChipPeriods(state);
	const uint64_t time_ns = state.host_step_count * state.physics_step_usec * 1000U;
	std::string error;
	for (auto& sensor : state.sensors) {
		bool ok = true;
		if (sensor.imu) {
			ok = sensor.imu->step(host::imu_truth(time_ns, state.motion, sensor.mount, state.world,
				state.vibration), sensor.imu_out, error);
		}
		else if (sensor.mag) {
			ok = sensor.mag->step(host::mag_truth(time_ns, state.motion, sensor.mount, state.world),
				sensor.mag_out, error);
		}
		else if (sensor.baro) {
			ok = sensor.baro->step(host::baro_truth(time_ns, state.motion, sensor.mount, state.world),
				sensor.baro_out, error);
		}
		else if (sensor.gps) {
			ok = sensor.gps->step(host::gps_truth(time_ns, state.motion, sensor.mount, state.world),
				sensor.gps_out, error);
		}
		if (!ok) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << ": " << error << std::endl;
			return false;
		}
	}
	state.host_step_count++;
	return true;
}

/* A FLU vector in the FRD axes of the same sensor. */
void toFrd(const hakoniwa::sensor_adapter::Vector3& flu, double scale, float out[3])
{
	out[0] = static_cast<float>(flu.x * scale);
	out[1] = static_cast<float>(-flu.y * scale);
	out[2] = static_cast<float>(-flu.z * scale);
}

/* The instances' outputs now, in the units of the HIL sample of time hil_time_usec. */
void prepareInstanceValues(InProcessLink& state, uint64_t hil_time_usec)
{
	if (state.sensors.empty()) {
		return;
	}
	/* HIL time = host time + base: the adapters' times start at the first physics step */
	const uint64_t host_time_usec = state.host_step_count * state.physics_step_usec;
	const int64_t base_usec = static_cast<int64_t>(hil_time_usec) - static_cast<int64_t>(host_time_usec);
	state.prepared_values.assign(state.sensors.size(), InProcessLink::InstanceValue{});
	for (std::size_t i = 0U; i < state.sensors.size(); ++i) {
		const auto& sensor = state.sensors[i];
		auto& value = state.prepared_values[i];
		value.sensor.time_usec = hil_time_usec;
		/* no thermal model: a die at the sea-level temperature */
		value.sensor.temperature = static_cast<float>(state.world.sea_level_temperature_c);
		if (sensor.imu) {
			toFrd(sensor.imu_out.accel.value, 1.0, value.sensor.acc);
			toFrd(sensor.imu_out.gyro.value, 1.0, value.sensor.gyro);
		}
		else if (sensor.mag) {
			toFrd(sensor.mag_out.value, 1.0e-5, value.sensor.mag);	/* nT to gauss */
		}
		else if (sensor.baro) {
			value.sensor.abs_pressure = static_cast<float>(sensor.baro_out.pressure_pa / 100.0);
			value.sensor.pressure_alt = static_cast<float>(sensor.baro_out.pressure_altitude_m);
			if (sensor.baro->capabilities().temperature) {
				value.sensor.temperature = static_cast<float>(sensor.baro_out.temperature_c);
			}
		}
		else if (sensor.gps && sensor.gps_out.fix_type != 0U) {
			const auto& out = sensor.gps_out;
			auto& gps = value.gps;
			gps.time_usec = static_cast<uint64_t>(static_cast<int64_t>(out.sample_time_ns / 1000U) + base_usec);
			gps.lat = static_cast<int32_t>(std::llround(out.position.latitude_deg * 1e7));
			gps.lon = static_cast<int32_t>(std::llround(out.position.longitude_deg * 1e7));
			/* mean sea level, as HIL_GPS */
			gps.alt = static_cast<int32_t>(std::llround((out.position.altitude_m - state.world.geoid_height_m) * 1000.0));
			const auto cm = [](double m) {
				return static_cast<uint16_t>(std::min(65535.0, std::max(0.0, std::round(m * 100.0))));
			};
			const auto cms = [](double mps) {
				return static_cast<int16_t>(std::min(32767.0, std::max(-32767.0, std::round(mps * 100.0))));
			};
			gps.eph = cm(out.horizontal_accuracy_m);
			gps.epv = cm(out.vertical_accuracy_m);
			gps.vel = cm(out.ground_speed_mps);
			gps.vn = cms(out.velocity_nwu.x);
			gps.ve = cms(-out.velocity_nwu.y);
			gps.vd = cms(-out.velocity_nwu.z);
			gps.cog = out.course_valid ? static_cast<uint16_t>(std::llround(out.course_deg * 100.0)) % 36000U : 65535U;
			/* an adapter without a receiver status gives no satellite count: a 3D fix as 10 */
			gps.satellites_visible = (out.satellites_visible != 0U) ? static_cast<uint8_t>(out.satellites_visible)
				: (out.fix_type >= 3U ? 10U : 0U);
			gps.fix_type = static_cast<uint8_t>(out.fix_type);
			value.has_gps = true;
		}
	}
}

/* The truth check: Hakoniwa Drone's accelerometer and gyro (its sensor models, FRD) beside
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

#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
/* The sensor instances (docs/sensor-host.md, Configuration). The mount is written as
 * Hakoniwa Drone's (positionM in body FRD, orientationDeg); absent, at the origin. */
bool loadSensorInstances(InProcessLink& state, const std::string& path)
{
	std::ifstream file(path);
	if (!file) {
		std::cerr << "ERROR: hako_sensor_device: can not read " << path << std::endl;
		return false;
	}
	const auto root = nlohmann::json::parse(file);
	if (!root.contains("sensors") || !root["sensors"].is_array()) {
		std::cerr << "ERROR: hako_sensor_device: " << path << ": sensors must be an array" << std::endl;
		return false;
	}
	const std::filesystem::path base = std::filesystem::absolute(path).parent_path();
	for (const auto& entry : root["sensors"]) {
		InProcessLink::SensorInstance sensor;
		sensor.name = entry.at("name").get<std::string>();
		sensor.type = entry.at("type").get<std::string>();
		sensor.implementation = entry.at("implementation").get<std::string>();
		const std::filesystem::path config = entry.at("config").get<std::string>();
		sensor.config_path = (config.is_absolute() ? config : base / config).lexically_normal().string();
		if (sensor.type != "imu" && sensor.type != "mag" && sensor.type != "baro" && sensor.type != "gps") {
			std::cerr << "ERROR: hako_sensor_device: " << path << ": " << sensor.name
				<< ": type must be imu, mag, baro or gps" << std::endl;
			return false;
		}
		for (const auto& other : state.sensors) {
			if (other.name == sensor.name) {
				std::cerr << "ERROR: hako_sensor_device: " << path << ": " << sensor.name
					<< " is there twice" << std::endl;
				return false;
			}
		}
		hakoniwa::sensor_adapter::Vector3 position{};
		double roll = 0.0, pitch = 0.0, yaw = 0.0;
		if (entry.contains("mount")) {
			const auto& mount = entry["mount"];
			if (mount.contains("positionM")) {
				const auto p = mount["positionM"].get<std::vector<double>>();
				if (p.size() != 3U) {
					std::cerr << "ERROR: hako_sensor_device: " << path << ": " << sensor.name
						<< ": mount.positionM must have 3 values" << std::endl;
					return false;
				}
				position = {p[0], p[1], p[2]};
			}
			if (mount.contains("orientationDeg")) {
				const auto& o = mount["orientationDeg"];
				roll = o.value("roll", 0.0);
				pitch = o.value("pitch", 0.0);
				yaw = o.value("yaw", 0.0);
			}
		}
		sensor.mount = hakoniwa::sensor_adapter::host::mount_from_frd_rpy_deg(position, roll, pitch, yaw);
		std::cout << "INFO: hako_sensor_device: sensor " << sensor.name << " (" << sensor.type << ", "
			<< sensor.implementation << ", " << sensor.config_path << ") mounted at FRD ["
			<< position.x << ", " << position.y << ", " << position.z << "] m, roll " << roll
			<< " pitch " << pitch << " yaw " << yaw << " deg" << std::endl;
		state.sensors.push_back(std::move(sensor));
	}
	if (state.sensors.size() > 16U) {	/* HAKO_SENSOR_INSTANCE_MAX of the device model */
		std::cerr << "ERROR: hako_sensor_device: " << path << ": at most 16 sensors" << std::endl;
		return false;
	}
	return true;
}
#endif

#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
#ifdef HAKO_SENSOR_DEVICE_WITH_AIRSIM
HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(airsim);
#endif
/* the implementations linked in by name (HAKO_SENSOR_DEVICE_IMPLEMENTATIONS) */
#ifdef HAKO_SENSOR_DEVICE_IMPLEMENTATION_LIST
#define HAKO_SENSOR_DEVICE_IMPLEMENTATION(name) HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(name);
HAKO_SENSOR_DEVICE_IMPLEMENTATION_LIST
#undef HAKO_SENSOR_DEVICE_IMPLEMENTATION
#endif

/* The adapters' implementations built into this plugin, by name. */
const hakoniwa::sensor_adapter::SensorAdapterImplementation *findImplementation(const std::string& name)
{
	std::vector<const hakoniwa::sensor_adapter::SensorAdapterImplementation *> built;
#ifdef HAKO_SENSOR_DEVICE_WITH_AIRSIM
	built.push_back(HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(airsim)());
#endif
#ifdef HAKO_SENSOR_DEVICE_IMPLEMENTATION_LIST
#define HAKO_SENSOR_DEVICE_IMPLEMENTATION(name) built.push_back(HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(name)());
	HAKO_SENSOR_DEVICE_IMPLEMENTATION_LIST
#undef HAKO_SENSOR_DEVICE_IMPLEMENTATION
#endif
	for (const auto *implementation : built) {
		if (implementation != nullptr
			&& implementation->version == hakoniwa::sensor_adapter::kSensorAdapterImplementationVersion
			&& name == implementation->name) {
			return implementation;
		}
	}
	return nullptr;
}

/* A seed from the instance's name (32-bit FNV-1a, which every adapter takes), so each chip
 * has its own stream and a run repeats with the same configuration. */
uint64_t seedFromName(const std::string& name)
{
	uint32_t hash = 2166136261U;
	for (const unsigned char c : name) {
		hash = (hash ^ c) * 16777619U;
	}
	return hash;
}

/* The world of the airframe (simulation.location) and every instance's adapter, with its
 * sensor log when its configuration asks for one (hakoniwa-sensor-adapter
 * docs/spec/sensor-log.md). An adapter refuses a period it can not measure with this
 * physics step. */
bool createSensorAdapters(InProcessLink& state, const DroneConfig& config)
{
	if (state.sensors.empty()) {
		return true;
	}
	state.world.origin = {config.getSimLatitude(), config.getSimLongitude(), config.getSimAltitude()};
	const auto field = config.getSimMagneticField();
	state.world.geomagnetic_field = {field.intensity_nT, field.declination_deg, field.inclination_deg};
	for (auto& sensor : state.sensors) {
		const auto *implementation = findImplementation(sensor.implementation);
		if (implementation == nullptr) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << ": implementation "
				<< sensor.implementation << " is not built into this plugin" << std::endl;
			return false;
		}
		const auto setup = [&](auto& value) {
			value.config_path = sensor.config_path;
			value.vehicle_name = "aircraft0";
			value.vehicle_index = 0U;
			value.sensor_name = sensor.name;
			value.physics_step_ns = state.physics_step_usec * 1000U;
			value.has_random_seed = true;
			value.random_seed = seedFromName(sensor.name);
		};
		std::string error;
		bool provided = true;
		if (sensor.type == "imu") {
			hakoniwa::sensor_adapter::ImuAdapterSetup value;
			setup(value);
			provided = implementation->create_imu != nullptr;
			if (provided) sensor.imu = host::create_logged_imu_sensor_adapter(value, error, implementation->create_imu);
		}
		else if (sensor.type == "mag") {
			hakoniwa::sensor_adapter::MagAdapterSetup value;
			setup(value);
			provided = implementation->create_mag != nullptr;
			if (provided) sensor.mag = host::create_logged_mag_sensor_adapter(value, error, implementation->create_mag);
		}
		else if (sensor.type == "baro") {
			hakoniwa::sensor_adapter::BaroAdapterSetup value;
			setup(value);
			provided = implementation->create_baro != nullptr;
			if (provided) sensor.baro = host::create_logged_baro_sensor_adapter(value, error, implementation->create_baro);
		}
		else {
			hakoniwa::sensor_adapter::GpsAdapterSetup value;
			setup(value);
			value.origin = state.world.origin;
			provided = implementation->create_gps != nullptr;
			if (provided) sensor.gps = host::create_logged_gps_sensor_adapter(value, error, implementation->create_gps);
		}
		if (!provided) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << ": implementation "
				<< sensor.implementation << " has no " << sensor.type << " adapter" << std::endl;
			return false;
		}
		if (!sensor.imu && !sensor.mag && !sensor.baro && !sensor.gps) {
			std::cerr << "ERROR: hako_sensor_device: sensor " << sensor.name << ": " << error << std::endl;
			return false;
		}
	}
	std::cout << "INFO: hako_sensor_device: " << state.sensors.size()
		<< " sensor instances measure every physics step of " << state.physics_step_usec << " us" << std::endl;
	return true;
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
			prepareInstanceValues(state, sensor.time_usec);
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
		if (!stepSensorInstances(state)) {
			return false;
		}
#endif
	}
	if (!state.mavlink_service->readSensor(sensor, gps, has_gps)
		|| sensor.time_usec <= last_time_usec) {
		return false;
	}
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	prepareInstanceValues(state, sensor.time_usec);
#endif
	return true;
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
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	state.read_values = state.prepared_values;
#endif
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
		if (config->sensors_config_path != nullptr
			&& !loadSensorInstances(*state, config->sensors_config_path)) {
			return false;
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
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
		if (!createSensorAdapters(*state, aircraft_config)) {
			return false;
		}
#endif
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
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
		if (!state->sensors.empty()) {
			/* The chips see each instance's measurement in the physics step it is made:
			 * the sensor device takes a sample, and gives the motors, every physics step. */
			if (state->sensor_period_usec != state->physics_step_usec) {
				std::cout << "INFO: hako_sensor_device: sensor instances: the sensor device samples every physics step ("
					<< state->physics_step_usec << " us) in place of the aircraft's sensor period ("
					<< state->sensor_period_usec << " us)" << std::endl;
			}
			state->sensor_period_usec = state->physics_step_usec;
			if (!state->aircraft_service->setSitlSensorPeriodUsec(0U, state->physics_step_usec)) {
				return false;
			}
		}
#endif
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
		/* At exit, the aircraft must go while Hakoniwa Drone's library is still whole. exit()
		 * runs its handlers in the reverse order of registration: the host's own teardown was
		 * registered at start-up, before this library and Hakoniwa Drone's were loaded, so it
		 * would run after their static objects are destroyed and reach into them. Registered
		 * now, this cleanup runs before those; the host's teardown then finds nothing left. */
		static bool exit_cleanup_registered = false;
		if (!exit_cleanup_registered) {
			exit_cleanup_registered = std::atexit([] { hako_sensor_link_cleanup(); }) == 0;
		}
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
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	link->read_values = link->prepared_values;
#endif
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

extern "C" bool hako_sensor_link_sensor_instance(const char *name, const char **type, uint32_t *index)
{
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	if (link && name != nullptr && type != nullptr && index != nullptr) {
		for (std::size_t i = 0U; i < link->sensors.size(); ++i) {
			if (link->sensors[i].name == name) {
				*type = link->sensors[i].type.c_str();
				*index = static_cast<uint32_t>(i);
				return true;
			}
		}
	}
#else
	(void)name;
	(void)type;
	(void)index;
#endif
	return false;
}

extern "C" bool hako_sensor_link_instance_sample_period(uint32_t index, uint32_t kind, uint64_t period_ns)
{
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	if (!link || index >= link->sensors.size()) {
		return false;
	}
	std::lock_guard<std::mutex> lock(link->period_mutex);
	link->period_reports.push_back({index, kind, period_ns});
	return true;
#else
	(void)index;
	(void)kind;
	(void)period_ns;
	return false;
#endif
}

extern "C" bool hako_sensor_link_read_instance(uint32_t index, HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps)
{
#ifdef HAKO_SENSOR_DEVICE_SENSOR_HOST
	if (!link || sensor == nullptr || gps == nullptr || has_gps == nullptr
		|| index >= link->read_values.size()) {
		return false;
	}
	const auto& value = link->read_values[index];
	*sensor = value.sensor;
	*gps = value.gps;
	*has_gps = value.has_gps;
	return true;
#else
	(void)index;
	(void)sensor;
	(void)gps;
	(void)has_gps;
	return false;
#endif
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
