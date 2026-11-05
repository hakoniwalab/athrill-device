#include "hako_sensor_device_link.h"
#include "hako_sensor_device_link_mavlink.h"

#include "service.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

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
	}
	return state.mavlink_service->readSensor(sensor, gps, has_gps)
		&& sensor.time_usec > last_time_usec;
}

}  // namespace

extern "C" bool hako_sensor_link_initialize(const HakoSensorLinkConfig *config)
{
	if (config == nullptr || config->aircraft_config_path == nullptr || link) {
		return false;
	}
	try {
		auto state = std::make_unique<InProcessLink>();
		if (!loadAircraftConfig(*state, config->aircraft_config_path)) {
			return false;
		}
		DroneConfig& aircraft_config = state->config_manager.getConfig(0U);
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
	if (!link || sensor == nullptr || gps == nullptr || has_gps == nullptr
		|| !stepUntilSensor(*link, last_time_usec)) {
		return false;
	}
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
	link->actuator = *actuator;
	link->has_actuator = true;
	return link->mavlink_service->deliverActuator(link->actuator);
}

extern "C" void hako_sensor_link_notify_time(uint64_t)
{
}

extern "C" void hako_sensor_link_cleanup(void)
{
	if (!link) {
		return;
	}
	link->aircraft_service->stopService();
	link->mavlink_service->stopService();
	link.reset();
}
