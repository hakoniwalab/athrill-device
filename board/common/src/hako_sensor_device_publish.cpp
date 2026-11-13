/*
 * The sensor device's Hakoniwa output when Athrill is the Hakoniwa asset
 * (athrill3 --hakoniwa <asset>): one vehicle's visual state as a
 * hako_msgs/DroneVisualStateArray PDU, for a viewer behind a PDU bridge
 * (hakoniwa-threejs-drone "fleets"). Only the in-process motor plugin
 * builds this; Athrill created the channel from its --pdudef.
 *
 * SPDX-License-Identifier: MIT
 */
#include "hako_sensor_device_publish.h"

#include "hakoniwa/hakoniwa_asset_polling.h"
#include "hako_msgs/pdu_cpptype_DroneVisualState.hpp"
#include "hako_msgs/pdu_cpptype_DroneVisualStateArray.hpp"
#include "hako_msgs/pdu_cpptype_conv_DroneVisualStateArray.hpp"

#include <vector>

extern "C" bool hako_sensor_publish_visual_state(const HakoVisualStateOutput *output,
	const HakoVehiclePoseSample *pose, const float *duty, uint32_t duty_count, uint32_t sequence)
{
	if (output == nullptr || output->asset_name == nullptr || output->robot_name == nullptr ||
	    pose == nullptr || (duty == nullptr && duty_count != 0U)) {
		return false;
	}
	HakoCpp_DroneVisualState state{};
	state.x = static_cast<float>(pose->x);
	state.y = static_cast<float>(pose->y);
	state.z = static_cast<float>(pose->z);
	state.roll = static_cast<float>(pose->roll);
	state.pitch = static_cast<float>(pose->pitch);
	state.yaw = static_cast<float>(pose->yaw);
	for (uint32_t i = 0U; i < duty_count; i++) {
		state.pwm_duty.push_back(duty[i]);
	}
	HakoCpp_DroneVisualStateArray array{};
	array.sequence_id = sequence;
	array.chunk_index = 0U;
	array.chunk_count = 1U;
	array.start_index = 0U;
	array.valid_count = 1U;
	array.drones.push_back(state);

	std::vector<char> buffer(output->pdu_size, 0);
	hako::pdu::msgs::hako_msgs::DroneVisualStateArray convertor;
	const int written = convertor.cpp2pdu(array, buffer.data(), static_cast<int>(buffer.size()));
	if (written <= 0) {
		return false;
	}
	return hakoniwa_asset_write_pdu(output->asset_name, output->robot_name, output->channel_id,
		buffer.data(), static_cast<size_t>(written)) == 0;
}
