/* Sensor-host start-up consistency checks. SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"

#include <stdio.h>
#include <string.h>

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps)
};

int main(int argc, char **argv)
{
	AthrillDevicePluginPath plugins[2];
	AthrillDeviceHostValue sensor_config = {
		"HAKO_SENSOR_DEVICE_SENSORS", "sensors.json"
	};
	AthrillDeviceBuildOptions options;
	AthrillDeviceRuntime *runtime = NULL;
	if (argc != 4) {
		fprintf(stderr, "usage: %s SCENARIO DTB PLUGIN\n", argv[0]);
		return 2;
	}
	memset(&options, 0, sizeof(options));
	plugins[0].compatible = "hakoniwa,sensor-device";
	plugins[0].library_path = argv[3];
	plugins[1].compatible = "hakoniwa,sensor-instance";
	plugins[1].library_path = argv[3];
	options.dtb_path = argv[2];
	options.plugins = plugins;
	options.plugin_count = strcmp(argv[1], "missing-plugin") == 0 ? 1U : 2U;
	options.maximum_dtb_size = 1024U * 1024U;
	if (strcmp(argv[1], "no-wiring") == 0) {
		options.host_values = &sensor_config;
		options.host_value_count = 1U;
	}
	if (athrill_device_build(&options, &target_ops, NULL, &runtime) == 0) {
		fprintf(stderr, "sensor-host scenario unexpectedly built: %s\n", argv[1]);
		athrill_device_destroy(runtime);
		return 1;
	}
	return 0;
}
