/* SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	uint64 address;
	uint64 size;
	const AthrillDeviceMmioOps *ops;
	void *instance;
	uint32 registered;
} TestTarget;

static sint32 mmio_register(void *context, uint64 address, uint64 size,
                            const AthrillDeviceMmioOps *ops, void *instance, AthrillDeviceMmioToken *token)
{
	TestTarget *target = context;
	target->address = address;
	target->size = size;
	target->ops = ops;
	target->instance = instance;
	target->registered++;
	token->value = target->registered;
	return 0;
}
static void mmio_unregister(void *context, AthrillDeviceMmioToken token)
{
	TestTarget *target = context;
	(void)token;
	target->registered--;
}

static const AthrillDeviceTargetOps target_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceTargetOps), mmio_register, mmio_unregister
};

static void path(char *buffer, size_t size, const char *name)
{
	snprintf(buffer, size, "%s/%s", TEST_DTB_DIR, name);
}

static int build(const char *dtb, const AthrillDevicePluginPath *plugins,
                 uint32 plugin_count, const AthrillDeviceTargetOps *ops,
                 TestTarget *target, AthrillDeviceRuntime **runtime)
{
	AthrillDeviceBuildOptions options;
	memset(&options, 0, sizeof(options));
	options.dtb_path = dtb;
	options.plugins = plugins;
	options.plugin_count = plugin_count;
	options.maximum_dtb_size = 1024U * 1024U;
	return athrill_device_build(&options, ops, target, runtime);
}

int main(int argc, char **argv)
{
	char dtb[1024];
	TestTarget target;
	AthrillDeviceRuntime *runtime = NULL;
	AthrillDeviceTickResult tick;
	uint64 value;
	AthrillDevicePluginPath plugin;
	AthrillDeviceTargetOps bad_ops = target_ops;
	assert(argc == 3);

	memset(&target, 0, sizeof(target));
	path(dtb, sizeof(dtb), "valid.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 1U && target.address == 0x1000U && target.size == 0x100U);
	assert(target.ops->write(target.instance, target.address, 4U, 0x12345678U) == 0);
	assert(target.ops->read(target.instance, target.address, 4U, &value) == 0 && value == 0x12345678U);
	assert(athrill_device_tick(runtime, 1U, &tick) == 0 && tick.can_skip);
	athrill_device_destroy(runtime);
	runtime = NULL;
	assert(target.registered == 0U);

	path(dtb, sizeof(dtb), "overlap.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) != 0);
	path(dtb, sizeof(dtb), "missing-provider.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) != 0);
	path(dtb, sizeof(dtb), "cycle.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) != 0);

	path(dtb, sizeof(dtb), "plugin.dtb");
	plugin.compatible = "vendor,plugin";
	plugin.library_path = argv[1];
	assert(build(dtb, &plugin, 1U, &target_ops, &target, &runtime) == 0);
	athrill_device_destroy(runtime);
	runtime = NULL;
	plugin.library_path = argv[2];
	assert(build(dtb, &plugin, 1U, &target_ops, &target, &runtime) != 0);
	plugin.compatible = "athrill,test-mmio";
	plugin.library_path = argv[1];
	path(dtb, sizeof(dtb), "valid.dtb");
	assert(build(dtb, &plugin, 1U, &target_ops, &target, &runtime) != 0);

	bad_ops.abi_major = 2U;
	assert(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	puts("device tree framework tests passed");
	return 0;
}
