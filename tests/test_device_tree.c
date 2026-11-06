/* SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	uint64 addresses[16];
	uint64 sizes[16];
	const AthrillDeviceMmioOps *ops;
	void *instance;
	uint32 registered;
	uint32 irq_resolved;
	uint32 gpio_resolved;
	uint32 dma_resolved;
	uint32 spi_bound;
	const AthrillDeviceSpiPeripheralOps *spi_ops;
	void *spi_instance;
	uint32 i2c_bound;
	const AthrillDeviceI2cPeripheralOps *i2c_ops;
	void *i2c_instance;
	uint32 uart_bound;
	const AthrillDeviceUartPeripheralOps *uart_ops;
	void *uart_instance;
	AthrillDeviceNodeId uart_peripheral;
	uint8 uart_output[256];
	uint32 uart_output_count;
} TestTarget;

static sint32 mmio_register(void *context, uint64 address, uint64 size,
	const AthrillDeviceMmioOps *ops, void *instance,
	AthrillDeviceMmioToken *token)
{
	TestTarget *target = context;
	assert(target->registered < 16U);
	target->addresses[target->registered] = address;
	target->sizes[target->registered] = size;
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

static sint32 irq_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceIrqLine *line)
{
	TestTarget *target = context;
	assert(spec->cell_count == 1U && spec->cells[0] == 17U);
	target->irq_resolved++;
	line->value = spec->cells[0];
	return 0;
}

static sint32 gpio_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceGpioLine *line)
{
	TestTarget *target = context;
	assert(spec->cell_count == 2U);
	target->gpio_resolved++;
	line->value = spec->cells[0];
	return 0;
}

static sint32 dma_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceDmaChannel *channel)
{
	TestTarget *target = context;
	assert(spec->cell_count == 1U && spec->cells[0] == 9U);
	target->dma_resolved++;
	channel->value = spec->cells[0];
	return 0;
}

static sint32 spi_bind(void *context, AthrillDeviceNodeId controller,
	uint32 chip_select, const AthrillDeviceSpecifier *gpio,
	const AthrillDeviceSpiPeripheralOps *ops, void *instance,
	AthrillDeviceSpiEndpoint *endpoint)
{
	TestTarget *target = context;
	(void)controller;
	(void)gpio;
	(void)ops;
	(void)instance;
	assert(chip_select == 0U);
	target->spi_bound++;
	endpoint->value = 101U;
	return 0;
}

static sint32 spi_bind_node(void *context,
	const AthrillDeviceNode *controller, uint32 chip_select,
	const AthrillDeviceSpecifier *gpio,
	const AthrillDeviceSpiPeripheralOps *ops, void *instance,
	AthrillDeviceSpiEndpoint *endpoint)
{
	TestTarget *target = context;
	(void)gpio;
	assert(controller != NULL && strstr(controller->path, "spi@") != NULL);
	assert(chip_select == 0U);
	target->spi_bound++;
	target->spi_ops = ops;
	target->spi_instance = instance;
	endpoint->value = 104U;
	return 0;
}

static sint32 i2c_bind(void *context, AthrillDeviceNodeId controller,
	uint16 address, const AthrillDeviceI2cPeripheralOps *ops, void *instance,
	AthrillDeviceI2cEndpoint *endpoint)
{
	TestTarget *target = context;
	(void)controller;
	(void)ops;
	(void)instance;
	assert(address == 0x0EU);
	target->i2c_bound++;
	endpoint->value = 102U;
	return 0;
}

static sint32 i2c_bind_node(void *context,
	const AthrillDeviceNode *controller, uint16 address,
	const AthrillDeviceI2cPeripheralOps *ops, void *instance,
	AthrillDeviceI2cEndpoint *endpoint)
{
	TestTarget *target = context;
	assert(controller != NULL && strstr(controller->path, "i2c@") != NULL);
	assert(address == 0x0eU);
	target->i2c_bound++;
	target->i2c_ops = ops;
	target->i2c_instance = instance;
	endpoint->value = 105U;
	return 0;
}

static sint32 uart_bind(void *context, AthrillDeviceNodeId controller,
	const AthrillDeviceUartPeripheralOps *ops, void *instance,
	AthrillDeviceUartEndpoint *endpoint)
{
	TestTarget *target = context;
	(void)controller;
	(void)ops;
	(void)instance;
	target->uart_bound++;
	endpoint->value = 103U;
	return 0;
}

static sint32 uart_bind_nodes(void *context,
	const AthrillDeviceNode *controller, const AthrillDeviceNode *peripheral,
	const AthrillDeviceUartPeripheralOps *ops, void *instance,
	AthrillDeviceUartEndpoint *endpoint)
{
	TestTarget *target = context;
	assert(controller != NULL && strstr(controller->path, "uart@") != NULL);
	assert(peripheral != NULL);
	target->uart_bound++;
	target->uart_ops = ops;
	target->uart_instance = instance;
	target->uart_peripheral = peripheral->id;
	endpoint->value = 106U;
	return 0;
}

static sint32 uart_send_from_node(void *context,
	AthrillDeviceNodeId peripheral, uint8 data)
{
	TestTarget *target = context;
	assert(peripheral == target->uart_peripheral);
	assert(target->uart_output_count < sizeof(target->uart_output));
	target->uart_output[target->uart_output_count++] = data;
	return 0;
}

static uint64 time_frequency(void *context)
{
	(void)context;
	return 216000000U;
}

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.mmio_register = mmio_register,
	.mmio_unregister = mmio_unregister,
	.irq_resolve = irq_resolve,
	.spi_bind = spi_bind,
	.i2c_bind = i2c_bind,
	.uart_bind = uart_bind,
	.gpio_resolve = gpio_resolve,
	.dma_resolve = dma_resolve,
	.time_frequency_hz = time_frequency,
	.spi_bind_node = spi_bind_node,
	.i2c_bind_node = i2c_bind_node,
	.uart_bind_nodes = uart_bind_nodes,
	.uart_send_from_node = uart_send_from_node,
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

static void expect_failure(const char *name, TestTarget *target)
{
	char dtb[1024];
	AthrillDeviceRuntime *runtime = NULL;
	path(dtb, sizeof(dtb), name);
	assert(build(dtb, NULL, 0U, &target_ops, target, &runtime) != 0);
	assert(runtime == NULL && target->registered == 0U);
}

int main(int argc, char **argv)
{
	char dtb[1024];
	TestTarget target;
	AthrillDeviceRuntime *runtime = NULL;
	AthrillDeviceTickResult tick;
	uint64 value;
	AthrillDevicePluginPath plugins[2];
	AthrillDeviceTargetOps bad_ops = target_ops;
	static const char *const negative_dtbs[] = {
		"overlap.dtb", "missing-provider.dtb", "disabled-provider.dtb",
		"duplicate-phandle.dtb", "cycle.dtb", "duplicate-spi.dtb",
		"duplicate-i2c.dtb", "bad-reg.dtb", "overflow-reg.dtb",
		"overflow-ranges.dtb", "truncated.dtb", "bad-string-offset.dtb",
	};
	uint32 i;
	assert(argc == 3);

	memset(&target, 0, sizeof(target));
	path(dtb, sizeof(dtb), "valid.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 1U && target.addresses[0] == 0x1000U &&
		target.sizes[0] == 0x100U);
	assert(target.ops->write(target.instance, target.addresses[0], 4U,
		0x12345678U) == 0);
	assert(target.ops->read(target.instance, target.addresses[0], 4U,
		&value) == 0 && value == 0x12345678U);
	assert(athrill_device_tick(runtime, 1U, &tick) == 0 && tick.can_skip);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "gps.dtb");
	target.uart_bound = 0U;
	target.uart_output_count = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.uart_bound == 1U && target.uart_ops != NULL);
	{
		static const uint8 poll[] = {
			0xb5U, 0x62U, 0x0aU, 0x04U, 0U, 0U, 0x0eU, 0x34U
		};
		for (i = 0U; i < sizeof(poll); i++) {
			assert(target.uart_ops->receive_byte(target.uart_instance,
				poll[i]) == 0);
		}
		assert(target.uart_output_count > 6U);
		assert(target.uart_output[0] == 0xb5U &&
			target.uart_output[1] == 0x62U &&
			target.uart_output[2] == 0x0aU &&
			target.uart_output[3] == 0x04U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.uart_bound = 0U;
	assert(target.registered == 0U);

	path(dtb, sizeof(dtb), "order.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 2U && target.addresses[0] == 0x3000U &&
		target.addresses[1] == 0x2000U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	assert(target.registered == 0U);

	path(dtb, sizeof(dtb), "resources.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.irq_resolved == 1U && target.gpio_resolved == 1U &&
		target.dma_resolved == 1U);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "wiring.dtb");
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.i2c_bound == 1U &&
		target.uart_bound == 1U);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "fram.dtb");
	target.spi_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.spi_ops != NULL);
	target.spi_ops->set_chip_select(target.spi_instance, TRUE);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0x9FU) == 0U);
	for (i = 0U; i < 6U; i++) {
		assert(target.spi_ops->exchange_byte(target.spi_instance, 0U) == 0x7FU);
	}
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0U) == 0xC2U);
	target.spi_ops->set_chip_select(target.spi_instance, FALSE);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "imu.dtb");
	target.spi_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.spi_ops != NULL);
	target.spi_ops->set_chip_select(target.spi_instance, TRUE);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0xf5U) == 0U);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0U) == 0x98U);
	target.spi_ops->set_chip_select(target.spi_instance, FALSE);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "magnetometer.dtb");
	target.i2c_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.i2c_bound == 1U && target.i2c_ops != NULL);
	{
		uint8 pointer = 0U;
		uint8 who_am_i = 0U;
		AthrillDeviceI2cMessage messages[2] = {
			{&pointer, 1U, FALSE, FALSE},
			{&who_am_i, 1U, TRUE, TRUE},
		};
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert(who_am_i == 0x10U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "barometer.dtb");
	target.spi_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.spi_ops != NULL);
	target.spi_ops->set_chip_select(target.spi_instance, TRUE);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0xa2U) == 0U);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0U) == 0x9cU);
	assert(target.spi_ops->exchange_byte(target.spi_instance, 0U) == 0xbfU);
	target.spi_ops->set_chip_select(target.spi_instance, FALSE);
	athrill_device_destroy(runtime);
	runtime = NULL;

	for (i = 0U; i < sizeof(negative_dtbs) / sizeof(negative_dtbs[0]); i++) {
		expect_failure(negative_dtbs[i], &target);
	}

	path(dtb, sizeof(dtb), "plugin.dtb");
	plugins[0].compatible = "vendor,plugin";
	plugins[0].library_path = argv[1];
	assert(build(dtb, plugins, 1U, &target_ops, &target, &runtime) == 0);
	athrill_device_destroy(runtime);
	runtime = NULL;
	plugins[0].library_path = argv[2];
	assert(build(dtb, plugins, 1U, &target_ops, &target, &runtime) != 0);
	plugins[0].compatible = "athrill,test-mmio";
	plugins[0].library_path = argv[1];
	path(dtb, sizeof(dtb), "valid.dtb");
	assert(build(dtb, plugins, 1U, &target_ops, &target, &runtime) != 0);

	plugins[0].compatible = "vendor,plugin";
	plugins[1] = plugins[0];
	path(dtb, sizeof(dtb), "plugin.dtb");
	assert(build(dtb, plugins, 2U, &target_ops, &target, &runtime) != 0);

	bad_ops.abi_major = 2U;
	assert(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	bad_ops = target_ops;
	bad_ops.struct_size = sizeof(AthrillDeviceTargetOps) - 1U;
	assert(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	puts("device tree framework tests passed");
	return 0;
}
