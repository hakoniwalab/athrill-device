/* SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"
#include "athrill_protocol_spi.h"

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
	AthrillDeviceGpioEdge gpio_edges[192];
	void *gpio_edge_instances[192];
	std_bool gpio_levels[192];
	uint32 gpio_driven;
	uint32 irq_set_count;
	uint64 last_irq;
	uint64 clock;
	float32 actuator_values[16];
	uint32 actuator_count;
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

typedef struct {
	AthrillChannel *channel;
	AthrillSpiProtocol *protocol;
} TestSpiConnection;

static void spi_child_select(void *context, uint64 time)
{
	TestTarget *target = context;
	(void)time;
	target->spi_ops->set_chip_select(target->spi_instance, TRUE);
}

static sint32 spi_child_exchange(void *context, uint8 transmitted,
	uint8 *received, uint64 time)
{
	TestTarget *target = context;
	(void)time;
	*received = target->spi_ops->exchange_byte(target->spi_instance,
		transmitted);
	return 0;
}

static void spi_child_deselect(void *context, uint64 time)
{
	TestTarget *target = context;
	(void)time;
	target->spi_ops->set_chip_select(target->spi_instance, FALSE);
}

static const AthrillSpiChildOps spi_child_ops = {
	spi_child_select, spi_child_exchange, spi_child_deselect
};

static void spi_connect(TestTarget *target, uint32 chip_select,
	TestSpiConnection *connection)
{
	AthrillChannelConfig config;
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_CHIP;
	config.selection.value = chip_select;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	assert(athrill_channel_create(&config, &connection->channel) == 0);
	assert(athrill_spi_protocol_create(connection->channel, &spi_child_ops,
		target, &connection->protocol) == 0);
	assert(athrill_spi_select(connection->protocol, chip_select, 1U) == 1);
}

static uint8 spi_transfer(TestSpiConnection *connection, uint8 transmitted,
	uint64 time)
{
	uint8 received;
	assert(athrill_spi_exchange(connection->protocol, transmitted, &received,
		time) == 0);
	return received;
}

static void spi_disconnect(TestSpiConnection *connection, uint64 time)
{
	assert(athrill_spi_deselect(connection->protocol, time) == 0);
	athrill_spi_protocol_destroy(connection->protocol);
	athrill_channel_destroy(connection->channel);
}

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
	assert(spec->cell_count == 1U);
	target->irq_resolved++;
	line->value = spec->cells[0];
	return 0;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line,
	std_bool asserted)
{
	TestTarget *target = context;
	assert(asserted);
	target->irq_set_count++;
	target->last_irq = line.value;
	return 0;
}

static std_bool irq_is_active(void *context, AthrillDeviceIrqLine line)
{
	(void)context;
	(void)line;
	return FALSE;
}

static sint32 gpio_drive(void *context, AthrillDeviceGpioLine line,
	std_bool level)
{
	TestTarget *target = context;
	uint32 pin = (uint32)line.value - 1U;
	assert(pin < 192U);
	target->gpio_levels[pin] = level;
	target->gpio_driven++;
	if (target->gpio_edges[pin] != NULL) {
		target->gpio_edges[pin](target->gpio_edge_instances[pin], level);
	}
	return 0;
}

static sint32 gpio_outputs(void *context, const AthrillDeviceGpioLine *lines,
	uint32 count, uint32 levels)
{
	TestTarget *target = context;
	uint32 index;
	for (index = 0U; index < count; index++) {
		uint32 pin = (uint32)lines[index].value - 1U;
		std_bool level = (levels & (1U << index)) != 0U;
		if (pin >= 192U) {
			return -1;
		}
		if (target->gpio_levels[pin] != level &&
		    gpio_drive(context, lines[index], level) != 0) {
			return -1;
		}
	}
	return 0;
}

static sint32 gpio_subscribe(void *context, AthrillDeviceGpioLine line,
	AthrillDeviceGpioEdge callback, void *instance)
{
	TestTarget *target = context;
	uint32 pin = (uint32)line.value - 1U;
	assert(pin < 192U && callback != NULL);
	target->gpio_edges[pin] = callback;
	target->gpio_edge_instances[pin] = instance;
	return 0;
}

static sint32 gpio_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceGpioLine *line)
{
	TestTarget *target = context;
	assert(spec->cell_count == 2U);
	target->gpio_resolved++;
	line->value = target->gpio_resolved;
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

static uint64 time_now(void *context)
{
	return ((TestTarget *)context)->clock;
}

static sint32 actuator_output(void *context, const float32 *values,
	uint32 count)
{
	TestTarget *target = context;
	assert(count <= 16U);
	memcpy(target->actuator_values, values, count * sizeof(*values));
	target->actuator_count = count;
	return 0;
}

static sint32 sensor_sample(void *context, AthrillDeviceSensorSample *sample)
{
	(void)context;
	memset(sample, 0, sizeof(*sample));
	sample->magnetic_field[0] = -0.05F;
	sample->magnetic_field[1] = 0.2F;
	sample->magnetic_field[2] = -0.4F;
	return 0;
}

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.mmio_register = mmio_register,
	.mmio_unregister = mmio_unregister,
	.irq_resolve = irq_resolve,
	.irq_set = irq_set,
	.irq_is_active = irq_is_active,
	.time_now = time_now,
	.spi_bind = spi_bind,
	.i2c_bind = i2c_bind,
	.uart_bind = uart_bind,
	.gpio_resolve = gpio_resolve,
	.gpio_drive = gpio_drive,
	.gpio_subscribe = gpio_subscribe,
	.gpio_outputs = gpio_outputs,
	.dma_resolve = dma_resolve,
	.time_frequency_hz = time_frequency,
	.spi_bind_node = spi_bind_node,
	.i2c_bind_node = i2c_bind_node,
	.uart_bind_nodes = uart_bind_nodes,
	.uart_send_from_node = uart_send_from_node,
	.actuator_output = actuator_output,
	.sensor_sample = sensor_sample,
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

	path(dtb, sizeof(dtb), "gpio.dtb");
	target.gpio_resolved = 0U;
	target.gpio_driven = 0U;
	memset(target.gpio_edges, 0, sizeof(target.gpio_edges));
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 1U && target.addresses[0] == 0x40020000U &&
		target.sizes[0] == 0x400U && target.gpio_resolved == 16U);
	assert(target.ops->write(target.instance, 0x40020000U, 4U, 1U << 8) == 0);
	assert(target.ops->write(target.instance, 0x40020014U, 4U, 1U << 4) == 0);
	assert(target.gpio_driven == 1U && target.gpio_levels[4]);
	{
		AthrillDeviceGpioLine line = {3U};
		assert(gpio_drive(&target, line, TRUE) == 0);
		assert(target.ops->read(target.instance, 0x40020010U, 4U,
			&value) == 0 && (value & (1U << 2)) != 0U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.gpio_resolved = 0U;
	memset(target.gpio_edges, 0, sizeof(target.gpio_edges));

	path(dtb, sizeof(dtb), "exti.dtb");
	target.gpio_resolved = 0U;
	target.irq_resolved = 0U;
	target.irq_set_count = 0U;
	memset(target.gpio_edges, 0, sizeof(target.gpio_edges));
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 2U && target.irq_resolved == 7U &&
		target.gpio_resolved == 176U);
	/* Route line 4 to GPIOB, enable its rising edge and drive PB4 high. */
	assert(target.ops->write(target.instance, 0x4001380cU, 4U, 1U) == 0);
	assert(target.ops->write(target.instance, 0x40013c00U, 4U, 1U << 4) == 0);
	assert(target.ops->write(target.instance, 0x40013c08U, 4U, 1U << 4) == 0);
	{
		AthrillDeviceGpioLine line = {16U + 4U + 1U};
		assert(gpio_drive(&target, line, TRUE) == 0);
	}
	assert(target.irq_set_count == 1U && target.last_irq == 10U);
	assert(target.ops->read(target.instance, 0x40013c14U, 4U,
		&value) == 0 && (value & (1U << 4)) != 0U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.gpio_resolved = 0U;
	target.irq_resolved = 0U;
	memset(target.gpio_edges, 0, sizeof(target.gpio_edges));

	path(dtb, sizeof(dtb), "timer-pwm.dtb");
	target.registered = 0U;
	target.irq_resolved = 0U;
	target.actuator_count = 0U;
	target.clock = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.registered == 1U && target.irq_resolved == 1U);
	assert(target.ops->write(target.instance, 0x40000828U, 4U, 107U) == 0);
	assert(target.ops->write(target.instance, 0x4000082cU, 4U, 19999U) == 0);
	assert(target.ops->write(target.instance, 0x40000834U, 4U, 1500U) == 0);
	assert(target.ops->write(target.instance, 0x40000818U, 4U, 6U << 4) == 0);
	assert(target.ops->write(target.instance, 0x40000820U, 4U, 1U) == 0);
	assert(target.ops->write(target.instance, 0x40000814U, 4U, 1U) == 0);
	assert(target.ops->write(target.instance, 0x40000800U, 4U, 1U) == 0);
	assert(athrill_device_tick(runtime, 1U, &tick) == 0);
	assert(target.actuator_count == 1U &&
		target.actuator_values[0] > 0.499F &&
		target.actuator_values[0] < 0.501F);
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.irq_resolved = 0U;

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
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		assert(spi_transfer(&connection, 0x9fU, 2U) == 0U);
		for (i = 0U; i < 6U; i++) {
			assert(spi_transfer(&connection, 0U, 3U + i) == 0x7fU);
		}
		assert(spi_transfer(&connection, 0U, 9U) == 0xc2U);
		spi_disconnect(&connection, 10U);

		spi_connect(&target, 0U, &connection);
		assert(spi_transfer(&connection, 0x06U, 11U) == 0U);
		spi_disconnect(&connection, 12U);
		spi_connect(&target, 0U, &connection);
		assert(spi_transfer(&connection, 0x05U, 13U) == 0U);
		assert(spi_transfer(&connection, 0U, 14U) == 0x02U);
		spi_disconnect(&connection, 15U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "imu.dtb");
	target.spi_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.spi_ops != NULL);
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		assert(spi_transfer(&connection, 0xf5U, 2U) == 0U);
		assert(spi_transfer(&connection, 0U, 3U) == 0x98U);
		spi_disconnect(&connection, 4U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "magnetometer.dtb");
	target.i2c_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.i2c_bound == 1U && target.i2c_ops != NULL);
	{
		uint8 command[2];
		uint8 result[7];
		AthrillDeviceI2cMessage messages[2] = {
			{command, 1U, FALSE, FALSE},
			{result, 1U, TRUE, TRUE},
		};
		command[0] = 0U;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert(result[0] == 0x10U);

		command[0] = 0x0dU;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert(result[0] == 0U);

		command[0] = 0x0aU;
		command[1] = 1U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 1U) == 0);
		command[0] = 0x02U;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = sizeof(result);
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert((result[0] & 1U) != 0U);
		assert(result[1] == 0xefU && result[2] == 0xffU);
		assert(result[3] == 0x43U && result[4] == 0U);
		assert(result[5] == 0x85U && result[6] == 0U);

		command[0] = 0x0dU;
		command[1] = 0xd0U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 1U) == 0);
		command[0] = 0x0dU;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = 1U;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert(result[0] == 0x50U);
		command[0] = 0x0aU;
		command[1] = 1U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 1U) == 0);
		command[0] = 0x02U;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = sizeof(result);
		assert(target.i2c_ops->transfer(target.i2c_instance,
			messages, 2U) == 0);
		assert(result[1] == 0xbeU && result[2] == 0xffU);
		assert(result[3] == 0x43U && result[4] == 0U);
		assert(result[5] == 0x10U && result[6] == 0x02U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "barometer.dtb");
	target.spi_bound = 0U;
	assert(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	assert(target.spi_bound == 1U && target.spi_ops != NULL);
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		assert(spi_transfer(&connection, 0xa2U, 2U) == 0U);
		assert(spi_transfer(&connection, 0U, 3U) == 0x9cU);
		assert(spi_transfer(&connection, 0U, 4U) == 0xbfU);
		spi_disconnect(&connection, 5U);
	}
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
