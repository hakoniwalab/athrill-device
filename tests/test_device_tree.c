/* SPDX-License-Identifier: MIT */
#include <stddef.h>
#include "athrill_device_if.h"
#include "athrill_protocol_i2c.h"
#include "athrill_protocol_spi.h"
#include "athrill_protocol_uart.h"

#include "test_check.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const AthrillDeviceModelOps athrill_device_stm32_tim_model;

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
	uint64 frequency;
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
	AthrillUartProtocol *uart_protocol;
	uint64 uart_time;
	uint8 uart_output[256];
	uint32 uart_output_count;
} TestTarget;

typedef struct {
	AthrillChannel *channel;
	AthrillSpiProtocol *protocol;
} TestSpiConnection;

typedef struct {
	AthrillChannel *channel;
	AthrillI2cProtocol *protocol;
	uint64 time;
} TestI2cConnection;

typedef struct {
	AthrillChannel *channel;
	AthrillUartProtocol *protocol;
} TestUartConnection;

static sint32 uart_child_receive(void *context, uint8 data,
	uint64 completion_time)
{
	TestTarget *target = context;
	(void)completion_time;
	return target->uart_ops->receive_byte(target->uart_instance, data);
}

static sint32 uart_owner_receive(void *context, uint8 data,
	uint64 completion_time)
{
	TestTarget *target = context;
	(void)completion_time;
	TEST_CHECK(target->uart_output_count < sizeof(target->uart_output));
	target->uart_output[target->uart_output_count++] = data;
	return 0;
}

static void uart_connect(TestTarget *target, TestUartConnection *connection)
{
	AthrillChannelConfig config;
	AthrillUartTiming timing = {1000000U, 100000U, 8U, 1U, FALSE};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &connection->channel) == 0);
	TEST_CHECK(athrill_uart_protocol_create(connection->channel, &timing,
		uart_child_receive, target, uart_owner_receive, target,
		&connection->protocol) == 0);
	TEST_CHECK(athrill_uart_open(connection->protocol, 0U) == 0);
	target->uart_protocol = connection->protocol;
	target->uart_time = 1U;
}

static void uart_transfer(TestTarget *target, uint8 data)
{
	uint64 completion;
	TEST_CHECK(athrill_uart_send_to_child(target->uart_protocol, data,
		target->uart_time++, &completion) == 0);
}

static void uart_disconnect(TestTarget *target,
	TestUartConnection *connection)
{
	TEST_CHECK(athrill_uart_close(connection->protocol,
		target->uart_time++) == 0);
	target->uart_protocol = NULL;
	athrill_uart_protocol_destroy(connection->protocol);
	athrill_channel_destroy(connection->channel);
}

static std_bool i2c_child_start(void *context, std_bool read,
	std_bool repeated, uint64 time)
{
	TestTarget *target = context;
	(void)repeated;
	(void)time;
	TEST_CHECK(target->i2c_ops->start != NULL);
	return target->i2c_ops->start(target->i2c_instance, read);
}

static std_bool i2c_child_write(void *context, uint8 data, uint64 time)
{
	TestTarget *target = context;
	(void)time;
	TEST_CHECK(target->i2c_ops->write_byte != NULL);
	target->i2c_ops->write_byte(target->i2c_instance, data);
	return TRUE;
}

static sint32 i2c_child_read(void *context, uint8 *data,
	std_bool master_ack, uint64 time)
{
	TestTarget *target = context;
	(void)master_ack;
	(void)time;
	TEST_CHECK(target->i2c_ops->read_byte != NULL);
	*data = target->i2c_ops->read_byte(target->i2c_instance);
	return 0;
}

static void i2c_child_stop(void *context, uint64 time)
{
	TestTarget *target = context;
	(void)time;
	TEST_CHECK(target->i2c_ops->stop != NULL);
	target->i2c_ops->stop(target->i2c_instance);
}

static const AthrillI2cChildOps i2c_child_ops = {
	i2c_child_start, i2c_child_write, i2c_child_read, i2c_child_stop
};

static void i2c_connect(TestTarget *target, uint16 address,
	TestI2cConnection *connection)
{
	AthrillChannelConfig config;
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_ADDRESS;
	config.selection.value = address;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &connection->channel) == 0);
	TEST_CHECK(athrill_i2c_protocol_create(connection->channel, &i2c_child_ops,
		target, &connection->protocol) == 0);
	connection->time = 1U;
}

static sint32 i2c_transfer_messages(TestI2cConnection *connection,
	uint16 address, AthrillDeviceI2cMessage *messages, uint32 count)
{
	uint32 message, byte;
	for (message = 0U; message < count; message++) {
		AthrillDeviceI2cMessage *current = &messages[message];
		if (athrill_i2c_start(connection->protocol, address, current->read,
			connection->time++) != 1) {
			return -1;
		}
		for (byte = 0U; byte < current->size; byte++) {
			if (current->read) {
				if (athrill_i2c_read(connection->protocol,
					&current->data[byte], byte + 1U < current->size,
					connection->time++) != 0) {
					return -1;
				}
			} else {
				std_bool ack;
				if (athrill_i2c_write(connection->protocol,
					current->data[byte], &ack,
					connection->time++) != 0 || !ack) {
					return -1;
				}
			}
		}
		if (current->stop && athrill_i2c_stop(connection->protocol,
			connection->time++) != 0) {
			return -1;
		}
	}
	return 0;
}

static void i2c_disconnect(TestI2cConnection *connection)
{
	athrill_i2c_protocol_destroy(connection->protocol);
	athrill_channel_destroy(connection->channel);
}

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
	TEST_CHECK(athrill_channel_create(&config, &connection->channel) == 0);
	TEST_CHECK(athrill_spi_protocol_create(connection->channel, &spi_child_ops,
		target, &connection->protocol) == 0);
	TEST_CHECK(athrill_spi_select(connection->protocol, chip_select, 1U) == 1);
}

static uint8 spi_transfer(TestSpiConnection *connection, uint8 transmitted,
	uint64 time)
{
	uint8 received;
	TEST_CHECK(athrill_spi_exchange(connection->protocol, transmitted, &received,
		time) == 0);
	return received;
}

static void spi_disconnect(TestSpiConnection *connection, uint64 time)
{
	TEST_CHECK(athrill_spi_deselect(connection->protocol, time) == 0);
	athrill_spi_protocol_destroy(connection->protocol);
	athrill_channel_destroy(connection->channel);
}

static sint32 mmio_register(void *context, uint64 address, uint64 size,
	const AthrillDeviceMmioOps *ops, void *instance,
	AthrillDeviceMmioToken *token)
{
	TestTarget *target = context;
	TEST_CHECK(target->registered < 16U);
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
	TEST_CHECK(spec->cell_count == 1U);
	target->irq_resolved++;
	line->value = spec->cells[0];
	return 0;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line,
	std_bool asserted)
{
	TestTarget *target = context;
	TEST_CHECK(asserted);
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
	TEST_CHECK(pin < 192U);
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
	TEST_CHECK(pin < 192U && callback != NULL);
	target->gpio_edges[pin] = callback;
	target->gpio_edge_instances[pin] = instance;
	return 0;
}

static sint32 gpio_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceGpioLine *line)
{
	TestTarget *target = context;
	TEST_CHECK(spec->cell_count == 2U);
	target->gpio_resolved++;
	line->value = target->gpio_resolved;
	return 0;
}

static sint32 dma_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceDmaChannel *channel)
{
	TestTarget *target = context;
	TEST_CHECK(spec->cell_count == 1U && spec->cells[0] == 9U);
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
	TEST_CHECK(chip_select == 0U);
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
	TEST_CHECK(controller != NULL && strstr(controller->path, "spi@") != NULL);
	TEST_CHECK(chip_select == 0U);
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
	TEST_CHECK(address == 0x0EU);
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
	TEST_CHECK(controller != NULL && strstr(controller->path, "i2c@") != NULL);
	TEST_CHECK(address == 0x0eU);
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
	TEST_CHECK(controller != NULL && strstr(controller->path, "uart@") != NULL);
	TEST_CHECK(peripheral != NULL);
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
	uint64 completion;
	TEST_CHECK(peripheral == target->uart_peripheral);
	if (target->uart_protocol != NULL) {
		return athrill_uart_send_from_child(target->uart_protocol, data,
			target->uart_time++, &completion);
	}
	TEST_CHECK(target->uart_output_count < sizeof(target->uart_output));
	target->uart_output[target->uart_output_count++] = data;
	return 0;
}

static uint64 time_frequency(void *context)
{
	TestTarget *target = context;
	return target->frequency != 0U ? target->frequency : 216000000U;
}

static uint64 time_now(void *context)
{
	return ((TestTarget *)context)->clock;
}

static sint32 actuator_output(void *context, const float32 *values,
	uint32 count)
{
	TestTarget *target = context;
	TEST_CHECK(count <= 16U);
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

static void check_pwm_timing(TestTarget *target, uint32 pulse_us,
	uint64 target_frequency)
{
	AthrillDevicePwmSample sample;
	const AthrillDevicePwmProviderOps *pwm =
		athrill_device_stm32_tim_model.pwm;
	TEST_CHECK(pwm != NULL && pwm->sample != NULL);
	TEST_CHECK(pwm->sample(target->instance, 1U, &sample) == 0);
	TEST_CHECK(sample.enabled);
	TEST_CHECK(sample.high_clocks ==
		target_frequency * (uint64)pulse_us / 1000000U);
	TEST_CHECK(sample.period_clocks ==
		target_frequency * 20000U / 1000000U);
}

static void expect_failure(const char *name, TestTarget *target)
{
	char dtb[1024];
	AthrillDeviceRuntime *runtime = NULL;
	path(dtb, sizeof(dtb), name);
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, target, &runtime) != 0);
	TEST_CHECK(runtime == NULL && target->registered == 0U);
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
	TEST_CHECK(argc == 4);

	memset(&target, 0, sizeof(target));
	path(dtb, sizeof(dtb), "valid.dtb");
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.registered == 1U && target.addresses[0] == 0x1000U &&
		target.sizes[0] == 0x100U);
	TEST_CHECK(target.ops->write(target.instance, target.addresses[0], 4U,
		0x12345678U) == 0);
	TEST_CHECK(target.ops->read(target.instance, target.addresses[0], 4U,
		&value) == 0 && value == 0x12345678U);
	TEST_CHECK(athrill_device_tick(runtime, 1U, &tick) == 0 && tick.can_skip);
	athrill_device_destroy(runtime);
	runtime = NULL;

	/* ABI 1.16: the GPIO lines live in the runtime; a level from outside
	 * goes through athrill_device_gpio_drive, a line's level is read back
	 * with athrill_device_gpio_level */
	path(dtb, sizeof(dtb), "gpio.dtb");
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.registered == 1U && target.addresses[0] == 0x40020000U &&
		target.sizes[0] == 0x400U);
	/* PA4 an output (MODER), driven high (ODR): its line reads high */
	TEST_CHECK(target.ops->write(target.instance, 0x40020000U, 4U, 1U << 8) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40020014U, 4U, 1U << 4) == 0);
	{
		std_bool level = FALSE;
		TEST_CHECK(athrill_device_gpio_level(runtime, "/gpio@40020000", 4U, &level) == 0 && level);
	}
	/* PA2 driven high from outside: IDR shows it */
	TEST_CHECK(athrill_device_gpio_drive(runtime, "/gpio@40020000", 2U, TRUE) == 0);
	TEST_CHECK(target.ops->read(target.instance, 0x40020010U, 4U,
		&value) == 0 && (value & (1U << 2)) != 0U);
	/* a pin of no controller */
	TEST_CHECK(athrill_device_gpio_drive(runtime, "/gpio@40020400", 2U, TRUE) != 0);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "exti.dtb");
	target.irq_resolved = 0U;
	target.irq_set_count = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	/* [ADV-DT-003.T1] EXTI owns only the window declared by its reg. */
	TEST_CHECK(target.registered == 1U &&
		target.addresses[0] == 0x40013c00U &&
		target.sizes[0] == 0x400U && target.irq_resolved == 7U);
	/* The reset-default route for line 4 is GPIOA; enable and drive PA4 high. */
	TEST_CHECK(target.ops->write(target.instance, 0x40013c00U, 4U, 1U << 4) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40013c08U, 4U, 1U << 4) == 0);
	TEST_CHECK(athrill_device_gpio_drive(runtime, "/gpio@40020000", 4U, TRUE) == 0);
	TEST_CHECK(target.irq_set_count == 1U && target.last_irq == 10U);
	TEST_CHECK(target.ops->read(target.instance, 0x40013c14U, 4U,
		&value) == 0 && (value & (1U << 4)) != 0U);
	/* the same level again: no edge */
	TEST_CHECK(athrill_device_gpio_drive(runtime, "/gpio@40020000", 4U, TRUE) == 0);
	TEST_CHECK(target.irq_set_count == 1U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.irq_resolved = 0U;

	path(dtb, sizeof(dtb), "timer-pwm.dtb");
	/* [ADV-PWM-002.T1] F765 TIM4: 108 MHz timer, PSC=107, ARR=19999. */
	target.registered = 0U;
	target.irq_resolved = 0U;
	target.actuator_count = 0U;
	target.clock = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.registered == 1U && target.irq_resolved == 1U);
	TEST_CHECK(target.ops->write(target.instance, 0x40000828U, 4U, 107U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x4000082cU, 4U, 19999U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40000834U, 4U, 1000U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40000818U, 4U, 6U << 4) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40000820U, 4U, 1U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40000814U, 4U, 1U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40000800U, 4U, 1U) == 0);
	check_pwm_timing(&target, 1000U, 216000000U);
	TEST_CHECK(target.ops->write(target.instance, 0x40000834U, 4U, 1500U) == 0);
	check_pwm_timing(&target, 1500U, 216000000U);
	TEST_CHECK(athrill_device_tick(runtime, 1U, &tick) == 0);
	TEST_CHECK(target.actuator_count == 1U &&
		target.actuator_values[0] > 0.499F &&
		target.actuator_values[0] < 0.501F);
	TEST_CHECK(target.ops->write(target.instance, 0x40000834U, 4U, 2000U) == 0);
	check_pwm_timing(&target, 2000U, 216000000U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.irq_resolved = 0U;

	path(dtb, sizeof(dtb), "timer-h7-pwm.dtb");
	/* [ADV-PWM-002.T1] H743 TIM1: 240 MHz timer, PSC=239, ARR=19999. */
	target.registered = 0U;
	target.irq_resolved = 0U;
	target.actuator_count = 0U;
	target.clock = 0U;
	target.frequency = 480000000U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.registered == 1U && target.irq_resolved == 4U);
	TEST_CHECK(target.ops->write(target.instance, 0x40010028U, 4U, 239U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x4001002cU, 4U, 19999U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010034U, 4U, 1000U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010018U, 4U, 6U << 4) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010020U, 4U, 1U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010014U, 4U, 1U) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010044U, 4U, 1U << 15) == 0);
	TEST_CHECK(target.ops->write(target.instance, 0x40010000U, 4U, 1U) == 0);
	check_pwm_timing(&target, 1000U, 480000000U);
	TEST_CHECK(target.ops->write(target.instance, 0x40010034U, 4U, 1500U) == 0);
	check_pwm_timing(&target, 1500U, 480000000U);
	TEST_CHECK(athrill_device_tick(runtime, 1U, &tick) == 0);
	TEST_CHECK(target.actuator_count == 1U &&
		target.actuator_values[0] > 0.499F &&
		target.actuator_values[0] < 0.501F);
	TEST_CHECK(target.ops->write(target.instance, 0x40010034U, 4U, 2000U) == 0);
	check_pwm_timing(&target, 2000U, 480000000U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.irq_resolved = 0U;
	target.frequency = 216000000U;

	path(dtb, sizeof(dtb), "gps.dtb");
	target.uart_bound = 0U;
	target.uart_output_count = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.uart_bound == 1U && target.uart_ops != NULL);
	{
		TestUartConnection connection;
		static const uint8 poll[] = {
			0xb5U, 0x62U, 0x0aU, 0x04U, 0U, 0U, 0x0eU, 0x34U
		};
		uart_connect(&target, &connection);
		for (i = 0U; i < sizeof(poll); i++) {
			uart_transfer(&target, poll[i]);
		}
		TEST_CHECK(target.uart_output_count > 6U);
		TEST_CHECK(target.uart_output[0] == 0xb5U &&
			target.uart_output[1] == 0x62U &&
			target.uart_output[2] == 0x0aU &&
			target.uart_output[3] == 0x04U);
		uart_disconnect(&target, &connection);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;
	target.uart_bound = 0U;
	TEST_CHECK(target.registered == 0U);

	path(dtb, sizeof(dtb), "order.dtb");
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.registered == 2U && target.addresses[0] == 0x3000U &&
		target.addresses[1] == 0x2000U);
	athrill_device_destroy(runtime);
	runtime = NULL;
	TEST_CHECK(target.registered == 0U);

	path(dtb, sizeof(dtb), "resources.dtb");
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	/* the GPIO tuple resolves in the runtime (ABI 1.16), not the target */
	TEST_CHECK(target.irq_resolved == 1U && target.gpio_resolved == 0U &&
		target.dma_resolved == 1U);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "wiring.dtb");
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.spi_bound == 1U && target.i2c_bound == 1U &&
		target.uart_bound == 1U);
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "fram.dtb");
	target.spi_bound = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.spi_bound == 1U && target.spi_ops != NULL);
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		TEST_CHECK(spi_transfer(&connection, 0x9fU, 2U) == 0U);
		for (i = 0U; i < 6U; i++) {
			TEST_CHECK(spi_transfer(&connection, 0U, 3U + i) == 0x7fU);
		}
		TEST_CHECK(spi_transfer(&connection, 0U, 9U) == 0xc2U);
		spi_disconnect(&connection, 10U);

		spi_connect(&target, 0U, &connection);
		TEST_CHECK(spi_transfer(&connection, 0x06U, 11U) == 0U);
		spi_disconnect(&connection, 12U);
		spi_connect(&target, 0U, &connection);
		TEST_CHECK(spi_transfer(&connection, 0x05U, 13U) == 0U);
		TEST_CHECK(spi_transfer(&connection, 0U, 14U) == 0x02U);
		spi_disconnect(&connection, 15U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "imu.dtb");
	target.spi_bound = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.spi_bound == 1U && target.spi_ops != NULL);
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		TEST_CHECK(spi_transfer(&connection, 0xf5U, 2U) == 0U);
		TEST_CHECK(spi_transfer(&connection, 0U, 3U) == 0x98U);
		spi_disconnect(&connection, 4U);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "magnetometer.dtb");
	target.i2c_bound = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.i2c_bound == 1U && target.i2c_ops != NULL);
	{
		TestI2cConnection connection;
		uint8 command[2];
		uint8 result[7];
		AthrillDeviceI2cMessage messages[2] = {
			{command, 1U, FALSE, FALSE},
			{result, 1U, TRUE, TRUE},
		};
		i2c_connect(&target, 0x0eU, &connection);
		command[0] = 0U;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 2U) == 0);
		TEST_CHECK(result[0] == 0x10U);

		command[0] = 0x0dU;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 2U) == 0);
		TEST_CHECK(result[0] == 0U);

		command[0] = 0x0aU;
		command[1] = 1U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 1U) == 0);
		command[0] = 0x02U;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = sizeof(result);
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 2U) == 0);
		TEST_CHECK((result[0] & 1U) != 0U);
		TEST_CHECK(result[1] == 0xefU && result[2] == 0xffU);
		TEST_CHECK(result[3] == 0x43U && result[4] == 0U);
		TEST_CHECK(result[5] == 0x85U && result[6] == 0U);

		command[0] = 0x0dU;
		command[1] = 0xd0U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 1U) == 0);
		command[0] = 0x0dU;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = 1U;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 2U) == 0);
		TEST_CHECK(result[0] == 0x50U);
		command[0] = 0x0aU;
		command[1] = 1U;
		messages[0].size = 2U;
		messages[0].stop = TRUE;
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 1U) == 0);
		command[0] = 0x02U;
		messages[0].size = 1U;
		messages[0].stop = FALSE;
		messages[1].size = sizeof(result);
		TEST_CHECK(i2c_transfer_messages(&connection, 0x0eU,
			messages, 2U) == 0);
		TEST_CHECK(result[1] == 0xbeU && result[2] == 0xffU);
		TEST_CHECK(result[3] == 0x43U && result[4] == 0U);
		TEST_CHECK(result[5] == 0x10U && result[6] == 0x02U);
		i2c_disconnect(&connection);
	}
	athrill_device_destroy(runtime);
	runtime = NULL;

	path(dtb, sizeof(dtb), "barometer.dtb");
	target.spi_bound = 0U;
	TEST_CHECK(build(dtb, NULL, 0U, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.spi_bound == 1U && target.spi_ops != NULL);
	{
		TestSpiConnection connection;
		spi_connect(&target, 0U, &connection);
		TEST_CHECK(spi_transfer(&connection, 0xa2U, 2U) == 0U);
		TEST_CHECK(spi_transfer(&connection, 0U, 3U) == 0x9cU);
		TEST_CHECK(spi_transfer(&connection, 0U, 4U) == 0xbfU);
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
	TEST_CHECK(build(dtb, plugins, 1U, &target_ops, &target, &runtime) == 0);
	athrill_device_destroy(runtime);
	runtime = NULL;
	plugins[0].library_path = argv[2];
	TEST_CHECK(build(dtb, plugins, 1U, &target_ops, &target, &runtime) != 0);
	/* [#52] a plugin built against an older minor, its tables unchanged */
	plugins[0].library_path = argv[3];
	TEST_CHECK(build(dtb, plugins, 1U, &target_ops, &target, &runtime) == 0);
	athrill_device_destroy(runtime);
	runtime = NULL;
	plugins[0].compatible = "athrill,test-mmio";
	plugins[0].library_path = argv[1];
	path(dtb, sizeof(dtb), "valid.dtb");
	TEST_CHECK(build(dtb, plugins, 1U, &target_ops, &target, &runtime) != 0);

	plugins[0].compatible = "vendor,plugin";
	plugins[1] = plugins[0];
	path(dtb, sizeof(dtb), "plugin.dtb");
	TEST_CHECK(build(dtb, plugins, 2U, &target_ops, &target, &runtime) != 0);

	bad_ops.abi_major = 2U;
	TEST_CHECK(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	bad_ops = target_ops;
	bad_ops.struct_size = sizeof(AthrillDeviceTargetOps) - 1U;
	TEST_CHECK(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	/* [#52] a target table without the 1.10 fields is still refused; with
	 * them, an older minor number is accepted */
	path(dtb, sizeof(dtb), "valid.dtb");
	bad_ops = target_ops;
	bad_ops.abi_minor = 9U;
	bad_ops.struct_size = (uint32)offsetof(AthrillDeviceTargetOps, console_write);
	TEST_CHECK(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) != 0);
	bad_ops.struct_size = sizeof(AthrillDeviceTargetOps);
	TEST_CHECK(build(dtb, NULL, 0U, &bad_ops, &target, &runtime) == 0);
	athrill_device_destroy(runtime);
	runtime = NULL;
	puts("device tree framework tests passed");
	return 0;
}
