/*
 * The Hakoniwa sensor device's registers as a Devicetree model
 * (board/common/src/hako_sensor_registers_model.c), in the test-vehicle mode
 * (no Hakoniwa). Expected values follow from the register map
 * (hako_sensor_device_regs.h) and the test vehicle's definition: a sample
 * every period at 200 MHz, GPS with every 25th sample, dither -0.002 first.
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "hako_sensor_device_regs.h"
#include "hako_sensor_registers_model.h"
#include "test_check.h"

#include <string.h>

#define BASE 0x20000000ULL

static uint64 now;
static uint32 pulses;
static sint32 level;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c; (void)name; (void)data; (void)size;
	return -1;
}
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	(void)c;
	if (index != 0U) {
		return -1;
	}
	*address = BASE;
	*size = 0x1000U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, NULL, NULL, NULL, get_reg
};
static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency(void *t) { (void)t; return 200000000ULL; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool asserted)
{
	(void)t;
	TEST_CHECK(line.value == 5U);
	if (asserted) {
		TEST_CHECK(level == 0);
		pulses++;
	}
	level = asserted ? 1 : 0;
	return 0;
}
static sint32 host_value(void *t, const char *key, const char **value)
{
	static const struct { const char *key; const char *value; } values[] = {
		{ "HAKO_SENSOR_DEVICE_TEST_PERIOD", "4000" },
		{ "HAKO_SENSOR_DEVICE_TEST_LAT_DEG", "35.681236" },
		{ "HAKO_SENSOR_DEVICE_TEST_LON_DEG", "139.767125" },
		{ "HAKO_SENSOR_DEVICE_TEST_ALT_M", "40" },
		{ "HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA", "1012.77" },
		{ "HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C", "20" },
		{ "HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS", "0.307" },
		{ "HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS", "-0.040" },
		/* a whole negative number arrives as its 32-bit two's complement */
		{ "HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS", "4294967295" },
	};
	uint32 i;
	(void)t;
	for (i = 0U; i < sizeof(values) / sizeof(values[0]); i++) {
		if (strcmp(values[i].key, key) == 0) {
			*value = values[i].value;
			return 0;
		}
	}
	return -1;
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)c; (void)name; (void)index;
	line->value = 5U;
	return 0;
}

static const AthrillDeviceModelOps *m = &hako_sensor_registers_model_ops;
static void *dev;

static uint32 rd(uint64 offset, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, BASE + offset, size, &value) == 0);
	return (uint32)value;
}
static uint64 tick(uint64 clock)
{
	AthrillDeviceTickResult result;
	now = clock;
	result.next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result.can_skip = TRUE;
	TEST_CHECK(m->tick(dev, clock, &result) == 0);
	return result.next_event_interval;
}
static uint32 bits_of(float value)
{
	uint32 bits;
	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

int main(void)
{
	AthrillDeviceTargetOps target_ops;
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceResourceOps resources;
	AthrillDeviceConnectArgs connect;
	uint64 value;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.time_frequency_hz = time_frequency;
	target_ops.irq_set = irq_set;
	target_ops.host_value = host_value;
	memset(&node, 0, sizeof(node));
	node.path = "/soc/sensor@20000000";
	node.matched_compatible = "hakoniwa,sensor-registers";
	node.ops = &node_ops;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	connect.resources = &resources;
	connect.context = NULL;
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);

	/* identity: "HAKO", version 1; a halfword read gives the addressed half */
	TEST_CHECK(rd(HAKO_REG_ID, 4U) == 0x4f4b4148U);
	TEST_CHECK(rd(HAKO_REG_ID + 2U, 2U) == 0x4f4bU);
	TEST_CHECK(rd(HAKO_REG_ID + 1U, 1U) == 0x41U);
	TEST_CHECK(rd(HAKO_REG_VERSION, 4U) == 1U);
	/* writes are 32-bit only */
	TEST_CHECK(m->mmio->write(dev, BASE + HAKO_REG_IRQ_ENABLE, 1U, 0xfU) != 0);
	TEST_CHECK(m->mmio->write(dev, BASE + HAKO_REG_IRQ_ENABLE, 4U, HAKO_STATUS_ALL) == 0);

	/* the first sample is due at 4 ms = 800,000 clocks */
	TEST_CHECK(tick(0U) == 800000U);
	TEST_CHECK(rd(HAKO_REG_STATUS, 4U) == 0U && pulses == 0U);
	TEST_CHECK(tick(799999U) == 1U);
	TEST_CHECK(tick(800000U) == 800000U);
	/* sensor and GPS latched (GPS with sample 0): two interrupt pulses */
	TEST_CHECK(rd(HAKO_REG_STATUS, 4U) == HAKO_STATUS_ALL);
	TEST_CHECK(pulses == 2U && level == 0);
	TEST_CHECK(rd(HAKO_REG_SENSOR_TIME_LO, 4U) == 4000U && rd(HAKO_REG_SENSOR_TIME_HI, 4U) == 0U);
	TEST_CHECK(rd(HAKO_REG_ACC_Z, 4U) == bits_of(-9.80665F + -0.002F));
	TEST_CHECK(rd(HAKO_REG_MAG_X, 4U) == bits_of((float)0.307 + -0.002F));
	TEST_CHECK(rd(HAKO_REG_MAG_Z, 4U) == bits_of((float)-1.0 + -0.002F));
	TEST_CHECK(rd(HAKO_REG_ABS_PRESSURE, 4U) == bits_of((float)1012.77 + -0.002F * 10.0F));
	TEST_CHECK(rd(HAKO_REG_GPS_LAT, 4U) == 356812360U);
	TEST_CHECK(rd(HAKO_REG_GPS_LON, 4U) == 1397671250U);
	TEST_CHECK(rd(HAKO_REG_GPS_ALT, 4U) == 40000U);
	TEST_CHECK(rd(HAKO_REG_GPS_FIX_TYPE, 4U) == 3U);
	/* STATUS: write 1 to clear */
	TEST_CHECK(m->mmio->write(dev, BASE + HAKO_REG_STATUS, 4U, HAKO_STATUS_ALL) == 0);
	TEST_CHECK(rd(HAKO_REG_STATUS, 4U) == 0U);
	/* the second sample: sensor only (GPS every 25th), one pulse */
	TEST_CHECK(tick(1600000U) == 800000U);
	TEST_CHECK(rd(HAKO_REG_STATUS, 4U) == (HAKO_STATUS_IMU | HAKO_STATUS_MAG | HAKO_STATUS_BARO));
	TEST_CHECK(pulses == 3U);
	TEST_CHECK(rd(HAKO_REG_ACC_Z, 4U) == bits_of(-9.80665F + 0.001F));
	/* outside the block */
	TEST_CHECK(m->mmio->read(dev, BASE + 0x1000U, 4U, &value) != 0);
	m->destroy(dev);
	return 0;
}
