/* Test-only models for the ABI 1.20 ESC fault entry point.
 *
 * "athrill,test-esc-consumer" stands in for an ESC: from connect it asks
 * for the fault model its "fault-model" edge names (get_esc_fault) and,
 * like an ESC, passes everything when there is none. Registers:
 *   0x00 R  1 when get_esc_fault succeeded at connect, else 0
 *   0x04 W  motor index used by the next calls
 *   0x08 W  a received frame: command() with it (valid, value, request decoded)
 *   0x0C R  action of the last command() (0 pass, 1 drop, 2 replace)
 *   0x10 R  value that reaches the motor after it (0xFFFF when dropped)
 *   0x14 W  a measured current in mA: telemetry() with it (eRPM = 10 x mA)
 *   0x18 R  action of the last telemetry()
 *   0x1C R  current in mA that is answered (0xFFFFFFFF when dropped)
 *   0x20 R  eRPM that is answered
 *
 * "athrill,test-esc-fault" is a fault model from a plugin. Registers:
 *   0x00 W  mode: 0 pass, 1 drop, 2 replace (value 48, current 0, eRPM 0)
 *   0x04 R  motor of the last call
 *   0x08 R  time of the last call (low 32 bits)
 *   0x0C R  calls so far
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include <stddef.h>
#include <stdlib.h>

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 base;
	const AthrillDeviceEscFaultOps *fault;
	void *fault_instance;
	uint32 found;
	uint32 motor;
	uint32 command_action;
	uint32 command_value;
	uint32 telemetry_action;
	uint32 telemetry_ma;
	uint32 telemetry_erpm;
} Consumer;

typedef struct {
	uint64 base;
	uint32 mode;
	uint32 motor;
	uint64 time;
	uint32 calls;
} Fault;

static sint32 base_of(const AthrillDeviceCreateArgs *args, uint64 *base)
{
	uint32 value;
	if (args->node->ops->get_u32(args->node->context, "reg", 0U, &value) != 0) {
		return -1;
	}
	*base = value;
	return 0;
}

/* ---- the consumer (an ESC stand-in) ---- */
static sint32 consumer_create(const AthrillDeviceCreateArgs *args, void **result)
{
	Consumer *c = calloc(1U, sizeof(*c));
	if (c == NULL || base_of(args, &c->base) != 0) {
		free(c);
		return -1;
	}
	c->target_ops = args->target_ops;
	c->target = args->target;
	*result = c;
	return 0;
}

static sint32 consumer_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Consumer *c = instance;
	const AthrillDeviceResourceOps *r = args->resources;
	if (r->struct_size < offsetof(AthrillDeviceResourceOps, get_esc_fault) +
	    sizeof(r->get_esc_fault)) {
		return -1;
	}
	c->found = (r->get_esc_fault(args->context, "fault-model", &c->fault,
		&c->fault_instance) == 0) ? 1U : 0U;
	if (c->found == 0U) {
		c->fault = NULL;	/* none: bypass */
	}
	return 0;
}

static sint32 consumer_reset(void *instance)
{
	Consumer *c = instance;
	c->motor = 0U;
	c->command_action = 0U;
	c->command_value = 0U;
	c->telemetry_action = 0U;
	c->telemetry_ma = 0U;
	c->telemetry_erpm = 0U;
	return 0;
}

static sint32 consumer_read(void *instance, uint64 address, uint32 size, uint64 *value)
{
	Consumer *c = instance;
	uint64 offset = address - c->base;
	if (size != 4U || value == NULL || address < c->base || offset >= 0x100U) {
		return -1;
	}
	switch (offset) {
	case 0x00U: *value = c->found; break;
	case 0x0CU: *value = c->command_action; break;
	case 0x10U: *value = c->command_value; break;
	case 0x18U: *value = c->telemetry_action; break;
	case 0x1CU: *value = c->telemetry_ma; break;
	case 0x20U: *value = c->telemetry_erpm; break;
	default: *value = 0U; break;
	}
	return 0;
}

static void consumer_command(Consumer *c, uint16 frame)
{
	AthrillDeviceEscCommand cmd;
	AthrillDeviceEscFaultAction action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	cmd.frame = frame;
	cmd.valid = TRUE;
	cmd.value = (uint16)(frame >> 5);
	cmd.telemetry_request = ((frame >> 4) & 1U) != 0U;
	if (c->fault != NULL &&
	    c->fault->command(c->fault_instance, c->motor,
		c->target_ops->time_now(c->target), &cmd, &action) != 0) {
		action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	}
	c->command_action = (uint32)action;
	c->command_value = (action == ATHRILL_DEVICE_ESC_FAULT_DROP) ? 0xFFFFU : cmd.value;
}

static void consumer_telemetry(Consumer *c, uint32 milliamps)
{
	AthrillDeviceEscTelemetry tel;
	AthrillDeviceEscFaultAction action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	tel.temperature_c = 30.0F;
	tel.voltage_v = 16.0F;
	tel.current_a = (float32)milliamps / 1000.0F;
	tel.consumption_mah = 0.0F;
	tel.erpm = (float32)milliamps * 10.0F;
	if (c->fault != NULL &&
	    c->fault->telemetry(c->fault_instance, c->motor,
		c->target_ops->time_now(c->target), &tel, &action) != 0) {
		action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	}
	c->telemetry_action = (uint32)action;
	if (action == ATHRILL_DEVICE_ESC_FAULT_DROP) {
		c->telemetry_ma = 0xFFFFFFFFU;
		c->telemetry_erpm = 0xFFFFFFFFU;
	} else {
		c->telemetry_ma = (uint32)(tel.current_a * 1000.0F + 0.5F);
		c->telemetry_erpm = (uint32)(tel.erpm + 0.5F);
	}
}

static sint32 consumer_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	Consumer *c = instance;
	uint64 offset = address - c->base;
	if (size != 4U || address < c->base || offset >= 0x100U) {
		return -1;
	}
	switch (offset) {
	case 0x04U: c->motor = (uint32)value; break;
	case 0x08U: consumer_command(c, (uint16)value); break;
	case 0x14U: consumer_telemetry(c, (uint32)value); break;
	default: break;
	}
	return 0;
}

static const AthrillDeviceMmioOps consumer_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), consumer_read, consumer_write
};

static const AthrillDeviceModelOps consumer_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "test-esc-consumer",
	.model_version = "1",
	.create = consumer_create,
	.destroy = free,
	.mmio = &consumer_mmio,
	.connect = consumer_connect,
	.reset = consumer_reset,
};

/* ---- a fault model from a plugin ---- */
static void seen(Fault *f, uint32 motor, uint64 time)
{
	f->motor = motor;
	f->time = time;
	f->calls++;
}

static sint32 fault_command(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscCommand *command, AthrillDeviceEscFaultAction *action)
{
	Fault *f = instance;
	seen(f, motor, time);
	*action = (AthrillDeviceEscFaultAction)f->mode;
	if (f->mode == 2U) {
		command->value = 48U;
	}
	return 0;
}

static sint32 fault_telemetry(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscTelemetry *telemetry, AthrillDeviceEscFaultAction *action)
{
	Fault *f = instance;
	seen(f, motor, time);
	*action = (AthrillDeviceEscFaultAction)f->mode;
	if (f->mode == 2U) {
		telemetry->current_a = 0.0F;
		telemetry->erpm = 0.0F;
	}
	return 0;
}

static const AthrillDeviceEscFaultOps fault_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceEscFaultOps), fault_command, fault_telemetry
};

static sint32 fault_create(const AthrillDeviceCreateArgs *args, void **result)
{
	Fault *f = calloc(1U, sizeof(*f));
	if (f == NULL || base_of(args, &f->base) != 0) {
		free(f);
		return -1;
	}
	*result = f;
	return 0;
}

static sint32 fault_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	const AthrillDeviceResourceOps *r = args->resources;
	if (r->struct_size < offsetof(AthrillDeviceResourceOps, esc_fault_provide) +
	    sizeof(r->esc_fault_provide)) {
		return -1;
	}
	return r->esc_fault_provide(args->context, &fault_ops, instance);
}

static sint32 fault_reset(void *instance)
{
	Fault *f = instance;
	f->mode = 0U;
	f->motor = 0U;
	f->time = 0U;
	f->calls = 0U;
	return 0;
}

static sint32 fault_read(void *instance, uint64 address, uint32 size, uint64 *value)
{
	Fault *f = instance;
	uint64 offset = address - f->base;
	if (size != 4U || value == NULL || address < f->base || offset >= 0x100U) {
		return -1;
	}
	switch (offset) {
	case 0x00U: *value = f->mode; break;
	case 0x04U: *value = f->motor; break;
	case 0x08U: *value = (uint32)f->time; break;
	case 0x0CU: *value = f->calls; break;
	default: *value = 0U; break;
	}
	return 0;
}

static sint32 fault_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	Fault *f = instance;
	uint64 offset = address - f->base;
	if (size != 4U || address < f->base || offset >= 0x100U) {
		return -1;
	}
	if (offset == 0x00U && value <= 2U) {
		f->mode = (uint32)value;
	}
	return 0;
}

static const AthrillDeviceMmioOps fault_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), fault_read, fault_write
};

static const AthrillDeviceModelOps fault_model_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "test-esc-fault",
	.model_version = "1",
	.create = fault_create,
	.destroy = free,
	.mmio = &fault_mmio,
	.connect = fault_connect,
	.reset = fault_reset,
};

static const char *const consumer_compatibles[] = { "athrill,test-esc-consumer" };
static const char *const fault_compatibles[] = { "athrill,test-esc-fault" };
static const AthrillDeviceModelRegistration registrations[] = {
	{ consumer_compatibles, 1U, &consumer_ops },
	{ fault_compatibles, 1U, &fault_model_ops }
};

#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const AthrillDevicePlugin athrill_device_plugin_v1 = {
	ATHRILL_DEVICE_PLUGIN_MAGIC, ATHRILL_DEVICE_ABI_MAJOR,
	ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDevicePlugin),
	"test-esc-fault-plugin", "1", registrations, 2U
};
