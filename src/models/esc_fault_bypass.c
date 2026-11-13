/*
 * The ESC fault model that injects nothing: every command and every
 * telemetry answer passes unchanged (ABI 1.20, AthrillDeviceEscFaultOps).
 *
 * It is what an ESC does with no fault-model edge, as a node an ESC can
 * name, and the template of a fault model: a model with no registers
 * that offers its operations from connect (esc_fault_provide).
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdlib.h>
#include <stddef.h>

typedef struct {
	uint32 unused;
} EscFaultBypass;

static sint32 bypass_command(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscCommand *command, AthrillDeviceEscFaultAction *action)
{
	(void)instance;
	(void)motor;
	(void)time;
	if (command == NULL || action == NULL) {
		return -1;
	}
	*action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	return 0;
}

static sint32 bypass_telemetry(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscTelemetry *telemetry, AthrillDeviceEscFaultAction *action)
{
	(void)instance;
	(void)motor;
	(void)time;
	if (telemetry == NULL || action == NULL) {
		return -1;
	}
	*action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	return 0;
}

static const AthrillDeviceEscFaultOps bypass_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceEscFaultOps), bypass_command, bypass_telemetry
};

static sint32 bypass_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	EscFaultBypass *b = calloc(1U, sizeof(*b));

	(void)args;
	if (b == NULL) {
		return -1;
	}
	*instance = b;
	return 0;
}

static sint32 bypass_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	const AthrillDeviceResourceOps *r = args->resources;

	if (r == NULL || r->struct_size < offsetof(AthrillDeviceResourceOps, esc_fault_provide) +
	    sizeof(r->esc_fault_provide) || r->esc_fault_provide == NULL) {
		return -1;
	}
	return r->esc_fault_provide(args->context, &bypass_ops, instance);
}

static sint32 bypass_reset(void *instance)
{
	(void)instance;
	return 0;
}

const AthrillDeviceModelOps athrill_device_esc_fault_bypass_model = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "esc-fault-bypass",
	.model_version = "1",
	.create = bypass_create,
	.destroy = free,
	.connect = bypass_connect,
	.reset = bypass_reset,
};
