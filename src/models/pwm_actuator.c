/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_OUTPUTS 16U
#define PULSE_MIN_US 1000.0F
#define PULSE_MAX_US 2000.0F

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource sink;
	AthrillDevicePwmChannel outputs[MAX_OUTPUTS];
	uint32 output_count;
	uint64 clocks_per_us;
	uint64 pulses[MAX_OUTPUTS];
	float32 commands[MAX_OUTPUTS];
	FILE *trace;
} PwmActuator;

static float32 motor_command(uint64 pulse_us)
{
	float32 value;
	if (pulse_us == 0U) {
		return 0.0F;
	}
	value = ((float32)pulse_us - PULSE_MIN_US) /
		(PULSE_MAX_US - PULSE_MIN_US);
	if (value < 0.0F) {
		return 0.0F;
	}
	return value > 1.0F ? 1.0F : value;
}

static sint32 actuator_create(const AthrillDeviceCreateArgs *args,
	void **instance)
{
	PwmActuator *actuator = calloc(1U, sizeof(*actuator));
	uint64 frequency;
	const char *trace_path;
	if (actuator == NULL || args->target_ops->time_frequency_hz == NULL ||
	    args->target_ops->actuator_output == NULL ||
	    (frequency = args->target_ops->time_frequency_hz(args->target)) < 1000000U) {
		free(actuator);
		return -1;
	}
	actuator->target_ops = args->target_ops;
	actuator->target = args->target;
	actuator->node = args->node;
	actuator->clocks_per_us = frequency / 1000000U;
	if (args->target_ops->host_value != NULL &&
	    args->target_ops->host_value(args->target,
		"DEVICE_CONFIG_STM32_ACTUATOR_FILE", &trace_path) == 0) {
		actuator->trace = fopen(trace_path, "w");
	}
	*instance = actuator;
	return 0;
}

static void actuator_destroy(void *instance)
{
	PwmActuator *actuator = instance;
	if (actuator->trace != NULL) {
		fclose(actuator->trace);
	}
	free(actuator);
}

static sint32 actuator_connect(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	PwmActuator *actuator = instance;
	uint32 index;
	if (args->resources->get_pwm == NULL) {
		return -1;
	}
	for (index = 0U; index < MAX_OUTPUTS; index++) {
		if (args->resources->get_pwm(args->context, "pwms", index,
			&actuator->outputs[index]) != 0) {
			break;
		}
	}
	actuator->output_count = index;
	return index != 0U ? athrill_device_connect_value_source(actuator->node,
		args, &actuator->sink) : -1;
}

static sint32 actuator_reset(void *instance)
{
	PwmActuator *actuator = instance;
	uint32 index;
	memset(actuator->commands, 0, sizeof(actuator->commands));
	for (index = 0U; index < MAX_OUTPUTS; index++) {
		actuator->pulses[index] = (uint64)-1;
	}
	return 0;
}

static sint32 actuator_tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	PwmActuator *actuator = instance;
	std_bool changed = FALSE;
	uint32 index;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	for (index = 0U; index < actuator->output_count; index++) {
		AthrillDevicePwmSample sample;
		uint64 pulse = 0U;
		float32 command;
		if (actuator->outputs[index].ops->sample(
			actuator->outputs[index].instance,
			actuator->outputs[index].channel, &sample) != 0) {
			return -1;
		}
		if (sample.enabled && sample.period_clocks != 0U) {
			pulse = sample.high_clocks / actuator->clocks_per_us;
		}
		command = motor_command(pulse);
		if (pulse != actuator->pulses[index]) {
			actuator->pulses[index] = pulse;
			if (actuator->trace != NULL) {
				fprintf(actuator->trace,
					"output=%u channel=%u pulse_us=%llu command=%.6f\n",
					index + 1U, actuator->outputs[index].channel,
					(unsigned long long)pulse, command);
				fflush(actuator->trace);
			}
		}
		if (command != actuator->commands[index]) {
			actuator->commands[index] = command;
			changed = TRUE;
		}
	}
	if (!changed) {
		return 0;
	}
	if (actuator->sink.ops != NULL) {
		return actuator->sink.ops->actuator_output(actuator->sink.instance,
			actuator->commands, actuator->output_count);
	}
	return actuator->target_ops->actuator_output(actuator->target,
		actuator->commands, actuator->output_count);
}

const AthrillDeviceModelOps athrill_device_pwm_actuator_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "pwm-actuator-map", "1", NULL,
	actuator_create, actuator_destroy, NULL, NULL, NULL, NULL,
	actuator_connect, actuator_reset, actuator_tick, NULL
};
