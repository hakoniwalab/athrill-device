/* Test-only model for the ABI 1.19 tick-contract runtime rules.
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"

#include <stdlib.h>

enum {
	MODE_EVERY = 0,
	MODE_NONE_WITH_EVENT,
	MODE_DEADLINE_BUSY,
	MODE_NONE,
	MODE_DEADLINE_REQUEST_DURING,
	MODE_DEADLINE
};

typedef struct {
	const AthrillDeviceResourceOps *resources;
	void *context;
	uint64 base;
	uint32 ticks;
	uint32 mode;
} ContractModel;

static sint32 create(const AthrillDeviceCreateArgs *args, void **result)
{
	ContractModel *model = calloc(1U, sizeof(*model));
	uint32 base;
	if (model == NULL || args->node->ops->get_u32(args->node->context,
		"reg", 0U, &base) != 0) {
		free(model);
		return -1;
	}
	model->base = base;
	*result = model;
	return 0;
}

static void destroy(void *instance)
{
	free(instance);
}

static sint32 connect_model(void *instance, const AthrillDeviceConnectArgs *args)
{
	ContractModel *model = instance;
	model->resources = args->resources;
	model->context = args->context;
	return 0;
}

static sint32 reset(void *instance)
{
	ContractModel *model = instance;
	model->ticks = 0U;
	model->mode = MODE_NONE;
	return 0;
}

static sint32 read_model(void *instance, uint64 address, uint32 size,
	uint64 *value)
{
	ContractModel *model = instance;
	uint64 offset = address - model->base;
	if (size != 4U || value == NULL || address < model->base || offset >= 0x100U) {
		return -1;
	}
	*value = offset == 0U ? model->ticks :
		(offset == 4U ? model->mode : 0U);
	return 0;
}

static sint32 write_model(void *instance, uint64 address, uint32 size,
	uint64 value)
{
	ContractModel *model = instance;
	uint64 offset = address - model->base;
	if (size != 4U || address < model->base || offset >= 0x100U) {
		return -1;
	}
	if (offset == 4U) {
		model->mode = (uint32)value;
	} else if (offset == 8U && model->resources->request_tick != NULL) {
		model->resources->request_tick(model->context);
	}
	return 0;
}

static const AthrillDeviceMmioOps mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), read_model, write_model
};

static sint32 tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	ContractModel *model = instance;
	(void)now;
	model->ticks++;
	switch (model->mode) {
	case MODE_NONE_WITH_EVENT:
		result->contract = ATHRILL_DEVICE_TICK_NONE;
		result->next_event_interval = 40U;
		break;
	case MODE_DEADLINE_BUSY:
		result->contract = ATHRILL_DEVICE_TICK_DEADLINE;
		result->next_event_interval = 40U;
		result->can_skip = FALSE;
		break;
	case MODE_NONE:
		result->contract = ATHRILL_DEVICE_TICK_NONE;
		break;
	case MODE_DEADLINE_REQUEST_DURING:
		result->contract = ATHRILL_DEVICE_TICK_DEADLINE;
		result->next_event_interval = 40U;
		model->resources->request_tick(model->context);
		break;
	case MODE_DEADLINE:
		result->contract = ATHRILL_DEVICE_TICK_DEADLINE;
		result->next_event_interval = 40U;
		break;
	default:
		break;
	}
	return 0;
}

static const AthrillDeviceModelOps model_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "test-tick-contract",
	.model_version = "1",
	.create = create,
	.destroy = destroy,
	.mmio = &mmio,
	.connect = connect_model,
	.reset = reset,
	.tick = tick,
};
static const char *const compatibles[] = { "athrill,test-tick-contract" };
static const AthrillDeviceModelRegistration registrations[] = {
	{ compatibles, 1U, &model_ops }
};

#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const AthrillDevicePlugin athrill_device_plugin_v1 = {
	ATHRILL_DEVICE_PLUGIN_MAGIC, ATHRILL_DEVICE_ABI_MAJOR,
	ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDevicePlugin),
	"test-tick-contract-plugin", "1", registrations, 1U
};
