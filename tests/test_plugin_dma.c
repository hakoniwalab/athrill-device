/*
 * Test plugin for the ABI 1.14 DMA request lines (tests/test_dma_request.c):
 *
 * - vendor,dma-requester: a write to its register 0 sets its request named
 *   by bits 8..15 (0 "tx", 1 "rx", 2 "none") to bit 0; register 4 reads the
 *   last dma_request result.
 * - vendor,dma-master: the DMA provider of the tuples that name it; its
 *   registers read the last line it was told (0x00 count, 0x04 requester,
 *   0x08 index, 0x0c cell count, 0x10/0x14 cells 0/1, 0x18 asserted) and
 *   the level of each stream (0x40 + 4 * cells[0]).
 *
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include <stdlib.h>
#include <string.h>

#define STREAMS	16U

typedef struct {
	uint64 base;
	const AthrillDeviceResourceOps *resources;
	void *context;
	sint32 result;
	uint32 count;
	AthrillDeviceDmaRequestLine line;
	std_bool asserted;
	std_bool stream[STREAMS];
} TestDma;

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	TestDma *d = calloc(1U, sizeof(*d));
	uint32 base = 0U;
	if (d == NULL || args->node->ops->get_u32(args->node->context, "reg", 0U,
		&base) != 0) {
		free(d);
		return -1;
	}
	d->base = base;
	*instance = d;
	return 0;
}

static void destroy(void *instance)
{
	free(instance);
}

static sint32 master_request(void *instance, const AthrillDeviceDmaRequestLine *line,
	std_bool asserted)
{
	TestDma *d = instance;
	d->count++;
	d->line = *line;
	d->asserted = asserted;
	if (line->cell_count > 0U && line->cells[0] < STREAMS) {
		d->stream[line->cells[0]] = asserted;
	}
	return 0;
}

static const AthrillDeviceDmaProviderOps master_provider = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceDmaProviderOps), master_request
};

static sint32 connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	TestDma *d = instance;
	d->resources = args->resources;
	d->context = args->context;
	return 0;
}

static sint32 master_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	(void)connect(instance, args);
	return args->resources->dma_provide(args->context, &master_provider, instance);
}

static sint32 master_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	TestDma *d = instance;
	uint64 offset = address - d->base;
	(void)size;
	switch (offset) {
	case 0x00U: *result = d->count; break;
	case 0x04U: *result = d->line.requester; break;
	case 0x08U: *result = d->line.index; break;
	case 0x0cU: *result = d->line.cell_count; break;
	case 0x10U: *result = d->line.cells[0]; break;
	case 0x14U: *result = d->line.cells[1]; break;
	case 0x18U: *result = d->asserted; break;
	default:
		if (offset >= 0x40U && offset < 0x40U + 4U * STREAMS) {
			*result = d->stream[(offset - 0x40U) / 4U];
			break;
		}
		*result = 0U;
	}
	return 0;
}

static sint32 ignore_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	(void)instance; (void)address; (void)size; (void)value;
	return 0;
}

static sint32 requester_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	TestDma *d = instance;
	(void)size;
	*result = (address - d->base) == 4U ? (uint64)(uint32)d->result : 0U;
	return 0;
}

static sint32 requester_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	static const char *const names[] = { "tx", "rx", "none" };
	TestDma *d = instance;
	uint32 name = (uint32)(value >> 8) & 0xffU;
	(void)size;
	if ((address - d->base) == 0U && name < 3U) {
		d->result = d->resources->dma_request(d->context, names[name],
			(value & 1U) != 0U);
	}
	return 0;
}

static const AthrillDeviceMmioOps master_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), master_read, ignore_write
};
static const AthrillDeviceMmioOps requester_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), requester_read, requester_write
};

static const AthrillDeviceModelOps master_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceModelOps),
	"test-dma-master", "1", NULL, create, destroy,
	&master_mmio, NULL, NULL, NULL, master_connect, NULL, NULL, NULL, NULL
};
static const AthrillDeviceModelOps requester_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceModelOps),
	"test-dma-requester", "1", NULL, create, destroy,
	&requester_mmio, NULL, NULL, NULL, connect, NULL, NULL, NULL, NULL
};
static const char *const master_compatibles[] = { "vendor,dma-master" };
static const char *const requester_compatibles[] = { "vendor,dma-requester" };
static const AthrillDeviceModelRegistration registrations[] = {
	{ master_compatibles, 1U, &master_ops },
	{ requester_compatibles, 1U, &requester_ops },
};

#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const AthrillDevicePlugin athrill_device_plugin_v1 = {
	ATHRILL_DEVICE_PLUGIN_MAGIC, ATHRILL_DEVICE_ABI_MAJOR,
	ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDevicePlugin),
	"test-plugin-dma", "1", registrations, 2U
};
