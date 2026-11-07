/*
 * An SD memory card (zephyr,sdmmc-disk), one model per Devicetree node, a
 * child of its SD controller's node; the card logic is sdcard_card.c
 * (ported unchanged from athrill-target-stm32's memory/sdcard.c).
 *
 * The card is the host image DEVICE_CONFIG_STM32_SDCARD_IMAGE (a raw disk
 * image), opened at reset. Without an image (or with one it cannot use)
 * the card answers no command, as the controller saw no card before. It
 * offers its operations to its controller through sd_card_provide
 * (ABI 1.15); the controller takes them with get_sd_card.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_protocol_sd.h"
#include "sdcard_card.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	SdcardType card;
} SdCardModel;

static sint32 card_command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	SdCardModel *m = context;
	SdcardResponse backend;
	uint32 i;

	sdcard_command(&m->card, index, argument, &backend);
	response->kind = (AthrillSdResponseKind)backend.kind;
	response->has_index = backend.has_index;
	for (i = 0U; i < 4U; i++) {
		response->words[i] = backend.resp[i];
	}
	response->data = (AthrillSdDataDirection)backend.data;
	return 0;
}

static sint32 card_read(void *context, uint8 *data, uint32 length)
{
	SdCardModel *m = context;

	sdcard_read(&m->card, data, length);
	return 0;
}

static sint32 card_write(void *context, const uint8 *data, uint32 length)
{
	SdCardModel *m = context;

	sdcard_write(&m->card, data, length);
	return 0;
}

static sint32 card_transfer_done(void *context)
{
	SdCardModel *m = context;

	sdcard_transfer_done(&m->card);
	return 0;
}

static const AthrillSdCardOps card_ops = {
	.command = card_command,
	.read = card_read,
	.write = card_write,
	.transfer_done = card_transfer_done,
};

static sint32 card_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	SdCardModel *m = calloc(1U, sizeof(*m));

	if (m == NULL || args->target_ops == NULL) {
		free(m);
		return -1;
	}
	m->target_ops = args->target_ops;
	m->target = args->target;
	m->card.fd = -1;
	*instance = m;
	return 0;
}

static void card_destroy(void *instance)
{
	SdCardModel *m = instance;

	sdcard_open(&m->card, NULL);	/* closes the image */
	free(m);
}

static sint32 card_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	return args->resources->sd_card_provide(args->context, &card_ops, instance);
}

static sint32 card_reset(void *instance)
{
	SdCardModel *m = instance;
	const char *image = NULL;

	if (m->target_ops->host_value == NULL ||
	    m->target_ops->host_value(m->target, "DEVICE_CONFIG_STM32_SDCARD_IMAGE", &image) != 0) {
		image = NULL;
	}
	sdcard_open(&m->card, image);
	return 0;
}

const AthrillDeviceModelOps athrill_device_sdcard_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "sdcard", "1", NULL,
	card_create, card_destroy, NULL, NULL, NULL, NULL,
	card_connect, card_reset, NULL
};
