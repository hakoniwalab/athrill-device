/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_PROTOCOL_SD_H
#define ATHRILL_PROTOCOL_SD_H

#include "athrill_device_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillSdProtocol AthrillSdProtocol;

typedef enum {
	ATHRILL_SD_RESPONSE_NONE = 0,
	ATHRILL_SD_RESPONSE_SHORT,
	ATHRILL_SD_RESPONSE_LONG
} AthrillSdResponseKind;

typedef enum {
	ATHRILL_SD_DATA_NONE = 0,
	ATHRILL_SD_DATA_READ,
	ATHRILL_SD_DATA_WRITE
} AthrillSdDataDirection;

typedef struct {
	AthrillSdResponseKind kind;
	std_bool has_index;
	uint32 words[4];
	AthrillSdDataDirection data;
} AthrillSdResponse;

typedef struct {
	sint32 (*command)(void *context, uint32 index, uint32 argument,
		AthrillSdResponse *response);
	sint32 (*read)(void *context, uint8 *data, uint32 length);
	sint32 (*write)(void *context, const uint8 *data, uint32 length);
	sint32 (*transfer_done)(void *context);
} AthrillSdCardOps;

sint32 athrill_sd_protocol_create(const AthrillSdCardOps *card_ops,
	void *card_context, AthrillSdProtocol **protocol);
void athrill_sd_protocol_destroy(AthrillSdProtocol *protocol);

sint32 athrill_sd_command(AthrillSdProtocol *protocol, uint32 index,
	uint32 argument, AthrillSdResponse *response);
sint32 athrill_sd_read(AthrillSdProtocol *protocol, uint8 *data,
	uint32 length);
sint32 athrill_sd_write(AthrillSdProtocol *protocol, const uint8 *data,
	uint32 length);
sint32 athrill_sd_transfer_done(AthrillSdProtocol *protocol);
AthrillSdDataDirection athrill_sd_pending(const AthrillSdProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif
