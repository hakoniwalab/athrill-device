/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_protocol_sd.h"

#include <stdlib.h>
#include <string.h>

struct AthrillSdProtocol {
	const AthrillSdCardOps *ops;
	void *context;
	AthrillSdDataDirection pending;
	std_bool data_moved;
};

sint32 athrill_sd_protocol_create(const AthrillSdCardOps *card_ops,
	void *card_context, AthrillSdProtocol **protocol)
{
	AthrillSdProtocol *created;
	if (protocol == NULL) {
		return -1;
	}
	*protocol = NULL;
	if (card_ops == NULL || card_ops->command == NULL ||
	    card_ops->read == NULL || card_ops->write == NULL ||
	    card_ops->transfer_done == NULL) {
		return -1;
	}
	created = calloc(1U, sizeof(*created));
	if (created == NULL) {
		return -1;
	}
	created->ops = card_ops;
	created->context = card_context;
	*protocol = created;
	return 0;
}

void athrill_sd_protocol_destroy(AthrillSdProtocol *protocol)
{
	free(protocol);
}

sint32 athrill_sd_command(AthrillSdProtocol *protocol, uint32 index,
	uint32 argument, AthrillSdResponse *response)
{
	AthrillSdResponse result;
	if (protocol == NULL || response == NULL) {
		return -1;
	}
	memset(&result, 0, sizeof(result));
	if (protocol->ops->command(protocol->context, index, argument,
		&result) != 0 || result.kind > ATHRILL_SD_RESPONSE_LONG ||
	    result.data > ATHRILL_SD_DATA_WRITE) {
		return -1;
	}
	protocol->pending = result.data;
	protocol->data_moved = FALSE;
	*response = result;
	return 0;
}

sint32 athrill_sd_read(AthrillSdProtocol *protocol, uint8 *data,
	uint32 length)
{
	if (protocol == NULL || data == NULL || length == 0U ||
	    protocol->pending != ATHRILL_SD_DATA_READ || protocol->data_moved ||
	    protocol->ops->read(protocol->context, data, length) != 0) {
		return -1;
	}
	protocol->data_moved = TRUE;
	return 0;
}

sint32 athrill_sd_write(AthrillSdProtocol *protocol, const uint8 *data,
	uint32 length)
{
	if (protocol == NULL || data == NULL || length == 0U ||
	    protocol->pending != ATHRILL_SD_DATA_WRITE || protocol->data_moved ||
	    protocol->ops->write(protocol->context, data, length) != 0) {
		return -1;
	}
	protocol->data_moved = TRUE;
	return 0;
}

sint32 athrill_sd_transfer_done(AthrillSdProtocol *protocol)
{
	if (protocol == NULL || protocol->pending == ATHRILL_SD_DATA_NONE ||
	    !protocol->data_moved ||
	    protocol->ops->transfer_done(protocol->context) != 0) {
		return -1;
	}
	protocol->pending = ATHRILL_SD_DATA_NONE;
	protocol->data_moved = FALSE;
	return 0;
}

AthrillSdDataDirection athrill_sd_pending(const AthrillSdProtocol *protocol)
{
	return protocol == NULL ? ATHRILL_SD_DATA_NONE : protocol->pending;
}
