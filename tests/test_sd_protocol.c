/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_sd.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
	uint32 commands;
	uint32 reads;
	uint32 writes;
	uint32 completions;
	uint8 bytes[4];
} Card;

static sint32 command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	Card *card = context;
	card->commands++;
	response->kind = ATHRILL_SD_RESPONSE_SHORT;
	response->has_index = TRUE;
	response->words[0] = argument;
	response->data = index == 17U ? ATHRILL_SD_DATA_READ :
		index == 24U ? ATHRILL_SD_DATA_WRITE : ATHRILL_SD_DATA_NONE;
	return 0;
}

static sint32 read_data(void *context, uint8 *data, uint32 length)
{
	Card *card = context;
	assert(length == sizeof(card->bytes));
	card->reads++;
	memcpy(data, card->bytes, length);
	return 0;
}

static sint32 write_data(void *context, const uint8 *data, uint32 length)
{
	Card *card = context;
	assert(length == sizeof(card->bytes));
	card->writes++;
	memcpy(card->bytes, data, length);
	return 0;
}

static sint32 done(void *context)
{
	((Card *)context)->completions++;
	return 0;
}

int main(void)
{
	static const AthrillSdCardOps ops = {
		command, read_data, write_data, done
	};
	Card card = { 0U, 0U, 0U, 0U, { 1U, 2U, 3U, 4U } };
	AthrillSdProtocol *protocol = NULL;
	AthrillSdResponse response;
	uint8 data[4];
	assert(athrill_sd_protocol_create(&ops, &card, &protocol) == 0);
	assert(athrill_sd_command(protocol, 17U, 7U, &response) == 0);
	assert(response.words[0] == 7U && response.data == ATHRILL_SD_DATA_READ);
	assert(athrill_sd_pending(protocol) == ATHRILL_SD_DATA_READ);
	assert(athrill_sd_write(protocol, data, sizeof(data)) == -1);
	assert(athrill_sd_read(protocol, data, sizeof(data)) == 0);
	assert(memcmp(data, card.bytes, sizeof(data)) == 0);
	assert(athrill_sd_read(protocol, data, sizeof(data)) == -1);
	assert(athrill_sd_transfer_done(protocol) == 0);
	assert(athrill_sd_pending(protocol) == ATHRILL_SD_DATA_NONE);
	data[0] = 9U;
	assert(athrill_sd_command(protocol, 24U, 8U, &response) == 0);
	assert(athrill_sd_write(protocol, data, sizeof(data)) == 0);
	assert(athrill_sd_transfer_done(protocol) == 0);
	assert(card.commands == 2U && card.reads == 1U && card.writes == 1U &&
		card.completions == 2U && card.bytes[0] == 9U);
	athrill_sd_protocol_destroy(protocol);
	puts("SD protocol tests passed");
	return 0;
}
