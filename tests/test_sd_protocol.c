/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_sd.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

#define BLOCK_SIZE 512U
#define BLOCK_COUNT 2U

typedef struct {
	uint8 storage[BLOCK_SIZE * BLOCK_COUNT];
	uint32 command_index;
	uint32 command_argument;
	uint32 commands;
	uint32 reads;
	uint32 writes;
	uint32 completions;
} Card;

static sint32 command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	Card *card = context;
	card->command_index = index;
	card->command_argument = argument;
	card->commands++;
	if (index == 0U) {
		response->kind = ATHRILL_SD_RESPONSE_NONE;
	} else if (index == 2U) {
		response->kind = ATHRILL_SD_RESPONSE_LONG;
		response->has_index = FALSE;
		response->words[0] = 0x11223344U;
		response->words[1] = 0x55667788U;
		response->words[2] = 0x99aabbccU;
		response->words[3] = 0xddeeff00U;
	} else {
		response->kind = ATHRILL_SD_RESPONSE_SHORT;
		response->has_index = TRUE;
		response->words[0] = 0x900U;
	}
	if (index == 17U || index == 18U) {
		response->data = ATHRILL_SD_DATA_READ;
	} else if (index == 24U || index == 25U) {
		response->data = ATHRILL_SD_DATA_WRITE;
	} else {
		response->data = ATHRILL_SD_DATA_NONE;
	}
	return 0;
}

static sint32 read_data(void *context, uint8 *data, uint32 length)
{
	Card *card = context;
	TEST_CHECK(length <= sizeof(card->storage));
	card->reads++;
	memcpy(data, card->storage, length);
	return 0;
}

static sint32 write_data(void *context, const uint8 *data, uint32 length)
{
	Card *card = context;
	TEST_CHECK(length <= sizeof(card->storage));
	card->writes++;
	memcpy(card->storage, data, length);
	return 0;
}

static sint32 done(void *context)
{
	((Card *)context)->completions++;
	return 0;
}

static const AthrillSdCardOps ops = {
	command, read_data, write_data, done
};

static AthrillSdProtocol *create_protocol(Card *card)
{
	AthrillSdProtocol *protocol = NULL;
	TEST_CHECK(athrill_sd_protocol_create(&ops, card, &protocol) == 0);
	return protocol;
}

static void fill_blocks(uint8 *data)
{
	uint32 i;
	for (i = 0U; i < BLOCK_SIZE; i++) {
		data[i] = (uint8)i;
		data[BLOCK_SIZE + i] = (uint8)(0xffU - (uint8)i);
	}
}

static void test_command_response_is_returned_unchanged(void)
{
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	TEST_CHECK(athrill_sd_command(protocol, 0U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_NONE);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_NONE);
	TEST_CHECK(athrill_sd_command(protocol, 2U, 0x12345678U,
		&response) == 0);
	TEST_CHECK(card.command_index == 2U);
	TEST_CHECK(card.command_argument == 0x12345678U);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_LONG);
	TEST_CHECK(!response.has_index);
	TEST_CHECK(response.words[0] == 0x11223344U);
	TEST_CHECK(response.words[1] == 0x55667788U);
	TEST_CHECK(response.words[2] == 0x99aabbccU);
	TEST_CHECK(response.words[3] == 0xddeeff00U);
	athrill_sd_protocol_destroy(protocol);
}

static void test_single_block_read_uses_cmd17(void)
{
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 data[BLOCK_SIZE];
	fill_blocks(card.storage);
	TEST_CHECK(athrill_sd_command(protocol, 17U, 7U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.words[0] == 0x900U);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_READ);
	TEST_CHECK(athrill_sd_pending(protocol) == ATHRILL_SD_DATA_READ);
	TEST_CHECK(athrill_sd_read(protocol, data, sizeof(data)) == 0);
	TEST_CHECK(memcmp(data, card.storage, BLOCK_SIZE) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);
	TEST_CHECK(athrill_sd_pending(protocol) == ATHRILL_SD_DATA_NONE);
	athrill_sd_protocol_destroy(protocol);
}

static void test_single_block_write_uses_cmd24(void)
{
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 data[BLOCK_SIZE];
	uint32 i;
	for (i = 0U; i < BLOCK_SIZE; i++) {
		data[i] = (uint8)(i ^ 0x5aU);
	}
	TEST_CHECK(athrill_sd_command(protocol, 24U, 8U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_WRITE);
	TEST_CHECK(athrill_sd_write(protocol, data, sizeof(data)) == 0);
	TEST_CHECK(memcmp(card.storage, data, BLOCK_SIZE) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);
	athrill_sd_protocol_destroy(protocol);
}

static void test_multi_block_read_preserves_block_order(void)
{
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 data[BLOCK_SIZE * BLOCK_COUNT];
	fill_blocks(card.storage);
	TEST_CHECK(athrill_sd_command(protocol, 18U, 0U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_READ);
	TEST_CHECK(athrill_sd_read(protocol, data, sizeof(data)) == 0);
	TEST_CHECK(memcmp(data, card.storage, sizeof(data)) == 0);
	TEST_CHECK(data[0] == 0U && data[BLOCK_SIZE - 1U] == 0xffU);
	TEST_CHECK(data[BLOCK_SIZE] == 0xffU &&
		data[2U * BLOCK_SIZE - 1U] == 0U);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);
	athrill_sd_protocol_destroy(protocol);
}

static void test_multi_block_write_preserves_block_order(void)
{
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 data[BLOCK_SIZE * BLOCK_COUNT];
	fill_blocks(data);
	TEST_CHECK(athrill_sd_command(protocol, 25U, 0U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_WRITE);
	TEST_CHECK(athrill_sd_write(protocol, data, sizeof(data)) == 0);
	TEST_CHECK(memcmp(card.storage, data, sizeof(data)) == 0);
	TEST_CHECK(card.writes == 1U);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);
	TEST_CHECK(card.completions == 1U);
	athrill_sd_protocol_destroy(protocol);
}

int main(void)
{
	test_command_response_is_returned_unchanged();
	test_single_block_read_uses_cmd17();
	test_single_block_write_uses_cmd24();
	test_multi_block_read_preserves_block_order();
	test_multi_block_write_preserves_block_order();
	puts("SD protocol tests passed");
	return 0;
}
