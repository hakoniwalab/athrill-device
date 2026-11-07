/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_sd.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

#define BLOCK_SIZE 512U
#define BLOCK_COUNT 2U

typedef struct {
	uint8 storage[BLOCK_SIZE * BLOCK_COUNT];
	uint32 command_indices[16];
	uint32 command_arguments[16];
	uint32 commands;
	uint32 reads;
	uint32 writes;
	uint32 completions;
} Card;

static sint32 command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	Card *card = context;
	TEST_CHECK(card->commands < 16U);
	card->command_indices[card->commands] = index;
	card->command_arguments[card->commands] = argument;
	card->commands++;
	if (index == 0U) {
		response->kind = ATHRILL_SD_RESPONSE_NONE;
	} else if (index == 8U) {
		/* R7: command index, voltage-accepted field and check pattern. */
		response->kind = ATHRILL_SD_RESPONSE_SHORT;
		response->has_index = TRUE;
		response->words[0] = 0x1aaU;
	} else if (index == 55U) {
		/* R1: APP_CMD is accepted before ACMD41. */
		response->kind = ATHRILL_SD_RESPONSE_SHORT;
		response->has_index = TRUE;
		response->words[0] = 0x120U;
	} else if (index == 41U) {
		/* R3: OCR has no command-index field. */
		response->kind = ATHRILL_SD_RESPONSE_SHORT;
		response->has_index = FALSE;
		response->words[0] = 0xc0ff8000U;
	} else if (index == 2U) {
		/* R2: the 128-bit CID response has no command-index field. */
		response->kind = ATHRILL_SD_RESPONSE_LONG;
		response->has_index = FALSE;
		response->words[0] = 0x11223344U;
		response->words[1] = 0x55667788U;
		response->words[2] = 0x99aabbccU;
		response->words[3] = 0xddeeff00U;
	} else if (index == 3U) {
		/* R6: assigned RCA plus card status, with command index. */
		response->kind = ATHRILL_SD_RESPONSE_SHORT;
		response->has_index = TRUE;
		response->words[0] = 0x12340700U;
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
	TEST_CHECK(length == (card->command_indices[card->commands - 1U] == 17U ?
		BLOCK_SIZE : BLOCK_SIZE * BLOCK_COUNT));
	card->reads++;
	memcpy(data, card->storage, length);
	return 0;
}

static sint32 write_data(void *context, const uint8 *data, uint32 length)
{
	Card *card = context;
	TEST_CHECK(length == (card->command_indices[card->commands - 1U] == 24U ?
		BLOCK_SIZE : BLOCK_SIZE * BLOCK_COUNT));
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

static void test_initialisation_response_types_and_fields(void)
{
	/* [ADV-SD-001.T1] CMD0/CMD8/CMD55+ACMD41/CMD2/CMD3; R1/R2/R3/R6/R7 */
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	static const uint32 sequence[] = {0U, 8U, 55U, 41U, 2U, 3U};
	uint32 index;

	TEST_CHECK(athrill_sd_command(protocol, 0U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_NONE);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_NONE);

	TEST_CHECK(athrill_sd_command(protocol, 8U, 0x1aaU, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.words[0] == 0x1aaU);

	TEST_CHECK(athrill_sd_command(protocol, 55U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.words[0] == 0x120U);

	TEST_CHECK(athrill_sd_command(protocol, 41U, 0x40300000U,
		&response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(!response.has_index && response.words[0] == 0xc0ff8000U);

	TEST_CHECK(athrill_sd_command(protocol, 2U, 0U,
		&response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_LONG);
	TEST_CHECK(!response.has_index);
	TEST_CHECK(response.words[0] == 0x11223344U);
	TEST_CHECK(response.words[1] == 0x55667788U);
	TEST_CHECK(response.words[2] == 0x99aabbccU);
	TEST_CHECK(response.words[3] == 0xddeeff00U);

	TEST_CHECK(athrill_sd_command(protocol, 3U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.words[0] == 0x12340700U);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_NONE);

	TEST_CHECK(card.commands == sizeof(sequence) / sizeof(sequence[0]));
	for (index = 0U; index < card.commands; index++) {
		TEST_CHECK(card.command_indices[index] == sequence[index]);
	}
	TEST_CHECK(card.command_arguments[1] == 0x1aaU);
	TEST_CHECK(card.command_arguments[3] == 0x40300000U);
	TEST_CHECK(card.command_arguments[4] == 0U);
	athrill_sd_protocol_destroy(protocol);
}

static void test_single_block_write_then_read_round_trip(void)
{
	/* [ADV-SD-002.T1] CMD24 then CMD17 on the same 512-byte block */
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 written[BLOCK_SIZE];
	uint8 read_back[BLOCK_SIZE];
	uint32 i;
	for (i = 0U; i < BLOCK_SIZE; i++) {
		written[i] = (uint8)(i ^ 0x5aU);
	}
	TEST_CHECK(athrill_sd_command(protocol, 24U, 7U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.words[0] == 0x900U);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_WRITE);
	TEST_CHECK(athrill_sd_write(protocol, written, sizeof(written)) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);

	memset(read_back, 0, sizeof(read_back));
	TEST_CHECK(athrill_sd_command(protocol, 17U, 7U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_READ);
	TEST_CHECK(athrill_sd_read(protocol, read_back, sizeof(read_back)) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);
	TEST_CHECK(memcmp(read_back, written, sizeof(written)) == 0);
	TEST_CHECK(card.commands == 2U);
	TEST_CHECK(card.command_indices[0] == 24U &&
		card.command_indices[1] == 17U);
	TEST_CHECK(card.command_arguments[0] == 7U &&
		card.command_arguments[1] == 7U);
	TEST_CHECK(card.writes == 1U && card.reads == 1U &&
		card.completions == 2U);
	athrill_sd_protocol_destroy(protocol);
}

static void test_multi_block_write_stop_read_stop(void)
{
	/* [ADV-SD-003.T1] CMD25/CMD12/CMD18/CMD12, preserving block order */
	Card card = {0};
	AthrillSdProtocol *protocol = create_protocol(&card);
	AthrillSdResponse response;
	uint8 written[BLOCK_SIZE * BLOCK_COUNT];
	uint8 read_back[BLOCK_SIZE * BLOCK_COUNT];
	static const uint32 sequence[] = {25U, 12U, 18U, 12U};
	uint32 index;

	fill_blocks(written);
	TEST_CHECK(athrill_sd_command(protocol, 25U, 9U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_WRITE);
	TEST_CHECK(athrill_sd_write(protocol, written, sizeof(written)) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);

	TEST_CHECK(athrill_sd_command(protocol, 12U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.data == ATHRILL_SD_DATA_NONE);

	memset(read_back, 0, sizeof(read_back));
	TEST_CHECK(athrill_sd_command(protocol, 18U, 9U, &response) == 0);
	TEST_CHECK(response.data == ATHRILL_SD_DATA_READ);
	TEST_CHECK(athrill_sd_read(protocol, read_back, sizeof(read_back)) == 0);
	TEST_CHECK(athrill_sd_transfer_done(protocol) == 0);

	TEST_CHECK(athrill_sd_command(protocol, 12U, 0U, &response) == 0);
	TEST_CHECK(response.kind == ATHRILL_SD_RESPONSE_SHORT);
	TEST_CHECK(response.has_index && response.data == ATHRILL_SD_DATA_NONE);

	TEST_CHECK(memcmp(read_back, written, sizeof(written)) == 0);
	TEST_CHECK(read_back[0] == 0U &&
		read_back[BLOCK_SIZE - 1U] == 0xffU);
	TEST_CHECK(read_back[BLOCK_SIZE] == 0xffU &&
		read_back[2U * BLOCK_SIZE - 1U] == 0U);
	TEST_CHECK(card.commands == sizeof(sequence) / sizeof(sequence[0]));
	for (index = 0U; index < card.commands; index++) {
		TEST_CHECK(card.command_indices[index] == sequence[index]);
	}
	TEST_CHECK(card.command_arguments[0] == 9U &&
		card.command_arguments[2] == 9U);
	TEST_CHECK(card.writes == 1U && card.reads == 1U &&
		card.completions == 2U);
	athrill_sd_protocol_destroy(protocol);
}

int main(void)
{
	test_initialisation_response_types_and_fields();
	test_single_block_write_then_read_round_trip();
	test_multi_block_write_stop_read_stop();
	puts("SD protocol tests passed");
	return 0;
}
