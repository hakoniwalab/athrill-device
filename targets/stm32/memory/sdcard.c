/*
 * The SDHC memory card (see sdcard.h).
 */
#include "sdcard.h"
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define STATE_IDLE	0U
#define STATE_READY	1U
#define STATE_IDENT	2U
#define STATE_STBY	3U
#define STATE_TRAN	4U
#define STATE_DATA	5U
#define STATE_RCV	6U
#define STATE_PRG	7U

#define R1_READY_FOR_DATA	(1U << 8)
#define R1_APP_CMD		(1U << 5)

#define OCR_BUSY	(1U << 31)	/* power-up done */
#define OCR_CCS		(1U << 30)	/* high capacity: block addressing */
#define OCR_VDD		0x00FF8000U	/* 2.7 to 3.6 V */

#define CARD_RCA	0x1234U
#define MIB_HALF	(512U * 1024U)

void sdcard_open(SdcardType *card, const char *path)
{
	struct stat st;

	if (card->fd >= 0) {
		close(card->fd);
	}
	memset(card, 0, sizeof(*card));
	card->fd = -1;
	card->state = STATE_IDLE;
	if (path == NULL) {
		return;
	}
	card->fd = open(path, O_RDWR);
	if ((card->fd < 0) || (fstat(card->fd, &st) != 0) || (st.st_size < (off_t)MIB_HALF)) {
		printf("WARNING: can not use SD card image %s (missing, or under 512 KiB)\n", path);
		if (card->fd >= 0) {
			close(card->fd);
		}
		card->fd = -1;
		return;
	}
	card->size = ((uint64)st.st_size / MIB_HALF) * MIB_HALF;
}

std_bool sdcard_present(const SdcardType *card)
{
	return card->fd >= 0;
}

/* Sets bits [hi:lo] of a 128-bit register, r[0] = bits 127..96. */
static void set_bits(uint32 *r, uint32 hi, uint32 lo, uint32 value)
{
	uint32 bit;

	for (bit = lo; bit <= hi; bit++) {
		uint32 word = 3U - (bit / 32U);
		uint32 mask = 1U << (bit % 32U);

		if (((value >> (bit - lo)) & 1U) != 0U) {
			r[word] |= mask;
		}
		else {
			r[word] &= ~mask;
		}
	}
}

/* CID: an Athrill card, made 2026-11. */
static void cid(uint32 *r)
{
	memset(r, 0, 16U);
	set_bits(r, 127U, 120U, 0x00U);		/* MID */
	set_bits(r, 119U, 104U, ('A' << 8) | 'T');	/* OID */
	set_bits(r, 103U, 96U, 'A');		/* PNM "ATHRL" */
	set_bits(r, 95U, 64U, ('T' << 24) | ('H' << 16) | ('R' << 8) | 'L');
	set_bits(r, 63U, 56U, 0x10U);		/* PRV 1.0 */
	set_bits(r, 55U, 24U, 1U);		/* PSN */
	set_bits(r, 19U, 8U, (26U << 4) | 11U);	/* MDT: 2000 + 26, November */
	set_bits(r, 0U, 0U, 1U);
}

/* CSD version 2.0 (SDHC): 512-byte blocks, capacity (C_SIZE + 1) * 512 KiB. */
static void csd(const SdcardType *card, uint32 *r)
{
	memset(r, 0, 16U);
	set_bits(r, 127U, 126U, 1U);		/* CSD_STRUCTURE 2.0 */
	set_bits(r, 119U, 112U, 0x0EU);		/* TAAC 1 ms */
	set_bits(r, 103U, 96U, 0x32U);		/* TRAN_SPEED 25 MHz */
	set_bits(r, 95U, 84U, 0x5B5U);		/* CCC */
	set_bits(r, 83U, 80U, 9U);		/* READ_BL_LEN 512 */
	set_bits(r, 69U, 48U, (uint32)(card->size / MIB_HALF) - 1U);	/* C_SIZE */
	set_bits(r, 46U, 46U, 1U);		/* ERASE_BLK_EN */
	set_bits(r, 45U, 39U, 0x7FU);		/* SECTOR_SIZE */
	set_bits(r, 28U, 26U, 2U);		/* R2W_FACTOR */
	set_bits(r, 25U, 22U, 9U);		/* WRITE_BL_LEN 512 */
	set_bits(r, 0U, 0U, 1U);
}

static uint32 r1(const SdcardType *card, uint32 state)
{
	return (state << 9) | R1_READY_FOR_DATA | (card->app_cmd ? R1_APP_CMD : 0U);
}

static void short_response(SdcardResponse *response, uint32 value, std_bool has_index)
{
	response->kind = SDCARD_RESP_SHORT;
	response->has_index = has_index;
	response->resp[0] = value;
}

/* A block read or write from the block address arg. */
static void block_command(SdcardType *card, uint32 arg, uint32 state, SdcardDataType data, std_bool multi,
	SdcardResponse *response)
{
	card->multi = multi;
	short_response(response, r1(card, card->state), TRUE);
	card->offset = (uint64)arg * SDCARD_BLOCK;
	card->reg_len = 0U;
	card->state = state;
	response->data = data;
}

static void register_read(SdcardType *card, uint32 len, SdcardResponse *response)
{
	short_response(response, r1(card, card->state), TRUE);
	card->reg_len = len;
	card->reg_pos = 0U;
	card->multi = FALSE;
	card->state = STATE_DATA;
	response->data = SDCARD_DATA_READ;
}

static void app_command(SdcardType *card, uint32 index, uint32 arg, SdcardResponse *response)
{
	switch (index) {
	case 41U:	/* SD_SEND_OP_COND: ready at once */
		short_response(response, OCR_BUSY | OCR_CCS | OCR_VDD, FALSE);
		if (card->state == STATE_IDLE) {
			card->state = STATE_READY;
		}
		break;
	case 6U:	/* SET_BUS_WIDTH */
	case 23U:	/* SET_WR_BLK_ERASE_COUNT */
	case 42U:	/* SET_CLR_CARD_DETECT */
		short_response(response, r1(card, card->state), TRUE);
		break;
	case 51U:	/* SEND_SCR: SD 2.0, SDHC security, 1- and 4-bit bus */
		memset(card->reg, 0, sizeof(card->reg));
		card->reg[0] = 0x02U;
		card->reg[1] = 0x35U;
		register_read(card, 8U, response);
		break;
	case 13U:	/* SD_STATUS: 4-bit bus, nothing else reported */
		memset(card->reg, 0, sizeof(card->reg));
		card->reg[0] = 0x80U;
		register_read(card, 64U, response);
		break;
	default:
		break;
	}
	(void)arg;
}

void sdcard_command(SdcardType *card, uint32 index, uint32 arg, SdcardResponse *response)
{
	std_bool app = card->app_cmd;

	memset(response, 0, sizeof(*response));
	if (!sdcard_present(card)) {
		return;
	}
	if (app && (index != 55U)) {
		app_command(card, index, arg, response);
		card->app_cmd = FALSE;
		if (response->kind != SDCARD_RESP_NONE) {
			return;
		}
	}
	switch (index) {
	case 0U:	/* GO_IDLE_STATE */
		card->state = STATE_IDLE;
		card->app_cmd = FALSE;
		break;
	case 2U:	/* ALL_SEND_CID */
		response->kind = SDCARD_RESP_LONG;
		cid(response->resp);
		card->state = STATE_IDENT;
		break;
	case 3U:	/* SEND_RELATIVE_ADDR: R6 */
		card->rca = CARD_RCA;
		short_response(response, ((uint32)card->rca << 16) | (card->state << 9) | R1_READY_FOR_DATA, TRUE);
		card->state = STATE_STBY;
		break;
	case 7U:	/* SELECT/DESELECT_CARD */
		if ((arg >> 16) == card->rca) {
			short_response(response, r1(card, card->state), TRUE);
			card->state = STATE_TRAN;
		}
		else {
			card->state = STATE_STBY;	/* deselected: no response */
		}
		break;
	case 8U:	/* SEND_IF_COND: R7 echoes the voltage and the check pattern */
		short_response(response, arg & 0xFFFU, TRUE);
		break;
	case 9U:	/* SEND_CSD */
		response->kind = SDCARD_RESP_LONG;
		csd(card, response->resp);
		break;
	case 12U:	/* STOP_TRANSMISSION */
		short_response(response, r1(card, card->state), TRUE);
		card->state = STATE_TRAN;
		break;
	case 13U:	/* SEND_STATUS */
	case 16U:	/* SET_BLOCKLEN (512 on SDHC) */
		short_response(response, r1(card, card->state), TRUE);
		break;
	case 17U:	/* READ_SINGLE_BLOCK */
	case 18U:	/* READ_MULTIPLE_BLOCK */
		block_command(card, arg, STATE_DATA, SDCARD_DATA_READ, index == 18U, response);
		break;
	case 24U:	/* WRITE_BLOCK */
	case 25U:	/* WRITE_MULTIPLE_BLOCK */
		block_command(card, arg, STATE_RCV, SDCARD_DATA_WRITE, index == 25U, response);
		break;
	case 55U:	/* APP_CMD */
		card->app_cmd = TRUE;
		short_response(response, r1(card, card->state), TRUE);
		break;
	default:
		break;	/* no response */
	}
}

void sdcard_read(SdcardType *card, uint8 *data, uint32 len)
{
	uint32 n;

	if (card->reg_len != 0U) {
		n = (len < card->reg_len) ? len : card->reg_len;
		memcpy(data, &card->reg[card->reg_pos], n);
		memset(&data[n], 0, len - n);
		card->reg_pos += n;
		card->reg_len -= n;
		return;
	}
	memset(data, 0, len);
	if ((card->fd >= 0) && (card->offset < card->size)) {
		n = ((card->size - card->offset) < len) ? (uint32)(card->size - card->offset) : len;
		if (pread(card->fd, data, n, (off_t)card->offset) != (ssize_t)n) {
			printf("WARNING: SD card image read failed at %llu\n", (unsigned long long)card->offset);
		}
	}
	card->offset += len;
}

void sdcard_write(SdcardType *card, const uint8 *data, uint32 len)
{
	uint32 n;

	if ((card->fd >= 0) && (card->offset < card->size)) {
		n = ((card->size - card->offset) < len) ? (uint32)(card->size - card->offset) : len;
		if (pwrite(card->fd, data, n, (off_t)card->offset) != (ssize_t)n) {
			printf("WARNING: SD card image write failed at %llu\n", (unsigned long long)card->offset);
		}
	}
	card->offset += len;
}

void sdcard_transfer_done(SdcardType *card)
{
	if (!card->multi) {
		card->state = STATE_TRAN;
	}
}
