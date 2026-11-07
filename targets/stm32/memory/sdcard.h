/*
 * An SDHC memory card in SD mode (SD Physical Layer Simplified
 * Specification): the card states, the commands NuttX's MMC/SD driver uses
 * and their responses, and the data of block reads and writes, kept in a
 * host image file (a raw disk image, e.g. FAT32 formatted on the host).
 *
 * Commands: CMD0, 2, 3, 7, 8, 9, 12, 13, 16, 17, 18, 24, 25, 55 and ACMD6,
 * 13, 23, 41, 42, 51. Any other command gets no response (a timeout), as
 * SDIO (CMD5) and MMC (CMD1) probes expect from an SD memory card.
 * Lightweight: no CRCs, no erase, no write protection, no card removal;
 * the card is ready at the first ACMD41.
 */
#ifndef ATHRILL_STM32_SDCARD_H
#define ATHRILL_STM32_SDCARD_H

#include "std_types.h"

#define SDCARD_BLOCK	512U

typedef enum {
	SDCARD_RESP_NONE = 0,	/* no response: the command times out, or needs none */
	SDCARD_RESP_SHORT,	/* 32 bits in resp[0] (R1, R1b, R3, R6, R7) */
	SDCARD_RESP_LONG,	/* 128 bits, resp[0] = bits 127..96 (R2) */
} SdcardRespType;

typedef enum {
	SDCARD_DATA_NONE = 0,
	SDCARD_DATA_READ,	/* the card sends data (sdcard_read) */
	SDCARD_DATA_WRITE,	/* the card receives data (sdcard_write) */
} SdcardDataType;

typedef struct {
	SdcardRespType kind;
	std_bool has_index;	/* RESPCMD is the command index (R1, R6, R7), else 0x3F */
	uint32 resp[4];
	SdcardDataType data;
} SdcardResponse;

typedef struct {
	int fd;			/* the image file, -1: no card */
	uint64 size;		/* bytes, a multiple of 512 KiB */
	uint32 state;		/* R1 CURRENT_STATE */
	std_bool app_cmd;	/* the next command is an application command (after CMD55) */
	uint16 rca;
	uint64 offset;		/* the next byte of a block read or write */
	uint8 reg[64];		/* the register a data command sends (SCR, SD status) */
	uint32 reg_len;		/* bytes of reg left to send; 0: the data are blocks */
	uint32 reg_pos;
	std_bool multi;		/* the data command is a multi-block one (ends at CMD12) */
} SdcardType;

/* Opens the image at path (NULL: no card); its size is rounded down to 512 KiB. */
void sdcard_open(SdcardType *card, const char *path);
std_bool sdcard_present(const SdcardType *card);
/* The card's answer to a command. */
void sdcard_command(SdcardType *card, uint32 index, uint32 arg, SdcardResponse *response);
/* Data of the last read command (blocks or a register), len bytes. */
void sdcard_read(SdcardType *card, uint8 *data, uint32 len);
/* Data of the last write command, len bytes. */
void sdcard_write(SdcardType *card, const uint8 *data, uint32 len);
/* The data transfer has ended: back to the transfer state, unless multi-block (CMD12 ends it). */
void sdcard_transfer_done(SdcardType *card);

#endif
