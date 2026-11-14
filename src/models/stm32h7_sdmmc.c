/*
 * The STM32H7 SDMMC v2 controller with its internal DMA (IDMA), RM0433
 * chapter 55 (offsets and bits from 55.9): the H7 half of the st,stm32-sdmmc
 * model (stm32_sdmmc.c chooses it for nodes marked athrill,stm32h7-sdmmc).
 *
 * Ported unchanged from athrill-target-stm32's stm32h7_sdmmc.c
 * (hakoniwalab/athrill-device#13 step 4-6): commands answered at once, the
 * data path moving DLEN bytes by IDMA to or from IDMABASE0 after the time
 * they take on the bus (started with the command, CMDTRANS, or by DCTRL
 * DTEN), DPSMACT while it runs, the interrupt the level of STA & MASK.
 * What changed with the move: one instance per node, the base and
 * interrupt from the node, the card from the node's child (an sdcard
 * model, get_sd_card), the CPU clock from time_frequency_hz, and a register
 * read ends the CPU batch (as every moved controller does).
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "stm32_sdmmc_internal.h"
#include "athrill_protocol_sd.h"
#include <stdlib.h>
#include <string.h>

#define R_CLKCR		0x04U
#define R_ARG		0x08U
#define R_CMD		0x0CU
#define R_RESPCMD	0x10U
#define R_RESP1		0x14U
#define R_DLEN		0x28U
#define R_DCTRL		0x2CU
#define R_DCOUNT	0x30U
#define R_STA		0x34U
#define R_ICR		0x38U
#define R_MASK		0x3CU
#define R_IDMACTRL	0x50U
#define R_IDMABASE0	0x58U
#define REGS		(0x84U / 4U)

#define CLKCR_WIDBUS_SHIFT	14U

#define CMD_CMDTRANS		(1U << 6)
#define CMD_CMDSTOP		(1U << 7)
#define CMD_WAITRESP_SHIFT	8U
#define CMD_CPSMEN		(1U << 12)

#define DCTRL_DTEN	(1U << 0)	/* start the data path (without CMDTRANS) */
#define DCTRL_DTDIR	(1U << 1)	/* card to controller */
#define IDMACTRL_IDMAEN	(1U << 0)

#define STA_DCRCFAIL	(1U << 1)
#define STA_CTIMEOUT	(1U << 2)
#define STA_DTIMEOUT	(1U << 3)
#define STA_CMDREND	(1U << 6)
#define STA_CMDSENT	(1U << 7)
#define STA_DATAEND	(1U << 8)
#define STA_DBCKEND	(1U << 10)
#define STA_DABORT	(1U << 11)
#define STA_DPSMACT	(1U << 12)
#define ICR_MASK	0x1FE00FFFU

#define KERNEL_HZ	240000000U	/* fmu-v6c PLL1Q, PX4 board clock profile */
#define COMMAND_BITS	100U

struct H7Sdmmc {
	Stm32SdmmcHost host;
	uint32 regs[REGS];
	uint32 sta;
	uint64 cpu_hz;
	uint64 now;
	AthrillSdProtocol *protocol;
	AthrillSdDataDirection pending;
	std_bool started;	/* the data path was started: CMDTRANS, or DCTRL DTEN */
	uint64 done_at;
	uint32 asserted;
};

static uint32 reg(const H7Sdmmc *sd, uint32 offset)
{
	return sd->regs[offset / 4U];
}

static void set_reg(H7Sdmmc *sd, uint32 offset, uint32 value)
{
	sd->regs[offset / 4U] = value;
}

static uint32 status(const H7Sdmmc *sd)
{
	return sd->sta | ((sd->done_at != 0U) ? STA_DPSMACT : 0U);
}

static void update_interrupt(H7Sdmmc *sd)
{
	uint32 level = status(sd) & reg(sd, R_MASK);
	uint32 rose = level & ~sd->asserted;

	sd->asserted = level;
	if ((level != 0U) && ((rose != 0U) || !stm32_sdmmc_irq_active(&sd->host))) {
		stm32_sdmmc_raise(&sd->host);
	}
}

/* CPU clocks for len bytes at SDMMC_CK and the configured bus width. */
static uint64 transfer_clocks(const H7Sdmmc *sd, uint32 len)
{
	uint32 clkcr = reg(sd, R_CLKCR);
	uint32 widths[4] = { 1U, 4U, 8U, 1U };
	uint32 div = clkcr & 0x3FFU;
	uint64 card_hz = (div == 0U) ? KERNEL_HZ : (KERNEL_HZ / (2U * div));
	uint64 cycles = ((uint64)len * 8U / widths[(clkcr >> CLKCR_WIDBUS_SHIFT) & 3U]) + COMMAND_BITS;

	return ((cycles * sd->cpu_hz) / card_hz) + 1U;
}

static void try_start(H7Sdmmc *sd)
{
	std_bool read = sd->pending == ATHRILL_SD_DATA_READ;

	if ((sd->pending == ATHRILL_SD_DATA_NONE) || !sd->started || (sd->done_at != 0U)
		|| ((reg(sd, R_IDMACTRL) & IDMACTRL_IDMAEN) == 0U) || (reg(sd, R_IDMABASE0) == 0U)
		|| (((reg(sd, R_DCTRL) & DCTRL_DTDIR) != 0U) != read) || (reg(sd, R_DLEN) == 0U)) {
		return;
	}
	sd->done_at = sd->now + transfer_clocks(sd, reg(sd, R_DLEN));
	update_interrupt(sd);
}

static void finish(H7Sdmmc *sd)
{
	uint32 len = reg(sd, R_DLEN);
	uint32 address = reg(sd, R_IDMABASE0);
	uint8 *data = (uint8 *)calloc(len, 1U);
	uint32 end = STA_DATAEND | STA_DBCKEND;

	/* A card that fails ends the data path with an error, never DATAEND
	 * (RM0433 55.10.11): no read data is a timeout, a refused write a CRC
	 * status error (#114). */
	if (sd->pending == ATHRILL_SD_DATA_READ) {
		if (athrill_sd_read(sd->protocol, data, len) == 0) {
			(void)sd->host.target_ops->memory_write(sd->host.target, address, data, len);
		}
		else {
			end = STA_DTIMEOUT;
		}
	}
	else {
		(void)sd->host.target_ops->memory_read(sd->host.target, address, data, len);
		if (athrill_sd_write(sd->protocol, data, len) != 0) {
			end = STA_DCRCFAIL;
		}
	}
	free(data);
	if (end == (STA_DATAEND | STA_DBCKEND)) {
		(void)athrill_sd_transfer_done(sd->protocol);
		set_reg(sd, R_DCOUNT, 0U);
	}
	sd->pending = ATHRILL_SD_DATA_NONE;
	sd->started = FALSE;
	sd->done_at = 0U;
	sd->sta |= end;
	update_interrupt(sd);
}

static void advance_to(H7Sdmmc *sd, uint64 now)
{
	if (now > sd->now) {
		sd->now = now;
	}
	try_start(sd);
	if ((sd->done_at != 0U) && (sd->now >= sd->done_at)) {
		finish(sd);
	}
}

static void command(H7Sdmmc *sd, uint32 cmd)
{
	AthrillSdResponse response;
	uint32 wait = (cmd >> CMD_WAITRESP_SHIFT) & 3U;
	uint32 i;

	memset(&response, 0, sizeof(response));
	/* RM0433 55.6.2: a command with CMDSTOP (CMD12) aborts the data path
	 * that is pending or running: no data moves, DABORT is set (#114). */
	if (((cmd & CMD_CMDSTOP) != 0U) && (sd->pending != ATHRILL_SD_DATA_NONE)) {
		sd->pending = ATHRILL_SD_DATA_NONE;
		sd->started = FALSE;
		sd->done_at = 0U;
		sd->sta |= STA_DABORT;
	}
	if (sd->protocol != NULL) {
		(void)athrill_sd_command(sd->protocol, cmd & 0x3FU, reg(sd, R_ARG),
			&response);
	}
	if ((wait == 0U) || (wait == 2U)) {
		sd->sta |= STA_CMDSENT;
	}
	else if (response.kind == ATHRILL_SD_RESPONSE_NONE) {
		sd->sta |= STA_CTIMEOUT;
	}
	else {
		set_reg(sd, R_RESPCMD, response.has_index ? (cmd & 0x3FU) : 0x3FU);
		for (i = 0U; i < 4U; i++) {
			set_reg(sd, R_RESP1 + (4U * i), response.words[i]);
		}
		sd->sta |= STA_CMDREND;
	}
	/* RM0433 55.5.4: the data path starts with the command (CMDTRANS, as
	 * NuttX does) or when DCTRL DTEN is written after it (ChibiOS's SDMMCv2
	 * driver sends CMD17/18/24/25 without CMDTRANS, then sets DTEN). */
	if (response.data != ATHRILL_SD_DATA_NONE) {
		sd->pending = response.data;
		sd->started = (cmd & CMD_CMDTRANS) != 0U;
		set_reg(sd, R_DCOUNT, reg(sd, R_DLEN));
		try_start(sd);
	}
	update_interrupt(sd);
}

uint32 stm32h7_sdmmc_read(H7Sdmmc *sd, uint32 offset)
{
	advance_to(sd, stm32_sdmmc_clock(&sd->host));
	if (offset == R_STA) {
		return status(sd);
	}
	if (offset == R_ICR) {
		return 0U;
	}
	return (offset < (REGS * 4U)) ? reg(sd, offset) : 0U;
}

void stm32h7_sdmmc_write(H7Sdmmc *sd, uint32 offset, uint32 value, uint32 mask)
{
	advance_to(sd, stm32_sdmmc_clock(&sd->host));
	if (offset == R_ICR) {
		sd->sta &= ~(value & ICR_MASK);
		update_interrupt(sd);
		return;
	}
	if ((offset == R_STA) || (offset == R_RESPCMD) || (offset == R_DCOUNT)) {
		return;
	}
	if (offset < (REGS * 4U)) {
		set_reg(sd, offset, (reg(sd, offset) & ~mask) | value);
	}
	if ((offset == R_CMD) && ((reg(sd, R_CMD) & CMD_CPSMEN) != 0U)) {
		command(sd, reg(sd, R_CMD));
	}
	else if ((offset == R_DCTRL) || (offset == R_DLEN) || (offset == R_IDMACTRL)
		|| (offset == R_IDMABASE0)) {
		if ((offset == R_DCTRL) && ((value & DCTRL_DTEN) != 0U) && (sd->pending != ATHRILL_SD_DATA_NONE)) {
			sd->started = TRUE;
		}
		if ((reg(sd, R_IDMACTRL) & IDMACTRL_IDMAEN) == 0U) {
			sd->done_at = 0U;
		}
		try_start(sd);
	}
	else if (offset == R_MASK) {
		update_interrupt(sd);
	}
}

H7Sdmmc *stm32h7_sdmmc_create(const Stm32SdmmcHost *host)
{
	H7Sdmmc *sd = calloc(1U, sizeof(*sd));

	if (sd != NULL) {
		sd->host = *host;
	}
	return sd;
}

void stm32h7_sdmmc_destroy(H7Sdmmc *sd)
{
	athrill_sd_protocol_destroy(sd->protocol);
	free(sd);
}

Stm32SdmmcHost *stm32h7_sdmmc_host(H7Sdmmc *sd)
{
	return &sd->host;
}

/* No card until the card is attached (its ops; NULL: none). */
void stm32h7_sdmmc_reset(H7Sdmmc *sd, const AthrillSdCardOps *card, void *card_instance)
{
	Stm32SdmmcHost host = sd->host;

	athrill_sd_protocol_destroy(sd->protocol);
	memset(sd, 0, sizeof(*sd));
	sd->host = host;
	sd->cpu_hz = host.target_ops->time_frequency_hz(host.target);
	if (card != NULL) {
		(void)athrill_sd_protocol_create(card, card_instance, &sd->protocol);
	}
}

void stm32h7_sdmmc_tick(H7Sdmmc *sd, uint64 now, AthrillDeviceTickResult *result)
{
	uint64 until;

	advance_to(sd, now);
	sd->asserted = status(sd) & reg(sd, R_MASK);
	if ((sd->asserted != 0U) && !stm32_sdmmc_irq_active(&sd->host)) {
		stm32_sdmmc_raise(&sd->host);
	}
	if (sd->done_at != 0U) {
		until = (sd->done_at > sd->now) ? (sd->done_at - sd->now) : 1U;
		if (until < result->next_event_interval) {
			result->next_event_interval = until;
		}
	}
}
