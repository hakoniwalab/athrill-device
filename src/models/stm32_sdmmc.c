/*
 * STM32 SDMMC (st,stm32-sdmmc), one model per Devicetree node, with its SD
 * card as the node's child (an sdcard model, ABI 1.15 get_sd_card). The
 * F7 SDMMC v1 (RM0410 chapter 35, offsets and bits from 35.8) is here; a
 * node marked athrill,stm32h7-sdmmc (the H7 SoC description: the H7 IP is
 * the SDMMC v2 with its internal DMA, RM0433 chapter 55) gets the H7 half
 * (stm32h7_sdmmc.c). Zephyr names both st,stm32-sdmmc.
 *
 * Ported unchanged from athrill-target-stm32's stm32_sdmmc.c
 * (hakoniwalab/athrill-device#13 step 4-6): writing CMD with CPSMEN sends
 * the command at once (RESPCMD/RESP1-4, CMDSENT/CMDREND/CTIMEOUT); the data
 * path (DCTRL, DLEN) moves the bytes of a read or write command by DMA or
 * through the FIFO register and ends with DATAEND and DBCKEND after the
 * time the bytes take on the bus at the configured clock and width; the
 * interrupt is the level of STA & MASK. No CRC errors, data timeouts, SDIO,
 * read wait or power-up delay.
 *
 * What changed with the move:
 * - One instance per node; base and interrupt from the node; the card from
 *   the node's child; the CPU clock from time_frequency_hz.
 * - DMA (F7) by request lines (ABI 1.14) instead of block transfers: with
 *   DMAEN, the controller raises "rx" while received data wait in the FIFO
 *   and "tx" while a write's data path waits for DLEN bytes; the DMA reads
 *   or writes the FIFO register on the bus. A read moves its data at the
 *   end of the bus time, as before. A write's data now come from memory
 *   when its stream starts (before, at the end of the bus time), so the
 *   stream completes before DATAEND, as on the chip.
 * - athrill,d0-gpios: the D0 line as the firmware reads it as a GPIO,
 *   driven high (the board's pull-up: the card is not busy); the target
 *   drove PC8 high before.
 * - Every register access ends the CPU batch: the target ended it on every
 *   F7 access and on H7 writes (board-independent now).
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "stm32_sdmmc_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SDMMC_BLOCK	0x400U

#define R_POWER		0x00U
#define R_CLKCR		0x04U
#define R_ARG		0x08U
#define R_CMD		0x0CU
#define R_RESPCMD	0x10U
#define R_RESP1		0x14U
#define R_DTIMER	0x24U
#define R_DLEN		0x28U
#define R_DCTRL		0x2CU
#define R_DCOUNT	0x30U
#define R_STA		0x34U
#define R_ICR		0x38U
#define R_MASK		0x3CU
#define R_FIFOCNT	0x48U
#define R_FIFO		0x80U
#define REGS		(0x84U / 4U)

#define CLKCR_BYPASS		(1U << 10)
#define CLKCR_WIDBUS_SHIFT	11U

#define CMD_WAITRESP_SHIFT	6U
#define CMD_CPSMEN		(1U << 10)

#define DCTRL_DTEN	(1U << 0)
#define DCTRL_DTDIR	(1U << 1)	/* card to controller */
#define DCTRL_DMAEN	(1U << 3)

#define STA_CTIMEOUT	(1U << 2)
#define STA_CMDREND	(1U << 6)
#define STA_CMDSENT	(1U << 7)
#define STA_DATAEND	(1U << 8)
#define STA_DBCKEND	(1U << 10)
#define STA_TXFIFOHE	(1U << 14)
#define STA_RXFIFOHF	(1U << 15)
#define STA_TXFIFOE	(1U << 18)
#define STA_RXFIFOE	(1U << 19)
#define STA_RXDAVL	(1U << 21)
#define ICR_MASK	0x004005FFU

#define KERNEL_HZ	48000000U	/* SDMMCCLK: PLL48CLK */
#define COMMAND_BITS	100U		/* a command, its response and the gaps around the data */

typedef struct {
	Stm32SdmmcHost host;
	uint32 regs[REGS];
	uint32 sta;		/* the static flags (the FIFO flags are worked out) */
	uint64 cpu_hz;
	uint64 now;
	AthrillSdProtocol *protocol;
	AthrillSdDataDirection pending;	/* a data command waits for its data path */
	uint64 done_at;		/* the clock the scheduled transfer ends; 0: none */
	uint8 *fifo;		/* the bytes of a transfer through the FIFO register */
	uint32 fifo_len;
	uint32 fifo_pos;
	uint32 asserted;
	/* DMA request lines and their last levels */
	const AthrillDeviceResourceOps *resources;
	void *context;
	std_bool rx_request;
	std_bool tx_request;
} SdmmcType;

static uint32 reg(const SdmmcType *sd, uint32 offset)
{
	return sd->regs[offset / 4U];
}

static void set_reg(SdmmcType *sd, uint32 offset, uint32 value)
{
	sd->regs[offset / 4U] = value;
}

static void drop_fifo(SdmmcType *sd)
{
	free(sd->fifo);
	sd->fifo = NULL;
	sd->fifo_len = 0U;
	sd->fifo_pos = 0U;
}

/* A write's data path takes the FIFO register: by the CPU (FIFO mode). */
static std_bool fifo_write_mode(const SdmmcType *sd)
{
	uint32 dctrl = reg(sd, R_DCTRL);

	return (sd->pending == ATHRILL_SD_DATA_WRITE) && (sd->done_at == 0U)
		&& ((dctrl & (DCTRL_DTEN | DCTRL_DTDIR | DCTRL_DMAEN)) == DCTRL_DTEN);
}

/* ... or by the DMA (DMAEN), until DLEN bytes are in. */
static std_bool dma_write_mode(const SdmmcType *sd)
{
	uint32 dctrl = reg(sd, R_DCTRL);

	return (sd->pending == ATHRILL_SD_DATA_WRITE) && (sd->done_at == 0U)
		&& ((dctrl & (DCTRL_DTEN | DCTRL_DTDIR | DCTRL_DMAEN)) == (DCTRL_DTEN | DCTRL_DMAEN))
		&& (sd->fifo_len < reg(sd, R_DLEN));
}

/* Received bytes wait in the FIFO register. */
static std_bool rx_waiting(const SdmmcType *sd)
{
	return (sd->fifo != NULL) && (sd->pending == ATHRILL_SD_DATA_NONE);
}

static uint32 status(const SdmmcType *sd)
{
	uint32 sta = sd->sta;
	uint32 words;

	if (rx_waiting(sd)) {
		/* received bytes waiting in the FIFO */
		words = (sd->fifo_len - sd->fifo_pos + 3U) / 4U;
		sta |= (words != 0U) ? STA_RXDAVL : STA_RXFIFOE;
		sta |= (words >= 8U) ? STA_RXFIFOHF : 0U;
	}
	else if (fifo_write_mode(sd)) {
		sta |= STA_TXFIFOHE | STA_TXFIFOE;
	}
	return sta;
}

/* The DMA requests (see the file comment); the DMA is told each change and
 * may move the data at once, coming back here. */
static void update_requests(SdmmcType *sd)
{
	std_bool rx = ((reg(sd, R_DCTRL) & DCTRL_DMAEN) != 0U) && rx_waiting(sd);
	std_bool tx = dma_write_mode(sd);

	if (sd->resources == NULL || sd->resources->dma_request == NULL) {
		return;
	}
	if (rx != sd->rx_request) {
		sd->rx_request = rx;
		(void)sd->resources->dma_request(sd->context, "rx", rx);
	}
	if (tx != sd->tx_request) {
		sd->tx_request = tx;
		(void)sd->resources->dma_request(sd->context, "tx", tx);
	}
}

static void update_interrupt(SdmmcType *sd)
{
	uint32 level = status(sd) & reg(sd, R_MASK);
	uint32 rose = level & ~sd->asserted;

	sd->asserted = level;
	if ((level != 0U) && ((rose != 0U) || !stm32_sdmmc_irq_active(&sd->host))) {
		stm32_sdmmc_raise(&sd->host);
	}
}

/* CPU clocks for len data bytes at the card clock and bus width of CLKCR. */
static uint64 transfer_clocks(const SdmmcType *sd, uint32 len)
{
	uint32 clkcr = reg(sd, R_CLKCR);
	uint32 widths[4] = { 1U, 4U, 8U, 1U };
	uint64 card_hz = ((clkcr & CLKCR_BYPASS) != 0U) ? KERNEL_HZ : (KERNEL_HZ / ((clkcr & 0xFFU) + 2U));
	uint64 cycles = ((uint64)len * 8U / widths[(clkcr >> CLKCR_WIDBUS_SHIFT) & 3U]) + COMMAND_BITS;

	return ((cycles * sd->cpu_hz) / card_hz) + 1U;
}

/* The data path is ready for the pending data command: the transfer runs
 * until done_at. A write waits for its DLEN bytes, from the CPU or the DMA. */
static void try_start(SdmmcType *sd)
{
	uint32 dctrl = reg(sd, R_DCTRL);
	std_bool read = (sd->pending == ATHRILL_SD_DATA_READ);

	if ((sd->pending == ATHRILL_SD_DATA_NONE) || (sd->done_at != 0U) || ((dctrl & DCTRL_DTEN) == 0U)
		|| (((dctrl & DCTRL_DTDIR) != 0U) != read)) {
		return;
	}
	if (!read && (sd->fifo_len < reg(sd, R_DLEN))) {
		return;		/* the firmware or its DMA stream still fills the FIFO */
	}
	sd->done_at = sd->now + transfer_clocks(sd, reg(sd, R_DLEN));
}

static void finish(SdmmcType *sd)
{
	uint32 len = reg(sd, R_DLEN);
	uint8 *data;

	if (sd->pending == ATHRILL_SD_DATA_READ) {
		/* the received bytes in the FIFO: the CPU or the DMA ("rx") takes them */
		data = (uint8 *)calloc(len + 4U, 1U);
		(void)athrill_sd_read(sd->protocol, data, len);
		drop_fifo(sd);
		sd->fifo = data;
		sd->fifo_len = len;
	}
	else {
		(void)athrill_sd_write(sd->protocol, sd->fifo, len);
		drop_fifo(sd);
	}
	(void)athrill_sd_transfer_done(sd->protocol);
	sd->pending = ATHRILL_SD_DATA_NONE;
	sd->done_at = 0U;
	set_reg(sd, R_DCOUNT, 0U);
	sd->sta |= STA_DATAEND | STA_DBCKEND;
	update_requests(sd);
	update_interrupt(sd);
}

static void advance_to(SdmmcType *sd, uint64 now)
{
	if (now > sd->now) {
		sd->now = now;
	}
	if ((sd->done_at != 0U) && (sd->now >= sd->done_at)) {
		finish(sd);
	}
}

static void command(SdmmcType *sd, uint32 cmd)
{
	AthrillSdResponse response;
	uint32 wait = (cmd >> CMD_WAITRESP_SHIFT) & 3U;
	uint32 i;

	memset(&response, 0, sizeof(response));
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
	if (response.data != ATHRILL_SD_DATA_NONE) {
		drop_fifo(sd);
		sd->pending = response.data;
		set_reg(sd, R_DCOUNT, reg(sd, R_DLEN));
		try_start(sd);
	}
	update_requests(sd);
	update_interrupt(sd);
}

static uint32 f7_read(SdmmcType *sd, uint32 offset)
{
	uint32 value = 0U;
	uint32 i;

	advance_to(sd, stm32_sdmmc_clock(&sd->host));
	switch (offset) {
	case R_STA:
		return status(sd);
	case R_ICR:
		return 0U;
	case R_FIFOCNT:
		return (sd->fifo == NULL) ? 0U : ((sd->fifo_len - sd->fifo_pos + 3U) / 4U);
	default:
		break;
	}
	if ((offset >= R_FIFO) && rx_waiting(sd)) {
		for (i = 0U; (i < 4U) && (sd->fifo_pos < sd->fifo_len); i++) {
			value |= (uint32)sd->fifo[sd->fifo_pos++] << (8U * i);
		}
		if (sd->fifo_pos >= sd->fifo_len) {
			drop_fifo(sd);
		}
		update_requests(sd);
		update_interrupt(sd);
		return value;
	}
	return (offset < (REGS * 4U)) ? reg(sd, offset) : 0U;
}

static void f7_write(SdmmcType *sd, uint32 offset, uint32 value, uint32 mask)
{
	uint32 i;

	advance_to(sd, stm32_sdmmc_clock(&sd->host));
	if (offset >= R_FIFO) {
		if (fifo_write_mode(sd) || dma_write_mode(sd)) {
			if (sd->fifo == NULL) {
				sd->fifo = (uint8 *)calloc(reg(sd, R_DLEN) + 4U, 1U);
				sd->fifo_len = 0U;
			}
			for (i = 0U; (i < 4U) && (sd->fifo_len < reg(sd, R_DLEN)); i++) {
				sd->fifo[sd->fifo_len++] = (uint8)(value >> (8U * i));
			}
			try_start(sd);
			update_requests(sd);
			update_interrupt(sd);
		}
		return;
	}
	switch (offset) {
	case R_ICR:
		sd->sta &= ~(value & ICR_MASK);
		update_interrupt(sd);
		return;
	case R_STA:
	case R_RESPCMD:
	case R_FIFOCNT:
	case R_DCOUNT:
		return;
	default:
		break;
	}
	if (offset < (REGS * 4U)) {
		set_reg(sd, offset, (reg(sd, offset) & ~mask) | value);
	}
	if ((offset == R_CMD) && ((reg(sd, R_CMD) & CMD_CPSMEN) != 0U)) {
		command(sd, reg(sd, R_CMD));
	}
	else if (offset == R_DCTRL) {
		if ((reg(sd, R_DCTRL) & DCTRL_DTEN) == 0U) {
			/* data path disabled: a scheduled transfer is dropped (abort), the received FIFO kept */
			if (sd->done_at != 0U) {
				sd->done_at = 0U;
				sd->pending = ATHRILL_SD_DATA_NONE;
			}
		}
		else {
			try_start(sd);
		}
		update_requests(sd);
		update_interrupt(sd);
	}
	else if (offset == R_MASK) {
		update_interrupt(sd);
	}
}

static void f7_reset(SdmmcType *sd, const AthrillSdCardOps *card, void *card_instance)
{
	Stm32SdmmcHost host = sd->host;
	const AthrillDeviceResourceOps *resources = sd->resources;
	void *context = sd->context;

	drop_fifo(sd);
	athrill_sd_protocol_destroy(sd->protocol);
	memset(sd, 0, sizeof(*sd));
	sd->host = host;
	sd->resources = resources;
	sd->context = context;
	sd->cpu_hz = host.target_ops->time_frequency_hz(host.target);
	if (card != NULL) {
		(void)athrill_sd_protocol_create(card, card_instance, &sd->protocol);
	}
}

static void f7_tick(SdmmcType *sd, uint64 now, AthrillDeviceTickResult *result)
{
	uint64 until;

	advance_to(sd, now);
	/* the interrupt is a level: pending again while an enabled flag stays set after the handler */
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

/* ---- the st,stm32-sdmmc model: the F7 or the H7 half ---- */

typedef struct {
	uint32 base;
	std_bool h7;
	SdmmcType f7;
	H7Sdmmc *h7_half;
	AthrillDeviceGpioLine d0;
	std_bool has_d0;
} Sdmmc;

static Stm32SdmmcHost *host_of(Sdmmc *m)
{
	return m->h7 ? stm32h7_sdmmc_host(m->h7_half) : &m->f7.host;
}

static sint32 sdmmc_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	uint32 size;

	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != SDMMC_BLOCK) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "SDMMC register window must be 0x%x bytes",
				SDMMC_BLOCK);
		}
		return -1;
	}
	return 0;
}

static sint32 sdmmc_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Sdmmc *m = calloc(1U, sizeof(*m));
	const void *marker;
	uint32 size;
	Stm32SdmmcHost host;

	if (m == NULL || args->target_ops == NULL || args->target_ops->time_now == NULL ||
	    args->target_ops->time_frequency_hz == NULL || args->target_ops->irq_set == NULL ||
	    args->target_ops->irq_is_active == NULL || args->target_ops->end_batch == NULL ||
	    args->target_ops->memory_read == NULL || args->target_ops->memory_write == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &m->base) != 0) {
		free(m);
		return -1;
	}
	memset(&host, 0, sizeof(host));
	host.target_ops = args->target_ops;
	host.target = args->target;
	m->h7 = args->node->ops->get_property(args->node->context, "athrill,stm32h7-sdmmc",
		&marker, &size) == 0;
	if (m->h7) {
		m->h7_half = stm32h7_sdmmc_create(&host);
		if (m->h7_half == NULL) {
			free(m);
			return -1;
		}
	}
	else {
		m->f7.host = host;
	}
	*instance = m;
	return 0;
}

static void sdmmc_destroy(void *instance)
{
	Sdmmc *m = instance;

	if (m->h7) {
		stm32h7_sdmmc_destroy(m->h7_half);
	}
	else {
		drop_fifo(&m->f7);
		athrill_sd_protocol_destroy(m->f7.protocol);
	}
	free(m);
}

static sint32 sdmmc_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Sdmmc *m = instance;

	if (args->resources->get_interrupt(args->context, NULL, 0U, &host_of(m)->irq) != 0) {
		return -1;
	}
	m->f7.resources = args->resources;
	m->f7.context = args->context;
	m->has_d0 = args->resources->get_gpio(args->context, "athrill,d0-gpios", 0U, &m->d0) == 0;
	return 0;
}

static sint32 sdmmc_reset(void *instance)
{
	Sdmmc *m = instance;
	const void *card = NULL;
	void *card_instance = NULL;
	const Stm32SdmmcHost *host = host_of(m);

	/* the card: the node's child (it connected after this controller) */
	if (m->f7.resources->get_sd_card(m->f7.context, &card, &card_instance) != 0) {
		card = NULL;
		card_instance = NULL;
	}
	if (m->h7) {
		stm32h7_sdmmc_reset(m->h7_half, card, card_instance);
	}
	else {
		f7_reset(&m->f7, card, card_instance);
	}
	/* D0 high: the board's pull-up, the card not busy */
	if (m->has_d0) {
		(void)m->f7.resources->gpio_drive(m->f7.context, m->d0, TRUE);
	}
	return 0;
}

static sint32 sdmmc_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	Sdmmc *m = instance;
	uint32 offset;
	uint32 value;

	if (address < m->base || address >= (uint64)m->base + SDMMC_BLOCK ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	/* STA, FIFO: the next instruction must see the effect */
	host_of(m)->target_ops->end_batch(host_of(m)->target);
	offset = ((uint32)address - m->base) & ~3U;
	value = m->h7 ? stm32h7_sdmmc_read(m->h7_half, offset) : f7_read(&m->f7, offset);
	*result = value >> (8U * (((uint32)address - m->base) & 3U));
	return 0;
}

static sint32 sdmmc_write(void *instance, uint64 address, uint32 size, uint64 input)
{
	Sdmmc *m = instance;
	uint32 offset, shift, mask, value;

	if (address < m->base || address >= (uint64)m->base + SDMMC_BLOCK ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	host_of(m)->target_ops->end_batch(host_of(m)->target);
	offset = ((uint32)address - m->base) & ~3U;
	shift = 8U * (((uint32)address - m->base) & 3U);
	mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	if (m->h7) {
		stm32h7_sdmmc_write(m->h7_half, offset, value, mask);
	}
	else {
		f7_write(&m->f7, offset, value, mask);
	}
	return 0;
}

static sint32 sdmmc_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	Sdmmc *m = instance;

	if (m->h7) {
		stm32h7_sdmmc_tick(m->h7_half, now, result);
	}
	else {
		f7_tick(&m->f7, now, result);
	}
	return 0;
}

static const AthrillDeviceMmioOps sdmmc_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), sdmmc_read, sdmmc_write
};

const AthrillDeviceModelOps athrill_device_stm32_sdmmc_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-sdmmc", "1", sdmmc_validate,
	sdmmc_create, sdmmc_destroy, &sdmmc_mmio, NULL, NULL, NULL,
	sdmmc_connect, sdmmc_reset, sdmmc_tick
};
