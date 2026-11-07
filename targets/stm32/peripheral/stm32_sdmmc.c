/*
 * The SDMMC1 controller (see stm32_sdmmc.h). Offsets and bits from
 * RM0410 35.8.
 */
#include "stm32_sdmmc.h"
#include "stm32_dma.h"
#include "athrill_protocol_sd.h"
#include "athrill_device_host.h"
#include <stdlib.h>
#include <string.h>

#define SDMMC_BASE	0x40012C00U
#define SDMMC_BLOCK	0x400U
#define SDMMC_IRQ	49U

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
	uint32 regs[REGS];
	uint32 sta;		/* the static flags (the FIFO flags are worked out) */
	uint64 cpu_hz;
	uint64 now;
	AthrillSdProtocol *protocol;
	SdcardDataType pending;	/* a data command waits for its data path */
	uint64 done_at;		/* the clock the scheduled transfer ends; 0: none */
	uint8 *fifo;		/* the bytes of a transfer through the FIFO register */
	uint32 fifo_len;
	uint32 fifo_pos;
	uint32 asserted;
} SdmmcType;

static SdmmcType sd;

static sint32 card_command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	SdcardResponse backend;
	SdcardType *card = context;
	uint32 i;

	sdcard_command(card, index, argument, &backend);
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
	sdcard_read(context, data, length);
	return 0;
}

static sint32 card_write(void *context, const uint8 *data, uint32 length)
{
	sdcard_write(context, data, length);
	return 0;
}

static sint32 card_transfer_done(void *context)
{
	sdcard_transfer_done(context);
	return 0;
}

static const AthrillSdCardOps card_ops = {
	.command = card_command,
	.read = card_read,
	.write = card_write,
	.transfer_done = card_transfer_done,
};

std_bool stm32_sdmmc_owns(uint32 addr)
{
	return (addr >= SDMMC_BASE) && (addr < (SDMMC_BASE + SDMMC_BLOCK));
}

static uint32 reg(uint32 offset)
{
	return sd.regs[offset / 4U];
}

static void set_reg(uint32 offset, uint32 value)
{
	sd.regs[offset / 4U] = value;
}

static void drop_fifo(void)
{
	free(sd.fifo);
	sd.fifo = NULL;
	sd.fifo_len = 0U;
	sd.fifo_pos = 0U;
}

static std_bool fifo_write_mode(void)
{
	uint32 dctrl = reg(R_DCTRL);

	return (sd.pending == SDCARD_DATA_WRITE) && (sd.done_at == 0U)
		&& ((dctrl & (DCTRL_DTEN | DCTRL_DTDIR | DCTRL_DMAEN)) == DCTRL_DTEN);
}

static uint32 status(void)
{
	uint32 sta = sd.sta;
	uint32 words;

	if ((sd.fifo != NULL) && (sd.pending == SDCARD_DATA_NONE)) {
		/* received bytes waiting in the FIFO */
		words = (sd.fifo_len - sd.fifo_pos + 3U) / 4U;
		sta |= (words != 0U) ? STA_RXDAVL : STA_RXFIFOE;
		sta |= (words >= 8U) ? STA_RXFIFOHF : 0U;
	}
	else if (fifo_write_mode()) {
		sta |= STA_TXFIFOHE | STA_TXFIFOE;
	}
	return sta;
}

static void update_interrupt(void)
{
	uint32 level = status() & reg(R_MASK);
	uint32 rose = level & ~sd.asserted;

	sd.asserted = level;
	if ((level != 0U) && ((rose != 0U) || !athrill_device_host_irq_pending(SDMMC_IRQ))) {
		athrill_device_host_raise(SDMMC_IRQ);
	}
}

/* CPU clocks for len data bytes at the card clock and bus width of CLKCR. */
static uint64 transfer_clocks(uint32 len)
{
	uint32 clkcr = reg(R_CLKCR);
	uint32 widths[4] = { 1U, 4U, 8U, 1U };
	uint64 card_hz = ((clkcr & CLKCR_BYPASS) != 0U) ? KERNEL_HZ : (KERNEL_HZ / ((clkcr & 0xFFU) + 2U));
	uint64 cycles = ((uint64)len * 8U / widths[(clkcr >> CLKCR_WIDBUS_SHIFT) & 3U]) + COMMAND_BITS;

	return ((cycles * sd.cpu_hz) / card_hz) + 1U;
}

/* The data path is ready for the pending data command: the transfer runs until done_at. */
static void try_start(void)
{
	uint32 dctrl = reg(R_DCTRL);
	std_bool read = (sd.pending == SDCARD_DATA_READ);

	if ((sd.pending == SDCARD_DATA_NONE) || (sd.done_at != 0U) || ((dctrl & DCTRL_DTEN) == 0U)
		|| (((dctrl & DCTRL_DTDIR) != 0U) != read)) {
		return;
	}
	if ((dctrl & DCTRL_DMAEN) != 0U) {
		if (!stm32_dma_peripheral_ready(SDMMC_BASE + R_FIFO, read)) {
			return;		/* the stream is enabled after DMAEN */
		}
	}
	else if (!read && (sd.fifo_len < reg(R_DLEN))) {
		return;		/* the firmware still fills the FIFO */
	}
	sd.done_at = sd.now + transfer_clocks(reg(R_DLEN));
}

static void finish(void)
{
	uint32 len = reg(R_DLEN);
	uint32 dctrl = reg(R_DCTRL);
	uint8 *data;

	if (sd.pending == SDCARD_DATA_READ) {
		data = (uint8 *)calloc(len + 4U, 1U);
		(void)athrill_sd_read(sd.protocol, data, len);
		if ((dctrl & DCTRL_DMAEN) != 0U) {
			(void)stm32_dma_peripheral_block_rx(SDMMC_BASE + R_FIFO, data, len);
			free(data);
		}
		else {
			drop_fifo();
			sd.fifo = data;
			sd.fifo_len = len;
		}
	}
	else if ((dctrl & DCTRL_DMAEN) != 0U) {
		data = (uint8 *)calloc(len + 4U, 1U);
		(void)stm32_dma_peripheral_block_tx(SDMMC_BASE + R_FIFO, data, len);
		(void)athrill_sd_write(sd.protocol, data, len);
		free(data);
	}
	else {
		(void)athrill_sd_write(sd.protocol, sd.fifo, len);
		drop_fifo();
	}
	(void)athrill_sd_transfer_done(sd.protocol);
	sd.pending = SDCARD_DATA_NONE;
	sd.done_at = 0U;
	set_reg(R_DCOUNT, 0U);
	sd.sta |= STA_DATAEND | STA_DBCKEND;
	update_interrupt();
}

static void advance_to(uint64 now)
{
	if (now > sd.now) {
		sd.now = now;
	}
	if (sd.pending == SDCARD_DATA_WRITE) {
		try_start();	/* the DMA stream enabled since */
	}
	if ((sd.done_at != 0U) && (sd.now >= sd.done_at)) {
		finish();
	}
}

static void command(uint32 cmd)
{
	AthrillSdResponse response;
	uint32 wait = (cmd >> CMD_WAITRESP_SHIFT) & 3U;
	uint32 i;

	memset(&response, 0, sizeof(response));
	if (sd.protocol != NULL) {
		(void)athrill_sd_command(sd.protocol, cmd & 0x3FU, reg(R_ARG),
			&response);
	}
	if ((wait == 0U) || (wait == 2U)) {
		sd.sta |= STA_CMDSENT;
	}
	else if (response.kind == ATHRILL_SD_RESPONSE_NONE) {
		sd.sta |= STA_CTIMEOUT;
	}
	else {
		set_reg(R_RESPCMD, response.has_index ? (cmd & 0x3FU) : 0x3FU);
		for (i = 0U; i < 4U; i++) {
			set_reg(R_RESP1 + (4U * i), response.words[i]);
		}
		sd.sta |= STA_CMDREND;
	}
	if (response.data != ATHRILL_SD_DATA_NONE) {
		drop_fifo();
		sd.pending = (SdcardDataType)response.data;
		set_reg(R_DCOUNT, reg(R_DLEN));
		try_start();
	}
	update_interrupt();
}

uint32 stm32_sdmmc_read(uint32 addr)
{
	uint32 offset = (addr - SDMMC_BASE) & ~3U;
	uint32 value = 0U;
	uint32 i;

	advance_to(athrill_device_host_clock());
	switch (offset) {
	case R_STA:
		return status();
	case R_ICR:
		return 0U;
	case R_FIFOCNT:
		return (sd.fifo == NULL) ? 0U : ((sd.fifo_len - sd.fifo_pos + 3U) / 4U);
	default:
		break;
	}
	if ((offset >= R_FIFO) && (sd.fifo != NULL) && (sd.pending == SDCARD_DATA_NONE)) {
		for (i = 0U; (i < 4U) && (sd.fifo_pos < sd.fifo_len); i++) {
			value |= (uint32)sd.fifo[sd.fifo_pos++] << (8U * i);
		}
		if (sd.fifo_pos >= sd.fifo_len) {
			drop_fifo();
		}
		update_interrupt();
		return value;
	}
	return (offset < (REGS * 4U)) ? reg(offset) : 0U;
}

void stm32_sdmmc_write(uint32 addr, uint32 size, uint32 value)
{
	uint32 offset = (addr - SDMMC_BASE) & ~3U;
	uint32 shift = 8U * ((addr - SDMMC_BASE) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	uint32 i;

	advance_to(athrill_device_host_clock());
	value = (value << shift) & mask;
	if (offset >= R_FIFO) {
		if (fifo_write_mode()) {
			if (sd.fifo == NULL) {
				sd.fifo = (uint8 *)calloc(reg(R_DLEN) + 4U, 1U);
				sd.fifo_len = 0U;
			}
			for (i = 0U; (i < 4U) && (sd.fifo_len < reg(R_DLEN)); i++) {
				sd.fifo[sd.fifo_len++] = (uint8)(value >> (8U * i));
			}
			try_start();
			update_interrupt();
		}
		return;
	}
	switch (offset) {
	case R_ICR:
		sd.sta &= ~(value & ICR_MASK);
		update_interrupt();
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
		set_reg(offset, (reg(offset) & ~mask) | value);
	}
	if ((offset == R_CMD) && ((reg(R_CMD) & CMD_CPSMEN) != 0U)) {
		command(reg(R_CMD));
	}
	else if (offset == R_DCTRL) {
		if ((reg(R_DCTRL) & DCTRL_DTEN) == 0U) {
			/* data path disabled: a scheduled transfer is dropped (abort), the received FIFO kept */
			if (sd.done_at != 0U) {
				sd.done_at = 0U;
				sd.pending = SDCARD_DATA_NONE;
			}
		}
		else {
			try_start();
		}
		update_interrupt();
	}
	else if (offset == R_MASK) {
		update_interrupt();
	}
}

void stm32_sdmmc_reset(uint64 cpu_hz)
{
	drop_fifo();
	athrill_sd_protocol_destroy(sd.protocol);
	memset(&sd, 0, sizeof(sd));
	sd.cpu_hz = cpu_hz;
}

void stm32_sdmmc_attach(SdcardType *card)
{
	athrill_sd_protocol_destroy(sd.protocol);
	sd.protocol = NULL;
	if (card != NULL) {
		(void)athrill_sd_protocol_create(&card_ops, card, &sd.protocol);
	}
}

void stm32_sdmmc_tick(DeviceClockType *clock)
{
	uint64 until;

	advance_to(clock->clock);
	/* the interrupt is a level: pending again while an enabled flag stays set after the handler */
	sd.asserted = status() & reg(R_MASK);
	if ((sd.asserted != 0U) && !athrill_device_host_irq_pending(SDMMC_IRQ)) {
		athrill_device_host_raise(SDMMC_IRQ);
	}
	if (sd.done_at != 0U) {
		until = (sd.done_at > sd.now) ? (sd.done_at - sd.now) : 1U;
		if (until < clock->min_intr_interval) {
			clock->min_intr_interval = until;
		}
	}
}
