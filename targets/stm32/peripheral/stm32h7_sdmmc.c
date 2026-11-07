/*
 * STM32H743 SDMMC2 v2 and internal DMA (see stm32h7_sdmmc.h). Register
 * offsets and bits are from RM0433 55.9. The card protocol and image are
 * the same memory/sdcard.c backend used by the F7 SDMMC1 front end.
 */
#include "stm32h7_sdmmc.h"
#include "athrill_protocol_sd.h"
#include "athrill_device_host.h"
#include <stdlib.h>
#include <string.h>

#define SDMMC_BASE	0x48022400U
#define SDMMC_BLOCK	0x400U
#define SDMMC_IRQ	124U

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
#define CMD_WAITRESP_SHIFT	8U
#define CMD_CPSMEN		(1U << 12)

#define DCTRL_DTDIR	(1U << 1)	/* card to controller */
#define IDMACTRL_IDMAEN	(1U << 0)

#define STA_CTIMEOUT	(1U << 2)
#define STA_CMDREND	(1U << 6)
#define STA_CMDSENT	(1U << 7)
#define STA_DATAEND	(1U << 8)
#define STA_DBCKEND	(1U << 10)
#define STA_DPSMACT	(1U << 12)
#define ICR_MASK	0x1FE00FFFU

#define KERNEL_HZ	240000000U	/* fmu-v6c PLL1Q, PX4 board clock profile */
#define COMMAND_BITS	100U

typedef struct {
	uint32 regs[REGS];
	uint32 sta;
	uint64 cpu_hz;
	uint64 now;
	AthrillSdProtocol *protocol;
	AthrillSdDataDirection pending;
	uint64 done_at;
	uint32 asserted;
} H7SdmmcType;

static H7SdmmcType sd;

static sint32 card_command(void *context, uint32 index, uint32 argument,
	AthrillSdResponse *response)
{
	SdcardResponse backend;
	uint32 i;

	sdcard_command(context, index, argument, &backend);
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

std_bool stm32h7_sdmmc_owns(uint32 addr)
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

static uint32 status(void)
{
	return sd.sta | ((sd.done_at != 0U) ? STA_DPSMACT : 0U);
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

/* CPU clocks for len bytes at SDMMC_CK and the configured bus width. */
static uint64 transfer_clocks(uint32 len)
{
	uint32 clkcr = reg(R_CLKCR);
	uint32 widths[4] = { 1U, 4U, 8U, 1U };
	uint32 div = clkcr & 0x3FFU;
	uint64 card_hz = (div == 0U) ? KERNEL_HZ : (KERNEL_HZ / (2U * div));
	uint64 cycles = ((uint64)len * 8U / widths[(clkcr >> CLKCR_WIDBUS_SHIFT) & 3U]) + COMMAND_BITS;

	return ((cycles * sd.cpu_hz) / card_hz) + 1U;
}

static void try_start(void)
{
	std_bool read = sd.pending == ATHRILL_SD_DATA_READ;

	if ((sd.pending == ATHRILL_SD_DATA_NONE) || (sd.done_at != 0U)
		|| ((reg(R_IDMACTRL) & IDMACTRL_IDMAEN) == 0U) || (reg(R_IDMABASE0) == 0U)
		|| (((reg(R_DCTRL) & DCTRL_DTDIR) != 0U) != read) || (reg(R_DLEN) == 0U)) {
		return;
	}
	sd.done_at = sd.now + transfer_clocks(reg(R_DLEN));
	update_interrupt();
}

static void memory_read(uint32 addr, uint8 *data, uint32 len)
{
	(void)athrill_device_host()->memory_read(NULL, addr, data, len);
}

static void memory_write(uint32 addr, const uint8 *data, uint32 len)
{
	(void)athrill_device_host()->memory_write(NULL, addr, data, len);
}

static void finish(void)
{
	uint32 len = reg(R_DLEN);
	uint32 address = reg(R_IDMABASE0);
	uint8 *data = (uint8 *)calloc(len, 1U);

	if (sd.pending == ATHRILL_SD_DATA_READ) {
		(void)athrill_sd_read(sd.protocol, data, len);
		memory_write(address, data, len);
	}
	else {
		memory_read(address, data, len);
		(void)athrill_sd_write(sd.protocol, data, len);
	}
	free(data);
	(void)athrill_sd_transfer_done(sd.protocol);
	sd.pending = ATHRILL_SD_DATA_NONE;
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
	try_start();
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
	if ((response.data != ATHRILL_SD_DATA_NONE) && ((cmd & CMD_CMDTRANS) != 0U)) {
		sd.pending = response.data;
		set_reg(R_DCOUNT, reg(R_DLEN));
		try_start();
	}
	update_interrupt();
}

uint32 stm32h7_sdmmc_read(uint32 addr)
{
	uint32 offset = (addr - SDMMC_BASE) & ~3U;

	advance_to(athrill_device_host_clock());
	if (offset == R_STA) {
		return status();
	}
	if (offset == R_ICR) {
		return 0U;
	}
	return (offset < (REGS * 4U)) ? reg(offset) : 0U;
}

void stm32h7_sdmmc_write(uint32 addr, uint32 size, uint32 value)
{
	uint32 offset = (addr - SDMMC_BASE) & ~3U;
	uint32 shift = 8U * ((addr - SDMMC_BASE) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	advance_to(athrill_device_host_clock());
	value = (value << shift) & mask;
	if (offset == R_ICR) {
		sd.sta &= ~(value & ICR_MASK);
		update_interrupt();
		return;
	}
	if ((offset == R_STA) || (offset == R_RESPCMD) || (offset == R_DCOUNT)) {
		return;
	}
	if (offset < (REGS * 4U)) {
		set_reg(offset, (reg(offset) & ~mask) | value);
	}
	if ((offset == R_CMD) && ((reg(R_CMD) & CMD_CPSMEN) != 0U)) {
		command(reg(R_CMD));
	}
	else if ((offset == R_DCTRL) || (offset == R_DLEN) || (offset == R_IDMACTRL)
		|| (offset == R_IDMABASE0)) {
		if ((reg(R_IDMACTRL) & IDMACTRL_IDMAEN) == 0U) {
			sd.done_at = 0U;
		}
		try_start();
	}
	else if (offset == R_MASK) {
		update_interrupt();
	}
}

void stm32h7_sdmmc_reset(uint64 cpu_hz)
{
	athrill_sd_protocol_destroy(sd.protocol);
	memset(&sd, 0, sizeof(sd));
	sd.cpu_hz = cpu_hz;
}

void stm32h7_sdmmc_attach(SdcardType *card)
{
	athrill_sd_protocol_destroy(sd.protocol);
	sd.protocol = NULL;
	if (card != NULL) {
		(void)athrill_sd_protocol_create(&card_ops, card, &sd.protocol);
	}
}

void stm32h7_sdmmc_tick(DeviceClockType *clock)
{
	uint64 until;

	advance_to(clock->clock);
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
