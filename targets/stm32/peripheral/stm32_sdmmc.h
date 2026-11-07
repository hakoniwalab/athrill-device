/*
 * The STM32F7 SDMMC1 controller (RM0410 chapter 35) with a memory card on
 * it (sdcard.c).
 *
 * Commands: writing CMD with CPSMEN sends the command to the card at once;
 * its response (or a timeout) is in RESPCMD/RESP1-4 and STA (CMDSENT,
 * CMDREND, CTIMEOUT) when the firmware polls. Data: the data path
 * (DCTRL, DLEN) moves the bytes of a read or write command by DMA (the
 * stream reading or writing FIFO, DMA2 stream 3 or 6, peripheral flow
 * control) or through the FIFO register, and ends with DATAEND and
 * DBCKEND after the time the bytes take on the bus at the configured clock
 * and bus width, so the driver waits for it as on the board. The
 * interrupt is the level of STA & MASK.
 * Lightweight: no CRC errors or data timeouts, no SDIO mode, no
 * read wait, no power-up delay.
 */
#ifndef ATHRILL_STM32_SDMMC_H
#define ATHRILL_STM32_SDMMC_H

#include "std_types.h"
#include "cpu.h"
#include "std_device_ops.h"
#include "memory/sdcard.h"

std_bool stm32_sdmmc_owns(uint32 addr);
uint32 stm32_sdmmc_read(uint32 addr);
void stm32_sdmmc_write(uint32 addr, uint32 size, uint32 value);
/* cpu_hz: the CPU clock (transfer times); no card until stm32_sdmmc_attach. */
void stm32_sdmmc_reset(uint64 cpu_hz);
void stm32_sdmmc_attach(SdcardType *card);
/* Transfers up to the device clock, the interrupt, the clocks to the next event. */
void stm32_sdmmc_tick(DeviceClockType *clock);

#endif
