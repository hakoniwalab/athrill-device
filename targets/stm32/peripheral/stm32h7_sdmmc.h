/*
 * The STM32H7 SDMMC2 v2 controller (RM0433 chapter 55) with its internal
 * DMA and a shared SD card backend (memory/sdcard.c).
 */
#ifndef ATHRILL_STM32H7_SDMMC_H
#define ATHRILL_STM32H7_SDMMC_H

#include "std_types.h"
#include "cpu.h"
#include "std_device_ops.h"
#include "memory/sdcard.h"

std_bool stm32h7_sdmmc_owns(uint32 addr);
uint32 stm32h7_sdmmc_read(uint32 addr);
void stm32h7_sdmmc_write(uint32 addr, uint32 size, uint32 value);
/* cpu_hz: CPU clock for transfer timing; no card until attach. */
void stm32h7_sdmmc_reset(uint64 cpu_hz);
void stm32h7_sdmmc_attach(SdcardType *card);
/* Completes transfers up to the device clock and raises the level interrupt. */
void stm32h7_sdmmc_tick(DeviceClockType *clock);

#endif /* ATHRILL_STM32H7_SDMMC_H */
