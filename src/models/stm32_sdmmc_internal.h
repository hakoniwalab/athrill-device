/*
 * The two halves of the st,stm32-sdmmc model (stm32_sdmmc.c): the F7
 * SDMMC v1 (in stm32_sdmmc.c) and the H7 SDMMC v2 with IDMA
 * (stm32h7_sdmmc.c), and what both get from the model: the host table,
 * the interrupt line, and (F7) the DMA request lines and the D0 line.
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_STM32_SDMMC_INTERNAL_H
#define ATHRILL_STM32_SDMMC_INTERNAL_H

#include "models.h"
#include "athrill_protocol_sd.h"

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	AthrillDeviceIrqLine irq;
} Stm32SdmmcHost;

static inline uint64 stm32_sdmmc_clock(const Stm32SdmmcHost *host)
{
	return host->target_ops->time_now(host->target);
}

static inline void stm32_sdmmc_raise(const Stm32SdmmcHost *host)
{
	(void)host->target_ops->irq_set(host->target, host->irq, TRUE);
}

static inline std_bool stm32_sdmmc_irq_active(const Stm32SdmmcHost *host)
{
	return host->target_ops->irq_is_active(host->target, host->irq);
}

/* The H7 half: offsets are from the node's base, aligned to 4. */
typedef struct H7Sdmmc H7Sdmmc;
H7Sdmmc *stm32h7_sdmmc_create(const Stm32SdmmcHost *host);
void stm32h7_sdmmc_destroy(H7Sdmmc *sd);
Stm32SdmmcHost *stm32h7_sdmmc_host(H7Sdmmc *sd);
void stm32h7_sdmmc_reset(H7Sdmmc *sd, const AthrillSdCardOps *card, void *card_instance);
uint32 stm32h7_sdmmc_read(H7Sdmmc *sd, uint32 offset);
void stm32h7_sdmmc_write(H7Sdmmc *sd, uint32 offset, uint32 value, uint32 mask);
void stm32h7_sdmmc_tick(H7Sdmmc *sd, uint64 now, AthrillDeviceTickResult *result);

#endif
