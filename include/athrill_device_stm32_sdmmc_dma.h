/*
 * TO BE REMOVED (hakoniwalab/athrill-device#13 step 4-6): a temporary bridge
 * for the target's F7 SDMMC model (targets/stm32/peripheral/stm32_sdmmc.c),
 * which controls its DMA transfers itself (peripheral flow control) and
 * hands the DMA model (src/models/stm32_dma.c) whole blocks. When the SDMMC
 * becomes an athrill-device model driven by request lines, this goes.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_STM32_SDMMC_DMA_H
#define ATHRILL_DEVICE_STM32_SDMMC_DMA_H

#include "athrill_device_if.h"

/* An enabled stream reading (to_memory) or writing peripheral register par. */
std_bool athrill_device_stm32_dma_ready(uint32 par, std_bool to_memory);
/* len bytes to the memory of the enabled stream reading par (rx), or from
 * the memory of the one writing par (tx); FALSE: no such stream enabled. */
std_bool athrill_device_stm32_dma_block_rx(uint32 par, const uint8 *data, uint32 len);
std_bool athrill_device_stm32_dma_block_tx(uint32 par, uint8 *data, uint32 len);

#endif
