/*
 * TO BE REMOVED (hakoniwalab/athrill-device#13 step 4-5): temporary bridges
 * between the STM32 USART/UART models (athrill-device models) and what is
 * still the target's.
 *
 * - The DMA (targets/stm32/peripheral/stm32_dma.c, built into the target)
 *   registers its peripheral-side calls here, and the UART models call them
 *   for TDR (memory to UART) and RDR (UART to memory) streams, as the target's
 *   UART called the DMA before. In step 4-5 the DMA becomes a model driven by
 *   the controllers' request lines, and this goes.
 * - The target's debugger console input (Athrill's debugger serial) reaches
 *   the console UART (DEVICE_CONFIG_STM32_CONSOLE_UART) through
 *   athrill_device_stm32_uart_console_input.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_STM32_BRIDGE_H
#define ATHRILL_DEVICE_STM32_BRIDGE_H

#include "athrill_device_if.h"

/* tx: the next datum of a stream whose peripheral address is address (TRUE
 * when one was given); rx: a datum for a stream reading address (TRUE when
 * a stream took it). */
typedef std_bool (*AthrillDeviceStm32DmaTx)(uint32 address, uint32 *data);
typedef std_bool (*AthrillDeviceStm32DmaRx)(uint32 address, uint32 data);
void athrill_device_stm32_dma_bridge(AthrillDeviceStm32DmaTx tx, AthrillDeviceStm32DmaRx rx);

/* A character typed on the debugger console; FALSE when there is no console
 * UART or its input queue is full. */
std_bool athrill_device_stm32_uart_console_input(uint8 data);

#endif
