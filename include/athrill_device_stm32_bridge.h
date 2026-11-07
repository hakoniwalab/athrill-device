/*
 * TO BE REMOVED (hakoniwalab/athrill-device#13 step 4-8): a temporary bridge
 * between the STM32 USART/UART models (athrill-device models) and what is
 * still the target's: the target's debugger console input (Athrill's
 * debugger serial) reaches the console UART (DEVICE_CONFIG_STM32_CONSOLE_UART)
 * through athrill_device_stm32_uart_console_input.
 *
 * (The DMA bridge that was here went in step 4-5: the UARTs raise DMA
 * request lines, ABI 1.14.)
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_STM32_BRIDGE_H
#define ATHRILL_DEVICE_STM32_BRIDGE_H

#include "athrill_device_if.h"

/* A character typed on the debugger console; FALSE when there is no console
 * UART or its input queue is full. */
std_bool athrill_device_stm32_uart_console_input(uint8 data);

#endif
