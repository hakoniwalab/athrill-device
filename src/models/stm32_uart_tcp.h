/*
 * A UART line over TCP (DEVICE_CONFIG_STM32_UART<n>_TCP_PORT): Athrill listens
 * on 127.0.0.1, accepts one client at a time; what the client sends arrives
 * at the UART, what the UART sends goes to the client. Ported unchanged from
 * athrill-target-stm32's stm32_uart_tcp.c; the listener of a UART number is
 * opened once per process.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_STM32_UART_TCP_H
#define ATHRILL_DEVICE_STM32_UART_TCP_H

#include "athrill_device_if.h"

typedef struct UartTcp UartTcp;

/* The link of UART number (1 to 8) listening on port; NULL when it cannot listen. */
UartTcp *stm32_uart_tcp_open(uint32 number, uint32 port);
/* A byte the UART sent, to the client. */
void stm32_uart_tcp_transmit(UartTcp *link, uint8 data);
/* The client's bytes into the UART (send returns nonzero when its queue is
 * full: the rest waits), then what the UART sent out to the client. */
void stm32_uart_tcp_poll(UartTcp *link, sint32 (*send)(void *context, uint8 data), void *context);

#endif
