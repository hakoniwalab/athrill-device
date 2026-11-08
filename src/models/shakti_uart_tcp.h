/*
 * The Shakti UART's line over TCP (shakti_uart_tcp.c).
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_SHAKTI_UART_TCP_H
#define ATHRILL_DEVICE_SHAKTI_UART_TCP_H

#include "athrill_device_if.h"

/* Listen on 127.0.0.1:port (once per process); 0 when listening. */
sint32 shakti_uart_tcp_open(uint32 port);
std_bool shakti_uart_tcp_enabled(void);
/* A byte from the client waits. */
std_bool shakti_uart_tcp_pending(void);
/* The next byte from the client, if any. */
std_bool shakti_uart_tcp_get(uint8 *data);
/* The collected transmit bytes, to the client. */
void shakti_uart_tcp_flush(void);
/* A byte the UART sent (collected until the next flush). */
void shakti_uart_tcp_put(uint8 data);

#endif
