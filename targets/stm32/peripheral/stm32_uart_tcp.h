/*
 * A UART line to a TCP client (DEVICE_CONFIG_STM32_UART<n>_TCP_PORT), for
 * example QGroundControl talking MAVLink to PX4 on TELEM1.
 *
 * Athrill listens on 127.0.0.1:<port> for one client at a time; only the
 * local host can connect. The link sits on the UART as a chip does
 * (stm32_uart_attach): what the UART sends goes to the client, what the
 * client sends arrives at the UART one character per frame time.
 * Without a client, sent characters are dropped as on an unconnected line.
 * Received bytes wait in a ring a thread fills (it waits when the ring is
 * full, so no byte is lost); the device tick moves them to the UART.
 */
#ifndef ATHRILL_STM32_UART_TCP_H
#define ATHRILL_STM32_UART_TCP_H

#include "std_types.h"

/* Opens the configured links (once) and attaches them to their UARTs (after the UARTs' reset). */
void stm32_uart_tcp_attach(void);
/* Received bytes to the UARTs, sent bytes to the clients. */
void stm32_uart_tcp_tick(void);

#endif
