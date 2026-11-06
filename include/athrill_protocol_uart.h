/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_PROTOCOL_UART_H
#define ATHRILL_PROTOCOL_UART_H

#include "athrill_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillUartProtocol AthrillUartProtocol;

typedef struct {
	uint64 clock_hz;
	uint32 baud;
	uint8 data_bits;
	uint8 stop_bits;
	std_bool parity;
} AthrillUartTiming;

typedef sint32 (*AthrillUartReceive)(void *context, uint8 data,
	uint64 completion_time);

sint32 athrill_uart_protocol_create(AthrillChannel *channel,
	const AthrillUartTiming *timing, AthrillUartReceive child_receive,
	void *child_context, AthrillUartReceive owner_receive,
	void *owner_context, AthrillUartProtocol **protocol);
void athrill_uart_protocol_destroy(AthrillUartProtocol *protocol);

sint32 athrill_uart_open(AthrillUartProtocol *protocol, uint64 time);
sint32 athrill_uart_close(AthrillUartProtocol *protocol, uint64 time);
sint32 athrill_uart_send_to_child(AthrillUartProtocol *protocol, uint8 data,
	uint64 requested_time, uint64 *completion_time);
sint32 athrill_uart_send_from_child(AthrillUartProtocol *protocol, uint8 data,
	uint64 requested_time, uint64 *completion_time);
uint64 athrill_uart_frame_ticks(const AthrillUartProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif
