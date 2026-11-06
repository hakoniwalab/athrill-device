/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_uart.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint8 data[4];
	uint64 time[4];
	uint32 count;
} Receiver;

static sint32 receive(void *context, uint8 data, uint64 completion_time)
{
	Receiver *receiver = context;
	TEST_CHECK(receiver->count < sizeof(receiver->data));
	receiver->data[receiver->count] = data;
	receiver->time[receiver->count] = completion_time;
	receiver->count++;
	return 0;
}

static AthrillChannel *make_channel(void)
{
	AthrillChannelConfig config;
	AthrillChannel *channel;
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &channel) == 0);
	return channel;
}

int main(void)
{
	AthrillUartTiming timing = {1000000U, 100000U, 8U, 1U, FALSE};
	AthrillUartTiming fractional = {8000000U, 115200U, 8U, 1U, TRUE};
	AthrillChannelStatus status;
	AthrillChannel *channel = make_channel();
	AthrillChannel *fractional_channel = make_channel();
	AthrillUartProtocol *uart;
	AthrillUartProtocol *fractional_uart;
	Receiver child = {0};
	Receiver owner = {0};
	uint64 completion;
	TEST_CHECK(athrill_uart_protocol_create(channel, &timing, receive, &child,
		receive, &owner, &uart) == 0);
	TEST_CHECK(athrill_uart_frame_ticks(uart) == 100U);
	TEST_CHECK(athrill_uart_send_to_child(uart, 0U, 0U, &completion) != 0);
	TEST_CHECK(athrill_uart_open(uart, 0U) == 0);

	TEST_CHECK(athrill_uart_send_to_child(uart, 0x11U, 10U, &completion) == 0 &&
		completion == 110U);
	TEST_CHECK(athrill_uart_send_from_child(uart, 0xa1U, 15U, &completion) == 0 &&
		completion == 115U);
	TEST_CHECK(athrill_uart_send_to_child(uart, 0x22U, 20U, &completion) == 0 &&
		completion == 210U);
	TEST_CHECK(athrill_uart_send_from_child(uart, 0xa2U, 25U, &completion) == 0 &&
		completion == 215U);
	TEST_CHECK(child.count == 2U && child.data[0] == 0x11U &&
		child.data[1] == 0x22U && child.time[0] == 110U &&
		child.time[1] == 210U);
	TEST_CHECK(owner.count == 2U && owner.data[0] == 0xa1U &&
		owner.data[1] == 0xa2U && owner.time[0] == 115U &&
		owner.time[1] == 215U);
	athrill_channel_status(channel, &status);
	TEST_CHECK(status.completed == 4U && status.completion_time == 215U);
	TEST_CHECK(athrill_uart_close(uart, 220U) == 0);
	TEST_CHECK(athrill_uart_send_from_child(uart, 0U, 221U, &completion) != 0);

	TEST_CHECK(athrill_uart_protocol_create(fractional_channel, &fractional,
		receive, &child, receive, &owner, &fractional_uart) == 0);
	TEST_CHECK(athrill_uart_frame_ticks(fractional_uart) == 764U);

	athrill_uart_protocol_destroy(fractional_uart);
	athrill_channel_destroy(fractional_channel);
	athrill_uart_protocol_destroy(uart);
	athrill_channel_destroy(channel);
	puts("UART protocol tests passed");
	return 0;
}
