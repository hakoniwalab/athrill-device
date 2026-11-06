/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_uart.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint8 data[8];
	uint64 time[8];
	uint32 count;
} Receiver;

typedef struct {
	AthrillChannel *channel;
	AthrillUartProtocol *protocol;
} UartFixture;

static sint32 receive(void *context, uint8 data, uint64 completion_time)
{
	Receiver *receiver = context;
	TEST_CHECK(receiver->count < sizeof(receiver->data));
	receiver->data[receiver->count] = data;
	receiver->time[receiver->count] = completion_time;
	receiver->count++;
	return 0;
}

static UartFixture create_uart(const AthrillUartTiming *timing,
	Receiver *child, Receiver *owner)
{
	AthrillChannelConfig config;
	UartFixture fixture = {0};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &fixture.channel) == 0);
	TEST_CHECK(athrill_uart_protocol_create(fixture.channel, timing, receive,
		child, receive, owner, &fixture.protocol) == 0);
	return fixture;
}

static void destroy_uart(UartFixture *fixture)
{
	athrill_uart_protocol_destroy(fixture->protocol);
	athrill_channel_destroy(fixture->channel);
}

static void test_frame_ticks_include_start_data_parity_and_stop(void)
{
	AthrillUartTiming eight_n_one = {1000000U, 100000U, 8U, 1U, FALSE};
	AthrillUartTiming seven_e_two = {1000000U, 100000U, 7U, 2U, TRUE};
	Receiver child = {0};
	Receiver owner = {0};
	UartFixture first = create_uart(&eight_n_one, &child, &owner);
	UartFixture second = create_uart(&seven_e_two, &child, &owner);
	TEST_CHECK(athrill_uart_frame_ticks(first.protocol) == 100U);
	TEST_CHECK(athrill_uart_frame_ticks(second.protocol) == 110U);
	destroy_uart(&second);
	destroy_uart(&first);
}

static void test_fractional_frame_ticks_round_up(void)
{
	AthrillUartTiming timing = {8000000U, 115200U, 8U, 1U, TRUE};
	Receiver child = {0};
	Receiver owner = {0};
	UartFixture fixture = create_uart(&timing, &child, &owner);
	TEST_CHECK(athrill_uart_frame_ticks(fixture.protocol) == 764U);
	destroy_uart(&fixture);
}

static void test_tx_fifo_preserves_order_and_frame_spacing(void)
{
	AthrillUartTiming timing = {1000000U, 100000U, 8U, 1U, FALSE};
	Receiver child = {0};
	Receiver owner = {0};
	UartFixture fixture = create_uart(&timing, &child, &owner);
	uint64 completion;
	TEST_CHECK(athrill_uart_open(fixture.protocol, 0U) == 0);
	TEST_CHECK(athrill_uart_send_to_child(fixture.protocol, 0x11U, 10U,
		&completion) == 0 && completion == 110U);
	TEST_CHECK(athrill_uart_send_to_child(fixture.protocol, 0x22U, 20U,
		&completion) == 0 && completion == 210U);
	TEST_CHECK(child.count == 2U);
	TEST_CHECK(child.data[0] == 0x11U && child.time[0] == 110U);
	TEST_CHECK(child.data[1] == 0x22U && child.time[1] == 210U);
	TEST_CHECK(owner.count == 0U);
	TEST_CHECK(athrill_uart_close(fixture.protocol, 211U) == 0);
	destroy_uart(&fixture);
}

static void test_rx_fifo_is_independent_from_tx(void)
{
	AthrillUartTiming timing = {1000000U, 100000U, 8U, 1U, FALSE};
	Receiver child = {0};
	Receiver owner = {0};
	UartFixture fixture = create_uart(&timing, &child, &owner);
	uint64 completion;
	TEST_CHECK(athrill_uart_open(fixture.protocol, 0U) == 0);
	TEST_CHECK(athrill_uart_send_to_child(fixture.protocol, 0x11U, 10U,
		&completion) == 0 && completion == 110U);
	TEST_CHECK(athrill_uart_send_from_child(fixture.protocol, 0xa1U, 15U,
		&completion) == 0 && completion == 115U);
	TEST_CHECK(athrill_uart_send_from_child(fixture.protocol, 0xa2U, 25U,
		&completion) == 0 && completion == 215U);
	TEST_CHECK(owner.count == 2U);
	TEST_CHECK(owner.data[0] == 0xa1U && owner.time[0] == 115U);
	TEST_CHECK(owner.data[1] == 0xa2U && owner.time[1] == 215U);
	TEST_CHECK(child.count == 1U && child.data[0] == 0x11U);
	TEST_CHECK(athrill_uart_close(fixture.protocol, 216U) == 0);
	destroy_uart(&fixture);
}

static void test_idle_line_starts_frame_at_requested_clock(void)
{
	AthrillUartTiming timing = {1000000U, 100000U, 8U, 1U, FALSE};
	Receiver child = {0};
	Receiver owner = {0};
	UartFixture fixture = create_uart(&timing, &child, &owner);
	AthrillChannelStatus status;
	uint64 completion;
	TEST_CHECK(athrill_uart_send_to_child(fixture.protocol, 0U, 0U,
		&completion) != 0);
	TEST_CHECK(athrill_uart_open(fixture.protocol, 50U) == 0);
	athrill_channel_status(fixture.channel, &status);
	TEST_CHECK(status.completed == 0U && status.completion_time == 0U);
	TEST_CHECK(child.count == 0U && owner.count == 0U);
	TEST_CHECK(athrill_uart_send_to_child(fixture.protocol, 0x5aU, 500U,
		&completion) == 0);
	TEST_CHECK(completion == 600U);
	TEST_CHECK(child.count == 1U && child.time[0] == 600U);
	TEST_CHECK(athrill_uart_close(fixture.protocol, 601U) == 0);
	TEST_CHECK(athrill_uart_send_from_child(fixture.protocol, 0U, 602U,
		&completion) != 0);
	destroy_uart(&fixture);
}

int main(void)
{
	test_frame_ticks_include_start_data_parity_and_stop();
	test_fractional_frame_ticks_round_up();
	test_tx_fifo_preserves_order_and_frame_spacing();
	test_rx_fifo_is_independent_from_tx();
	test_idle_line_starts_frame_at_requested_clock();
	puts("UART protocol tests passed");
	return 0;
}
