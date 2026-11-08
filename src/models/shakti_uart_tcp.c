/*
 * The Shakti UART's line over TCP (DEVICE_CONFIG_ERBIUM_UART_TCP_PORT).
 * Ported unchanged from athrill-target-riscv src/device/peripheral/
 * erbium_uart_tcp.c (only the names and the Std_ReturnType results changed);
 * the listener is opened once per process, so a reset (a rebuilt runtime)
 * keeps the client.
 *
 * Athrill listens on 127.0.0.1:<port> for one client at a time, for example
 * QGroundControl talking MAVLink to PX4. Only the local host can connect.
 *
 * - Receive: a thread accepts the client and copies what it sends into a
 *   ring buffer; the UART takes one byte per character period while its RX
 *   FIFO has room (shakti_uart_tcp_get, called from the device tick). When
 *   the ring is full the thread waits, so no byte is lost.
 * - Transmit: bytes the guest writes are collected (shakti_uart_tcp_put)
 *   and sent from the device thread (shakti_uart_tcp_flush). Without a
 *   client they are dropped, as on an unconnected serial line.
 *
 * The ring has one producer (the thread) and one consumer (the device
 * tick), synchronised with C11 atomics.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "shakti_uart_tcp.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define UART_TCP_RING_SIZE 65536U	/* power of two */
#define UART_TCP_TX_SIZE 4096U

static uint8 rx_ring[UART_TCP_RING_SIZE];
static atomic_uint rx_head;	/* next byte to take (consumer) */
static atomic_uint rx_tail;	/* next byte to fill (producer) */
static uint8 tx_buffer[UART_TCP_TX_SIZE];
static uint32 tx_count;
static int listen_fd = -1;
static pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;
static int client_fd = -1;
static std_bool enabled = FALSE;

static void set_client(int fd)
{
	pthread_mutex_lock(&client_mutex);
	client_fd = fd;
	pthread_mutex_unlock(&client_mutex);
}

static void put_into_ring(const uint8 *data, uint32 size)
{
	uint32 index;
	uint32 tail;

	for (index = 0U; index < size; index++) {
		tail = atomic_load_explicit(&rx_tail, memory_order_relaxed);
		while ((tail - atomic_load_explicit(&rx_head, memory_order_acquire)) >= UART_TCP_RING_SIZE) {
			usleep(1000);
		}
		rx_ring[tail & (UART_TCP_RING_SIZE - 1U)] = data[index];
		atomic_store_explicit(&rx_tail, tail + 1U, memory_order_release);
	}
}

static void *receive_thread(void *arg)
{
	uint8 buffer[1024];
	ssize_t received;
	int fd;

	(void)arg;
	for (;;) {
		fd = accept(listen_fd, NULL, NULL);
		if (fd < 0) {
			if (errno == EINTR) {
				continue;
			}
			return NULL;
		}
		printf("UART TCP: client connected\n");
		set_client(fd);
		for (;;) {
			received = recv(fd, buffer, sizeof(buffer), 0);
			if (received <= 0) {
				break;
			}
			put_into_ring(buffer, (uint32)received);
		}
		set_client(-1);
		close(fd);
		printf("UART TCP: client disconnected\n");
	}
}

sint32 shakti_uart_tcp_open(uint32 port)
{
	struct sockaddr_in address;
	pthread_t thread;
	int reuse = 1;

	listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_fd < 0) {
		return -1;
	}
	(void)setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons((uint16)port);
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if ((bind(listen_fd, (struct sockaddr *)&address, sizeof(address)) != 0)
		|| (listen(listen_fd, 1) != 0)
		|| (pthread_create(&thread, NULL, receive_thread, NULL) != 0)) {
		printf("ERROR: UART TCP: can not listen on 127.0.0.1:%u (errno=%d)\n", port, errno);
		close(listen_fd);
		listen_fd = -1;
		return -1;
	}
	(void)pthread_detach(thread);
	enabled = TRUE;
	printf("UART TCP: listening on 127.0.0.1:%u\n", port);
	return 0;
}

std_bool shakti_uart_tcp_enabled(void)
{
	return enabled;
}

std_bool shakti_uart_tcp_pending(void)
{
	return atomic_load_explicit(&rx_head, memory_order_relaxed)
		!= atomic_load_explicit(&rx_tail, memory_order_acquire);
}

std_bool shakti_uart_tcp_get(uint8 *data)
{
	uint32 head = atomic_load_explicit(&rx_head, memory_order_relaxed);

	if (head == atomic_load_explicit(&rx_tail, memory_order_acquire)) {
		return FALSE;
	}
	*data = rx_ring[head & (UART_TCP_RING_SIZE - 1U)];
	atomic_store_explicit(&rx_head, head + 1U, memory_order_release);
	return TRUE;
}

void shakti_uart_tcp_flush(void)
{
	uint32 sent = 0U;
	ssize_t result;

	if (tx_count == 0U) {
		return;
	}
	pthread_mutex_lock(&client_mutex);
	while ((client_fd >= 0) && (sent < tx_count)) {
		result = send(client_fd, &tx_buffer[sent], tx_count - sent, 0);
		if (result <= 0) {
			break;
		}
		sent += (uint32)result;
	}
	pthread_mutex_unlock(&client_mutex);
	tx_count = 0U;
}

void shakti_uart_tcp_put(uint8 data)
{
	if (tx_count == UART_TCP_TX_SIZE) {
		shakti_uart_tcp_flush();
	}
	tx_buffer[tx_count] = data;
	tx_count++;
}
