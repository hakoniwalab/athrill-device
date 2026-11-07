/*
 * UART lines over TCP (see stm32_uart_tcp.h). The ring has one producer
 * (the link's thread) and one consumer (the device tick), synchronised with
 * C11 atomics.
 */
#include "stm32_uart_tcp.h"
#include "stm32_uart.h"
#include "athrill_device_host.h"
#include "std_errno.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define UART_COUNT	8U
#define RING_SIZE	65536U	/* power of two */
#define TX_SIZE		4096U

typedef struct {
	uint32 number;		/* UART 1 to 8 */
	uint32 port;
	int listen_fd;
	pthread_mutex_t client_mutex;
	int client_fd;
	uint8 rx_ring[RING_SIZE];
	atomic_uint rx_head;	/* next byte to take (consumer) */
	atomic_uint rx_tail;	/* next byte to fill (producer) */
	uint8 tx[TX_SIZE];
	uint32 tx_count;
} UartTcpType;

static UartTcpType links[UART_COUNT];
static std_bool opened;

static void set_client(UartTcpType *l, int fd)
{
	pthread_mutex_lock(&l->client_mutex);
	l->client_fd = fd;
	pthread_mutex_unlock(&l->client_mutex);
}

static void put_into_ring(UartTcpType *l, const uint8 *data, uint32 size)
{
	uint32 i;
	uint32 tail;

	for (i = 0U; i < size; i++) {
		tail = atomic_load_explicit(&l->rx_tail, memory_order_relaxed);
		while ((tail - atomic_load_explicit(&l->rx_head, memory_order_acquire)) >= RING_SIZE) {
			usleep(1000);
		}
		l->rx_ring[tail & (RING_SIZE - 1U)] = data[i];
		atomic_store_explicit(&l->rx_tail, tail + 1U, memory_order_release);
	}
}

static void *receive_thread(void *arg)
{
	UartTcpType *l = (UartTcpType *)arg;
	uint8 buffer[1024];
	ssize_t received;
	int fd;

	for (;;) {
		fd = accept(l->listen_fd, NULL, NULL);
		if (fd < 0) {
			if (errno == EINTR) {
				continue;
			}
			return NULL;
		}
		printf("UART%u TCP: client connected\n", l->number);
		set_client(l, fd);
		for (;;) {
			received = recv(fd, buffer, sizeof(buffer), 0);
			if (received <= 0) {
				break;
			}
			put_into_ring(l, buffer, (uint32)received);
		}
		set_client(l, -1);
		close(fd);
		printf("UART%u TCP: client disconnected\n", l->number);
	}
}

static std_bool listen_on(UartTcpType *l)
{
	struct sockaddr_in address;
	pthread_t thread;
	int reuse = 1;

	l->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (l->listen_fd < 0) {
		return FALSE;
	}
	(void)setsockopt(l->listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons((uint16)l->port);
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if ((bind(l->listen_fd, (struct sockaddr *)&address, sizeof(address)) != 0)
		|| (listen(l->listen_fd, 1) != 0)
		|| (pthread_create(&thread, NULL, receive_thread, l) != 0)) {
		printf("ERROR: UART%u TCP: can not listen on 127.0.0.1:%u (errno=%d)\n", l->number, l->port, errno);
		close(l->listen_fd);
		l->listen_fd = -1;
		return FALSE;
	}
	(void)pthread_detach(thread);
	printf("UART%u TCP: listening on 127.0.0.1:%u\n", l->number, l->port);
	return TRUE;
}

static void flush(UartTcpType *l)
{
	uint32 sent = 0U;
	ssize_t result;

	if (l->tx_count == 0U) {
		return;
	}
	pthread_mutex_lock(&l->client_mutex);
	while ((l->client_fd >= 0) && (sent < l->tx_count)) {
		result = send(l->client_fd, &l->tx[sent], l->tx_count - sent, 0);
		if (result <= 0) {
			break;
		}
		sent += (uint32)result;
	}
	pthread_mutex_unlock(&l->client_mutex);
	l->tx_count = 0U;
}

static void transmit(void *chip, uint8 data)
{
	UartTcpType *l = (UartTcpType *)chip;

	if (l->tx_count == TX_SIZE) {
		flush(l);
	}
	l->tx[l->tx_count] = data;
	l->tx_count++;
}

static const Stm32UartChipOps tcp_ops = { transmit };

static void open_links(void)
{
	uint32 i;
	char key[64];
	const char *text = NULL;

	for (i = 0U; i < UART_COUNT; i++) {
		UartTcpType *l = &links[i];

		l->number = i + 1U;
		l->port = 0U;
		l->listen_fd = -1;
		l->client_fd = -1;
		(void)pthread_mutex_init(&l->client_mutex, NULL);
		atomic_init(&l->rx_head, 0U);
		atomic_init(&l->rx_tail, 0U);
		l->tx_count = 0U;
		(void)snprintf(key, sizeof(key), "DEVICE_CONFIG_STM32_UART%u_TCP_PORT", l->number);
		if (athrill_device_host()->host_value(NULL, key, &text) == 0) {
			l->port = (uint32)strtoul(text, NULL, 10);
		}
		if ((l->port == 0U) || !listen_on(l)) {
			l->port = 0U;
		}
	}
}

void stm32_uart_tcp_attach(void)
{
	uint32 i;

	if (!opened) {
		open_links();
		opened = TRUE;
	}
	for (i = 0U; i < UART_COUNT; i++) {
		if (links[i].port != 0U) {
			stm32_uart_attach(links[i].number, &tcp_ops, &links[i]);
		}
	}
}

void stm32_uart_tcp_tick(void)
{
	uint32 i;

	for (i = 0U; i < UART_COUNT; i++) {
		UartTcpType *l = &links[i];
		uint32 head;

		if (l->port == 0U) {
			continue;
		}
		head = atomic_load_explicit(&l->rx_head, memory_order_relaxed);
		while (head != atomic_load_explicit(&l->rx_tail, memory_order_acquire)) {
			if (stm32_uart_chip_send(l->number, l->rx_ring[head & (RING_SIZE - 1U)]) != STD_E_OK) {
				break;	/* the UART's queue is full: the rest waits in the ring */
			}
			head++;
		}
		atomic_store_explicit(&l->rx_head, head, memory_order_release);
		flush(l);
	}
}
