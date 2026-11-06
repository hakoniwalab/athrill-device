/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_PROTOCOL_SPI_H
#define ATHRILL_PROTOCOL_SPI_H

#include "athrill_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillSpiProtocol AthrillSpiProtocol;

typedef struct {
	void (*select)(void *context, uint64 time);
	sint32 (*exchange)(void *context, uint8 transmitted, uint8 *received,
		uint64 time);
	void (*deselect)(void *context, uint64 time);
} AthrillSpiChildOps;

sint32 athrill_spi_protocol_create(AthrillChannel *channel,
	const AthrillSpiChildOps *child_ops, void *child_context,
	AthrillSpiProtocol **protocol);
void athrill_spi_protocol_destroy(AthrillSpiProtocol *protocol);

sint32 athrill_spi_select(AthrillSpiProtocol *protocol, uint32 chip_select,
	uint64 time);
sint32 athrill_spi_exchange(AthrillSpiProtocol *protocol, uint8 transmitted,
	uint8 *received, uint64 completion_time);
sint32 athrill_spi_deselect(AthrillSpiProtocol *protocol, uint64 time);
std_bool athrill_spi_is_selected(const AthrillSpiProtocol *protocol);
uint64 athrill_spi_frame_items(const AthrillSpiProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif
