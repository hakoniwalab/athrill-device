/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_PROTOCOL_I2C_H
#define ATHRILL_PROTOCOL_I2C_H

#include "athrill_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillI2cProtocol AthrillI2cProtocol;

typedef struct {
	std_bool (*start)(void *context, std_bool read, std_bool repeated,
		uint64 time);
	std_bool (*write)(void *context, uint8 data, uint64 time);
	sint32 (*read)(void *context, uint8 *data, std_bool master_ack,
		uint64 time);
	void (*stop)(void *context, uint64 time);
} AthrillI2cChildOps;

sint32 athrill_i2c_protocol_create(AthrillChannel *channel,
	const AthrillI2cChildOps *child_ops, void *child_context,
	AthrillI2cProtocol **protocol);
void athrill_i2c_protocol_destroy(AthrillI2cProtocol *protocol);

/* START returns 1 for address ACK, 0 for address NACK, and -1 for misuse. */
sint32 athrill_i2c_start(AthrillI2cProtocol *protocol, uint16 address,
	std_bool read, uint64 time);
sint32 athrill_i2c_write(AthrillI2cProtocol *protocol, uint8 data,
	std_bool *ack, uint64 completion_time);
sint32 athrill_i2c_read(AthrillI2cProtocol *protocol, uint8 *data,
	std_bool master_ack, uint64 completion_time);
sint32 athrill_i2c_stop(AthrillI2cProtocol *protocol, uint64 time);
std_bool athrill_i2c_is_addressed(const AthrillI2cProtocol *protocol);
std_bool athrill_i2c_is_reading(const AthrillI2cProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif
