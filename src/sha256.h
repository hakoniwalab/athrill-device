/* SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_SHA256_H
#define ATHRILL_DEVICE_SHA256_H

#include <stddef.h>
#include <stdint.h>

void athrill_sha256(const void *data, size_t size, uint8_t digest[32]);

#endif
