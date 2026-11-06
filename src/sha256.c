/* SPDX-License-Identifier: MIT */
#include "sha256.h"

#include <string.h>

typedef struct {
	uint32_t h[8];
	uint64_t bits;
	uint8_t block[64];
	size_t used;
} Sha256;

static const uint32_t constants[64] = {
	0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
	0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
	0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
	0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
	0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
	0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
	0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
	0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U
};

static uint32_t rotate(uint32_t value, unsigned int bits)
{
	return (value >> bits) | (value << (32U - bits));
}

static uint32_t load32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | p[3];
}

static void transform(Sha256 *s, const uint8_t block[64])
{
	uint32_t w[64], a, b, c, d, e, f, g, h, t1, t2;
	unsigned int i;
	for (i = 0; i < 16; i++) {
		w[i] = load32(block + i * 4U);
	}
	for (; i < 64; i++) {
		uint32_t x = rotate(w[i - 15], 7) ^ rotate(w[i - 15], 18) ^ (w[i - 15] >> 3);
		uint32_t y = rotate(w[i - 2], 17) ^ rotate(w[i - 2], 19) ^ (w[i - 2] >> 10);
		w[i] = w[i - 16] + x + w[i - 7] + y;
	}
	a = s->h[0];
	b = s->h[1];
	c = s->h[2];
	d = s->h[3];
	e = s->h[4];
	f = s->h[5];
	g = s->h[6];
	h = s->h[7];
	for (i = 0; i < 64; i++) {
		uint32_t s1 = rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t s0 = rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
		t1 = h + s1 + ch + constants[i] + w[i];
		t2 = s0 + maj;
		h = g;
		g = f;
		f = e;
		e = d + t1;
		d = c;
		c = b;
		b = a;
		a = t1 + t2;
	}
	s->h[0] += a;
	s->h[1] += b;
	s->h[2] += c;
	s->h[3] += d;
	s->h[4] += e;
	s->h[5] += f;
	s->h[6] += g;
	s->h[7] += h;
}

static void update(Sha256 *s, const uint8_t *data, size_t size)
{
	while (size != 0U) {
		size_t take = 64U - s->used;
		if (take > size) {
			take = size;
		}
		memcpy(s->block + s->used, data, take);
		s->used += take;
		data += take;
		size -= take;
		if (s->used == 64U) {
			transform(s, s->block);
			s->bits += 512U;
			s->used = 0U;
		}
	}
}

void athrill_sha256(const void *data, size_t size, uint8_t digest[32])
{
	Sha256 s = {{
			0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
			0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U
		}, 0, {0}, 0
	};
	uint64_t bits;
	unsigned int i;
	update(&s, data, size);
	bits = s.bits + (uint64_t)s.used * 8U;
	s.block[s.used++] = 0x80U;
	if (s.used > 56U) {
		memset(s.block + s.used, 0, 64U-s.used);
		transform(&s, s.block);
		s.used = 0U;
	}
	memset(s.block + s.used, 0, 56U-s.used);
	for (i = 0; i < 8; i++) {
		s.block[63U-i] = (uint8_t)(bits >> (i * 8U));
	}
	transform(&s, s.block);
	for (i = 0; i < 8; i++) {
		digest[i * 4U] = (uint8_t)(s.h[i] >> 24);
		digest[i * 4U+1U] = (uint8_t)(s.h[i] >> 16);
		digest[i * 4U+2U] = (uint8_t)(s.h[i] >> 8);
		digest[i * 4U+3U] = (uint8_t)s.h[i];
	}
}
