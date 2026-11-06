/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/* Read-only libfdt API subset, pinned to dtc/libfdt v1.7.2. */
#ifndef LIBFDT_H
#define LIBFDT_H

#include <stddef.h>
#include <stdint.h>
#include "fdt.h"

#define FDT_ERR_NOTFOUND       1
#define FDT_ERR_BADOFFSET      4
#define FDT_ERR_BADPATH        5
#define FDT_ERR_BADPHANDLE     6
#define FDT_ERR_TRUNCATED      8
#define FDT_ERR_BADMAGIC       9
#define FDT_ERR_BADVERSION    10
#define FDT_ERR_BADSTRUCTURE  11
#define FDT_ERR_BADVALUE      15
#define FDT_ERR_ALIGNMENT     19

static inline uint32_t fdt32_to_cpu(fdt32_t value)
{
	const uint8_t *p = (const uint8_t *)&value;
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		((uint32_t)p[2] << 8) | p[3];
}

static inline uint64_t fdt64_to_cpu(fdt64_t value)
{
	const uint8_t *p = (const uint8_t *)&value;
	return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
		((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
		((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
		((uint64_t)p[6] << 8) | p[7];
}

uint32_t fdt_totalsize(const void *fdt);
int fdt_check_header(const void *fdt);
int fdt_check_full(const void *fdt, size_t bufsize);
int fdt_next_node(const void *fdt, int offset, int *depth);
const char *fdt_get_name(const void *fdt, int nodeoffset, int *length);
const void *fdt_getprop(const void *fdt, int nodeoffset, const char *name,
	int *length);
int fdt_first_property_offset(const void *fdt, int nodeoffset);
int fdt_next_property_offset(const void *fdt, int offset);
const struct fdt_property *fdt_get_property_by_offset(const void *fdt,
	int offset, int *length);
const char *fdt_string(const void *fdt, int stringoffset);
int fdt_parent_offset(const void *fdt, int nodeoffset);
int fdt_path_offset(const void *fdt, const char *path);
int fdt_get_path(const void *fdt, int nodeoffset, char *buf, int buflen);
uint32_t fdt_get_phandle(const void *fdt, int nodeoffset);
int fdt_node_offset_by_phandle(const void *fdt, uint32_t phandle);
const char *fdt_strerror(int error);

#endif
