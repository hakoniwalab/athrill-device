// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * Compact read-only libfdt subset for athrill-device.
 * Semantics and wire checks follow dtc/libfdt v1.7.2.
 * Copyright (C) 2006 David Gibson, IBM Corporation.
 */
#include "libfdt.h"

#include <limits.h>
#include <string.h>

#define ALIGN4(v) (((v) + 3U) & ~3U)

static const struct fdt_header *header(const void *fdt)
{
	return (const struct fdt_header *)fdt;
}

uint32_t fdt_totalsize(const void *fdt)
{
	return fdt32_to_cpu(header(fdt)->totalsize);
}

static int block_ok(uint32_t total, uint32_t offset, uint32_t size)
{
	return offset <= total && size <= total - offset;
}

int fdt_check_header(const void *fdt)
{
	const struct fdt_header *h;
	uint32_t total;

	if (fdt == NULL) return -FDT_ERR_TRUNCATED;
	if (((uintptr_t)fdt & 7U) != 0U) return -FDT_ERR_ALIGNMENT;
	h = header(fdt);
	if (fdt32_to_cpu(h->magic) != FDT_MAGIC) return -FDT_ERR_BADMAGIC;
	if (fdt32_to_cpu(h->version) < 2U || fdt32_to_cpu(h->version) > 17U ||
		fdt32_to_cpu(h->last_comp_version) > 16U ||
		fdt32_to_cpu(h->version) < fdt32_to_cpu(h->last_comp_version)) {
		return -FDT_ERR_BADVERSION;
	}
	total = fdt_totalsize(fdt);
	if (total < sizeof(*h) || total > INT_MAX) return -FDT_ERR_TRUNCATED;
	if (!block_ok(total, fdt32_to_cpu(h->off_dt_struct),
		fdt32_to_cpu(h->size_dt_struct)) ||
		!block_ok(total, fdt32_to_cpu(h->off_dt_strings),
		fdt32_to_cpu(h->size_dt_strings)) ||
		fdt32_to_cpu(h->off_mem_rsvmap) < sizeof(*h) ||
		fdt32_to_cpu(h->off_mem_rsvmap) > total) {
		return -FDT_ERR_TRUNCATED;
	}
	return 0;
}

static const uint8_t *structure(const void *fdt)
{
	return (const uint8_t *)fdt + fdt32_to_cpu(header(fdt)->off_dt_struct);
}

static uint32_t structure_size(const void *fdt)
{
	return fdt32_to_cpu(header(fdt)->size_dt_struct);
}

static uint32_t get_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		((uint32_t)p[2] << 8) | p[3];
}

static int next_tag(const void *fdt, uint32_t offset, uint32_t *tag,
	uint32_t *next)
{
	const uint8_t *s = structure(fdt);
	uint32_t size = structure_size(fdt);
	uint32_t len;

	if (offset > size || size - offset < 4U) return -FDT_ERR_TRUNCATED;
	*tag = get_be32(s + offset);
	offset += 4U;
	switch (*tag) {
	case FDT_BEGIN_NODE:
		while (offset < size && s[offset] != 0U) offset++;
		if (offset >= size) return -FDT_ERR_TRUNCATED;
		offset = ALIGN4(offset + 1U);
		break;
	case FDT_PROP:
		if (size - offset < 8U) return -FDT_ERR_TRUNCATED;
		len = get_be32(s + offset);
		offset += 8U;
		if (len > size - offset) return -FDT_ERR_TRUNCATED;
		offset = ALIGN4(offset + len);
		break;
	case FDT_END_NODE:
	case FDT_NOP:
	case FDT_END:
		break;
	default:
		return -FDT_ERR_BADSTRUCTURE;
	}
	if (offset > size) return -FDT_ERR_TRUNCATED;
	*next = offset;
	return 0;
}

int fdt_check_full(const void *fdt, size_t bufsize)
{
	uint32_t off = 0U, tag, next;
	int depth = 0;
	int seen_root = 0;
	int ended = 0;
	int rc;

	rc = fdt_check_header(fdt);
	if (rc != 0) return rc;
	if (fdt_totalsize(fdt) > bufsize) return -FDT_ERR_TRUNCATED;
	while (off < structure_size(fdt)) {
		rc = next_tag(fdt, off, &tag, &next);
		if (rc != 0) return rc;
		if (tag == FDT_BEGIN_NODE) {
			seen_root = 1;
			depth++;
		} else if (tag == FDT_END_NODE) {
			if (--depth < 0) return -FDT_ERR_BADSTRUCTURE;
		} else if (tag == FDT_PROP && depth <= 0) {
			return -FDT_ERR_BADSTRUCTURE;
		} else if (tag == FDT_END) {
			if (depth != 0 || !seen_root) return -FDT_ERR_BADSTRUCTURE;
			ended = 1;
			break;
		}
		off = next;
	}
	return ended ? 0 : -FDT_ERR_TRUNCATED;
}

const char *fdt_get_name(const void *fdt, int nodeoffset, int *length)
{
	const uint8_t *s = structure(fdt);
	uint32_t off = (uint32_t)nodeoffset;
	uint32_t size = structure_size(fdt);
	const char *name;
	const uint8_t *end;

	if (nodeoffset < 0 || off > size || size - off < 5U ||
		get_be32(s + off) != FDT_BEGIN_NODE) {
		if (length != NULL) *length = -FDT_ERR_BADOFFSET;
		return NULL;
	}
	name = (const char *)(s + off + 4U);
	end = memchr(name, 0, size - off - 4U);
	if (end == NULL) {
		if (length != NULL) *length = -FDT_ERR_TRUNCATED;
		return NULL;
	}
	if (length != NULL) *length = (int)(end - (const uint8_t *)name);
	return name;
}

static int node_end_offset(const void *fdt, int nodeoffset, uint32_t *end)
{
	uint32_t off = (uint32_t)nodeoffset, tag, next;
	int depth = 0, rc;

	if (fdt_get_name(fdt, nodeoffset, NULL) == NULL) return -FDT_ERR_BADOFFSET;
	do {
		rc = next_tag(fdt, off, &tag, &next);
		if (rc != 0) return rc;
		if (tag == FDT_BEGIN_NODE) depth++;
		else if (tag == FDT_END_NODE) depth--;
		off = next;
	} while (depth > 0);
	*end = off;
	return 0;
}

int fdt_next_node(const void *fdt, int offset, int *depth)
{
	uint32_t off, tag, next;
	int d = depth != NULL ? *depth : 0;
	int rc;

	if (offset < 0) {
		if (structure_size(fdt) < 4U || get_be32(structure(fdt)) != FDT_BEGIN_NODE)
			return -FDT_ERR_BADSTRUCTURE;
		if (depth != NULL) *depth = 0;
		return 0;
	}
	off = (uint32_t)offset;
	rc = next_tag(fdt, off, &tag, &next);
	if (rc != 0 || tag != FDT_BEGIN_NODE) return -FDT_ERR_BADOFFSET;
	off = next;
	while (off < structure_size(fdt)) {
		rc = next_tag(fdt, off, &tag, &next);
		if (rc != 0) return rc;
		if (tag == FDT_BEGIN_NODE) {
			if (depth != NULL) *depth = d + 1;
			return (int)off;
		}
		if (tag == FDT_END_NODE) {
			d--;
			if (d < 0) return -FDT_ERR_NOTFOUND;
		}
		off = next;
	}
	return -FDT_ERR_NOTFOUND;
}

static const char *string_at(const void *fdt, uint32_t off)
{
	const struct fdt_header *h = header(fdt);
	uint32_t size = fdt32_to_cpu(h->size_dt_strings);
	const char *base;

	if (off >= size) return NULL;
	base = (const char *)fdt + fdt32_to_cpu(h->off_dt_strings);
	if (memchr(base + off, 0, size - off) == NULL) return NULL;
	return base + off;
}

const void *fdt_getprop(const void *fdt, int nodeoffset, const char *name,
	int *length)
{
	const uint8_t *s = structure(fdt);
	uint32_t off = (uint32_t)nodeoffset, tag, next, len, nameoff;
	int rc;

	if (fdt_get_name(fdt, nodeoffset, NULL) == NULL) goto bad;
	rc = next_tag(fdt, off, &tag, &off);
	if (rc != 0) goto bad;
	while (off < structure_size(fdt)) {
		rc = next_tag(fdt, off, &tag, &next);
		if (rc != 0) goto bad;
		if (tag == FDT_PROP) {
			const char *pname;
			len = get_be32(s + off + 4U);
			nameoff = get_be32(s + off + 8U);
			pname = string_at(fdt, nameoff);
			if (pname == NULL) goto bad;
			if (strcmp(pname, name) == 0) {
				if (length != NULL) *length = (int)len;
				return s + off + 12U;
			}
		} else if (tag == FDT_BEGIN_NODE || tag == FDT_END_NODE) {
			break;
		}
		off = next;
	}
	if (length != NULL) *length = -FDT_ERR_NOTFOUND;
	return NULL;
bad:
	if (length != NULL) *length = -FDT_ERR_BADSTRUCTURE;
	return NULL;
}

int fdt_parent_offset(const void *fdt, int nodeoffset)
{
	int off = -1, depth = 0;
	int stack[64];

	while ((off = fdt_next_node(fdt, off, &depth)) >= 0) {
		if (depth < 0 || depth >= (int)(sizeof(stack) / sizeof(stack[0])))
			return -FDT_ERR_BADSTRUCTURE;
		if (off == nodeoffset) return depth == 0 ? -FDT_ERR_NOTFOUND : stack[depth - 1];
		stack[depth] = off;
	}
	return -FDT_ERR_BADOFFSET;
}

int fdt_get_path(const void *fdt, int nodeoffset, char *buf, int buflen)
{
	int chain[64], count = 0, off = nodeoffset, i, n;
	char *p = buf;
	int left = buflen;
	const char *name;

	if (buf == NULL || buflen <= 0) return -FDT_ERR_BADVALUE;
	while (off >= 0 && count < (int)(sizeof(chain) / sizeof(chain[0]))) {
		chain[count++] = off;
		off = fdt_parent_offset(fdt, off);
	}
	if (count == 0 || count >= (int)(sizeof(chain) / sizeof(chain[0])))
		return -FDT_ERR_BADSTRUCTURE;
	if (left < 2) return -FDT_ERR_TRUNCATED;
	*p++ = '/'; left--;
	for (i = count - 2; i >= 0; i--) {
		name = fdt_get_name(fdt, chain[i], &n);
		if (name == NULL || n + (i != 0 ? 1 : 0) >= left)
			return -FDT_ERR_TRUNCATED;
		memcpy(p, name, (size_t)n); p += n; left -= n;
		if (i != 0) { *p++ = '/'; left--; }
	}
	*p = 0;
	return 0;
}

int fdt_path_offset(const void *fdt, const char *path)
{
	int off = -1, depth = 0;
	char candidate[1024];

	if (path == NULL || path[0] != '/') return -FDT_ERR_BADPATH;
	while ((off = fdt_next_node(fdt, off, &depth)) >= 0) {
		if (fdt_get_path(fdt, off, candidate, sizeof(candidate)) == 0 &&
			strcmp(candidate, path) == 0) return off;
	}
	return -FDT_ERR_NOTFOUND;
}

uint32_t fdt_get_phandle(const void *fdt, int nodeoffset)
{
	const fdt32_t *p;
	int len;
	uint32_t value;

	p = fdt_getprop(fdt, nodeoffset, "phandle", &len);
	if (p == NULL) p = fdt_getprop(fdt, nodeoffset, "linux,phandle", &len);
	if (p == NULL || len != 4) return 0U;
	value = fdt32_to_cpu(*p);
	return (value == 0U || value == 0xffffffffU) ? 0U : value;
}

int fdt_node_offset_by_phandle(const void *fdt, uint32_t phandle)
{
	int off = -1, depth = 0;
	if (phandle == 0U || phandle == 0xffffffffU) return -FDT_ERR_BADPHANDLE;
	while ((off = fdt_next_node(fdt, off, &depth)) >= 0)
		if (fdt_get_phandle(fdt, off) == phandle) return off;
	return -FDT_ERR_NOTFOUND;
}

const char *fdt_strerror(int error)
{
	if (error > 0) error = -error;
	switch (-error) {
	case 0: return "<no error>";
	case FDT_ERR_NOTFOUND: return "FDT_ERR_NOTFOUND";
	case FDT_ERR_BADOFFSET: return "FDT_ERR_BADOFFSET";
	case FDT_ERR_BADPATH: return "FDT_ERR_BADPATH";
	case FDT_ERR_BADPHANDLE: return "FDT_ERR_BADPHANDLE";
	case FDT_ERR_TRUNCATED: return "FDT_ERR_TRUNCATED";
	case FDT_ERR_BADMAGIC: return "FDT_ERR_BADMAGIC";
	case FDT_ERR_BADVERSION: return "FDT_ERR_BADVERSION";
	case FDT_ERR_BADSTRUCTURE: return "FDT_ERR_BADSTRUCTURE";
	case FDT_ERR_BADVALUE: return "FDT_ERR_BADVALUE";
	case FDT_ERR_ALIGNMENT: return "FDT_ERR_ALIGNMENT";
	default: return "<unknown error>";
	}
}
