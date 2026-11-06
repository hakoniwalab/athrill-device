/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/* Read-only definitions from dtc/libfdt v1.7.2. */
#ifndef FDT_H
#define FDT_H

#include <stdint.h>

typedef uint32_t fdt32_t;
typedef uint64_t fdt64_t;

#define FDT_MAGIC       0xd00dfeedU
#define FDT_BEGIN_NODE  0x1U
#define FDT_END_NODE    0x2U
#define FDT_PROP        0x3U
#define FDT_NOP         0x4U
#define FDT_END         0x9U

struct fdt_header {
	fdt32_t magic;
	fdt32_t totalsize;
	fdt32_t off_dt_struct;
	fdt32_t off_dt_strings;
	fdt32_t off_mem_rsvmap;
	fdt32_t version;
	fdt32_t last_comp_version;
	fdt32_t boot_cpuid_phys;
	fdt32_t size_dt_strings;
	fdt32_t size_dt_struct;
};

struct fdt_reserve_entry { fdt64_t address; fdt64_t size; };
struct fdt_property { fdt32_t tag; fdt32_t len; fdt32_t nameoff; char data[]; };

#endif
