#!/usr/bin/env python3
"""Emit deterministic, small DTB v17 fixtures (no host dtc dependency)."""
from __future__ import annotations

import pathlib
import struct
import sys

BEGIN_NODE, END_NODE, PROP, END = 1, 2, 3, 9


def be32(*values: int) -> bytes:
    return b"".join(struct.pack(">I", value) for value in values)


def strings(*values: str) -> bytes:
    return b"".join(value.encode() + b"\0" for value in values)


def node(name: str, props: list[tuple[str, bytes]], children: list[bytes]) -> bytes:
    result = be32(BEGIN_NODE) + name.encode() + b"\0"
    result += b"\0" * (-len(result) % 4)
    for prop_name, value in props:
        result += be32(PROP, len(value), STRING_OFFSETS[prop_name]) + value
        result += b"\0" * (-len(result) % 4)
    result += b"".join(children) + be32(END_NODE)
    return result


PROPERTY_NAMES = [
    "compatible", "#address-cells", "#size-cells", "#clock-cells",
    "phandle", "reg", "clocks",
]
STRING_BLOCK = strings(*PROPERTY_NAMES)
STRING_OFFSETS: dict[str, int] = {}
at = 0
for item in PROPERTY_NAMES:
    STRING_OFFSETS[item] = at
    at += len(item) + 1


def make_blob(root: bytes) -> bytes:
    structure = root + be32(END)
    reserve = b"\0" * 16
    struct_offset = 40 + len(reserve)
    strings_offset = struct_offset + len(structure)
    total = strings_offset + len(STRING_BLOCK)
    header = be32(0xD00DFEED, total, struct_offset, strings_offset, 40,
                  17, 16, 0, len(STRING_BLOCK), len(structure))
    return header + reserve + structure + STRING_BLOCK


def board(children: list[bytes]) -> bytes:
    return node("", [("compatible", strings("athrill,test-board")),
                     ("#address-cells", be32(1)),
                     ("#size-cells", be32(1))], children)


def clock(name: str, phandle: int, dependency: int | None = None) -> bytes:
    props = [("compatible", strings("fixed-clock")),
             ("#clock-cells", be32(0)), ("phandle", be32(phandle))]
    if dependency is not None:
        props.append(("clocks", be32(dependency)))
    return node(name, props, [])


def device(name: str, address: int, size: int, clock_handle: int,
           compatible: str = "athrill,test-mmio") -> bytes:
    return node(name, [("compatible", strings(compatible)),
                       ("reg", be32(address, size)),
                       ("clocks", be32(clock_handle))], [])


def main() -> None:
    output = pathlib.Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    blobs = {
        "valid.dtb": board([clock("clock", 1), device("device@1000", 0x1000, 0x100, 1)]),
        "overlap.dtb": board([clock("clock", 1),
                              device("device@1000", 0x1000, 0x100, 1),
                              device("device@1080", 0x1080, 0x100, 1)]),
        "missing-provider.dtb": board([device("device@1000", 0x1000, 0x100, 99)]),
        "cycle.dtb": board([clock("clock-a", 1, 2), clock("clock-b", 2, 1)]),
        "plugin.dtb": board([clock("clock", 1),
                             device("device@1000", 0x1000, 0x100, 1, "vendor,plugin")]),
    }
    for filename, root in blobs.items():
        (output / filename).write_bytes(make_blob(root))


if __name__ == "__main__":
    main()
