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


PROPERTY_NAMES = [
    "compatible", "#address-cells", "#size-cells", "#clock-cells",
    "#interrupt-cells", "#gpio-cells", "#dma-cells", "phandle", "reg",
    "ranges", "clocks", "interrupt-controller", "interrupt-parent",
    "interrupts", "interrupt-names", "test-gpios", "dmas", "dma-names",
    "spi-max-frequency", "athrill,test-resources",
    "athrill,test-spi-child", "athrill,test-i2c-child",
    "athrill,test-uart-child", "status", "size",
    "int-gpios", "axis-align-x",
]
STRING_BLOCK = strings(*PROPERTY_NAMES)
STRING_OFFSETS: dict[str, int] = {}
at = 0
for item in PROPERTY_NAMES:
    STRING_OFFSETS[item] = at
    at += len(item) + 1


def node(name: str, props: list[tuple[str, bytes]], children: list[bytes]) -> bytes:
    result = be32(BEGIN_NODE) + name.encode() + b"\0"
    result += b"\0" * (-len(result) % 4)
    for prop_name, value in props:
        result += be32(PROP, len(value), STRING_OFFSETS[prop_name]) + value
        result += b"\0" * (-len(result) % 4)
    result += b"".join(children) + be32(END_NODE)
    return result


def make_blob(root: bytes) -> bytes:
    structure = root + be32(END)
    reserve = b"\0" * 16
    struct_offset = 40 + len(reserve)
    strings_offset = struct_offset + len(structure)
    total = strings_offset + len(STRING_BLOCK)
    header = be32(0xD00DFEED, total, struct_offset, strings_offset, 40,
                  17, 16, 0, len(STRING_BLOCK), len(structure))
    return header + reserve + structure + STRING_BLOCK


def board(children: list[bytes], address_cells: int = 1,
          size_cells: int = 1) -> bytes:
    return node("", [("compatible", strings("athrill,test-board")),
                     ("#address-cells", be32(address_cells)),
                     ("#size-cells", be32(size_cells))], children)


def provider(name: str, phandle: int, cells_name: str,
             cell_count: int, dependency: int | None = None,
             disabled: bool = False) -> bytes:
    props = [("compatible", strings("fixed-clock")),
             (cells_name, be32(cell_count)), ("phandle", be32(phandle))]
    if dependency is not None:
        props.append(("clocks", be32(dependency)))
    if disabled:
        props.append(("status", strings("disabled")))
    return node(name, props, [])


def clock(name: str, phandle: int, dependency: int | None = None) -> bytes:
    return provider(name, phandle, "#clock-cells", 0, dependency)


def device(name: str, address: int, size: int, clock_handle: int | None = None,
           compatible: str = "athrill,test-mmio",
           extra: list[tuple[str, bytes]] | None = None) -> bytes:
    props = [("compatible", strings(compatible)),
             ("reg", be32(address, size))]
    if clock_handle is not None:
        props.append(("clocks", be32(clock_handle)))
    if extra:
        props.extend(extra)
    return node(name, props, [])


def bus_controller(kind: str, address: int, children: list[bytes],
                   expected_property: str | None = None,
                   expected: int = 0) -> bytes:
    props = [("compatible", strings(f"athrill,test-{kind}-controller")),
             ("reg", be32(address, 0x100)),
             ("#address-cells", be32(1)), ("#size-cells", be32(0))]
    if expected_property:
        props.append((expected_property, be32(expected)))
    return node(f"{kind}@{address:x}", props, children)


def peripheral(name: str, address: int | None = None,
               spi: bool = False) -> bytes:
    props = [("compatible", strings("athrill,test-peripheral"))]
    if address is not None:
        props.append(("reg", be32(address)))
    if spi:
        props.append(("spi-max-frequency", be32(1_000_000)))
    return node(name, props, [])


def main() -> None:
    output = pathlib.Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    resources = device("device@1000", 0x1000, 0x100, extra=[
        ("interrupt-parent", be32(2)), ("interrupts", be32(17)),
        ("interrupt-names", strings("irq")),
        ("test-gpios", be32(3, 5, 1)),
        ("dmas", be32(4, 9)), ("dma-names", strings("rx")),
        ("athrill,test-resources", b""),
    ])
    wiring = board([
        bus_controller("spi", 0x2000,
                       [peripheral("chip@0", 0, spi=True)],
                       "athrill,test-spi-child", 0),
        bus_controller("i2c", 0x3000,
                       [peripheral("chip@e", 0x0E)],
                       "athrill,test-i2c-child", 0x0E),
        bus_controller("uart", 0x4000,
                       [peripheral("peer")],
                       "athrill,test-uart-child", 0),
    ])
    fram = board([
        bus_controller("spi", 0x2000, [node("fram@0", [
            ("compatible", strings("infineon,fm25xxx")),
            ("reg", be32(0)),
            ("spi-max-frequency", be32(20_000_000)),
            ("size", be32(32768)),
        ], [])]),
    ])
    barometer = board([
        bus_controller("spi", 0x2000, [node("pressure@0", [
            ("compatible", strings("meas,ms5611")),
            ("reg", be32(0)),
            ("spi-max-frequency", be32(20_000_000)),
        ], [])]),
    ])
    magnetometer = board([
        bus_controller("i2c", 0x3000, [node("magnetometer@e", [
            ("compatible", strings("isentek,ist8310")),
            ("reg", be32(0x0e)),
        ], [])]),
    ])
    gps = board([
        bus_controller("uart", 0x4000, [node("gnss", [
            ("compatible", strings("u-blox,m8")),
        ], [])]),
    ])
    imu = board([
        provider("gpio", 3, "#gpio-cells", 2),
        bus_controller("spi", 0x2000, [node("imu@0", [
            ("compatible", strings("invensense,icm20689")),
            ("reg", be32(0)),
            ("spi-max-frequency", be32(8_000_000)),
            ("int-gpios", be32(3, 4, 0)),
            ("axis-align-x", be32(2)),
        ], [])]),
    ])
    overflow_reg = node("", [
        ("compatible", strings("athrill,test-board")),
        ("#address-cells", be32(2)), ("#size-cells", be32(2)),
    ], [node("device@fffffffffffffff0", [
        ("compatible", strings("athrill,test-mmio")),
        ("reg", be32(0xFFFFFFFF, 0xFFFFFFF0, 0, 0x20)),
    ], [])])
    overflow_ranges = node("", [
        ("compatible", strings("athrill,test-board")),
        ("#address-cells", be32(2)), ("#size-cells", be32(2)),
    ], [node("bus", [
        ("compatible", strings("simple-bus")),
        ("#address-cells", be32(2)), ("#size-cells", be32(2)),
        ("ranges", be32(0, 0, 0xFFFFFFFF, 0xFFFFFF80, 0, 0x100)),
    ], [node("device@0", [
        ("compatible", strings("athrill,test-mmio")),
        ("reg", be32(0, 0, 0, 0x100)),
    ], [])])])
    blobs = {
        "valid.dtb": board([clock("clock", 1),
                            device("device@1000", 0x1000, 0x100, 1)]),
        "order.dtb": board([clock("clock", 1),
                            device("device@3000", 0x3000, 0x100, 1),
                            device("device@2000", 0x2000, 0x100, 1)]),
        "resources.dtb": board([
            provider("intc", 2, "#interrupt-cells", 1),
            provider("gpio", 3, "#gpio-cells", 2),
            provider("dma", 4, "#dma-cells", 1), resources,
        ]),
        "wiring.dtb": wiring,
        "fram.dtb": fram,
        "barometer.dtb": barometer,
        "magnetometer.dtb": magnetometer,
        "gps.dtb": gps,
        "imu.dtb": imu,
        "overlap.dtb": board([clock("clock", 1),
                              device("device@1000", 0x1000, 0x100, 1),
                              device("device@1080", 0x1080, 0x100, 1)]),
        "missing-provider.dtb": board([
            device("device@1000", 0x1000, 0x100, 99)]),
        "disabled-provider.dtb": board([
            provider("clock", 1, "#clock-cells", 0, disabled=True),
            device("device@1000", 0x1000, 0x100, 1)]),
        "duplicate-phandle.dtb": board([clock("clock-a", 1),
                                         clock("clock-b", 1)]),
        "cycle.dtb": board([clock("clock-a", 1, 2),
                            clock("clock-b", 2, 1)]),
        "duplicate-spi.dtb": board([
            bus_controller("spi", 0x2000,
                           [peripheral("a@0", 0, spi=True),
                            peripheral("b@0", 0, spi=True)])]),
        "duplicate-i2c.dtb": board([
            bus_controller("i2c", 0x3000,
                           [peripheral("a@e", 0x0E),
                            peripheral("b@e", 0x0E)])]),
        "bad-reg.dtb": board([node("device@1000", [
            ("compatible", strings("athrill,test-mmio")),
            ("reg", be32(0x1000)),
        ], [])]),
        "overflow-reg.dtb": overflow_reg,
        "overflow-ranges.dtb": overflow_ranges,
        "plugin.dtb": board([clock("clock", 1),
                             device("device@1000", 0x1000, 0x100, 1,
                                    "vendor,plugin")]),
    }
    for filename, root in blobs.items():
        (output / filename).write_bytes(make_blob(root))

    valid = make_blob(blobs["valid.dtb"])
    (output / "truncated.dtb").write_bytes(valid[:-1])
    malformed = bytearray(valid)
    structure_offset = struct.unpack_from(">I", malformed, 8)[0]
    first_prop = structure_offset + 8
    struct.pack_into(">I", malformed, first_prop + 8, len(STRING_BLOCK) + 4)
    (output / "bad-string-offset.dtb").write_bytes(malformed)


if __name__ == "__main__":
    main()
