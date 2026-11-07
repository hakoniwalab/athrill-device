#!/usr/bin/env python3
"""Generate bus traceability tests from board DTBs without using libfdt."""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import struct
from pathlib import Path
from typing import Iterable


FDT_MAGIC = 0xD00DFEED
FDT_BEGIN_NODE = 1
FDT_END_NODE = 2
FDT_PROP = 3
FDT_NOP = 4
FDT_END = 9


@dataclasses.dataclass
class Node:
    name: str
    parent: "Node | None"
    properties: dict[str, bytes] = dataclasses.field(default_factory=dict)
    property_offsets: dict[str, int] = dataclasses.field(default_factory=dict)
    children: list["Node"] = dataclasses.field(default_factory=list)

    @property
    def path(self) -> str:
        if self.parent is None:
            return "/"
        prefix = self.parent.path.rstrip("/")
        return prefix + "/" + self.name


@dataclasses.dataclass(frozen=True)
class Specifier:
    provider_path: str
    provider_compatible: str
    cells: tuple[int, ...]


@dataclasses.dataclass
class SpiChild:
    node: Node
    chip_select: int
    cs_gpio: Specifier | None


@dataclasses.dataclass
class SpiController:
    node: Node
    children: list[SpiChild]
    dmas: list[Specifier]
    dma_names: list[str]


@dataclasses.dataclass
class I2cChild:
    node: Node
    address: int


@dataclasses.dataclass
class I2cController:
    node: Node
    children: list[I2cChild]


@dataclasses.dataclass
class UartChild:
    node: Node


@dataclasses.dataclass
class UartController:
    node: Node
    children: list[UartChild]
    dmas: list[Specifier]
    dma_names: list[str]


@dataclasses.dataclass(frozen=True)
class DmaReference:
    consumer_path: str
    specifier: Specifier
    index: int
    name: str


@dataclasses.dataclass
class DmaController:
    node: Node
    references: list[DmaReference]


@dataclasses.dataclass(frozen=True)
class SdCard:
    node: Node
    controller: Node


@dataclasses.dataclass(frozen=True)
class MmioWindow:
    address: int
    size: int


@dataclasses.dataclass(frozen=True)
class Interrupt:
    provider_path: str
    cells: tuple[int, ...]


# These native models own MMIO in the public architecture.  Transitional
# target adapters are instances, but their windows remain target-owned.
MMIO_MODEL_COMPATIBLES = {
    "st,stm32-gpio", "st,stm32-exti", "st,stm32-timers",
}
IRQ_RESOLVING_COMPATIBLES = {"st,stm32-exti", "st,stm32-timers"}
TARGET_ARCHITECTURAL_COMPATIBLES = {
    "arm,armv7m-systick", "arm,v7m-nvic", "arm,cortex-m7", "arm,armv7m-mpu",
}


def align4(value: int) -> int:
    return (value + 3) & ~3


def c_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def c_identifier(value: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not identifier or identifier[0].isdigit():
        identifier = "n_" + identifier
    return identifier


def strings(value: bytes | None) -> list[str]:
    if not value:
        return []
    if value[-1:] != b"\0":
        raise ValueError("unterminated Devicetree string list")
    return [part.decode("utf-8") for part in value[:-1].split(b"\0")]


def cells(value: bytes | None) -> list[int]:
    if value is None or len(value) % 4:
        raise ValueError("Devicetree cell property has invalid length")
    return list(struct.unpack(">" + "I" * (len(value) // 4), value))


class Fdt:
    """Small, independent reader for the flattened Devicetree format."""

    def __init__(self, path: Path):
        data = path.read_bytes()
        if len(data) < 40:
            raise ValueError(f"{path}: truncated FDT header")
        header = struct.unpack_from(">10I", data)
        (magic, total, struct_off, strings_off, _reserve_off, version,
         last_compatible, _boot_cpu, strings_size, struct_size) = header
        if magic != FDT_MAGIC or total > len(data) or version < 17 or last_compatible > 17:
            raise ValueError(f"{path}: unsupported FDT header")
        if struct_off + struct_size > total or strings_off + strings_size > total:
            raise ValueError(f"{path}: FDT block outside total size")
        self.data = data
        self._strings = data[strings_off:strings_off + strings_size]
        self.root = self._parse_structure(
            data[struct_off:struct_off + struct_size], struct_off)
        self.nodes = list(self._walk(self.root))
        self.phandles: dict[int, Node] = {}
        for node in self.nodes:
            for prop in ("phandle", "linux,phandle"):
                if prop in node.properties:
                    values = cells(node.properties[prop])
                    if len(values) != 1 or values[0] == 0 or values[0] in self.phandles:
                        raise ValueError(f"{path}: invalid or duplicate {prop} at {node.path}")
                    self.phandles[values[0]] = node

    def _parse_structure(self, block: bytes, block_offset: int) -> Node:
        offset = 0
        stack: list[Node] = []
        root: Node | None = None
        while offset + 4 <= len(block):
            token = struct.unpack_from(">I", block, offset)[0]
            offset += 4
            if token == FDT_BEGIN_NODE:
                end = block.find(b"\0", offset)
                if end < 0:
                    raise ValueError("unterminated FDT node name")
                name = block[offset:end].decode("utf-8")
                offset = align4(end + 1)
                node = Node(name=name, parent=stack[-1] if stack else None)
                if stack:
                    stack[-1].children.append(node)
                elif root is None:
                    root = node
                else:
                    raise ValueError("multiple FDT roots")
                stack.append(node)
            elif token == FDT_END_NODE:
                if not stack:
                    raise ValueError("unbalanced FDT_END_NODE")
                stack.pop()
            elif token == FDT_PROP:
                if not stack or offset + 8 > len(block):
                    raise ValueError("invalid FDT property record")
                length, name_offset = struct.unpack_from(">II", block, offset)
                offset += 8
                end = offset + length
                if end > len(block):
                    raise ValueError("FDT property outside structure block")
                name = self._string_at(name_offset)
                if name in stack[-1].properties:
                    raise ValueError(f"duplicate property {name} at {stack[-1].path}")
                stack[-1].property_offsets[name] = block_offset + offset
                stack[-1].properties[name] = block[offset:end]
                offset = align4(end)
            elif token == FDT_NOP:
                continue
            elif token == FDT_END:
                if stack or root is None:
                    raise ValueError("unbalanced FDT structure")
                return root
            else:
                raise ValueError(f"unknown FDT token {token}")
        raise ValueError("missing FDT_END")

    def _string_at(self, offset: int) -> str:
        if offset >= len(self._strings):
            raise ValueError("FDT string offset outside strings block")
        end = self._strings.find(b"\0", offset)
        if end < 0:
            raise ValueError("unterminated FDT property name")
        return self._strings[offset:end].decode("ascii")

    @staticmethod
    def _walk(node: Node) -> Iterable[Node]:
        yield node
        for child in node.children:
            yield from Fdt._walk(child)


def enabled(node: Node) -> bool:
    current: Node | None = node
    while current is not None:
        status = strings(current.properties.get("status"))
        if status and status[0] not in ("okay", "ok"):
            return False
        current = current.parent
    return True


def compatible(node: Node) -> list[str]:
    return strings(node.properties.get("compatible"))


def is_model_instance(node: Node) -> bool:
    values = compatible(node)
    return enabled(node) and bool(values) and not TARGET_ARCHITECTURAL_COMPATIBLES.intersection(values)


def integer(values: list[int]) -> int:
    if len(values) > 2:
        raise ValueError("Devicetree integer has more than two cells")
    result = 0
    for value in values:
        result = (result << 32) | value
    return result


def inherited_cell_count(node: Node | None, name: str, default: int) -> int:
    current = node
    while current is not None:
        if name in current.properties:
            value = cells(current.properties[name])
            if len(value) != 1 or value[0] > 2:
                raise ValueError(f"{current.path}: invalid {name}")
            return value[0]
        current = current.parent
    return default


def translated_address(node: Node, address: int, size: int) -> int:
    bus = node.parent
    while bus is not None and bus.parent is not None:
        raw = bus.properties.get("ranges")
        if raw:
            values = cells(raw)
            child_ac = inherited_cell_count(bus, "#address-cells", 2)
            parent_ac = inherited_cell_count(bus.parent, "#address-cells", 2)
            size_cells = inherited_cell_count(bus, "#size-cells", 1)
            width = child_ac + parent_ac + size_cells
            if width == 0 or len(values) % width:
                raise ValueError(f"{bus.path}: invalid ranges")
            for at in range(0, len(values), width):
                child = integer(values[at:at + child_ac])
                parent = integer(values[at + child_ac:at + child_ac + parent_ac])
                extent = integer(values[at + child_ac + parent_ac:at + width])
                if address >= child and address - child <= extent and size <= extent - (address - child):
                    address = parent + address - child
                    break
            else:
                raise ValueError(f"{node.path}: reg is outside parent ranges")
        bus = bus.parent
    return address


def reg_windows(node: Node) -> list[MmioWindow]:
    if node.parent is None or "reg" not in node.properties:
        return []
    address_cells = inherited_cell_count(node.parent, "#address-cells", 2)
    size_cells = inherited_cell_count(node.parent, "#size-cells", 1)
    width = address_cells + size_cells
    values = cells(node.properties["reg"])
    if size_cells == 0 or width == 0 or len(values) % width:
        return []
    result = []
    for at in range(0, len(values), width):
        address = integer(values[at:at + address_cells])
        size = integer(values[at + address_cells:at + width])
        if size == 0:
            raise ValueError(f"{node.path}: zero-sized reg")
        result.append(MmioWindow(translated_address(node, address, size), size))
    return result


def node_interrupts(fdt: Fdt, node: Node) -> list[Interrupt]:
    raw = node.properties.get("interrupts")
    if raw is None:
        return []
    current: Node | None = node
    parent_handle: int | None = None
    while current is not None:
        if "interrupt-parent" in current.properties:
            handles = cells(current.properties["interrupt-parent"])
            if len(handles) != 1:
                raise ValueError(f"{current.path}: invalid interrupt-parent")
            parent_handle = handles[0]
            break
        current = current.parent
    provider = fdt.phandles.get(parent_handle or 0)
    if provider is None:
        raise ValueError(f"{node.path}: missing interrupt parent")
    count_values = cells(provider.properties.get("#interrupt-cells"))
    if len(count_values) != 1 or count_values[0] == 0:
        raise ValueError(f"{provider.path}: invalid #interrupt-cells")
    width = count_values[0]
    values = cells(raw)
    if len(values) % width:
        raise ValueError(f"{node.path}: invalid interrupts")
    return [Interrupt(provider.path, tuple(values[at:at + width]))
            for at in range(0, len(values), width)]


def negative_dtb(fdt: Fdt, output: Path) -> Node:
    candidates = [node for node in fdt.nodes
                  if enabled(node) and node.parent is not None and compatible(node)]
    if not candidates:
        raise ValueError("board has no enabled non-root compatible to mutate")
    node = candidates[0]
    raw = node.properties["compatible"]
    mutated = bytes(ord("x") if value else 0 for value in raw)
    data = bytearray(fdt.data)
    at = node.property_offsets["compatible"]
    data[at:at + len(mutated)] = mutated
    output.write_bytes(data)
    return node


def is_spi_controller(node: Node) -> bool:
    return enabled(node) and any(
        item.startswith("st,stm32") and "spi" in item for item in compatible(node)
    )


def is_i2c_controller(node: Node) -> bool:
    return enabled(node) and any(
        item.startswith("st,stm32") and "i2c" in item for item in compatible(node)
    )


def is_uart_controller(node: Node) -> bool:
    return enabled(node) and bool(
        {"st,stm32-usart", "st,stm32-uart"}.intersection(compatible(node))
    )


def decode_specifiers(fdt: Fdt, node: Node, property_name: str,
                      cell_property: str) -> list[Specifier]:
    raw = node.properties.get(property_name)
    if raw is None:
        return []
    values = cells(raw)
    result: list[Specifier] = []
    index = 0
    while index < len(values):
        phandle = values[index]
        index += 1
        provider = fdt.phandles.get(phandle)
        if provider is None:
            raise ValueError(f"{node.path}: {property_name} has unknown phandle {phandle}")
        count_values = cells(provider.properties.get(cell_property))
        if len(count_values) != 1:
            raise ValueError(f"{provider.path}: missing {cell_property}")
        count = count_values[0]
        if index + count > len(values):
            raise ValueError(f"{node.path}: truncated {property_name} specifier")
        provider_compatible = compatible(provider)
        result.append(Specifier(
            provider_path=provider.path,
            provider_compatible=provider_compatible[0] if provider_compatible else "",
            cells=tuple(values[index:index + count]),
        ))
        index += count
    return result


def find_spi(fdt: Fdt) -> list[SpiController]:
    result: list[SpiController] = []
    for node in fdt.nodes:
        if not is_spi_controller(node):
            continue
        gpios = decode_specifiers(fdt, node, "cs-gpios", "#gpio-cells")
        children: list[SpiChild] = []
        for child in node.children:
            if not enabled(child) or "reg" not in child.properties:
                continue
            reg = cells(child.properties["reg"])
            if len(reg) != 1:
                raise ValueError(f"{child.path}: SPI reg must be one cell")
            chip_select = reg[0]
            gpio = gpios[chip_select] if chip_select < len(gpios) else None
            children.append(SpiChild(child, chip_select, gpio))
        result.append(SpiController(
            node=node,
            children=children,
            dmas=decode_specifiers(fdt, node, "dmas", "#dma-cells"),
            dma_names=strings(node.properties.get("dma-names")),
        ))
    return result


def find_i2c(fdt: Fdt) -> list[I2cController]:
    result: list[I2cController] = []
    for node in fdt.nodes:
        if not is_i2c_controller(node):
            continue
        children: list[I2cChild] = []
        for child in node.children:
            if not enabled(child) or "reg" not in child.properties:
                continue
            reg = cells(child.properties["reg"])
            if len(reg) != 1 or reg[0] > 0x7f:
                raise ValueError(f"{child.path}: I2C reg must be one 7-bit address cell")
            children.append(I2cChild(child, reg[0]))
        result.append(I2cController(node=node, children=children))
    return result


def find_uart(fdt: Fdt) -> list[UartController]:
    result: list[UartController] = []
    for node in fdt.nodes:
        if not is_uart_controller(node):
            continue
        children = [UartChild(child) for child in node.children if enabled(child)]
        result.append(UartController(
            node=node,
            children=children,
            dmas=decode_specifiers(fdt, node, "dmas", "#dma-cells"),
            dma_names=strings(node.properties.get("dma-names")),
        ))
    return result


def find_dma_controllers(fdt: Fdt) -> list[DmaController]:
    providers = [node for node in fdt.nodes
                 if enabled(node) and "#dma-cells" in node.properties]
    references: dict[str, list[DmaReference]] = {node.path: [] for node in providers}
    for consumer in fdt.nodes:
        if not enabled(consumer) or "dmas" not in consumer.properties:
            continue
        specifiers = decode_specifiers(fdt, consumer, "dmas", "#dma-cells")
        names = strings(consumer.properties.get("dma-names"))
        for index, specifier in enumerate(specifiers):
            if specifier.provider_path in references:
                references[specifier.provider_path].append(DmaReference(
                    consumer.path, specifier, index, dma_name_at(names, index)))
    return [DmaController(node, references[node.path]) for node in providers]


def find_sd_cards(fdt: Fdt) -> list[SdCard]:
    result = []
    for node in fdt.nodes:
        if not enabled(node) or "zephyr,sdmmc-disk" not in compatible(node):
            continue
        if node.parent is None or "st,stm32-sdmmc" not in compatible(node.parent):
            raise ValueError(f"{node.path}: SD card is not below an STM32 SDMMC controller")
        result.append(SdCard(node, node.parent))
    return result


def load_test_specs(paths: list[Path]) -> dict[str, str]:
    """Read the small documented YAML subset without a YAML dependency."""
    result: dict[str, str] = {}
    for path in paths:
        applies_to: str | None = None
        for line in path.read_text(encoding="utf-8").splitlines():
            match = re.match(r"^  applies_to:\s*(\S+)\s*$", line)
            if match:
                applies_to = match.group(1)
                continue
            match = re.match(r"^\s+- id:\s*(ADV-[A-Z0-9-]+-\d+\.T\d+)\s*$", line)
            if match:
                if applies_to is None:
                    raise ValueError(f"{path}: test before applies_to")
                if match.group(1) in result:
                    raise ValueError(f"{path}: duplicate test specification {match.group(1)}")
                result[match.group(1)] = applies_to
    required = {
        "ADV-SPI-001.T1": "spi-child",
        "ADV-SPI-002.T1": "spi-controller",
        "ADV-SPI-006.T1": "spi-controller-with-dmas",
        "ADV-I2C-001.T1": "i2c-child",
        "ADV-I2C-002.T1": "i2c-controller",
        "ADV-UART-001.T1": "uart-child",
        "ADV-UART-002.T1": "uart-controller",
        "ADV-UART-006.T1": "uart-controller-with-dmas",
        "ADV-DT-001.T1": "dt-negative",
        "ADV-DT-002.T1": "dt-node",
        "ADV-DT-003.T1": "dt-mmio-node",
        "ADV-DT-004.T1": "dt-irq-node",
        "ADV-DT-005.T1": "dt-board",
        "ADV-DMA-005.T1": "dma-controller",
        "ADV-SD-004.T1": "sd-card",
    }
    for test_id, target in required.items():
        if result.get(test_id) != target:
            raise ValueError(f"spec set: {test_id} must apply to {target}")
    return result


def specifier_json(value: Specifier) -> dict[str, object]:
    return {
        "provider_path": value.provider_path,
        "provider_compatible": value.provider_compatible,
        "cells": list(value.cells),
    }


def case_name(board: str, path: str, test_id: str) -> str:
    path_name = path.strip("/").replace("/", "__") or "root"
    return f"{board}__{path_name}__{test_id}"


def dma_name_at(names: list[str], index: int) -> str:
    """Return the same-index dma-names entry, or the ABI's unnamed value."""
    return names[index] if index < len(names) else ""


def gpio_check(binding_index: int, gpio: Specifier | None) -> str:
    if gpio is None:
        return f"CHECK(!target.bindings[{binding_index}].has_gpio);"
    checks = [
        f"CHECK(target.bindings[{binding_index}].has_gpio);",
        f"CHECK(strcmp(target.bindings[{binding_index}].gpio_provider, {c_string(gpio.provider_path)}) == 0);",
        f"CHECK(target.bindings[{binding_index}].gpio_cell_count == {len(gpio.cells)}U);",
    ]
    for index, value in enumerate(gpio.cells):
        checks.append(
            f"CHECK(target.bindings[{binding_index}].gpio_cells[{index}] == {value}U);"
        )
    return "\n\t".join(checks)


def generated_spi_cases(board: str, controllers: list[SpiController]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for controller in controllers:
        inventory_name = case_name(board, controller.node.path, "ADV-SPI-002.T1")
        result.append({
            "name": inventory_name,
            "test_spec_id": "ADV-SPI-002.T1",
            "item_id": "ADV-SPI-002",
            "node_path": controller.node.path,
            "derived_properties": {
                "child_count": len(controller.children),
                "reg": [child.chip_select for child in controller.children],
            },
            "kind": "inventory",
            "area": "spi",
            "controller": controller,
        })
        for child in controller.children:
            result.append({
                "name": case_name(board, child.node.path, "ADV-SPI-001.T1"),
                "test_spec_id": "ADV-SPI-001.T1",
                "item_id": "ADV-SPI-001",
                "node_path": child.node.path,
                "derived_properties": {
                    "reg": child.chip_select,
                    "cs_gpio": specifier_json(child.cs_gpio) if child.cs_gpio else None,
                },
                "kind": "child",
                "area": "spi",
                "controller": controller,
                "child": child,
            })
        if controller.dmas:
            result.append({
                "name": case_name(board, controller.node.path, "ADV-SPI-006.T1"),
                "test_spec_id": "ADV-SPI-006.T1",
                "item_id": "ADV-SPI-006",
                "node_path": controller.node.path,
                "derived_properties": {
                    "dmas": [specifier_json(dma) for dma in controller.dmas],
                    "dma-names": controller.dma_names,
                },
                "kind": "dma",
                "area": "spi",
                "controller": controller,
            })
    return result


def generated_i2c_cases(board: str, controllers: list[I2cController]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for controller in controllers:
        result.append({
            "name": case_name(board, controller.node.path, "ADV-I2C-002.T1"),
            "test_spec_id": "ADV-I2C-002.T1",
            "item_id": "ADV-I2C-002",
            "node_path": controller.node.path,
            "derived_properties": {
                "child_count": len(controller.children),
                "reg": [child.address for child in controller.children],
            },
            "kind": "i2c-inventory",
            "area": "i2c",
            "controller": controller,
        })
        for child in controller.children:
            result.append({
                "name": case_name(board, child.node.path, "ADV-I2C-001.T1"),
                "test_spec_id": "ADV-I2C-001.T1",
                "item_id": "ADV-I2C-001",
                "node_path": child.node.path,
                "derived_properties": {"reg": child.address},
                "kind": "i2c-child",
                "area": "i2c",
                "controller": controller,
                "child": child,
            })
    return result


def generated_uart_cases(board: str, controllers: list[UartController]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for controller in controllers:
        result.append({
            "name": case_name(board, controller.node.path, "ADV-UART-002.T1"),
            "test_spec_id": "ADV-UART-002.T1",
            "item_id": "ADV-UART-002",
            "node_path": controller.node.path,
            "derived_properties": {
                "child_count": len(controller.children),
                "children": [child.node.path for child in controller.children],
            },
            "kind": "uart-inventory",
            "area": "uart",
            "controller": controller,
        })
        for child in controller.children:
            result.append({
                "name": case_name(board, child.node.path, "ADV-UART-001.T1"),
                "test_spec_id": "ADV-UART-001.T1",
                "item_id": "ADV-UART-001",
                "node_path": child.node.path,
                "derived_properties": {"controller": controller.node.path},
                "kind": "uart-child",
                "area": "uart",
                "controller": controller,
                "child": child,
            })
        if controller.dmas:
            result.append({
                "name": case_name(board, controller.node.path, "ADV-UART-006.T1"),
                "test_spec_id": "ADV-UART-006.T1",
                "item_id": "ADV-UART-006",
                "node_path": controller.node.path,
                "derived_properties": {
                    "dmas": [specifier_json(dma) for dma in controller.dmas],
                    "dma-names": controller.dma_names,
                },
                "kind": "uart-dma",
                "area": "uart",
                "controller": controller,
            })
    return result


def generated_dt_cases(board: str, fdt: Fdt, negative_path: Path,
                       negative_node: Node) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    instances = [node for node in fdt.nodes if is_model_instance(node)]
    expected_windows = [window for node in fdt.nodes
                        if enabled(node) and MMIO_MODEL_COMPATIBLES.intersection(compatible(node))
                        for window in reg_windows(node)]
    result.append({
        "name": case_name(board, negative_node.path, "ADV-DT-001.T1"),
        "test_spec_id": "ADV-DT-001.T1", "item_id": "ADV-DT-001",
        "node_path": negative_node.path,
        "derived_properties": {"negative_dtb": str(negative_path),
                               "mutated_property": "compatible"},
        "kind": "dt-negative", "area": "dt", "negative_path": negative_path,
        "negative_node": negative_node,
    })
    result.append({
        "name": case_name(board, "/", "ADV-DT-002.T1"),
        "test_spec_id": "ADV-DT-002.T1", "item_id": "ADV-DT-002",
        "node_path": "/",
        "derived_properties": {"enabled_compatible_nodes": [node.path for node in instances]},
        "kind": "dt-inventory", "area": "dt", "instances": instances,
    })
    for node in fdt.nodes:
        if not enabled(node) or not MMIO_MODEL_COMPATIBLES.intersection(compatible(node)):
            continue
        windows = reg_windows(node)
        if not windows:
            continue
        result.append({
            "name": case_name(board, node.path, "ADV-DT-003.T1"),
            "test_spec_id": "ADV-DT-003.T1", "item_id": "ADV-DT-003",
            "node_path": node.path,
            "derived_properties": {"reg": [dataclasses.asdict(value) for value in windows]},
            "kind": "dt-mmio", "area": "dt", "node": node,
            "windows": windows, "all_windows": expected_windows,
            "will_fail": True,
            "known_defect": "EXTI registers an extra SYSCFG window absent from its reg property",
        })
    for node in fdt.nodes:
        if not is_model_instance(node):
            continue
        interrupts = node_interrupts(fdt, node)
        if not interrupts:
            continue
        result.append({
            "name": case_name(board, node.path, "ADV-DT-004.T1"),
            "test_spec_id": "ADV-DT-004.T1", "item_id": "ADV-DT-004",
            "node_path": node.path,
            "derived_properties": {"interrupts": [dataclasses.asdict(value) for value in interrupts]},
            "kind": "dt-irq", "area": "dt", "node": node,
            "interrupts": interrupts,
            "will_fail": not IRQ_RESOLVING_COMPATIBLES.intersection(compatible(node)),
            "known_defect": (None if IRQ_RESOLVING_COMPATIBLES.intersection(compatible(node)) else
                             "instantiated adapter does not resolve its interrupts entries"),
        })
    result.append({
        "name": case_name(board, "/", "ADV-DT-005.T1"),
        "test_spec_id": "ADV-DT-005.T1", "item_id": "ADV-DT-005",
        "node_path": "/", "derived_properties": {"dtb_size": len(fdt.data)},
        "kind": "dt-determinism", "area": "dt",
    })
    return result


def generated_dma_cases(board: str, controllers: list[DmaController]) -> list[dict[str, object]]:
    result = []
    for controller in controllers:
        result.append({
            "name": case_name(board, controller.node.path, "ADV-DMA-005.T1"),
            "test_spec_id": "ADV-DMA-005.T1", "item_id": "ADV-DMA-005",
            "node_path": controller.node.path,
            "derived_properties": {
                "references": [{"consumer_path": ref.consumer_path,
                                "provider": specifier_json(ref.specifier),
                                "index": ref.index, "name": ref.name}
                               for ref in controller.references],
            },
            "kind": "dma-controller", "area": "dma", "controller": controller,
        })
    return result


def generated_sd_cases(board: str, cards: list[SdCard]) -> list[dict[str, object]]:
    return [{
        "name": case_name(board, card.node.path, "ADV-SD-004.T1"),
        "test_spec_id": "ADV-SD-004.T1", "item_id": "ADV-SD-004",
        "node_path": card.node.path,
        "derived_properties": {"controller": card.controller.path},
        "kind": "sd-card", "area": "sd", "card": card,
    } for card in cards]


def c_windows(values: list[MmioWindow]) -> str:
    return ", ".join(f"{{{value.address}ULL, {value.size}ULL}}" for value in values)


def c_interrupt(value: Interrupt) -> tuple[str, int]:
    wanted = ", ".join(f"{cell}U" for cell in value.cells) or "0U"
    return wanted, len(value.cells)


def render_case(case: dict[str, object]) -> str:
    name = str(case["name"])
    controller = case.get("controller")
    prebuild = case["kind"] not in ("dt-negative", "dt-determinism")
    lines = [f"static int case_{c_identifier(name)}(void)", "{", "\tuint32 index;"]
    if prebuild:
        lines.append("\tCHECK(build_board() == 0);")
    if case["kind"] == "child":
        assert isinstance(controller, SpiController)
        child = case["child"]
        assert isinstance(child, SpiChild)
        lines.extend([
            "\tindex = find_binding(" + c_string(controller.node.path) +
            f", {child.chip_select}U);",
            "\tCHECK(index < target.binding_count);",
            "\t" + gpio_check(0, child.cs_gpio).replace("target.bindings[0]", "target.bindings[index]"),
        ])
    elif case["kind"] == "inventory":
        assert isinstance(controller, SpiController)
        lines.append(f"\tCHECK(count_bindings({c_string(controller.node.path)}) == {len(controller.children)}U);")
        for child in controller.children:
            lines.append(
                f"\tCHECK(find_binding({c_string(controller.node.path)}, {child.chip_select}U) < target.binding_count);"
            )
    elif case["kind"] == "dma":
        assert isinstance(controller, SpiController)
        for index, dma in enumerate(controller.dmas):
            cell_values = ", ".join(f"{value}U" for value in dma.cells) or "0U"
            lines.extend([
                f"\t{{ static const uint32 wanted[] = {{{cell_values}}};",
                f"\t  CHECK(find_dma({c_string(dma.provider_path)}, wanted, {len(dma.cells)}U,",
                f"\t                 {index}U, {c_string(dma_name_at(controller.dma_names, index))})); }}",
            ])
    elif case["kind"] == "i2c-child":
        assert isinstance(controller, I2cController)
        child = case["child"]
        assert isinstance(child, I2cChild)
        lines.extend([
            "\tindex = find_i2c_binding(" + c_string(controller.node.path) +
            f", {child.address}U);",
            "\tCHECK(index < target.i2c_binding_count);",
        ])
    elif case["kind"] == "i2c-inventory":
        assert isinstance(controller, I2cController)
        lines.append(
            f"\tCHECK(count_i2c_bindings({c_string(controller.node.path)}) == {len(controller.children)}U);"
        )
        for child in controller.children:
            lines.append(
                f"\tCHECK(find_i2c_binding({c_string(controller.node.path)}, {child.address}U) < target.i2c_binding_count);"
            )
    elif case["kind"] == "uart-child":
        assert isinstance(controller, UartController)
        child = case["child"]
        assert isinstance(child, UartChild)
        lines.extend([
            "\tindex = find_uart_binding(" + c_string(controller.node.path) +
            f", {c_string(child.node.path)});",
            "\tCHECK(index < target.uart_binding_count);",
            f"\tCHECK(count_uart_peripheral({c_string(child.node.path)}) == 1U);",
        ])
    elif case["kind"] == "uart-inventory":
        assert isinstance(controller, UartController)
        lines.append(
            f"\tCHECK(count_uart_bindings({c_string(controller.node.path)}) == {len(controller.children)}U);"
        )
        lines.append(f"\tCHECK({len(controller.children)}U <= 1U);")
        for child in controller.children:
            lines.append(
                f"\tCHECK(find_uart_binding({c_string(controller.node.path)}, {c_string(child.node.path)}) < target.uart_binding_count);"
            )
    elif case["kind"] == "uart-dma":
        assert isinstance(controller, UartController)
        for index, dma in enumerate(controller.dmas):
            cell_values = ", ".join(f"{value}U" for value in dma.cells) or "0U"
            lines.extend([
                f"\t{{ static const uint32 wanted[] = {{{cell_values}}};",
                f"\t  CHECK(find_dma({c_string(dma.provider_path)}, wanted, {len(dma.cells)}U,",
                f"\t                 {index}U, {c_string(dma_name_at(controller.dma_names, index))})); }}",
            ])
    elif case["kind"] == "dt-negative":
        node = case["negative_node"]
        assert isinstance(node, Node)
        lines.extend([
            "\tCapture output = {0};",
            f"\tCHECK(capture_build({c_string(str(case['negative_path']))}, &output) != 0);",
            f"\tCHECK(strstr(output.out, {c_string('node=' + node.path)}) != NULL ||",
            f"\t      strstr(output.err, {c_string('node=' + node.path)}) != NULL);",
            "\tfree_capture(&output);",
        ])
    elif case["kind"] == "dt-inventory":
        instances = case["instances"]
        assert isinstance(instances, list)
        lines.append(f"\tCHECK(count_lines(captured_stderr, \"DEVICE_MODEL path=\") == {len(instances)}U);")
        for node in instances:
            assert isinstance(node, Node)
            lines.append(
                f"\tCHECK(count_model_path(captured_stderr, {c_string(node.path)}) == 1U);"
            )
    elif case["kind"] == "dt-mmio":
        all_windows = case["all_windows"]
        assert isinstance(all_windows, list)
        lines.extend([
            f"\t{{ static const ExpectedWindow expected[] = {{{c_windows(all_windows)}}};",
            "\t  CHECK(mmio_set_matches(expected, sizeof(expected) / sizeof(expected[0])));",
            "\t  CHECK(mmio_pairwise_disjoint()); }",
        ])
    elif case["kind"] == "dt-irq":
        interrupts = case["interrupts"]
        assert isinstance(interrupts, list)
        for interrupt in interrupts:
            assert isinstance(interrupt, Interrupt)
            wanted, count = c_interrupt(interrupt)
            lines.extend([
                f"\t{{ static const uint32 wanted[] = {{{wanted}}};",
                f"\t  CHECK(find_irq({c_string(interrupt.provider_path)}, wanted, {count}U)); }}",
            ])
    elif case["kind"] == "dt-determinism":
        lines.extend([
            "\tCapture first = {0}, second = {0};",
            "\tCHECK(capture_build(board_dtb_path, &first) == 0);",
            "\tdestroy_board();",
            "\tCHECK(capture_build(board_dtb_path, &second) == 0);",
            "\tCHECK(strcmp(first.out, second.out) == 0);",
            "\tCHECK(strcmp(first.err, second.err) == 0);",
            "\tCHECK(strstr(first.out, \"DEVICE_TREE sha256=\") != NULL);",
            "\tCHECK(strstr(first.err, \"DEVICE_MODEL path=\") != NULL);",
            "\tfree_capture(&first); free_capture(&second);",
        ])
    elif case["kind"] == "dma-controller":
        assert isinstance(controller, DmaController)
        lines.extend([
            f"\tCHECK(count_model_path(captured_stderr, {c_string(controller.node.path)}) == 1U);",
            f"\tCHECK(count_dma_provider({c_string(controller.node.path)}) == {len(controller.references)}U);",
        ])
        for reference in controller.references:
            dma = reference.specifier
            cell_values = ", ".join(f"{value}U" for value in dma.cells) or "0U"
            lines.extend([
                f"\t{{ static const uint32 wanted[] = {{{cell_values}}};",
                f"\t  CHECK(find_dma({c_string(dma.provider_path)}, wanted, {len(dma.cells)}U,",
                f"\t                 {reference.index}U, {c_string(reference.name)})); }}",
            ])
    elif case["kind"] == "sd-card":
        card = case["card"]
        assert isinstance(card, SdCard)
        lines.extend([
            f"\tCHECK(find_adapter({c_string(card.controller.path)}, \"stm32-sdmmc\"));",
            f"\tCHECK(find_adapter({c_string(card.node.path)}, \"sdcard\"));",
        ])
    else:
        raise ValueError(f"unknown generated case kind {case['kind']}")
    lines.extend(["\tdestroy_board();", "\treturn 0;", "}"])
    return "\n".join(lines)


def render_source(dtb: Path, fdt: Fdt, cases: list[dict[str, object]],
                  use_plugin: bool) -> str:
    compatibles = sorted({item for node in fdt.nodes if enabled(node) for item in compatible(node)})
    compatibility_lines = "\n".join(f"\t{c_string(item)}," for item in compatibles)
    case_functions = "\n\n".join(render_case(case) for case in cases)
    dispatch_lines = "\n".join(
        f"\tif (strcmp(argv[1], {c_string(str(case['name']))}) == 0) return case_{c_identifier(str(case['name']))}();"
        for case in cases
    )
    plugin_code = """
\tAthrillDevicePluginPath plugin = {"hakoniwa,sensor-device", plugin_path};
\tif (plugin_path != NULL) {
\t\toptions.plugins = &plugin;
\t\toptions.plugin_count = 1U;
\t}
""" if use_plugin else "\t(void)plugin_path;\n"
    return f'''/* Generated by tools/dt_testgen.py; do not edit. */
#include "athrill_device_if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHECK(condition) do {{ if (!(condition)) {{ \\
\tfprintf(stderr, "%s:%d: check failed: %s\\n", __FILE__, __LINE__, #condition); \\
\tdestroy_board(); return 1; }} }} while (0)
#define MAX_BINDINGS 32U
#define MAX_I2C_BINDINGS 32U
#define MAX_UART_BINDINGS 32U
#define MAX_DMAS 32U
#define MAX_MMIO 64U
#define MAX_IRQS 128U
#define MAX_ADAPTERS 64U

typedef struct {{
\tchar controller[192];
\tuint32 chip_select;
\tstd_bool has_gpio;
\tchar gpio_provider[192];
\tuint32 gpio_cell_count;
\tuint32 gpio_cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
}} Binding;

typedef struct {{
\tchar controller[192];
\tuint16 address;
}} I2cBinding;

typedef struct {{
\tchar controller[192];
\tchar peripheral[192];
}} UartBinding;

typedef struct {{
\tchar provider[192];
\tuint32 cell_count;
\tuint32 cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
\tuint32 tuple_index;
\tchar name[192];
}} DmaResolution;

typedef struct {{ uint64 address; uint64 size; }} ExpectedWindow;
typedef struct {{ uint64 address; uint64 size; }} MmioResolution;
typedef struct {{
\tchar provider[192];
\tuint32 cell_count;
\tuint32 cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
}} IrqResolution;
typedef struct {{ char *out; char *err; }} Capture;
typedef struct {{ char node[192]; char name[64]; }} AdapterConnection;

typedef struct {{
\tBinding bindings[MAX_BINDINGS];
\tuint32 binding_count;
\tI2cBinding i2c_bindings[MAX_I2C_BINDINGS];
\tuint32 i2c_binding_count;
\tUartBinding uart_bindings[MAX_UART_BINDINGS];
\tuint32 uart_binding_count;
\tDmaResolution dmas[MAX_DMAS];
\tuint32 dma_count;
\tMmioResolution mmio[MAX_MMIO];
\tuint32 mmio_count;
\tIrqResolution irqs[MAX_IRQS];
\tuint32 irq_count;
\tAdapterConnection adapters[MAX_ADAPTERS];
\tuint32 adapter_count;
\tuint64 next_handle;
}} TestTarget;

static TestTarget target;
static AthrillDeviceRuntime *runtime;
static const char *plugin_path;
static const char *const board_dtb_path = {c_string(str(dtb.resolve()))};
static char *captured_stderr;

static void copy_text(char *to, size_t size, const char *from)
{{
\tif (from == NULL) from = "";
\tsnprintf(to, size, "%s", from);
}}

static sint32 mmio_register(void *context, uint64 address, uint64 size,
\tconst AthrillDeviceMmioOps *ops, void *instance, AthrillDeviceMmioToken *token)
{{
\tTestTarget *value = context;
\t(void)ops; (void)instance;
\tif (value->mmio_count >= MAX_MMIO) return -1;
\tvalue->mmio[value->mmio_count].address = address;
\tvalue->mmio[value->mmio_count].size = size;
\tvalue->mmio_count++;
\ttoken->value = ++value->next_handle;
\treturn 0;
}}

static void mmio_unregister(void *context, AthrillDeviceMmioToken token)
{{ (void)context; (void)token; }}

static sint32 irq_resolve(void *context, const AthrillDeviceSpecifier *spec,
\tAthrillDeviceIrqLine *line)
{{
\tTestTarget *value = context;
\tIrqResolution *resolution;
\tuint32 index;
\tif (value->irq_count >= MAX_IRQS) return -1;
\tresolution = &value->irqs[value->irq_count++];
\tcopy_text(resolution->provider, sizeof(resolution->provider), spec->provider_path);
\tresolution->cell_count = spec->cell_count;
\tfor (index = 0U; index < spec->cell_count; index++) resolution->cells[index] = spec->cells[index];
\tline->value = ++value->next_handle;
\treturn 0;
}}
static sint32 irq_set(void *context, AthrillDeviceIrqLine line, std_bool asserted)
{{ (void)context; (void)line; (void)asserted; return 0; }}
static std_bool irq_is_active(void *context, AthrillDeviceIrqLine line)
{{ (void)context; (void)line; return FALSE; }}
static uint64 time_now(void *context) {{ (void)context; return 0U; }}
static uint64 time_frequency(void *context) {{ (void)context; return 480000000U; }}
static sint32 clock_frequency(void *context, AthrillDeviceNodeId node,
\tconst char *name, uint64 *frequency)
{{ (void)context; (void)node; (void)name; *frequency = 240000000U; return 0; }}
static sint32 memory_read(void *context, uint64 address, void *data, uint32 size)
{{ (void)context; (void)address; memset(data, 0, size); return 0; }}
static sint32 memory_write(void *context, uint64 address, const void *data, uint32 size)
{{ (void)context; (void)address; (void)data; (void)size; return 0; }}

static sint32 spi_bind_node(void *context, const AthrillDeviceNode *controller,
\tuint32 chip_select, const AthrillDeviceSpecifier *gpio,
\tconst AthrillDeviceSpiPeripheralOps *ops, void *instance,
\tAthrillDeviceSpiEndpoint *endpoint)
{{
\tTestTarget *value = context;
\tBinding *binding;
\tuint32 index;
\t(void)instance;
\tif (controller == NULL || ops == NULL || value->binding_count >= MAX_BINDINGS) return -1;
\tbinding = &value->bindings[value->binding_count++];
\tcopy_text(binding->controller, sizeof(binding->controller), controller->path);
\tbinding->chip_select = chip_select;
\tif (gpio != NULL) {{
\t\tbinding->has_gpio = TRUE;
\t\tcopy_text(binding->gpio_provider, sizeof(binding->gpio_provider), gpio->provider_path);
\t\tbinding->gpio_cell_count = gpio->cell_count;
\t\tfor (index = 0; index < gpio->cell_count; index++) binding->gpio_cells[index] = gpio->cells[index];
\t}}
\tendpoint->value = ++value->next_handle;
\treturn 0;
}}

static sint32 spi_bind(void *context, AthrillDeviceNodeId controller,
\tuint32 chip_select, const AthrillDeviceSpecifier *gpio,
\tconst AthrillDeviceSpiPeripheralOps *ops, void *instance,
\tAthrillDeviceSpiEndpoint *endpoint)
{{ (void)context; (void)controller; (void)chip_select; (void)gpio; (void)ops; (void)instance; (void)endpoint; return -1; }}
static sint32 spi_set_cs(void *context, AthrillDeviceSpiEndpoint endpoint, std_bool asserted)
{{ (void)context; (void)endpoint; (void)asserted; return 0; }}
static sint32 spi_exchange(void *context, AthrillDeviceSpiEndpoint endpoint, uint8 mosi, uint8 *miso)
{{ (void)context; (void)endpoint; *miso = mosi; return 0; }}

static sint32 i2c_bind_node(void *context, const AthrillDeviceNode *controller,
\tuint16 address, const AthrillDeviceI2cPeripheralOps *ops, void *instance,
\tAthrillDeviceI2cEndpoint *endpoint)
{{
\tTestTarget *value = context;
\tI2cBinding *binding;
\t(void)instance;
\tif (controller == NULL || ops == NULL ||
\t    value->i2c_binding_count >= MAX_I2C_BINDINGS) return -1;
\tbinding = &value->i2c_bindings[value->i2c_binding_count++];
\tcopy_text(binding->controller, sizeof(binding->controller), controller->path);
\tbinding->address = address;
\tendpoint->value = ++value->next_handle;
\treturn 0;
}}
static sint32 i2c_transfer(void *context, AthrillDeviceI2cEndpoint endpoint,
\tAthrillDeviceI2cMessage *messages, uint32 count)
{{ (void)context; (void)endpoint; (void)messages; (void)count; return 0; }}
static sint32 uart_bind_nodes(void *context, const AthrillDeviceNode *controller,
\tconst AthrillDeviceNode *peripheral, const AthrillDeviceUartPeripheralOps *ops,
\tvoid *instance, AthrillDeviceUartEndpoint *endpoint)
{{
\tTestTarget *value = context;
\tUartBinding *binding;
\t(void)instance;
\tif (controller == NULL || peripheral == NULL || ops == NULL ||
\t    value->uart_binding_count >= MAX_UART_BINDINGS) return -1;
\tbinding = &value->uart_bindings[value->uart_binding_count++];
\tcopy_text(binding->controller, sizeof(binding->controller), controller->path);
\tcopy_text(binding->peripheral, sizeof(binding->peripheral), peripheral->path);
\tendpoint->value = ++value->next_handle;
\treturn 0;
}}
static sint32 uart_send(void *context, AthrillDeviceNodeId peripheral, uint8 data)
{{ (void)context; (void)peripheral; (void)data; return 0; }}

static sint32 gpio_resolve(void *context, const AthrillDeviceSpecifier *spec,
\tAthrillDeviceGpioLine *line)
{{ (void)spec; line->value = ++((TestTarget *)context)->next_handle; return 0; }}
static sint32 gpio_read(void *context, AthrillDeviceGpioLine line, std_bool *level)
{{ (void)context; (void)line; *level = FALSE; return 0; }}
static sint32 gpio_drive(void *context, AthrillDeviceGpioLine line, std_bool level)
{{ (void)context; (void)line; (void)level; return 0; }}
static sint32 gpio_subscribe(void *context, AthrillDeviceGpioLine line,
\tAthrillDeviceGpioEdge callback, void *instance)
{{ (void)context; (void)line; (void)callback; (void)instance; return 0; }}
static sint32 gpio_outputs(void *context, const AthrillDeviceGpioLine *lines,
\tuint32 count, uint32 levels)
{{ (void)context; (void)lines; (void)count; (void)levels; return 0; }}

static sint32 dma_resolve(void *context, const AthrillDeviceSpecifier *spec,
\tAthrillDeviceDmaChannel *channel)
{{
\tTestTarget *value = context;
\tDmaResolution *resolution;
\tuint32 index;
\tif (value->dma_count >= MAX_DMAS) return -1;
\tresolution = &value->dmas[value->dma_count++];
\tcopy_text(resolution->provider, sizeof(resolution->provider), spec->provider_path);
\tresolution->cell_count = spec->cell_count;
\tfor (index = 0; index < spec->cell_count; index++) resolution->cells[index] = spec->cells[index];
\tresolution->tuple_index = spec->index;
\tcopy_text(resolution->name, sizeof(resolution->name), spec->name);
\tchannel->value = ++value->next_handle;
\treturn 0;
}}
static sint32 dma_request(void *context, AthrillDeviceDmaChannel channel, std_bool asserted)
{{ (void)context; (void)channel; (void)asserted; return 0; }}
static sint32 adapter_connect(void *context, const AthrillDeviceNode *node, const char *name)
{{
\tTestTarget *value = context;
\tAdapterConnection *connection;
\tif (node == NULL || name == NULL || value->adapter_count >= MAX_ADAPTERS) return -1;
\tconnection = &value->adapters[value->adapter_count++];
\tcopy_text(connection->node, sizeof(connection->node), node->path);
\tcopy_text(connection->name, sizeof(connection->name), name);
\treturn 0;
}}
static sint32 sensor_sample(void *context, AthrillDeviceSensorSample *sample)
{{ (void)context; memset(sample, 0, sizeof(*sample)); return 0; }}
static sint32 gps_sample(void *context, AthrillDeviceGpsSample *sample)
{{ (void)context; memset(sample, 0, sizeof(*sample)); return 0; }}
static sint32 actuator_output(void *context, const float32 *values, uint32 count)
{{ (void)context; (void)values; (void)count; return 0; }}

static const AthrillDeviceTargetOps target_ops = {{
\t.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
\t.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
\t.struct_size = sizeof(AthrillDeviceTargetOps),
\t.mmio_register = mmio_register, .mmio_unregister = mmio_unregister,
\t.irq_resolve = irq_resolve, .irq_set = irq_set, .irq_is_active = irq_is_active,
\t.time_now = time_now, .time_frequency_hz = time_frequency,
\t.clock_frequency_hz = clock_frequency,
\t.memory_read = memory_read, .memory_write = memory_write,
\t.spi_bind = spi_bind, .spi_bind_node = spi_bind_node,
\t.spi_set_chip_select = spi_set_cs, .spi_exchange_byte = spi_exchange,
\t.i2c_bind_node = i2c_bind_node, .i2c_transfer = i2c_transfer,
\t.uart_bind_nodes = uart_bind_nodes, .uart_send_from_node = uart_send,
\t.gpio_resolve = gpio_resolve, .gpio_read = gpio_read, .gpio_drive = gpio_drive,
\t.gpio_subscribe = gpio_subscribe, .gpio_outputs = gpio_outputs,
\t.dma_resolve = dma_resolve, .dma_request = dma_request,
\t.model_adapter_connect = adapter_connect,
\t.sensor_sample = sensor_sample, .gps_sample = gps_sample,
\t.actuator_output = actuator_output,
}};

static const char *const target_compatibles[] = {{
{compatibility_lines}
}};

static void destroy_board(void)
{{
\tif (runtime != NULL) athrill_device_destroy(runtime);
\truntime = NULL;
}}

static int raw_build_board(const char *dtb_path)
{{
\tAthrillDeviceBuildOptions options;
\tmemset(&target, 0, sizeof(target));
\tmemset(&options, 0, sizeof(options));
\toptions.dtb_path = dtb_path;
\toptions.target_compatibles = target_compatibles;
\toptions.target_compatible_count = sizeof(target_compatibles) / sizeof(target_compatibles[0]);
\toptions.maximum_dtb_size = 16U * 1024U * 1024U;
{plugin_code}
\treturn athrill_device_build(&options, &target_ops, &target, &runtime);
}}

static char *read_capture(FILE *stream)
{{
\tlong length;
\tchar *text;
\tif (fseek(stream, 0L, SEEK_END) != 0 || (length = ftell(stream)) < 0 ||
\t    fseek(stream, 0L, SEEK_SET) != 0) return NULL;
\ttext = malloc((size_t)length + 1U);
\tif (text == NULL) return NULL;
\tif (length != 0 && fread(text, 1U, (size_t)length, stream) != (size_t)length) {{
\t\tfree(text); return NULL;
\t}}
\ttext[length] = 0;
\treturn text;
}}

static int capture_build(const char *dtb_path, Capture *capture)
{{
\tFILE *out = tmpfile(), *err = tmpfile();
\tint saved_out, saved_err, result;
\tif (out == NULL || err == NULL) return -1;
\tfflush(NULL);
\tsaved_out = dup(STDOUT_FILENO); saved_err = dup(STDERR_FILENO);
\tif (saved_out < 0 || saved_err < 0 || dup2(fileno(out), STDOUT_FILENO) < 0 ||
\t    dup2(fileno(err), STDERR_FILENO) < 0) return -1;
\tresult = raw_build_board(dtb_path);
\tfflush(NULL);
\t(void)dup2(saved_out, STDOUT_FILENO); (void)dup2(saved_err, STDERR_FILENO);
\t(void)close(saved_out); (void)close(saved_err);
\tcapture->out = read_capture(out); capture->err = read_capture(err);
\tfclose(out); fclose(err);
\treturn capture->out != NULL && capture->err != NULL ? result : -1;
}}

static void free_capture(Capture *capture)
{{ free(capture->out); free(capture->err); capture->out = NULL; capture->err = NULL; }}

static int build_board(void)
{{
\tCapture capture = {{0}};
\tint result;
\tfree(captured_stderr); captured_stderr = NULL;
\tresult = capture_build(board_dtb_path, &capture);
\tfree(capture.out); captured_stderr = capture.err;
\treturn result;
}}

static uint32 count_lines(const char *text, const char *prefix)
{{
\tuint32 count = 0U;
\tconst char *at = text;
\twhile (at != NULL && (at = strstr(at, prefix)) != NULL) {{ count++; at += strlen(prefix); }}
\treturn count;
}}

static uint32 count_model_path(const char *text, const char *path)
{{
\tchar needle[512];
\t(void)snprintf(needle, sizeof(needle), "DEVICE_MODEL path=%s compatible=", path);
\treturn count_lines(text, needle);
}}

static int mmio_set_matches(const ExpectedWindow *expected, uint32 count)
{{
\tuint32 i, j, found;
\tif (target.mmio_count != count) return 0;
\tfor (i = 0U; i < count; i++) {{
\t\tfound = 0U;
\t\tfor (j = 0U; j < target.mmio_count; j++)
\t\t\tif (target.mmio[j].address == expected[i].address &&
\t\t\t    target.mmio[j].size == expected[i].size) found++;
\t\tif (found != 1U) return 0;
\t}}
\treturn 1;
}}

static int mmio_pairwise_disjoint(void)
{{
\tuint32 i, j;
\tfor (i = 0U; i < target.mmio_count; i++)
\t\tfor (j = 0U; j < i; j++)
\t\t\tif (target.mmio[i].address < target.mmio[j].address + target.mmio[j].size &&
\t\t\t    target.mmio[j].address < target.mmio[i].address + target.mmio[i].size) return 0;
\treturn 1;
}}

static int find_irq(const char *provider, const uint32 *wanted, uint32 count)
{{
\tuint32 index;
\tfor (index = 0U; index < target.irq_count; index++)
\t\tif (target.irqs[index].cell_count == count &&
\t\t    strcmp(target.irqs[index].provider, provider) == 0 &&
\t\t    memcmp(target.irqs[index].cells, wanted, count * sizeof(*wanted)) == 0) return 1;
\treturn 0;
}}

static uint32 count_bindings(const char *controller)
{{
\tuint32 index, count = 0;
\tfor (index = 0; index < target.binding_count; index++)
\t\tif (strcmp(target.bindings[index].controller, controller) == 0) count++;
\treturn count;
}}

static uint32 find_binding(const char *controller, uint32 chip_select)
{{
\tuint32 index;
\tfor (index = 0; index < target.binding_count; index++)
\t\tif (target.bindings[index].chip_select == chip_select &&
\t\t    strcmp(target.bindings[index].controller, controller) == 0) return index;
\treturn target.binding_count;
}}

static uint32 count_i2c_bindings(const char *controller)
{{
\tuint32 index, count = 0;
\tfor (index = 0; index < target.i2c_binding_count; index++)
\t\tif (strcmp(target.i2c_bindings[index].controller, controller) == 0) count++;
\treturn count;
}}

static uint32 find_i2c_binding(const char *controller, uint16 address)
{{
\tuint32 index;
\tfor (index = 0; index < target.i2c_binding_count; index++)
\t\tif (target.i2c_bindings[index].address == address &&
\t\t    strcmp(target.i2c_bindings[index].controller, controller) == 0) return index;
\treturn target.i2c_binding_count;
}}

static uint32 count_uart_bindings(const char *controller)
{{
\tuint32 index, count = 0;
\tfor (index = 0; index < target.uart_binding_count; index++)
\t\tif (strcmp(target.uart_bindings[index].controller, controller) == 0) count++;
\treturn count;
}}

static uint32 count_uart_peripheral(const char *peripheral)
{{
\tuint32 index, count = 0;
\tfor (index = 0; index < target.uart_binding_count; index++)
\t\tif (strcmp(target.uart_bindings[index].peripheral, peripheral) == 0) count++;
\treturn count;
}}

static uint32 find_uart_binding(const char *controller, const char *peripheral)
{{
\tuint32 index;
\tfor (index = 0; index < target.uart_binding_count; index++)
\t\tif (strcmp(target.uart_bindings[index].controller, controller) == 0 &&
\t\t    strcmp(target.uart_bindings[index].peripheral, peripheral) == 0) return index;
\treturn target.uart_binding_count;
}}

static int dma_matches(uint32 index, const char *provider,
\tconst uint32 *wanted, uint32 count, uint32 tuple_index, const char *name)
{{
\treturn index < target.dma_count && target.dmas[index].cell_count == count &&
\t\tstrcmp(target.dmas[index].provider, provider) == 0 &&
\t\tmemcmp(target.dmas[index].cells, wanted, count * sizeof(*wanted)) == 0 &&
\t\ttarget.dmas[index].tuple_index == tuple_index &&
\t\tstrcmp(target.dmas[index].name, name) == 0;
}}

static int find_dma(const char *provider, const uint32 *wanted, uint32 count,
\tuint32 tuple_index, const char *name)
{{
\tuint32 index;
\tfor (index = 0; index < target.dma_count; index++)
\t\tif (dma_matches(index, provider, wanted, count, tuple_index, name)) return 1;
\treturn 0;
}}

static uint32 count_dma_provider(const char *provider)
{{
\tuint32 index, count = 0U;
\tfor (index = 0U; index < target.dma_count; index++)
\t\tif (strcmp(target.dmas[index].provider, provider) == 0) count++;
\treturn count;
}}

static int find_adapter(const char *node, const char *name)
{{
\tuint32 index;
\tfor (index = 0U; index < target.adapter_count; index++)
\t\tif (strcmp(target.adapters[index].node, node) == 0 &&
\t\t    strcmp(target.adapters[index].name, name) == 0) return 1;
\treturn 0;
}}

{case_functions}

int main(int argc, char **argv)
{{
\tif (argc < 2) {{ fprintf(stderr, "usage: %s CASE [PLUGIN]\\n", argv[0]); return 2; }}
\tplugin_path = argc > 2 ? argv[2] : NULL;
{dispatch_lines}
\tfprintf(stderr, "unknown generated trace case: %s\\n", argv[1]);
\treturn 2;
}}
'''


def render_cmake(targets: list[dict[str, object]]) -> str:
    lines = ["# Generated by tools/dt_testgen.py; do not edit."]
    for target in targets:
        executable = str(target["target"])
        source = str(target["source"])
        lines.extend([
            f"add_executable({executable} {json.dumps(source)})",
            f"target_compile_definitions({executable} PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)",
            f"target_link_libraries({executable} PRIVATE athrill_device)",
        ])
        if target["use_plugin"]:
            lines.append(f"add_dependencies({executable} athrill_hakoniwa_sensor)")
        for case in target["cases"]:
            name = str(case["name"])
            command = f"add_test(NAME {json.dumps(name)} COMMAND {executable} {json.dumps(name)}"
            if target["use_plugin"]:
                command += " $<TARGET_FILE:athrill_hakoniwa_sensor>"
            command += ")"
            lines.extend([
                command,
                f"set_tests_properties({json.dumps(name)} PROPERTIES LABELS \"traceability;{case['area']}\")",
            ])
            if case.get("will_fail"):
                lines.append(f"set_tests_properties({json.dumps(name)} PROPERTIES WILL_FAIL TRUE)")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", action="append", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("dtbs", nargs="+", type=Path)
    args = parser.parse_args()
    load_test_specs(args.spec)
    args.out.mkdir(parents=True, exist_ok=True)
    trace_map: dict[str, object] = {
        "schema_version": 1,
        "board_coverage": [],
        "generated_tests": [],
    }
    targets: list[dict[str, object]] = []
    for dtb in sorted(args.dtbs):
        fdt = Fdt(dtb)
        spi_controllers = find_spi(fdt)
        i2c_controllers = find_i2c(fdt)
        uart_controllers = find_uart(fdt)
        dma_controllers = find_dma_controllers(fdt)
        sd_cards = find_sd_cards(fdt)
        relative = dtb.as_posix().split("/boards/", 1)[-1]
        board = relative.replace("/", "__").removesuffix(".dtb")
        negative_path = args.out / f"{c_identifier(board)}__unknown-compatible.dtb"
        negative_node = negative_dtb(fdt, negative_path)
        cases = (generated_spi_cases(board, spi_controllers) +
                 generated_i2c_cases(board, i2c_controllers) +
                 generated_uart_cases(board, uart_controllers) +
                 generated_dt_cases(board, fdt, negative_path, negative_node) +
                 generated_dma_cases(board, dma_controllers) +
                 generated_sd_cases(board, sd_cards))
        dt_instances = [node for node in fdt.nodes if is_model_instance(node)]
        dt_mmio_nodes = [node for node in fdt.nodes
                         if enabled(node) and MMIO_MODEL_COMPATIBLES.intersection(compatible(node))
                         and reg_windows(node)]
        dt_irq_nodes = [node for node in fdt.nodes if is_model_instance(node) and node_interrupts(fdt, node)]
        trace_map["board_coverage"].append({
            "board": relative,
            "uart_controllers": len(uart_controllers),
            "uart_children": sum(len(value.children) for value in uart_controllers),
            "uart_controllers_with_dmas": sum(bool(value.dmas) for value in uart_controllers),
            "dt_negative_nodes": 1,
            "dt_nodes": len(dt_instances),
            "dt_mmio_nodes": len(dt_mmio_nodes),
            "dt_irq_nodes": len(dt_irq_nodes),
            "dt_boards": 1,
            "dma_controllers": len(dma_controllers),
            "dma_references": sum(len(value.references) for value in dma_controllers),
            "sd_cards": len(sd_cards),
        })
        use_plugin = "hakoniwa" in dtb.stem
        source = args.out / f"{c_identifier(board)}.c"
        source.write_text(render_source(dtb, fdt, cases, use_plugin), encoding="utf-8")
        target = "trace_bus_" + c_identifier(board)
        targets.append({
            "target": target,
            "source": str(source.resolve()),
            "use_plugin": use_plugin,
            "cases": cases,
        })
        for case in cases:
            trace_map["generated_tests"].append({
                "name": case["name"],
                "test_spec_id": case["test_spec_id"],
                "item_id": case["item_id"],
                "board": relative,
                "node_path": case["node_path"],
                "derived_properties": case["derived_properties"],
                "source_file": str(source.resolve()),
                "will_fail": bool(case.get("will_fail", False)),
                "known_defect": case.get("known_defect"),
            })
    (args.out / "trace-map.json").write_text(
        json.dumps(trace_map, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.out / "trace-tests.cmake").write_text(render_cmake(targets), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
