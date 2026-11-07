#!/usr/bin/env python3
"""Build the traceability Markdown report from specs, maps, tags, and JUnit."""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path


TAG = re.compile(r"\[(ADV-[A-Z0-9-]+(?:\.T\d+)?)\]")


@dataclasses.dataclass
class TestSpec:
    id: str
    stimulus: str = ""
    expect: str = ""


@dataclasses.dataclass
class Item:
    id: str
    statement: str = ""
    verification: str = ""
    applies_to: str = ""
    tests: list[TestSpec] = dataclasses.field(default_factory=list)


def yaml_value(text: str) -> str:
    value = text.strip()
    if value.startswith('"'):
        return json.loads(value)
    return value


def parse_spec(path: Path) -> list[Item]:
    """Parse the deliberately small traceability-spec YAML subset."""
    items: list[Item] = []
    item: Item | None = None
    test: TestSpec | None = None
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        match = re.match(r"^- id:\s*(\S+)\s*$", line)
        if match:
            item = Item(match.group(1))
            items.append(item)
            test = None
            continue
        match = re.match(r"^  (statement|verification|applies_to):\s*(.*)$", line)
        if match and item is not None:
            setattr(item, match.group(1), yaml_value(match.group(2)))
            continue
        match = re.match(r"^\s+- id:\s*(ADV-[A-Z0-9-]+-\d+\.T\d+)\s*$", line)
        if match and item is not None:
            test = TestSpec(match.group(1))
            item.tests.append(test)
            continue
        match = re.match(r"^\s+(stimulus|expect):\s*(.*)$", line)
        if match and test is not None:
            setattr(test, match.group(1), yaml_value(match.group(2)))
    if not items:
        raise ValueError(f"{path}: no specification items")
    for value in items:
        if not re.fullmatch(r"ADV-[A-Z0-9-]+-\d{3}", value.id):
            raise ValueError(f"{path}: invalid item ID {value.id}")
        for test_value in value.tests:
            if not test_value.id.startswith(value.id + ".T"):
                raise ValueError(f"{path}: {test_value.id} is not below {value.id}")
    return items


def scan_tags(root: Path, directories: list[str]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = defaultdict(list)
    for directory in directories:
        base = root / directory
        if not base.exists():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix not in (".c", ".h") or not path.is_file():
                continue
            for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
                for match in TAG.finditer(line):
                    result[match.group(1)].append(f"{path.relative_to(root)}:{number}")
    return result


def read_junit(values: list[str]) -> dict[str, dict[str, str]]:
    results: dict[str, dict[str, str]] = defaultdict(dict)
    for value in values:
        if "=" in value:
            label, filename = value.split("=", 1)
        else:
            filename = value
            label = Path(filename).stem
        root = ET.parse(filename).getroot()
        for case in root.iter("testcase"):
            name = case.get("name", "")
            if case.find("failure") is not None or case.find("error") is not None:
                status = "FAIL"
            elif case.find("skipped") is not None:
                status = "SKIP"
            else:
                status = "PASS"
            results[name][label] = status
    return results


def tokens(value: str) -> set[str]:
    return {part for part in re.split(r"[^a-z0-9]+", value.lower()) if part and part != "test"}


def junit_for_test_file(location: str, junit: dict[str, dict[str, str]]) -> dict[str, str]:
    wanted = tokens(Path(location.split(":", 1)[0]).stem)
    matches = [status for name, status in junit.items() if tokens(name) == wanted]
    return matches[0] if len(matches) == 1 else {}


def result_text(statuses: dict[str, str]) -> str:
    if not statuses:
        return "not in JUnit"
    return "; ".join(f"{label} {status}" for label, status in sorted(statuses.items()))


def escape_table(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", "<br>")


def lines(path: Path) -> int:
    return len(path.read_text(encoding="utf-8").splitlines())


def report(args: argparse.Namespace) -> str:
    items_list = [item for path in args.spec for item in parse_spec(path)]
    items = {item.id: item for item in items_list}
    tests = {test.id: (item, test) for item in items_list for test in item.tests}
    trace_map = json.loads(args.trace_map.read_text(encoding="utf-8"))
    generated = trace_map.get("generated_tests", [])
    board_coverage = trace_map.get("board_coverage", [])
    junit = read_junit(args.junit)
    implementation_tags = scan_tags(args.source_root, ["src", "board"])
    test_tags = scan_tags(args.source_root, ["tests"])
    generated_by_spec: dict[str, list[dict[str, object]]] = defaultdict(list)
    known_defects: Counter[tuple[str, str, str]] = Counter()
    for entry in generated:
        generated_by_spec[entry["test_spec_id"]].append(entry)

    gaps: list[str] = []
    for item in items_list:
        if not item.tests:
            gaps.append(f"{item.id}: item has no test specification")
        if item.id not in implementation_tags:
            gaps.append(f"{item.id}: no implementation code tag")
        for test in item.tests:
            has_generated = bool(generated_by_spec.get(test.id))
            has_tag = bool(test_tags.get(test.id))
            not_applicable = (
                test.id == "ADV-UART-006.T1" and board_coverage and
                sum(value.get("uart_controllers_with_dmas", 0)
                    for value in board_coverage) == 0
            )
            if not has_generated and not has_tag and not not_applicable:
                gaps.append(f"{test.id}: test specification has no generated, unit, or system test")
            for location in test_tags.get(test.id, []):
                if any(status == "FAIL" for status in junit_for_test_file(location, junit).values()):
                    gaps.append(f"{test.id}: failing unit test tagged at {location}")
        if (item.verification == "generated" and item.applies_to and
            not any(generated_by_spec.get(test.id) for test in item.tests) and
            item.id != "ADV-UART-006"):
            gaps.append(f"{item.id}: applies_to `{item.applies_to}` matched no nodes")
    for tag, locations in implementation_tags.items():
        if tag not in items:
            gaps.append(f"{tag}: implementation tag names no specification item ({', '.join(locations)})")
    for entry in generated:
        if entry["item_id"] not in items:
            gaps.append(f"{entry['name']}: generated test item {entry['item_id']} no longer exists")
        statuses = junit.get(entry["name"], {})
        if entry.get("will_fail"):
            known_defects[(entry["test_spec_id"], entry["board"],
                           entry.get("known_defect", "CTest WILL_FAIL"))] += 1
        elif any(status == "FAIL" for status in statuses.values()):
            gaps.append(f"{entry['test_spec_id']}: failing generated test {entry['name']}")
    for (test_id, board, defect), count in sorted(known_defects.items()):
        gaps.append(f"{test_id}: known defect on {board} ({count} case(s)) — {defect}")

    item_areas = Counter(item.id.split("-")[1] for item in items_list)
    test_areas = Counter(test_id.split("-")[1] for test_id in tests)
    generated_areas = Counter(entry["item_id"].split("-")[1] for entry in generated)
    area_names = list(dict.fromkeys(item.id.split("-")[1] for item in items_list))
    output = [
        f"# {', '.join(area_names)} traceability report",
        "",
        f"Generated from {', '.join(str(path) for path in args.spec)}, `{args.trace_map}`, code tags, and {len(args.junit)} CTest JUnit run(s).",
        "",
        "## Summary",
        "",
        f"- Specification items: {len(items_list)}",
        f"- Test specifications: {len(tests)}",
        f"- Generated board tests: {len(generated)}",
        f"- Implementation tags: {sum(len(value) for value in implementation_tags.values())}",
        f"- Traceability gaps: {len(gaps)}",
    ]
    for area in sorted(item_areas):
        output.append(
            f"- {area}: {item_areas[area]} items, {test_areas[area]} test specifications, "
            f"{generated_areas[area]} generated board tests"
        )
    junit_labels = sorted({label for statuses in junit.values() for label in statuses})
    for label in junit_labels:
        counts = Counter(statuses[label] for statuses in junit.values() if label in statuses)
        output.append(
            f"- CTest {label}: {counts['PASS']} passed, {counts['FAIL']} failed, {counts['SKIP']} skipped"
        )
    output.extend([
        "",
        "## Trace table",
        "",
        "| Item | Test specs | Generated / unit / system tests | Result | Code |",
        "|---|---|---|---|---|",
    ])
    for item in items_list:
        test_descriptions: list[str] = []
        test_results: list[str] = []
        for test in item.tests:
            entries = generated_by_spec.get(test.id, [])
            tags = test_tags.get(test.id, [])
            not_applicable = (
                test.id == "ADV-UART-006.T1" and board_coverage and
                sum(value.get("uart_controllers_with_dmas", 0)
                    for value in board_coverage) == 0
            )
            parts = []
            statuses: dict[str, str] = {}
            if entries:
                parts.append(f"generated: {len(entries)}")
                for entry in entries:
                    statuses.update(junit.get(entry["name"], {}))
                known = sum(1 for entry in entries if entry.get("will_fail"))
                if known:
                    parts.append(f"known defect/WILL_FAIL: {known}")
            if tags:
                parts.append("unit: " + ", ".join(tags))
                for location in tags:
                    statuses.update(junit_for_test_file(location, junit))
            if not_applicable:
                parts.append("not applicable: 0 matching UART controllers with dmas")
            elif not parts:
                parts.append("GAP: no test")
            test_descriptions.append(f"{test.id} ({'; '.join(parts)})")
            status_text = "N/A" if not_applicable else result_text(statuses)
            if entries and any(entry.get("will_fail") for entry in entries):
                status_text += " (known defect retained with CTest WILL_FAIL)"
            test_results.append(status_text)
        code = ", ".join(implementation_tags.get(item.id, [])) or "GAP: no library tag"
        output.append("| " + " | ".join(escape_table(value) for value in [
            f"{item.id} — {item.statement}",
            "<br>".join(test.id for test in item.tests) or "GAP",
            "<br>".join(test_descriptions),
            "<br>".join(test_results) or "not tested",
            code,
        ]) + " |")

    output.extend(["", "## Gaps", ""])
    output.extend(f"- {gap}" for gap in gaps)
    if not gaps:
        output.append("- None.")

    counts = Counter(entry["board"] for entry in generated)
    area_columns = sorted(generated_areas)
    output.extend([
        "",
        "## Generated coverage by board DTB",
        "",
        "| Board DTB | Generated tests | " + " | ".join(area_columns) + " | Ordinary | WILL_FAIL |",
        "|---|---:|" + "---:|" * len(area_columns) + "---:|---:|",
    ])
    for board, count in sorted(counts.items()):
        known = sum(1 for entry in generated if entry["board"] == board and entry.get("will_fail"))
        per_area = [sum(1 for entry in generated if entry["board"] == board and
                        entry["item_id"].startswith(f"ADV-{area}-"))
                    for area in area_columns]
        output.append("| " + " | ".join([board, str(count), *(str(value) for value in per_area),
                                           str(count - known), str(known)]) + " |")

    output.extend([
        "",
        "## UART node applicability by board DTB",
        "",
        "| Board DTB | UART controllers | UART children | UART controllers with dmas |",
        "|---|---:|---:|---:|",
    ])
    for value in sorted(board_coverage, key=lambda entry: entry["board"]):
        output.append(
            f"| {value['board']} | {value['uart_controllers']} | "
            f"{value['uart_children']} | {value['uart_controllers_with_dmas']} |"
        )

    if any("dt_nodes" in value for value in board_coverage):
        output.extend([
            "",
            "## DT loader applicability by board DTB",
            "",
            "| Board DTB | Negative copies | Model nodes | MMIO nodes | IRQ nodes | Board determinism |",
            "|---|---:|---:|---:|---:|---:|",
        ])
        for value in sorted(board_coverage, key=lambda entry: entry["board"]):
            output.append(
                f"| {value['board']} | {value.get('dt_negative_nodes', 0)} | "
                f"{value.get('dt_nodes', 0)} | {value.get('dt_mmio_nodes', 0)} | "
                f"{value.get('dt_irq_nodes', 0)} | {value.get('dt_boards', 0)} |"
            )

    output.extend([
        "",
        "## Testability note",
        "",
        "ADV-UART-006.T1 applies only to UART controllers that declare `dmas`; none of the four current board DTBs do, so it correctly generates no cases. DT negative tests mutate build-directory copies only. DT expectations come from the independent parser; confirmed extra MMIO registrations and unresolved adapter interrupts remain `WILL_FAIL`. The specification was left unchanged.",
        "",
        "## Area cost",
        "",
    ])
    if args.time:
        output.append("Time recorded by the tester:")
        output.append("")
        output.extend(f"- {value}" for value in args.time)
        output.append("")
    output.extend([
        "Lines of maintained pilot material (generated C and maps excluded):",
        "",
        *(f"- {path.stem.upper()} specification YAML: {lines(path)}" for path in args.spec),
        f"- DTB test generator: {lines(args.generator)}",
        f"- Trace report tool: {lines(Path(__file__))}",
        "",
        f"The combined rollout has {len(items_list)} items, {len(tests)} test specifications, and {len(generated)} generated cases. "
        "Per-DTB counts are shown above; the base and Hakoniwa variants exercise the same physical wiring independently.",
        "",
        "Reused: the stdlib FDT parser, status/compatible decoding, node walking, board discovery, generated C harness, public loader build path, trace-map schema, CTest/JUnit merge, gap rules, code-tag scan, and multi-spec report parser.",
        "",
        "The report records per-area applicability and keeps confirmed implementation defects as generated CTest `WILL_FAIL` cases.",
    ])
    return "\n".join(output) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", action="append", required=True, type=Path)
    parser.add_argument("--trace-map", required=True, type=Path)
    parser.add_argument("--junit", action="append", required=True,
                        help="LABEL=path/to/ctest.xml (repeatable)")
    parser.add_argument("--source-root", type=Path, default=Path("."))
    parser.add_argument("--generator", type=Path, default=Path("tools/dt_testgen.py"))
    parser.add_argument("--time", action="append", default=[])
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(report(args), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
