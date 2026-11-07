# Traceability: from the Devicetree to the tested library

This document defines how athrill-device shows what was verified and how. The
Devicetree is the canonical description of a board (the requirement). What it
guarantees is written down as numbered specification items. Tests are derived
from those items, generated code carries a map back to them, and the shared
device library is linked to them by the same numbers. A reader can then follow
any guarantee to the tests that check it and to the code that implements it,
whoever wrote that code.

## Chain

```
Devicetree (board requirement)
  └─ specification item      ADV-<AREA>-<NNN>        what a node, property or compatible guarantees
       └─ test specification ADV-<AREA>-<NNN>.T<n>   how that guarantee is checked
            ├─ generated test  per board, per node, carries the test-spec ID
            └─ library code    implements the item, tagged with the spec ID
```

## Specification items

A specification item states one guarantee that follows from the Devicetree.
It is written so that it can be checked without reading the implementation.

Three levels:

| Level | Area prefix | Guarantees | Example |
|---|---|---|---|
| Loader | `DT` | how any Devicetree is turned into a board | `ADV-DT-001` An enabled node whose `compatible` has no model stops the start-up with an error naming the node. |
| Binding | the compatible's area, e.g. `SPI`, `I2C`, `UART`, `DMA`, `SD`, `TIM`, `GPIO`, `EXTI` | what a node of that kind provides, from its properties | `ADV-SPI-003` A child of an SPI controller is selected only by the chip select given in its `reg`. |
| Chip | `CHIP-<PART>` | what a chip model answers on its bus, as far as firmware drivers depend on it | `ADV-CHIP-ICM42688-001` WHO_AM_I (0x75) reads 0x47. |

Each item records:

- `id`, `statement` (one sentence, testable);
- `source`: the Devicetree property or binding it follows from, and the
  manual section (reference manual, datasheet, Devicetree specification) that
  fixes the expected value;
- `scope`: the firmware behaviour it supports (see "Fidelity" in
  `device-tree-design.md`; no item is written for behaviour no driver uses);
- `verification`: `generated` (per board, from the Devicetree), `unit`
  (engine or protocol test), or `system` (boot, armed trace, flight).

Board instances are not specification items. A board's DTS is data: the
generator turns "`ADV-SPI-003` for every SPI child of this board" into
concrete tests.

## Test specifications

Every item has at least one test specification `ADV-<AREA>-<NNN>.T<n>`:
the stimulus, the expected result and where the expected value comes from.
A test specification never names an implementation function; it is written
from the item and the manuals.

## Files

```
spec/
  dt.yaml             loader items
  spi.yaml, i2c.yaml, ...   binding items
  chips/icm42688p.yaml, ... chip items
tools/
  dt_testgen.py       Devicetree + spec -> generated tests + trace map
  trace_report.py     spec + trace map + code tags + test results -> report
```

Items and their test specifications live together in one YAML file per area:

```yaml
- id: ADV-SPI-003
  statement: A child of an SPI controller is selected only by the chip select given in its reg.
  source: {devicetree: "spi-controller binding, reg = chip select", manual: "RM0433 50.4.7"}
  scope: drivers address one sensor per chip select
  verification: generated
  tests:
    - id: ADV-SPI-003.T1
      stimulus: assert the chip select of the child; read its ID register
      expect: the child answers; every other child on the bus does not drive MISO
```

## Generated tests

`dt_testgen.py` reads each board DTB with its own Devicetree reader (it does
not use the C loader) and the spec files. For every node and every item that
applies to it, it writes a C test case. Each case:

- is named after the board, the node path and the test-spec ID;
- builds the board with the real loader through the common interface and
  checks the expectation;
- is listed in `trace-map.json` with its test-spec ID, board, node path and
  the Devicetree properties it was derived from.

CTest runs the generated tests for every DTB under `boards/`. Adding a board
adds its tests without new test code.

## Library tags

Code that implements an item carries the item ID in a comment next to the
implementation:

```c
/* [ADV-SPI-003] chip select decides the selected child */
```

One function may carry several IDs; an ID may appear in several places.

## Trace report

`trace_report.py` combines the spec files, `trace-map.json`, the code tags
and the CTest results into one table per item:

| Item | Test specs | Generated / unit / system tests | Result | Code |
|---|---|---|---|---|

and lists the gaps:

- an item without a test specification, or a test specification without a
  test;
- a failing test, per item;
- a code tag naming no item, and an item no code is tagged with;
- a generated test whose item no longer exists.

CI publishes the report for every change. It answers what was verified, by
which test, from which requirement, and how the expected value was obtained.

## Independence

- Items, test specifications, the generator and chip ID tables are written
  in the tester role; library code in the implementer role.
- The generator has its own Devicetree reader and never includes loader code.
- Expected values come from the Devicetree and the manuals, never from the
  output of the implementation.
- Who wrote a piece of code (a person or an AI) does not change how it is
  verified.

## Existing tests

The engine, protocol and front-end unit tests receive test-spec IDs for the
items they already check. System checks (boot equality of the C-built and
Devicetree-built board, the armed take-off trace, hover) are recorded as
`system` verification of the items they cover.

## Keeping the cost low

Traceability is only kept up if it costs little. Rules:

- Nothing in the chain is maintained by hand except the spec files. The
  per-board tests, the trace map and the report are generated.
- Items are coarse: one item per guarantee a firmware driver depends on, not
  one per register or per line of code. A binding has a handful of items.
- Library tags go on the function that implements the item, not on every
  line.
- Board-specific facts stay in the DTS; adding a board adds no spec text.
- Gaps are reported, not enforced, until an area is fully rolled out; then
  CI fails on gaps in that area only.
- Each area is introduced in one small step that ends green. The pilot's
  effort is measured before the next area starts.

## Rollout

1. SPI as the pilot: items, test specifications, generated tests for
   Pixhawk 4 and Pixhawk 6C, library tags, report.
2. The loader items, then I2C, UART, DMA, SD, timers, GPIO/EXTI.
3. Chip items for the chips on the two boards.
