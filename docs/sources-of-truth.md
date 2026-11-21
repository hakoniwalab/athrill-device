# Sources of truth: Devicetree, SVD, and everything else

A device model answers a firmware's register accesses. Each fact it uses comes
from exactly one authoritative source. This page says which source owns which
kind of fact, and how each is checked. A fact that is in the wrong place, or
copied from one source into another without a check, is a defect waiting to
happen.

Details live in [device-tree-design.md](device-tree-design.md) (Devicetree),
[model-spec-design.md](model-spec-design.md) (SVD, semantics, bindings) and
[traceability-design.md](traceability-design.md) (requirements and tests).

## The three sources

| Source | Owns | Does not own | Form |
| --- | --- | --- | --- |
| **Devicetree** (Zephyr DTSI + board DTS) | Where things are and how they are connected: which IP instances exist (`compatible`, `status`), base addresses (`reg`), interrupt lines (`interrupts`), DMA request lines (`dmas`), pins and chip selects (`*-gpios`), bus children and their addresses, clock rate, mount orientation, the value source of a sensor, board identity (ADC ladder values). | What is inside a peripheral, or how it behaves. | Read at run time from the DTB. Nothing is generated from it. |
| **SVD** (vendor CMSIS-SVD, pinned, with `spec/svd-patches.yaml`) | What is inside an on-chip peripheral: register offsets, fields, reset values, access, and the SoC's interrupt numbers. | Meaning and behaviour (the SVD knows positions, not what a bit does), DMA request numbers, pins, board wiring. | Generated into `spec/bindings/` through `spec/semantics/`. |
| **Everything else**: reference manuals, chip data sheets, firmware sources, measurements | Behaviour: what happens when a register is written or read, timing, FIFO and overflow rules, wire format (bit and byte order), chip identity and conversion formulas, noise and bandwidth; also tables the SVD does not carry (for example the STM32H7 DMAMUX request table). | Placement and wiring (Devicetree), register layout of on-chip peripherals (SVD). | Hand-written model code and model specifications (`spec/models/`, `spec/chips.yaml`), each rule numbered and citing its document, section and revision. |

Off-chip chips (IMUs, barometers, magnetometers, ESCs) have no SVD. Their
registers and behaviour come from the chip data sheet and are written in
`spec/chips.yaml`; the Devicetree only places them on a bus.

Physical values (angular rate, pressure, field) are not a board fact at all:
they come from the value source (the sensor host / Hakoniwa) named in the
Devicetree.

## Where each kind of fact is checked

| Fact | Owner | Check |
| --- | --- | --- |
| Register offsets, fields, reset values | SVD | `tools/svd_map.py check-all`; generated reset-value tests (#31) |
| Interrupt numbers in a board | Devicetree, against SVD | cross-check of every `interrupts` cell with the SVD (#31) |
| DMA request numbers in a board | Devicetree, against the reference manual table | cross-check of every `dmas` cell with a request table taken from the manual (#31; the SVD does not carry this table) |
| Wiring, pins, chip selects, bus addresses | Devicetree | generated wiring and construction tests; the board's README tables, from the firmware's board sources |
| Behaviour, timing, wire format | Reference manual / data sheet | numbered requirements, tests whose expected values cite the document, not the model |
| Chip identity, conversions, bandwidth, output rates | Data sheet | the chip's requirements in `spec/chips.yaml` and their tests |
| Sensor noise and bias (the chip's intrinsic errors) | Data sheet, applied by the value source | the sensor host's per-instance sensor configuration, citing the data-sheet table; real firmware rejects noise-free sensors as stale |
| Board identity as the firmware sees it | Devicetree, against the firmware | boot check: the firmware reports the expected board and revision |

## Things that are easy to put in the wrong place

- **A number in the Devicetree is not checked just because it is in the
  Devicetree.** `dmas`, `interrupts` and similar cells are written by hand for
  each board. They must be checked against the source that owns their meaning
  (the SVD or the manual's table). Example: a Pixhawk 6X board gave TIM5 the
  DMA request 50 (TIM8_CH4) instead of 59 (TIM5_UP); the structure checks
  passed and DShot never ran.
- **Wire format is behaviour, not a Devicetree property.** Frame size and bit
  order are written by the firmware into the controller's registers (for
  STM32H7 SPI: CFG1.DSIZE, CFG2.LSBFRST). The model follows those registers;
  the Devicetree's `spi-lsb-first` only tells a driver what a device needs.
  Example: 16-bit SPI frames must leave high byte first (RM0433 50.4.7).
- **The SVD does not say what a bit means.** Meaning comes from the manual
  through `spec/semantics/`; the SVD is only the position.
- **Data-sheet figures belong to the chip, not to the board.** Bandwidth,
  rates and conversions are chip requirements with their table cited; noise
  and bias are set in the value source's sensor configuration with the same
  citation. The Devicetree carries none of them.
