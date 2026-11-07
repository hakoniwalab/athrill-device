# Device behaviour model specifications

athrill-device models must not depend on a board, and as far as possible not on
one vendor IP version. This document defines how the behaviour of a device
class is written once, in abstract terms, and bound to the registers of each IP
version by a map.

## Boundaries

```
 Conversion tool (tools/bindings_gen.py)
   inputs only:  ST SVD (pinned)            register definitions
                 spec/svd-patches.yaml      corrections of SVD errors, with manual evidence
                 spec/semantics/ + spec/models/   model semantics: which model term is which SVD name
   output only:  spec/bindings/             bindings

 Device implementation
   reads only:   spec/bindings/ + spec/models/
```

- **Model specification** (`spec/models/`): the meaning — terms and behaviour,
  no registers.
- **Bindings** (`spec/bindings/<ip>__<soc>_gen.yaml`, generated): the
  translation between register accesses and meaning. Input side: register
  offset and bits (the base address comes from the Devicetree `reg` at start-up);
  output side: model terms. The device implementation receives meaning from
  the bindings and gives meaning back; it never sees the SVD.
- **Semantics** (`spec/semantics/<ip>.yaml`, hand-written or proposed by a tool
  and reviewed): which model term is which SVD register/field name, per IP
  version (and per SoC where the SVD names differ). The SVD knows positions, not
  meaning; this is the only place meaning meets SVD names.
- **SVD** (pinned) and **SVD patches**: register definitions; the SVD is
  canonical and wrong values are corrected only in `spec/svd-patches.yaml`.
  SVD quirks that are not errors (split fields, different spellings) are not
  patched; the semantics use the SVD names and note the manual's.

The Devicetree places IP instances (`reg`, `interrupts`, ...) and selects the
bindings by `compatible` and SoC. Boards add no bindings.

## Model specification

A model specification describes one device class, for example an SPI
controller, without register names.

- `model`, `version` (semantic: a new required term or a changed behaviour is a
  major change; a new optional feature is a minor change).
- `terms`: the abstract controls, statuses, data ports and events, each with a
  kind:
  - `control` (written by the driver: enable, mode, sizes),
  - `status` (read by the driver: data available, buffer empty, busy, errors),
  - `data` (data ports: transmit data, receive data),
  - `event` (something the model raises: interrupt request, DMA request),
  - `ack` (driver action that clears a status or event).
- `features`: optional groups of terms and behaviour that only some IP versions
  have. Every term belongs to `base` or to one feature.
- `behaviour`: numbered rules written only with terms, in the form
  "when ..., then ..."; the rule numbers are the specification items
  (`ADV-<CLASS>-B<NNN>`) of the traceability design.
- `timing`: when events happen, by the timing rules of `device-tree-design.md`
  (completion clock, same-clock order).

Scope follows the fidelity rule: only behaviour that PX4 or ArduPilot drivers
depend on. Everything else is out of the model, and the map says so.

## Semantics file

A semantics file binds one IP version to a model specification by SVD names.

- `ip`: the IP version, and the `compatible` strings it serves.
- `model`, `model_version`, `features`: what it implements.
- `bindings`: for every term of the base and of each declared feature, the
  register and field (names as in the reference manual / SVD) and the access
  kind: `rw`, `ro`, `wo`, `w1c` (write 1 to clear), `rc` (clear on read),
  `w1s` (write 1 to set), `fifo` (data port with a FIFO behind it).
- `constants`: IP-specific values the behaviour uses (FIFO depth, data size
  encoding).
- `unmodelled`: registers and fields the IP has that the model does not cover,
  with how the implementation answers them (store and read back, read as reset
  value).

Register definitions (offsets, fields, reset values, access) have one canonical
source: the **ST SVD** of the SoC, at the revision pinned in
`spec/svd-sources.yaml` (fetched by `tools/fetch_svd.sh`, not committed).
Where the SVD disagrees with the reference manual, the manual wins and the
correction is recorded in `spec/svd-patches.yaml`, one entry per difference with
the manual section and page as evidence. Nothing else overrides the SVD. The
patches carry the SVD value they correct, so a new SVD revision that fixes or
changes the value stops the tools until the patch is reviewed.

During the transition, maps still carry hand-written `registers`;
`tools/svd_map.py check-all` compares them with the patched SVD and must report
no mismatch. The next step generates the maps' `registers` from the patched SVD
and keeps only the bindings (terms to registers and fields) hand-written.
The stm32-rs project's patched SVDs may be consulted as a reference when
investigating a difference; they are not a source.

## Verification

- The model specification's behaviour rules are spec items; tests are written
  in model terms and run against every map that declares the feature.
- For each map, generated tests check that every bound term is reachable at its
  register and field with its access kind (after #31, also the SVD position and
  reset value).
- Board-level tests stay as in `traceability-design.md`.

## First class: SPI controller

`spec/models/spi-controller.yaml` with two maps:
`spec/maps/st-stm32-spi-v1.yaml` (STM32F7) and
`spec/maps/st-stm32-spi-v2.yaml` (STM32H7).
