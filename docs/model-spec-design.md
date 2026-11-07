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

## Bindings interface (format 1)

The bindings interface is one C interface for every model and every IP
version (`include/athrill_bindings.h`). Differences between IP versions and
between models are values in its structures, never different structures:
a 16-bit data register and a 32-bit one are both an `AthrillBindRegister`
with `size` 16 or 32.

Format and code are versioned together: the generated YAML carries
`bindings_format: 1`; `src/bindings.c` dispatches on it to `src/bindings_v1.c`,
which reads exactly format 1 into the structures below. An unknown format, an
unknown key or a shape the structures cannot hold stops loading with an error;
nothing is dropped silently. The member names equal the YAML keys.

### Shapes the interface must hold

Counted over the 17 generated bindings (2026-11-07, 320 terms):

| Shape | Terms | Example |
|---|---|---|
| one field | 161 | I2C `enable` = `CR1.PE` |
| indexed repetition | 79 | DMA `stream_enable` = `S{s}CR.EN`, s = 0..7 |
| Devicetree resource | 31 | DMA `stream_irq` |
| several fields of one register | 18 | I2C `status_ack` = `ICR.{STOPCF,NACKCF}` |
| whole register | 15 | I2C `tx_data` = `TXDR` |
| indexed whole registers | 6 | DMA `memory_address` = `S{s}M0AR`, `S{s}M1AR` |
| several whole registers | 5 | GPIO `pin_config` |
| indexed, other peripheral | 2 | EXTI `line_source` = SYSCFG `EXTICR{r}.EXTI{n}` |
| not applicable on a SoC | 1 | H7 DMA `request_select` |
| sequence | 1 | F7 SPI `overrun_ack` (read DR, then SR) |
| derived | 1 | H7 SPI `busy` |

Further shapes already known from the SVD/IP survey and covered by the same
structures: a value split over fields (H7 I2C `SADD0..9`, USART `BRR`),
instances with a different layout (TIM2/TIM5 32-bit counters), and value
encodings (`DS` = bits - 1, `BR` = divider 2^(BR+1)).

### Structures

```c
typedef struct {            /* one SVD field */
	char   name[24];
	uint8  lsb, width;
	AthrillBindSvdAccess access;      /* read-write, read-only, write-only, ... (patched SVD) */
} AthrillBindField;

typedef struct {            /* one SVD register of one peripheral */
	char   name[24];
	uint16 peripheral;                /* index into AthrillBindings.peripherals */
	uint32 offset, size, reset;       /* size in bits: 16 or 32 is a value, not a type */
	uint32 stored_mask;               /* bits of writable fields */
	uint16 field_first, field_count;  /* slice of AthrillBindings.fields */
} AthrillBindRegister;

typedef struct {            /* one place a term lives at */
	uint16 reg;                       /* index into AthrillBindings.registers */
	uint16 field_first, field_count;  /* slice of AthrillBindings.place_fields (0 = whole register) */
	uint8  index_count;               /* 0..2 placeholder values, e.g. s=3 or r=2,n=5 */
	char   index_name[2][4];
	uint16 index_value[2];
} AthrillBindPlace;

typedef struct {            /* how the register value maps to the term value */
	AthrillBindEncodingKind kind;     /* IDENTITY, MINUS_ONE, POW2_PLUS_ONE, TABLE */
	int32  arg;
} AthrillBindEncoding;

typedef struct {            /* one model term */
	char   name[40];
	AthrillBindKind kind;             /* REGISTER, DEVICETREE, DERIVED, SEQUENCE, NOT_APPLICABLE */
	AthrillBindAccess access;         /* rw, ro, wo, w1c, w1s, w0c, rs, rc, fifo */
	AthrillBindEncoding encoding;
	uint16 place_first, place_count;  /* slice of AthrillBindings.places, in index order */
	const char *text;                 /* Devicetree property, derivation or sequence (documentation) */
} AthrillBindTerm;

typedef struct {            /* an instance whose layout differs (e.g. TIM2) */
	char   instance[16];
	uint16 peripheral;                /* the SVD peripheral to use for it */
} AthrillBindInstance;
```

`AthrillBindings` holds the header data (`bindings_format`, `ip`, `soc`,
`compatible[]`, `model`, `model_version`, `features[]`, `constants[]`) and the
flat arrays `peripherals[]`, `registers[]`, `fields[]`, `places[]`,
`place_fields[]`, `terms[]`, `instances[]`.

### Operations

- `athrill_bindings_for(compatible, soc)`: one bindings set; `soc` selects
  between sets that share a compatible (F7 and H7 USART).
- `athrill_bindings_register(b, peripheral, name)` / `athrill_bindings_field(...)`:
  look up by SVD name (used once at start-up).
- `athrill_bindings_term(b, name)`: a term with its places.
- `athrill_bindings_resolve_places(term, index_name, out[], count)`: the places
  of an indexed term as an array indexed by the placeholder value (stream,
  channel, line), so a model indexes `out[s]` at run time.

Models resolve once at start-up into their own arrays and variables; at run
time no lookup or loop over bindings takes place.

### Conformance test

A test loads every file in `spec/bindings/` and checks, against the YAML read
independently by the test, that every term, place, index and field arrived
with its values: no term lost, no place dropped.

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
