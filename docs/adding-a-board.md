# Adding a board from a Devicetree

Athrill uses a compiled Devicetree blob (DTB) to describe device instances,
addresses, and wiring. The DTB contains structure and static configuration;
the C models selected by each node's `compatible` property contain behaviour.
The graph is validated and built once at startup, so Devicetree is not parsed
in the instruction loop.

Every board is built only from its DTB; no target builds a board in C code.
The STM32 target runs Pixhawk 4 (`boards/pixhawk4/`), Pixhawk 6C
(`boards/pixhawk6c/`), NUCLEO-F767ZI (`boards/nucleo-f767zi/`) and
NUCLEO-H743ZI (`boards/nucleo-h743zi/`), and the RISC-V target runs Erbium
(`boards/erbium/`). Every peripheral of those boards is an athrill-device
model; the targets keep only the CPU's architectural devices (section 4).

The STM32 boards share Athrill's per-family facts (start-up read rules of the
register blocks, the NVIC line count) in `boards/stm32-athrill/stm32f7-athrill.dtsi`
and `stm32h7-athrill.dtsi`. The NUCLEO boards include Zephyr's own board DTS
unchanged, so a Zephyr application built for that board runs on the board
Athrill builds from the same description; peripherals without a model are
switched off in the Athrill layer (see each board README).

This guide uses Pixhawk 6C as the running example. Its files in
`boards/pixhawk6c/` are the result of the steps below and can be compared
with a new board at each step.

## 1. Choose and record the SoC source

Prefer an existing public SoC DTSI over creating one. For STM32F765 and
STM32H743, use Zephyr's `stm32f765Xi.dtsi` and `stm32h743Xi.dtsi`. Pin the
Zephyr revision rather than following a branch; the existing boards use
`f3fc249035296b87c75fa808b3abf78829fe9378`.

Zephyr's DTSI files, bindings, and Devicetree headers are Apache-2.0. An
external checkout is only a build input, but the board README must still name
the revision and licence. If any Zephyr file is copied or modified in this
repository, retain its copyright, Apache-2.0 notice, and attribution.

Create `boards/pixhawk6c/README.md`, `boards/pixhawk6c/pixhawk6c.dts`, and,
after compilation, `boards/pixhawk6c/pixhawk6c.dtb`. Start the DTS with an MIT
SPDX line, a comment naming the included Zephyr DTSI and revision, and the
public board-definition files used for wiring facts. Include the H743 DTSI as
`st/h7/stm32h743Xi.dtsi`.

If there is no usable public DTSI, write the smallest SoC DTSI that describes
the devices required by the board. Keep register behaviour and bit-field
definitions in C models, not in Devicetree.

When an SVD is the only usable source:

1. Use the silicon vendor's official SVD and verify its redistribution terms.
   For STM32, the SVDs in the official Open-CMSIS-Pack device-family packs are
   Apache-2.0; pin the pack and file versions and retain their notices.
2. Generate only structural facts such as peripheral instances, `reg` ranges,
   interrupt numbers, and reset values needed by a model. Review the generated
   DTSI against the reference manual.
3. Keep the SVD's copyright and Apache-2.0 notice with bundled SVD-derived
   data. Do not use an STM32 SVD from a collection whose licence restricts it
   to ST tools.
4. Treat the generator as an MIT project tool. A generated DTSI does not make
   register behaviour data-driven; the C model remains the implementation.

## 2. Write the board layer

The SoC DTSI owns on-chip instances: buses, peripheral MMIO ranges, interrupts,
clocks, resets, GPIO controllers, and DMA controllers. The board DTS enables
the instances that are connected on the board and adds off-chip devices and
board wiring.

Use the board's own public definition as the source of truth. For Pixhawk 6C,
the PX4 FMUv6C `spi.cpp`, `i2c.cpp`, `timer_config.cpp`, sensor startup file,
board defaults, and `board.h` establish the V6C00 baseline. PX4's board files
are BSD-3-Clause: hardware facts may be recorded in the MIT DTS, but cite the
exact source files and retain notices for any copied material. Choose one
hardware revision explicitly instead of combining alternative sensor sets.

ArduPilot `hwdef.dat` is GPL. A local importer may read a user's local copy to
cross-check a board, but do not commit the input, generated output, copied
comments, or other material derived from it. Do not reuse ArduPilot's
generator code in this repository.

Apply these placement rules:

| Information | Where it belongs |
| --- | --- |
| On-chip bus controller, MMIO range, IRQ, clock, reset, and DMA controller | SoC DTSI; the board DTS normally only sets `status = "okay"` and adds board connections |
| SPI chip | Child of its SPI controller; `reg` is the zero-based chip-select index and selects the same entry in the parent's `cs-gpios` list |
| I2C chip | Child of its I2C controller; `reg` is its 7-bit bus address |
| UART peer such as a GPS | Child of the UART controller; it has no chip-select or I2C address |
| Device interrupt or data-ready line | The binding's GPIO or `interrupts` property, referring to the actual provider and line |
| DMA request | Consumer's `dmas` specifiers, with matching `dma-names`; the specifier cells follow the DMA provider binding |
| Host files, Hakoniwa asset/channel names, trace paths, and TCP ports | `device_config.txt`, never the DTS |
| Ordered actuator outputs | One `athrill,pwm-actuator-map` node whose `pwms` list is in firmware output order; node declaration order is irrelevant |

For the Pixhawk 6C V6C00 baseline, this gives three children below SPI1
(BMI055 accelerometer on CS 0, BMI055 gyroscope on CS 1, and ICM-42688-P on
CS 2), the F-RAM below SPI2, MS5611 and IST8310 below I2C4, GPS below USART1,
the SD card below SDMMC2, and eight ordered PWM references to TIM1, TIM4, and
TIM5. Record mounting orientation and interrupt GPIOs on the corresponding
chip nodes, not on the bus.

The loader ignores disabled nodes and nodes without a `compatible` property.
It also knows a limited set of structural compatibles and target-owned Arm
architectural nodes. If an enabled topology-only node has another compatible,
mark it with the boolean `athrill,structural` property and explain why in a
comment. Never use that property to hide a functional device whose model is
missing: every enabled functional node must resolve to a built-in model, an
explicit plugin, or a target-owned architectural device.

Sensor chips and the ordered PWM output read and write physical values
through an explicit `value-source` phandle; nothing reaches them through the
target. The base board declares two sources at board scope: an
`athrill,bench-sensor-source` node, which the sensor chips and the
`athrill,pwm-actuator-map` node point at, and a `hakoniwa,sensor-device` node
with `status = "disabled"`. A second DTS (`<board>-hakoniwa.dts`) includes
the base, enables the Hakoniwa node, and points the same consumers at it. Each
of those consumers also carries the boolean `athrill,tick-before-value-source`
property, so it ticks before the source and reads the previously latched
value. The base board then runs without the Hakoniwa plugin (SIH and bench
runs), and the Hakoniwa board needs it (section 4). Link, channel, and trace
settings stay in `device_config.txt`.

## 3. Compile and identify the DTB

Run the following commands from the athrill-device repository root in a Linux
environment with GNU `cpp`, `dtc` 1.7.x, Git, and `sha256sum`. They fetch the
pinned full Zephyr tree as an ignored build input. The board DTS must already
exist at `boards/pixhawk6c/pixhawk6c.dts`.

```bash
set -eu
ROOT=$PWD
ZEPHYR_REV=f3fc249035296b87c75fa808b3abf78829fe9378
ZEPHYR=$ROOT/build/zephyr

if [ ! -d "$ZEPHYR/.git" ]; then
  git clone --filter=blob:none https://github.com/zephyrproject-rtos/zephyr.git "$ZEPHYR"
fi
git -C "$ZEPHYR" fetch --depth=1 origin "$ZEPHYR_REV"
git -C "$ZEPHYR" checkout --detach FETCH_HEAD
test "$(git -C "$ZEPHYR" rev-parse HEAD)" = "$ZEPHYR_REV"
```

Preprocess the DTS and compile the exact result that will be committed. For a
RISC-V board, replace the `dts/arm` include with `dts/riscv`:

```bash
set -eu
ROOT=$PWD
BOARD=pixhawk6c
ZEPHYR=$ROOT/build/zephyr

mkdir -p "$ROOT/build/devicetree"
cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp -P \
  -I "$ZEPHYR/dts" \
  -I "$ZEPHYR/dts/arm" \
  -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  "$ROOT/boards/$BOARD/$BOARD.dts" \
  "$ROOT/build/devicetree/$BOARD.pre.dts"

dtc -I dts -O dtb \
  -o "$ROOT/boards/$BOARD/$BOARD.dtb" \
  "$ROOT/build/devicetree/$BOARD.pre.dts"

sha256sum "$ROOT/boards/$BOARD/$BOARD.dtb"
```

The preprocessed source is disposable build output. Commit the source DTS and
the DTB under `boards/pixhawk6c/`, and copy the printed SHA-256 into that
directory's README. Rebuild from a clean checkout and require the same hash.
A changed hash is a reviewable board-description change even when no C source
changed.

## 4. Resolve every `compatible`

The loader chooses the first exact registered string in a node's ordered
`compatible` list. It never guesses from the node name. The production
registry accepts the following strings; test-only `athrill,test-*` entries are
not board interfaces.

| Kind | Registered compatible strings |
| --- | --- |
| Board and SoC roots (structural) | `holybro,pixhawk4`; `holybro,pixhawk6c`; `st,stm32f767zi-nucleo`; `st,stm32h743zi-nucleo`; `openhwgroup,erbium`; `st,stm32f765`; `st,stm32f767`; `st,stm32f7`; `st,stm32h743`; `st,stm32h7` |
| Structural nodes | `simple-bus`; `simple-mfd`; `syscon`; `fixed-clock`; `zephyr,memory-region`; `zephyr,memory-attr`; `mmio-sram`; `st,stm32f7-pinctrl`; `st,stm32-pinctrl`; `st,stm32-reset`; `st,stm32-rcc`; `st,stm32-rcc-rctl`; `st,stm32-nv-flash`; `soc-nv-flash`; `st,stm32-nvm-otp`; `fixed-layout`; `usb-nop-xceiv`; `st,stm32-smbus`; `st,stm32-pwm`; Zephyr's description-only nodes `zephyr,mapped-partition`; `fixed-partitions`; `gpio-leds`; `pwm-leds`; `gpio-keys`; `arduino-header-r3`; `st,stm32-hse-clock`; `st,stm32fx-pll-clock`; `st,stm32h7-pll-clock`; `st,stm32-vref`; `st,stm32-vbat`; `st,stm32-temp-cal` |
| STM32 on-chip models | `st,stm32-gpio`; `st,stm32-exti`; `st,stm32f7-syscfg`; `st,stm32h7-syscfg`; `st,stm32-dma-v1`; `st,stm32-dmamux`; `st,stm32-spi-fifo`; `st,stm32-spi`; `st,stm32h7-spi`; `st,stm32-i2c-v2`; `st,stm32-timers`; `st,stm32-usart`; `st,stm32-uart`; `st,stm32-sdmmc` |
| Register blocks (start-up registers whose read rules are in the DTS) | `athrill,register-block`; `st,stm32f4-rcc`; `st,stm32h7-rcc`; `st,stm32h7-pwr`; `st,stm32-flash-controller`; `st,stm32f7-flash-controller`; `st,stm32h7-flash-controller`; `zephyr,flash-controller`; `st,stm32-rtc`; `st,stm32-adc`; `st,stm32f4-adc`; `st,stm32-rng`; `zephyr,entropy`; `st,stm32-otgfs`; `st,stm32-watchdog`; `zephyr,crc`; `openhwgroup,erbium-mram-controller` |
| RISC-V (Erbium) on-chip models | `riscv,machine-timer`; `openhwgroup,shakti-uart-v1` |
| Off-chip chip models | `invensense,icm20689`; `invensense,icm42688`; `invensense,icm4268x`; `bosch,bma280` (BMI055 accelerometer); `bosch,bmg160` (BMI055 gyroscope); `infineon,fm25xxx`; `ramtron,fm25v02a`; `isentek,ist8310`; `meas,ms5611`; `u-blox,m8`; `zephyr,sdmmc-disk` |
| Value sources and sinks | `athrill,bench-sensor-source`; `athrill,pwm-actuator-map` |
| Plugin (not built in) | `hakoniwa,sensor-device` (STM32 boards) and `hakoniwa,sensor-registers` (Erbium), from the `board/common/` library (`libhako_sensor_device`) |
| STM32 target-owned architectural nodes | `arm,cortex-m7`; `arm,v7m-nvic`; `arm,armv7m-systick`; `arm,armv7m-mpu`; `arm,armv7m-dwt`; `arm,dtcm` |
| RISC-V target-owned architectural nodes | `riscv`; `riscv,cpu-intc`; `sifive,plic-1.0.0` |

An enabled node whose own compatible has no model can still be kept in two
ways, both visible in the DTS. Put `athrill,register-block` first in its
`compatible` list and give its start-up read rules, as the Athrill layers do
for `st,stm32-lpuart`, `st,stm32-bxcan`, `st,stm32h7-fdcan` and `st,stm32-bdma`;
or, for a node without registers of its own, add the empty property
`athrill,structural`. Otherwise switch it off with `status = "disabled"`; an
enabled node that matches nothing stops the start-up with
`DEVICE_TREE_ERROR code=unmodelled`.

### Add a missing built-in chip model

A chip model implements the chip, not the controller that carries it. Add a
source file under `src/models/` containing its reset state, register or command
state machine, and datasheet-visible output formatting. Export only the
appropriate peripheral operations from `AthrillDeviceModelOps` and keep
target-private headers and STM32 controller register bits out of the model.

Use the common channel engine and the SPI, I2C, UART, or DMA protocol plug-in
for selection, byte movement, buffering, completion time, and notifications.
For example, an SPI chip handles select, exchange, and deselect events; it does
not decode STM32 SPI registers or own a FIFO that duplicates the channel
engine. A sensor chip reads the typed sample source and converts values into
the chip's register layout; physical sensor simulation remains outside the
chip.

Declare the model in `src/models/models.h`, add its source to `CMakeLists.txt`,
and add its exact public compatible string to the built-in registry in
`src/athrill_device.c`. Reuse an existing vendor binding when it describes the
same part. Otherwise add an Athrill binding with the real vendor prefix; do not
label one part as a similar but different chip.

Test reset values, identity registers, read/write masks, command sequencing,
and one representative sample-to-register conversion. Test the bus protocol
through the channel engine separately from the chip register tests.

### Supply a model plugin

A plugin library exports the single data symbol `athrill_device_plugin_v1` as
an `AthrillDevicePlugin`. Its registrations use the same model operations as a
built-in. The library name is not searched implicitly: map each compatible to
one library with a contiguous, zero-based
`DEVICE_CONFIG_DEVICE_PLUGIN_<n>` entry in `device_config.txt`. Enumeration
stops at the first missing index.

The configured value has the form
`compatible=library-path`. Write the compatible's comma as `%2C`, because the
Athrill configuration tokenizer treats a literal comma as a field separator.
The loader checks the plugin magic, ABI, structure
sizes, names, versions, callbacks, and exported compatible. A plugin cannot
replace a built-in compatible, and two plugins cannot claim the same
compatible. Keep the library loaded for the lifetime of every model instance.

The Hakoniwa sensor library still also exports the legacy EXDEV ABI. A board
built from its DTB does not use it: map `hakoniwa,sensor-device` with a plugin
entry and add no `DEV` line.

## 5. Run the board

`DEVICE_CONFIG_DEVICE_TREE` in `device_config.txt` names the DTB. The path and
all host-resource paths are resolved from Athrill's working directory. Without
the key no board devices are built: the STM32 target stops with an error if
`DEVICE_CONFIG_STM32_BOARD` is set, and the RISC-V target runs only the hart,
its local interrupts, and the PLIC.

Host-only values such as file paths, TCP ports, and Hakoniwa asset and channel
names remain outside the DTB. Each target reads its own keys from
`device_config.txt` and hands them to the models; the keys are listed in the
target's configuration reference (athrill-target-stm32 and
athrill-target-riscv `docs/configuration.md`).

The following is a complete board-selection portion for the Pixhawk 4 base
DTB (PX4 SIH and bench runs) when Athrill starts in the athrill-device
repository root:

```text
DEVICE_CPU_FREQ 216
DEVICE_CONFIG_STM32_BOOT_ADDR 0x08008000
DEVICE_CONFIG_STM32_BOARD fmu-v5
DEVICE_CONFIG_DEVICE_TREE boards/pixhawk4/pixhawk4.dtb
DEVICE_CONFIG_STM32_FRAM_FILE fram.bin
DEVICE_CONFIG_STM32_SDCARD_IMAGE sd.img
DEVICE_CONFIG_STM32_ACTUATOR_FILE actuators.txt
```

On STM32, `DEVICE_CONFIG_STM32_BOARD` selects only the SoC profile: `fmu-v5`
for STM32F7 and `fmu-v6c` for STM32H7. The devices and wiring come from the
DTB. Pixhawk 6C uses `fmu-v6c` with `boards/pixhawk6c/pixhawk6c.dtb`.
The NUCLEO boards run Zephyr applications without this key; their complete
`device_config.txt` and memory map are written by athrill-target-stm32
`examples/zephyr/run.sh`.

A Hakoniwa run uses the `-hakoniwa` DTB, the plugin mapping from section 4,
and the `HAKO_SENSOR_DEVICE_*` keys described in this repository's
`docs/configuration.md`. Erbium uses `boards/erbium/erbium.dtb`; its UART
input file and TCP port are the RISC-V target's `DEVICE_CONFIG_ERBIUM_*` keys.

On successful startup, Athrill prints one graph line with the DTB's SHA-256
and size. The checked-in blobs produce:

| DTB | Startup line |
| --- | --- |
| `boards/pixhawk4/pixhawk4.dtb` | `DEVICE_TREE sha256=cd3a21918920144f266486817916342f5f841f551bbfde0e9576f5a95f28b412 size=26140` |
| `boards/pixhawk4/pixhawk4-hakoniwa.dtb` | `DEVICE_TREE sha256=60c9e2dd35d89b919c39a1bb169ce1ab15929c6e52b1be439f959a653ea28fe0 size=26229` |
| `boards/pixhawk4/pixhawk4-hakoniwa-dshot.dtb` | `DEVICE_TREE sha256=338872bd171ac2cc4c037d5174c5d9422ddc93d4c73cc6be91971e70a369fef3 size=26557` |
| `boards/pixhawk4/pixhawk4-hakoniwa-dshot-asset.dtb` | `DEVICE_TREE sha256=a908983bd499399b7c82035c71adb259a0121cf6fbebcff0072070332ddcc8cc size=26541` |
| `boards/pixhawk6c/pixhawk6c.dtb` | `DEVICE_TREE sha256=669120919cd12ce860f55edf014a6b6704e8b4947496a3aa7ca262e8a41c7093 size=31323` |
| `boards/pixhawk6c/pixhawk6c-hakoniwa.dtb` | `DEVICE_TREE sha256=95460b4820b1527c8a731a00443ca02227cc7e9c5ccd01fc2e94415d362ccac2 size=31436` |
| `boards/pixhawk6c/pixhawk6c-hakoniwa-dshot.dtb` | `DEVICE_TREE sha256=cc7cb4bd211ac6106050c0c8cdeb47459b7ce8b99d9882986085abc02cddbe50 size=31764` |
| `boards/nucleo-f767zi/nucleo-f767zi.dtb` | `DEVICE_TREE sha256=6a72e8dccf8e54f2c65f53beaa706622d7834a437920e77f0e8fa962ef6474e3 size=29491` |
| `boards/nucleo-h743zi/nucleo-h743zi.dtb` | `DEVICE_TREE sha256=f14668b555edd421fe21b99294b854a47c676fd7d19c1f7730de2102b6f10c0f size=34742` |
| `boards/erbium/erbium.dtb` | `DEVICE_TREE sha256=74d0308a9d6ded43a601f10e6b6be893b77acb5bee4a89259b111a15ee1afda8 size=1969` |
| `boards/erbium/erbium-hakoniwa.dtb` | `DEVICE_TREE sha256=1f27925e82d2a2441860445af2747aff41544a94dd7376e30305f16dcbd8176f size=2097` |

Each present host-only value is then logged as
`DEVICE_TREE_HOST sha256=... key=... value=...`, followed by one stable
`DEVICE_MODEL` line per instantiated model with its path, matched compatible,
provider, model name, and version. Require the startup hash to equal both the
board README and a fresh `sha256sum` of the file. A mismatch means the run did
not use the reviewed DTB.

## 6. Check the board

Do not accept a new board because it merely boots. Check it from allocation to
flight:

1. Generate board-allocation expectations from the reviewed board definition
   and keep them as a board test. Assert every enabled MMIO range and IRQ, each
   SPI chip-select and I2C address, every DMA request, GPIO line, UART child,
   SD host, and ordered PWM channel. Keep the expected hardware facts visible
   in the test so it can catch an incorrect DTS rather than reproducing it.
   Pixhawk 6C tests must include H7 TIM15, TIM16, and TIM17 IRQs 116, 117, and
   118.
2. Run the framework and protocol tests:

   ```bash
   set -eu
   cmake -S . -B build-test -DCMAKE_BUILD_TYPE=Debug
   cmake --build build-test -j
   ctest --test-dir build-test --output-on-failure
   ```

3. Add representative chip tests. Check identification and reset registers,
   one write/read sequence, interrupt behaviour, and a known physical sample
   encoded into the datasheet register format. For Pixhawk 6C, cover the
   ICM-42688-P and both BMI055 dies as well as H7-only SPI, DMAMUX/BDMA, and
   SDMMC v2 front ends.
4. If the board replaces an existing implementation (the Pixhawk boards
   replaced C-built boards, and Erbium replaced the RISC-V target's C
   devices), run the same firmware with the same host configuration, initial
   F-RAM, SD image, and sensor input on both. Compare instruction counts, console output, trace records, F-RAM,
   the complete SD image, and flight logs. For Pixhawk 6C, require the
   60-virtual-second in-process PIL trace to be identical for PX4 and
   ArduCopter and report the first differing record if it is not.
5. Boot each supported firmware and then fly the full Hakoniwa scenario. Check
   sensor and GPS timing, arming, actuator ordering, takeoff, hover, landing,
   and deterministic reruns.

A board pull request should contain the DTS, rebuilt DTB, board README with
the pinned upstream revision and SHA-256, provenance and licence notes, any
new bindings or models, and the allocation and representative-behaviour
tests. Its description should state what the code does and report the exact
test, equivalence, boot, and flight results. Include the firmware revisions,
DTB hash, model versions, host inputs, instruction counts, and artifact hashes
needed to reproduce the comparison. Do not rely on private paths, unpublished
sources, or tracking references.
