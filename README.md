# athrill-device

Virtual devices that connect firmware running on [Athrill](https://github.com/toppers/athrill) to the
[Hakoniwa](https://github.com/toppers/hakoniwa-drone-core) simulation.

The first device is the **Hakoniwa sensor device**. With it, unmodified PX4 firmware on Athrill flies the
Hakoniwa Drone x500 in MuJoCo: Processor-in-the-Loop (PIL).

Every sensor-model parameter, including the Hakoniwa link, trace, and in-process aircraft,
is listed in the [configuration reference](docs/configuration.md).

The repository also provides the target-independent Devicetree device-graph
framework in `include/`, `src/`, and `third_party/libfdt/`.  A target opts in
with `DEVICE_CONFIG_DEVICE_TREE`; without that key its existing device path is
unchanged.  The framework validates a DTB, resolves compatible models and
explicit plugins, constructs them in stable dependency order, rolls back a
failed build, and logs the exact DTB SHA-256 plus host-only values.

Build and test the framework on its own:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`libfdt` is the read-only subset pinned to dtc v1.7.2 and used under its
BSD-2-Clause option; its retained notice is in
`LICENSES/BSD-2-Clause-libfdt.txt`.

The checked-in boards, each with its source and blob documented:

| Board | SoC | Firmware checked |
| --- | --- | --- |
| [`pixhawk4`](boards/pixhawk4/README.md) | STM32F765 | PX4, ArduPilot |
| [`pixhawk6c`](boards/pixhawk6c/README.md) | STM32H743 | PX4 |
| [`nucleo-f767zi`](boards/nucleo-f767zi/README.md) | STM32F767 | Zephyr samples, built for Zephyr's `nucleo_f767zi` |
| [`nucleo-h743zi`](boards/nucleo-h743zi/README.md) | STM32H743 | Zephyr samples, built for Zephyr's `nucleo_h743zi` |
| [`erbium`](boards/erbium/README.md) | Erbium (RV64) | PX4, and Zephyr samples built for Zephyr's `erbium` |

The nucleo boards include Zephyr's own board files unchanged. To add
another board, follow [Adding a board from a Devicetree](docs/adding-a-board.md).

```
  Hakoniwa Drone (aircraft service)        Athrill
  ┌─────────────────────────┐   shared    ┌──────────────────────────────┐
  │ physics (MuJoCo)         │   memory   │ Hakoniwa sensor device        │
  │ sensor model              │ ─────────▶ │ Devicetree model plugin       │
  │                          │ hil_sensor │   ↓ typed sources / chips      │
  │                          │ hil_gps    │ PX4 firmware (unmodified)      │
  │                          │ ◀───────── │   ↑ motor outputs (PWM)        │
  │                          │hil_actuator│                                │
  └─────────────────────────┘             └──────────────────────────────┘
```

Both sides move in lockstep on the data:
1. The sensor device waits for each `hil_sensor` sample.
2. It runs the CPU up to the sample's time and latches the sample, raising the data-ready interrupt.
3. It writes the motor outputs at that time to `hil_actuator`.
4. Hakoniwa Drone steps its physics on each `hil_actuator`.

Nothing depends on wall-clock time.

A second build of the device can run the Hakoniwa Drone aircraft inside the Athrill process instead, with the
same flight and no second process: see the design, [docs/in-process-aircraft.md](docs/in-process-aircraft.md).

## Layout

| Path | What |
| --- | --- |
| `board/common/` | The `hakoniwa,sensor-device` model plugin, common to all boards, with shared-memory and in-process link backends |
| `board/stm32f7xx/` | [Pixhawk 4 (STM32F765)](board/stm32f7xx/README.md) on Athrill STM32. The firmware reads the emulated chips with its own drivers, so **PX4 and ArduPilot** both work unchanged |
| `board/stm32h7xx/` | [Pixhawk 6C (STM32H743)](board/stm32h7xx/README.md) on Athrill STM32, using the same common device through the v6C sensor, GPS and FMU-output wiring |
| `board/erbium/` | [AIFoundry Erbium (RISC-V)](board/erbium/README.md) on Athrill RISC-V. No chips: a PX4 driver reads the registers (**PX4 only**) |

## Build the device

Put the repositories side by side, as the defaults expect:

```text
work/
├── hakoniwa-pdu-registry/    # PDU C types (hako_mavlink_msgs)
├── athrill/                  # core headers for the retained EXDEV ABI
└── athrill-device/           # this repository
```

Build against the same Hakoniwa Foundation as Hakoniwa Drone. With the Business Pack, its install prefix is
`hakoniwa-business-pack/work/foundation/install`; set `HAKONIWA_FOUNDATION_PREFIX` to that path.

```bash
cd athrill-device
cmake -S board/common -B build-stm32 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HAKONIWA_FOUNDATION_PREFIX" \
  -DATHRILL_SOURCE_DIR=/path/to/athrill -DATHRILL_ADDRESS_WIDTH=32
cmake --build build-stm32 -j
```

The result is `build-stm32/libhako_sensor_device.dylib` (`.so` on Linux).
Other locations of the PDU types can be given with `-DHAKO_PDU_REGISTRY_DIR=...`.
During the EXDEV migration window the same library exports both the model
plugin ABI and the legacy `athrill_ex_device` ABI. You must set
`ATHRILL_ADDRESS_WIDTH` to the bus width of the loading Athrill target (32
for STM32, 64 for the current RISC-V target); there is no cross-target
default. The EXDEV entry also rejects a width mismatch at load time, before
starting Hakoniwa or supplying sensor samples.

The macOS in-process prototype is an opt-in second target. It links the v5 release binaries; because that archive
omits some transitive public C++ headers, it also needs a matching v5 source tree for headers only:

```bash
cmake -S board/common -B build-stm32-inproc -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HAKONIWA_FOUNDATION_PREFIX" \
  -DHAKO_SENSOR_DEVICE_BUILD_INPROC=ON \
  -DATHRILL_SOURCE_DIR=/path/to/athrill -DATHRILL_ADDRESS_WIDTH=32 \
  -DHAKO_DRONE_ROOT=/path/to/hakoniwa-drone-core \
  -DHAKO_DRONE_SOURCE_DIR=/path/to/matching-hakoniwa-drone-v5-source
cmake --build build-stm32-inproc -j
```

This produces `libhako_sensor_device_inproc.dylib` beside the default library and copies its two runtime release
dependencies beside it.

## Use it

**Hakoniwa Drone (v5.0.0 or later):**
- airframe `config/drone/x500-athrill-px4`;
- PDU definition `config/pdudef/drone-pdudef-athrill-px4.json` (channels 19 `hil_sensor`, 20 `hil_gps`,
  21 `hil_actuator`).

Hakoniwa Drone's aircraft service is the conductor, the only one. Start it first, then Athrill, then
`hako-cmd start`.

**Athrill:** select `boards/pixhawk4/pixhawk4-hakoniwa.dtb` and explicitly map
`hakoniwa%2Csensor-device` to the plugin library in `device_config.txt`.
The memory definition needs no `DEV` line. Sensor chips and ordered PWM
actuators are connected to the model by phandles in the board DTB.

**Take-off over MAVLink:** the altitude of `MAV_CMD_NAV_TAKEOFF` is above mean sea level. Give the home altitude
plus the height to climb. QGroundControl does this for you.

For a prepared disposable copy of Athrill STM32's `examples/px4-hakoniwa`, the two launchers under `tools/` build
the mode-specific memory and device configuration, start the required processes, and wait. Run either script without arguments for its path list;
set `ATHRILL_TICKS` for a finite run and `HAKO_SENSOR_DEVICE_TRACE_PATH` to enable the exact trace. The Hakoniwa
launcher also honors `HAKO_CONFIG_PATH`. The in-process launcher defaults to `HAKO_AIRCRAFT_WORKER=1`; set it to
`0` to retain the serial stepping path for exact-result and performance comparisons.

## Checked

macOS (Apple M-series), Hakoniwa Drone v5.0.0 (sensor model v2), PX4 `px4_fmu-v5_default` on Athrill STM32 with
this model: the x500 arms and takes off to 2.51 m (target 2.5 m).

## Licence

MIT (see `LICENSE`). `board/erbium/px4/erbium-hakoniwa.patch` changes files of
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), which are BSD-3-Clause; the patched files keep their licence.
