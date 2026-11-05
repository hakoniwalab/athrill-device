# athrill-device

Virtual devices that connect firmware running on [Athrill](https://github.com/toppers/athrill) to the
[Hakoniwa](https://github.com/toppers/hakoniwa-drone-core) simulation.

The first device is the **Hakoniwa sensor device**. With it, unmodified PX4 firmware on Athrill flies the
Hakoniwa Drone x500 in MuJoCo: Processor-in-the-Loop (PIL).

```
  Hakoniwa Drone (aircraft service)        Athrill
  ┌─────────────────────────┐   shared    ┌──────────────────────────────┐
  │ physics (MuJoCo)         │   memory   │ Hakoniwa sensor device        │
  │ sensor model              │ ─────────▶ │ (this repository, an EXDEV)   │
  │                          │ hil_sensor │   ↓ registers / chips          │
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

## Layout

| Path | What |
| --- | --- |
| `board/common/` | The sensor device, common to all boards: an Athrill external device (EXDEV) and a Hakoniwa polling asset, with the register map `include/hako_sensor_device_regs.h` |
| `board/stm32f7xx/` | [Pixhawk 4 (STM32F765)](board/stm32f7xx/README.md) on Athrill STM32. The firmware reads the emulated chips with its own drivers, so **PX4 and ArduPilot** both work unchanged |
| `board/erbium/` | [AIFoundry Erbium (RISC-V)](board/erbium/README.md) on Athrill RISC-V. No chips: a PX4 driver reads the registers (**PX4 only**) |

## Build the device

Put the repositories side by side, as the defaults expect:

```text
work/
├── athrill/                  # Athrill core (toppers/athrill v3.0.0 or later): EXDEV headers
├── hakoniwa-pdu-registry/    # PDU C types (hako_mavlink_msgs)
└── athrill-device/           # this repository
```

Build against the same Hakoniwa Foundation as Hakoniwa Drone. With the Business Pack, its install prefix is
`hakoniwa-business-pack/work/foundation/install`; set `HAKONIWA_FOUNDATION_PREFIX` to that path. `ATHRILL_ADDRESS_WIDTH` is the width of the
`athrill3` that loads the device: 32 for STM32, 64 for RISC-V.

```bash
cd athrill-device
cmake -S board/common -B build-stm32 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HAKONIWA_FOUNDATION_PREFIX" -DATHRILL_ADDRESS_WIDTH=32
cmake --build build-stm32 -j
```

The result is `build-stm32/libhako_sensor_device.dylib` (`.so` on Linux). For Erbium, use `build-erbium` and
`-DATHRILL_ADDRESS_WIDTH=64`. Other locations of the sources can be given with `-DATHRILL_SOURCE_DIR=...` and
`-DHAKO_PDU_REGISTRY_DIR=...`.

## Use it

**Hakoniwa Drone (v5.0.0 or later):**
- airframe `config/drone/x500-athrill-px4`;
- PDU definition `config/pdudef/drone-pdudef-athrill-px4.json` (channels 19 `hil_sensor`, 20 `hil_gps`,
  21 `hil_actuator`).

Hakoniwa Drone's aircraft service is the conductor, the only one. Start it first, then Athrill, then
`hako-cmd start`.

**Athrill:** the memory definition maps the device, and the device configuration names it. The address and the
lines differ by board; see the board's README.

To check the firmware's drivers without Hakoniwa, replace the channel and asset lines with
`HAKO_SENSOR_DEVICE_TEST_PERIOD 4000`. The device then gives a vehicle at rest every 4 ms, where Hakoniwa Drone's
x500 starts (place, pressure and magnetic field). PX4 stores a barometer offset and the magnetic declination at
its first boot, so a first boot with this test vehicle (to write the parameters, for example) leaves the values a
flight in Hakoniwa needs.

**Take-off over MAVLink:** the altitude of `MAV_CMD_NAV_TAKEOFF` is above mean sea level. Give the home altitude
plus the height to climb. QGroundControl does this for you.

## Checked

macOS (Apple M-series), Hakoniwa Drone v5.0.0 (sensor model v2), PX4 `px4_fmu-v5_default` on Athrill STM32 with
this device (32-bit): the x500 arms and takes off to 2.51 m (target 2.5 m).

## Licence

MIT (see `LICENSE`). `board/erbium/px4/erbium-hakoniwa.patch` changes files of
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), which are BSD-3-Clause; the patched files keep their licence.
