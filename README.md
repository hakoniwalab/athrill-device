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
| `include/hako_sensor_device_regs.h` | The register map, shared by the device and the PX4 driver |
| `hako-sensor-device/` | The device: an Athrill external device (EXDEV), a Hakoniwa polling asset |
| `px4/` | For the RISC-V target only: the PX4 driver `hako_sensor_device` (an external module) and the board variant patch `erbium-hakoniwa.patch` |

On the STM32 target ([Athrill STM32](https://github.com/hakoniwalab/athrill-target-stm32)), PX4's own drivers
read the emulated chips (ICM-20689, IST8310, MS5611, u-blox). The board model takes the samples from this
device and sends the PWM outputs back to it, so `px4/` is not needed there.

## Build the device

Put the repositories side by side, as the defaults expect:

```text
work/
├── athrill/                  # Athrill core (toppers/athrill), EXDEV headers
├── hakoniwa-pdu-registry/    # PDU C types (hako_mavlink_msgs)
└── athrill-device/           # this repository
```

Build against the same Hakoniwa Foundation as Hakoniwa Drone. Set `ATHRILL_ADDRESS_WIDTH` to the width of the
`athrill3` that loads the device: 32 for STM32, 64 for RISC-V.

```bash
cd athrill-device
cmake -S hako-sensor-device -B build-stm32 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HAKONIWA_FOUNDATION_PREFIX" -DATHRILL_ADDRESS_WIDTH=32
cmake --build build-stm32 -j
```

The result is `build-stm32/libhako_sensor_device.dylib` (`.so` on Linux). Other locations of the sources can be
given with `-DATHRILL_SOURCE_DIR=...` and `-DHAKO_PDU_REGISTRY_DIR=...`.

## Use it

**Hakoniwa Drone (v5.0.0 or later):**
- airframe `config/drone/x500-athrill-px4`;
- PDU definition `config/pdudef/drone-pdudef-athrill-px4.json` (channels 19 `hil_sensor`, 20 `hil_gps`,
  21 `hil_actuator`).

Hakoniwa Drone's aircraft service is the conductor. Start it first, then Athrill, then `hako-cmd start`.

**Athrill:** map the device in the memory definition and name it in the device configuration.

```text
DEV, 0x60000000, <path to libhako_sensor_device.dylib>
```

STM32 uses `0x60000000`; RISC-V (Erbium) uses `0x20000000`.

```text
HAKO_SENSOR_DEVICE_ASSET athrill
HAKO_SENSOR_DEVICE_ROBOT Drone
HAKO_SENSOR_DEVICE_SENSOR_CHANNEL 19
HAKO_SENSOR_DEVICE_GPS_CHANNEL 20
HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL 21
```

To check the firmware's drivers without Hakoniwa, replace the five lines with `HAKO_SENSOR_DEVICE_TEST_PERIOD 4000`.
The device then gives a vehicle at rest every 4 ms.

The complete STM32 procedure (memory, device configuration, first-boot parameters, run) is
[Athrill STM32: PX4 with Hakoniwa](https://github.com/hakoniwalab/athrill-target-stm32/blob/main/docs/hakoniwa.md).

**Take-off over MAVLink:** the altitude of `MAV_CMD_NAV_TAKEOFF` is above mean sea level. Give the home altitude
plus the height to climb. QGroundControl does this for you.

## Checked

macOS (Apple M-series), Hakoniwa Drone v5.0.0 (sensor model v2), PX4 `px4_fmu-v5_default` on Athrill STM32: the x500
arms and takes off to 2.51 m (target 2.5 m).

## Licence

MIT (see `LICENSE`). `px4/erbium-hakoniwa.patch` changes files of
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), which are BSD-3-Clause; the patched files keep their licence.
