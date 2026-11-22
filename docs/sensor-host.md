# Sensor host: the board's sensor chips fed from the vehicle's true motion

This is the design of the sensor host in the in-process build of the Hakoniwa sensor device
([in-process aircraft](in-process-aircraft.md)). The aircraft gives only the vehicle's true motion. The sensor
device makes the values of every sensor chip on the board from it, one chip at a time. The firmware then reads the
chips with its own drivers.

## Why

Without the sensor host, Hakoniwa Drone's sensor models measure one IMU, one magnetometer, one barometer and one GPS,
and the sensor device feeds the same sample to every chip of the board. A board with two IMUs (the Pixhawk 6C has a
BMI055 and an ICM-42688-P) then sees the same noise and the same mounting position twice. The values also change only
at the aircraft's SITL sensor period, while the firmware reads the chips at the rate it programs into them.

With the sensor host each chip gets its own sensor instance: its own mount, its own error model and its own random
stream, measured on every physics step.

## The pipeline

The order of the stages is Hakoniwa Drone's sensor specification
([hakoniwa-drone-core `docs/architecture/aircraft/sensor`](https://github.com/toppers/hakoniwa-drone-core/tree/main/docs/architecture/aircraft/sensor), chapter 4):

```text
truth (MuJoCo) → mounting pose (position → orientation) → delay → measure → noise → drift → clamp → output
```

| Stage | Done by |
| --- | --- |
| The truth: the body origin's position, velocity, attitude, angular velocity, angular acceleration and linear acceleration; the rotor vibration at the IMU point | Hakoniwa Drone, with its sensor models off (`simulation.sensorModels: false`) |
| The mounting pose and the truth at the sensor: lever arm with the true angular motion, then the vibration, then the rotation; the earth field in the sensor axes; the antenna on WGS-84; the barometer's height | hakoniwa-sensor-adapter's host helpers (`include/hakoniwa/sensor_adapter/host/sensor_host.hpp`) |
| delay → measure → noise → drift → clamp (with scale and cross-axis errors, the IMU filter, zero offset, resolution) | the sensor adapter of each instance, chosen by implementation name (hakoniwa-sensor-adapter `SensorAdapterImplementation`) |
| The chip as the firmware programs it: registers, range, output data rate, FIFO, data-ready, bus | the chip models of this repository |

The rotor vibration is a body-axis vector at the IMU point. All chips on the flight controller use the same one.

## Mount

| What | Where it is written |
| --- | --- |
| Where the sensor is, from the body origin (position, m) and how it is turned on the vehicle (orientation) | the sensor instance's configuration. Its meaning is Hakoniwa Drone's `mount` (`positionM` in body FRD, `orientationDeg`). Absent: at the origin, aligned with the body |
| How the chip sits on the real board (rotation only) | the board's Devicetree, `athrill,mount-rotation` of the chip node |

The board files of the firmware give only each chip's rotation (PX4 `-R`, ArduPilot `hwdef` `ROTATION_…`), and the
firmware undoes it itself; no firmware knows the chips' positions. The chip's orientation is therefore the instance's
orientation composed with the board's rotation, and its position is the instance's. The autopilot's own settings for
the mount (PX4 `SENS_BOARD_ROT`, `EKF2_IMU_POS_*`; ArduPilot `AHRS_ORIENTATION`, `INS_POS*_*`) are made from the
instance's mount by the same rules as for SITL.

## Timing

- **One rhythm: the physics step.** On every physics step the sensor host reads the truth once and calls every
  instance's adapter once. Each adapter measures at its own `updatePeriodMsec` boundaries and holds the value in
  between. The chip model reads the value at the exact time the firmware-programmed rate requires, interpolating
  between retained physics-step values when that time lies between them.
- **The physics step must not be longer than any sensor's period.** A sensor cannot be measured
  more often than the physics steps. In practice the fastest IMU sets the physics step: PX4 runs the Pixhawk 4's
  ICM-20689 gyro at 8 kHz, so that board needs a physics step of 125 us or less. The Pixhawk 6C's ICM-42688-P also
  runs at 8 kHz under PX4. See Chip rates.
  - At start-up, an adapter whose `updatePeriodMsec` is shorter than the physics step is an error.
  - A configured adapter period that is not a multiple of the physics step is rounded to the nearest multiple with a
    warning, as in Hakoniwa Drone.
  - If the chip period itself is not a whole number of physics steps, the adapter continues once per physics step and
    the chip values at its exact sample times are interpolated from the retained adapter values.
  - When the firmware programs a chip's output data rate faster than the physics rate, the chip model reports it.
- **The chip comes first.** What the firmware sets in the chip's registers (range, output data rate, resolution) is
  applied by the chip model. For whole physics-step multiples the adapter's period follows the chip's output data
  rate; otherwise the adapter stays at the physics step and the chip interpolates (Chip rates). Settings that
  the chip model does not have come from the adapter's configuration, which the user fills from the chip's datasheet;
  a range or resolution written there as well is applied twice, so it is left to the chip.
- **Δt is the adapter cadence.** With a noise density, σ = noiseDensity / √Δt, where Δt is the adapter's
  `updatePeriodMsec`. That period follows a whole-multiple chip period; for a chip period between physics steps it is
  the physics step, and the chip reads the resulting values by interpolation at its exact cadence.
- Hakoniwa Drone's `softDelayMsec` (the autopilot's handling delay) is not used: the real firmware's own handling is
  that delay.

## Chip rates

The rule: **the firmware-programmed chip period is exact.** When it is a whole number of physics steps, the host makes
the sensor adapter follow. When it falls between physics steps, the adapter remains at one sample per physics step and
the chip uses interpolated values at its exact sample times.

- The chip models take the output data rate and the full scale from the registers the firmware writes, with the
  datasheets' tables (`spec/chips.yaml` ADV-CHIP-BMI055A-002, ADV-CHIP-BMI055G-002, ADV-CHIP-ICM42688-002 and -003).
  No rate is fixed in the models.
- When a chip starts producing samples (its FIFO runs with its interrupt), and each time its rate changes while it
  does, it tells its sensor instance the period (ABI 1.22, `AthrillDeviceSamplePeriodOps`), per part: accelerometer
  and gyro.
- The sensor host checks it before the next physics step (hakoniwa-sensor-adapter `docs/spec/update-period.md`):

| The chip's period | What the host does | Reported as |
| --- | --- | --- |
| equal to the adapter's | nothing | INFO |
| a whole number of physics steps, different from the adapter's | asks the adapter to measure at the chip's period from its next multiple | INFO when it follows; ERROR with the adapter's reason when it can not (an adapter without the call can not) |
| finer than the physics step | nothing: the physics step is fixed for the run, so the chip repeats values | ERROR, with the physics step to use (`HAKO_AIRCRAFT_PHYSICS_STEP_USEC`) |
| at least one physics step but not a whole number of steps | keeps the adapter at one sample per physics step; the chip reads interpolated values at its exact sample times | INFO naming both periods and interpolation |
| a reserved register setting | nothing | ERROR |

For an IMU adapter with one shared accelerometer/gyro period, the host first asks to change the reported part. If
that is not supported, it changes the shared period when the other part is not known yet or has reported the same
period. This is how the AirSim adapter follows a BMI055 whose two parts are both at 2 kHz, or an ICM-42688-P whose
two parts are both at 8 kHz. If the two chip parts differ, the host does not change the shared period back and forth:
it reports an ERROR naming the instance, part, chip period, adapter period and the adapter's reason. A fractional
chip period is the exception: its shared target is always the physics step, so a rate change restores that cadence
before interpolation even while the other part's new report is pending.

The reference AirSim IMU adapter implements `part_update_period_ns`, `set_part_update_period_ns` and the shared
`set_update_period_ns`. Its part setter deliberately refuses an independent change because its two parts share one
period; the shared fallback above is therefore required. A plugin built from an older AirSim tree without the
update-period implementation cannot follow and is reported as an ERROR.

So the physics step is chosen at start-up for the finest rate the firmware will program. Adapters follow whole-step
chip periods; for periods between steps they remain at the physics rate. What the firmwares program on the supported
boards:

| Board / chip | PX4 v1.17.0 | ArduPilot |
| --- | --- | --- |
| Pixhawk 4 ICM-20689 | 8 kHz gyro, 4 kHz accelerometer; ±2000 dps, ±16 g | firmware-dependent |
| Pixhawk 4 and 6C BMI055 accelerometer and gyro | 2 kHz each (unfiltered data) | 2 kHz each |
| Pixhawk 6C ICM-42688-P | 8 kHz, ±16 g, ±2000 dps | 1 kHz, or 2, 4 or 8 kHz with fast sampling (2 kHz observed with ArduCopter on the 6C at its 400 Hz loop); ±16 g, ±2000 dps |

Thus PX4 on FMUv5 needs `HAKO_AIRCRAFT_PHYSICS_STEP_USEC=125`: the 125 us step equals the ICM-20689 gyro period,
divides its 250 us accelerometer period and the BMI055's 500 us period, and lets the adapters follow all three rates.

An implementation linked into the plugin (`HAKO_SENSOR_DEVICE_IMPLEMENTATIONS`) is built against the same
hakoniwa-sensor-adapter interface as the plugin, and built again whenever that interface changes: the interface is C++,
so a library built against an older one fails at run time.

## The physics step from outside

`HAKO_AIRCRAFT_PHYSICS_STEP_USEC` sets the physics step in microseconds. The sensor device applies it right after the
airframe's configuration is read, before the aircraft is made (`DroneConfig::setSimTimeStep`). Every part of the
aircraft takes its step from it, and it also replaces the MuJoCo model's `<option timestep>`. Both replacements are
reported as INFO. Without it the airframe's `simulation.timeStep` is used, and it must equal the MuJoCo model's
timestep.

MuJoCo steps the whole world with one timestep, so every aircraft in one world has the same physics step.

## Configuration

| Key | Meaning |
| --- | --- |
| `HAKO_AIRCRAFT_PHYSICS_STEP_USEC` | the physics step [us] over the airframe's own |
| `HAKO_SENSOR_DEVICE_TRUTH_LOG` | a file for the truth check: on every sample of Hakoniwa Drone's sensor models, their accelerometer and gyro beside the host helpers' ideal IMU at the body origin (both FRD). With an error-free airframe the two agree |

| `HAKO_SENSOR_DEVICE_SENSORS` | the sensor instances: a JSON file `{"sensors": [ ... ]}`, one entry per instance with `name`, `type` (`imu`, `mag`, `baro`, `gps`), `implementation` (the adapters' implementation name), `config` (the adapter's configuration, relative to this file) and an optional `mount` (`positionM` in body FRD [m], `orientationDeg` `roll`/`pitch`/`yaw`). Each instance is reported as INFO at start-up |

### Wiring the chips to their instances

A sensor instance is a Devicetree node with compatible `hakoniwa,sensor-instance`, a child of the board's
`hakoniwa,sensor-device` node. Its node name is its `name` in `HAKO_SENSOR_DEVICE_SENSORS`; a node without an entry
there stops the start-up. A chip names its instance with `value-source`. One chip with two functions (the BMI055's
accelerometer and gyro) has one instance. The motors stay on the sensor device.

The run loads the plugin for the instances too: next to `DEVICE_CONFIG_DEVICE_PLUGIN_0` for
`hakoniwa%2Csensor-device`, a `DEVICE_CONFIG_DEVICE_PLUGIN_1` for `hakoniwa%2Csensor-instance` with the same library.

### Wiring consistency

The runtime checks the sensor-host wiring before the firmware starts:

- If `HAKO_SENSOR_DEVICE_SENSORS` is set but no enabled sensor chip's `value-source` points to a
  `hakoniwa,sensor-instance`, construction stops. This normally means the board used the ordinary Hakoniwa DTB
  instead of its `-sensors` DTB.
- If any sensor-instance node exists, every enabled sensor chip must point to an instance. A chip still pointing to
  `hakoniwa,sensor-device` stops construction with an error naming the chip and both compatibles. Actuator consumers
  such as the DShot ESC continue to point to the sensor device.
- If an instance node has no plugin mapping, the error names the line to add:
  `DEVICE_CONFIG_DEVICE_PLUGIN_1 hakoniwa%2Csensor-instance=<same library as DEVICE_CONFIG_DEVICE_PLUGIN_0>`.

The chip-period contract currently changes IMU accelerometer (kind 0) and
gyro (kind 1) adapter periods only. Pressure (kind 2), magnetometer (kind 3)
and unknown future non-IMU kinds are logged as informational and ignored;
they do not change an IMU period or fail the run.

These are errors, not warnings: each setup would otherwise run while measuring a different signal from the one the
sensor-instance configuration describes. The first programmed IMU FIFO period is checked before the next physics
step. For a whole-multiple chip period, a slower adapter follows the chip or produces an ERROR that names both
periods; for a fractional chip period, it returns to the physics cadence for interpolation.

The ready-made DShot boards wired this way are:

| Board | Devicetree | Instances |
| --- | --- | --- |
| Pixhawk 4 / FMUv5 | `boards/pixhawk4/pixhawk4-hakoniwa-dshot-sensors.dtb` | `icm20689`, `bmi055`, `barometer`, `magnetometer`, `gps1` |
| Pixhawk 6C / FMUv6C | `boards/pixhawk6c/pixhawk6c-hakoniwa-dshot-sensors.dtb` | `bmi055`, `icm42688p`, `barometer`, `magnetometer`, `gps1` |
| Pixhawk 6X revision 6 / FMUv6X | `boards/pixhawk6x/pixhawk6x-rev6-hakoniwa-dshot-sensors.dtb` | `adis16470`, `icp201xx`, `bmm150`, `ist8310`, `gps1` |

For Pixhawk 4, a `sensors.json` suitable for `HAKO_SENSOR_DEVICE_SENSORS` is, with the named AirSim adapter
configuration files beside it:

```json
{"sensors": [
  {"name": "icm20689",    "type": "imu",  "implementation": "airsim", "config": "imu-ideal.json"},
  {"name": "bmi055",      "type": "imu",  "implementation": "airsim", "config": "imu-ideal.json"},
  {"name": "barometer",    "type": "baro", "implementation": "airsim", "config": "baro-ideal.json"},
  {"name": "magnetometer", "type": "mag",  "implementation": "airsim", "config": "mag-ideal.json"},
  {"name": "gps1",         "type": "gps",  "implementation": "airsim", "config": "gps-default.json"}
]}
```

The corresponding device configuration includes the sensor-instance plugin alongside the sensor-device plugin
(replace the library path with the run's plugin):

```text
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-device=/path/to/libhako_sensor_device.dylib
DEVICE_CONFIG_DEVICE_PLUGIN_1 hakoniwa%2Csensor-instance=/path/to/libhako_sensor_device.dylib
```

### Adapter implementations

An instance's `implementation` names one of the implementations built into the plugin. A name that is not built in,
or an implementation without an adapter for the instance's type, stops the start-up. So does an adapter that refuses
its configuration, for example a period shorter than the physics step or not a multiple of it.

| Name | Built in with |
| --- | --- |
| `airsim` | `HAKONIWA_SENSOR_ADAPTER_AIRSIM_DIR`: the hakoniwa-sensor-adapter-airsim tree (IMU, magnetometer, barometer, GPS; needs the Boost headers). Its `config/*-ideal.json` files measure without errors |
| any other | `HAKO_SENSOR_DEVICE_IMPLEMENTATIONS`: the names of implementations built elsewhere (each defines `HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(name)`), with `HAKO_SENSOR_DEVICE_IMPLEMENTATION_LIBRARIES`: the static libraries that hold them, built position-independent with the same compiler |

Each adapter gets the instance's name as its sensor name and a fixed random seed made from that name, so every chip
has its own random stream and a run with the same configuration repeats.

### What the chip receives

The host turns each adapter's output into the units the chip models take: the sensor axes in FRD, the magnetic field
in gauss, the pressure in hPa, the GPS altitude above mean sea level.

- An IMU has no thermal model: its temperature is the sea-level temperature of the world.
- A barometer gives its own temperature when its adapter has one.
- A GPS adapter that gives no satellite count reports 10 satellites with a 3D fix.
- The world is the airframe's `simulation.location`: the origin, its altitude, and the earth's magnetic field there.

The autopilot's allowance for a sensor's delay must be the delay of that instance's adapter, by the same rule as for
SITL: for PX4, `EKF2_BARO_DELAY` and `EKF2_GPS_DELAY`. The x500 parameters hold 60 ms and 100 ms, the delays of
Hakoniwa Drone's barometer and GPS models; with adapters without delay they are 0.

The chip reads the values of the sample the sensor device took last. With sensor instances the sensor device takes
a sample, and gives the motors their commands, on every physics step, in place of the aircraft's SITL sensor period
(`simulation.sitl.sensor_period_msec`; reported as INFO). So a chip sees each measurement in the physics step its
adapter makes it, and the motors follow the firmware's output within one physics step.

The plugin is built with `HAKO_SENSOR_DEVICE_SENSOR_HOST=ON` and `HAKONIWA_SENSOR_ADAPTER_DIR` (the
hakoniwa-sensor-adapter tree). Without it, nothing of this section exists and the sensor device works as before.

## Logs

With `simulation.sensorModels: false`, Hakoniwa Drone writes no sensor logs (`log_acc.csv` and the others). Their
place is taken by the sensor logs of hakoniwa-sensor-adapter (`docs/spec/sensor-log.md`): the sensor host creates
every instance's adapter through them, so any implementation can be logged. An instance is logged when its `config`
is a sensor log configuration:

```json
{"model": "imu-ideal.json", "log": {"path": "logs/bmi055.bin", "format": "binary"}}
```

`model` is the implementation's own configuration for the sensor. The log has one row per physics step: the
simulation time, the truth at the sensor's mount, the adapter's output (sensor axes FLU, SI units), then the
adapter's own columns. The format is CSV or Hakoniwa Drone's binary log; the reader is hakoniwa-sensor-adapter's
`tools/read_sensor_log.py`. The firmware's own log, a ULog or a DataFlash log, holds what the firmware read through
the chip.

## Scope

- In process only. Over the Hakoniwa link the aircraft runs in another process, and the true motion and the vibration
  would have to cross as PDUs.
- The sensor host is chosen per run; the sensor device's previous behaviour (Hakoniwa Drone's sensor models, one
  sample for every chip) stays the default.
- Random seeds: one per instance, made from its name (see Adapter implementations).

## Requirements

| ID | Requirement |
|---|---|
| ADV-SENSOR-HOST-001 | With `HAKO_SENSOR_DEVICE_SENSORS`, at least one enabled sensor chip is wired by `value-source` to a `hakoniwa,sensor-instance`; otherwise construction stops with an actionable error. |
| ADV-SENSOR-HOST-002 | When instance nodes exist, no enabled sensor chip remains wired to `hakoniwa,sensor-device`; an error names the chip and both compatibles. |
| ADV-SENSOR-HOST-003 | An operating IMU FIFO reports each part's programmed period; whole-step periods make the adapter follow, fractional periods use a physics-step adapter with interpolated chip values, and finer-than-physics periods are errors. |
| ADV-SENSOR-HOST-004 | A sensor-instance node without its plugin mapping stops with a message naming the `DEVICE_CONFIG_DEVICE_PLUGIN_1 hakoniwa%2Csensor-instance` line. |
| ADV-SENSOR-HOST-005 | Pressure, magnetometer and unknown future non-IMU sample-period reports are informational and harmless; they are not treated as accelerometer periods. |
