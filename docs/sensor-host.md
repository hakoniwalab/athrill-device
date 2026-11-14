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
  between. The chip model reads the latest value at the rate the firmware programs.
- **The physics step must not be longer than any sensor's period, and should divide it.** A sensor cannot be measured
  more often than the physics steps. In practice the fastest IMU sets the physics step: the Pixhawk 6C's BMI055 runs
  at 2 kHz under PX4, so the 6C needs a physics step of 0.5 ms or less.
  - At start-up, an adapter whose `updatePeriodMsec` is shorter than the physics step is an error.
  - A period that is not a multiple of the physics step is rounded to the nearest multiple with a warning, as in
    Hakoniwa Drone.
  - When the firmware programs a chip's output data rate faster than the physics rate, the chip model reports it.
- **The chip comes first.** What the firmware sets in the chip's registers (range, output data rate, resolution) is
  applied by the chip model, and the adapter's period follows the chip's output data rate (Chip rates). Settings that
  the chip model does not have come from the adapter's configuration, which the user fills from the chip's datasheet;
  a range or resolution written there as well is applied twice, so it is left to the chip.
- **Δt is the output data rate.** With a noise density, σ = noiseDensity / √Δt, where Δt is the period at which the
  autopilot reads the sensor: the chip's output data rate. The adapter's `updatePeriodMsec` is written as that period.
- Hakoniwa Drone's `softDelayMsec` (the autopilot's handling delay) is not used: the real firmware's own handling is
  that delay.

## Chip rates

The rule: **the period at which the firmware programs a chip to produce samples is the period of the chip's sensor
adapter.** The firmware decides; the host checks and, where it can, makes the adapter follow.

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
| not a whole number of physics steps | the adapter refuses it | ERROR |
| a reserved register setting | nothing | ERROR |

So the physics step is chosen at start-up for the finest rate the firmware will program, and the adapters' periods
need not be written for each firmware: they follow the chips. What the firmwares program on the Pixhawk 6C:

| Chip | PX4 v1.17.0 | ArduPilot |
| --- | --- | --- |
| BMI055 accelerometer and gyro | 2 kHz each (unfiltered data) | 2 kHz each |
| ICM-42688-P | 8 kHz, ±16 g, ±2000 dps | 1 kHz by default; 2, 4 or 8 kHz with fast sampling; ±16 g, ±2000 dps |

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

`boards/pixhawk6c/pixhawk6c-hakoniwa-dshot-sensors.dts` is the Pixhawk 6C with DShot wired this way, with the
instances `bmi055`, `icm42688p`, `barometer`, `magnetometer` and `gps1`.

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
