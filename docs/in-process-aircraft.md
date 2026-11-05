# In-process aircraft: Athrill flies the Hakoniwa Drone without a second process

This is the design of a second build of the Hakoniwa sensor device. It runs the Hakoniwa Drone aircraft (physics,
sensor models, actuator handling) inside the Athrill process. The firmware, the device registers and the timeline
stay the same as with the Hakoniwa link; only the transport between the device and the aircraft changes.

## Why

In Processor-in-the-Loop (PIL) with the Hakoniwa link, Athrill and the Hakoniwa Drone aircraft service run as two
processes in strict lockstep. Each side waits while the other runs. A profile of one 4 ms exchange (PX4 on Athrill
STM32, Apple M2 Pro, RTF 1.85) gave:

| Part of one exchange (2.16 ms of wall time) | ms |
| --- | ---: |
| Athrill: firmware and its devices | 0.26 |
| Aircraft step, 4 × 1 ms (MuJoCo itself: 0.07) | 0.21 |
| Device side of the Hakoniwa PDU access | 0.10 |
| Polling granularity and OS wake-up, two hand-overs | 0.03 |
| Aircraft service PDU traffic, conductor, barriers, thread scheduling | 1.55 |

The work that the flight needs is under 0.5 ms. Most of the rest is the cost of two processes meeting through
Hakoniwa at every exchange. Running the aircraft in the Athrill process removes that part. The expected RTF for
PX4 on Athrill STM32 is about 2.5 to 3 instead of 1.85; the prototype measures it.

## What stays the same

- The firmware, the board models and the sensor device's registers, interrupts and latching.
- The order of one exchange (Gauss-Seidel lockstep):
  1. the aircraft publishes a sensor sample with its time;
  2. the device runs the CPU to that time, latches the sample and raises data ready;
  3. the device hands the actuator at that time to the aircraft;
  4. the aircraft holds that actuator over its 1 ms steps up to the next sensor sample and publishes it.
- The aircraft itself: the same libraries and the same airframe configuration as the Hakoniwa Drone aircraft service.
  Nothing in the physics or the sensor models is re-implemented.

So, with the same configuration and the same random seed, every sensor sample and every actuator value, with its
timestamp, is the same as with the Hakoniwa link.

## Where it goes

The sensor device already reaches Hakoniwa through one small interface, `board/common/src/hako_sensor_device_link.h`:

| Function | Hakoniwa link (today) | In-process link (this design) |
| --- | --- | --- |
| `hako_sensor_link_initialize` | registers a Hakoniwa polling asset | loads the airframe configuration and creates the aircraft |
| `hako_sensor_link_poll_events` | START / STOP / RESET from Hakoniwa | always running |
| `hako_sensor_link_read` | reads `hil_sensor` / `hil_gps` PDUs | returns the aircraft's next sample; steps the aircraft first if needed |
| `hako_sensor_link_write_actuator` | writes the `hil_actuator` PDU | stores the actuator for the aircraft's next steps |
| `hako_sensor_link_notify_time` | reports the asset time to Hakoniwa | nothing |
| `hako_sensor_link_cleanup` | leaves Hakoniwa | destroys the aircraft |

The in-process link is a second implementation of this interface, `board/common/src/hako_sensor_device_link_inproc.cpp`.
The device code (`hako_sensor_device_exdev.c`, `hako_sensor_device.c`) does not change. The build makes a second
library, `libhako_sensor_device_inproc`, next to `libhako_sensor_device`. The Athrill memory definition chooses one of
the two, as it chooses the device today. The default library keeps no dependency on the aircraft libraries.

Nothing changes in the Athrill core or in the targets. STM32 and RISC-V get it the same way, because the device is
shared by all boards.

## The aircraft libraries

The in-process link uses the libraries and headers of the Hakoniwa Drone release (`hakoniwa-drone-core`), the same
ones the aircraft service is built from:

- `aircraft` (`IAirCraftContainer`): the airframe, its physics (MuJoCo) and its sensor models;
- `service` (`IAircraftServiceContainer`): the 1 ms stepping, the actuator hold over the sensor period, the sensor
  timestamps and the HIL message encoding;
- `mavlink` (`IMavLinkService`): the message boundary of the aircraft service.

The aircraft service talks to a flight controller through an `IMavLinkService`. Today that is TCP (SITL) or the
Hakoniwa shared-memory service (PIL). The in-process link adds a third one in this repository:

- `sendMessage(HIL_SENSOR / HIL_GPS)` from the aircraft service stores the message for `hako_sensor_link_read`;
- `hako_sensor_link_write_actuator` turns the actuator into `HIL_ACTUATOR_CONTROLS` and passes it to the aircraft
  service through `readMessage` and the receiver callback, which makes the aircraft step.

The messages are the same C structures as the PDUs (`hako_mavlink_msgs`), so the conversion code in
`hako_sensor_device_link.c` is reused as it is. The aircraft service's PDU syncher is a no-operation one: no
Hakoniwa PDU is written on the control path.

Builds link the release binaries by default. A Hakoniwa Drone PRO build has the same headers and can be linked
instead. The source in this repository is MIT. A built library that links the release binaries is under their
licence: free for non-commercial use, see the `hakoniwa-drone-core` release.

## Threads and timing

- The aircraft runs in the thread that calls the device, at the device's call. When the device asks for the next
  sample and the aircraft has not produced it yet, the aircraft steps (1 ms per step, the airframe's physics period)
  with the last actuator until the next sensor sample, then returns it. No other thread takes part in the control
  path, so no hand-over or scheduling is involved.
- If the aircraft service runs its stepping in a worker thread of its own, the in-process `IMavLinkService` waits on
  a condition variable for the sample. That costs about 2 µs per hand-over on macOS, against about 16 µs for the
  20 µs polling of the Hakoniwa link. The prototype checks which of the two the service needs.
- The device asks for samples only at sensor periods; between them, the CPU runs alone, as today.
- The asset-time notification has no receiver and does nothing.

## Viewer

The MuJoCo viewer (GLFW) has to run in the main thread on macOS, and the main thread of Athrill runs the CPU. The
design keeps the control path free of the viewer:

- The aircraft's thread copies the vehicle pose (position, attitude, rotor angles) into a small buffer under a mutex
  after each step. That is the only shared state.
- The viewer draws from that buffer at its own rate, with its own copy of the MuJoCo model. It never reads the
  aircraft's simulation data and never waits for it. A slow viewer cannot slow the flight or change it.
- Running the viewer in the main thread needs one entry point in the Athrill core: a device may register a
  main-thread function. If one is registered, the core runs the CPU loop in a worker thread and calls the function in
  the main thread until the run ends. Without it, nothing changes. This is the only core change, and it is optional:
  the in-process link works without a viewer (batch runs, measurement).

## Configuration

The device configuration of the in-process library takes:

| Key | Value |
| --- | --- |
| `HAKO_AIRCRAFT_CONFIG_PATH` | the airframe directory or `drone_config_0.json` of the Hakoniwa Drone, e.g. `x500-athrill-px4` |
| `HAKO_AIRCRAFT_VIEWER` | `0` (default) or `1` for the MuJoCo viewer (needs the core entry point above) |

The channel and asset lines of the Hakoniwa link are not used. The airframe configuration gives the sensor period,
the sensor models and the random seed (`simulation.randomSeed`), as for the aircraft service.

## Checks of the prototype

1. **Same flight.** PX4 on Athrill STM32, the Hakoniwa Drone x500, take-off to 2.5 m and hover, once with the
   Hakoniwa link and once in process, same configuration and seed. The device logs every latched sensor sample and
   every actuator value with its time; the two logs are identical over the whole flight.
2. **Repeatable.** Two in-process runs give identical logs.
3. **Speed.** RTF over the hover, no viewer, for PX4 on Athrill STM32, PX4 on Athrill RISC-V (Erbium) and ArduCopter
   on Athrill STM32, against the Hakoniwa link (1.85, 2.07 to 2.11, 1.16 to 1.26).
4. **The Hakoniwa link unchanged.** The default library and its PIL runs behave as before.
5. **Viewer.** With the viewer on, the logs of check 1 stay identical.

## When to use which

| | Hakoniwa link | In-process link |
| --- | --- | --- |
| Other Hakoniwa assets (other vehicles, tools, external viewers) | yes | no |
| Speed | the two processes meet at every exchange | no meeting; the firmware and the aircraft alternate in one thread |
| Same flight | reference | identical to the Hakoniwa link |
| Typical use | a vehicle in a larger Hakoniwa world | one vehicle: firmware tests, tuning, regression runs, CI |
