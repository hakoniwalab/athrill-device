# Design: ticking only the models that need it (tick skip)

Status: experiment. Off by default: a build option and a run-time option are both needed.

## Why

Today `athrill_device_tick()` ticks every model of the board each time the target calls it. The target calls it:

- after every batch of instructions;
- after a device register access, which ends the batch.

Measured on the Pixhawk 4 board (39 ticked models), Athrill STM32 with its JIT:

| Firmware | Share of the emulation thread in device ticks |
|---|---|
| PX4 fmu-v5 bench | about 15% |
| ArduCopter (run-full) | about 32% |

Of the ArduCopter share, the loop over the models alone is about 12%. These are host-time shares from `sample`.

Most of these ticks do nothing:

- **The batches are short.** The target ticks about every 620 clocks on average. The models report their next event about 51,000 clocks ahead on average. Most batches end at a register access, not at a device event.
- **Many models never need a tick.** The 11 GPIO ports' tick returns "no event" and does nothing else.
- **A timer or UART tick before its next event changes nothing observable.** Its registers are computed from the clock when they are read.

## What changes

### 1. The model declares how its tick behaves (ABI 1.19)

`AthrillDeviceTickResult` gains a field:

```c
uint32 contract;   /* ATHRILL_DEVICE_TICK_EVERY (0, preset by the runtime), _NONE or _DEADLINE */
```

It is reported in every tick's result, not in `AthrillDeviceModelOps`. The reason is ABI compatibility:

- The ABI check accepts a model table only if its `struct_size` is at least the runtime's size. A new field in the model table would refuse every model built against an older header.
- The tick result is the runtime's own structure. The runtime presets `EVERY`, so a model that does not know the field keeps today's behaviour.

| Value | Meaning | The runtime |
|---|---|---|
| `ATHRILL_DEVICE_TICK_EVERY` (0) | As today: the tick may do something at any time. | Ticks it every time, as today. |
| `ATHRILL_DEVICE_TICK_NONE` | The tick never does anything. The model also reports no event and is never busy. | Does not tick it again until a reset. |
| `ATHRILL_DEVICE_TICK_DEADLINE` | Before the time of the event it last reported, a tick does nothing observable, unless the model asked for a tick since (see 2). | Ticks it when that time has come or after it asked; otherwise skips it, and still reports the remaining interval to the target. |

Rules:

- A model that reports `can_skip = FALSE`, or `NONE` together with an event, is treated as `EVERY`.
- After `athrill_device_reset()`, every model is `EVERY` again until its next tick reports otherwise.

### 2. A `DEADLINE` model asks for a tick when something changes it (ABI 1.19)

`AthrillDeviceResourceOps` gains `request_tick(context)`.

- A `DEADLINE` model calls it from every entry that can change when its next event is, or what its tick would do. For example: a register write, a byte arriving from its bus, a GPIO edge, a DMA request, a value set from outside.
- The runtime then ticks the model at the next `athrill_device_tick()`.
- Adopting `DEADLINE` is a review of all of the model's entries. A model that cannot be reviewed stays `EVERY`.

### 3. Where it is switched on

| Level | Switch | Effect when absent |
|---|---|---|
| Build | `ATHRILL_DEVICE_TICK_SKIP` (CMake option `ATHRILL_DEVICE_ENABLE_TICK_SKIP`, OFF) | The runtime code is today's code. The contract field and `request_tick` exist, but the runtime never reads them; `request_tick` does nothing. |
| Run | the host value `DEVICE_CONFIG_DEVICE_TICK_SKIP` = `1` (`ATHRILL_DEVICE_TICK_SKIP_HOST_VALUE`), which the target passes in `AthrillDeviceBuildOptions.host_values` from its configuration | Every model is ticked, as today. |

The tick order and the `DEVICE_TICK` start-up lines do not change. Skipping a model does not change the order in which the others are ticked.

## Order of adoption

| Step | Models | Contract | Why first |
|---|---|---|---|
| 1 | GPIO ports, other models whose tick is empty | `NONE` | Nothing to review: the tick has no body. |
| 2 | STM32 timers | `DEADLINE` | 8 nodes. Counter state is already computed from the clock on read. |
| 3 | STM32 UARTs without a TCP link | `DEADLINE` | 7 nodes. A UART linked over TCP polls the host on every tick, so it stays `EVERY`. |

Each step is a separate commit, measured and verified on its own.

## How it is verified

**The same results.** Each workload is run with the option on and off. The run's output must be byte-identical, and so must the executed instruction count and the final state.

| Workload | Output compared |
|---|---|
| PX4 fmu-v5 bench | console |
| PX4 SIH | its recorded run |
| ArduCopter run-full, idle executed and spin-halt | MAVLink bytes |
| Zephyr samples on the NUCLEO boards | console |
| athrill-device's own tests | the tests, run with the build option on and off |

**Per-model tests.** For every `DEADLINE` model, a test drives it through its entries and compares two schedules. One ticks every time; the other ticks only as the runtime would. The registers read back, the interrupts raised and the reported intervals must match. These tests are a candidate task for Codex.

**Speed.** The measurement is the same as for the JIT. For ArduCopter: the RTF over virtual 20 to 80 s. For PX4: the 12e9-clock bench.

## What does not change

- The model ABI for models that do nothing new: the new field is in the runtime's tick result, which old models never write.
- Targets that do not pass the run-time option.
- The default build.

## Measured

### Step 1 (GPIO ports `NONE`)

Athrill STM32 with its JIT, the same binary, the option off and on.

| Workload | Off | On | Output |
|---|---|---|---|
| PX4 fmu-v5 bench, 12e9 clocks | 19.90–20.13 s | 19.56–19.71 s | console and final state identical |
| ArduCopter run-full, 80 virtual s | 41.12 s | 40.57 s | MAVLink bytes, steps, final PC identical |

### Steps 2 and 3 (STM32 timers and UARTs `DEADLINE`)

Each model reports `DEADLINE` only when skipping its tick provably changes nothing.

**STM32 timer**
- Reports `DEADLINE` when no interrupt line is high.
- A high line must be ticked: it is pended again once its handler is no longer active, which the CPU decides.
- Asks for a tick on every register write.

**STM32 UART**
- Reports `DEADLINE` when:
  - it is not linked over TCP (that link is polled every tick);
  - no interrupt line is high;
  - nothing is being sent: the end of a frame writes the console or file and moves the DMA requests at a time that is reported as an event only when its interrupt is enabled.
- Asks for a tick on every register access, and on input from its chip or from the console.

Athrill STM32 with its JIT, the same binary, the option off and on:

| Workload | Off | On | Output |
|---|---|---|---|
| PX4 fmu-v5 bench, 12e9 clocks | 20.38 s | 18.55–18.62 s | console and final state identical |
| ArduCopter run-full, spin-halt, 20 / 80 virtual s | 9.74 / 41.27 s (RTF 1.90) | 8.76 / 37.18 s (RTF 2.11) | MAVLink bytes, steps, final PC identical |
| ArduCopter run-full, idle loop executed, 40 virtual s | 22.40 s | 19.52 s | MAVLink bytes, steps, final PC identical |

athrill-device ctest: 661/661 with the build option ON and OFF.
