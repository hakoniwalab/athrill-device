# Athrill device Recipes

`athrill-stm32-board-build` is the base Recipe for the STM32 target. It builds
one `athrill3` binary for both supported boards. The `board` argument selects
the DTB, firmware input, and smoke oracle; it does not select compiler flags.

The component-owned runner is the normal entry point:

```bash
python3 recipes/recipe.py plan \
  --board pixhawk4 \
  --firmware /path/to/px4_fmu-v5_default.elf \
  --business-pack-root /path/to/hakoniwa-business-pack

python3 recipes/recipe.py run \
  --board pixhawk4 \
  --firmware /path/to/px4_fmu-v5_default.elf \
  --business-pack-root /path/to/hakoniwa-business-pack
```

Use `pixhawk6c` with `px4_fmu-v6c_default.elf` for the other board. Set
`--work-dir /tmp/rc-...` when a disposable clean build is wanted. Otherwise
the runner uses `work/recipes/athrill-stm32-board-build/` in this repository.
`doctor` and `plan` are read-only. `configure` is the explicit source-clone
and Foundation-install step; `build`, `smoke`, and `install` are also exposed
separately. `run` executes that finite sequence end to end.

The exact source revisions are data in `pins/sources.json`. The Athrill core
and Arm-common values must match the pinned checkout entries in the selected
athrill-target-stm32 revision's CI workflow; `configure` checks this. Existing
source overrides are treated as user-owned input and are never updated,
reset, or replaced:

- `ATHRILL_ROOT`
- `ATHRILL_ARM_COMMON_ROOT`
- `ATHRILL_TARGET_STM32_ROOT`
- `HAKONIWA_PDU_REGISTRY_ROOT`

The normal sensor-device plugin needs the `hakoniwa-core-pro` CMake package
and the PDU registry C types. They are both declared by the Recipe. A selected
Business Pack lets `configure` install the Foundation package when it is
missing. The runner never installs host packages implicitly.

The mandatory plugin option is `ATHRILL_ADDRESS_WIDTH=32`. STM32 athrill3 has
a 32-bit EXDEV ABI; a 64-bit plugin can corrupt decoded addresses and has
historically produced silent GPS errors. The Receipt records this option.

Without `--hako-drone-root`, the normal shared-memory
`libhako_sensor_device` is built and the optional in-process library is
reported as omitted. On macOS, provide both an unpacked Hakoniwa Drone v5
release and its matching source tree to add
`libhako_sensor_device_inproc`:

```bash
python3 recipes/recipe.py run \
  --board pixhawk6c \
  --firmware /path/to/px4_fmu-v6c_default.elf \
  --business-pack-root /path/to/hakoniwa-business-pack \
  --hako-drone-root /path/to/unpacked/hakoniwa-drone-core \
  --hako-drone-source-dir /path/to/matching/hakoniwa-drone-v5-source
```

The release archive supplies executable code; the source tree supplies only
public headers omitted from the v5 archive. Its licensing terms therefore
apply to the linked in-process plugin. Omitting these inputs does not affect
the athrill3 build, the normal plugin, DTBs, or either board smoke.

## Firmware inputs

The smoke needs the unmodified PX4 v1.17.0 ELF with symbols, not a `.px4`
upload package. Obtain both measured ELFs from the
[athrill-target-stm32 v0.1.1 release](https://github.com/hakoniwalab/athrill-target-stm32/releases/tag/v0.1.1),
or build PX4 tag `v1.17.0` in the pinned build image:

```bash
git clone --branch v1.17.0 --depth 1 \
  https://github.com/PX4/PX4-Autopilot.git PX4
docker run --rm -v "$PWD/PX4:/src" -w /src \
  px4io/px4-dev:v1.17.0-beta1 make px4_fmu-v5_default
docker run --rm -v "$PWD/PX4:/src" -w /src \
  px4io/px4-dev:v1.17.0-beta1 make px4_fmu-v6c_default
```

The resulting inputs are:

```text
PX4/build/px4_fmu-v5_default/px4_fmu-v5_default.elf
PX4/build/px4_fmu-v6c_default/px4_fmu-v6c_default.elf
```

Their expected SHA-256 values are in `pins/smoke.json`; `smoke` rejects a
different ELF before starting Athrill.

No other human action is expected. The smoke is software-only, does not use
library injection, and does not bind TCP. Pixhawk 4 runs
`examples/px4-bench/check.sh` with `boards/pixhawk4/pixhawk4.dtb`. Pixhawk 6C
runs the bounded `examples/px4-v6c-boot` configuration with
`boards/pixhawk6c/pixhawk6c.dtb`. Both use virtual-clock measurements; the
fixed values live in `pins/smoke.json`, and measured values are written to
the Receipt.

The Pixhawk 6C smoke always removes its prior smoke directory and invokes the
pinned target's `examples/px4-sih/make-sd-image.sh` to create `sd.img` inside
`<work-dir>/smoke/pixhawk6c/`. Its fixed initial content is a 1 GiB FAT32
filesystem without a partition table, labeled `PX4SD`, with the generator's
fixed volume serial and label date and no files. The Recipe verifies both the
initial image hash and the post-smoke image hash, as well as the instruction
count and console hash. This exercises PX4's SD/parameter startup path without
depending on an ignored or pre-existing image.

Pixhawk 4 does not depend on a pre-existing image. Its pinned upstream
`examples/px4-bench/check.sh` deletes `examples/px4-bench/work`, creates a new
deterministic image with its local `make-sd-image.sh`, clones that seed for
both benchmark runs, and requires the two resulting SD images, F-RAM images,
and consoles to be identical.

Installed outputs are below `<work-dir>/install/`:

```text
bin/athrill3
lib/libhako_sensor_device.{dylib,so}
lib/libhako_sensor_device_inproc.dylib       # only when requested
boards/pixhawk4/{pixhawk4,pixhawk4-hakoniwa}.dtb
boards/pixhawk6c/{pixhawk6c,pixhawk6c-hakoniwa}.dtb
share/hakoniwa/receipts/athrill-stm32-board-build-<board>.yaml
```
