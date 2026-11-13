# Pixhawk 4 / FMUv5 Devicetree

`pixhawk4.dts` is the MIT-licensed board layer.  It includes Zephyr's
Apache-2.0 `stm32f765Xi.dtsi` from pinned Zephyr revision
`f3fc249035296b87c75fa808b3abf78829fe9378`.  The comments identify the
BSD-3-Clause PX4 board files from which wiring facts were taken.  No
ArduPilot hwdef text is used.

The base board keeps `/sensor-device` disabled and leaves the chip models on
the target-provided sensor source.  This is the DTB for self-contained PX4
SIH and bench runs.  `pixhawk4-hakoniwa.dts` includes the base, enables the
Hakoniwa node, and adds the `value-source` edges used by Hakoniwa and PIL.
`pixhawk4-hakoniwa-dshot.dts` includes that and puts motors 1-4 on a DShot ESC
(`athrill,dshot-esc` on TIM1 CH4..CH1, feeding the Hakoniwa sink) in place of
the PWM actuator map. PX4 v1.17.0 drives them with DShot600 when
`PWM_AUX_TIM0` is -3. The ESC sits on UART4 (TELEM4) and answers telemetry
requests with KISS frames, which PX4 reads with `DSHOT_TEL_CFG` 104.
Its fault-model is `athrill,esc-fault-schedule` with no stop: nothing changes
until a run sets the host value `DEVICE_CONFIG_ESC_MOTOR_STOP`, e.g. `2@40000`
to stop PX4's motor 3 at 40 s.

The checked-in blobs have SHA-256:

```text
cd3a21918920144f266486817916342f5f841f551bbfde0e9576f5a95f28b412  pixhawk4.dtb
60c9e2dd35d89b919c39a1bb169ce1ab15929c6e52b1be439f959a653ea28fe0  pixhawk4-hakoniwa.dtb
338872bd171ac2cc4c037d5174c5d9422ddc93d4c73cc6be91971e70a369fef3  pixhawk4-hakoniwa-dshot.dtb
```

It was made with GNU cpp and dtc 1.7.x from a Linux container (Ubuntu
24.04, packages `cpp` and `device-tree-compiler`), from the repository
containing the pinned Zephyr sparse checkout (`dts`,
`include/zephyr/dt-bindings` and the `include/zephyr/sys/util_*.h` headers
they include):

```bash
ROOT=$PWD
ZEPHYR=$ROOT/zephyr/zephyr
mkdir -p "$ROOT/build"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4.dts" "$ROOT/build/pixhawk4.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dts" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4.dtb" \
  "$ROOT/build/pixhawk4.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot.dts" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot.pre.dts"
```

The Zephyr include remains an external build input; it is not copied into
this repository.  If it is ever vendored, retain its Apache-2.0 notices and
attribution.
