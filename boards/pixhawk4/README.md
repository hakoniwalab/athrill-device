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
`pixhawk4-hakoniwa-dshot-asset.dts` makes the Hakoniwa sensor device the ESC's
fault-model, for Athrill as the Hakoniwa asset with the aircraft in process
(the in-process motor plugin): rotor faults then come from a
`hako_msgs/Disturbance` PDU, e.g. hakoniwa-threejs-drone's Rotor Fault Injection.

The checked-in blobs have SHA-256:

```text
04d6db97f4528f5ec3c7574d3687c22c4df2a02cb0ee65ab590dd61203d5fd29  pixhawk4.dtb
c44ce148aad800bfc16b4ee1ec4ad5ea9016cf5140c8b30cd09572cf6b33525a  pixhawk4-hakoniwa.dtb
3d4649b0073d6a4221948e36d68a6694316c4e518c239fdca39b6f14bd79aa8f  pixhawk4-hakoniwa-dshot.dtb
74c3a536a0dbe8dac178343e12f94acad358fe7392bf1141b22e59bf9590f0ac  pixhawk4-hakoniwa-dshot-asset.dtb
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
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot-asset.dts" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot-asset.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot-asset.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot-asset.pre.dts"
```

The Zephyr include remains an external build input; it is not copied into
this repository.  If it is ever vendored, retain its Apache-2.0 notices and
attribution.
