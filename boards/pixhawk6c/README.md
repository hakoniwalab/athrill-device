# Pixhawk 6C Devicetree

`pixhawk6c.dts` is the MIT-licensed V6C00 board layer. The SoC layer includes
Zephyr's Apache-2.0 `stm32h743Xi.dtsi` at revision
`f3fc249035296b87c75fa808b3abf78829fe9378`; its attribution and SPDX notice
are retained in `stm32h743-athrill.dtsi`. Board wiring is recorded from the
PX4-Autopilot v1.17.0 BSD-3-Clause FMUv6C board files listed in the DTS.

`pixhawk6c-hakoniwa.dts` enables the same explicit Hakoniwa sensor-source and
actuator edges used by Pixhawk 4. Host paths, channel names and trace settings
remain in `device_config.txt`.

The reviewed DTB hashes are filled in after compilation:

```text
aacc3d7b91bb171d926430a883a4d851c2c66727e09a32d4075dd3a006aeae11  pixhawk6c.dtb
989b985a9ac7dbfef7caa4c34d5491e096fd2c37554640c5122618c080ee4326  pixhawk6c-hakoniwa.dtb
```
