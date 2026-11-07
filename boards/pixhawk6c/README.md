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
94be95a46eef8c0aa01e9e58de4b8655bb0cd3a35afde246bcd549d66b72d9ae  pixhawk6c.dtb
05c5f7c85f07d8eb507dbac2ccc1835aae64a0ed715f49fea9a22f37b6425fd2  pixhawk6c-hakoniwa.dtb
```
