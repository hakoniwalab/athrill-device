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
2dc51449975c554f0cf5f95c3eb84cef4f7bd09395d2a14d0c52cadfcaedb740  pixhawk6c.dtb
d83aa92a81acb0eb4e764c97a7ceee0fb567cb88302bc85cc4f586b5758ffd98  pixhawk6c-hakoniwa.dtb
```
