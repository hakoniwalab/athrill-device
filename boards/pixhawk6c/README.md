# Pixhawk 6C Devicetree

`pixhawk6c.dts` is the MIT-licensed V6C00 board layer. The SoC layer includes
Zephyr's Apache-2.0 `stm32h743Xi.dtsi` at revision
`f3fc249035296b87c75fa808b3abf78829fe9378`; its attribution and SPDX notice
are retained in `stm32h743-athrill.dtsi`. Board wiring is recorded from the
PX4-Autopilot v1.17.0 BSD-3-Clause FMUv6C board files listed in the DTS.

`pixhawk6c-hakoniwa.dts` enables the same explicit Hakoniwa sensor-source and
actuator edges used by Pixhawk 4. Host paths, channel names and trace settings
remain in `device_config.txt`.

`pixhawk6c-hakoniwa-dshot.dts` includes that and puts motors 1-4 on a DShot
ESC (`athrill,dshot-esc` on TIM1 CH1..CH4, feeding the Hakoniwa sink) in place
of the PWM actuator map. TIM1's update DMA request (DMAMUX1 request 15, in
`pixhawk6c.dts`) carries PX4's DShot burst. PX4 v1.17.0 drives the motors with
DShot600 when `PWM_AUX_TIM0` is -3. The ESC sits on USART2 (TELEM3) and
answers telemetry requests with KISS frames, which PX4 reads with
`DSHOT_TEL_CFG` 103. Its fault-model is `athrill,esc-fault-schedule` with no
stop: nothing changes until a run sets the host value
`DEVICE_CONFIG_ESC_MOTOR_STOP`, e.g. `2@40000` to stop PX4's motor 3 at 40 s.

`pixhawk6c-hakoniwa-dshot-sensors.dts` includes that and gives each sensor chip its own sensor instance of the
in-process sensor host (`docs/sensor-host.md`): `bmi055` (the BMI055's accelerometer and gyro), `icm42688p`,
`barometer`, `magnetometer` and `gps1`, children of the Hakoniwa sensor device. The motors stay on the sensor device.

The reviewed DTB hashes are filled in after compilation:

```text
df101d83cc918571884495d46ed861ee5a44ceaee5f4644fa3d268ac1e0ca262  pixhawk6c.dtb
cc7359fda3d1c779ff7f63cc48d2ab96119553c8f74227c50a231fa6c80876c3  pixhawk6c-hakoniwa.dtb
601acc6b14f87aa34ee6144ae7ddac251ff3b8c5fcc3ae119497266e4e5a594c  pixhawk6c-hakoniwa-dshot.dtb
71fd1159e80cedb38fd68f2d5efb4207d7e7a5f013021e483d2571860c70a454  pixhawk6c-hakoniwa-dshot-sensors.dtb
```
