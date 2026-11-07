# STM32 models ported from athrill-target-stm32

The STM32 peripheral models that athrill-target-stm32 used to hold (SPI F7/H7,
UART, UART TCP, I2C, DMA, DMAMUX, SDMMC F7/H7, SD card), moved here unchanged
(hakoniwalab/athrill-device#13, step 3). Their history is in
athrill-target-stm32 up to the tag `c-path-final` and main 605fa89.

Already athrill-device models (src/models/, step 4): DMAMUX, I2C, SPI F7/H7,
UART and its TCP links, DMA. What remains here: SDMMC F7/H7 and the SD card.

State of what remains (not yet athrill-device models):

- They reach the host only through `include/athrill_device_host.h` (the
  required interface); they no longer call the Athrill core's time, IRQ,
  memory or configuration functions.
- They are compiled by athrill-target-stm32's CMake (`ATHRILL_DEVICE_DIR`),
  not by the athrill-device library: their tick functions still take the
  core's `DeviceClockType`, the UART console echo and `cpu.h` remain.
- The target still calls their functions directly and the Devicetree runtime
  hands their nodes to the target by name (`model_adapter_connect`).

Step 4 turns each into an athrill-device model created from the Devicetree
(model table: create, reset, tick, MMIO), adds the `console_write` and
`end_batch` required operations, and removes this directory's special build.

`config/`: values the SVD does not hold (hakoniwalab/athrill-device#32 rule 3);
replace with your own header via `ATHRILL_STM32_CONFIG_DIR`.
