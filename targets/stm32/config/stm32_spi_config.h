/*
 * STM32 SPI: values the SVD does not hold (hakoniwalab/athrill-device#32,
 * rule 2). Defaults are the reference-manual values; the page is given with
 * each one.
 *
 * To customise one value, define it when building, e.g.
 *   -DSTM32F7_SPI_RX_FIFO_BYTES=8U
 * To replace the whole file, put your own stm32_spi_config.h in a directory
 * and give it to CMake as ATHRILL_STM32_CONFIG_DIR; that directory comes
 * first on the include path.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef STM32_SPI_CONFIG_H
#define STM32_SPI_CONFIG_H

/* RM0410 (STM32F76x/77x) PDF p.1300: two 32-bit embedded Rx and Tx FIFOs,
 * the same for SPI1-SPI6. */
#ifndef STM32F7_SPI_RX_FIFO_BYTES
#define STM32F7_SPI_RX_FIFO_BYTES	4U
#endif

/* RM0433 (STM32H742/743/753/750) Table 422, PDF p.2157: Rx and Tx FIFO size
 * 16 bytes for SPI1-SPI3, 8 bytes for SPI4-SPI6. One value per SPI1..SPI6. */
#ifndef STM32H7_SPI_RX_FIFO_BYTES
#define STM32H7_SPI_RX_FIFO_BYTES	{ 16U, 16U, 16U, 8U, 8U, 8U }
#endif

/* Storage per H7 SPI: at least the largest of STM32H7_SPI_RX_FIFO_BYTES
 * (checked at reset). */
#ifndef STM32H7_SPI_RX_FIFO_MAX
#define STM32H7_SPI_RX_FIFO_MAX		16U
#endif

#endif
