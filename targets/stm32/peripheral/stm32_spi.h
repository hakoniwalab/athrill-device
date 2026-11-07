/*
 * The STM32F7 SPI controllers SPI1 to SPI6 (RM0410 chapter 35) as masters,
 * and the chips on their buses.
 *
 * The registers are those of the hardware; a frame written to DR is
 * translated into a byte exchange on the selected common channel at once
 * (no bit timing), and the channel's answer goes
 * to the RX FIFO, and SR reports TXE, RXNE and the FIFO levels as the
 * firmware's polling or DMA expects. A chip is selected while its chip
 * select pin (a GPIO output) is low.
 */
#ifndef ATHRILL_STM32_SPI_H
#define ATHRILL_STM32_SPI_H

#include "std_types.h"

/* A chip on an SPI bus. */
typedef struct {
	/* the chip select went low (TRUE) or high (FALSE) */
	void (*select)(void *chip, std_bool selected);
	/* one byte out (MOSI), the chip's byte back (MISO) */
	uint8 (*exchange)(void *chip, uint8 mosi);
} Stm32SpiChipOps;

std_bool stm32_spi_owns(uint32 addr);
uint32 stm32_spi_read(uint32 addr, uint32 size);
void stm32_spi_write(uint32 addr, uint32 size, uint32 value);
void stm32_spi_reset(void);

/* Shared chip-select/backend path used by register-compatible and H7 SPI front ends. */
uint8 stm32_spi_chip_exchange(uint32 bus, uint8 mosi);

/* Puts chip on SPIbus (1 to 6), selected by pin of port (STM32_GPIO_PORT('F'), 10 ...). */
void stm32_spi_attach(uint32 bus, uint32 port, uint32 pin, const Stm32SpiChipOps *ops, void *chip);
/* Output pins of port changed from old to now (stm32_gpio.c): chip selects. */
void stm32_spi_pins_changed(uint32 port, uint32 old, uint32 now);

#endif
