/*
 * The STM32F7/H7 I2C controllers I2C1 to I2C4 (RM0410 chapter 33 and
 * RM0433 chapter 47) as masters with 7-bit addresses, and the chips on
 * their buses. The register engine is shared; a board table selects bases.
 *
 * The registers are those of the hardware and the transfer follows its
 * flags: START sends the address (ACK from a chip at it, else NACKF and an
 * automatic STOP), TXIS asks for each byte to send, RXNE offers each byte
 * received, and after NBYTES comes TCR (RELOAD), a STOP (AUTOEND) or TC; the
 * event interrupt follows the enabled flags. Lightweight: no bit timing
 * (each step happens at the register access that allows it), no slave
 * mode, no SMBus, no DMA.
 */
#ifndef ATHRILL_STM32_I2C_H
#define ATHRILL_STM32_I2C_H

#include "std_types.h"

/* A chip on an I2C bus. */
typedef struct {
	/* a START (or repeated START) addressed the chip for reading or writing; FALSE: NACK */
	std_bool (*start)(void *chip, std_bool read);
	void (*write)(void *chip, uint8 data);
	uint8 (*read)(void *chip);
	void (*stop)(void *chip);
} Stm32I2cChipOps;

void stm32_i2c_set_h7(std_bool h7);
std_bool stm32_i2c_owns(uint32 addr);
uint32 stm32_i2c_read(uint32 addr);
void stm32_i2c_write(uint32 addr, uint32 size, uint32 value);
void stm32_i2c_reset(void);

/* The event interrupts that stay asserted, pending again once their handlers end. */
void stm32_i2c_tick(void);

/* Puts chip on I2Cbus (1 to 4) at the 7-bit address. */
void stm32_i2c_attach(uint32 bus, uint32 address, const Stm32I2cChipOps *ops, void *chip);

#endif
