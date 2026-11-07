/* STM32H7 DMAMUX request-selection register files. */
#ifndef ATHRILL_STM32_DMAMUX_H
#define ATHRILL_STM32_DMAMUX_H

#include "std_types.h"

std_bool stm32_dmamux_owns(uint32 addr);
uint32 stm32_dmamux_read(uint32 addr);
void stm32_dmamux_write(uint32 addr, uint32 size, uint32 value);
void stm32_dmamux_reset(void);

#endif
