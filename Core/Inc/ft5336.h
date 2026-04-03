#ifndef FT5336_H
#define FT5336_H

#include "stm32f7xx_hal.h"

#define FT5336_I2C_ADDR (0x38 << 1) // 0x70

// Returns 1 if touched, 0 if not touched
uint8_t FT5336_ReadTouch(uint16_t *x, uint16_t *y);

#endif /* FT5336_H */
