// ft5336.c
#include "ft5336.h"

extern I2C_HandleTypeDef hi2c3; // From CubeMX
#define FT5336_ADDR (0x38 << 1) // 0x70

uint8_t FT5336_ReadTouch(uint16_t *x, uint16_t *y) {
    uint8_t data[4];
    // Register 0x02 holds the number of touch points
    if (HAL_I2C_Mem_Read(&hi2c3, FT5336_ADDR, 0x02, 1, data, 1, 10) == HAL_OK) {
        if (data[0] > 0 && data[0] <= 5) {
            // Read registers 0x03 to 0x06 for X and Y data
            HAL_I2C_Mem_Read(&hi2c3, FT5336_ADDR, 0x03, 1, data, 4, 10);
            *y = ((data[0] & 0x0F) << 8) | data[1];
            *x = ((data[2] & 0x0F) << 8) | data[3];
            return 1; // Touched
        }
    }
    return 0; // Not touched
}
