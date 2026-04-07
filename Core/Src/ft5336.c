// ft5336.c
#include "ft5336.h"

extern I2C_HandleTypeDef hi2c3;
#define FT5336_ADDR (0x70)

uint8_t FT5336_ReadTouch(uint16_t *x, uint16_t *y)
{
    // register 0x02 has number of touch points, and 0x03 to 0x06 holds X and Y data
    uint8_t data[4];
    if (HAL_I2C_Mem_Read(&hi2c3, FT5336_ADDR, 0x02, 1, data, 1, 10) == HAL_OK)
    {
        if (data[0] > 0 && data[0] <= 5)
        {
            HAL_I2C_Mem_Read(&hi2c3, FT5336_ADDR, 0x03, 1, data, 4, 10);
            *y = ((data[0] & 0x0F) << 8) | data[1];
            *x = ((data[2] & 0x0F) << 8) | data[3];
            return 1;
        }
    }
    return 0;
}
