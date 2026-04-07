// lcd_display.c
#include "lcd_display.h"
#include "dma2d.h"

uint16_t *frameBuffer = (uint16_t *)0xC0000000;

void LCD_Wakeup(void)
{
    // reset LCD driver logic
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(20);

    // show image
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
}

void LCD_FillScreen(uint16_t color)
{
    // converting colour to hardware specs
    uint8_t r = (color & 0xF800) >> 8;
    uint8_t g = (color & 0x07E0) >> 3;
    uint8_t b = (color & 0x001F) << 3;

    uint32_t argb_color = 0xFF000000 | (r << 16) | (g << 8) | b;

    // configuring
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_R2M;
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;

    hdma2d.Init.OutputOffset = 0; // use entire thing

    if (HAL_DMA2D_Init(&hdma2d) == HAL_OK)
    {
        HAL_DMA2D_Start(&hdma2d, argb_color, (uint32_t)frameBuffer, 480, 272);

        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
    }
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    // check bounds
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
    {
        return;
    }

    // coordinate conversion + write
    uint32_t index = (y * LCD_WIDTH) + x;
    frameBuffer[index] = color;
}
