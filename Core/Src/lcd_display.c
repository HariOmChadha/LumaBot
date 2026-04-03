// lcd_display.c
#include "lcd_display.h"
#include "dma2d.h"

// SDRAM starts at 0xC0000000. We put the LCD framebuffer at the very beginning.
uint16_t* frameBuffer = (uint16_t*)0xC0000000;

void LCD_Wakeup(void)
{
    // 1. Hardware Reset/Cycle the LCD Driver Logic (Pin PI12)
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET); // Pull LOW to reset/disable
    HAL_Delay(20);                                         // Wait 20ms
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);   // Pull HIGH to wake up/enable
    HAL_Delay(20);

    // 2. Enable the LCD backlight (Pin PK3) to reveal the image
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
}

void LCD_FillScreen(uint16_t color) {
    // 1. UNPACK RGB565 TO ARGB8888 FOR THE DMA2D HARDWARE
    uint8_t r = (color & 0xF800) >> 8;
    uint8_t g = (color & 0x07E0) >> 3;
    uint8_t b = (color & 0x001F) << 3;

    // Repack into 32-bit ARGB
    uint32_t argb_color = 0xFF000000 | (r << 16) | (g << 8) | b;

    // 2. Configure DMA2D
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode         = DMA2D_R2M;
    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;

    // Offset is 0 because we are filling the entire continuous memory block
    hdma2d.Init.OutputOffset = 0;

    if (HAL_DMA2D_Init(&hdma2d) == HAL_OK) {
        // Blast the corrected ARGB color to the whole screen instantly!
        HAL_DMA2D_Start(&hdma2d, argb_color, (uint32_t)frameBuffer, 480, 272);

        // Wait for the ultra-fast transfer to finish
        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
    }
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    // 1. Bounds checking: Prevent Hard Faults by making sure we don't draw off the screen
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) {
        return;
    }

    // 2. Convert the 2D (x, y) coordinate into a 1D memory index
    uint32_t index = (y * LCD_WIDTH) + x;

    // 3. Write the color to the SDRAM
    frameBuffer[index] = color;
}
