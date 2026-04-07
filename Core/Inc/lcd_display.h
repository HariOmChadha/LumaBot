#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

/* dimensions and pixel math */
#define LCD_WIDTH 480
#define LCD_HEIGHT 272
#define PIXEL_COUNT (LCD_WIDTH * LCD_HEIGHT)

/* RGB565 colours */
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F

/* declaration of the framebuffer so DMA/LTDC can access */
extern uint16_t *frameBuffer;

/* functions */
void LCD_Wakeup(void);
void LCD_FillScreen(uint16_t color);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

#endif /* LCD_DISPLAY_H */
