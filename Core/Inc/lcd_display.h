#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

/* Display dimensions and pixel math */
#define LCD_WIDTH          480
#define LCD_HEIGHT         272
#define PIXEL_COUNT        (LCD_WIDTH * LCD_HEIGHT)

/* Standard RGB565 Colors */
#define COLOR_BLACK        0x0000
#define COLOR_WHITE        0xFFFF
#define COLOR_RED          0xF800
#define COLOR_GREEN        0x07E0
#define COLOR_BLUE         0x001F
#define COLOR_YELLOW       0xFFE0
#define COLOR_CYAN         0x07FF
#define COLOR_MAGENTA      0xF81F

/* Extern declaration of the framebuffer so DMA/LTDC can access it */
extern uint16_t* frameBuffer;

/* Function Prototypes */
void LCD_Wakeup(void);
void LCD_FillScreen(uint16_t color);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

#endif /* LCD_DISPLAY_H */
