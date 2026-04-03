#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>
#include <stdio.h>
#include "angle_compute.h"

/* --- High-Level UI Buffers --- */
extern uint16_t* raw_camBuffer;
extern uint16_t* debug_camBuffer;

/* --- UI API Prototypes --- */
void UI_Init(void);
void UI_Process_Touch(uint16_t touch_x, uint16_t touch_y, uint8_t is_touching);
SystemMode_t UI_Get_Requested_Mode(void);
void UI_Force_Redraw(void);
void UI_Render_Screen(SystemMode_t current_mode, MotorAngles_t* current_angles, uint16_t touch_x, uint16_t touch_y, uint8_t is_touching);

/* --- Public Drawing API --- */
void UI_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg_color, uint8_t scale);
void UI_DrawStringCentered(uint16_t box_x, uint16_t box_y, uint16_t box_w, uint16_t box_h, const char* str, uint16_t color, uint16_t bg_color, uint8_t scale);

#endif /* LCD_UI_H */
