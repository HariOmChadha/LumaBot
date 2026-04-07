#include "lcd_ui.h"
#include "lcd_display.h"
#include <string.h>
#include "stm32f7xx.h"
#include "cmsis_os.h"
#include "dma2d.h"

extern osMessageQueueId_t motorCmdQueueHandle;

extern uint16_t *frameBuffer;

extern float actual_motor_angles[5];

/* --- UI buffers --- */

// space to hold image in SDRAM
uint16_t *camBuffer_A = (uint16_t *)(0xC0000000 + 0x40000);
uint16_t *camBuffer_B = (uint16_t *)(0xC0000000 + 0x80000);

// space to draw to
uint16_t *stable_camBuffer = (uint16_t *)(0xC0000000 + 0x40000);

// debug
uint16_t *debug_camBuffer = (uint16_t *)(0xC0000000 + 0xC0000);
#define LCD_FRAME_BUFFER_ADDR ((uint16_t *)0xC0000000)

/* --- UI states --- */
static SystemMode_t requested_mode = MODE_MAIN_MENU;
static uint8_t touch_was_pressed_last_frame = 0;
static uint8_t needs_full_redraw = 1;

static uint8_t ignore_touches_until_lifted = 0;

// main menu/home
static float saved_main_menu_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

// manual mode (state 3)
static float previous_slider_angles[5] = {-1, -1, -1, -1, -1};
static uint8_t manual_active_preset = 0;
static uint8_t manual_is_saving = 0;

// track mode (state 4)
static uint8_t track_active_preset = 0;

static int8_t active_slider = -1;
static uint32_t touch_lockout_timer = 0;

/* --- some constants --- */
#define BTN_BACK_X 5
#define BTN_BACK_Y 5
#define BTN_BACK_W 60
#define BTN_BACK_H 30

#define COLOR_GRAY 0x7BEF
#define COLOR_DARK_GRAY 0x39E7
#define COLOR_DARK_CYAN 0x03EF

static const uint8_t font8x8[59][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 32: Space
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // 33: !
    {0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00}, // 34: "
    {0x6C, 0x6C, 0xFE, 0x6C, 0xFE, 0x6C, 0x6C, 0x00}, // 35: #
    {0x18, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x18, 0x00}, // 36: $
    {0x00, 0xC6, 0xCC, 0x18, 0x30, 0x66, 0xC6, 0x00}, // 37: %
    {0x38, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0x76, 0x00}, // 38: &
    {0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}, // 39: '
    {0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00}, // 40: (
    {0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00}, // 41: )
    {0x00, 0x18, 0x7E, 0x3C, 0x7E, 0x18, 0x00, 0x00}, // 42: *
    {0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00}, // 43: +
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30}, // 44: ,
    {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00}, // 45: -
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00}, // 46: .
    {0x00, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00, 0x00}, // 47: /
    {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00}, // 48: 0
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00}, // 49: 1
    {0x3C, 0x66, 0x06, 0x1C, 0x30, 0x60, 0x7E, 0x00}, // 50: 2
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00}, // 51: 3
    {0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00}, // 52: 4
    {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00}, // 53: 5
    {0x3C, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 54: 6
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00}, // 55: 7
    {0x3C, 0x66, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 56: 8
    {0x3C, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x3C, 0x00}, // 57: 9
    {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00}, // 58: :
    {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x30, 0x00}, // 59: ;
    {0x00, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x00, 0x00}, // 60: <
    {0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00}, // 61: =
    {0x00, 0x30, 0x18, 0x0C, 0x18, 0x30, 0x00, 0x00}, // 62: >
    {0x3C, 0x66, 0x06, 0x1C, 0x18, 0x00, 0x18, 0x00}, // 63: ?
    {0x3C, 0x66, 0x6E, 0x6E, 0x60, 0x66, 0x3C, 0x00}, // 64: @
    {0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x00}, // 65: A
    {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00}, // 66: B
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00}, // 67: C
    {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00}, // 68: D
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7E, 0x00}, // 69: E
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x60, 0x00}, // 70: F
    {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00}, // 71: G
    {0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00}, // 72: H
    {0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00}, // 73: I
    {0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3C, 0x00}, // 74: J
    {0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00}, // 75: K
    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00}, // 76: L
    {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x00}, // 77: M
    {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00}, // 78: N
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 79: O
    {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00}, // 80: P
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x0E, 0x00}, // 81: Q
    {0x7C, 0x66, 0x66, 0x7C, 0x78, 0x6C, 0x66, 0x00}, // 82: R
    {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00}, // 83: S
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, // 84: T
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 85: U
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00}, // 86: V
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, // 87: W
    {0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00}, // 88: X
    {0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00}, // 89: Y
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00}  // 90: Z
};

/* --- functions --- */

void UI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // bounds check
    if (x >= 480 || y >= 272)
        return;
    if (x + w > 480)
        w = 480 - x;
    if (y + h > 272)
        h = 272 - y;

    // colour conversion
    uint8_t r = (color & 0xF800) >> 8;
    uint8_t g = (color & 0x07E0) >> 3;
    uint8_t b = (color & 0x001F) << 3;

    uint32_t argb_color = 0xFF000000 | (r << 16) | (g << 8) | b;

    // configure
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_R2M;
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
    hdma2d.Init.OutputOffset = 480 - w;

    if (HAL_DMA2D_Init(&hdma2d) == HAL_OK)
    {
        uint32_t destination = (uint32_t)&frameBuffer[(y * 480) + x];
        HAL_DMA2D_Start(&hdma2d, argb_color, destination, w, h);

        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
    }
}

static void UI_DrawHollowRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color, uint8_t thickness)
{
    UI_FillRect(x, y, w, thickness, color);                 // top
    UI_FillRect(x, y + h - thickness, w, thickness, color); // bottom
    UI_FillRect(x, y, thickness, h, color);                 // left
    UI_FillRect(x + w - thickness, y, thickness, h, color); // right
}

static void UI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t scale)
{
    if (c < 32 || c > 90)
        return;
    uint8_t font_index = c - 32;

    for (uint8_t row = 0; row < 8; row++)
    {
        uint8_t row_data = font8x8[font_index][row];
        for (uint8_t col = 0; col < 8; col++)
        {
            uint16_t pixel_color = (row_data & (0x80 >> col)) ? color : bg_color;

            if (scale == 1)
            {
                LCD_DrawPixel(x + col, y + row, pixel_color);
            }
            else
            {
                UI_FillRect(x + (col * scale), y + (row * scale), scale, scale, pixel_color);
            }
        }
    }
}

void UI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t scale)
{
    uint16_t current_x = x;
    while (*str)
    {
        char c = *str;
        if (c >= 'a' && c <= 'z')
            c -= 32;
        UI_DrawChar(current_x, y, c, color, bg_color, scale);
        current_x += (8 * scale);
        str++;
    }
}

void UI_DrawStringCentered(uint16_t box_x, uint16_t box_y, uint16_t box_w, uint16_t box_h, const char *str, uint16_t color, uint16_t bg_color, uint8_t scale)
{
    uint16_t text_width = strlen(str) * 8 * scale;
    uint16_t text_height = 8 * scale;

    uint16_t start_x = box_x + ((box_w - text_width) / 2);
    uint16_t start_y = box_y + ((box_h - text_height) / 2);

    UI_DrawString(start_x, start_y, str, color, bg_color, scale);
}

void UI_DrawCamera(uint16_t start_x, uint16_t start_y, uint16_t *cam_buffer, uint8_t scale_mode)
{
    if (cam_buffer == NULL)
        return;

    uint16_t cam_w = 320;
    uint16_t cam_h = 240;

    if (scale_mode == 1)
    {
        // mode 1: half scale (160x120)
        uint16_t draw_y = 0;
        for (uint16_t y = 0; y < cam_h; y += 2)
        {
            // calculate the row addresses
            uint32_t dest_row_offset = (start_y + draw_y) * 480 + start_x;
            uint32_t src_row_offset = y * cam_w;

            uint16_t draw_x = 0;
            for (uint16_t x = 0; x < cam_w; x += 2)
            {
                frameBuffer[dest_row_offset + draw_x] = cam_buffer[src_row_offset + x];
                draw_x++;
            }
            draw_y++;
        }
    }
    else if (scale_mode == 2)
    {
        // mode 2: 3/4 scale (240x180), drop every 4th row/col
        uint16_t draw_y = 0;
        for (uint16_t y = 0; y < cam_h; y++)
        {
            if ((y + 1) % 4 == 0)
                continue;

            uint32_t dest_row_offset = (start_y + draw_y) * 480 + start_x;
            uint32_t src_row_offset = y * cam_w;

            uint16_t draw_x = 0;
            for (uint16_t x = 0; x < cam_w; x++)
            {
                if ((x + 1) % 4 == 0)
                    continue;

                frameBuffer[dest_row_offset + draw_x] = cam_buffer[src_row_offset + x];
                draw_x++;
            }
            draw_y++;
        }
    }
    else
    {
        // mode 0: full scale (320x240)
        for (uint16_t y = 0; y < cam_h; y++)
        {
            for (uint16_t x = 0; x < cam_w; x++)
            {
                uint16_t color = cam_buffer[y * cam_w + x];
                LCD_DrawPixel(start_x + x, start_y + y, color);
            }
        }
    }
}

static uint8_t Is_Touch_In_Box(uint16_t tx, uint16_t ty, uint16_t bx, uint16_t by, uint16_t bw, uint16_t bh)
{
    return (tx >= bx && tx <= bx + bw && ty >= by && ty <= by + bh);
}

extern LTDC_HandleTypeDef hltdc;

void Wait_For_LCD_VSYNC(void)
{
    // wait for the screen to draw
    while ((LTDC->CPSR & LTDC_CPSR_CYPOS) >= 272)
        ;

    // wait for it to finish
    while ((LTDC->CPSR & LTDC_CPSR_CYPOS) < 272)
        ;
}

/* --- visible UI functions --- */
void UI_Init(void)
{
    requested_mode = MODE_MAIN_MENU;
    needs_full_redraw = 1;
}

SystemMode_t UI_Get_Requested_Mode(void) { return requested_mode; }
void UI_Force_Redraw(void) { needs_full_redraw = 1; }

void UI_Process_Touch(uint16_t touch_x, uint16_t touch_y, uint8_t is_touching)
{

    uint32_t current_time = HAL_GetTick();

    // don't capture during transition
    if (current_time < touch_lockout_timer)
    {
        touch_was_pressed_last_frame = is_touching;
        active_slider = -1;
        return;
    }

    if (!is_touching)
    {
        active_slider = -1;
    }

    uint8_t just_pressed = (is_touching && !touch_was_pressed_last_frame);

    if (is_touching)
    {
        // back button
        if (just_pressed && requested_mode != MODE_MAIN_MENU &&
            Is_Touch_In_Box(touch_x, touch_y, BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H))
        {
            if (requested_mode == MODE_TRACKING && Track_Get_State() != TRACK_IDLE)
                return;

            // state logic:
            if (requested_mode == MODE_MANUAL)
            {
                // leave manual - save as default angles
                for (int i = 0; i < 5; i++)
                {
                    saved_main_menu_angles[i] = actual_motor_angles[i];
                }
            }
            else if (requested_mode == MODE_AUTO || requested_mode == MODE_DEBUG || requested_mode == MODE_TRACKING)
            {
                // leave dynamic (track/debug) - motors return to default
                MotorAngles_t restore_cmd = {0};
                restore_cmd.is_valid = 1;
                for (int i = 0; i < 5; i++)
                {
                    restore_cmd.angles[i] = saved_main_menu_angles[i];
                }
                osMessageQueuePut(motorCmdQueueHandle, &restore_cmd, 0, 0);
            }

            if (requested_mode == MODE_TRACKING)
                Track_Exit_Mode();

            requested_mode = MODE_MAIN_MENU;
            needs_full_redraw = 1;
            ignore_touches_until_lifted = 1;
            touch_lockout_timer = current_time + 300;
            touch_was_pressed_last_frame = is_touching;
            return;
        }

        switch (requested_mode)
        {
        case MODE_MAIN_MENU:
            if (!just_pressed)
                break;

            if (Is_Touch_In_Box(touch_x, touch_y, 40, 35, 180, 80))
            {
                for (int i = 0; i < 5; i++)
                    saved_main_menu_angles[i] = actual_motor_angles[i];
                requested_mode = MODE_DEBUG;
                needs_full_redraw = 1;
                touch_lockout_timer = current_time + 300;
            }
            if (Is_Touch_In_Box(touch_x, touch_y, 260, 35, 180, 80))
            {
                for (int i = 0; i < 5; i++)
                    saved_main_menu_angles[i] = actual_motor_angles[i];
                requested_mode = MODE_AUTO;
                needs_full_redraw = 1;
                touch_lockout_timer = current_time + 300;
            }
            if (Is_Touch_In_Box(touch_x, touch_y, 40, 135, 180, 80))
            {
                requested_mode = MODE_MANUAL;
                needs_full_redraw = 1;
                touch_lockout_timer = current_time + 300;

                // sync sliders to current
                manual_active_preset = 0;
                for (int i = 0; i < 5; i++)
                {
                    Set_Manual_Angle(i, actual_motor_angles[i]);
                }
            }
            if (Is_Touch_In_Box(touch_x, touch_y, 260, 135, 180, 80))
            {
                for (int i = 0; i < 5; i++)
                    saved_main_menu_angles[i] = actual_motor_angles[i];
                requested_mode = MODE_TRACKING;
                Track_Enter_Mode();
                track_active_preset = 0;
                needs_full_redraw = 1;
                touch_lockout_timer = current_time + 300;
            }

            if (Is_Touch_In_Box(touch_x, touch_y, 40, 225, 400, 30))
            {
                MotorAngles_t reset_cmd = {0};
                reset_cmd.is_valid = 1;
                for (int i = 0; i < 5; i++)
                {
                    reset_cmd.angles[i] = 90.0f;
                    // reset default
                    saved_main_menu_angles[i] = 90.0f;
                }
                osMessageQueuePut(motorCmdQueueHandle, &reset_cmd, 0, 0);

                needs_full_redraw = 1;
                touch_lockout_timer = current_time + 300;
            }
            break;

        case MODE_MANUAL:
            if (manual_is_saving == 0)
            {
                if (just_pressed)
                {
                    for (int i = 0; i < 5; i++)
                    {
                        if (Is_Touch_In_Box(touch_x, touch_y, 80, 40 + (i * 40) - 15, 340, 30))
                        {
                            active_slider = i;
                            break;
                        }
                    }
                }

                if (active_slider != -1)
                {
                    uint16_t clamped_x = (touch_x < 80) ? 80 : (touch_x > 400 ? 400 : touch_x);
                    Set_Manual_Angle(active_slider, ((float)(clamped_x - 80) / 320.0f) * 180.0f);
                    manual_active_preset = 0;

                    // smoothen movement
                    MotorAngles_t live_cmd = {0};
                    live_cmd.is_valid = 1;
                    for (int i = 0; i < 5; i++)
                        live_cmd.angles[i] = Get_Manual_Angle(i);
                    osMessageQueuePut(motorCmdQueueHandle, &live_cmd, 0, 0);
                }
            }

            uint8_t hit_preset = 0;

            if (!just_pressed)
                break;

            if (manual_is_saving == 0)
            {
                if (Is_Touch_In_Box(touch_x, touch_y, 15, 230, 90, 35))
                {
                    Load_Manual_Preset(1);
                    manual_active_preset = 1;
                    needs_full_redraw = 1;
                    hit_preset = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 125, 230, 90, 35))
                {
                    Load_Manual_Preset(2);
                    manual_active_preset = 2;
                    needs_full_redraw = 1;
                    hit_preset = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 235, 230, 90, 35))
                {
                    Load_Manual_Preset(3);
                    manual_active_preset = 3;
                    needs_full_redraw = 1;
                    hit_preset = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 345, 230, 120, 35))
                {
                    manual_is_saving = 1;
                    hit_preset = 1;
                    needs_full_redraw = 1;
                }

                if (!hit_preset && touch_y > 200)
                    manual_active_preset = 0;
            }
            else
            {
                if (Is_Touch_In_Box(touch_x, touch_y, 15, 230, 90, 35))
                {
                    Save_Manual_Preset(1);
                    manual_active_preset = 1;
                    manual_is_saving = 0;
                    needs_full_redraw = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 125, 230, 90, 35))
                {
                    Save_Manual_Preset(2);
                    manual_active_preset = 2;
                    manual_is_saving = 0;
                    needs_full_redraw = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 235, 230, 90, 35))
                {
                    Save_Manual_Preset(3);
                    manual_active_preset = 3;
                    manual_is_saving = 0;
                    needs_full_redraw = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 345, 230, 120, 35))
                {
                    manual_is_saving = 0;
                    needs_full_redraw = 1;
                }
            }
            break;

        case MODE_TRACKING:
            if (!just_pressed)
                break;
            TrackState_t state = Track_Get_State();
            uint8_t hit_track_btn = 0;

            if (Is_Touch_In_Box(touch_x, touch_y, 10, 60, 90, 50))
            {
                if (state == TRACK_IDLE)
                {
                    Track_Set_State(TRACK_RECORDING);
                    Track_Set_State(TRACK_TRACKING);
                    track_active_preset = 0;
                }
                else if (state == TRACK_TRACKING)
                    Track_Set_State(TRACK_IDLE);
                hit_track_btn = 1;
            }
            if (Is_Touch_In_Box(touch_x, touch_y, 10, 130, 90, 50))
            {
                if (state == TRACK_IDLE)
                {
                    Track_Set_State(TRACK_RECORDING);
                    track_active_preset = 0;
                }
                else if (state == TRACK_RECORDING)
                    Track_Set_State(TRACK_IDLE);
                hit_track_btn = 1;
            }

            if (Is_Touch_In_Box(touch_x, touch_y, 380, 60, 90, 50))
            {
                if (state == TRACK_REPLAYING)
                    Track_Set_State(TRACK_IDLE);
                else if (state == TRACK_IDLE && (Track_Has_Temp_Data() || Track_Has_Loaded_Preset()))
                    Track_Set_State(TRACK_REPLAYING);
                hit_track_btn = 1;
            }
            if (Is_Touch_In_Box(touch_x, touch_y, 380, 130, 90, 50))
            {
                if (state == TRACK_SAVING)
                    Track_Set_State(TRACK_IDLE);
                else if (state == TRACK_IDLE && Track_Has_Temp_Data())
                    Track_Set_State(TRACK_SAVING);
                hit_track_btn = 1;
            }

            if (state == TRACK_IDLE)
            {
                if (Is_Touch_In_Box(touch_x, touch_y, 85, 225, 90, 35))
                {
                    Track_Load_Preset(1);
                    track_active_preset = 1;
                    hit_track_btn = 1;
                    needs_full_redraw = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 195, 225, 90, 35))
                {
                    Track_Load_Preset(2);
                    track_active_preset = 2;
                    hit_track_btn = 1;
                    needs_full_redraw = 1;
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 305, 225, 90, 35))
                {
                    Track_Load_Preset(3);
                    track_active_preset = 3;
                    hit_track_btn = 1;
                    needs_full_redraw = 1;
                }

                if (!hit_track_btn)
                {
                    if (track_active_preset != 0)
                    {
                        needs_full_redraw = 1;
                    }
                    track_active_preset = 0;
                    Track_Clear_Loaded_Preset();
                }
            }
            else if (state == TRACK_SAVING)
            {
                if (Is_Touch_In_Box(touch_x, touch_y, 85, 225, 90, 35))
                {
                    Track_Save_To_Preset(1);
                    Track_Load_Preset(1);
                    track_active_preset = 1;
                    Track_Set_State(TRACK_IDLE);
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 195, 225, 90, 35))
                {
                    Track_Save_To_Preset(2);
                    Track_Load_Preset(2);
                    track_active_preset = 2;
                    Track_Set_State(TRACK_IDLE);
                }
                if (Is_Touch_In_Box(touch_x, touch_y, 305, 225, 90, 35))
                {
                    Track_Save_To_Preset(3);
                    Track_Load_Preset(3);
                    track_active_preset = 3;
                    Track_Set_State(TRACK_IDLE);
                }
            }

            break;

        default:
            break;
        }
    }
    touch_was_pressed_last_frame = is_touching;
}

void UI_Render_Screen(SystemMode_t current_mode, MotorAngles_t *current_angles, uint16_t touch_x, uint16_t touch_y, uint8_t is_touching)
{

    char str_buf[64];

    static TrackState_t last_drawn_track_state = TRACK_IDLE;
    static uint8_t last_manual_preset = 255;
    static uint8_t last_track_preset = 255;

    if (current_mode == MODE_TRACKING)
    {
        if (Track_Get_State() != last_drawn_track_state)
        {
            needs_full_redraw = 1;
            last_drawn_track_state = Track_Get_State();
        }
    }

    /* --- static drawing --- */
    if (needs_full_redraw)
    {
        Wait_For_LCD_VSYNC();
        LCD_FillScreen(COLOR_BLACK);

        last_manual_preset = 255;
        last_track_preset = 255;

        if (current_mode != MODE_MAIN_MENU)
        {
            UI_FillRect(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, COLOR_RED);
            UI_DrawStringCentered(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", COLOR_WHITE, COLOR_RED, 1);
        }

        switch (current_mode)
        {
        case MODE_MAIN_MENU:
            UI_FillRect(40, 35, 180, 80, COLOR_YELLOW);
            UI_DrawStringCentered(40, 35, 180, 80, "DEBUG", COLOR_BLACK, COLOR_YELLOW, 2);
            UI_FillRect(260, 35, 180, 80, COLOR_GREEN);
            UI_DrawStringCentered(260, 35, 180, 80, "AUTO", COLOR_BLACK, COLOR_GREEN, 2);
            UI_FillRect(40, 135, 180, 80, COLOR_BLUE);
            UI_DrawStringCentered(40, 135, 180, 80, "MANUAL", COLOR_WHITE, COLOR_BLUE, 2);
            UI_FillRect(260, 135, 180, 80, COLOR_MAGENTA);
            UI_DrawStringCentered(260, 135, 180, 80, "TRACK", COLOR_WHITE, COLOR_MAGENTA, 2);

            UI_FillRect(40, 225, 400, 30, COLOR_RED);
            UI_DrawStringCentered(40, 225, 400, 30, "RESET MOTORS", COLOR_WHITE, COLOR_RED, 2);
            break;

        case MODE_DEBUG:
            UI_DrawStringCentered(60, 35, 160, 20, "RAW SENSOR", COLOR_WHITE, COLOR_BLACK, 1);
            UI_DrawStringCentered(260, 35, 160, 20, "CV OUTPUT", COLOR_WHITE, COLOR_BLACK, 1);
            UI_DrawHollowRect(59, 59, 162, 122, COLOR_WHITE, 1);
            UI_DrawHollowRect(259, 59, 162, 122, COLOR_WHITE, 1);
            break;

        case MODE_AUTO:
            UI_DrawHollowRect(119, 39, 242, 182, COLOR_WHITE, 1);
            break;

        case MODE_MANUAL:
            for (int i = 0; i < 5; i++)
                previous_slider_angles[i] = -1;

            uint16_t btn_color = manual_is_saving ? COLOR_YELLOW : COLOR_GREEN;
            UI_FillRect(15, 230, 90, 35, btn_color);
            UI_DrawStringCentered(15, 230, 90, 35, "PRESET 1", COLOR_BLACK, btn_color, 1);
            UI_FillRect(125, 230, 90, 35, btn_color);
            UI_DrawStringCentered(125, 230, 90, 35, "PRESET 2", COLOR_BLACK, btn_color, 1);
            UI_FillRect(235, 230, 90, 35, btn_color);
            UI_DrawStringCentered(235, 230, 90, 35, "PRESET 3", COLOR_BLACK, btn_color, 1);

            if (manual_is_saving)
            {
                UI_FillRect(345, 230, 120, 35, COLOR_RED);
                UI_DrawStringCentered(345, 230, 120, 35, "CANCEL", COLOR_WHITE, COLOR_RED, 1);
            }
            else
            {
                UI_FillRect(345, 230, 120, 35, COLOR_MAGENTA);
                UI_DrawStringCentered(345, 230, 120, 35, "SAVE PRESET", COLOR_WHITE, COLOR_MAGENTA, 1);
            }
            break;

        case MODE_TRACKING:
        {
            TrackState_t state = Track_Get_State();

            UI_DrawHollowRect(119, 19, 242, 182, COLOR_WHITE, 1);

            if (state == TRACK_TRACKING)
            {
                UI_FillRect(10, 60, 90, 50, COLOR_RED);
                UI_DrawStringCentered(10, 60, 90, 50, "STOP", COLOR_WHITE, COLOR_RED, 1);
            }
            else if (state == TRACK_IDLE)
            {
                UI_FillRect(10, 60, 90, 50, COLOR_GREEN);
                UI_DrawStringCentered(10, 60, 90, 50, "TRACK", COLOR_BLACK, COLOR_GREEN, 1);
            }
            else
            {
                UI_FillRect(10, 60, 90, 50, COLOR_DARK_GRAY);
                UI_DrawStringCentered(10, 60, 90, 50, "TRACK", COLOR_GRAY, COLOR_DARK_GRAY, 1);
            }

            if (state == TRACK_RECORDING)
            {
                UI_FillRect(10, 130, 90, 50, COLOR_RED);
                UI_DrawStringCentered(10, 130, 90, 50, "END", COLOR_WHITE, COLOR_RED, 1);
            }
            else if (state == TRACK_IDLE)
            {
                UI_FillRect(10, 130, 90, 50, COLOR_GREEN);
                UI_DrawStringCentered(10, 130, 90, 50, "RECORD", COLOR_BLACK, COLOR_GREEN, 1);
            }
            else
            {
                UI_FillRect(10, 130, 90, 50, COLOR_DARK_GRAY);
                UI_DrawStringCentered(10, 130, 90, 50, "RECORD", COLOR_GRAY, COLOR_DARK_GRAY, 1);
            }

            if (state == TRACK_REPLAYING)
            {
                UI_FillRect(380, 60, 90, 50, COLOR_RED);
                UI_DrawStringCentered(380, 60, 90, 50, "END", COLOR_WHITE, COLOR_RED, 1);
            }
            else if (state == TRACK_IDLE && (Track_Has_Temp_Data() || Track_Has_Loaded_Preset()))
            {
                UI_FillRect(380, 60, 90, 50, COLOR_CYAN);
                UI_DrawStringCentered(380, 60, 90, 50, "REPLAY", COLOR_BLACK, COLOR_CYAN, 1);
            }
            else
            {
                UI_FillRect(380, 60, 90, 50, COLOR_DARK_GRAY);
                UI_DrawStringCentered(380, 60, 90, 50, "REPLAY", COLOR_GRAY, COLOR_DARK_GRAY, 1);
            }

            if (state == TRACK_SAVING)
            {
                UI_FillRect(380, 130, 90, 50, COLOR_RED);
                UI_DrawStringCentered(380, 130, 90, 50, "CANCEL", COLOR_WHITE, COLOR_RED, 1);
            }
            else if (state == TRACK_IDLE && Track_Has_Temp_Data())
            {
                UI_FillRect(380, 130, 90, 50, COLOR_MAGENTA);
                UI_DrawStringCentered(380, 130, 90, 50, "SAVE", COLOR_WHITE, COLOR_MAGENTA, 1);
            }
            else
            {
                UI_FillRect(380, 130, 90, 50, COLOR_DARK_GRAY);
                UI_DrawStringCentered(380, 130, 90, 50, "SAVE", COLOR_GRAY, COLOR_DARK_GRAY, 1);
            }

            uint16_t p_color = (state == TRACK_IDLE || state == TRACK_SAVING) ? COLOR_GREEN : COLOR_DARK_GRAY;
            uint16_t t_color = (state == TRACK_IDLE || state == TRACK_SAVING) ? COLOR_BLACK : COLOR_GRAY;

            UI_FillRect(85, 225, 90, 35, p_color);
            UI_DrawStringCentered(85, 225, 90, 35, "PRESET 1", t_color, p_color, 1);
            UI_FillRect(195, 225, 90, 35, p_color);
            UI_DrawStringCentered(195, 225, 90, 35, "PRESET 2", t_color, p_color, 1);
            UI_FillRect(305, 225, 90, 35, p_color);
            UI_DrawStringCentered(305, 225, 90, 35, "PRESET 3", t_color, p_color, 1);
            break;
        }
        }
        needs_full_redraw = 0;
    }

    /* --- dynamic drawing --- */
    Wait_For_LCD_VSYNC();
    switch (current_mode)
    {
    case MODE_MAIN_MENU:
        sprintf(str_buf, "TOUCH -> X:%03d Y:%03d", touch_x, touch_y);
        UI_DrawString(10, 10, str_buf, COLOR_WHITE, COLOR_BLACK, 1);
        break;

        case MODE_DEBUG:
        	UI_DrawCamera(60, 60, stable_camBuffer, 1);
			UI_DrawCamera(260, 60, stable_camBuffer, 1);
			if (current_angles->is_valid) {
				UI_DrawHollowRect(260 + (current_angles->box_x / 2),
								  60 + (current_angles->box_y / 2),
								  current_angles->box_w / 2,
								  current_angles->box_h / 2,
								  COLOR_GREEN, 2);
			}

			// --- BLOB DEBUG ---
			extern uint32_t blob_pixels;
			sprintf(str_buf, "Blob px: %lu", blob_pixels);
			UI_DrawStringCentered(0, 185, 480, 15, str_buf, COLOR_GREEN, COLOR_BLACK, 1);

			// --- PIXEL SAMPLER: point glove at camera centre, read these values ---
			if (stable_camBuffer != NULL) {
				uint16_t px = stable_camBuffer[120 * 320 + 160];  /* centre pixel */
				uint8_t r = (px >> 11) & 0x1F;
				uint8_t g = (px >> 5)  & 0x3F;
				uint8_t b =  px        & 0x1F;
				sprintf(str_buf, "Centre px  R:%02d G:%02d B:%02d", r, g, b);
				UI_DrawStringCentered(0, 200, 480, 20, str_buf, COLOR_YELLOW, COLOR_BLACK, 1);
			}

            // --- READS DIRECTLY FROM THE PHYSICAL HARDWARE STATE ---
            sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
            UI_DrawStringCentered(0, 220, 480, 20, str_buf, COLOR_WHITE, COLOR_BLACK, 1);
            break;

        case MODE_AUTO:
        	UI_DrawCamera(120, 40, stable_camBuffer, 2);
			if (current_angles->is_valid) {
				UI_DrawHollowRect(120 + (current_angles->box_x * 3 / 4),
								  40 + (current_angles->box_y * 3 / 4),
								  current_angles->box_w * 3 / 4,
								  current_angles->box_h * 3 / 4,
								  COLOR_GREEN, 2);
			}

            // --- EE DESIRED POSITION (mm, integer to avoid nano.specs float printf) ---
            {
                float ee[3];
                IK_Get_EE_Pos(ee);
                int ex = (int)(ee[0] * 1000.0f);
                int ey = (int)(ee[1] * 1000.0f);
                int ez = (int)(ee[2] * 1000.0f);
                sprintf(str_buf, "EE X:%+dmm Y:%+dmm Z:%+dmm", ex, ey, ez);
                UI_DrawStringCentered(0, 225, 480, 14, str_buf, COLOR_CYAN, COLOR_BLACK, 1);
            }

            // --- READS DIRECTLY FROM THE PHYSICAL HARDWARE STATE ---
            sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
            UI_DrawStringCentered(0, 240, 480, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);
            break;

        case MODE_MANUAL:
            for (int i = 0; i < 5; i++) {
                float angle = Get_Manual_Angle(i);
                if (angle != previous_slider_angles[i]) {
                    uint16_t slider_y = 40 + (i * 40);
                    uint16_t track_c = manual_is_saving ? COLOR_GRAY : COLOR_WHITE;
                    uint16_t knob_c  = manual_is_saving ? COLOR_DARK_CYAN : COLOR_CYAN;

                    uint16_t new_knob_x = 80 + (uint16_t)((angle / 180.0f) * 320.0f);
                    if(new_knob_x > 390) new_knob_x = 390;

                    if (previous_slider_angles[i] == -1) {
                        UI_FillRect(80, slider_y - 10, 350, 30, COLOR_BLACK);
                        UI_FillRect(80, slider_y, 320, 10, track_c);
                        UI_FillRect(new_knob_x, slider_y - 10, 20, 30, knob_c);
                    } else {
                        uint16_t old_knob_x = 80 + (uint16_t)((previous_slider_angles[i] / 180.0f) * 320.0f);
                        if(old_knob_x > 390) old_knob_x = 390;

                        if (new_knob_x > old_knob_x) {
                            UI_FillRect(old_knob_x, slider_y - 10, new_knob_x - old_knob_x, 10, COLOR_BLACK);
                            UI_FillRect(old_knob_x, slider_y + 10, new_knob_x - old_knob_x, 10, COLOR_BLACK);
                            UI_FillRect(old_knob_x, slider_y, new_knob_x - old_knob_x, 10, track_c);
                        } else if (new_knob_x < old_knob_x) {
                            UI_FillRect(new_knob_x + 20, slider_y - 10, old_knob_x - new_knob_x, 10, COLOR_BLACK);
                            UI_FillRect(new_knob_x + 20, slider_y + 10, old_knob_x - new_knob_x, 10, COLOR_BLACK);
                            UI_FillRect(new_knob_x + 20, slider_y, old_knob_x - new_knob_x, 10, track_c);
                        }
                        UI_FillRect(new_knob_x, slider_y - 10, 20, 30, knob_c);
                    }

                    sprintf(str_buf, "%03d", (int)angle);
                    UI_DrawString(415, slider_y - 2, str_buf, knob_c, COLOR_BLACK, 2);
                    previous_slider_angles[i] = angle;
                }
            }
            sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
				(int)actual_motor_angles[0], (int)actual_motor_angles[1],
				(int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
            UI_DrawStringCentered(80, 10, 320, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);

            if (manual_is_saving == 0) {
                if (manual_active_preset != last_manual_preset) {
                    UI_DrawHollowRect(13, 228, 94, 39, COLOR_BLACK, 2);
                    UI_DrawHollowRect(123, 228, 94, 39, COLOR_BLACK, 2);
                    UI_DrawHollowRect(233, 228, 94, 39, COLOR_BLACK, 2);
                    if (manual_active_preset == 1) UI_DrawHollowRect(13, 228, 94, 39, COLOR_WHITE, 2);
                    if (manual_active_preset == 2) UI_DrawHollowRect(123, 228, 94, 39, COLOR_WHITE, 2);
                    if (manual_active_preset == 3) UI_DrawHollowRect(233, 228, 94, 39, COLOR_WHITE, 2);
                    last_manual_preset = manual_active_preset;
                }
            } else {
                last_manual_preset = 255;
            }
            break;

        case MODE_TRACKING: {
            TrackState_t state = Track_Get_State();

            UI_DrawCamera(120, 20, stable_camBuffer, 2);
            if (current_angles->is_valid) {
				UI_DrawHollowRect(120 + (current_angles->box_x * 3 / 4),
								  20 + (current_angles->box_y * 3 / 4),
								  current_angles->box_w * 3 / 4,
								  current_angles->box_h * 3 / 4,
								  COLOR_GREEN, 2);
			}

            if (state == TRACK_IDLE)           sprintf(str_buf, "-- IDLE --");
            else if (state == TRACK_TRACKING)  sprintf(str_buf, ">> TRACKING LIVE <<");
            else if (state == TRACK_RECORDING) sprintf(str_buf, "[REC] RECORDING...");
            else if (state == TRACK_REPLAYING) sprintf(str_buf, ">> REPLAYING <<");
            else if (state == TRACK_SAVING)    sprintf(str_buf, "CHOOSE SAVE SLOT");

            UI_DrawStringCentered(120, 5, 240, 10, str_buf, (state == TRACK_IDLE) ? COLOR_WHITE : COLOR_GREEN, COLOR_BLACK, 1);

            // --- READS DIRECTLY FROM THE PHYSICAL HARDWARE STATE ---
            sprintf(str_buf, "A1:%03d A2:%03d A3:%03d A4:%03d A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
            UI_DrawStringCentered(80, 207, 320, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);

            if (state == TRACK_IDLE || state == TRACK_SAVING) {
                if (track_active_preset != last_track_preset) {
                    UI_DrawHollowRect(83, 223, 94, 39, COLOR_BLACK, 2);
                    UI_DrawHollowRect(193, 223, 94, 39, COLOR_BLACK, 2);
                    UI_DrawHollowRect(303, 223, 94, 39, COLOR_BLACK, 2);
                    if (track_active_preset == 1) UI_DrawHollowRect(83, 223, 94, 39, COLOR_WHITE, 2);
                    if (track_active_preset == 2) UI_DrawHollowRect(193, 223, 94, 39, COLOR_WHITE, 2);
                    if (track_active_preset == 3) UI_DrawHollowRect(303, 223, 94, 39, COLOR_WHITE, 2);
                    last_track_preset = track_active_preset;
                }
            } else {
                last_track_preset = 255;
            }
            break;
        }

        extern uint32_t blob_pixels;
        sprintf(str_buf, "Blob px: %lu", blob_pixels);
        UI_DrawStringCentered(0, 185, 480, 15, str_buf, COLOR_GREEN, COLOR_BLACK, 1);

        // samples pixels from glove
        if (stable_camBuffer != NULL)
        {
            uint16_t px = stable_camBuffer[120 * 320 + 160]; // center pixel
            uint8_t r = (px >> 11) & 0x1F;
            uint8_t g = (px >> 5) & 0x3F;
            uint8_t b = px & 0x1F;
            sprintf(str_buf, "Centre px  R:%02d G:%02d B:%02d", r, g, b);
            UI_DrawStringCentered(0, 200, 480, 20, str_buf, COLOR_YELLOW, COLOR_BLACK, 1);
        }

        // read physical hardware state
        sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
        UI_DrawStringCentered(0, 220, 480, 20, str_buf, COLOR_WHITE, COLOR_BLACK, 1);
        break;

    case MODE_AUTO:
        UI_DrawCamera(120, 40, stable_camBuffer, 2);
        if (current_angles->is_valid)
        {
            UI_DrawHollowRect(120 + (current_angles->box_x * 3 / 4),
                              40 + (current_angles->box_y * 3 / 4),
                              current_angles->box_w * 3 / 4,
                              current_angles->box_h * 3 / 4,
                              COLOR_GREEN, 2);
        }

        sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
        UI_DrawStringCentered(0, 240, 480, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);
        break;

    case MODE_MANUAL:
        for (int i = 0; i < 5; i++)
        {
            float angle = Get_Manual_Angle(i);
            if (angle != previous_slider_angles[i])
            {
                uint16_t slider_y = 40 + (i * 40);
                uint16_t track_c = manual_is_saving ? COLOR_GRAY : COLOR_WHITE;
                uint16_t knob_c = manual_is_saving ? COLOR_DARK_CYAN : COLOR_CYAN;

                uint16_t new_knob_x = 80 + (uint16_t)((angle / 180.0f) * 320.0f);
                if (new_knob_x > 390)
                    new_knob_x = 390;

                if (previous_slider_angles[i] == -1)
                {
                    UI_FillRect(80, slider_y - 10, 350, 30, COLOR_BLACK);
                    UI_FillRect(80, slider_y, 320, 10, track_c);
                    UI_FillRect(new_knob_x, slider_y - 10, 20, 30, knob_c);
                }
                else
                {
                    uint16_t old_knob_x = 80 + (uint16_t)((previous_slider_angles[i] / 180.0f) * 320.0f);
                    if (old_knob_x > 390)
                        old_knob_x = 390;

                    if (new_knob_x > old_knob_x)
                    {
                        UI_FillRect(old_knob_x, slider_y - 10, new_knob_x - old_knob_x, 10, COLOR_BLACK);
                        UI_FillRect(old_knob_x, slider_y + 10, new_knob_x - old_knob_x, 10, COLOR_BLACK);
                        UI_FillRect(old_knob_x, slider_y, new_knob_x - old_knob_x, 10, track_c);
                    }
                    else if (new_knob_x < old_knob_x)
                    {
                        UI_FillRect(new_knob_x + 20, slider_y - 10, old_knob_x - new_knob_x, 10, COLOR_BLACK);
                        UI_FillRect(new_knob_x + 20, slider_y + 10, old_knob_x - new_knob_x, 10, COLOR_BLACK);
                        UI_FillRect(new_knob_x + 20, slider_y, old_knob_x - new_knob_x, 10, track_c);
                    }
                    UI_FillRect(new_knob_x, slider_y - 10, 20, 30, knob_c);
                }

                sprintf(str_buf, "%03d", (int)angle);
                UI_DrawString(415, slider_y - 2, str_buf, knob_c, COLOR_BLACK, 2);
                previous_slider_angles[i] = angle;
            }
        }
        sprintf(str_buf, "A1:%03d  A2:%03d  A3:%03d  A4:%03d  A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
        UI_DrawStringCentered(80, 10, 320, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);

        if (manual_is_saving == 0)
        {
            if (manual_active_preset != last_manual_preset)
            {
                UI_DrawHollowRect(13, 228, 94, 39, COLOR_BLACK, 2);
                UI_DrawHollowRect(123, 228, 94, 39, COLOR_BLACK, 2);
                UI_DrawHollowRect(233, 228, 94, 39, COLOR_BLACK, 2);
                if (manual_active_preset == 1)
                    UI_DrawHollowRect(13, 228, 94, 39, COLOR_WHITE, 2);
                if (manual_active_preset == 2)
                    UI_DrawHollowRect(123, 228, 94, 39, COLOR_WHITE, 2);
                if (manual_active_preset == 3)
                    UI_DrawHollowRect(233, 228, 94, 39, COLOR_WHITE, 2);
                last_manual_preset = manual_active_preset;
            }
        }
        else
        {
            last_manual_preset = 255;
        }
        break;

    case MODE_TRACKING:
    {
        TrackState_t state = Track_Get_State();

        UI_DrawCamera(120, 20, stable_camBuffer, 2);
        if (current_angles->is_valid)
        {
            UI_DrawHollowRect(120 + (current_angles->box_x * 3 / 4),
                              20 + (current_angles->box_y * 3 / 4),
                              current_angles->box_w * 3 / 4,
                              current_angles->box_h * 3 / 4,
                              COLOR_GREEN, 2);
        }

        if (state == TRACK_IDLE)
            sprintf(str_buf, "-- IDLE --");
        else if (state == TRACK_TRACKING)
            sprintf(str_buf, ">> TRACKING LIVE <<");
        else if (state == TRACK_RECORDING)
            sprintf(str_buf, "[REC] RECORDING...");
        else if (state == TRACK_REPLAYING)
            sprintf(str_buf, ">> REPLAYING <<");
        else if (state == TRACK_SAVING)
            sprintf(str_buf, "CHOOSE SAVE SLOT");

        UI_DrawStringCentered(120, 5, 240, 10, str_buf, (state == TRACK_IDLE) ? COLOR_WHITE : COLOR_GREEN, COLOR_BLACK, 1);

        sprintf(str_buf, "A1:%03d A2:%03d A3:%03d A4:%03d A5:%03d",
                (int)actual_motor_angles[0], (int)actual_motor_angles[1],
                (int)actual_motor_angles[2], (int)actual_motor_angles[3], (int)actual_motor_angles[4]);
        UI_DrawStringCentered(80, 207, 320, 12, str_buf, COLOR_WHITE, COLOR_BLACK, 1);

        if (state == TRACK_IDLE || state == TRACK_SAVING)
        {
            if (track_active_preset != last_track_preset)
            {
                UI_DrawHollowRect(83, 223, 94, 39, COLOR_BLACK, 2);
                UI_DrawHollowRect(193, 223, 94, 39, COLOR_BLACK, 2);
                UI_DrawHollowRect(303, 223, 94, 39, COLOR_BLACK, 2);
                if (track_active_preset == 1)
                    UI_DrawHollowRect(83, 223, 94, 39, COLOR_WHITE, 2);
                if (track_active_preset == 2)
                    UI_DrawHollowRect(193, 223, 94, 39, COLOR_WHITE, 2);
                if (track_active_preset == 3)
                    UI_DrawHollowRect(303, 223, 94, 39, COLOR_WHITE, 2);
                last_track_preset = track_active_preset;
            }
        }
        else
        {
            last_track_preset = 255;
        }
        break;
    }
    default:
        break;
    }
}
