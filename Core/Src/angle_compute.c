#include "angle_compute.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"

extern osMessageQueueId_t motorCmdQueueHandle;
extern float actual_motor_angles[5]; // Pull in the physical reality tracker

/* ========================================================================= */
/* MODE 3: MANUAL MEMORY                                                     */
/* ========================================================================= */
static MotorAngles_t current_manual_state = { {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 };
static MotorAngles_t manual_presets[3] = {
    { {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 },
    { {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 },
    { {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 }
};

/* ========================================================================= */
/* MODE 4: TRACKING MEMORY & LOGIC                                           */
/* ========================================================================= */
typedef struct {
    MotorAngles_t frames[MAX_RECORD_FRAMES];
    uint16_t length;
} TrackHistory_t;

static TrackHistory_t temp_history = {0};
static TrackHistory_t preset_history[3] = {0};
static TrackHistory_t* playback_target = &temp_history;

static TrackState_t current_track_state = TRACK_IDLE;
static MotorAngles_t pre_track_angles = {0};
static MotorAngles_t live_track_angles = { {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 };
static uint16_t replay_index = 0;
static uint8_t has_loaded_preset = 0;

/* ========================================================================= */
/* MODE 3: MANUAL MEMORY (UPDATED FOR RTOS DISPATCH)                         */
/* ========================================================================= */

// Helper to blast the current manual state to the motor queue
static void Dispatch_Manual_To_Motors(void) {
    osMessageQueuePut(motorCmdQueueHandle, &current_manual_state, 0, 0);
}


float Get_Manual_Angle(uint8_t joint_index) {
    if (joint_index < 5) return current_manual_state.angles[joint_index];
    return 0.0f;
}

void Set_Manual_Angle(uint8_t joint_index, float angle_value) {
    if (joint_index < 5) {
        current_manual_state.angles[joint_index] = angle_value;
        // We don't dispatch here because the UI slider loop
        // will handle high-frequency updates to avoid queue overflow.
    }
}

void Save_Manual_Preset(uint8_t preset_slot) {
    if (preset_slot >= 1 && preset_slot <= 3) {
        manual_presets[preset_slot - 1] = current_manual_state;
    }
}

void Load_Manual_Preset(uint8_t preset_slot) {
    if (preset_slot >= 1 && preset_slot <= 3) {
        current_manual_state = manual_presets[preset_slot - 1];
        // DISPATCH: Tell the motors to move to the preset immediately
        Dispatch_Manual_To_Motors();
    }
}

void Track_Enter_Mode(void) {
    // Save the actual physical hardware state
    for (int i = 0; i < 5; i++) {
        pre_track_angles.angles[i] = actual_motor_angles[i];
    }
    pre_track_angles.is_valid = 1;

    live_track_angles = (MotorAngles_t){ {90.0f, 90.0f, 90.0f, 90.0f, 90.0f}, 1 };
    current_track_state = TRACK_IDLE;
    temp_history.length = 0;
    has_loaded_preset = 0;

    // FIX 1: Reset the playback pointer to temp on entry
    playback_target = &temp_history;
}

void Track_Exit_Mode(void) {
    current_manual_state = pre_track_angles;
}

void Track_Set_State(TrackState_t new_state) {
    if (new_state == TRACK_RECORDING) {
        temp_history.length = 0;
        has_loaded_preset = 0;

        // FIX 2: Force playback target back to temp when a new recording starts
        // (Otherwise it gets stuck pointing at an old preset while you record new temp data)
        playback_target = &temp_history;

    } else if (new_state == TRACK_REPLAYING) {
        replay_index = 0;
    }
    current_track_state = new_state;
}

TrackState_t Track_Get_State(void) { return current_track_state; }
uint8_t Track_Has_Temp_Data(void) { return (temp_history.length > 0); }
uint8_t Track_Has_Loaded_Preset(void) { return has_loaded_preset; }
void Track_Clear_Loaded_Preset(void) {
    has_loaded_preset = 0;

    // FIX 3: Revert the pointer back to temp data when clearing a loaded preset
    playback_target = &temp_history;
}

void Track_Save_To_Preset(uint8_t preset_slot) {
    if (preset_slot >= 1 && preset_slot <= 3) {
        // 1. Move temp to the preset
        preset_history[preset_slot - 1] = temp_history;

        // 2. "Load" that preset immediately
        has_loaded_preset = 1;
        playback_target = &preset_history[preset_slot - 1];

        // 3. FIX 4: Explicitly clear the temp memory now that it is safely in a preset
        temp_history.length = 0;
    }
}

void Track_Load_Preset(uint8_t preset_slot) {
    if (preset_slot >= 1 && preset_slot <= 3) {
        // 1. Unconditionally point to the requested preset, even if empty
        playback_target = &preset_history[preset_slot - 1];

        // 2. Only flag it as a valid loaded preset if it actually has data
        if (playback_target->length > 0) {
            has_loaded_preset = 1;
        } else {
            has_loaded_preset = 0;
        }

        // 3. Clear temp memory
        temp_history.length = 0;
    }
}

/* ========================================================================= */
/* CV SIMULATION HELPERS                                                     */
/* ========================================================================= */
static void Draw_CV_Bounding_Box(uint16_t* buffer, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (!buffer) return;
    uint8_t thick = 4;
    for(uint16_t i = 0; i < w; i++) {
        for(uint8_t t = 0; t < thick; t++) {
            if (x+i < 320 && y+t < 240) buffer[(y+t) * 320 + (x + i)] = color;
            if (x+i < 320 && y+h-t < 240) buffer[(y + h - t) * 320 + (x + i)] = color;
        }
    }
    for(uint16_t i = 0; i < h; i++) {
        for(uint8_t t = 0; t < thick; t++) {
            if (x+t < 320 && y+i < 240) buffer[(y + i) * 320 + (x+t)] = color;
            if (x+w-t < 320 && y+i < 240) buffer[(y + i) * 320 + (x + w - t)] = color;
        }
    }
}

static void CV_Pipeline(uint16_t* target_buffer, MotorAngles_t* target_angles, SystemMode_t current_mode) {
    osDelay(10 + (rand() % 40));

    static int hold_frames = 0;
    static int16_t base_x = 140, base_y = 100, base_w = 40, base_h = 40;
    static float base_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

    if (hold_frames <= 0) {
        base_w = 40 + (rand() % 40);
        base_h = 40 + (rand() % 40);
        base_x = rand() % (320 - base_w);
        base_y = rand() % (240 - base_h);
        for (int i = 0; i < 5; i++) base_angles[i] = (float)(rand() % 181);
        hold_frames = 20 + (rand() % 60);
    } else {
        hold_frames--;
    }

    int16_t noisy_x = base_x + ((rand() % 5) - 2);
    int16_t noisy_y = base_y + ((rand() % 5) - 2);
    int16_t noisy_w = base_w + ((rand() % 5) - 2);
    int16_t noisy_h = base_h + ((rand() % 5) - 2);

    if (noisy_w < 10) noisy_w = 10;
    if (noisy_h < 10) noisy_h = 10;
    if (noisy_x < 0) noisy_x = 0;
    if (noisy_y < 0) noisy_y = 0;
    if (noisy_x + noisy_w > 320) noisy_x = 320 - noisy_w;
    if (noisy_y + noisy_h > 240) noisy_y = 240 - noisy_h;

    target_angles->box_x = noisy_x;
    target_angles->box_y = noisy_y;
    target_angles->box_w = noisy_w;
    target_angles->box_h = noisy_h;

    for (int i = 0; i < 5; i++) {
        float noise = ((float)(rand() % 51) / 10.0f) - 2.5f;
        float noisy_angle = base_angles[i] + noise;
        if (noisy_angle < 0.0f) noisy_angle = 0.0f;
        if (noisy_angle > 180.0f) noisy_angle = 180.0f;
        target_angles->angles[i] = noisy_angle;
    }
    target_angles->is_valid = 1;
}

/* ========================================================================= */
/* MAIN CORE COMPUTE ENGINE                                                  */
/* ========================================================================= */
void Compute_Motor_Angles(SystemMode_t current_mode, uint16_t* raw_frame_buffer, uint16_t* debug_frame_buffer, MotorAngles_t* output_angles) {
    output_angles->is_valid = 0;

    switch (current_mode) {
        case MODE_MAIN_MENU:
            break;

        case MODE_DEBUG:
            CV_Pipeline(raw_frame_buffer, &live_track_angles, current_mode);
            *output_angles = live_track_angles;
            break;

        case MODE_AUTO:
            CV_Pipeline(raw_frame_buffer, &live_track_angles, current_mode);
            *output_angles = live_track_angles;
            break;

        case MODE_MANUAL:
            // FIXED: The UI Task now completely owns the Motor Queue in Manual Mode.
            // Do NOT output anything here, or you will spam the queue and cause massive lag!
            output_angles->is_valid = 0;
            break;

        case MODE_TRACKING:
            if (current_track_state == TRACK_IDLE || current_track_state == TRACK_SAVING) {
            	CV_Pipeline(raw_frame_buffer, &live_track_angles, current_mode);
            }
            else if (current_track_state == TRACK_TRACKING) {
                CV_Pipeline(raw_frame_buffer, &live_track_angles, current_mode);
                *output_angles = live_track_angles;
            }
            else if (current_track_state == TRACK_RECORDING) {
                CV_Pipeline(raw_frame_buffer, &live_track_angles, current_mode);
                *output_angles = live_track_angles;

                if (temp_history.length < MAX_RECORD_FRAMES) {
                    temp_history.frames[temp_history.length++] = live_track_angles;
                } else {
                    current_track_state = TRACK_IDLE;
                }
            }
            else if (current_track_state == TRACK_REPLAYING) {
                if (replay_index < playback_target->length) {
                    live_track_angles = playback_target->frames[replay_index++];
                    *output_angles = live_track_angles;
                } else {
                    current_track_state = TRACK_IDLE;
                }
            }
            break;
    }
}
