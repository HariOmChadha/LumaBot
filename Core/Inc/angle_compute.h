#ifndef ANGLE_COMPUTE_H
#define ANGLE_COMPUTE_H

#include <stdint.h>

/* --- System Modes --- */
typedef enum {
    MODE_MAIN_MENU = 0,
    MODE_DEBUG = 1,
    MODE_AUTO = 2,
    MODE_MANUAL = 3,
    MODE_TRACKING = 4
} SystemMode_t;

/* --- Tracking States for Mode 4 --- */
typedef enum {
    TRACK_IDLE = 0,
    TRACK_TRACKING = 1,
    TRACK_RECORDING = 2,
    TRACK_REPLAYING = 3,
    TRACK_SAVING = 4
} TrackState_t;

#define MAX_RECORD_FRAMES 150  // ~5 seconds of recording at 30fps

/* --- Motor Command Structure --- */
typedef struct {
    float angles[5];      // The 5 target angles
    uint8_t is_valid;     // 1 if data is fresh/safe, 0 to ignore

    // Debug info passed back to the UI
    uint16_t box_x;
    uint16_t box_y;
    uint16_t box_w;
    uint16_t box_h;
} MotorAngles_t;


/* --- Mode 3 API --- */
void Set_Manual_Angle(uint8_t joint_index, float angle_value);
float Get_Manual_Angle(uint8_t joint_index);
void Save_Manual_Preset(uint8_t preset_slot);
void Load_Manual_Preset(uint8_t preset_slot);

/* --- Mode 4 API --- */
void Track_Enter_Mode(void);
void Track_Exit_Mode(void);
void Track_Set_State(TrackState_t new_state);
TrackState_t Track_Get_State(void);

uint8_t Track_Has_Temp_Data(void);
uint8_t Track_Has_Loaded_Preset(void);
void Track_Clear_Loaded_Preset(void);

void Track_Save_To_Preset(uint8_t preset_slot);
void Track_Load_Preset(uint8_t preset_slot);

/* --- Main Compute Entry Point --- */
void Compute_Motor_Angles(SystemMode_t current_mode, uint16_t* raw_frame_buffer, uint16_t* debug_frame_buffer, MotorAngles_t* output_angles);

/* --- IK Debug --- */
void IK_Get_EE_Pos(float out[3]);   /* returns last computed EE position (metres) */

#endif /* ANGLE_COMPUTE_H */
