#include "angle_compute.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "cmsis_os.h"
#include "lcd_display.h"

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
/* CCD INVERSE KINEMATICS                                                    */
/* ========================================================================= */

/* --- Arm geometry from URDF (origin xyz and rpy for each joint) --- */
static const float IK_JOINT_XYZ[5][3] = {
    { 0.000f,  0.000f,  0.000f },   /* joint1 */
    { 0.000f,  0.000f,  0.044f },   /* joint2 */
    {-0.140f,  0.000f,  0.000f },   /* joint3 */
    { 0.105f,  0.005f,  0.014f },   /* joint4 */
    { 0.016f, -0.006f,  0.029f },   /* joint5 */
};
static const float IK_JOINT_RPY[5][3] = {  /* [roll, pitch, yaw] */
    { 0.0f,      0.0f,      0.0f     },
    { 0.0f,      1.5707f,   0.0f     },
    { 3.1416f,   0.0f,      2.0943f  },
    {-1.5707f,  -1.5707f,  -1.5707f  },
    { 1.5707f,  -1.5707f,   1.5707f  },
};
static const float IK_EE_XYZ[3]    = { 0.030f, 0.0f, 0.0f };
static const float IK_Q_LOWER[5]   = {-1.5707f, -1.5707f, -1.5707f, -0.1f, -1.5707f};
static const float IK_Q_UPPER[5]   = { 1.5707f,  1.5707f,  1.5707f,  0.1f,  1.5707f};

#define CCD_MAX_ITER   100
#define CCD_TOL        0.008f   /* 5 mm position tolerance */

/* Persistent joint angles — warm-start each CCD call from last solution */
static float ik_q[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* 4x4 row-major homogeneous transform */
typedef struct { float m[4][4]; } IK_Mat4_t;

static IK_Mat4_t ik_mat4_identity(void) {
    IK_Mat4_t r = {{{ 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 }}};
    return r;
}

static IK_Mat4_t ik_mat4_mul(const IK_Mat4_t *a, const IK_Mat4_t *b) {
    IK_Mat4_t r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            r.m[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a->m[i][k] * b->m[k][j];
        }
    return r;
}

/* Build T = Trans(x,y,z) * Rz(yaw) * Ry(pitch) * Rx(roll)  (URDF RPY convention) */
static IK_Mat4_t ik_mat4_from_origin(float x, float y, float z,
                                     float roll, float pitch, float yaw) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);
    IK_Mat4_t r;
    r.m[0][0] = cy*cp;              r.m[0][1] = cy*sp*sr - sy*cr;  r.m[0][2] = cy*sp*cr + sy*sr;  r.m[0][3] = x;
    r.m[1][0] = sy*cp;              r.m[1][1] = sy*sp*sr + cy*cr;  r.m[1][2] = sy*sp*cr - cy*sr;  r.m[1][3] = y;
    r.m[2][0] = -sp;                r.m[2][1] = cp*sr;              r.m[2][2] = cp*cr;              r.m[2][3] = z;
    r.m[3][0] = 0.0f;               r.m[3][1] = 0.0f;               r.m[3][2] = 0.0f;               r.m[3][3] = 1.0f;
    return r;
}

static IK_Mat4_t ik_mat4_rotz(float a) {
    float c = cosf(a), s = sinf(a);
    IK_Mat4_t r = {{{ c,-s,0,0 },{ s,c,0,0 },{ 0,0,1,0 },{ 0,0,0,1 }}};
    return r;
}

static float ik_dot3(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static void ik_cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* Forward kinematics.
 * joint_pos[i]  = world-space pivot position of joint i (before q[i] rotation)
 * joint_axis[i] = world-space Z rotation axis of joint i (before q[i] rotation)
 * ee_pos        = world-space end-effector position
 * ee_x          = EE X axis in world (column 0 — camera forward / optical axis)
 * ee_heading    = EE Z axis in world (column 2 — camera left) */
static void IK_FK(const float q[5],
                  float joint_pos[5][3],
                  float joint_axis[5][3],
                  float ee_pos[3],
                  float ee_x[3],
                  float ee_heading[3])
{
    IK_Mat4_t T = ik_mat4_identity();
    for (int i = 0; i < 5; i++) {
        IK_Mat4_t off = ik_mat4_from_origin(
            IK_JOINT_XYZ[i][0], IK_JOINT_XYZ[i][1], IK_JOINT_XYZ[i][2],
            IK_JOINT_RPY[i][0], IK_JOINT_RPY[i][1], IK_JOINT_RPY[i][2]);
        T = ik_mat4_mul(&T, &off);

        /* Record pivot position and Z-axis (before q[i] is applied) */
        joint_pos[i][0]  = T.m[0][3];
        joint_pos[i][1]  = T.m[1][3];
        joint_pos[i][2]  = T.m[2][3];
        joint_axis[i][0] = T.m[0][2];   /* column 2 = Z axis */
        joint_axis[i][1] = T.m[1][2];
        joint_axis[i][2] = T.m[2][2];

        IK_Mat4_t Rz = ik_mat4_rotz(q[i]);
        T = ik_mat4_mul(&T, &Rz);
    }
    /* End-effector offset (fixed joint, pure translation in link5 frame) */
    ee_pos[0] = T.m[0][3] + T.m[0][0]*IK_EE_XYZ[0] + T.m[0][1]*IK_EE_XYZ[1] + T.m[0][2]*IK_EE_XYZ[2];
    ee_pos[1] = T.m[1][3] + T.m[1][0]*IK_EE_XYZ[0] + T.m[1][1]*IK_EE_XYZ[1] + T.m[1][2]*IK_EE_XYZ[2];
    ee_pos[2] = T.m[2][3] + T.m[2][0]*IK_EE_XYZ[0] + T.m[2][1]*IK_EE_XYZ[1] + T.m[2][2]*IK_EE_XYZ[2];
    ee_x[0] = T.m[0][0];  ee_x[1] = T.m[1][0];  ee_x[2] = T.m[2][0];   /* col 0 */
    ee_heading[0] = T.m[0][2];
    ee_heading[1] = T.m[1][2];
    ee_heading[2] = T.m[2][2];
}

/* CCD solver — position + optional EE X-axis orientation hold.
 *
 * target_pos  : desired EE world position
 * target_x    : desired EE X-axis direction (camera fwd), or NULL to skip
 * x_weight    : 0.0 = position only, 1.0 = orientation only (0.4 typical) */
static void IK_CCD_Solve(const float target_pos[3],
                         const float target_x[3],
                         float x_weight) {
    float joint_pos[5][3];
    float joint_axis[5][3];
    float ee_pos[3];
    float ee_x[3];
    float ee_heading[3];

    for (int iter = 0; iter < CCD_MAX_ITER; iter++) {
        IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_x, ee_heading);

        float dx = ee_pos[0] - target_pos[0];
        float dy = ee_pos[1] - target_pos[1];
        float dz = ee_pos[2] - target_pos[2];
        if (dx*dx + dy*dy + dz*dz < CCD_TOL*CCD_TOL) break;

        for (int i = 4; i >= 0; i--) {
            IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_x, ee_heading);
            const float *ax = joint_axis[i];

            /* --- Position correction --- */
            float r_ee[3]  = { ee_pos[0] - joint_pos[i][0],
                                ee_pos[1] - joint_pos[i][1],
                                ee_pos[2] - joint_pos[i][2] };
            float r_tgt[3] = { target_pos[0] - joint_pos[i][0],
                                target_pos[1] - joint_pos[i][1],
                                target_pos[2] - joint_pos[i][2] };

            float d_ee  = ik_dot3(r_ee,  ax);
            float d_tgt = ik_dot3(r_tgt, ax);
            float perp_ee[3]  = { r_ee[0]  - d_ee *ax[0], r_ee[1]  - d_ee *ax[1], r_ee[2]  - d_ee *ax[2] };
            float perp_tgt[3] = { r_tgt[0] - d_tgt*ax[0], r_tgt[1] - d_tgt*ax[1], r_tgt[2] - d_tgt*ax[2] };
            float len_ee  = sqrtf(ik_dot3(perp_ee,  perp_ee));
            float len_tgt = sqrtf(ik_dot3(perp_tgt, perp_tgt));

            float pos_angle = 0.0f;
            if (len_ee > 1e-6f && len_tgt > 1e-6f) {
                float cross[3];
                ik_cross3(perp_ee, perp_tgt, cross);
                float sin_a = ik_dot3(cross, ax) / (len_ee * len_tgt);
                float cos_a = ik_dot3(perp_ee, perp_tgt) / (len_ee * len_tgt);
                pos_angle = atan2f(sin_a, cos_a);
            }

            /* --- X-axis orientation hold (keep camera heading fixed) --- */
            float ori_angle = 0.0f;
            if (target_x != NULL && x_weight > 1e-6f) {
                float d_cur = ik_dot3(ee_x,    ax);
                float d_des = ik_dot3(target_x, ax);
                float perp_cur[3] = { ee_x[0]     - d_cur*ax[0],
                                       ee_x[1]     - d_cur*ax[1],
                                       ee_x[2]     - d_cur*ax[2] };
                float perp_des[3] = { target_x[0] - d_des*ax[0],
                                       target_x[1] - d_des*ax[1],
                                       target_x[2] - d_des*ax[2] };
                float len_cur = sqrtf(ik_dot3(perp_cur, perp_cur));
                float len_des = sqrtf(ik_dot3(perp_des, perp_des));
                if (len_cur > 1e-6f && len_des > 1e-6f) {
                    float cross[3];
                    ik_cross3(perp_cur, perp_des, cross);
                    float sin_a = ik_dot3(cross, ax) / (len_cur * len_des);
                    float cos_a = ik_dot3(perp_cur, perp_des) / (len_cur * len_des);
                    ori_angle = atan2f(sin_a, cos_a);
                }
            }

            float angle = (1.0f - x_weight) * pos_angle + x_weight * ori_angle;
            ik_q[i] += angle;
            if (ik_q[i] < IK_Q_LOWER[i]) ik_q[i] = IK_Q_LOWER[i];
            if (ik_q[i] > IK_Q_UPPER[i]) ik_q[i] = IK_Q_UPPER[i];
        }
    }
}

/* ========================================================================= */
/* COLOR BLOB DETECTOR                                                       */
/* ========================================================================= */

/* RGB565 blob detection.
 * Channels: R = 5-bit (0-31), G = 6-bit (0-63), B = 5-bit (0-31)
 * Calibrate by holding glove in front of camera in MODE_DEBUG and reading
 * the centre pixel R/G/B values from the LCD. */
#define BLOB_R_MIN   0
#define BLOB_R_MAX   10
#define BLOB_G_MIN   0
#define BLOB_G_MAX   15
#define BLOB_B_MIN   0
#define BLOB_B_MAX   10
#define BLOB_MIN_PIXELS 100

static uint32_t Blob_Detect(uint16_t* buffer, float* out_cx, float* out_cy) {
    uint32_t sum_x = 0, sum_y = 0, count = 0;

    for (uint16_t y = 0; y < 240; y++) {
        const uint16_t* row = buffer + y * 320;
        for (uint16_t x = 0; x < 320; x++) {
            uint16_t px = row[x];
            uint8_t r = (px >> 11) & 0x1F;
            uint8_t g = (px >>  5) & 0x3F;
            uint8_t b =  px        & 0x1F;

            if (r >= BLOB_R_MIN && r <= BLOB_R_MAX &&
                g >= BLOB_G_MIN && g <= BLOB_G_MAX &&
                b >= BLOB_B_MIN && b <= BLOB_B_MAX) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if (count < BLOB_MIN_PIXELS) return 0;

    *out_cx = (float)sum_x / (float)count;
    *out_cy = (float)sum_y / (float)count;
    return count;
}

/* -------------------------------------------------------------------------
 * Pixels-from-centre to metres step gain (tune per camera FOV).
 * Each frame, centroid error drives a small world-space position step.
 * Fixed overhead EE position for MODE_TRACKING (metres).
 * Assumed hand depth for MODE_TRACKING pointing direction.
 * ------------------------------------------------------------------------- */
#define AUTO_STEP_GAIN   0.0003f   /* m per pixel — MODE_AUTO position step   */
#define LAMP_ORI_GAIN    0.005f    /* rad per pixel — MODE_TRACKING tilt gain  */
#define LAMP_POS_Z       0.30f     /* target overhead Z height (m)             */
#define LAMP_CENTRE_X    0.0f      /* fallback X when arm can't reach height   */
#define LAMP_CENTRE_Y    0.15f     /* fallback Y when arm can't reach height   */

/* Last computed EE position — updated by IK_Set_Angles, readable via getter */
static float last_ee_pos[3] = {0.0f, 0.0f, 0.0f};

/* -------------------------------------------------------------------------
 * High-level controller: centroid + mode → IK solve → output_angles
 *
 * MODE_AUTO — hand tracking (position control):
 *   Each frame the centroid error (pixels from frame centre) is converted
 *   to a small world-space step along the EE's current image-plane axes
 *   (EE X = camera right, EE Y = camera up).  The arm moves so the hand
 *   drifts toward the frame centre over successive frames.
 *   Position-only CCD (no orientation target).
 *
 * MODE_TRACKING — lighting workstation (orientation control):
 *   EE position is locked to a fixed overhead point (LAMP_POS_*).
 *   The EE Z-axis is steered to point from that overhead point toward the
 *   estimated hand world position, so the "lamp" always faces the hand.
 *   Orientation-only CCD (ori_weight = 1.0).
 * ------------------------------------------------------------------------- */
/* Joint 5 servo angle (degrees) when entering MODE_AUTO.
 * 40° servo = -50° IK = camera pointing downward. */
#define AUTO_INIT_J5_DEG  40.0f

static void IK_Set_Angles(float cx, float cy, SystemMode_t mode,
                           MotorAngles_t *output_angles) {


    /* Run FK once to get current EE pose and all three axes */
    float joint_pos[5][3], joint_axis[5][3], ee_pos[3], ee_x_axis[3], ee_z[3];
    IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_x_axis, ee_z);
    last_ee_pos[0] = ee_pos[0];
    last_ee_pos[1] = ee_pos[1];
    last_ee_pos[2] = ee_pos[2];

    /* EE Y axis from the same transform (column 1) — reuse the loop */
    IK_Mat4_t T = ik_mat4_identity();
    for (int i = 0; i < 5; i++) {
        IK_Mat4_t off = ik_mat4_from_origin(
            IK_JOINT_XYZ[i][0], IK_JOINT_XYZ[i][1], IK_JOINT_XYZ[i][2],
            IK_JOINT_RPY[i][0], IK_JOINT_RPY[i][1], IK_JOINT_RPY[i][2]);
        T = ik_mat4_mul(&T, &off);
        IK_Mat4_t Rz = ik_mat4_rotz(ik_q[i]);
        T = ik_mat4_mul(&T, &Rz);
    }
    float ee_y_axis[3] = { T.m[0][1], T.m[1][1], T.m[2][1] };


    /* ------------------------------------------------------------------
     * Both MODE_AUTO and MODE_TRACKING: nudge EE position in image plane.
     *
     * EE frame: X = camera fwd, Y = camera down, Z = camera left.
     * camera right = -ee_z,  camera down = +ee_y_axis
     *
     * err_x > 0 → hand right of centre → step along -ee_z
     * err_y > 0 → hand below centre    → step along +ee_y_axis
     *
     * MODE_AUTO starts with joint 5 at AUTO_INIT_J5_DEG (looking down)
     * so the arm begins from a high overhead pose.
     * ------------------------------------------------------------------ */
    float err_x = (cx - 160.0f);
    float err_y = (cy - 120.0f);

    /* Dead zone — zero out error when centroid is within DEAD_ZONE pixels
     * of centre so the arm holds still when the detection is centred. */
#define DEAD_ZONE  50.0f
    if (err_x > -DEAD_ZONE && err_x < DEAD_ZONE) err_x = 0.0f;
    if (err_y > -DEAD_ZONE && err_y < DEAD_ZONE) err_y = 0.0f;

    float target[3] = {
        ee_pos[0] + AUTO_STEP_GAIN * (-err_x * ee_z[0] + err_y * ee_y_axis[0]),
        ee_pos[1] + AUTO_STEP_GAIN * (-err_x * ee_z[1] + err_y * ee_y_axis[1]),
        ee_pos[2] + AUTO_STEP_GAIN * (-err_x * ee_z[2] + err_y * ee_y_axis[2]),
    };

#define MAX_EE_STEP  0.05f
    float step[3] = { target[0] - ee_pos[0],
                      target[1] - ee_pos[1],
                      target[2] - ee_pos[2] };
    float step_len = sqrtf(step[0]*step[0] + step[1]*step[1] + step[2]*step[2]);
    if (step_len > MAX_EE_STEP) {
        float scale = MAX_EE_STEP / step_len;
        target[0] = ee_pos[0] + step[0] * scale;
        target[1] = ee_pos[1] + step[1] * scale;
        target[2] = ee_pos[2] + step[2] * scale;
    }

    IK_CCD_Solve(target, NULL, 0.0f);

    /* Rate limiter — clamp how much each joint can move per frame.
     * MAX_DELTA_DEG is the max change in servo degrees per frame.
     * Reduce this to slow the arm down further. */
#define MAX_DELTA_DEG  1.0f

    for (int i = 0; i < 5; i++) {
        float new_angle = ik_q[i] * (180.0f / 3.14159265f) + 90.0f;
        float delta = new_angle - actual_motor_angles[i];
        if (delta >  MAX_DELTA_DEG) delta =  MAX_DELTA_DEG;
        if (delta < -MAX_DELTA_DEG) delta = -MAX_DELTA_DEG;
        output_angles->angles[i] = actual_motor_angles[i] + delta;

        /* Sync ik_q back so the next CCD warm-start matches what we actually
         * output. Without this, ik_q drifts away from the real arm position
         * and the rate limiter becomes ineffective. */
        ik_q[i] = (output_angles->angles[i] - 90.0f) * (3.14159265f / 180.0f);
    }
    output_angles->is_valid = 1;
}

void IK_Get_EE_Pos(float out[3]) {
    out[0] = last_ee_pos[0];
    out[1] = last_ee_pos[1];
    out[2] = last_ee_pos[2];
}

#define CENTROID_BOX_HALF 30

/* Temporal stability: blob must stay within CONFIRM_RADIUS for CONFIRM_FRAMES
 * consecutive frames before it triggers IK or draws a bounding box. */
#define CONFIRM_FRAMES   3
#define CONFIRM_RADIUS   120.0f

uint32_t       blob_pixels    = 0;
static uint8_t confirm_count  = 0;
static float   confirm_cx     = 0.0f;
static float   confirm_cy     = 0.0f;

static void CV_Pipeline(uint16_t* target_buffer, MotorAngles_t* target_angles, SystemMode_t current_mode) {
    float cx, cy;
    uint32_t count = Blob_Detect(target_buffer, &cx, &cy);
    blob_pixels = count;

    if (!count) {
        confirm_count = 0;
        target_angles->is_valid = 0;
        return;
    }

    if (confirm_count == 0) {
        confirm_cx    = cx;
        confirm_cy    = cy;
        confirm_count = 1;
        target_angles->is_valid = 0;
        return;
    }

    float dx = cx - confirm_cx;
    float dy = cy - confirm_cy;
    if (dx*dx + dy*dy > CONFIRM_RADIUS * CONFIRM_RADIUS) {
        /* Blob jumped — restart streak */
        confirm_cx    = cx;
        confirm_cy    = cy;
        confirm_count = 1;
        target_angles->is_valid = 0;
        return;
    }

    /* Running average of confirmed position */
    confirm_cx = (confirm_cx * confirm_count + cx) / (confirm_count + 1);
    confirm_cy = (confirm_cy * confirm_count + cy) / (confirm_count + 1);
    if (confirm_count < CONFIRM_FRAMES) {
        confirm_count++;
        target_angles->is_valid = 0;
        return;
    }

    /* Confirmed — drive IK directly from raw centroid */
    target_angles->box_x = (uint16_t)(confirm_cx > CENTROID_BOX_HALF ? confirm_cx - CENTROID_BOX_HALF : 0);
    target_angles->box_y = (uint16_t)(confirm_cy > CENTROID_BOX_HALF ? confirm_cy - CENTROID_BOX_HALF : 0);
    target_angles->box_w = 2 * CENTROID_BOX_HALF;
    target_angles->box_h = 2 * CENTROID_BOX_HALF;

    IK_Set_Angles(confirm_cx, confirm_cy, current_mode, target_angles);
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
