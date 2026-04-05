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
static const float IK_EE_XYZ[3]    = { 0.025f, 0.0f, 0.0f };
static const float IK_Q_LOWER[5]   = {-1.5707f, -1.5707f, -1.5707f, -0.1f, -1.5707f};
static const float IK_Q_UPPER[5]   = { 1.5707f,  1.5707f,  1.5707f,  0.1f,  1.5707f};

#define CCD_MAX_ITER   100
#define CCD_TOL        0.005f   /* 5 mm position tolerance */

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
 * ee_pos        = world-space end-effector position */
static void IK_FK(const float q[5],
                  float joint_pos[5][3],
                  float joint_axis[5][3],
                  float ee_pos[3], 
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
    ee_heading[0] = T.m[0][2];
    ee_heading[1] = T.m[1][2];
    ee_heading[2] = T.m[2][2];
}

/* CCD solver — position + optional Z-axis orientation.
 *
 * target_pos     : desired EE world position
 * target_z       : desired EE Z-axis direction (unit vector), or NULL to skip
 * ori_weight     : 0.0 = position only, 1.0 = orientation only, 0.3 = typical blend
 *
 * Per joint, the total angle correction is:
 *   angle = (1 - ori_weight) * pos_angle  +  ori_weight * ori_angle
 * Both corrections use identical CCD geometry — project vectors onto the plane
 * perpendicular to the joint axis, compute signed angle between them. */
static void IK_CCD_Solve(const float target_pos[3],
                         const float target_z[3],
                         float ori_weight) {
    float joint_pos[5][3];
    float joint_axis[5][3];
    float ee_pos[3];
    float ee_heading[3];

    for (int iter = 0; iter < CCD_MAX_ITER; iter++) {
        IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_heading);

        float dx = ee_pos[0] - target_pos[0];
        float dy = ee_pos[1] - target_pos[1];
        float dz = ee_pos[2] - target_pos[2];
        if (dx*dx + dy*dy + dz*dz < CCD_TOL*CCD_TOL) break;

        for (int i = 4; i >= 0; i--) {
            IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_heading);
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

            /* --- Orientation correction (EE Z-axis toward target_z) --- */
            float ori_angle = 0.0f;
            if (target_z != NULL && ori_weight > 1e-6f) {
                float d_cur = ik_dot3(ee_heading, ax);
                float d_des = ik_dot3(target_z,   ax);
                float perp_cur[3] = { ee_heading[0] - d_cur*ax[0],
                                       ee_heading[1] - d_cur*ax[1],
                                       ee_heading[2] - d_cur*ax[2] };
                float perp_des[3] = { target_z[0]   - d_des*ax[0],
                                       target_z[1]   - d_des*ax[1],
                                       target_z[2]   - d_des*ax[2] };
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

            float angle = (1.0f - ori_weight) * pos_angle
                        +          ori_weight  * ori_angle;

            ik_q[i] += angle;
            if (ik_q[i] < IK_Q_LOWER[i]) ik_q[i] = IK_Q_LOWER[i];
            if (ik_q[i] > IK_Q_UPPER[i]) ik_q[i] = IK_Q_UPPER[i];
        }
    }
}

/* ========================================================================= */
/* COLOR BLOB DETECTOR                                                       */
/* ========================================================================= */

/* Glove color range in RGB565 channel values.
 *
 * How to calibrate:
 *   1. Hold your glove in front of the camera in MODE_DEBUG
 *   2. Note the pixel values at the glove location
 *   3. Set ranges with some margin around those values
 *
 * Channels:  R = 5-bit (0-31),  G = 6-bit (0-63),  B = 5-bit (0-31)
 *
 * Defaults below are for a BRIGHT GREEN glove. Change to match yours. */
#define BLOB_R_MIN   2
#define BLOB_R_MAX   12
#define BLOB_G_MIN   25
#define BLOB_G_MAX   38
#define BLOB_B_MIN   15
#define BLOB_B_MAX   22

/* Minimum number of matching pixels to count as a valid detection.
 * Increase to reject small false positives, decrease if glove is far away. */
#define BLOB_MIN_PIXELS  300

/* Returns 1 if a blob was found, 0 otherwise.
 * out_cx, out_cy are the centroid in image pixels (0-320, 0-240).
 * Naturally finds the center between two gloves if both are visible. */
static uint8_t Blob_Detect(uint16_t* buffer, float* out_cx, float* out_cy) {
    uint32_t sum_x = 0, sum_y = 0, count = 0;

    for (uint16_t y = 0; y < 240; y++) {
        const uint16_t* row = buffer + y * 320;
        for (uint16_t x = 0; x < 320; x++) {
            uint16_t px = row[x];
            uint8_t r = (px >> 11) & 0x1F;
            uint8_t g = (px >> 5)  & 0x3F;
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
    return count;  /* return pixel count so caller can use it for debug */
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
static void IK_Set_Angles(float cx, float cy, SystemMode_t mode,
                           MotorAngles_t *output_angles) {

    /* Run FK once to get current EE pose */
    float joint_pos[5][3], joint_axis[5][3], ee_pos[3], ee_z[3];
    IK_FK(ik_q, joint_pos, joint_axis, ee_pos, ee_z);

    /* Also need EE X and Y axes — recompute full FK transform */
    IK_Mat4_t T = ik_mat4_identity();
    for (int i = 0; i < 5; i++) {
        IK_Mat4_t off = ik_mat4_from_origin(
            IK_JOINT_XYZ[i][0], IK_JOINT_XYZ[i][1], IK_JOINT_XYZ[i][2],
            IK_JOINT_RPY[i][0], IK_JOINT_RPY[i][1], IK_JOINT_RPY[i][2]);
        T = ik_mat4_mul(&T, &off);
        IK_Mat4_t Rz = ik_mat4_rotz(ik_q[i]);
        T = ik_mat4_mul(&T, &Rz);
    }
    float ee_x_axis[3] = { T.m[0][0], T.m[1][0], T.m[2][0] };
    float ee_y_axis[3] = { T.m[0][1], T.m[1][1], T.m[2][1] };

    if (mode == MODE_TRACKING) {
        /* ------------------------------------------------------------------
         * LIGHTING WORKSTATION: fixed overhead position, steer EE Z-axis
         * toward the hand using centroid error directly in image-plane coords.
         *
         * EE Z is the "looking" direction (optical axis).
         * EE X and EE Y span the image plane.
         *
         * centroid error in pixels → tilt Z-axis by that fraction:
         *   err_x > 0 : hand right of centre → tilt Z toward +ee_x
         *   err_y > 0 : hand below centre    → tilt Z toward -ee_y
         *             (image Y is flipped relative to world Y)
         *
         * desired_z = normalize(ee_z + gain * (err_x * ee_x - err_y * ee_y))
         *
         * No depth assumption needed — centroid error alone drives orientation.
         * ------------------------------------------------------------------ */
        float err_x = (cx - 160.0f);
        float err_y = (cy - 120.0f);

        /* Desired EE Z-axis: tilt current Z toward the hand centroid */
        float dz[3] = {
            ee_z[0] + LAMP_ORI_GAIN * (err_x * ee_x_axis[0] - err_y * ee_y_axis[0]),
            ee_z[1] + LAMP_ORI_GAIN * (err_x * ee_x_axis[1] - err_y * ee_y_axis[1]),
            ee_z[2] + LAMP_ORI_GAIN * (err_x * ee_x_axis[2] - err_y * ee_y_axis[2]),
        };
        float dz_len = sqrtf(dz[0]*dz[0] + dz[1]*dz[1] + dz[2]*dz[2]);
        if (dz_len > 1e-6f) { dz[0] /= dz_len; dz[1] /= dz_len; dz[2] /= dz_len; }

        /* Lamp position: keep current EE X and Y, only lift Z to 30 cm.
         * First try with X/Y unchanged. If CCD doesn't converge (EE stays
         * far from target), pull X/Y back toward a neutral position. */
        float lamp[3] = { ee_pos[0], ee_pos[1], LAMP_POS_Z };
        IK_CCD_Solve(lamp, dz, 1.0f);

        /* Check convergence — if EE is still far from the target Z,
         * relax X and Y toward centre so the arm can reach the height. */
        float jp[5][3], ja[5][3], ep[3], ez[3];
        IK_FK(ik_q, jp, ja, ep, ez);
        float z_err = ep[2] - LAMP_POS_Z;
        if (z_err * z_err > CCD_TOL * CCD_TOL) {
            /* Nudge X/Y toward the reachable overhead centre */
            lamp[0] = ee_pos[0] * 0.9f + LAMP_CENTRE_X * 0.1f;
            lamp[1] = ee_pos[1] * 0.9f + LAMP_CENTRE_Y * 0.1f;
            IK_CCD_Solve(lamp, dz, 1.0f);
        }

    } else {
        /* ------------------------------------------------------------------
         * STANDARD TRACKING: nudge EE position in image plane each frame.
         * err_x > 0 → hand right of centre → step EE along +ee_x_axis
         * err_y > 0 → hand below centre   → step EE along -ee_y_axis
         * ------------------------------------------------------------------ */
        float err_x =  (cx - 160.0f);
        float err_y =  (cy - 120.0f);

        float target[3] = {
            ee_pos[0] + AUTO_STEP_GAIN * (err_x * ee_x_axis[0] - err_y * ee_y_axis[0]),
            ee_pos[1] + AUTO_STEP_GAIN * (err_x * ee_x_axis[1] - err_y * ee_y_axis[1]),
            ee_pos[2] + AUTO_STEP_GAIN * (err_x * ee_x_axis[2] - err_y * ee_y_axis[2]),
        };

        IK_CCD_Solve(target, NULL, 0.0f);   /* position only, no orientation */
    }

    for (int i = 0; i < 5; i++) {
        output_angles->angles[i] = ik_q[i] * (180.0f / 3.14159265f) + 90.0f;
    }
    output_angles->is_valid = 1;
}

/* -------------------------------------------------------------------------
 * Centroid moving average + outlier rejection
 *
 * MA_LEN       : number of frames averaged (larger = smoother, more lag)
 * OUTLIER_DIST : pixel distance from current average beyond which a new
 *                detection is rejected as an outlier (tune to your scene)
 * ------------------------------------------------------------------------- */
#define MA_LEN            8
#define OUTLIER_DIST      120.0f
#define CENTROID_BOX_HALF 30   /* half-size of the box drawn around the centroid (pixels) */

static float ma_cx[MA_LEN];
static float ma_cy[MA_LEN];
static uint8_t  ma_idx      = 0;
static uint8_t  ma_count    = 0;
uint32_t        blob_pixels = 0;  /* exposed for debug display */

static void CV_Pipeline(uint16_t* target_buffer, MotorAngles_t* target_angles, SystemMode_t current_mode) {
    float cx, cy;
    uint32_t count = Blob_Detect(target_buffer, &cx, &cy);
    blob_pixels = count;
    if (!count) {
        target_angles->is_valid = 0;
        return;
    }

    /* Compute current moving average */
    float avg_cx = cx, avg_cy = cy;
    if (ma_count > 0) {
        float sum_x = 0.0f, sum_y = 0.0f;
        for (uint8_t i = 0; i < ma_count; i++) {
            sum_x += ma_cx[i];
            sum_y += ma_cy[i];
        }
        avg_cx = sum_x / ma_count;
        avg_cy = sum_y / ma_count;
    }

    /* Outlier rejection — skip if too far from current average */
    if (ma_count > 0) {
        float dx = cx - avg_cx;
        float dy = cy - avg_cy;
        if (dx*dx + dy*dy > OUTLIER_DIST * OUTLIER_DIST) {
            /* Rejected — hold last known good position */
            target_angles->box_x = (uint16_t)(avg_cx > CENTROID_BOX_HALF ? avg_cx - CENTROID_BOX_HALF : 0);
            target_angles->box_y = (uint16_t)(avg_cy > CENTROID_BOX_HALF ? avg_cy - CENTROID_BOX_HALF : 0);
            target_angles->box_w = 2 * CENTROID_BOX_HALF;
            target_angles->box_h = 2 * CENTROID_BOX_HALF;
            target_angles->is_valid = 1;
            return;
        }
    }

    /* Accepted — push into circular buffer and recompute average */
    ma_cx[ma_idx] = cx;
    ma_cy[ma_idx] = cy;
    ma_idx = (ma_idx + 1) % MA_LEN;
    if (ma_count < MA_LEN) ma_count++;

    float sum_x = 0.0f, sum_y = 0.0f;
    for (uint8_t i = 0; i < ma_count; i++) {
        sum_x += ma_cx[i];
        sum_y += ma_cy[i];
    }
    avg_cx = sum_x / ma_count;
    avg_cy = sum_y / ma_count;

    target_angles->box_x = (uint16_t)(avg_cx > CENTROID_BOX_HALF ? avg_cx - CENTROID_BOX_HALF : 0);
    target_angles->box_y = (uint16_t)(avg_cy > CENTROID_BOX_HALF ? avg_cy - CENTROID_BOX_HALF : 0);
    target_angles->box_w = 2 * CENTROID_BOX_HALF;
    target_angles->box_h = 2 * CENTROID_BOX_HALF;

    IK_Set_Angles(avg_cx, avg_cy, current_mode, target_angles);

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
