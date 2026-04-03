// motor_control.c
#include "motor_control.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

// The ultimate source of truth for the physical motors
float actual_motor_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

void Motors_Start(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PB4
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // PH6
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // PA15
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  // PI0
}

void Motors_Update(MotorAngles_t* cmd) {
    if (!cmd->is_valid) return;

    // 1. Sync the global tracker to reality
    for (int i = 0; i < 5; i++) {
        actual_motor_angles[i] = cmd->angles[i];
    }

    // 2. Convert 0-180 degrees to 500-2500 microsecond PWM pulse
    // (Assuming ARR is 20000 and prescaler is set for 1us ticks)
    TIM3->CCR1  = 500 + (uint32_t)((actual_motor_angles[0] / 180.0f) * 2000.0f);
    TIM1->CCR1  = 500 + (uint32_t)((actual_motor_angles[1] / 180.0f) * 2000.0f);
    TIM12->CCR1 = 500 + (uint32_t)((actual_motor_angles[2] / 180.0f) * 2000.0f);
    TIM2->CCR1  = 500 + (uint32_t)((actual_motor_angles[3] / 180.0f) * 2000.0f);
    TIM5->CCR4  = 500 + (uint32_t)((actual_motor_angles[4] / 180.0f) * 2000.0f);
}
