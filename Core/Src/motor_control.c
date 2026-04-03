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

    for (int i = 0; i < 5; i++) {
        // Check if this specific motor's angle has changed
        if (actual_motor_angles[i] != cmd->angles[i]) {

            // 1. Update the global tracking array
            actual_motor_angles[i] = cmd->angles[i];

            // 2. Calculate the new PWM pulse width just for this motor
            uint32_t pulse_width = 500 + (uint32_t)((actual_motor_angles[i] / 180.0f) * 2000.0f);

            // 3. Update ONLY the corresponding hardware register
            switch (i) {
                case 0: TIM3->CCR1  = pulse_width; break;
                case 1: TIM1->CCR1  = pulse_width; break;
                case 2: TIM12->CCR1 = pulse_width; break;
                case 3: TIM2->CCR1  = pulse_width; break;
                case 4: TIM5->CCR4  = pulse_width; break;
            }
        }
    }
}
