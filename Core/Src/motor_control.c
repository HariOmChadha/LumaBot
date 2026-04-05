// motor_control.c
#include "motor_control.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

// The physical reality
float actual_motor_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
// The destination
float target_motor_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

// SPEED CONTROL: Maximum degrees the motor is allowed to move per RTOS tick
// (At 100Hz, 1.0f means 100 degrees per second. Lower this to move slower!)
const float MAX_STEP_SIZE = 1.0f;

void Motors_Start(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PB4
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // PH6
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // PA15
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  // PI0
}

// Just saves the destination from the RTOS Queue
void Motors_Set_Target(MotorAngles_t* cmd) {
    if (!cmd->is_valid) return;
    for (int i = 0; i < 5; i++) {
        target_motor_angles[i] = cmd->angles[i];
    }
}

// Called 100 times a second by the RTOS to smoothly inch the motors forward
void Motors_Tick(void) {
    uint8_t needs_update = 0;

    for (int i = 0; i < 5; i++) {
        if (actual_motor_angles[i] != target_motor_angles[i]) {
            needs_update = 1;
            float diff = target_motor_angles[i] - actual_motor_angles[i];

            // Slew Rate Limiter: Step towards the target
            if (diff > MAX_STEP_SIZE) {
                actual_motor_angles[i] += MAX_STEP_SIZE;
            } else if (diff < -MAX_STEP_SIZE) {
                actual_motor_angles[i] -= MAX_STEP_SIZE;
            } else {
                actual_motor_angles[i] = target_motor_angles[i]; // Arrived exactly
            }

            // Calculate the new PWM pulse width for this tiny step
            uint32_t pulse_width = 500 + (uint32_t)((actual_motor_angles[i] / 180.0f) * 2000.0f);

            // Update the hardware register
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
