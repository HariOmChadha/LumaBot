// motor_control.c
#include "motor_control.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

// physical state
float actual_motor_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
// target destination
float target_motor_angles[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

// speed control
// at 100Hz, 1.0f == 100 degrees/s - lower to move slower!)
const float MAX_STEP_SIZE = 0.5f;

void Motors_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PB4
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // PH6
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // PA15
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  // PI0
}

// get target from RTOS queue
void Motors_Set_Target(MotorAngles_t *cmd)
{
    if (!cmd->is_valid)
        return;
    for (int i = 0; i < 5; i++)
    {
        target_motor_angles[i] = cmd->angles[i];
    }
}

void Motors_Tick(void)
{

    for (int i = 0; i < 5; i++)
    {
        if (actual_motor_angles[i] != target_motor_angles[i])
        {
            float diff = target_motor_angles[i] - actual_motor_angles[i];

            // slew rate limiter
            if (diff > MAX_STEP_SIZE)
            {
                actual_motor_angles[i] += MAX_STEP_SIZE;
            }
            else if (diff < -MAX_STEP_SIZE)
            {
                actual_motor_angles[i] -= MAX_STEP_SIZE;
            }
            else
            {
                actual_motor_angles[i] = target_motor_angles[i];
            }

            // calculate needed PWM pulse width
            uint32_t pulse_width = 500 + (uint32_t)((actual_motor_angles[i] / 180.0f) * 2000.0f);

            // update registers
            switch (i)
            {
            case 0:
                TIM3->CCR1 = pulse_width;
                break;
            case 1:
                TIM1->CCR1 = pulse_width;
                break;
            case 2:
                TIM12->CCR1 = pulse_width;
                break;
            case 3:
                TIM2->CCR1 = pulse_width;
                break;
            case 4:
                TIM5->CCR4 = pulse_width;
                break;
            }
        }
    }
}
