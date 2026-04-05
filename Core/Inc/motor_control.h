#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f7xx_hal.h"
#include "angle_compute.h" // Needed for MotorAngles_t

void Motors_Start(void);
void Motors_Update(MotorAngles_t* cmd);
void Motors_Tick(void);
void Motors_Set_Target(MotorAngles_t* cmd);

#endif /* MOTOR_CONTROL_H */
