#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_command.h"

void MotorControl_Init(void);
void MotorControl_Apply(const MotorCommand *command1,
                        const MotorCommand *command2);

#endif
