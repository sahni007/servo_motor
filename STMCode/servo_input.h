#ifndef SERVO_INPUT_H
#define SERVO_INPUT_H

#include "motor_command.h"

void ServoInput_Init(void);
void ServoInput_Read(MotorCommand *command1, MotorCommand *command2);

#endif
