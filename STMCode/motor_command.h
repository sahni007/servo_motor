#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include "stm8s.h"

typedef struct
{
    int8_t direction;
    uint8_t speed;
    uint8_t valid;
} MotorCommand;

#endif
