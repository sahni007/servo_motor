#include "stm8s.h"
#include "app_config.h"
#include "board.h"
#include "fault_monitor.h"
#include "motor_control.h"
#include "motor_command.h"
#include "servo_input.h"
#include "status_led.h"

int main(void)
{
    MotorCommand command1;
    MotorCommand command2;
    uint8_t fault_active;

    Board_Init();
    ServoInput_Init();
    MotorControl_Init();
    FaultMonitor_Init();
    rim();

    for (;;)
    {
        ServoInput_Read(&command1, &command2);
        fault_active = FaultMonitor_Update();

#if FAULT_SHUTDOWN
        if (fault_active)
        {
            command1.valid = 0;
            command1.direction = 0;
            command1.speed = 0;
            command2.valid = 0;
            command2.direction = 0;
            command2.speed = 0;
        }
#endif

        MotorControl_Apply(&command1, &command2);
        StatusLed_Update(fault_active);
    }
}
