#include "stm8s.h"
#include "motor_control.h"

#define REG8(address) (*(volatile uint8_t *)(address))

#define GPIOC_ODR   REG8(0x500A)
#define DRIVER_ENABLE 0x08
#define MOTOR1_A_MASK 0x40
#define MOTOR2_A_MASK 0x80

#define TIM2_CR1    REG8(0x5300)
#define TIM2_EGR    REG8(0x5306)
#define TIM2_CCMR1  REG8(0x5307)
#define TIM2_CCMR3  REG8(0x5309)
#define TIM2_CCER1  REG8(0x530A)
#define TIM2_CCER2  REG8(0x530B)
#define TIM2_PSCR   REG8(0x530E)
#define TIM2_ARRH   REG8(0x530F)
#define TIM2_ARRL   REG8(0x5310)
#define TIM2_CCR1H  REG8(0x5311)
#define TIM2_CCR1L  REG8(0x5312)
#define TIM2_CCR3H  REG8(0x5315)
#define TIM2_CCR3L  REG8(0x5316)

#define PWM_PERIOD_COUNTS 1000U

/*******************************************************************************
 * @brief   Writes an 8-bit duty command to a TIM2 PWM channel.
 *
 * @details Scales the command to the configured period and writes the compare
 *          register for channel 1 or channel 3.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   channel  TIM2 channel number: 1 selects channel 1; otherwise 3.
 * @param   value    PWM duty command from 0 to 255.
 *
 * @return  None.
 ******************************************************************************/
static void set_pwm(uint8_t channel, uint8_t value)
{
    uint16_t compare;

    compare = (uint16_t)(((uint32_t)PWM_PERIOD_COUNTS * value) / 255UL);
    if (channel == 1)
    {
        TIM2_CCR1H = (uint8_t)(compare >> 8);
        TIM2_CCR1L = (uint8_t)compare;
    }
    else
    {
        TIM2_CCR3H = (uint8_t)(compare >> 8);
        TIM2_CCR3L = (uint8_t)compare;
    }
}

/*******************************************************************************
 * @brief   Applies direction and PWM duty to one motor.
 *
 * @details Sets the direction GPIO and updates its PWM channel. A zero
 *          direction clears the direction pin and sets duty to zero.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   direction_mask  GPIOC bit mask for the motor direction output.
 * @param   pwm_channel     TIM2 PWM channel used by the motor.
 * @param   direction       Direction: 1 forward, -1 reverse, 0 stopped.
 * @param   speed           PWM speed command from 0 to 255.
 *
 * @return  None.
 ******************************************************************************/
static void drive_motor(uint8_t direction_mask, uint8_t pwm_channel,
                        int8_t direction, uint8_t speed)
{
    if (direction > 0)
    {
        GPIOC_ODR |= direction_mask;
        set_pwm(pwm_channel, (uint8_t)(255U - speed));
    }
    else if (direction < 0)
    {
        GPIOC_ODR &= (uint8_t)~direction_mask;
        set_pwm(pwm_channel, speed);
    }
    else
    {
        GPIOC_ODR &= (uint8_t)~direction_mask;
        set_pwm(pwm_channel, 0);
    }
}

/*******************************************************************************
 * @brief   Configures TIM2 channels 1 and 3 for motor PWM.
 *
 * @details Sets a 1000-count PWM period, initializes both compare registers
 *          to zero, enables PWM outputs, and starts the timer.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void MotorControl_Init(void)
{
    TIM2_CR1 = 0x00;
    TIM2_CCER1 = 0x00;
    TIM2_CCER2 = 0x00;
    TIM2_CCMR1 = 0x68;
    TIM2_CCMR3 = 0x68;
    TIM2_PSCR = 0x00;
    TIM2_ARRH = 0x03;
    TIM2_ARRL = 0xE7;
    TIM2_CCR1H = 0x00;
    TIM2_CCR1L = 0x00;
    TIM2_CCR3H = 0x00;
    TIM2_CCR3L = 0x00;
    TIM2_EGR = 0x01;
    TIM2_CCER1 = 0x01;
    TIM2_CCER2 = 0x01;
    TIM2_CR1 = 0x81;
}

/*******************************************************************************
 * @brief   Applies both motor commands and updates driver enable.
 *
 * @details Drives each motor using its direction and speed, then enables the
 *          driver while at least one servo command is valid.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   command1  Motor command for motor 1.
 * @param   command2  Motor command for motor 2.
 *
 * @return  None.
 ******************************************************************************/
void MotorControl_Apply(const MotorCommand *command1,
                        const MotorCommand *command2)
{
    drive_motor(MOTOR1_A_MASK, 1, command1->direction, command1->speed);
    drive_motor(MOTOR2_A_MASK, 3, command2->direction, command2->speed);

    if (command1->valid || command2->valid)
        GPIOC_ODR |= DRIVER_ENABLE;
    else
        GPIOC_ODR &= (uint8_t)~DRIVER_ENABLE;
}
