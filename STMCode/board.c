#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "board.h"

#define REG8(address) (*(volatile uint8_t *)(address))

#define CLK_CKDIVR REG8(0x50C6)

#define GPIOA_ODR  REG8(0x5000)
#define GPIOA_DDR  REG8(0x5002)
#define GPIOA_CR1  REG8(0x5003)
#define GPIOA_CR2  REG8(0x5004)

#define GPIOC_ODR  REG8(0x500A)
#define GPIOC_DDR  REG8(0x500C)
#define GPIOC_CR1  REG8(0x500D)
#define GPIOC_CR2  REG8(0x500E)

#define GPIOD_ODR  REG8(0x500F)
#define GPIOD_DDR  REG8(0x5011)
#define GPIOD_CR1  REG8(0x5012)
#define GPIOD_CR2  REG8(0x5013)

#define DRIVER_ENABLE 0x08
#define MOTOR1_A_MASK 0x40
#define MOTOR2_A_MASK 0x80
#define LED_MASK      0x20

/*******************************************************************************
 * @brief   Selects the undivided 16 MHz HSI clock.
 *
 * @details Clears the CPU and peripheral clock prescalers in CLK_CKDIVR.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void clock_init(void)
{
    CLK_CKDIVR = 0x00;
}

/*******************************************************************************
 * @brief   Configures GPIO pins used by the firmware.
 *
 * @details Sets motor control and LED pins as outputs, servo pins as
 *          interrupt inputs, PWM pins as outputs, and sense pins as inputs.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void gpio_init(void)
{
    GPIOC_ODR &= (uint8_t)~(DRIVER_ENABLE | MOTOR1_A_MASK |
                            MOTOR2_A_MASK | LED_MASK);
    GPIOC_DDR |= DRIVER_ENABLE | MOTOR1_A_MASK | MOTOR2_A_MASK | LED_MASK;
    GPIOC_CR1 |= DRIVER_ENABLE | MOTOR1_A_MASK | MOTOR2_A_MASK | LED_MASK;
    GPIOC_CR2 &= (uint8_t)~(DRIVER_ENABLE | MOTOR1_A_MASK |
                            MOTOR2_A_MASK | LED_MASK);

    GPIO_Init(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_MODE_IN_FL_IT);

    GPIOA_ODR &= (uint8_t)~0x08;
    GPIOA_DDR |= 0x08;
    GPIOA_CR1 |= 0x08;
    GPIOA_CR2 |= 0x08;
    GPIOD_ODR &= (uint8_t)~0x10;
    GPIOD_DDR |= 0x10;
    GPIOD_CR1 |= 0x10;
    GPIOD_CR2 |= 0x10;

    GPIOC_DDR &= (uint8_t)~0x10;
    GPIOC_CR1 &= (uint8_t)~0x10;
    GPIOC_CR2 &= (uint8_t)~0x10;
    GPIOD_DDR &= (uint8_t)~0x6C;
    GPIOD_CR1 &= (uint8_t)~0x6C;
    GPIOD_CR2 &= (uint8_t)~0x6C;
}

/*******************************************************************************
 * @brief   Initializes the system clock, GPIO, and TIM1 timebase.
 *
 * @details Configures TIM1 with a 128:1 prescaler for an 8-microsecond
 *          counter tick used to timestamp servo pulses and elapsed time.
 *
 * @author  Varun Sahni
 * @date    2026-11-17
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void Board_Init(void)
{
    clock_init();
    gpio_init();
    TIM1_TimeBaseInit(127, TIM1_COUNTERMODE_UP, 0xFFFF, 0);
    TIM1_Cmd(ENABLE);
}
