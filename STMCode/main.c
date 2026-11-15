/*
 * Two-channel servo tester and motor driver for STM8S003F3.
 * STVD / Cosmic C with the STM8S Standard Peripheral Library.
 *
 * Servo inputs: PA1, PA2 (Port A external interrupt on both edges)
 * Motor 1: PC6 direction, PD4/TIM2_CH1 PWM
 * Motor 2: PC7 direction, PA3/TIM2_CH3 PWM
 * Driver enable: PC3; status LED: PC5
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_exti.h"
#include "stm8s_tim1.h"

#define true  1
#define false 0

#define REG8(address) (*(volatile uint8_t *)(address))

#define CLK_CKDIVR  REG8(0x50C6)
#define GPIOA_ODR   REG8(0x5000)
#define GPIOA_IDR   REG8(0x5001)
#define GPIOA_DDR   REG8(0x5002)
#define GPIOA_CR1   REG8(0x5003)
#define GPIOA_CR2   REG8(0x5004)

#define GPIOC_ODR   REG8(0x500A)
#define GPIOC_DDR   REG8(0x500C)
#define GPIOC_CR1   REG8(0x500D)
#define GPIOC_CR2   REG8(0x500E)

#define GPIOD_ODR   REG8(0x500F)
#define GPIOD_DDR   REG8(0x5011)
#define GPIOD_CR1   REG8(0x5012)
#define GPIOD_CR2   REG8(0x5013)

/* TIM1 is a 16-bit, 8-us pulse-measurement timebase. */
/* TIM2 hardware PWM outputs: CH1 on PD4 and CH3 on PA3. */
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

/* ADC1 registers */
#define ADC1_CSR    REG8(0x5400)
#define ADC1_CR1    REG8(0x5401)
#define ADC1_CR2    REG8(0x5402)
#define ADC1_CR3    REG8(0x5403)
#define ADC1_DRH    REG8(0x5404)
#define ADC1_DRL    REG8(0x5405)

#define SERVO1_MASK       0x02
#define SERVO2_MASK       0x04
#define SERVO_MASK        (SERVO1_MASK | SERVO2_MASK)
#define DRIVER_ENABLE     0x08
#define MOTOR1_A_MASK     0x40
#define MOTOR2_A_MASK     0x80
#define LED_MASK          0x20

#define PULSE_MIN_US      1000U
#define PULSE_MAX_US      2000U
#define NEUTRAL_US        1500U
#define DEADBAND_US       50U
#define VALID_MIN_US      800U
#define VALID_MAX_US      2200U
#define TIMEOUT_TICKS     3750U  /* 30 ms at 8 us/tick */
#define PULSE_STALE_TICKS TIMEOUT_TICKS

#define IS_FAULT_COUNTS   600U
#define ENABLE_IS4_CHECK  1
#define ENABLE_VSENSE_CHECK 0
#define FAULT_SHUTDOWN    0
#define FAULT_CONFIRM     3U
#define FAULT_CHECK_TICKS 2500U  /* 20 ms at 8 us/tick */

#define VCC_MIN_MV        9000UL
#define VCC_MAX_MV        18000UL
#define DIVIDER_NUMERATOR 1887UL

#define BLINK_NORMAL_MS   1000U
#define BLINK_FAULT_MS    100U
#define PWM_PERIOD_COUNTS 1000U

/* These fields are published atomically by the Port A ISR sequence counter. */
static volatile uint8_t capture_seq = 0;
static volatile uint16_t pulse1_us = 0;
static volatile uint16_t pulse2_us = 0;
static volatile uint16_t pulse1_ticks = 0;
static volatile uint16_t pulse2_ticks = 0;
static volatile uint8_t pulse1_ready = 0;
static volatile uint8_t pulse2_ready = 0;

/* Used only by the Port A interrupt handler. */
static uint8_t servo1_was_high = 0;
static uint8_t servo2_was_high = 0;
static uint8_t servo1_capturing = 0;
static uint8_t servo2_capturing = 0;
static uint16_t servo1_rise_ticks = 0;
static uint16_t servo2_rise_ticks = 0;
static uint8_t bad_count = 0;
static uint8_t fault_active = false;
static uint16_t last_blink_ticks = 0;
static uint32_t blink_elapsed_ticks = 0;
static uint16_t last_fault_check_ticks = 0;

@far @interrupt void EXTI_PORTA_IRQHandler(void)
{
    uint8_t pins;
    uint8_t servo1_high;
    uint8_t servo2_high;
    uint16_t now_ticks;
    uint16_t width_ticks;

    capture_seq++;
    pins = GPIOA_IDR;
    servo1_high = (pins & SERVO1_MASK) != 0;
    servo2_high = (pins & SERVO2_MASK) != 0;
    now_ticks = TIM1_GetCounter();

    if (servo1_high != servo1_was_high)
    {
        servo1_was_high = servo1_high;
        if (servo1_high)
        {
            servo1_rise_ticks = now_ticks;
            servo1_capturing = 1;
        }
        else if (servo1_capturing)
        {
            width_ticks = (uint16_t)(now_ticks - servo1_rise_ticks);
            servo1_capturing = 0;
            if (width_ticks <= TIMEOUT_TICKS)
            {
                pulse1_us = (uint16_t)(width_ticks << 3);
                pulse1_ticks = now_ticks;
                pulse1_ready = 1;
            }
        }
    }

    if (servo2_high != servo2_was_high)
    {
        servo2_was_high = servo2_high;
        if (servo2_high)
        {
            servo2_rise_ticks = now_ticks;
            servo2_capturing = 1;
        }
        else if (servo2_capturing)
        {
            width_ticks = (uint16_t)(now_ticks - servo2_rise_ticks);
            servo2_capturing = 0;
            if (width_ticks <= TIMEOUT_TICKS)
            {
                pulse2_us = (uint16_t)(width_ticks << 3);
                pulse2_ticks = now_ticks;
                pulse2_ready = 1;
            }
        }
    }
    capture_seq++;
}

/* Kept because the project's existing vector table names this handler.
 * TIM4 is not started and no timer interrupt is used for pulse capture.
 */
@far @interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
}

static void get_servo_pulses(uint16_t *pulse1, uint16_t *pulse2,
                             uint8_t *valid1, uint8_t *valid2)
{
    uint8_t seq_before;
    uint8_t seq_after;
    uint8_t ready1;
    uint8_t ready2;
    uint16_t captured_pulse1;
    uint16_t captured_pulse2;
    uint16_t captured_ticks1;
    uint16_t captured_ticks2;
    uint16_t now;

    for (;;)
    {
        seq_before = capture_seq;
        if (seq_before & 1)
            continue;
        captured_pulse1 = pulse1_us;
        captured_pulse2 = pulse2_us;
        captured_ticks1 = pulse1_ticks;
        captured_ticks2 = pulse2_ticks;
        ready1 = pulse1_ready;
        ready2 = pulse2_ready;
        seq_after = capture_seq;
        if (seq_before == seq_after && !(seq_after & 1))
            break;
    }

    now = TIM1_GetCounter();
    *valid1 = ready1 && (uint16_t)(now - captured_ticks1) <= PULSE_STALE_TICKS;
    *valid2 = ready2 && (uint16_t)(now - captured_ticks2) <= PULSE_STALE_TICKS;
    *pulse1 = *valid1 ? captured_pulse1 : 0;
    *pulse2 = *valid2 ? captured_pulse2 : 0;
}

static uint8_t decode_servo(uint16_t pulse, int8_t *direction, uint8_t *speed)
{
    uint32_t scaled;

    *direction = 0;
    *speed = 0;

    if (pulse < VALID_MIN_US || pulse > VALID_MAX_US)
        return false;

    if (pulse > NEUTRAL_US + DEADBAND_US)
    {
        scaled = ((uint32_t)(pulse - (NEUTRAL_US + DEADBAND_US)) * 255UL)
               / (PULSE_MAX_US - (NEUTRAL_US + DEADBAND_US));
        if (scaled > 255UL)
            scaled = 255UL;
        *direction = 1;
        *speed = (uint8_t)scaled;
    }
    else if (pulse < NEUTRAL_US - DEADBAND_US)
    {
        scaled = ((uint32_t)((NEUTRAL_US - DEADBAND_US) - pulse) * 255UL)
               / ((NEUTRAL_US - DEADBAND_US) - PULSE_MIN_US);
        if (scaled > 255UL)
            scaled = 255UL;
        *direction = -1;
        *speed = (uint8_t)scaled;
    }

    return true;
}

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

static uint16_t analog_read(uint8_t channel)
{
    uint8_t high;
    uint8_t low;

    ADC1_CSR = channel & 0x0F;
    ADC1_CR1 |= 0x01;
    while ((ADC1_CSR & 0x80) == 0)
    {
    }

    high = ADC1_DRH;
    low = ADC1_DRL;
    return ((uint16_t)(high & 0x03) << 8) | low;
}

static uint8_t check_fault(void)
{
    uint16_t value;

    value = analog_read(2);              /* PC4 / IS1 */
    if (value > IS_FAULT_COUNTS)
        return true;
    value = analog_read(4);              /* PD3 / IS2 */
    if (value > IS_FAULT_COUNTS)
        return true;
    value = analog_read(3);              /* PD2 / IS3 */
    if (value > IS_FAULT_COUNTS)
        return true;

#if ENABLE_IS4_CHECK
    value = analog_read(6);              /* PD6 / IS4 */
    if (value > IS_FAULT_COUNTS)
        return true;
#endif

#if ENABLE_VSENSE_CHECK
    value = analog_read(5);              /* PD5 / voltage sense */
    {
        uint32_t pin_mv;
        uint32_t vcc_mv;

        pin_mv = ((uint32_t)value * 5000UL) / 1023UL;
        vcc_mv = (pin_mv * 10000UL) / DIVIDER_NUMERATOR;
        if (vcc_mv < VCC_MIN_MV || vcc_mv > VCC_MAX_MV)
            return true;
    }
#endif
    return false;
}

static void update_led(void)
{
    uint16_t now_ticks;
    uint16_t delta_ticks;
    uint32_t interval_ticks;

    now_ticks = TIM1_GetCounter();
    delta_ticks = (uint16_t)(now_ticks - last_blink_ticks);
    last_blink_ticks = now_ticks;
    blink_elapsed_ticks += delta_ticks;
    interval_ticks = (uint32_t)(fault_active ? BLINK_FAULT_MS : BLINK_NORMAL_MS)
                   * 125UL;
    if (blink_elapsed_ticks >= interval_ticks)
    {
        blink_elapsed_ticks -= interval_ticks;
        GPIOC_ODR ^= LED_MASK;
    }
}

static void clock_init(void)
{
    CLK_CKDIVR = 0x00;                   /* HSI clock at 16 MHz */
}

static void gpio_init(void)
{
    GPIOC_ODR &= (uint8_t)~(DRIVER_ENABLE | MOTOR1_A_MASK |
                            MOTOR2_A_MASK | LED_MASK);
    GPIOC_DDR |= DRIVER_ENABLE | MOTOR1_A_MASK | MOTOR2_A_MASK | LED_MASK;
    GPIOC_CR1 |= DRIVER_ENABLE | MOTOR1_A_MASK | MOTOR2_A_MASK | LED_MASK;
    GPIOC_CR2 &= (uint8_t)~(DRIVER_ENABLE | MOTOR1_A_MASK |
                            MOTOR2_A_MASK | LED_MASK);

    /* Configure the servo inputs with the STM8S Standard Peripheral Library. */
    GPIO_Init(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_MODE_IN_FL_IT);

    /* PWM alternate-function outputs: PA3/TIM2_CH3 and PD4/TIM2_CH1. */
    GPIOA_ODR &= (uint8_t)~0x08;
    GPIOA_DDR |= 0x08;
    GPIOA_CR1 |= 0x08;
    GPIOA_CR2 |= 0x08;
    GPIOD_ODR &= (uint8_t)~0x10;
    GPIOD_DDR |= 0x10;
    GPIOD_CR1 |= 0x10;
    GPIOD_CR2 |= 0x10;

    /* Analog sense inputs: PC4 and PD2/PD3/PD5/PD6. */
    GPIOC_DDR &= (uint8_t)~0x10;
    GPIOC_CR1 &= (uint8_t)~0x10;
    GPIOC_CR2 &= (uint8_t)~0x10;
    GPIOD_DDR &= (uint8_t)~0x6C;
    GPIOD_CR1 &= (uint8_t)~0x6C;
    GPIOD_CR2 &= (uint8_t)~0x6C;
}

static void exti_init(void)
{
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);

    /* Establish the initial levels so only real subsequent edges are used. */
    servo1_was_high = (GPIOA_IDR & SERVO1_MASK) != 0;
    servo2_was_high = (GPIOA_IDR & SERVO2_MASK) != 0;
}

static void timer_init(void)
{
    /* Free-running TIM1 counter: 16 MHz / 128 = 125 kHz, 8 us/tick.
     * No timer interrupt is enabled; GPIOA interrupts timestamp the edges.
     */
    TIM1_TimeBaseInit(127, TIM1_COUNTERMODE_UP, 0xFFFF, 0);
    TIM1_Cmd(ENABLE);

    /* TIM2: 16 MHz / 1000 = 16 kHz PWM. */
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

static void adc_init(void)
{
    uint16_t delay;

    ADC1_CR1 = 0x10;                     /* ADC clock = 16 MHz / 4 */
    ADC1_CR2 = 0x08;                     /* Right-aligned result */
    ADC1_CR3 = 0x00;
    ADC1_CSR = 0x00;
    ADC1_CR1 |= 0x01;
    for (delay = 0; delay < 2000; delay++)
    {
    }
}

int main(void)
{
    uint16_t pulse1;
    uint16_t pulse2;
    uint16_t now_ticks;
    int8_t direction1;
    int8_t direction2;
    uint8_t speed1;
    uint8_t speed2;
    uint8_t valid1;
    uint8_t valid2;
    uint8_t drivers_on;

    clock_init();
    gpio_init();
    timer_init();
    adc_init();
    exti_init();
    rim();

    for (;;)
    {
        get_servo_pulses(&pulse1, &pulse2, &valid1, &valid2);
        valid1 = decode_servo(pulse1, &direction1, &speed1) && valid1;
        valid2 = decode_servo(pulse2, &direction2, &speed2) && valid2;

        now_ticks = TIM1_GetCounter();
        if ((uint16_t)(now_ticks - last_fault_check_ticks) >= FAULT_CHECK_TICKS)
        {
            last_fault_check_ticks = now_ticks;
            if (check_fault())
            {
                if (bad_count < FAULT_CONFIRM)
                    bad_count++;
            }
            else
            {
                bad_count = 0;
            }
            fault_active = (bad_count >= FAULT_CONFIRM);
        }

        drivers_on = valid1 || valid2;
#if FAULT_SHUTDOWN
        if (fault_active)
        {
            drivers_on = false;
            direction1 = 0;
            speed1 = 0;
            direction2 = 0;
            speed2 = 0;
        }
#endif

        drive_motor(MOTOR1_A_MASK, 1, direction1, speed1);
        drive_motor(MOTOR2_A_MASK, 3, direction2, speed2);
        if (drivers_on)
            GPIOC_ODR |= DRIVER_ENABLE;
        else
            GPIOC_ODR &= (uint8_t)~DRIVER_ENABLE;

        update_led();
    }
}
