#include "stm8s.h"
#include "stm8s_exti.h"
#include "stm8s_tim1.h"
#include "servo_input.h"

#define REG8(address) (*(volatile uint8_t *)(address))
#define GPIOA_IDR REG8(0x5001)

#define SERVO1_MASK 0x02
#define SERVO2_MASK 0x04

#define PULSE_MIN_US      1000U
#define PULSE_MAX_US      2000U
#define NEUTRAL_US        1500U
#define DEADBAND_US       50U
#define VALID_MIN_US      800U
#define VALID_MAX_US      2200U
#define TIMEOUT_TICKS     3750U
#define PULSE_STALE_TICKS TIMEOUT_TICKS

static volatile uint8_t capture_seq = 0;
static volatile uint16_t pulse1_us = 0;
static volatile uint16_t pulse2_us = 0;
static volatile uint16_t pulse1_ticks = 0;
static volatile uint16_t pulse2_ticks = 0;
static volatile uint8_t pulse1_ready = 0;
static volatile uint8_t pulse2_ready = 0;

static uint8_t servo1_was_high = 0;
static uint8_t servo2_was_high = 0;
static uint8_t servo1_capturing = 0;
static uint8_t servo2_capturing = 0;
static uint16_t servo1_rise_ticks = 0;
static uint16_t servo2_rise_ticks = 0;

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

@far @interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
}

static void get_pulses(uint16_t *pulse1, uint16_t *pulse2,
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

static uint8_t decode_pulse(uint16_t pulse, MotorCommand *command)
{
    uint32_t scaled;

    command->direction = 0;
    command->speed = 0;

    if (pulse < VALID_MIN_US || pulse > VALID_MAX_US)
        return 0;

    if (pulse > NEUTRAL_US + DEADBAND_US)
    {
        scaled = ((uint32_t)(pulse - (NEUTRAL_US + DEADBAND_US)) * 255UL)
               / (PULSE_MAX_US - (NEUTRAL_US + DEADBAND_US));
        if (scaled > 255UL)
            scaled = 255UL;
        command->direction = 1;
        command->speed = (uint8_t)scaled;
    }
    else if (pulse < NEUTRAL_US - DEADBAND_US)
    {
        scaled = ((uint32_t)((NEUTRAL_US - DEADBAND_US) - pulse) * 255UL)
               / ((NEUTRAL_US - DEADBAND_US) - PULSE_MIN_US);
        if (scaled > 255UL)
            scaled = 255UL;
        command->direction = -1;
        command->speed = (uint8_t)scaled;
    }

    return 1;
}

void ServoInput_Init(void)
{
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);
    servo1_was_high = (GPIOA_IDR & SERVO1_MASK) != 0;
    servo2_was_high = (GPIOA_IDR & SERVO2_MASK) != 0;
}

void ServoInput_Read(MotorCommand *command1, MotorCommand *command2)
{
    uint16_t pulse1;
    uint16_t pulse2;
    uint8_t valid1;
    uint8_t valid2;

    get_pulses(&pulse1, &pulse2, &valid1, &valid2);
    command1->valid = decode_pulse(pulse1, command1) && valid1;
    command2->valid = decode_pulse(pulse2, command2) && valid2;
}
