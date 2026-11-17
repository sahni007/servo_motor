#include "stm8s.h"
#include "stm8s_tim1.h"
#include "status_led.h"

#define REG8(address) (*(volatile uint8_t *)(address))
#define GPIOC_ODR REG8(0x500A)
#define LED_MASK  0x20

#define BLINK_NORMAL_MS 1000U
#define BLINK_FAULT_MS  100U

static uint16_t last_blink_ticks = 0;
static uint32_t blink_elapsed_ticks = 0;

void StatusLed_Update(uint8_t fault_active)
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
