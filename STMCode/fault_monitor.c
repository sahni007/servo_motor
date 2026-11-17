#include "stm8s.h"
#include "stm8s_tim1.h"
#include "fault_monitor.h"

#define REG8(address) (*(volatile uint8_t *)(address))

#define ADC1_CSR REG8(0x5400)
#define ADC1_CR1 REG8(0x5401)
#define ADC1_CR2 REG8(0x5402)
#define ADC1_CR3 REG8(0x5403)
#define ADC1_DRH REG8(0x5404)
#define ADC1_DRL REG8(0x5405)

#define IS_FAULT_COUNTS     600U
#define ENABLE_IS4_CHECK    1
#define ENABLE_VSENSE_CHECK 0
#define FAULT_CONFIRM       3U
#define FAULT_CHECK_TICKS   2500U

#define VCC_MIN_MV        9000UL
#define VCC_MAX_MV        18000UL
#define DIVIDER_NUMERATOR 1887UL

static uint8_t bad_count = 0;
static uint8_t fault_active = 0;
static uint16_t last_fault_check_ticks = 0;

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

    value = analog_read(2);
    if (value > IS_FAULT_COUNTS)
        return 1;
    value = analog_read(4);
    if (value > IS_FAULT_COUNTS)
        return 1;
    value = analog_read(3);
    if (value > IS_FAULT_COUNTS)
        return 1;

#if ENABLE_IS4_CHECK
    value = analog_read(6);
    if (value > IS_FAULT_COUNTS)
        return 1;
#endif

#if ENABLE_VSENSE_CHECK
    value = analog_read(5);
    {
        uint32_t pin_mv;
        uint32_t vcc_mv;

        pin_mv = ((uint32_t)value * 5000UL) / 1023UL;
        vcc_mv = (pin_mv * 10000UL) / DIVIDER_NUMERATOR;
        if (vcc_mv < VCC_MIN_MV || vcc_mv > VCC_MAX_MV)
            return 1;
    }
#endif
    return 0;
}

void FaultMonitor_Init(void)
{
    uint16_t delay;

    ADC1_CR1 = 0x10;
    ADC1_CR2 = 0x08;
    ADC1_CR3 = 0x00;
    ADC1_CSR = 0x00;
    ADC1_CR1 |= 0x01;
    for (delay = 0; delay < 2000; delay++)
    {
    }
}

uint8_t FaultMonitor_Update(void)
{
    uint16_t now_ticks;

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
    return fault_active;
}
