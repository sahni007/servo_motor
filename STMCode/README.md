# STM8 Servo Motor Controller

## Overview

This firmware runs on an STM8S003F3 using STVD, the Cosmic C compiler, and the
STM8S Standard Peripheral Library. It reads two RC servo-style PWM inputs,
converts each pulse width to a direction and speed command, and controls two
motors. It also checks the configured analog fault-sense inputs and indicates
status with an LED.

## Hardware connections

| Signal | MCU pin | Function |
|---|---|---|
| Servo input 1 | PA1 | Port A GPIO interrupt input |
| Servo input 2 | PA2 | Port A GPIO interrupt input |
| Motor 1 direction | PC6 | Direction control |
| Motor 1 PWM | PD4 / TIM2_CH1 | PWM output |
| Motor 2 direction | PC7 | Direction control |
| Motor 2 PWM | PA3 / TIM2_CH3 | PWM output |
| Driver enable (INH) | PC3 | Enables/disables both motor drivers |
| Status LED | PC5 | Normal/fault blink indication |
| Current sense 1 | PC4 / ADC channel 2 | Analog fault input |
| Current sense 2 | PD3 / ADC channel 4 | Analog fault input |
| Current sense 3 | PD2 / ADC channel 3 | Analog fault input |
| Current sense 4 | PD6 / ADC channel 6 | Optional analog fault input |
| Voltage sense | PD5 / ADC channel 5 | Optional analog fault input |

Confirm these pin assignments against the target board schematic before
powering the motors.

## How the code works

### 1. Initialization

`main()` initializes the HSI clock, GPIO pins, TIM1/TIM2, and ADC1, then sets
Port A to trigger an interrupt on both rising and falling edges. Finally, it
enables CPU interrupts and enters the control loop.

TIM1 runs freely at 125 kHz (one count every 8 microseconds). It does not use a
timer interrupt. TIM2 generates motor PWM on channels 1 and 3.

### 2. Servo pulse measurement

Both servo inputs share the Port A external interrupt. The
`EXTI_PORTA_IRQHandler()` interrupt handler reads the state of PA1 and PA2 and
compares it with the previously observed state:

1. On a rising edge, it saves the current TIM1 count for that input.
2. On the falling edge, it subtracts the saved count from the current count.
3. The elapsed count is converted to microseconds by multiplying by 8.
4. If the measured pulse is no longer than the 30 ms capture limit, its width
   and timestamp are published for the main loop.

The timer is only the time reference between GPIO edges; the GPIO interrupt is
what detects and records those edges.

### 3. Reading and decoding input values

`get_servo_pulses()` copies a consistent snapshot of both measurements. The
`capture_seq` counter prevents the main loop from seeing data while the
interrupt is updating it. A measurement is considered recent for up to 30 ms;
an input with no recent pulse is returned as invalid with a width of zero.

`decode_servo()` rejects widths outside 800–2200 microseconds. Widths above
1550 microseconds command forward motion, widths below 1450 microseconds
command reverse motion, and widths in the 100-microsecond neutral deadband
command a stop. Speed is scaled to 0–255.

### 4. Driving motors

For each channel, `drive_motor()` sets the direction GPIO and calls `set_pwm()`
to update the matching TIM2 compare register. The driver enable pin is high
while either servo input is valid and low when neither input is valid.

### 5. Fault checks and LED

Every 20 ms, the main loop calls `check_fault()`, which reads the enabled ADC
current-sense channels and optionally the voltage-sense input. A fault becomes
active after three consecutive fault checks. With `FAULT_SHUTDOWN` set to `1`,
an active fault also stops both motors and disables the drivers. It is `0` by
default.

`update_led()` uses elapsed TIM1 counts to toggle the status LED without
requiring a timer interrupt. The configured intervals are 1000 ms for normal
status and 100 ms while a fault is active.

## Build setup

Configure STVD to use the Cosmic STM8 compiler and add the STM8S Standard
Peripheral Library `inc` directory to the compiler include paths. The source
uses these library headers:

- `stm8s.h`
- `stm8s_gpio.h`
- `stm8s_exti.h`
- `stm8s_tim1.h`

Add the corresponding library source files to the project:

- `stm8s_gpio.c`
- `stm8s_exti.c`
- `stm8s_tim1.c`

The project also needs the library's device and compiler configuration to match
the STM8S003F3 and Cosmic toolchain.

## Interrupt vector configuration

The existing `stm8_interrupt_vector.c` maps Port A external interrupt IRQ3 to
`EXTI_PORTA_IRQHandler()`. Keep this mapping enabled. The existing TIM4 vector
may remain in the vector table, but TIM4 is not initialized or used by this
firmware.

## Configuration and safety notes

- `ENABLE_IS4_CHECK` enables the fourth current-sense channel.
- `ENABLE_VSENSE_CHECK` is disabled by default; enable it only after confirming
  that the voltage divider is safe for the MCU input.
- `IS_FAULT_COUNTS` must be calibrated for the board's current-sense circuit.
- `FAULT_SHUTDOWN` is disabled by default to avoid uncalibrated readings
  unexpectedly stopping the motors.
- Test signal polarity, timer frequency, PWM polarity, analog channel mapping,
  driver enable behavior, and fault thresholds with the motor power stage
  safely disabled before normal operation.
