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

### Modules

- `main.c` initializes the modules and runs the control loop.
- `board.c` configures the clock, GPIO pins, and TIM1 pulse-measurement timer.
- `servo_input.c` captures and decodes servo pulses; its interrupt handler is
  connected through `stm8_interrupt_vector.c`.
- `motor_control.c` configures TIM2 PWM and drives both motors.
- `fault_monitor.c` reads the enabled analog inputs and tracks confirmed faults.
- `status_led.c` updates the normal/fault LED blink pattern.

`motor_command.h` defines the command passed from servo decoding to motor
control. `app_config.h` contains the application-wide `FAULT_SHUTDOWN` option.
TIM1 runs at 125 kHz (8-microsecond ticks) for pulse capture and elapsed time;
TIM2 generates 16 kHz motor PWM. The control behavior and pin assignments are
unchanged.

### Servo pulse measurement

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

### Reading and decoding input values

`ServoInput_Read()` copies a consistent snapshot of both measurements. The
`capture_seq` counter prevents the main loop from seeing data while the
interrupt is updating it. A measurement is considered recent for up to 30 ms;
an input with no recent pulse is returned as invalid.

Pulse widths outside 800–2200 microseconds are rejected. Widths above
1550 microseconds command forward motion, widths below 1450 microseconds
command reverse motion, and widths in the 100-microsecond neutral deadband
command a stop. Speed is scaled to 0–255.

### Driving motors

`motor_control.c` sets motor direction and PWM. The driver enable pin is high
while either servo input is valid and low when neither input is valid.

### Fault checks and LED

`fault_monitor.c` checks enabled ADC channels every 20 ms. A fault becomes
active after three consecutive fault checks. With `FAULT_SHUTDOWN` set to `1`
in `app_config.h`, an active fault also stops both motors and disables the
drivers; it is `0` by default.

`status_led.c` uses elapsed TIM1 counts to toggle the LED without a timer
interrupt (1000 ms normal, 100 ms fault).

## Build setup

Configure STVD to use the Cosmic STM8 compiler and add the STM8S Standard
Peripheral Library `inc` directory to the compiler include paths. Add these
application source files to the STVD project (the project file is not included
in this repository):

- `main.c`
- `board.c`
- `servo_input.c`
- `motor_control.c`
- `fault_monitor.c`
- `status_led.c`
- `stm8_interrupt_vector.c`

The application headers should be in the project directory or on the compiler
include path:

- `app_config.h`
- `board.h`
- `fault_monitor.h`
- `motor_command.h`
- `motor_control.h`
- `servo_input.h`
- `status_led.h`

The application uses these library headers:

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

The existing `stm8_interrupt_vector.c` maps Port A external interrupt IRQ3 to the handler in
`servo_input.c`. Keep this mapping enabled. The TIM4 vector maps to an empty
handler in `servo_input.c`; TIM4 is not initialized or used.

## Configuration and safety notes

- `ENABLE_IS4_CHECK`, `ENABLE_VSENSE_CHECK`, `IS_FAULT_COUNTS`, and voltage
  limits are configured in `fault_monitor.c`. Enable voltage sensing only
  after confirming that the voltage divider is safe for the MCU input.
- `FAULT_SHUTDOWN` is disabled by default to avoid uncalibrated readings
  unexpectedly stopping the motors.
- Test signal polarity, timer frequency, PWM polarity, analog channel mapping,
  driver enable behavior, and fault thresholds with the motor power stage
  safely disabled before normal operation.
