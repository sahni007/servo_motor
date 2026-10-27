/*
  Two servo-tester inputs -> two motors, with fault detection and status LED
  STM8S003F3U6 20A DC Controller (sduino / SDCC)
  No serial output (the board has no free serial port).

  Input 1 (J3, PA1) -> Motor 1 (H3/H4): A = PC6 (U2), B = PD4 (U3)
  Input 2 (J4, PA2) -> Motor 2 (H5/H6): A = PC7 (U4), B = PA3 (U5)

  Each input is a servo pulse (1000-2000 us, ~50 Hz):
    above 1550 us = forward, below 1450 us = reverse, between = stop.
  An input with no valid pulse stops its own motor.
  INH (PC3) is switched off only when BOTH inputs have no valid pulse.

  PWM is on the B pins (PD4, PA3), the default timer-2 PWM pins.

  LED D2 (PC5):
    slow blink (1000 ms on / 1000 ms off) = OK
    fast blink (100 ms on / 100 ms off) = fault
*/

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

// ---- Inputs ----
#define SERVO_IN_1   PA1
#define SERVO_IN_2   PA2

// ---- Motor pins ----
#define INH_PIN      PC3
#define M1_A         PC6
#define M1_B         PD4
#define M2_A         PC7
#define M2_B         PA3
#define LED_PIN      PC5

// ---- Analog sense pins ----
#define IS1_PIN      PC4   // U2
#define IS2_PIN      PD3   // U3
#define IS3_PIN      PD2   // U4
#define IS4_PIN      PD6   // U5
#define VSENSE_PIN   PD5   // Voltage_Sense

// Serial is not used, so PD6 is free and the IS4 check can run.
#define ENABLE_IS4_CHECK     1

// Voltage check: leave at 0 until R16 and R18 are swapped on the board.
// As drawn, the divider puts about 81% of VCC on PD5, which can damage the pin.
#define ENABLE_VSENSE_CHECK  0

// 1 = switch the drivers off while a fault is active.
// Leave at 0 until IS_FAULT_COUNTS is calibrated, or false faults will stop the motors.
#define FAULT_SHUTDOWN       0

// ---- Fault thresholds ----
#define IS_FAULT_COUNTS   600
#define VCC_MIN_V         9.0f
#define VCC_MAX_V         18.0f
#define DIVIDER_RATIO     0.1887f   // 43K top, 10K bottom (after the swap)
#define FAULT_CONFIRM     3         // consecutive bad loops before a fault is declared

// ---- Servo pulse settings (microseconds) ----
#define PULSE_MIN_US     1000UL
#define PULSE_MAX_US     2000UL
#define NEUTRAL_US       1500UL
#define DEADBAND_US      50UL
#define VALID_MIN_US     800UL
#define VALID_MAX_US     2200UL
#define TIMEOUT_US       30000UL

// ---- LED timing ----
#define BLINK_NORMAL_MS  1000
#define BLINK_FAULT_MS   100

uint8_t  badCount = 0;
bool     faultActive = false;
unsigned long lastBlink = 0;

unsigned long pulse1 = 0, pulse2 = 0;
int8_t   dir1 = 0, dir2 = 0;          // +1 forward, -1 reverse, 0 stop
uint8_t  speed1 = 0, speed2 = 0;      // 0-255

// Returns the HIGH pulse width in microseconds, or 0 if no pulse was seen.
unsigned long readServoPulse(uint8_t pin)
{
    unsigned long t, rise;

    t = micros();
    while (digitalRead(pin))
        if (micros() - t > TIMEOUT_US) return 0;

    t = micros();
    while (!digitalRead(pin))
        if (micros() - t > TIMEOUT_US) return 0;

    rise = micros();
    while (digitalRead(pin))
        if (micros() - rise > TIMEOUT_US) return 0;

    return micros() - rise;
}

// Turns a pulse width into direction and speed. Returns true if the pulse is valid.
bool decodeServo(unsigned long p, int8_t *dir, uint8_t *speed)
{
    unsigned long s;

    *dir = 0;
    *speed = 0;

    if (p < VALID_MIN_US || p > VALID_MAX_US)
        return false;

    if (p > NEUTRAL_US + DEADBAND_US)
    {
        s = ((p - (NEUTRAL_US + DEADBAND_US)) * 255UL)
            / (PULSE_MAX_US - (NEUTRAL_US + DEADBAND_US));
        if (s > 255UL) s = 255UL;
        *dir = 1;
        *speed = (uint8_t)s;
    }
    else if (p < NEUTRAL_US - DEADBAND_US)
    {
        s = (((NEUTRAL_US - DEADBAND_US) - p) * 255UL)
            / ((NEUTRAL_US - DEADBAND_US) - PULSE_MIN_US);
        if (s > 255UL) s = 255UL;
        *dir = -1;
        *speed = (uint8_t)s;
    }

    return true;
}

// Drives one motor. dir: +1 forward, -1 reverse, 0 stop (brake). speed: 0-255.
void driveMotor(uint8_t pinA, uint8_t pinB, int8_t dir, uint8_t speed)
{
    if (dir > 0)
    {
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 - speed);
    }
    else if (dir < 0)
    {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, speed);
    }
    else
    {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, 0);
    }
}

// Returns true if any enabled check is out of range.
bool checkFault(void)
{
    if (analogRead(IS1_PIN) > IS_FAULT_COUNTS) return true;
    if (analogRead(IS2_PIN) > IS_FAULT_COUNTS) return true;
    if (analogRead(IS3_PIN) > IS_FAULT_COUNTS) return true;

#if ENABLE_IS4_CHECK
    if (analogRead(IS4_PIN) > IS_FAULT_COUNTS) return true;
#endif

#if ENABLE_VSENSE_CHECK
    {
        float vcc = (analogRead(VSENSE_PIN) * 5.0f / 1023.0f) / DIVIDER_RATIO;
        if (vcc < VCC_MIN_V || vcc > VCC_MAX_V) return true;
    }
#endif

    return false;
}

// Toggles the LED every BLINK_NORMAL_MS, or every BLINK_FAULT_MS during a fault.
void updateLed(void)
{
    unsigned long interval = faultActive ? BLINK_FAULT_MS : BLINK_NORMAL_MS;

    if (millis() - lastBlink >= interval)
    {
        lastBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void setup(void)
{
    pinMode(SERVO_IN_1, INPUT);
    pinMode(SERVO_IN_2, INPUT);

    pinMode(INH_PIN, OUTPUT);
    pinMode(M1_A, OUTPUT);
    pinMode(M1_B, OUTPUT);
    pinMode(M2_A, OUTPUT);
    pinMode(M2_B, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(INH_PIN, LOW);      // drivers off while pins are set up
    driveMotor(M1_A, M1_B, 0, 0);
    driveMotor(M2_A, M2_B, 0, 0);
}

void loop(void)
{
    bool valid1, valid2, driversOn;

    // ---- Read both servo inputs ----
    pulse1 = readServoPulse(SERVO_IN_1);
    pulse2 = readServoPulse(SERVO_IN_2);

    valid1 = decodeServo(pulse1, &dir1, &speed1);
    valid2 = decodeServo(pulse2, &dir2, &speed2);

    // ---- Fault detection with debounce ----
    if (checkFault())
    {
        if (badCount < FAULT_CONFIRM)
            badCount++;
    }
    else
    {
        badCount = 0;
    }

    faultActive = (badCount >= FAULT_CONFIRM);

    // ---- Apply to both motors ----
    driversOn = (valid1 || valid2);

#if FAULT_SHUTDOWN
    if (faultActive)
    {
        driversOn = false;
        dir1 = 0; speed1 = 0;
        dir2 = 0; speed2 = 0;
    }
#endif

    driveMotor(M1_A, M1_B, dir1, speed1);
    driveMotor(M2_A, M2_B, dir2, speed2);
    digitalWrite(INH_PIN, driversOn ? HIGH : LOW);

    // ---- Status LED ----
    updateLed();
}
