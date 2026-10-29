/*
  Servo-tester inputs -> two motors, with fault detection and status LED
  STM8S003F3U6 20A DC Controller (sduino / SDCC)
  No serial output (the board has no free serial port).

  Motor 1 (H3/H4): A = PC6 (U2), B = PD4 (U3)
  Motor 2 (H5/H6): A = PC7 (U4), B = PA3 (U5)

  MIX_MODE 0 (normal): Input 1 (J3, PA1) -> Motor 1, Input 2 (J4, PA2) -> Motor 2
  MIX_MODE 1 (mix):    Input 1 = throttle, Input 2 = steering (optional)
                         left  = throttle + steering
                         right = throttle - steering
                       With steering centred or not connected, the single
                       throttle drives both motors together.

  Each input is a servo pulse (1000-2000 us, ~50 Hz):
    above 1550 us = forward, below 1450 us = reverse, between = stop.
  No valid pulse stops the motor(s). In mix mode, no valid throttle stops both.
  INH (PC3) is switched off when no valid command is present.

  LED D2 (PC5): slow blink = OK, fast blink = fault
*/

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

// ---- Mode selection ----
// 0 = normal (two independent inputs), 1 = mix mode
#define MIX_MODE          1
// In mix mode: 1 = read input 2 as steering, 0 = throttle only (input 2 is ignored
// and never read, which keeps the loop fast when nothing is connected to J4)
#define MIX_USE_STEERING  1
// Flip if the robot turns the wrong way when steering right
#define MIX_STEER_INVERT  0

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

#define ENABLE_IS4_CHECK     1

// Leave at 0 until R16 and R18 are swapped on the board.
#define ENABLE_VSENSE_CHECK  0

// 1 = switch the drivers off while a fault is active.
#define FAULT_SHUTDOWN       0

// ---- Fault thresholds ----
#define IS_FAULT_COUNTS   600
#define VCC_MIN_V         9.0f
#define VCC_MAX_V         18.0f
#define DIVIDER_RATIO     0.1887f
#define FAULT_CONFIRM     3

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

// Turns a pulse width into a signed command, -255 (full reverse) to +255 (full forward).
// Returns true if the pulse is valid. Invalid pulses give 0.
bool decodeServoSigned(unsigned long p, int16_t *val)
{
    unsigned long s;

    *val = 0;

    if (p < VALID_MIN_US || p > VALID_MAX_US)
        return false;

    if (p > NEUTRAL_US + DEADBAND_US)
    {
        s = ((p - (NEUTRAL_US + DEADBAND_US)) * 255UL)
            / (PULSE_MAX_US - (NEUTRAL_US + DEADBAND_US));
        if (s > 255UL) s = 255UL;
        *val = (int16_t)s;
    }
    else if (p < NEUTRAL_US - DEADBAND_US)
    {
        s = (((NEUTRAL_US - DEADBAND_US) - p) * 255UL)
            / ((NEUTRAL_US - DEADBAND_US) - PULSE_MIN_US);
        if (s > 255UL) s = 255UL;
        *val = -(int16_t)s;
    }

    return true;
}

// Splits a signed command (-255..255) into direction and speed.
void splitSigned(int16_t v, int8_t *dir, uint8_t *speed)
{
    if (v > 255)  v = 255;
    if (v < -255) v = -255;

    if (v > 0)      { *dir = 1;  *speed = (uint8_t)v; }
    else if (v < 0) { *dir = -1; *speed = (uint8_t)(-v); }
    else            { *dir = 0;  *speed = 0; }
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

    digitalWrite(INH_PIN, LOW);
    driveMotor(M1_A, M1_B, 0, 0);
    driveMotor(M2_A, M2_B, 0, 0);
}
#if 0
void loop(void)
{
    bool valid1, valid2, driversOn;
    int16_t v1, v2;

    // ---- Read servo inputs ----
    pulse1 = readServoPulse(SERVO_IN_1);
    valid1 = decodeServoSigned(pulse1, &v1);

#if MIX_MODE
    {
        int16_t throttle = v1;
        int16_t steer = 0;
        int16_t left, right;

  #if MIX_USE_STEERING
        pulse2 = readServoPulse(SERVO_IN_2);
        decodeServoSigned(pulse2, &steer);   // missing/invalid steering = 0 (straight)
    #if MIX_STEER_INVERT
        steer = -steer;
    #endif
  #endif

        // Without a valid throttle, both motors stop (failsafe).
        if (!valid1)
        {
            throttle = 0;
            steer = 0;
        }

        left  = throttle + steer;
        right = throttle - steer;

        // Clamp to the output range
        if (left > 255)   left = 255;
        if (left < -255)  left = -255;
        if (right > 255)  right = 255;
        if (right < -255) right = -255;

        splitSigned(left,  &dir1, &speed1);   // Motor 1 = left
        splitSigned(right, &dir2, &speed2);   // Motor 2 = right

        driversOn = valid1;
    }
#else
    pulse2 = readServoPulse(SERVO_IN_2);
    valid2 = decodeServoSigned(pulse2, &v2);

    splitSigned(v1, &dir1, &speed1);
    splitSigned(v2, &dir2, &speed2);

    driversOn = (valid1 || valid2);
#endif

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

#if FAULT_SHUTDOWN
    if (faultActive)
    {
        driversOn = false;
        dir1 = 0; speed1 = 0;
        dir2 = 0; speed2 = 0;
    }
#endif

    // ---- Apply to both motors ----
    driveMotor(M1_A, M1_B, dir1, speed1);
    driveMotor(M2_A, M2_B, dir2, speed2);
    digitalWrite(INH_PIN, driversOn ? HIGH : LOW);

    updateLed();
}

#else
void loop(void)
{
    bool valid1, valid2, driversOn;
    int16_t throttle = 0;
    int16_t steer = 0;
    int16_t left, right;

    // -------- Read Channel 1: THROTTLE (PA1) --------
    pulse1 = readServoPulse(SERVO_IN_1);
    valid1 = decodeServoSigned(pulse1, &throttle);

    // -------- Read Channel 2: STEERING (PA2) --------
    pulse2 = readServoPulse(SERVO_IN_2);
    valid2 = decodeServoSigned(pulse2, &steer);

    // Optional steering reversal
#if MIX_STEER_INVERT
    steer = -steer;
#endif

    // -------- Fail-safe --------
    // Invalid throttle: stop both motors.
    if (!valid1)
    {
        throttle = 0;
        steer = 0;
    }

    // Invalid steering: continue straight using throttle only.
    if (!valid2)
    {
        steer = 0;
    }

    // -------- Mix throttle and steering --------
    // Motor 1 = left motor
    // Motor 2 = right motor
    left  = throttle + steer;
    right = throttle - steer;

    // Limit commands to -255 ... +255
    if (left > 255)   left = 255;
    if (left < -255)  left = -255;

    if (right > 255)  right = 255;
    if (right < -255) right = -255;

    // Convert commands to direction and speed
    splitSigned(left,  &dir1, &speed1);
    splitSigned(right, &dir2, &speed2);

    // -------- Fault detection --------
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

#if FAULT_SHUTDOWN
    if (faultActive)
    {
        dir1 = 0;
        speed1 = 0;
        dir2 = 0;
        speed2 = 0;
    }
#endif

    // -------- Drive both motors --------
    driveMotor(M1_A, M1_B, dir1, speed1);
    driveMotor(M2_A, M2_B, dir2, speed2);

    // Enable drivers only when throttle is valid
    driversOn = valid1;

#if FAULT_SHUTDOWN
    if (faultActive)
        driversOn = false;
#endif

    digitalWrite(INH_PIN, driversOn ? HIGH : LOW);

    updateLed();
}
#endif
