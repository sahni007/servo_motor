
/*
  STM8S003F3U6 - Dual Motor Controller
  Non-blocking RC PWM input capture
  Compatible approach for Sduino / SDCC
  No serial output

  Inputs:
    PA1 = Throttle
    PA2 = Steering

  Motors:
    Left:  PC6 / PD4
    Right: PC7 / PA3
    INH:   PC3
    LED:   PC5

  Mixing:
    left  = throttle + steer
    right = throttle - steer
*/

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// -------- Mode selection --------
#define MIX_MODE             1
#define MIX_USE_STEERING     1
#define MIX_STEER_INVERT     0

// -------- Inputs --------
#define SERVO_IN_1           PA1
#define SERVO_IN_2           PA2

// -------- Motor pins --------
#define INH_PIN              PC3
#define M1_A                 PC6
#define M1_B                 PD4
#define M2_A                 PC7
#define M2_B                 PA3
#define LED_PIN              PC5

// -------- Analog sense pins --------
#define IS1_PIN              PC4
#define IS2_PIN              PD3
#define IS3_PIN              PD2
#define IS4_PIN              PD6
#define VSENSE_PIN           PD5

#define ENABLE_IS4_CHECK     1
#define ENABLE_VSENSE_CHECK  0
#define FAULT_SHUTDOWN       0

// -------- Fault thresholds --------
#define IS_FAULT_COUNTS      600
#define VCC_MIN_V            9.0f
#define VCC_MAX_V            18.0f
#define DIVIDER_RATIO        0.1887f
#define FAULT_CONFIRM        3

// -------- Servo pulse settings --------
#define PULSE_MIN_US         1000UL
#define PULSE_MAX_US         2000UL
#define NEUTRAL_US           1500UL
#define DEADBAND_US          50UL
#define VALID_MIN_US         800UL
#define VALID_MAX_US         2200UL
#define TIMEOUT_US           30000UL

// -------- LED timing --------
#define BLINK_NORMAL_MS      1000UL
#define BLINK_FAULT_MS       100UL

// -------- Input state: throttle --------
uint8_t throttlePreviousLevel = LOW;
unsigned long throttleRiseTime = 0;
unsigned long throttlePulseWidth = 0;
unsigned long throttleLastUpdate = 0;
bool throttleHavePulse = false;

// -------- Input state: steering --------
uint8_t steeringPreviousLevel = LOW;
unsigned long steeringRiseTime = 0;
unsigned long steeringPulseWidth = 0;
unsigned long steeringLastUpdate = 0;
bool steeringHavePulse = false;

// -------- Runtime state --------
uint8_t badCount = 0;
bool faultActive = false;
unsigned long lastBlink = 0;

int8_t dir1 = 0;
int8_t dir2 = 0;
uint8_t speed1 = 0;
uint8_t speed2 = 0;

// -------- Non-blocking input capture --------
void updateServoInput(
    uint8_t pin,
    uint8_t *previousLevel,
    unsigned long *riseTime,
    unsigned long *pulseWidth,
    unsigned long *lastUpdate,
    bool *havePulse,
    unsigned long now)
{
    uint8_t level = digitalRead(pin);

    if ((level == HIGH) && (*previousLevel == LOW)) {
        // Rising edge: start of pulse
        *riseTime = now;
    }
    else if ((level == LOW) && (*previousLevel == HIGH)) {
        // Falling edge: end of pulse
        unsigned long width = now - *riseTime;

        if ((width >= VALID_MIN_US) && (width <= VALID_MAX_US)) {
            *pulseWidth = width;
            *lastUpdate = now;
            *havePulse = true;
        }
    }

    *previousLevel = level;
}

bool getServoPulse(
    bool havePulse,
    unsigned long lastUpdate,
    unsigned long pulseWidth,
    unsigned long now,
    unsigned long *pulse)
{
    if (!havePulse) {
        return false;
    }

    if ((unsigned long)(now - lastUpdate) >= TIMEOUT_US) {
        return false;
    }

    *pulse = pulseWidth;
    return true;
}

// -------- Pulse decoding --------
bool decodeServoSigned(unsigned long p, int16_t *val)
{
    unsigned long s;

    *val = 0;

    if ((p < VALID_MIN_US) || (p > VALID_MAX_US)) {
        return false;
    }

    if (p > (NEUTRAL_US + DEADBAND_US)) {
        s = ((p - (NEUTRAL_US + DEADBAND_US)) * 255UL) /
            (PULSE_MAX_US - (NEUTRAL_US + DEADBAND_US));

        if (s > 255UL) {
            s = 255UL;
        }

        *val = (int16_t)s;
    }
    else if (p < (NEUTRAL_US - DEADBAND_US)) {
        s = (((NEUTRAL_US - DEADBAND_US) - p) * 255UL) /
            ((NEUTRAL_US - DEADBAND_US) - PULSE_MIN_US);

        if (s > 255UL) {
            s = 255UL;
        }

        *val = -(int16_t)s;
    }

    return true;
}

// -------- Signed speed to direction/speed --------
void splitSigned(int16_t v, int8_t *dir, uint8_t *speed)
{
    if (v > 255) {
        v = 255;
    }

    if (v < -255) {
        v = -255;
    }

    if (v > 0) {
        *dir = 1;
        *speed = (uint8_t)v;
    }
    else if (v < 0) {
        *dir = -1;
        *speed = (uint8_t)(-v);
    }
    else {
        *dir = 0;
        *speed = 0;
    }
}

// -------- Motor control --------
void driveMotor(
    uint8_t pinA,
    uint8_t pinB,
    int8_t dir,
    uint8_t speed)
{
    if (dir > 0) {
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 - speed);
    }
    else if (dir < 0) {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, speed);
    }
    else {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, 0);
    }
}

// -------- Fault checking --------
bool checkFault(void)
{
    if (analogRead(IS1_PIN) > IS_FAULT_COUNTS) {
        return true;
    }

    if (analogRead(IS2_PIN) > IS_FAULT_COUNTS) {
        return true;
    }

    if (analogRead(IS3_PIN) > IS_FAULT_COUNTS) {
        return true;
    }

#if ENABLE_IS4_CHECK
    if (analogRead(IS4_PIN) > IS_FAULT_COUNTS) {
        return true;
    }
#endif

#if ENABLE_VSENSE_CHECK
    {
        float vcc =
            (analogRead(VSENSE_PIN) * 5.0f / 1023.0f)
            / DIVIDER_RATIO;

        if ((vcc < VCC_MIN_V) || (vcc > VCC_MAX_V)) {
            return true;
        }
    }
#endif

    return false;
}

// -------- LED status --------
void updateLed(void)
{
    unsigned long interval;

    if (faultActive) {
        interval = BLINK_FAULT_MS;
    }
    else {
        interval = BLINK_NORMAL_MS;
    }

    if ((unsigned long)(millis() - lastBlink) >= interval) {
        lastBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

// -------- Setup --------
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
    digitalWrite(LED_PIN, LOW);

    driveMotor(M1_A, M1_B, 0, 0);
    driveMotor(M2_A, M2_B, 0, 0);

    // Initialize current input levels.
    throttlePreviousLevel = digitalRead(SERVO_IN_1);
    steeringPreviousLevel = digitalRead(SERVO_IN_2);
}

// -------- Main loop --------
void loop(void)
{
    bool valid1;
    bool valid2;
    bool driversOn;

    int16_t throttle = 0;
    int16_t steer = 0;
    int16_t left;
    int16_t right;

    unsigned long now;
    unsigned long p1 = 0;
    unsigned long p2 = 0;

    // Capture both RC inputs without waiting for edges.
    now = micros();

    updateServoInput(
        SERVO_IN_1,
        &throttlePreviousLevel,
        &throttleRiseTime,
        &throttlePulseWidth,
        &throttleLastUpdate,
        &throttleHavePulse,
        now);

    updateServoInput(
        SERVO_IN_2,
        &steeringPreviousLevel,
        &steeringRiseTime,
        &steeringPulseWidth,
        &steeringLastUpdate,
        &steeringHavePulse,
        now);

    now = micros();

    valid1 = getServoPulse(
        throttleHavePulse,
        throttleLastUpdate,
        throttlePulseWidth,
        now,
        &p1);

    valid2 = getServoPulse(
        steeringHavePulse,
        steeringLastUpdate,
        steeringPulseWidth,
        now,
        &p2);

    if (valid1) {
        valid1 = decodeServoSigned(p1, &throttle);
    }

    if (valid2) {
        valid2 = decodeServoSigned(p2, &steer);
    }

#if !MIX_USE_STEERING
    steer = 0;
    valid2 = true;
#endif

#if MIX_STEER_INVERT
    steer = -steer;
#endif

#if !MIX_MODE
    // Independent mode: each channel commands one motor.
    if (!valid1) {
        throttle = 0;
    }

    if (!valid2) {
        steer = 0;
    }

    splitSigned(throttle, &dir1, &speed1);
    splitSigned(steer, &dir2, &speed2);

    driversOn = valid1 && valid2;
#else
    // Mixing mode: invalid throttle stops both motors.
    if (!valid1) {
        throttle = 0;
        steer = 0;
    }

    if (!valid2) {
        steer = 0;
    }

    left = throttle + steer;
    right = throttle - steer;

    if (left > 255) {
        left = 255;
    }

    if (left < -255) {
        left = -255;
    }

    if (right > 255) {
        right = 255;
    }

    if (right < -255) {
        right = -255;
    }

    splitSigned(left, &dir1, &speed1);
    splitSigned(right, &dir2, &speed2);

    driversOn = valid1;
#endif

    // Fault detection and confirmation.
    if (checkFault()) {
        if (badCount < FAULT_CONFIRM) {
            badCount++;
        }
    }
    else {
        badCount = 0;
    }

    faultActive = (badCount >= FAULT_CONFIRM);

#if FAULT_SHUTDOWN
    if (faultActive) {
        dir1 = 0;
        dir2 = 0;
        speed1 = 0;
        speed2 = 0;
        driversOn = false;
    }
#endif

    driveMotor(M1_A, M1_B, dir1, speed1);
    driveMotor(M2_A, M2_B, dir2, speed2);

    digitalWrite(INH_PIN, driversOn ? HIGH : LOW);

    updateLed();
}
