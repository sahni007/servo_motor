
/*
  STM8S003F3U6 - Dual Motor Controller
  Interrupt-based RC PWM input capture (PA1/PA2 share Port A EXTI,
  serviced by throttleISR()/steeringISR() attached in setup())
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
// volatile: written inside the ISR (updateServoInputISR), read in loop()
volatile uint8_t throttlePreviousLevel = LOW;
volatile unsigned long throttleRiseTime = 0;
volatile unsigned long throttlePulseWidth = 0;
volatile unsigned long throttleLastUpdate = 0;
volatile bool throttleHavePulse = false;

// -------- Input state: steering --------
volatile uint8_t steeringPreviousLevel = LOW;
volatile unsigned long steeringRiseTime = 0;
volatile unsigned long steeringPulseWidth = 0;
volatile unsigned long steeringLastUpdate = 0;
volatile bool steeringHavePulse = false;

// -------- Runtime state --------
uint8_t badCount = 0;
bool faultActive = false;
unsigned long lastBlink = 0;

// -------- Steering debug (temporary) --------
// Set DEBUG_STEER_LED to 1 to use the status LED to visually confirm the
// decoded steering value, bypassing the normal fault-blink behaviour:
//   LED solid ON   -> steer > +40 (stick held right)
//   LED solid OFF  -> steer < -40 (stick held left)
//   LED blinking   -> steer centered (-40..+40) OR channel invalid/timed out
// Move the steering stick fully left/right and watch the LED. If it never
// goes solid ON/OFF, the steering channel (PA2) isn't producing a valid,
// swinging pulse (wiring / receiver channel / endpoint travel problem).
// Set back to 0 to restore normal fault-blink LED behaviour.
#define DEBUG_STEER_LED       1
#define DEBUG_STEER_THRESHOLD 40
volatile int16_t gSteerDebug = 0;

int8_t dir1 = 0;
int8_t dir2 = 0;
uint8_t speed1 = 0;
uint8_t speed2 = 0;

// -------- Non-blocking input capture (interrupt-based) --------
// NOTE on porting from polling to interrupts:
// STM8 EXTI hardware triggers per PORT, not per individual pin - both
// PA1 (throttle) and PA2 (steering) live on Port A, so a single Port-A
// "pin change" interrupt services both channels. Each ISR below only
// touches ITS OWN channel's volatile state variables (exactly like the
// old updateServoInput() calls did), so there is no conflict between them
// even though both can be triggered by the same underlying port event.
//
// IMPORTANT: attachInterrupt()/CHANGE support depends on the specific
// Arduino-STM8 core (Sduino/SDCC) in use. If this core does not support
// attachInterrupt(pin, isr, CHANGE) on arbitrary GPIO pins, this will need
// to be replaced with the core's native EXTI/port-interrupt API instead
// (e.g. a manually written __interrupt ISR tied to the Port A vector).
// Verify this compiles against your actual toolchain before relying on it.
void updateServoInput(
    uint8_t pin,
    volatile uint8_t *previousLevel,
    volatile unsigned long *riseTime,
    volatile unsigned long *pulseWidth,
    volatile unsigned long *lastUpdate,
    volatile bool *havePulse,
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

// ISR for the throttle channel (PA1). Called by the core whenever PA1
// changes level. Keep this as short/fast as possible - just capture the
// timestamp and run the same edge logic as before.
void throttleISR(void)
{
    unsigned long now = micros();

    updateServoInput(
        SERVO_IN_1,
        &throttlePreviousLevel,
        &throttleRiseTime,
        &throttlePulseWidth,
        &throttleLastUpdate,
        &throttleHavePulse,
        now);
}

// ISR for the steering channel (PA2). Called by the core whenever PA2
// changes level.
void steeringISR(void)
{
    unsigned long now = micros();

    updateServoInput(
        SERVO_IN_2,
        &steeringPreviousLevel,
        &steeringRiseTime,
        &steeringPulseWidth,
        &steeringLastUpdate,
        &steeringHavePulse,
        now);
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

#if DEBUG_STEER_LED
    if (gSteerDebug > DEBUG_STEER_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);   // steering RIGHT -> solid ON
        return;
    }
    if (gSteerDebug < -DEBUG_STEER_THRESHOLD) {
        digitalWrite(LED_PIN, LOW);    // steering LEFT -> solid OFF
        return;
    }
    // centered or invalid -> fall through to normal blink below
#endif

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

    // Attach interrupt-based capture (replaces polling in loop()).
    // CHANGE = fire on both rising and falling edges, required since we
    // need to timestamp both the start and end of each pulse.
    // NOTE: verify your specific Sduino/SDCC core actually supports
    // attachInterrupt() with CHANGE mode on PA1/PA2. If not supported,
    // this needs to be replaced with the core's native EXTI/port
    // interrupt registration API instead.
    attachInterrupt(SERVO_IN_1, throttleISR, CHANGE);
    attachInterrupt(SERVO_IN_2, steeringISR, CHANGE);
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

    // Pulse capture now happens asynchronously via throttleISR()/steeringISR()
    // (attached in setup()), triggered by hardware on every PA1/PA2 edge.
    // loop() just reads the latest captured values below - no polling needed.
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

#if DEBUG_STEER_LED
    // Capture the raw decoded steering value (0 if channel invalid/timed
    // out) for the LED diagnostic, before any mixing/invalidation logic
    // below can alter it.
    gSteerDebug = valid2 ? steer : 0;
#endif

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

