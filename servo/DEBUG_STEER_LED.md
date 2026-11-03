# DEBUG_STEER_LED — How It Works

## Flowchart

```mermaid
flowchart TD
    A[loop start] --> B[now = micros]
    B --> C[updateServoInput: PA2 steering]
    C --> D[getServoPulse: p2, valid2]
    D --> E{valid2?}
    E -->|Yes| F[decodeServoSigned p2 -> steer]
    E -->|No| G[steer stays previous/0]
    F --> H
    G --> H{"#if DEBUG_STEER_LED"}
    H -->|compiled in, value=1| I["gSteerDebug = valid2 ? steer : 0"]
    H -.->|if value=0, block skipped entirely| Z1[gSteerDebug unused / not compiled]

    I --> J[... rest of loop: mixing, driveMotor, etc ...]
    Z1 --> J
    J --> K[updateLed called at end of loop]

    K --> L{"#if DEBUG_STEER_LED"}
    L -->|value=1| M{gSteerDebug > +40 ?}
    L -.->|value=0, skip to normal blink| R[Normal fault blink logic]

    M -->|Yes| N[digitalWrite LED_PIN HIGH\nreturn immediately]
    M -->|No| O{gSteerDebug < -40 ?}
    O -->|Yes| P[digitalWrite LED_PIN LOW\nreturn immediately]
    O -->|No, centered/invalid| R

    R --> S{faultActive?}
    S -->|Yes| T[interval = 100ms]
    S -->|No| U[interval = 1000ms]
    T --> V[toggle LED if interval elapsed]
    U --> V

    N --> W[loop end]
    P --> W
    V --> W
    W --> A
```

## State Diagram — what the LED shows for each steering position

```mermaid
stateDiagram-v2
    [*] --> Centered
    Centered: steer in -40..+40\nor channel invalid/timed out\nLED: blinking (1Hz, or 100ms if fault)
    Right: steer > +40\nstick held RIGHT\nLED: solid ON
    Left: steer < -40\nstick held LEFT\nLED: solid OFF

    Centered --> Right: stick moved right past +40
    Right --> Centered: stick returns toward center
    Centered --> Left: stick moved left past -40
    Left --> Centered: stick returns toward center
    Right --> Left: stick swept fully across (passes through Centered internally each loop)
    Left --> Right: stick swept fully across
```

## What each outcome tells you

| Observation while moving steering stick | Meaning |
|---|---|
| LED snaps solid ON at right, solid OFF at left | `PA2` capture + decode works correctly - bug must be elsewhere (unlikely given motor side already verified) |
| LED just keeps blinking no matter what | `gSteerDebug` never leaves -40..+40 - PA2 isn't producing a valid, wide-swinging pulse (wiring / receiver channel / endpoint travel problem) |
| LED flickers erratically | Noisy/intermittent signal on PA2 |

