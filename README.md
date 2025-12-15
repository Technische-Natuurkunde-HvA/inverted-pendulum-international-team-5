# Inverted Reaction Wheel Pendulum (IRWP)

Control and experimental study of an inverted pendulum stabilized using a reaction wheel, with a focus on embedded control, PID tuning, and experimental validation.

---

## Project Overview

This project investigates the stabilization of an inverted pendulum using a reaction wheel as the actuation mechanism.  
The objective is to design, implement, and experimentally validate control strategies capable of maintaining the pendulum in its unstable upright equilibrium.

The project is carried out in an academic context and involves:
- embedded control on Arduino,
- sensor integration (AS5600 magnetic encoder),
- DC motor characterization,
- PID-based control design,
- experimental data acquisition and analysis.

---

## System Description

### Hardware
- DC motor with gearbox (nominal speed ≈ 600 RPM @ 12 V)
- Reaction wheel mounted on the pendulum
- AS5600 magnetic encoder for angular position measurement
- Incremental encoder for motor speed measurement
- Arduino microcontroller
- Motor driver (H-bridge)

### Sensors and Actuation
- Pendulum angle measured using the AS5600 encoder
- Motor speed measured via encoder pulses
- Motor driven using PWM with bidirectional control

---

## Control Architecture

The control strategy is based on PID controllers implemented on the Arduino:

- **Angle PID**  
  Main controller responsible for stabilizing the pendulum around the upright equilibrium.

- **Speed PID (experimental)**  
  Secondary controller introduced to limit reaction wheel speed accumulation and mitigate long-term drift.

A weighted combination of both controllers was tested to reduce conflicts between competing objectives.

- **Essential control logic**
  
```cpp
// --- Libraries ---
#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

// --- Sensors & Motor Hardware ---
AS5600 as5600;
const int motorPin1 = 10, motorPin2 = 11, enablePin = 9;
const int encoderPin = 2;

// --- Timing (angle loop + speed loop) ---
const unsigned int LOOP_PERIOD_MS  = 5;
const unsigned int SPEED_PERIOD_MS = 20;
unsigned long lastLoopMs, lastSpeedMs;

// --- Angle Measurement ---
double angleDeg = 0.0;
double angleSet = 0.0;     // target angle
double angleIn  = 0.0;     // measured angle

// --- Angle PID (main controller → PWM) ---
double pwmAngle = 0.0;
double aKp = 50, aKi = 3, aKd = 0.01;
PID anglePID(&angleIn, &pwmAngle, &angleSet, aKp, aKi, aKd, DIRECT);

// --- Speed Measurement ---
volatile long pulseCount = 0;
const double pulsesPerRev = 11 * 9.6;   // motor encoder × gearbox

// --- Speed PID (small damping contribution) ---
double speedIn = 0.0;
double speedSet = 0.0;     // want RPM ≈ 0
double pwmSpeed = 0.0;
double sKp = 2, sKi = 0.1, sKd = 1;
PID speedPID(&speedIn, &pwmSpeed, &speedSet, sKp, sKi, sKd, DIRECT);

// --- Weight of speed correction ---
const double SPEED_WEIGHT = 0.20;

// =========================================================================
//                                SETUP
// =========================================================================
void setup() {
    Wire.begin();
    as5600.begin();
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPin), []{ pulseCount++; }, RISING);

    // Initial angle defines setpoint (vertical = +30°)
    angleDeg = as5600.readAngle() * 0.0879;
    angleSet = angleDeg + 30;

    anglePID.SetMode(AUTOMATIC);
    anglePID.SetOutputLimits(-255, 255);

    speedPID.SetMode(AUTOMATIC);
    speedPID.SetOutputLimits(-60, 60);

    lastLoopMs  = millis();
    lastSpeedMs = millis();
}

// =========================================================================
//                                MAIN LOOP
// =========================================================================
void loop() {
    unsigned long now = millis();

    // --- Speed PID every SPEED_PERIOD_MS ---
    if (now - lastSpeedMs >= SPEED_PERIOD_MS) {
        lastSpeedMs = now;

        noInterrupts();
        long count = pulseCount;
        pulseCount = 0;
        interrupts();

        double rpm = (count * 60.0 / pulsesPerRev) / (SPEED_PERIOD_MS / 1000.0);
        speedIn = rpm;

        speedPID.Compute();
    }

    // --- Angle PID every LOOP_PERIOD_MS ---
    if (now - lastLoopMs >= LOOP_PERIOD_MS) {
        lastLoopMs = now;

        angleDeg = as5600.readAngle() * 0.0879;
        angleIn  = angleDeg;

        anglePID.Compute();                    // main control
        double pwm = pwmAngle + SPEED_WEIGHT * pwmSpeed;

        driveMotor((int)pwm);
    }
}

// =========================================================================
//                             MOTOR DRIVE
// =========================================================================
void driveMotor(int pwm) {
    if (pwm > 0) {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(enablePin, min(pwm, 255));
    } else {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(enablePin, min(-pwm, 255));
    }
}


```
---

## Experimental Results

### Motor Characterization

Motor behaviour was experimentally characterized by sweeping the PWM command and measuring the resulting rotational speed.

Key observations:
- Presence of a dead zone due to static friction.
- Approximately linear RPM–PWM relationship in the mid-range.
- Speed saturation near the nominal motor speed.

Figures are available in the `Visuals/` directory.

---

## Weekly Progress

A detailed week-by-week development log is provided in a separate document:

➡️ **[`docs/weekly_progress.md`](docs/weekly_progress.md)**

This includes:
- hardware assembly,
- sensor debugging,
- motor characterization,
- initial PID stabilization,
- introduction of a second PID loop,
- observed limitations and experimental insights.

---

## Repository Structure

```text
.
├── README.md
├── src/
│   ├── arduino/
│   └── python/
├── Visuals/
│   ├── PWM_RPM_vs_time.png
│   └── RPM_vs_PWM.png
├── docs/
│   └── weekly_progress.md
└── data/

```
---

## Limitations and Future Work

Current limitations include:

- Structural conflict between angle and speed control loops.

- Reaction wheel speed saturation due to motor torque–speed limits.

- Lack of formal anti-windup and state constraints.

Planned improvements:

- Hierarchical or state-based control architecture.

- Explicit speed saturation handling.

- Energy-based swing-up control.
