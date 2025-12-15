# Inverted Reaction Wheel Pendulum (IRWP)
**Embedded control, motor characterization, and experimental stabilization of an inverted pendulum using a reaction wheel**

---

## 1. Project Overview

This project focuses on the control and experimental study of an inverted pendulum stabilized by a reaction wheel.  
The objective is to design, implement, and validate control strategies capable of maintaining the pendulum in its unstable upright equilibrium.

The work includes:

- Embedded real-time control on Arduino  
- Magnetic angle sensing via AS5600  
- DC motor characterization and modelling  
- Implementation of PID-based controllers  
- Experimental data acquisition and analysis  
- Identification of system limitations  

This system is a classical benchmark in nonlinear control, combining mechanical design, electronics, and control theory.

---

## 2. System Description

### Hardware Components
- DC motor with gearbox (≈600 RPM @ 12 V)  
- Custom reaction wheel mounted on the pendulum  
- AS5600 magnetic encoder (absolute angle)  
- Incremental encoder (motor speed)  
- Arduino microcontroller  
- H-bridge motor driver (L298N)  
- 3D-printed structural components  

### Sensor & Actuator Signals
- Pendulum angle → AS5600  
- Wheel speed → encoder pulses (interrupt-based)  
- Motor torque → PWM command (bidirectional)  

---

## 3. Control Architecture

The system uses a dual-loop PID structure:

### Angle PID (main controller)
- Stabilizes the pendulum at the upright equilibrium  
- Produces the main PWM command  
- Tuned for fast yet stable response  

### Speed PID (secondary controller)
- Limits long-term drift of the reaction wheel  
- Prevents motor speed runaway and saturation  
- Contributes a small corrective signal (≈20%)  

### Combined Control Signal
\[
\text{PWM} = \text{PWM}_{angle} + w \cdot \text{PWM}_{speed}
\]

with \( w = 0.20 \).

### PID Tuning Strategy
- **Increase Kp** → faster response; if oscillatory → reduce  
- **Add Ki** → removes steady-state error; if unstable → reduce  
- **Add Kd** → adds damping; if noisy → reduce  

This step-by-step tuning ensures a justified and reproducible methodology.

---

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
A PWM sweep revealed:

- Dead zone due to static friction

- Linear mid-range RPM–PWM behavior

- Saturation near nominal speed

Plots available in Visuals/:

- PWM_RPM_vs_time.png

- RPM_vs_PWM.png

Stabilization Tests

- Initial stabilization achieved with the angle PID

- Adding the speed PID improved long-term equilibrium

- Drift and saturation significantly reduced
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

## Conclusion 
This project demonstrates the full workflow of implementing and experimentally validating a reaction-wheel inverted pendulum.
The system successfully achieves stabilization, and the framework is ready for more advanced nonlinear control strategies.
