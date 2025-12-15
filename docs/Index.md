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

## 4. Core Embedded Code (Essential Extract)

```cpp
#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600 as5600;
const int motorPin1 = 10, motorPin2 = 11, enablePin = 9;
const int encoderPin = 2;

const unsigned int LOOP_PERIOD_MS  = 5;
const unsigned int SPEED_PERIOD_MS = 20;
unsigned long lastLoopMs, lastSpeedMs;

double angleDeg = 0.0, angleSet = 0.0, angleIn = 0.0;
double pwmAngle = 0.0;
double aKp = 50, aKi = 3, aKd = 0.01;
PID anglePID(&angleIn, &pwmAngle, &angleSet, aKp, aKi, aKd, DIRECT);

volatile long pulseCount = 0;
const double pulsesPerRev = 11 * 9.6;

double speedIn = 0.0, speedSet = 0.0, pwmSpeed = 0.0;
double sKp = 2, sKi = 0.1, sKd = 1;
PID speedPID(&speedIn, &pwmSpeed, &speedSet, sKp, sKi, sKd, DIRECT);

const double SPEED_WEIGHT = 0.20;

