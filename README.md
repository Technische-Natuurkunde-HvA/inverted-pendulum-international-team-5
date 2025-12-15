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
