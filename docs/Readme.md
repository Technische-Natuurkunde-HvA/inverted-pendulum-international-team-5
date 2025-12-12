#Week 5 — Integration of a Second PID Controller for Motor Speed Regulation

During the fifth week, we worked on extending the control architecture by introducing a second PID loop dedicated to regulating the motor speed. The objective was to maintain the reaction wheel’s angular velocity close to zero during stabilization, thereby preventing the long-term drift observed in the previous implementation.

After coding the speed-regulation PID, we rapidly identified a fundamental issue:
the new controller issued commands that opposed those of the angle-regulation PID.
While the angle PID increases motor speed to keep the pendulum upright, the speed PID simultaneously attempts to reduce that speed to zero. This antagonistic behaviour created a direct conflict between the two control loops.

The practical consequence was a pronounced vibration of the motor, caused by the two PIDs continuously correcting each other. The system oscillated around contradictory objectives, resulting in a high-frequency switching of the command signal.

The main challenge for the week was therefore to determine appropriate tuning parameters for the second PID so that its influence remained sufficiently limited and did not destabilize the primary angle-regulation loop. This required extensive experimentation: implementing the controller, adjusting its gains, analysing the motor response, and iteratively reducing its authority until the vibrations were mitigated.

Although this tuning process was time-consuming, it allowed us to better understand the interaction between the two nested control loops and to identify the need for a more structured coordination strategy (e.g., loop decoupling, hierarchical prioritization, or state-based switching) in future iterations.# Week 4 — Development of Bidirectional Motion and First PID Tests

During this week, we focused on implementing the bidirectional actuation of the reaction-wheel pendulum.
We first measured the angular limits corresponding to the left, right, and center positions, and then developed a function that automatically inverts the rotation direction of the motor once one of the extreme angles is reached. This enabled the pendulum to swing continuously from left to right.

After establishing this open-loop behaviour, we integrated our teacher’s PID control structure into the code.
We experimented with different combinations of Kp, Ki, and Kd in order to stabilize the pendulum around the upright equilibrium (θ = 0).

We managed to obtain a set of PID gains that temporarily stabilized the pendulum for approximately 7 seconds.
However, the system still exhibits the following limitation:

* The reaction wheel gradually accumulates angular velocity,
* The motor eventually reaches a speed region where it loses torque capability,
* As a result, the pendulum can no longer be balanced and eventually falls to one side.

This behaviour is consistent with torque–speed limitations of DC motors and must be addressed in future iterations (anti-windup, state constraints, velocity saturation, etc.).

# Week 3 — Diagnosis and Resolution of AS5600 Sensor Failures

This week was dedicated to solving a critical issue with the AS5600 magnetic encoder, which repeatedly failed to operate.
We tested:

* four different AS5600 sensors,
* several sets of wires,
* and even another Arduino board.

All combinations reproduced the same issue until we tried a fifth AS5600 module without any soldering applied.
This one worked immediately.

Root cause analysis

The consistent failures observed only after soldering strongly indicate that the AS5600 boards were damaged during soldering operations. Possible mechanisms include:

* overheating of PCB pads or internal traces,
* accidental short-circuits on SDA/SCL lines,
* lifted pads or micro-fractures due to mechanical stress,
* cold solder joints or partial connections.

The fact that the unsoldered module worked perfectly confirms the diagnosis.

Recommendations

* Avoid soldering until all electrical tests are validated.
* Prefer pre-soldered header pins or low-temperature solder.
* Systematically check continuity (SDA, SCL, VCC, GND) with a multimeter before powering.
* Handle small-pad PCBs carefully to prevent thermal or mechanical damage.

Thanks to these corrections, the sensor is now operational and can reliably measure the pendulum angle.

After solving these problems, we were able to create some graphs concerning the DC motor curves which was tested under these conditions :

* Speed = 620 RPM
* Voltage = 12V

![Une image contenant texte, diagramme, ligne, Tracé  Le contenu généré par l’IA peut être incorrect.](data:image/png;base64...)

The Arduino code increases the PWM value by 1 every second, and the corresponding rotational speed is measured.

The curves are not perfectly smooth, probably because the time interval between two PWM values was too short, meaning the rotor could not fully stabilize before the next measurement.

There is a zone where the rotor speed remains equal to zero.In this interval, the PWM value is too small, therefore the average motor current is low, and the electromagnetic torque is not sufficient to overcome static friction. This region corresponds to the classical dead zone.

![Une image contenant texte, diagramme, ligne, Tracé  Le contenu généré par l’IA peut être incorrect.](data:image/png;base64...)

This graph clearly shows the evolution of RPM as a function of PWM.
The dead zone is very visible and is directly caused by friction and the minimum torque required to initiate rotation.

We also observe that:

* The motor tends toward a maximum speed close to 600 RPM, which is consistent with its nominal value.
* However, the motor reaches saturation before PWM = 255.
  Beyond approximately PWM ≈ 200, the speed no longer increases significantly because the motor is already operating near its maximum no-load speed at the given supply voltage.
* Between PWM ≈ 50 and PWM ≈ 180, the relationship between PWM and RPM is nearly linear, as expected for a DC motor operating in its linear torque–speed region.

# Week 2 — Pendulum Fabrication, RPM Measurement, and Data Acquisition Tools

This week we crafted the physical pendulum and checked that all mechanical and electrical components functioned correctly.

We then modified the Arduino code to compute rotational speed in RPM instead of Hz, in order to verify whether the motor reached its nominal specification (600 RPM ±10 %, considering sensor tolerance).

Next, we extended the Arduino firmware to automatically increment the PWM signal and print the corresponding motor speed in CSV format, enabling direct plotting in the Arduino Serial Plotter or external software.

We also developed a Python script capable of reading the pendulum angle in real time and visualizing its motion dynamically.
This provided immediate insight into oscillation behaviour, signal quality, and sensor responsiveness.

Finally, we ensured that the Python environment was configured and running correctly on all laptops so that the whole team could work efficiently in parallel

# Week 1 — Hardware Assembly and First Motor Tests

During the first week, we focused on assembling the hardware.
The motor driver was mounted without any issues, and initial tests were performed on the motor.

At first, we used a 130 RPM motor, but quickly switched to what we believed was a 620 RPM motor.
Later, we confirmed that its actual nominal speed was 600 RPM, which matches the motor used by the Amsterdam team — this alignment is preferable for consistency across sites.

The only remaining issue identified at this stage was the reaction-wheel design, which turned out to be too small for the required inertia and will therefore need to be redesigned.
