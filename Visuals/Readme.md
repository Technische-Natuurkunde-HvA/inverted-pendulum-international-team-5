
[[_TOC_]]
  # Week 5 — Integration of a Second PID Controller for Motor Speed Regulation

During the fifth week, we worked on extending the control architecture by introducing a second PID loop dedicated to regulating the motor speed. The objective was to maintain the reaction wheel’s angular velocity close to zero during stabilization, thereby preventing the long-term drift observed in the previous implementation.

After coding the speed-regulation PID, we rapidly identified a fundamental issue:
the new controller issued commands that opposed those of the angle-regulation PID.
While the angle PID increases motor speed to keep the pendulum upright, the speed PID simultaneously attempts to reduce that speed to zero. This antagonistic behaviour created a direct conflict between the two control loops.

The practical consequence was a pronounced vibration of the motor, caused by the two PIDs continuously correcting each other. The system oscillated around contradictory objectives, resulting in a high-frequency switching of the command signal.

The main challenge for the week was therefore to determine appropriate tuning parameters for the second PID so that its influence remained sufficiently limited and did not destabilize the primary angle-regulation loop. This required extensive experimentation: implementing the controller, adjusting its gains, analysing the motor response, and iteratively reducing its authority until the vibrations were mitigated.

Although this tuning process was time-consuming, it allowed us to better understand the interaction between the two nested control loops and to identify the need for a more structured coordination strategy (e.g., loop decoupling, hierarchical prioritization, or state-based switching) in future iterations.

```cpp

#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <math.h>

AS5600 as5600;

// --- TIMING ---
unsigned long currentMs;
unsigned long lastLoopMs;
unsigned long lastSpeedMeasureMs;

const unsigned int LOOP_PERIOD_MS  = 5;   // boucle angle
const unsigned int SPEED_PERIOD_MS = 20;  // mesure RPM

// --- HARDWARE ---
const int motorPin1  = 10;
const int motorPin2  = 11;
const int enablePin  = 9;

const int encoderPin = 2;
volatile long pulseCount = 0;            // pulses de l’encodeur

// Encodeur / mécanique
const int    pulsesPerRevolution = 11;   // pulses / tour moteur
const double gearRatio           = 9.6;  // rapport de réduction
const double PULSES_PER_OUTPUT_REV = pulsesPerRevolution * gearRatio;

// --- ANGLE (AS5600) ---
double sig_angle_deg = 0.0;

// --- ANGLE + PID ANGLE ---
// Angle
double angleSetpointBase = 0.0;   // sera défini à l'initialisation
double angleSetpoint     = 0.0;
double angleInput        = 0.0;

// Sortie du PID angle (commande principale)
double pwmAngle = 0.0;

// PID ANGLE : COMMANDE PRINCIPALE
double aKp = 50.0, aKi = 3.0, aKd = 0.01;
PID anglePID(&angleInput, &pwmAngle, &angleSetpoint, aKp, aKi, aKd, DIRECT);

// --- VITESSE + PID VITESSE (AMORTISSEUR) ---
double speedInput    = 0.0;  // RPM mesuré (avec signe approximatif)
double speedSetpoint = 0.0;  // on veut vitesse ≈ 0 RPM

// Sortie du PID vitesse (petite correction de PWM)
double pwmSpeed = 0.0;

// PID VITESSE : faible gain, pour amortir
double sKp = 2, sKi = 0.1, sKd = 1;
PID speedPID(&speedInput, &pwmSpeed, &speedSetpoint, sKp, sKi, sKd, DIRECT);

// Poids de la contribution vitesse dans la PWM totale
const double SPEED_WEIGHT = 0.20;  // 0.3 => 30 % du poids de l’angle

// --- DEADZONE MOTEUR ---
const int MIN_PWM       = 35; // pour vaincre les frottements
const int CMD_DEADBAND  = 10; // petite commande ignorée
const double SPEED_DEADBAND_RPM = 5.0; // vitesse considérée nulle

// Pour donner un signe au RPM (mono-canal)
int lastMotorDir = 0; // -1, 0, +1

// --- PROTOTYPES ---
void countPulse();
double calculateRPM();
void readAngle();
void driveMotorFromPWM(int cmdPWM);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  as5600.begin();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  delay(100);          // petite pause pour laisser l’AS5600 se stabiliser
  readAngle();         // mesure de l’angle initial
  // --- Calibration auto : consigne = angle initial + 15° ---
  angleSetpointBase = sig_angle_deg + 30;
  angleSetpoint     = angleSetpointBase;

  // PID ANGLE
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(LOOP_PERIOD_MS);
  anglePID.SetOutputLimits(-255.0, 255.0); // sortie = PWM

  // PID VITESSE
  speedSetpoint = 0.0;         // on veut RPM = 0
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(SPEED_PERIOD_MS);
  speedPID.SetOutputLimits(-60.0, 60.0);   // correction max faible

  lastLoopMs         = millis();
  lastSpeedMeasureMs = millis();

  Serial.println("STARTING...");
  Serial.print("Angle initial = "); Serial.print(sig_angle_deg);
  Serial.print(" deg, consigne = "); Serial.println(angleSetpointBase);
}

void loop() {
  currentMs = millis();

  // --- Mesure + PID VITESSE toutes les SPEED_PERIOD_MS ---
  if (currentMs - lastSpeedMeasureMs >= SPEED_PERIOD_MS) {
    lastSpeedMeasureMs = currentMs;

    double currentRPM = calculateRPM();
    speedInput = currentRPM;

    // Zone morte sur la vitesse
    if (fabs(speedInput) < SPEED_DEADBAND_RPM) {
      speedInput = 0.0;
    }

    // PID VITESSE -> pwmSpeed (petite correction)
    speedPID.Compute();
  }

  // --- BOUCLE ANGLE toutes les LOOP_PERIOD_MS ---
  if (currentMs - lastLoopMs >= LOOP_PERIOD_MS) {
    lastLoopMs = currentMs;

    // 1) Mesurer l’angle
    readAngle();
    angleInput    = sig_angle_deg;
    angleSetpoint = angleSetpointBase;

    // 2) PID ANGLE -> pwmAngle (commande principale)
    anglePID.Compute();

    // 3) Somme pondérée des deux contributions
    double pwmTotalDouble = pwmAngle + SPEED_WEIGHT * pwmSpeed;
    int    pwmTotal       = (int)pwmTotalDouble;

    // Deadband sur la commande totale
    if (abs(pwmTotal) < CMD_DEADBAND) {
      pwmTotal = 0;
    }

    // 4) Appliquer au moteur
    driveMotorFromPWM(pwmTotal);

    // DEBUG
    /*
    Serial.print("Ang:");   Serial.print(angleInput);
    Serial.print("\tSet:"); Serial.print(angleSetpoint);
    Serial.print("\tRPM:"); Serial.print(speedInput);
    Serial.print("\tPWMang:"); Serial.print(pwmAngle);
    Serial.print("\tPWMspd:"); Serial.print(pwmSpeed);
    Serial.print("\tPWMtot:"); Serial.println(pwmTotal);
    */
  }
}

// --- PILOTAGE MOTEUR AVEC DEADZONE ---
void driveMotorFromPWM(int cmdPWM) {
  int motorPWM = 0;

  if (cmdPWM > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    motorPWM = cmdPWM + MIN_PWM;
    lastMotorDir = +1;
  } else if (cmdPWM < 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    motorPWM = abs(cmdPWM) + MIN_PWM;
    lastMotorDir = -1;
  } else {
    motorPWM = 0;
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    lastMotorDir = 0;
  }

  if (motorPWM > 255) motorPWM = 255;
  if (motorPWM < 0)   motorPWM = 0;

  analogWrite(enablePin, motorPWM);
}

// --- INTERRUPT ENCODEUR ---
void countPulse() {
  pulseCount++;
}

// --- CALCUL RPM SORTIE ---
double calculateRPM() {
  noInterrupts();
  long count = pulseCount;
  pulseCount = 0;
  interrupts();

  double dt = SPEED_PERIOD_MS / 1000.0; // en secondes

  double pulsesPerSec     = count / dt;
  double revPerSecOutput  = pulsesPerSec / PULSES_PER_OUTPUT_REV;
  double rpmMag           = revPerSecOutput * 60.0;

  // Filtrage exponentiel simple
  static double filteredRPM = 0.0;
  double alpha = 0.3;
  filteredRPM = alpha * rpmMag + (1.0 - alpha) * filteredRPM;

  // Donne un signe au RPM en fonction du dernier sens de commande
  if (lastMotorDir < 0) return -filteredRPM;
  if (lastMotorDir > 0) return  filteredRPM;
  return 0.0; // si on ne commande plus, on considère qu'on s'arrête
}

// --- LECTURE ANGLE AS5600 ---
void readAngle() {
  // 0.0879 ≈ 360 / 4096 -> conversion en degrés
  sig_angle_deg = (float)as5600.readAngle() * 0.0879;
}

```

# Week 4 — Development of Bidirectional Motion and First PID Tests

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


```cpp
#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600  as5600;  //create sensor object

unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5; //sampling period in milliseconds
double sig_angle_deg;  // angle measurement

// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)


double setpoint = 349; // Desired angle (vertical position)
double output = 0;

// PID parameters
double Kp = 40;        //45
double Ki = 5;         //0.3
double Kd = 0.001;     //0.001
PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void readAndPrintAngle();

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Wire.begin();     // Initialize I2C
  as5600.begin();   // Initialize sensor
  lastMs= millis();   // Initialize timing
  Serial.begin(9600);  // Initialize Serial Monitor
  delay(5000);
  Serial.print("Test: ");
  Serial.println();

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREE_RUN_PERIOD_MS); // Set sample time in milliseconds
  myPID.SetOutputLimits(-255,255); // YOU CAN ADJUST THESE OUTPUT LIMITS IF YOU WISH
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling

    readAndPrintAngle();

    myPID.Compute(); // Calculate PID output

    // Set motor direction based on PID output
    if (output > 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      
    } else {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      
    }

    analogWrite(enablePin, abs(output));

    // Print the angle to the Serial Monitor
    Serial.print(sig_angle_deg);
    Serial.print(" ");
    // Print PID output for debugging
    Serial.println(output);
 
  }
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = (float)as5600.readAngle()*0.0879; //0.0879=360/4096;  // degrees [0..360) 
}
```
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

![PWM,RPM = f(time)](Visuals/Measured%20PWM%20and%20RPM%20against%20time.png)


The Arduino code increases the PWM value by 1 every second, and the corresponding rotational speed is measured.

The curves are not perfectly smooth, probably because the time interval between two PWM values was too short, meaning the rotor could not fully stabilize before the next measurement.

There is a zone where the rotor speed remains equal to zero.In this interval, the PWM value is too small, therefore the average motor current is low, and the electromagnetic torque is not sufficient to overcome static friction. This region corresponds to the classical dead zone.

![Une image contenant texte, diagramme, ligne, Tracé  Le contenu généré par l’IA peut être incorrect.](data:Visuals/Measured PWM and RPM against time.png)

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

```cpp

// Motor control pins
const int motorPin1 = 10; // IN1 
const int motorPin2 = 11; // IN2 
const int enablePin = 9;  // ENA (PWM pin for speed control)

// Encoder
const int encoderPin = 2;
volatile long pulseCount = 0;
const int pulsesPerRevolution = 11;  // pulses par tour moteur

// Mécanique
const double gearRatio = 9.6;

// Timing
const unsigned long samplePeriodMs = 500;   // mesure toutes les 0.5 s
const unsigned long pwmStepPeriodMs = 1000; // 1 seconde entre les changements de PWM
unsigned long lastMeasureTime = 0;
unsigned long lastPwmStepTime = 0;

// Mesure lissée
double rpmAccumulator = 0;
int rpmSamples = 0;

// PWM
int pwmValue = -255;   // départ


void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  Serial.println("PWM,RPM");

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  lastMeasureTime = millis();
  lastPwmStepTime  = millis();
}

void loop() {

  // ---- 1. Appliquer le PWM ----
  if (pwmValue >= 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, pwmValue);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, -pwmValue);
  }


  // ---- 2. Mesure toutes les 0.5 s ----
  if (millis() - lastMeasureTime >= samplePeriodMs) {

    noInterrupts();
    long count = pulseCount;
    pulseCount = 0;
    interrupts();

    double T = samplePeriodMs / 1000.0; // en secondes

    // pulses moteur / seconde
    double pulsesPerSec = count / T;

    // tours moteur / sec
    double revPerSecMotor = pulsesPerSec / pulsesPerRevolution;

    // RPM moteur
    double rpmMotor = revPerSecMotor * 60.0;

    // RPM en sortie
    double rpmOutput = rpmMotor / gearRatio;

    // ---- Lisser (moyenne sur 1 s = 2 mesures) ----
    rpmAccumulator += rpmOutput;
    rpmSamples++;

    lastMeasureTime = millis();
  }


  // ---- 3. Changement de PWM toutes les 1 s ----
  if (millis() - lastPwmStepTime >= pwmStepPeriodMs) {

    if (rpmSamples > 0) {
      double rpmAvg = rpmAccumulator / rpmSamples;

      // ---- Impression CSV ----
      Serial.print(pwmValue);
      Serial.print(",");
      Serial.println(rpmAvg, 2);
    }

    // Reset accumulation pour prochaine mesure
    rpmAccumulator = 0;
    rpmSamples = 0;

    // ---- Étape PWM suivante ----
    pwmValue++;

    if (pwmValue > 255) {
      Serial.println("END");
      while (true); // stop tout
    }

    lastPwmStepTime = millis();
  }
}

```

# Week 1 — Hardware Assembly and First Motor Tests

During the first week, we focused on assembling the hardware.
The motor driver was mounted without any issues, and initial tests were performed on the motor.

At first, we used a 130 RPM motor, but quickly switched to what we believed was a 620 RPM motor.
Later, we confirmed that its actual nominal speed was 600 RPM, which matches the motor used by the Amsterdam team — this alignment is preferable for consistency across sites.

The only remaining issue identified at this stage was the reaction-wheel design, which turned out to be too small for the required inertia and will therefore need to be redesigned.
