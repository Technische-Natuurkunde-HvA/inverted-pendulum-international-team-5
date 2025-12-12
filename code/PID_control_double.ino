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
