#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <math.h>

// ==========================
//   CAPTEUR ANGULAIRE
// ==========================
AS5600 as5600;

// ==========================
//   TIMING
// ==========================
unsigned long currentMs;
unsigned long lastLoopMs;
unsigned long lastSpeedMeasureMs;
unsigned long lastLogMs;

const unsigned int LOOP_PERIOD_MS   = 5;
const unsigned int SPEED_PERIOD_MS  = 20;
const unsigned int LOG_PERIOD_MS    = 20;

// ==========================
//   HARDWARE MOTEUR
// ==========================
const int motorPin1  = 10;
const int motorPin2  = 11;
const int enablePin  = 9;

// ==========================
//   ENCODEUR
// ==========================
const int encoderPin = 2;
volatile long pulseCount = 0;

const int pulsesPerRevolution = 11;
const double gearRatio        = 9.6;
const double PULSES_PER_OUTPUT_REV = pulsesPerRevolution * gearRatio;

int lastMotorDir = 0;

// ==========================
//   ANGLE
// ==========================
double sig_angle_deg = 0.0;
double angle0        = 0.0;

// ==========================
//   PID ANGLE
// ==========================
double angleSetpointBase = 0.0;
double angleSetpoint     = 0.0;
double angleInput        = 0.0;
double pwmAngle          = 0.0;

double aKp = 35.0, aKi = 4.0, aKd = 2.0;
PID anglePID(&angleInput, &pwmAngle, &angleSetpoint, aKp, aKi, aKd, DIRECT);

// ==========================
//   PID VITESSE
// ==========================
double speedInput    = 0.0;
double speedMeasured = 0.0;
double speedSetpoint = 0.0;
double pwmSpeed      = 0.0;

double sKp = 1.0, sKi = 0.01, sKd = 0.1;
PID speedPID(&speedInput, &pwmSpeed, &speedSetpoint, sKp, sKi, sKd, DIRECT);

const double SPEED_WEIGHT = 0.3;

const int MIN_PWM = 35;
const int CMD_DEADBAND = 10;
const double SPEED_DEADBAND_RPM = 5.0;


// ==========================
//   PROTOTYPES
// ==========================
void countPulse();
double calculateRPM(unsigned long dtMs);
void readAngle();
void driveMotorFromPWM(int cmdPWM);

// ==========================
//   SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  as5600.begin();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  delay(100);
  readAngle();

  angle0 = sig_angle_deg;
  angleSetpointBase = angle0 + 28.0;
  angleSetpoint = angleSetpointBase;

  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(LOOP_PERIOD_MS);
  anglePID.SetOutputLimits(-255, 255);

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(SPEED_PERIOD_MS);
  speedPID.SetOutputLimits(-60, 60);

  lastLoopMs = lastSpeedMeasureMs = lastLogMs = millis();

  Serial.println("STARTING...");
}

// ==========================
//   LOOP
// ==========================
void loop() {
  currentMs = millis();

  // -------- MESURE RPM --------
  if (currentMs - lastSpeedMeasureMs >= SPEED_PERIOD_MS) {
    unsigned long dtMs = currentMs - lastSpeedMeasureMs;
    lastSpeedMeasureMs = currentMs;

    speedMeasured = calculateRPM(dtMs);
    speedInput = fabs(speedMeasured) < SPEED_DEADBAND_RPM ? 0.0 : speedMeasured;

    speedPID.Compute();
  }

  // -------- PID ANGLE + PWM --------
  if (currentMs - lastLoopMs >= LOOP_PERIOD_MS) {
    lastLoopMs = currentMs;

    readAngle();
    angleInput = sig_angle_deg;
    angleSetpoint = angleSetpointBase;

    anglePID.Compute();

    double pwmTotalDouble = pwmAngle + SPEED_WEIGHT * pwmSpeed;
    int pwmTotal = (int)pwmTotalDouble;

    if (abs(pwmTotal) < CMD_DEADBAND) pwmTotal = 0;

    driveMotorFromPWM(pwmTotal);

    // -------- LOG (temps, angle, rpm, pwm) --------
    if (currentMs - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = currentMs;

      double t_s = currentMs / 1000.0;
      double angleCorrected = angleInput - angle0 - 28.0;

      Serial.print(t_s, 3); Serial.print(',');
      Serial.print(angleCorrected, 3); Serial.print(',');
      Serial.print(speedMeasured, 3); Serial.print(',');
      Serial.println(pwmTotal);
    }
  }
}


// ==========================
//   FONCTIONS
// ==========================
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
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    motorPWM = 0;
    lastMotorDir = 0;
  }

  motorPWM = constrain(motorPWM, 0, 255);
  analogWrite(enablePin, motorPWM);
}

void countPulse() {
  pulseCount++;
}

double calculateRPM(unsigned long dtMs) {
  noInterrupts();
  long count = pulseCount;
  pulseCount = 0;
  interrupts();

  if (dtMs == 0) return 0.0;

  double dt = dtMs / 1000.0;
  double pulsesPerSec = count / dt;
  double revPerSec = pulsesPerSec / PULSES_PER_OUTPUT_REV;
  double rpmMag = revPerSec * 60.0;

  static double filtered = 0.0;
  filtered = 0.3 * rpmMag + 0.7 * filtered;

  if (lastMotorDir < 0) return -filtered;
  if (lastMotorDir > 0) return  filtered;

  return filtered; // moteur neutre → signe positif par défaut
}

void readAngle() {
  sig_angle_deg = (float)as5600.readAngle() * 0.0879;
}
