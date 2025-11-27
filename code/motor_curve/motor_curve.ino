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
