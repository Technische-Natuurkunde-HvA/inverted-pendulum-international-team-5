// Motor control pins
const int motorPin1 = 10; // IN1 
const int motorPin2 = 11; // IN2 
const int enablePin = 9;  // ENA (PWM pin for speed control)

// Encoder
const int encoderPin = 2;
volatile int pulseCount = 0;
const int pulsesPerRevolution = 11; 

// Timing
unsigned long lastTime = 0;

// Variables
int pwmValue = 0;  // Start at 0


void countPulse() {
  pulseCount++;
}

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  Serial.println("PWM,RPM");  // header for Excel
}

void loop() {

  // ---- 1. Envoie du PWM courant ----
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(enablePin, pwmValue);

  // ---- 2. Mesure chaque seconde ----
  if (millis() - lastTime >= 5000) {

    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    // freq encoder (Hz)
    double freq = count / (double)pulsesPerRevolution;

    // RPM
    int gearRatio = 46;
    double rpm = freq * 60.0/gearRatio;

    // ---- 3. Impression CSV ----
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.println(rpm);

    // ---- 4. PWM suivant ----
    pwmValue += 5;    // step de 5 pour lisser la courbe (ou mets +1 si tu veux 256 points)
    if (pwmValue > 255) {
      pwmValue = 255;
      // tu peux stopper ou boucler :
      // while(true);   // stop après 255
    }

    lastTime = millis();
  }
}
