#include <Wire.h>
#include <AS5600.h>
#include <SPI.h>
#include <SD.h>

AS5600 as5600;

unsigned long currentMs;
unsigned long lastMs;
const unsigned int FREE_RUN_PERIOD_MS = 100;
float sig_angle_deg;

const int chipSelect = 4;   // change if you use a different CS-pin

File dataFile;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  as5600.begin();
  lastMs = millis();

  Serial.println("Initialising SD-card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("SD-card not found!");
    while (1);
  }
  Serial.println("SD-card found.");

  // opening or creating CSV-file 
  dataFile = SD.open("data.csv", FILE_WRITE);         #file name (when changed also change the name in the plotting code)

  if (dataFile) {
    // writing CSV-header 
    dataFile.println("time_ms,angle_deg");
    dataFile.close();
  } else {
    Serial.println("could not open data.csv!");
  }

  delay(2000);
}

void loop() {
  currentMs = millis();

  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {
    lastMs = currentMs;

    sig_angle_deg = as5600.readAngle() * 0.0879;

    // opening file again in append-mode
    dataFile = SD.open("data.csv", FILE_WRITE);

    if (dataFile) {
      dataFile.print(currentMs);
      dataFile.print(",");
      dataFile.println(sig_angle_deg);
      dataFile.close();

      Serial.print("Logged: ");
      Serial.print(currentMs);
      Serial.print(", ");
      Serial.println(sig_angle_deg);
    } 
    else {
      Serial.println("Error: could not write CSV-file");
    }
  }
}

