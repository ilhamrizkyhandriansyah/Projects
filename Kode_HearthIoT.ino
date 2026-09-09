#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

PulseOximeter pox;
uint32_t tsLastReport = 0;

void onBeatDetected() {
Serial.println("💓 Beat!");
}

void setup() {
Serial.begin(115200);
Serial.println("Start Pulse Oximeter...");

if (!pox.begin()) {
Serial.println("FAILED to initialize pulse oximeter");
while (1);
} else {
Serial.println("SUCCESS");
pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
pox.setOnBeatDetectedCallback(onBeatDetected);
}
}
int suhuPin = A0;
float suhu;

void loop() {
pox.update();

// Suhu
int analogValue = analogRead(suhuPin);

suhu = analogValue * (3.3 / 1024.0) * 100.0;

if (millis() - tsLastReport > 1000) {
tsLastReport = millis();
Serial.print("Heart rate: ");
Serial.print(pox.getHeartRate());
Serial.print(" bpm / SpO2: ");
Serial.print(pox.getSpO2());
Serial.print(" % / Suhu: ");
Serial.print(suhu);
Serial.println(" °C");
}
}