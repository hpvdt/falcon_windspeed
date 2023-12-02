#include <Arduino.h>
#include "nd005.hpp"

const pin_size_t CS[]  = {17, 5, 9, 13};
const pin_size_t DAV[] = {14, 15, 16, 18};

void setup() {
  for (byte i = 0; i < 4; i++) setupSensor(CS[i], DAV[i]);
  adjustRange(PressureRangeSettings::PSI05); // Note this applies to all sensors!

  Serial.begin(9600);

  while (!Serial) delay(10); // Wait until a PC is connected

  Serial.println("PIETOSTATIC SYSTEM\n");
}

void loop() {

  Serial.print("Pressure readings");
  for (byte i = 0; i < 4; i++) {
    Serial.print(" ");
    Serial.print(readPressure(CS[i]));
  }
  Serial.println("");
  delay(100);
}
