#include <Arduino.h>
#include "nd005.hpp"

const pin_size_t CS  = 17;
const pin_size_t DAV = 14;

void setup() {
  setupSensor(CS, DAV);
  adjustRange(PressureRangeSettings::PSI05);

  Serial.begin(9600);

  while (!Serial) delay(10); // Wait until a PC is connected

  Serial.println("PIETOSTATIC SYSTEM\n");
}

void loop() {

  Serial.print("Pressure reading ");
  Serial.println(readPressure(CS));

  delay(10);
}
