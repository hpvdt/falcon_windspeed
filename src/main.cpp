#include <Arduino.h>
#include "nd005.hpp"

const pin_size_t CS  = 17;
const pin_size_t DAV = 14;
const pin_size_t RESET = 30;

void setup() {
  setupSensor(CS, DAV, RESET);
  adjustRange(PressureRangeSettings::PSI05);

  Serial.begin(9600);
}

void loop() {

  Serial.println(readTemperature(CS));

  delay(100);
}
