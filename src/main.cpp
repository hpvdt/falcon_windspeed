#include <Arduino.h>
#include "nd005.hpp"
#include <SPI.h>

// Added "_USED" due to naming collision in library with just functional names
static const pin_size_t MISO_USED = 4;
static const pin_size_t MOSI_USED = 7;
static const pin_size_t SCK_USED = 6;
MbedSPI sensorSPI(MISO_USED, MOSI_USED, SCK_USED);

PressureSensor sensor[4] = {
  PressureSensor(17, 14, &sensorSPI), 
  PressureSensor(5, 15, &sensorSPI), 
  PressureSensor(9, 16, &sensorSPI), 
  PressureSensor(13, 18, &sensorSPI)
  };


void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    sensor[i].adjustRange(PressureRangeSettings::PSI05);
  }

  Serial.begin(9600);

  while (!Serial) delay(10); // Wait until a PC is connected

  Serial.println("PEETOESTATIK CYSTDUMB\n");
}

int j = 0;

void loop() {
  j = j + 1;
  Serial.print(j);
  Serial.print(" Pressure readings");
  for (byte i = 0; i < 4; i++) {
    Serial.print(" ");
    Serial.print(sensor[i].readPressure());
  }
  Serial.println("");
  delay(250);
}