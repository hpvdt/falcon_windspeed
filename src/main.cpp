#include <Arduino.h>
#include "nd005.hpp"
#include <SPI.h>

// Added "_USED" due to naming collision in library with just functional names
static const pin_size_t MISO_USED = 4;
static const pin_size_t MOSI_USED = 7;
static const pin_size_t SCK_USED = 6;
MbedSPI sensorSPI(MISO_USED, MOSI_USED, SCK_USED);

PressureSensor sensor[4] = {
  PressureSensor(17, 14, &sensorSPI, 1.04719755, 1.570796323), // deg: 60, 90 
  PressureSensor(5, 15, &sensorSPI, 2.09439510, 1.57079633), // deg: 120, 90 
  PressureSensor(9, 16, &sensorSPI, 3.14159265, 1.57079633), // deg: 180, 90 
  PressureSensor(13, 18, &sensorSPI, 1.57079632, 0.00000000)   // deg: 90, 0 
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
  // output all sensor readings
  j = j + 1;
  Serial.print(j);
  Serial.print("\tAirspeed Readings:");
  for (byte i = 0; i < 4; i++) {
    float reading = sensor[i].readSensorWindspeed();
    sensor[i].buildCartesianVector(reading);
  }
  float windSpeedVector[3];
  float windSpeedValue[1];
  computeGlobalWindspeed(windSpeedValue, windSpeedVector, &sensor[0], &sensor[1], &sensor[2], &sensor[3]);
  Serial.print("\tx: ");
  Serial.print(windSpeedVector[0]);
  Serial.print("\ty: ");
  Serial.print(windSpeedVector[1]);
  Serial.print("\tz: ");
  Serial.print(windSpeedVector[2]);
  Serial.println("");
  delay(100);
}