#include <Arduino.h>
#include "nd005.hpp"
#include <SPI.h>
#include <Wire.h>
#include "main.h"

const uint8_t I2C_ADDRESS = 0x19;
const uint32_t MAIN_SDA = 2;
const uint32_t MAIN_SCL = 3;
MbedI2C main_i2c(MAIN_SDA, MAIN_SCL);

#ifdef ARDUINO_ARCH_MBED
   REDIRECT_STDOUT_TO(Serial)  // MBED  printf(...) to console
#endif

// Added "_USED" due to naming collision in library with just functional names
static const pin_size_t MISO_USED = 4;
static const pin_size_t MOSI_USED = 7;
static const pin_size_t SCK_USED = 6;
MbedSPI sensorSPI(MISO_USED, MOSI_USED, SCK_USED);

PressureSensor sensor[4] = {
  PressureSensor(17, 14, &sensorSPI, 0.00000000, 1.57079633), // deg:   0, 90 
  PressureSensor( 5, 15, &sensorSPI, 1.04719755, 1.57079633), // deg:  60, 90 
  PressureSensor( 9, 16, &sensorSPI, 2.09439510, 1.57079633), // deg: 120, 90 
  PressureSensor(13, 18, &sensorSPI, 0.00000000, 0.00000000)  // deg:   0,  0 
  };


void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    sensor[i].adjustRange(PressureRangeSettings::PSI05);
    sensor[i].calibrateZero(100);
  }

  Serial.begin(9600);

  while (!Serial) delay(10); // Wait until a PC is connected

  Serial.println("PEETOESTATIK CYSTDUMB\n");

  main_i2c.begin(I2C_ADDRESS);
  main_i2c.onRequest(send_i2c_response); // register event
}

int j = 0;

void loop() {
  // output all sensor readings
  float reading[4];
  for (byte i = 0; i < 4; i++) {
    reading[i] = sensor[i].readSensorWindspeed();
  }
  float windSpeedVector[3];
  float windSpeedValue[1];
  computeGlobalWindspeed(windSpeedValue, windSpeedVector, sensor);
  printf("Airspeed Readings: x: %6.2f y: %6.2f z: %6.2f\n", windSpeedVector[0], windSpeedVector[1], windSpeedVector[2]);
  printf("Sensor Readings (m/s): %6.2f | %6.2f | %6.2f | %6.2f\n\n", reading[0], reading[1], reading[2], reading[3]);
  delay(250);
}

void send_i2c_response() {
  send_float_i2c(23.5);     // 0x41BC_0000
  send_float_i2c(4983.976); // 0x459B_BFCF
  send_float_i2c(-5675.05); // 0xC5B1_5866

  /* NOTE:

  Something about this function seems to cause instability if the full 12 bytes are not
  read. The microcontroller seems to get stuck trying to transmit messing up the I2C bus.

  Might make sense in the future to opt for shorter exchanges where only a single axis is 
  commanded for at a time by the motherboard writing to this board first. Even if the bus 
  gets more congested overall.
  */
}

void send_float_i2c(float data) {
  char * dataPointer = (char *) & data;

  for (int i = 4; i > 0; i--) main_i2c.write(dataPointer[i-1]); // Most significant byte first
  // for (int i = 0; i < 4; i++) main_i2c.write(dataPointer[i]); // Least significant byte first
}