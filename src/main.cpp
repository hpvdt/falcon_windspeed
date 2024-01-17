#include <Arduino.h>
#include "nd005.hpp"
#include <SPI.h>

class PressureSensor {

  private:
    pin_size_t CS;
    pin_size_t DAV;
    pin_size_t RESET;

    static uint8_t rateControl = 0x00;
    static uint8_t modeControl = 0xF6;

    static const SPISettings SPI_SETTINGS = SPISettings(1000000, BitOrder::MSBFIRST, SPIMode::SPI_MODE1);
    static const int initialPause = 20; // Pause between SS goes low and transfer in us (100 is recommended)

    // Added "_USED" due to naming collision in library with just functional names
    static const PinName MISO_USED   = digitalPinToPinName(4);
    static const PinName MOSI_USED   = digitalPinToPinName(7);
    static const PinName SCK_USED    = digitalPinToPinName(6);

    static MbedSPI pressureSPI(MISO_USED, MOSI_USED, SCK_USED);
  
  public:
    PressureSensor(pin_size_t CSin, pin_size_t DAVin) {
      CS = CSin;
      DAV = DAVin;

      setupSensor(CS, DAV, RESET);
    }
    
    void setupSensor() {
      pinMode(CS, OUTPUT);
      digitalWrite(CS, HIGH); // Default to high since CS is active low

      pinMode(RESET, OUTPUT);
      digitalWrite(RESET, HIGH); // Default to high since CS is active low

      pinMode(DAV, INPUT_PULLDOWN); // Pulldown to avoid mistaken highs

      // Just showing this off limiting inputs
      adjustRange(PressureRangeSettings::PSI20);
      //adjustRange(0b101); // This won't compile, even if valid value for the enum
    }

    int16_t readPressure() {
      int16_t pressureReading = 0;

      uint16_t combinedControl = (modeControl << 8) | rateControl; // *BITWISE* OR to combine bytes

      pressureSPI.beginTransaction(SPI_SETTINGS);
      digitalWrite(CS, LOW);
      delayMicroseconds(initialPause);

      pressureReading = pressureSPI.transfer16(combinedControl);

      digitalWrite(CS, HIGH);
      pressureSPI.endTransaction();

      // Add the math to convert the integer from the sensor to a meaningful float here

      return pressureReading;
    }


    int16_t readTemperature(uint8_t CS) {
      int16_t temperatureReading = 0;

      uint16_t combinedControl = (modeControl << 8) | rateControl;

      pressureSPI.beginTransaction(SPI_SETTINGS);
      digitalWrite(CS, LOW);
      delayMicroseconds(initialPause);

      pressureSPI.transfer16(combinedControl); // Discard the pressure reading that leads
      temperatureReading = pressureSPI.transfer16(0xCAFE);
      /* Extended Transfers
      As per section 10.5.2 of the ND005D manual, we don't need to send control values past the 
      first two bytes that are exchanged for pressure. Any data sent past these is discarded.
      */

      digitalWrite(CS, HIGH);
      pressureSPI.endTransaction();

      // Once again we need to add some math here

      return temperatureReading;
    }

    void adjustRange(PressureRangeSettings newRange) {
      const uint8_t PRESSUREMASK = 0x07; // Locations of range bits

      modeControl = modeControl & (~PRESSUREMASK); // Mask out (clear) the bits for the existing range
      modeControl = modeControl | newRange; // Set the new range's bits
    }
};

PressureSensor sensor[4] = {
  PressureSensor(17, 14), 
  PressureSensor(5, 15), 
  PressureSensor(9, 16), 
  PressureSensor(13, 18)
  };

// Added "_USED" due to naming collision in library with just functional names
static const pin_size_t MISO_USED = 4;
static const pin_size_t MOSI_USED = 7;
static const pin_size_t SCK_USED = 6;

void setup() {
  adjustRange(PressureRangeSettings::PSI05); // Note this applies to all sensors!

  Serial.begin(9600);

  while (!Serial) delay(10); // Wait until a PC is connected

  Serial.println("PEETOESTATIK CYSTDUMB\n");

}

void loop() {

  Serial.print("Pressure readings");
  for (byte i = 0; i < 4; i++) {
    Serial.print(" ");
    Serial.print(readPressure(sensor[i].CS));
  }
  Serial.println("");
  delay(100);
}
