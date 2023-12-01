#include <Arduino.h>
#include <SPI.h>

#include "nd005.hpp"

// Added "_USED" due to naming collision in library with just functional names
static const pin_size_t MISO_USED   = 7;
static const pin_size_t MOSI_USED   = 4;
static const pin_size_t SCK_USED    = 6;

/* SPI Settings for Pressure sensors

Derived from datasheet and referenceing the WikiPedia page on SPI to determine SPI mode
https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase

A bit surprized the minimum clock period is 6us, thus max clock is 166 666 Hz.

`static const` so it isn't visible outside of this file pair (`.cpp` and `.hpp`)
*/
static const SPISettings SPI_SETTINGS = SPISettings(166666, MSBFIRST, SPIMode::SPI_MODE1);

// Variables to store control register values, initialized to device defaults from datasheet
static uint8_t rateControl = 0x00; 
static uint8_t modeControl = 0xF6;

void setupSensor(uint8_t CS, uint8_t DAV, uint8_t RESET) {
    // `setXX` must be called before `begin`
    SPI.setRX(MISO_USED);
    SPI.setSCK(SCK_USED);
    SPI.setTX(MOSI_USED);
    SPI.begin(false); // We will manually actuate CS, hence the `false`
    // This SPI config really only needs to be run once, but repeated calls probably won't cause issues

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH); // Default to high since CS is active low

    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH); // Default to high since CS is active low

    pinMode(DAV, INPUT_PULLDOWN); // Pulldown to avoid mistaken highs

    // Just showing this off limiting inputs
    adjustRange(PressureRangeSettings::PSI20); 
    //adjustRange(0b101); // This won't compile, even if valid value for the enum
}

int16_t readPressure(uint8_t CS) {
    int16_t pressureReading = 0;

    uint16_t combinedControl = (modeControl << 8) | rateControl; // *BITWISE* OR to combine bytes

    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(CS, LOW);

    pressureReading = SPI.transfer16(combinedControl);

    digitalWrite(CS, HIGH);
    SPI.endTransaction();

    // Add the math to convert the integer from the sensor to a meaningful float here

    return pressureReading;
}

int16_t readTemperature(uint8_t CS) {
    int16_t temperatureReading = 0;

    uint16_t combinedControl = (modeControl << 8) | rateControl;

    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(CS, LOW);

    SPI.transfer16(combinedControl); // Discard the pressure reading that leads
    temperatureReading = SPI.transfer16(0xCAFE);
    /* Extended Transfers
    As per section 10.5.2 of the ND005D manual, we don't need to send control values past the 
    first two bytes that are exchanged for pressure. Any data sent past these is discarded.
    */

    digitalWrite(CS, HIGH);
    SPI.endTransaction();

    // Once again we need to add some math here

    return temperatureReading;
}

void adjustRange(PressureRangeSettings newRange) {
    const uint8_t PRESSUREMASK = 0x07; // Locations of range bits

    modeControl = modeControl & (~PRESSUREMASK); // Mask out (clear) the bits for the existing range
    modeControl = modeControl | newRange; // Set the new range's bits
}
