#include <Arduino.h>
#include <SPI.h>

#include "nd005.hpp"

const SPISettings PressureSensor::SPI_SETTINGS = SPISettings(1000000, BitOrder::MSBFIRST, SPIMode::SPI_MODE1); // SPI config for pressure sensors
const int PressureSensor::initialPause = 20; // Pause between SS goes low and transfer in us (100 is recommended)

/**
 * @note THETA and PHI values must be in RADIANS
*/
PressureSensor::PressureSensor(pin_size_t CSin, pin_size_t DAVin, MbedSPI * addressSPI, float THETA, float PHI) {
    CS = CSin;
    DAV = DAVin;
    pressureSPI = addressSPI;
    RANGE = PSI50;
    spherical[0] = 1.0;
    spherical[1] = THETA;
    spherical[2] = PHI;

    setupSensor();
}

void PressureSensor::setupSensor() {
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH); // Default to high since CS is active low

    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH); // Default to high since CS is active low

    pinMode(DAV, INPUT_PULLDOWN); // Pulldown to avoid mistaken highs

    pressureSPI->begin();

    adjustRange(PressureRangeSettings::PSI50); // Default to 5 psi range
}

/**
 * @name convertPressure
 * @brief converts pressure from raw sonsor output to prassure value based on selected range
 * @returns pressure in inches of H2O
*/

float PressureSensor::convertPressure(float rawReading, PressureRangeSettings RANGE) {
    float rangeValue = 0;
    if (RANGE == PSI05) {
        rangeValue = 0.5;
    } // replace this shit with a case switch thing for all the available range settings
    return ((rawReading / 29491.2) * (rangeValue)); // divide by 90% of 2^15 and multiply by range value
}

/**
 * @name readPressure
 * @brief Rreads pressure
 * @returns tpressure as float
*/

float PressureSensor::readPressure() {
    int16_t rawReading = 0;
    float pressureReading = 0;

    uint16_t combinedControl = (modeControl << 8) | rateControl; // *BITWISE* OR to combine bytes

    pressureSPI->beginTransaction(SPI_SETTINGS);
    digitalWrite(CS, LOW);
    delayMicroseconds(initialPause);

    rawReading = pressureSPI->transfer16(combinedControl);

    digitalWrite(CS, HIGH);
    pressureSPI->endTransaction();

    pressureReading = convertPressure(rawReading, RANGE);

    // Add the math to convert the integer from the sensor to a meaningful float here

    // return pressureReading;
    return pressureReading;
}

/**
 * @name readTemperature
 * @brief Rreads tem
 * @returns temperature as uint15=
*/
int16_t PressureSensor::readTemperature() {
    int16_t temperatureReading = 0;

    uint16_t combinedControl = (modeControl << 8) | rateControl;

    pressureSPI->beginTransaction(SPI_SETTINGS);
    digitalWrite(CS, LOW);
    delayMicroseconds(initialPause);

    pressureSPI->transfer16(combinedControl); // Discard the pressure reading that leads
    temperatureReading = pressureSPI->transfer16(0xCAFE);
    /* Extended Transfers
    As per section 10.5.2 of the ND005D manual, we don't need to send control values past the 
    first two bytes that are exchanged for pressure. Any data sent past these is discarded.
    */

    digitalWrite(CS, HIGH);
    pressureSPI->endTransaction();

    // Once again we need to add some math here

    return temperatureReading;
}

void PressureSensor::adjustRange(PressureRangeSettings newRange) {

    // dynamically adjust range
    // if measured values are less than or greater than a certain threshold
        // threshopld = a function of range maximum
    // then step up or step down to the next or pervious range setting

    const uint8_t PRESSUREMASK = 0x07; // Locations of range bits

    modeControl = modeControl & (~PRESSUREMASK); // Mask out (clear) the bits for the existing range
    modeControl = modeControl | newRange; // Set the new range's bits

    RANGE = newRange;
}

/**
 * @name buildVector
 * @brief builds vector output for each sensor in cartesian coordinates
 * @returns void, modifies object arrays
*/

void PressureSensor::buildVector(float reading) {
    // this fucntion will write to pre-initialized cartesian and spherical coordinate arrays within each sensor object.
    float r = reading;
    float theta = spherical[1];
    float phi = spherical[2];

    spherical[0] = r; // set reading as r in spherical coordinate array

    // converting spherical to cartesian
    cartesian[0] = r * sin(theta) * cos(phi); // cartesian x
    cartesian[1] = r * sin(theta) * sin(phi); // cartesian y
    cartesian[2] = r * cos(theta);            // cartesian z
    // NOTE: built-in trig functions take radian arguments
}
