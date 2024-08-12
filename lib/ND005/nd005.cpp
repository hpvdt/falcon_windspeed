#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <SPI.h>

#include "nd005.hpp"

const SPISettings PressureSensor::SPI_SETTINGS = SPISettings(1000000, BitOrder::MSBFIRST, SPIMode::SPI_MODE1); // SPI config for pressure sensors
const int PressureSensor::INITIAL_PAUSE_US = 20; // Pause between SS goes low and transfer (100 is recommended)

const float PSI_TO_KPA = 6.89476;               // Multiply psi by this to get kpa 
const float PSI_TO_PA = PSI_TO_KPA * 1000.0;    // Multiply psi by this to get pa

const bool DAV_ACTIVE = true; // What is the active state of the DAV line for the ND005

/**
 * @note THETA and PHI values must be in RADIANS
*/
PressureSensor::PressureSensor(pin_size_t CSin, pin_size_t DAVin, MbedSPI * addressSPI, float THETA, float PHI) {
    CS = CSin;
    DAV = DAVin;
    pressureSPI = addressSPI;
    RANGE = PSI05;
    spherical[0] = 1.0;
    spherical[1] = THETA;
    spherical[2] = PHI;

    setupSensor();
}


/**
 * @name setupSensor
 * @brief initializes pinmodes, sets default range, this function runs automatically when each object is initialized
 * @returns void, writing internal or pre-initialized registers
*/
void PressureSensor::setupSensor() {
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH); // Default to high since CS is active low

    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH); // Default to high since CS is active low

    pinMode(DAV, INPUT_PULLDOWN); // Pulldown to avoid mistaken highs

    pressureSPI->begin();

    adjustRange(PressureRangeSettings::PSI05); // Default to 5 psi range
}


/**
 * \brief Gets a new pressure reading for a given sensor
 * 
 * \param waitNew Do we wait unit a new value is available (as indicated by DAV pin)
 * \return Raw preassure reading from the sensor
 */
int16_t PressureSensor::readRawPressure(bool waitNew) {
    static int16_t reading = 0; // Use static so we can repeat values if needed

    do {
        if (digitalRead(DAV) == DAV_ACTIVE) {
            waitNew = false; // Indicate new data was recieved

            uint16_t combinedControl = (modeControl << 8) | rateControl; // *BITWISE* OR to combine bytes

            pressureSPI->beginTransaction(SPI_SETTINGS);
            digitalWrite(CS, LOW);
            delayMicroseconds(INITIAL_PAUSE_US);

            reading = pressureSPI->transfer16(combinedControl);

            digitalWrite(CS, HIGH);
            pressureSPI->endTransaction();
        }
    } while (waitNew == true);

    return reading;
}


/**
 * \brief Gets sensor output to pressure value based on sensor's active range
 * 
 * \param unit Desired unit for returned pressure value
 * \return Pressure on sensor in desired unit 
 */
float PressureSensor::readPressure(enum PressureUnits unit) {
    int16_t rawReading = readRawPressure(false); // Get new value
    // Don't block for new value

    int32_t reading = rawReading - readingOffset;

    float rangeScale = 0;
    switch (RANGE) {   
        case PSI05:
            rangeScale = 0.5;
            break;
        case PSI08:
            rangeScale = 0.8;
            break;
        case PSI10:
            rangeScale = 1.0;
            break;
        case PSI20:
            rangeScale = 2.0;
            break;
        case PSI40:
            rangeScale = 4.0;
            break;
        case PSI50:
            rangeScale = 5.0;
            break;
    }

    // Divide by 90% of 2^15 and multiply by range value, as per data sheet
    float psi = (reading / 29491.2) * (rangeScale);

    float unitScale = 0;
    switch (unit) {
    case UNIT_KPA:
        unitScale = PSI_TO_KPA;
        break;
    case UNIT_PA:
        unitScale = PSI_TO_PA;
        break;
    default:
        unitScale = 1.0;
        break;
    }
    float final = unitScale * psi;

    // printf("RAW:%7d ADJ:%7ld PSI:%9.4f FINAL:%9.4f\n", rawReading, reading, psi, final);

    return (final); 
}


/**
 * @name readSensorWindspeed
 * @brief reads sensor, converts to pressure, converts to windspeed
 * @returns float for winspeed, scalar value
*/
float PressureSensor::readSensorWindspeed() {
    float diff_pressure = readPressure(UNIT_PA);
    float wind_speed = (2 * fabsf(diff_pressure));
    wind_speed = wind_speed / AIR_DENSITY;
    wind_speed = sqrt(wind_speed);
    if (diff_pressure < 0.0) wind_speed = -wind_speed;
    return wind_speed;
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
    delayMicroseconds(INITIAL_PAUSE_US);

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


/**
 * @name adjustRange
 * @brief adjusts the range setting of the ND005 using the PressureRangeSettings enumeration
 * @returns void, modifies modecontrol byte
*/
void PressureSensor::adjustRange(PressureRangeSettings newRange) {

    // dynamically adjust range
    // if measured values are less than or greater than a certain threshold
        // threshopld = a function of range maximum
    // then step up or step down to the next or pervious range setting

    const uint8_t PRESSUREMASK = 0x07; // Locations of range bits

    modeControl = modeControl & (~PRESSUREMASK); // Mask out (clear) the bits for the existing range
    modeControl = modeControl | newRange; // Set the new range's bits

    RANGE = newRange;

    // Read from the sensor to provide it the command bytes for the new range for future readings
    // Needs to happen twice to properly take effect as per datasheet
    readRawPressure(true);
    readRawPressure(true);
}


/**
 * @name buildVector
 * @brief builds vector output for each sensor in cartesian coordinates
 * @returns void, modifies object arrays
*/
void PressureSensor::buildCartesianVector(float sensorWindspeed) {
    // this fucntion will write to pre-initialized cartesian and spherical coordinate arrays within each sensor object.
    float r = sensorWindspeed;
    float theta = spherical[1];
    float phi = spherical[2];

    spherical[0] = r; // set reading as r in spherical coordinate array

    // converting spherical to cartesian
    cartesian[0] = r * sin(theta) * cos(phi); // cartesian x
    cartesian[1] = r * sin(theta) * sin(phi); // cartesian y
    cartesian[2] = r * cos(theta);            // cartesian z
    // NOTE: built-in trig functions take radian arguments
}


/**
 * @name windSpeed
 * @brief copmiles the vector form of each sensor reading into a single 3D vector in cartesian representing aircraft windspeed and direction
 * @returns void, modifies pre-initialized array, writes global 
 * @note MUST declare array[3] destination before function call in main to hold windSpeed measurement
*/
void computeGlobalWindspeed(float* globalWindspeedValue, float* globalWindspeedVector, PressureSensor* sensor1, PressureSensor* sensor2, PressureSensor* sensor3, PressureSensor* sensor4) {
    // this function will perform vector addition on all 4 pressure sensor readings and output overall windspeed and direction 

    // Read pressure and build vectors for each sensor
    /*
    sensor1->buildCartesianVector(sensor1->readPressure());
    sensor2->buildCartesianVector(sensor2->readPressure());
    sensor3->buildCartesianVector(sensor3->readPressure());
    sensor4->buildCartesianVector(sensor4->readPressure());
    */

   // LOADING VECTORS INTO VARIABLES

   // sensor 1 pressure reading in cartesian
   float x1 = sensor1->cartesian[0];
   float y1 = sensor1->cartesian[1];
   float z1 = sensor1->cartesian[2];
   
   // sensor 2 pressure reading in cartesian
   float x2 = sensor2->cartesian[0];
   float y2 = sensor2->cartesian[1];
   float z2 = sensor2->cartesian[2];
   
   // sensor 3 pressure reading in cartesian
   float x3 = sensor3->cartesian[0];
   float y3 = sensor3->cartesian[1];
   float z3 = sensor3->cartesian[2];
   
   // sensor 4 pressure reading in cartesian
   float x4 = sensor4->cartesian[0];
   float y4 = sensor4->cartesian[1];
   float z4 = sensor4->cartesian[2];
   
   // pressure vector in cartesian
   float x = x1 + x2 + x3 + x4;
   float y = y1 + y2 + y3 + y4;
   float z = z1 + z2 + z3 + z4;
   
   // write new [x, y, z] values to windSpeed array
   globalWindspeedVector[0] = x;
   globalWindspeedVector[1] = y;
   globalWindspeedVector[2] = z;
   
   // compute ||windSpeedVector|| and write to windSpeedValue
   globalWindspeedValue[0] = sqrt((x * x) + (y * y) + (z * z));
}


/**
 * \brief Calibrates the pressure sensor's zero offset
 * \note This is currently set to match the present pressure range, so be careful changing pressure
 * 
 * \param samples Number of samples to average for the zero point (0 to reset zero point to 0)
 */
void PressureSensor::calibrateZero(int16_t samples) {
    int32_t sumOfReadings = 0;

    // Reset zero point and return to avoid division by zero later
    if (samples == 0) {
        readingOffset = 0;
        return;
    }

    // Collect the required number of unique readings
    for (uint16_t i = 0; i < samples; i++) {
        int16_t reading = readRawPressure(true);
        sumOfReadings = sumOfReadings + reading;
    }

    readingOffset = sumOfReadings / samples;
}