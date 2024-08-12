#ifndef ND005D_HEADER_H_
#define ND005D_HEADER_H_

#include <Arduino.h>
#include <SPI.h>

/* Pressure Enumerator

`PSI05` is 0.5 PSI, etc. Can't have punctuation in identifiers.

Enumerator are handy when dealing with values we only want to take certain 
values and nothing else. In this case we are using an enumerator to store 
the valid pressure settings for the sensor. 

We don't have to assign a type for the enumerator data itself but I chose
to set this enumerator value to be stored as an eight bit unsigned integer. 
Nor do we have to assign values for each option as I have, however it makes 
sense for us in a lot of our application when using these to describe 
registers to alight the options to their value, otherwise the compiler will 
assign whatever it feels like.
*/
enum PressureRangeSettings : uint8_t {
    PSI05 = 0b000,
    PSI08 = 0b011,
    PSI10 = 0b100,
    PSI20 = 0b101,
    PSI40 = 0b110,
    PSI50 = 0b111
};

enum PressureUnits : uint8_t {
    UNIT_PSI,
    UNIT_KPA,
    UNIT_PA
};

const float AIR_DENSITY = 1.225; // Air density in kg/m3

class PressureSensor {

    private:
        pin_size_t CS;
        pin_size_t DAV;
        pin_size_t RESET;
        PressureRangeSettings RANGE;
        MbedSPI * pressureSPI;

        uint8_t rateControl = 0x00;
        uint8_t modeControl = 0x46;

        static const SPISettings SPI_SETTINGS;
        static const int INITIAL_PAUSE_US;

        int16_t readingOffset = 0; // Stores calibrated zero point

        int16_t readRawPressure(bool waitNew);
    
    public:
        float spherical[3];
        float cartesian[3];
        PressureSensor(pin_size_t CSin, pin_size_t DAVin, MbedSPI * addressSPI, float THETA, float PHI);
        void setupSensor();
        float readPressure(enum PressureUnits unit);
        float readSensorWindspeed();
        int16_t readTemperature();
        void adjustRange(PressureRangeSettings newRange);
        void buildCartesianVector(float reading);
        void calibrateZero(int16_t samples);
};

void computeGlobalWindspeed(float* windSpeedValue, float* windSpeedVector, PressureSensor sensors[]);

extern const float PSI_TO_KPA;
extern const float PSI_TO_PA;

#endif