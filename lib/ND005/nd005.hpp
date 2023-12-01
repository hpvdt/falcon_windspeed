#ifndef ND005D_HEADER_H_
#define ND005D_HEADER_H_

#include <Arduino.h>

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

void setupSensor(uint8_t CS, uint8_t DAV, uint8_t RESET);
int16_t readPressure(uint8_t CS);
int16_t readTemperature(uint8_t CS);
void adjustRange(PressureRangeSettings newRange);

#endif