#ifndef BAROMETER_H
#define BAROMETER_H

#include "i2c.h"
#include "units.h"

class Barometer
{
public:
    Barometer(I2C& i2c);
    Meters getAltitude();
    Pascals getPressure();
    Celcius getTemp();
    void setActive();
    void waitForLaunch(float* buffer, uint8_t length);
    Milliseconds minInterval() const { return minSampleInterval; }
    bool calibrateStartingHeight();
    void setOverSamplingRatio(int ratio=0b10100000);
    // void setPressure(uint16_t p=101326);
    Meters startingHeight = 0.0;
    uint8_t getMode();

private:
    I2C& i2c;
    const uint8_t addr = 0x60;
    bool activated = false;
    const Milliseconds minSampleInterval = 66;
    const int16_t positiveOffset = 600;

    void setSleep();
    bool isDataReady();
    void initiateSample();
    float findRange(float* numbers, uint8_t length);
    float findAvg(float* numbers, uint8_t length);
};

#endif