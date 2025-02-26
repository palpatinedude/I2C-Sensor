#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include "I2C.h"

// Abstract class for all ToF sensors
class ToFSensor {
public:
    ToFSensor(HAL_I2C& i2c, uint8_t address) : _i2c(i2c), _address(address) {}

    // Pure virtual functions that derived classes must implement
    virtual bool initialize() = 0;
    virtual bool readDistance(uint16_t *distance) = 0;
    virtual bool deinitialize() = 0;

protected:
    HAL_I2C& _i2c;
    uint8_t _address; // I2C address for the sensor
};

#endif // TOF_SENSOR_H
