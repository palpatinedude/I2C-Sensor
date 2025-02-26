// sensor.h (Base Sensor Class)
#ifndef SENSOR_H
#define SENSOR_H

#include "I2C.h"

struct SensorData {
    double distance;       // For ToF sensor
    double acceleration[3]; // For IMU sensor (x, y, z)
};

class Sensor {
public:
    virtual bool initialize() = 0;
    virtual bool readData(SensorData &data) = 0;
    virtual bool selfCheck() = 0;
    virtual bool deinitialize() = 0;
    virtual void handleSensorError(uint8_t errorCode) = 0;

    virtual bool configure() = 0;  // Configuration for sensor specific settings
    virtual void calibrate() = 0;  // 

    virtual ~Sensor() { deinitialize(); }

protected:
    HAL_I2C& _i2c;
    const uint8_t _address;
    bool _isCalibrated = false;

    Sensor(HAL_I2C& i2c, uint8_t address) : _i2c(i2c), _address(address) {}

    void setCalibrationStatus(bool status) {
        _isCalibrated = status;
    }
};
#endif // SENSOR_H


