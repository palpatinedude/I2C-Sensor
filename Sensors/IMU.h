#include "Sensor.h"

class BMI260 : public Sensor {
public:
    BMI260(HAL_I2C& i2c, uint8_t address = 0x68)
        : Sensor(i2c, address) {}

    bool initialize() override;  // Initialize the sensor
    bool readData(uint8_t *data, size_t length) override;  // Read accelerometer and gyroscope data
    bool writeData(uint8_t regAddr, uint8_t *data, size_t length) override;  // Write data to the sensor
    bool deinitialize() override;  // Reset the sensor

private:
    bool configure();
};

bool BMI260::initialize() {
    return configure();  // Initialize the sensor with custom configuration
}

bool BMI260::configure() {
    uint8_t data = 0x01;  // Example config value
    if (!writeData(0x40, &data, 1)) {
        return false;
    }
    return true;
}

bool BMI260::readData(uint8_t *data, size_t length) {
    return _i2c.read(_address, 0x12, data, length);  // Read data from accelerometer/gyroscope registers
}

bool BMI260::writeData(uint8_t regAddr, uint8_t *data, size_t length) {
    return _i2c.write(_address, regAddr, data, length);
}

bool BMI260::deinitialize() {
    uint8_t data = 0x00;  // Power down
    return writeData(0x7E, &data, 1);  // Example register to reset or deinitialize
}