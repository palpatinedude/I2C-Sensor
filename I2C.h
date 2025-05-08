/*
 * @file I2C.h
 * @brief Hardware Abstraction Layer for I2C communication on Arduino-compatible boards.
 
 * This header file defines the HAL_I2C class, which wraps the Arduino TwoWire interface
 * to provide a consistent, reusable, and configurable I2C interface with support for:
 *  - Custom SDA and SCL pins
 *  - Adjustable I2C frequency
 *  - Error handling and debugging messages
 *  - Device scanning and testing

 * It is designed to support multiple I2C bus instances and is especially useful in
 * STM32, ESP32, or other platforms where multiple I2C hardware buses may be available.
   
 */


#ifndef I2C_H
#define I2C_H

#include <Wire.h>
#include <Arduino.h>

// I2C Status Codes
#define SUCCESS 0
#define DATA_TOO_LONG 1
#define NACK_ON_ADDRESS 2
#define NACK_ON_DATA 3
#define OTHER_ERROR 4

// Debugging flag
#define HAL_I2C_DEBUG 1
#define DEBUG_PRINT(msg) if (HAL_I2C_DEBUG) Serial.println(msg)

// I2C Timeout
#define I2C_TIMEOUT_MS 100

#define SDA_PIN PB7
#define SCL_PIN PB6
#define I2C_FREQUENCY 100000
#define LCD_ADDRESS 0x27

#define MAX_I2C_BUSES 3

class HAL_I2C {
public:
    HAL_I2C(uint8_t busId, TwoWire &wireInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = I2C_FREQUENCY);
    void initialize();
    void setClock(uint32_t frequency);
    uint8_t write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
    uint8_t read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
    void scanBus();
    bool testDevice(uint8_t deviceAddr);
    void testDevices();
    void handleError(uint8_t errorCode);

private:
    uint8_t _busId;
    TwoWire &_wire;
    uint8_t _sda, _scl;
    uint32_t _frequency;
    unsigned long _startTime;
};

#endif // I2C_H

