/**
 * @file I2C.cpp
 * @brief Implementation of the HAL_I2C class for managing I2C communication.
 
 * This source file provides the implementation of the HAL_I2C class methods defined in I2C.h.
 * It enables hardware abstraction for I2C communication using the Arduino Wire library, and
 * supports multiple I2C buses with configurable pins and clock frequency.
 
 * Core features include:
 *  - Initialization of I2C hardware
 *  - Reading and writing to I2C devices
 *  - I2C device scanning and identification
 *  - Error handling and timeout management
 *  - Debug message output via Serial
 
 * This implementation is suitable for use on platforms such as STM32, ESP32, and AVR,
 * where multiple I2C channels and flexible pin assignments are supported.
 */
#include "I2C.h"

HAL_I2C::HAL_I2C(uint8_t busId, TwoWire &wireInstance, uint8_t sda, uint8_t scl, uint32_t frequency)
    : _busId(busId), _wire(wireInstance), _sda(sda), _scl(scl), _frequency(frequency) {}

void HAL_I2C::initialize() {
    _wire.begin();
    _wire.setClock(_frequency);
    DEBUG_PRINT("I2C Initialized on Bus " + String(_busId));
}

void HAL_I2C::setClock(uint32_t frequency) {
    _frequency = frequency;
    _wire.setClock(_frequency);
    DEBUG_PRINT("I2C Clock Updated on Bus " + String(_busId));
}

uint8_t HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    if (!data) {
        handleError(OTHER_ERROR);
        return OTHER_ERROR;
    }

    _wire.beginTransmission(deviceAddr);
    _wire.write(regAddr);
    _wire.write(data, length);

    _startTime = millis();
    while (_wire.endTransmission() != SUCCESS) {
        if (millis() - _startTime > I2C_TIMEOUT_MS) {
            handleError(OTHER_ERROR);
            return OTHER_ERROR;
        }
    }

    return SUCCESS;
}

uint8_t HAL_I2C::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    if (!buffer) {
        handleError(OTHER_ERROR);
        return OTHER_ERROR;
    }

    _wire.beginTransmission(deviceAddr);
    _wire.write(regAddr);
    if (_wire.endTransmission(false) != SUCCESS) {
        handleError(OTHER_ERROR);
        return OTHER_ERROR;
    }

    _startTime = millis();
    _wire.requestFrom(deviceAddr, (uint8_t)length);

    size_t received = 0;
    while (_wire.available() && received < length) {
        buffer[received++] = _wire.read();
        if (millis() - _startTime > I2C_TIMEOUT_MS) {
            handleError(OTHER_ERROR);
            return OTHER_ERROR;
        }
    }

    return (received == length) ? SUCCESS : OTHER_ERROR;
}

void HAL_I2C::scanBus() {
    Serial.println("\nScanning I2C Bus...");
    uint8_t deviceCount = 0;
    for (uint8_t address = 1; address < 127; address++) {
        _wire.beginTransmission(address);
        if (_wire.endTransmission() == SUCCESS) {
            Serial.print("0x"); Serial.print(address, HEX); Serial.print(" ");
            deviceCount++;
        }
    }
    Serial.println(deviceCount ? "\nScan Complete." : "No I2C devices found!");
}

bool HAL_I2C::testDevice(uint8_t deviceAddr) {
    _wire.beginTransmission(deviceAddr);
    if (_wire.endTransmission() != SUCCESS) return false;
    Serial.print("Device found at 0x"); Serial.println(deviceAddr, HEX);
    return true;
}

void HAL_I2C::testDevices() {
    Serial.println("\nTesting I2C devices...");
    for (uint8_t address = 1; address < 127; address++) {
        if (testDevice(address)) {
            Serial.print("Device at address 0x");
            Serial.println(address, HEX);
        }
    }
}

void HAL_I2C::handleError(uint8_t errorCode) {
    const char* errorMsg[] = {"Success", "Data Too Long", "NACK on Address", "NACK on Data", "Other Error"};
    Serial.print("I2C Error on Bus ");
    Serial.print(_busId);
    Serial.print(": ");
    Serial.println(errorMsg[errorCode]);
}


