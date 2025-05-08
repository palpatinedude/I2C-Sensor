# HAL_I2C - Hardware Abstraction Layer for I2C Communication

## Description

This library provides a lightweight and flexible Hardware Abstraction Layer (HAL) for I2C communication using the Arduino `Wire` library. It supports multiple I2C buses, customizable pin assignments, adjustable clock speed, error handling, and device scanning. Ideal for projects using microcontrollers like STM32, ESP32, or AVR boards with multiple I2C interfaces.

---

## Features

- Initialize and configure I2C buses with custom SDA, SCL pins, and clock frequency
- Read/write to I2C devices with timeout handling
- Scan for connected I2C devices
- Error handling and debug logging via `Serial`
- Support for multiple I2C bus instances

---

## Files

- `I2C.h` — Header file containing the `HAL_I2C` class definition and constants.
- `I2C.cpp` — Implementation of the `HAL_I2C` class functions.

---
