// I2C HAL initial based in arduino framework with stm32H437 .
// I2C  serial communication protocol to connect sensors ;; for microcontrollers .
// I2C works on two wires: SDA (Serial Data) and SCL (Serial Clock) .
// This protocol works in a master/slave setup, where the master initiates the communication and the slave responds to the master’s requests .

// dma(future) vs pooling(start)??
//  2 diffrent i2c buses for tof and imu multiplexer or multiple  i2c buses?????
// stm32h743 3 i2c buses?? one for tofs one for imu??
// test device function to check if the device is connected to the bus and responding to the master’s requests??
// baudrate,frequency???
// future "stm32f1xx_hal.h"?
// _wire.begin(_sda, _scl);  STM32 doesn't support this in Arduino core
// address conflict???
#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <Wire.h>
#include <Arduino.h>
//#include "stm32h7xx_hal.h"

// I2C Status Codes
#define SUCCESS 0
#define DATA_TOO_LONG 1
#define NACK_ON_ADDRESS 2
#define NACK_ON_DATA 3
#define OTHER_ERROR 4
#define I2C_FREQUENCY 100000  // I2C frequency in Hz

// Debugging flag
#define HAL_I2C_DEBUG 1  // Enable/Disable debugging output

// Default I2C Pins (can be overridden per instance)
#define SDA_PIN PB7
#define SCL_PIN PB6

// Debugging macros
#if HAL_I2C_DEBUG
    #define DEBUG_PRINT(msg) Serial.println(msg)
#else
    #define DEBUG_PRINT(msg) 
#endif

class HAL_I2C {
public:
  //  HAL_I2C(TwoWire &wireInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = 100000);
    HAL_I2C(uint8_t busId, TwoWire &wireInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = I2C_FREQUENCY);
    //HAL_I2C(I2C_TypeDef *i2cInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = 100000);
    
    void initialize();                  // Initialize I2C bus
    void setClock(uint32_t frequency);  // Set I2C clock speed
    uint8_t write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);  // Write data
    uint8_t read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length); // Read data
    void scanBus();                     // Scan all devices on I2C bus
    void testDevices();                // Test all devices on I2C bus
    bool testDevice(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedStatus); // Test I2C device

    void handleError(uint8_t errorCode); // Handle I2C errors

    // DMA Support (non-blocking)
  //  bool writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
  //  bool readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
    
    // DMA Interrupt Handler (optional)
  //  void DMA1_Stream6_IRQHandler(void); // TX Interrupt
  //  void DMA1_Stream0_IRQHandler(void); // RX Interrupt

    void handleError(uint8_t errorCode); // Handle I2C errors

    // DMA Support (non-blocking)
  //  bool writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
  //  bool readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
    
    // DMA Interrupt Handler (optional)
  //  void DMA1_Stream6_IRQHandler(void); // TX Interrupt
  //  void DMA1_Stream0_IRQHandler(void); // RX Interrupt

    // DMA Support (non-blocking )
 //   bool writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
  //  bool readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);

private:
    TwoWire &_wire;  // I2C instance 
    uint8_t _sda, _scl;  // SDA and SCL pins
    uint32_t _frequency;  // I2C frequency

    // DMA Handles for TX and RX
 //   DMA_HandleTypeDef hdma_i2c_tx;
 //   DMA_HandleTypeDef hdma_i2c_rx;

 //   I2C_HandleTypeDef hi2c1;                  // I2C handle

  //  void configureDMA();                      // Configure DMA for I2C
  //  void setupI2C();                          // Setup I2C peripheral
  //  void initPins();                          // Initialize the I2C pins

   // I2C_HandleTypeDef _hi2c;  // I2C handler for STM32
  //  I2C_TypeDef *_i2cInstance;  // I2C instance 
  //  void initPins();

};

#endif // HAL_I2C_H
