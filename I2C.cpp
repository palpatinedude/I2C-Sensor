#include "I2C.h"
/*
// Constructor for I2C initialization with custom parameters
HAL_I2C::HAL_I2C(TwoWire &wireInstance, uint8_t sda, uint8_t scl, uint32_t frequency)
    : _wire(wireInstance), _sda(sda), _scl(scl), _frequency(frequency) {
    // Initialize the I2C bus
    initialize();
}
*/
// Initialize the I2C bus
void HAL_I2C::initialize() {
    _wire.begin();      // Start I2C with SDA and SCL pins
    _wire.setClock(_frequency);   // Set clock speed
    DEBUG_PRINT("I2C Initialized");
}

// Set the I2C clock speed
void HAL_I2C::setClock(uint32_t frequency) {
    _frequency = frequency;       // Set new frequency
    _wire.setClock(_frequency);   // Apply clock speed
    DEBUG_PRINT("I2C Clock Updated");
}

// --- WRITE OPERATION ---
uint8_t HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    if (!data) {
        DEBUG_PRINT("I2C Write Failed: NULL Data Pointer");
        return OTHER_ERROR;
    }

    _wire.beginTransmission(deviceAddr);
    _wire.write(regAddr);
    _wire.write(data, length);
    uint8_t result = _wire.endTransmission();
    
    if (result == SUCCESS) {
        DEBUG_PRINT("I2C Write Success");
    } else {
        handleError(result);
    }
    return result;
}

// --- READ OPERATION ---
uint8_t HAL_I2C::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    if (!buffer) {
        DEBUG_PRINT("I2C Read Failed: NULL Buffer Pointer");
        return OTHER_ERROR;
    }

    _wire.beginTransmission(deviceAddr);
    _wire.write(regAddr);
    if (_wire.endTransmission(false) != SUCCESS) {
        DEBUG_PRINT("I2C Read Request Failed");
        return OTHER_ERROR;
    }

    _wire.requestFrom(deviceAddr, (uint8_t)length);
    unsigned long startTime = millis();
    for (size_t i = 0; i < length; i++) {
        while (!_wire.available()) {
            if (millis() - startTime > I2C_TIMEOUT) {
                DEBUG_PRINT("I2C Read Timeout");
                return I2C_TIMEOUT;
            }
        }
        buffer[i] = _wire.read();
    }
    DEBUG_PRINT("I2C Read Success");
    return SUCCESS;
}

// --- SCAN I2C BUS ---
void HAL_I2C::scanBus() {
    Serial.println("\nScanning I2C Bus...");
    
    bool deviceFound = false;
    uint8_t deviceCount = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        _wire.beginTransmission(address);
        if (_wire.endTransmission() == SUCCESS) {
            if (!deviceFound) Serial.print("Devices found at: ");
            Serial.print("0x");
            Serial.print(address, HEX);
            Serial.print(" ");
            deviceFound = true;
            deviceCount++;
        }
    }

    if (!deviceFound) {
        Serial.println("No I2C devices found!");
    } else {
        Serial.println();  
        Serial.print("Total I2C devices found: ");
        Serial.println(deviceCount);
    }

    Serial.println("I2C scan complete.\n");
}

// --- DEVICE TEST FUNCTION ---
void HAL_I2C::testDevices() {
    Serial.println("\nTesting I2C devices...");
    bool deviceFound = false;

    for (uint8_t address = 1; address < 127; address++) {
        _wire.beginTransmission(address);
        if (_wire.endTransmission() == SUCCESS) {
            Serial.print("\nDevice found at 0x");
            Serial.println(address, HEX);
            deviceFound = true;

            // Handle Specific Devices
            switch (address) {
                case 0x27:
                case 0x3F:
                    Serial.println("  -> LCD detected, testing...");
                    // testLCD(address);
                    break;
                case 0x29:
                    Serial.println("  -> ToF Sensor detected, reading distance...");
                    // readToFSensor(address);
                    break;
                case 0x68:
                case 0x69:
                    Serial.println("  -> IMU (MPU6050/MPU9250) detected, testing...");
                    // testIMU(address);
                    break;
                default:
                    Serial.println("  -> Unknown device, reading register 0x00...");
                    // testDevice(address, 0x00, 0x01); // Try reading first register
                    break;
            }
        }
    }
}

// --- ERROR HANDLING FUNCTION ---
void HAL_I2C::handleError(uint8_t errorCode) {
    switch (errorCode) {
        case SUCCESS:
            DEBUG_PRINT("I2C Operation Successful");
            break;
        case DATA_TOO_LONG:
            DEBUG_PRINT("I2C Error: Data Too Long");
            break;
        case NACK_ON_ADDRESS:
            DEBUG_PRINT("I2C Error: No Acknowledgement from Slave (Address)");
            break;
        case NACK_ON_DATA:
            DEBUG_PRINT("I2C Error: No Acknowledgement from Slave (Data)");
            break;
        case OTHER_ERROR:
            DEBUG_PRINT("I2C Error: Other Error");
            break;
        default:
            DEBUG_PRINT("I2C Unknown Error");
            break;
    }
}

#define I2C_FREQUENCY 100000  // I2C clock speed (100kHz)

// Define the I2C device address (16x2 LCD usually uses 0x27 or 0x3F)
#define LCD_ADDRESS 0x27
#define SDA_PIN A4     // SDA pin
#define SCL_PIN A5     // SCL pin

// Initialize HAL_I2C instance
HAL_I2C i2c(Wire, SDA_PIN, SCL_PIN, I2C_FREQUENCY);

void setup() {

    /*
    // Start serial communication for debugging
    Serial.begin(115200);
    delay(1000);  // Wait for Serial Monitor to open

    // Initialize the I2C bus
    i2c.initialize();
    
    // Test scanning the I2C bus
    i2c.scanBus();  // This will print all detected I2C devices on the bus

    // Test if a specific I2C device (e.g., LCD) is working
    if (i2c.testDevice(LCD_ADDRESS, 0x00, 0x01)) {
        Serial.println("I2C Device (LCD) found and responded correctly.");
    } else {
        Serial.println("I2C Device (LCD) test failed.");
    }

    // Write data to a known I2C device
    uint8_t data[] = {0x01, 0x02, 0x03};
    uint8_t result = i2c.write(LCD_ADDRESS, 0x00, data, sizeof(data));

    if (result == SUCCESS) {
        Serial.println("I2C Write Successful.");
    } else {
        Serial.println("I2C Write Failed.");
    }

    // Read data from a known I2C device
    uint8_t buffer[3];
    result = i2c.read(LCD_ADDRESS, 0x00, buffer, sizeof(buffer));

    if (result == SUCCESS) {
        Serial.print("I2C Read Successful. Data: ");
        for (size_t i = 0; i < sizeof(buffer); i++) {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("I2C Read Failed.");
    }*/
}

void loop() {
    // Optionally, keep doing something in the loop
    // For now, the I2C test is complete in the setup
}



/*
// --- DMA SUPPORT PLACEHOLDERS ---
bool HAL_I2C::writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    // Placeholder for DMA implementation
    DEBUG_PRINT("I2C DMA Write Not Implemented");
    return false;
}

bool HAL_I2C::readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    // Placeholder for DMA implementation
    DEBUG_PRINT("I2C DMA Read Not Implemented");
    return false;
}
*
/*
// --- DMA WRITE OPERATION ---
bool HAL_I2C::writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    if (data == nullptr) {
        DEBUG_PRINT("I2C DMA Write Failed: NULL Data Pointer");
        return false;
    }

    uint8_t buffer[length + 1];
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, length);

    // Perform DMA write (non-blocking)
    if (HAL_I2C_Master_Transmit_DMA(&_wire, deviceAddr << 1, buffer, length + 1) == HAL_OK) {
        DEBUG_PRINT("I2C DMA Write Success");
        return true;
    }

    DEBUG_PRINT("I2C DMA Write Failed");
    return false;
}

// --- DMA READ OPERATION ---
bool HAL_I2C::readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    // Perform DMA read (non-blocking)
    if (HAL_I2C_Mem_Read_DMA(&_wire, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, length) == HAL_OK) {
        DEBUG_PRINT("I2C DMA Read Success");
        return true;
    }

    DEBUG_PRINT("I2C DMA Read Failed");
    return false;
}




// Constructor to initialize I2C with custom SDA/SCL pins
HAL_I2C::HAL_I2C(I2C_TypeDef *i2cInstance, uint8_t sda, uint8_t scl, uint32_t frequency)
    : _i2cInstance(i2cInstance), _sda(sda), _scl(scl), _frequency(frequency) {
    // Initialize I2C peripheral
    initialize();
}

// Initialize I2C with specified settings
void HAL_I2C::initialize() {
    // Initialize the I2C pins
    initPins();

    // Initialize the I2C peripheral (using STM32 HAL library)
    _hi2c.Instance = _i2cInstance;
    _hi2c.Init.ClockSpeed = _frequency;
    _hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;  // 50% duty cycle
    _hi2c.Init.OwnAddress1 = 0;               // Default address
    _hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    // Initialize I2C
    if (HAL_I2C_Init(&_hi2c) != HAL_OK) {
        DEBUG_PRINT("I2C initialization failed!");
        // Handle error
    }
}

// Set I2C clock frequency
void HAL_I2C::setClock(uint32_t frequency) {
    _frequency = frequency;
    _hi2c.Init.ClockSpeed = frequency;
    if (HAL_I2C_Init(&_hi2c) != HAL_OK) {
        DEBUG_PRINT("I2C clock setting failed!");
    }
}

// Write data to an I2C device
bool HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&_hi2c, deviceAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, 1000);
    return status == HAL_OK;
}

// Read data from an I2C device
bool HAL_I2C::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&_hi2c, deviceAddr, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, length, 1000);
    return status == HAL_OK;
}

// Scan the I2C bus for devices
void HAL_I2C::scanBus() {
    for (uint8_t i = 1; i < 128; ++i) {
        if (testDevice(i)) {
            DEBUG_PRINT("Device found at address: " + String(i, HEX));
        }
    }
}

// Test I2C device presence
bool HAL_I2C::testDevice(uint8_t deviceAddr) {
    uint8_t testData;
    return read(deviceAddr, 0x00, &testData, 1);
}

// DMA Support (non-blocking write)
bool HAL_I2C::writeDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(&_hi2c, deviceAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, length);
    return status == HAL_OK;
}

// DMA Support (non-blocking read)
bool HAL_I2C::readDMA(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&_hi2c, deviceAddr, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, length);
    return status == HAL_OK;
}

// Initialize the I2C pins (using STM32 HAL functions)
void HAL_I2C::initPins() {
    // Pin setup for SDA/SCL using STM32 HAL GPIO functions
    pinMode(_sda, INPUT_PULLUP);  // This can vary based on your MCU
    pinMode(_scl, INPUT_PULLUP);
}

*/


