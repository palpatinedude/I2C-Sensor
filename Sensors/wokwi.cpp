
/*

#define SDA D14
#define SCL D15
#define HAL_I2C_DEBUG 1

#define MPU6050_ADDR 0x69  
#define LCD_ADDR 0x27
#define WHO_AM_I_REG 0x75  
#define PWR_MGMT_1 0x6B  

class HAL_I2C {
public:
    HAL_I2C(uint8_t sda = SDA, uint8_t scl = SCL, uint32_t frequency = 50000);
    void begin();
    bool write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
    bool read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
    void scanBus();
    bool testDevice(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedStatus, uint8_t idRegAddr);


private:
    uint8_t _sda, _scl;
    uint32_t _frequency;
    bool testConnection(uint8_t deviceAddr);
    bool checkStatus(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedValue);  // Check device status
    uint8_t readID(uint8_t deviceAddr, uint8_t regAddr);  // Read device-specific ID (e.g., WHO_AM_I)
};

HAL_I2C::HAL_I2C(uint8_t sda, uint8_t scl, uint32_t frequency) 
    : _sda(sda), _scl(scl), _frequency(frequency) {}

void HAL_I2C::begin() {
    Serial.println("Initiliaze I2C bus");
    Wire.begin(_sda, _scl);
    Wire.setClock(_frequency);
}

bool HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

bool HAL_I2C::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(deviceAddr, (uint8_t)length);
    for (size_t i = 0; i < length; i++) {
        if (Wire.available()) buffer[i] = Wire.read();
        else return false;
    }
    return true;
}

void HAL_I2C::scanBus() {
    Serial.println(F("Scanning I2C bus..."));
    bool foundDevice = false;
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            foundDevice = true;
            Serial.print(F("Device found at 0x"));
            Serial.println(address, HEX);
        }
    }
    if (!foundDevice) {
        Serial.println(F("No devices found on the bus."));
    } else {
        Serial.println(F("Scan complete."));
    }
}
bool HAL_I2C::testDevice(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedStatus, uint8_t idRegAddr) {
    // Test connection
    if (!testConnection(deviceAddr)) {
        Serial.print(F("Device at 0x"));
        Serial.print(deviceAddr, HEX);
        Serial.println(F(" not found!"));
        return false;
    }

    // Check device status (e.g., device should not be in sleep mode)
    if (!checkStatus(deviceAddr, regAddr, expectedStatus)) {
        Serial.print(F("Device at 0x"));
        Serial.print(deviceAddr, HEX);
        Serial.println(F(" failed status check!"));
        return false;
    }

    // Read device ID (e.g., WHO_AM_I)
    uint8_t deviceID = readID(deviceAddr, idRegAddr);
    if (deviceID == 0xFF) {
        Serial.print(F("Failed to read device ID from 0x"));
        Serial.print(deviceAddr, HEX);
        Serial.println(F("!"));
        return false;
    }

    Serial.print(F("Device at 0x"));
    Serial.print(deviceAddr, HEX);
    Serial.print(F(" passed all tests. Device ID: 0x"));
    Serial.println(deviceID, HEX);

    return true;
}


bool HAL_I2C::testConnection(uint8_t deviceAddr) {
    Wire.beginTransmission(deviceAddr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        return true;  // Device is found
    } else {
        return false;  // Device not found
    }
}

//  method to check the device status by reading a register
bool HAL_I2C::checkStatus(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedValue) {
    uint8_t status;
    if (read(deviceAddr, regAddr, &status, 1)) {
        return (status == expectedValue);
    }
    return false;
}

// method to read the device's ID (e.g., WHO_AM_I)
uint8_t HAL_I2C::readID(uint8_t deviceAddr, uint8_t regAddr) {
    uint8_t deviceID = 0;
    if (read(deviceAddr, regAddr, &deviceID, 1)) {
        return deviceID;
    }
    return 0xFF;  
}


/*

// Function to read accelerometer data
void readMPU6050(int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t rawData[6]; 
    if (i2c.read(MPU6050_ADDR, 0x3B, rawData, 6)) { 
        ax = (rawData[0] << 8) | rawData[1];
        ay = (rawData[2] << 8) | rawData[3];
        az = (rawData[4] << 8) | rawData[5];
    } else {
        Serial.println("Error reading MPU6050!");
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);  

    i2c.scanBus();

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Testing Devices");

    bool lcdStatus = i2c.testDevice(LCD_ADDR, 0x00, 0x08, 0x00);
    bool mpuStatus = i2c.testDevice(MPU6050_ADDR, PWR_MGMT_1, 0x00, WHO_AM_I_REG);

    if (lcdStatus && mpuStatus) {
        Serial.println("Both LCD & MPU6050 operational!");
        lcd.setCursor(0, 1);
        lcd.print("MPU & LCD OK!");
    } else {
        Serial.println("One or more devices failed!");
        lcd.setCursor(0, 1);
        lcd.print("Device Error!");
    }

    // Wake up MPU6050 (it's in sleep mode by default)
    i2c.write(MPU6050_ADDR, PWR_MGMT_1, 0x00);
}

void loop() {
    int16_t ax, ay, az;
    readMPU6050(ax, ay, az);

    Serial.print("Ax: "); Serial.print(ax);
    Serial.print(" Ay: "); Serial.print(ay);
    Serial.print(" Az: "); Serial.println(az);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ax:");
    lcd.print(ax);
    lcd.setCursor(0, 1);
    lcd.print("Ay:");
    lcd.print(ay);

    delay(1000);
}



*/

/*
void setup() {

    Serial.begin(115200);
    Wire.begin();
    lcd.begin(16, 2);
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("I2C Test Start");
    delay(2000);

    i2c.scanBus();
    delay(500);

    // Test LCD connection
    if (i2c.testDevice(LCD_ADDR, 0x00, 0x08, 0x00)) {  // Example: Address, Status Register, Expected Value, ID Register
        Serial.println("LCD found and operational!");
        lcd.setCursor(0, 1);
        lcd.print("LCD operational!");
    } else {
        Serial.println("LCD test failed!");
        lcd.setCursor(0, 1);
        lcd.print("LCD test failed!");
    }

    // Test MPU6050 connection
    if (i2c.testDevice(MPU6050_ADDR, 0x6B, 0x00, WHO_AM_I_REG)) {  // 0x6B = PWR_MGMT_1, 0x00 = expected value, WHO_AM_I_REG = ID register
        Serial.println("MPU6050 found and operational!");
        lcd.setCursor(0, 1);
        lcd.print("MPU6050 operational!");
    } else {
        Serial.println("MPU6050 test failed!");
        lcd.setCursor(0, 1);
        lcd.print("MPU6050 test failed!");
    }

    delay(2000);

}

void loop() {
    delay(5000);
}
*/

/*

#include <Arduino.h>
#include <Wire.h>

#define I2C_SUCCESS 0
#define I2C_ERROR 1
#define I2C_TIMEOUT 5  // Timeout in ms

#define HAL_I2C_DEBUG 1  // Set to 1 to enable debugging

// I2C Default Pins
#define SDA_PIN PB7
#define SCL_PIN PB6


#if HAL_I2C_DEBUG
    #define DEBUG_PRINT(msg) Serial.println(msg)
#else
    #define DEBUG_PRINT(msg) // No-op in release mode
#endif

class HAL_I2C {
public:
    HAL_I2C(uint8_t sda, uint8_t scl, uint32_t frequency = 100000);
    void initialize();
    void setClock(uint32_t frequency);

    bool write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
    bool read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);

    void scanBus();
    bool testDevice(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedStatus);
    uint8_t readID(uint8_t deviceAddr, uint8_t regAddr);
    
private:
    uint8_t _sda, _scl;
    uint32_t _frequency;

    uint8_t performWrite(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
    uint8_t performRead(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
};

// Constructor: Initializes I2C with configurable SDA, SCL, and frequency
HAL_I2C::HAL_I2C(uint8_t sda, uint8_t scl, uint32_t frequency) 
    : _sda(sda), _scl(scl), _frequency(frequency) {}

// Initialize I2C
void HAL_I2C::initialize() {
    Wire.begin();
    Wire.setClock(_frequency);
    DEBUG_PRINT("I2C Initialized");
}

// Set I2C clock speed
void HAL_I2C::setClock(uint32_t frequency) {
    _frequency = frequency;
    Wire.setClock(_frequency);
    DEBUG_PRINT("I2C Clock Updated");
}

// --- WRITE OPERATION ---
bool HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    Wire.write(data, length);
    
    if (Wire.endTransmission() == I2C_SUCCESS) {
        DEBUG_PRINT("I2C Write Success");
        return true;
    }

    DEBUG_PRINT("I2C Write Failed");
    return false;
}

// --- READ OPERATION ---
bool HAL_I2C::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);
    
    if (Wire.endTransmission(false) != I2C_SUCCESS) {
        DEBUG_PRINT("I2C Read Request Failed");
        return false;
    }

    Wire.requestFrom(deviceAddr, (uint8_t)length);
    unsigned long startTime = millis();

    for (size_t i = 0; i < length; i++) {
        while (!Wire.available()) {
            if (millis() - startTime > I2C_TIMEOUT) {
                DEBUG_PRINT("I2C Read Timeout");
                return false;
            }
        }
        buffer[i] = Wire.read();
    }

    DEBUG_PRINT("I2C Read Success");
    return true;
}

// --- I2C BUS SCAN ---
void HAL_I2C::scanBus() {
    Serial.println("Scanning I2C bus...");
    bool deviceFound = false;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == I2C_SUCCESS) {
            Serial.print("Device found at: 0x");
            Serial.println(address, HEX);
            deviceFound = true;
        }
    }

    if (!deviceFound) {
        Serial.println("No I2C devices found!");
    }

    Serial.println("I2C scan complete.");
}

// --- DEVICE TESTING FUNCTIONS ---
bool HAL_I2C::testDevice(uint8_t deviceAddr, uint8_t regAddr, uint8_t expectedStatus) {
    Wire.beginTransmission(deviceAddr);
    if (Wire.endTransmission() != I2C_SUCCESS) {
        DEBUG_PRINT("Device not connected!");
        return false;
    }

    uint8_t status;
    if (read(deviceAddr, regAddr, &status, 1) && (status == expectedStatus)) {
        DEBUG_PRINT("Device passed status check!");
        return true;
    }

    DEBUG_PRINT(F("Device status check failed!"));
    return false;
}



HAL_I2C i2cBus(SDA_PIN, SCL_PIN, 100000); 
void setup() {
    Serial.begin(115200);

    // Initialize I2C bus
    i2cBus.initialize();
    i2cBus.scanBus();  // Scan for I2C devices

     // Test MPU6050
    Serial.println("Testing MPU6050...");
    if (i2cBus.testDevice(MPU6050_ADDR, 0x75, 0x69)) { // 0x75 is WHO_AM_I register, should return 0x68
        Serial.println("MPU6050 communication OK.");
    } else {
        Serial.println("MPU6050 NOT responding!");
    }

    // Test LCD
    Serial.println("Testing LCD...");
    if (i2cBus.testDevice(LCD_ADDR, 0x00, 0x00)) {  // LCD doesn't have a standard WHO_AM_I register, simple check
        Serial.println("LCD communication OK.");
    } else {
        Serial.println("LCD NOT responding!");
    }
}

void loop() {}

*/

  /*
    // Start serial communication for debugging
    Serial.begin(115200);
    delay(1000);  // Wait for Serial Monitor to open

    // Initialize the I2C bus
    i2c.initialize();
    delay(4000);
    // Test scanning the I2C bus
    i2c.scanBus();  // This will print all detected I2C devices on the bus

    // Test if a specific I2C device (e.g., LCD) is working
    if (i2c.testDevice(0x29, 0x00, 0xEE);) {
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
    }
    */

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
#define I2C_FREQUENCY 100000 // dependable if  we use different i2c channels???
#define LCD_ADDRESS 0x27


#define MAX_I2C_BUSES 3    // Maximum I2C buses

class HAL_I2C {
public:
    HAL_I2C(TwoWire &wireInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = I2C_FREQUENCY);
   // HAL_I2C(uint8_t busId, TwoWire &wireInstance, uint8_t sda = SDA_PIN, uint8_t scl = SCL_PIN, uint32_t frequency = I2C_FREQUENCY);
    void initialize();
    void initialize();
    void setClock(uint32_t frequency);
    uint8_t write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length);
    uint8_t read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length);
    void scanBus();
    bool testDevice(uint8_t deviceAddr);
    void testDevices();
    void handleError(uint8_t errorCode);
private:
  //  TwoWire &_wire;
  //  uint8_t _sda, _scl;
  //  uint32_t _frequency;
    uint8_t _busId;
    TwoWire &_wire;
    uint8_t _sda, _scl;
    uint32_t _frequency;
    unsigned long _startTime;
};


/*
HAL_I2C::HAL_I2C(TwoWire &wireInstance, uint8_t sda, uint8_t scl, uint32_t frequency)
    : _wire(wireInstance), _sda(sda), _scl(scl), _frequency(frequency) {}

void HAL_I2C::initialize() {
    _wire.begin();
    _wire.setClock(_frequency);
    DEBUG_PRINT("I2C Initialized");
}

void HAL_I2C::setClock(uint32_t frequency) {
    _frequency = frequency;
    _wire.setClock(_frequency);
    DEBUG_PRINT("I2C Clock Updated");
}

uint8_t HAL_I2C::write(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t length) {
    if (!data) {
        handleError(OTHER_ERROR);
        return OTHER_ERROR;
    }

    _wire.beginTransmission(deviceAddr);
    _wire.write(regAddr);
    _wire.write(data, length);

    unsigned long startTime = millis();
    while (_wire.endTransmission() != SUCCESS) {
        if (millis() - startTime > I2C_TIMEOUT_MS) {
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

    unsigned long startTime = millis();
    _wire.requestFrom(deviceAddr, (uint8_t)length);
    
    size_t received = 0;
    while (_wire.available() && received < length) {
        buffer[received++] = _wire.read();
        if (millis() - startTime > I2C_TIMEOUT_MS) {
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
            switch (address) {
                case 0x27: case 0x3F:
                    Serial.println("  -> LCD detected."); break;
                case 0x29:
                    Serial.println("  -> ToF Sensor detected."); break;
                case 0x68: case 0x69:
                    Serial.println("  -> IMU detected."); break;
                default:
                    Serial.println("  -> Unknown device."); break;
            }
        }
    }
}

void HAL_I2C::handleError(uint8_t errorCode) {
    const char* errorMsg[] = {"Success", "Data Too Long", "NACK on Address", "NACK on Data", "Other Error"};
    Serial.print("I2C Error: ");
    Serial.println(errorMsg[errorCode]);
}
*/

// Constructor to initialize the I2C bus with the provided settings
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

// Write data to a device with error handling and timeout
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

// Read data from a device with error handling and timeout
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

// Scan the I2C bus for devices
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

// Test if a specific I2C device is responding
bool HAL_I2C::testDevice(uint8_t deviceAddr) {
    _wire.beginTransmission(deviceAddr);
    if (_wire.endTransmission() != SUCCESS) return false;
    Serial.print("Device found at 0x"); Serial.println(deviceAddr, HEX);
    return true;
}

// Test multiple devices on the I2C bus
void HAL_I2C::testDevices() {
    Serial.println("\nTesting I2C devices...");
    for (uint8_t address = 1; address < 127; address++) {
        if (testDevice(address)) {
            Serial.print("Device at address 0x");
            Serial.println(address, HEX);
        }
    }
}

// Error handling function
void HAL_I2C::handleError(uint8_t errorCode) {
    const char* errorMsg[] = {"Success", "Data Too Long", "NACK on Address", "NACK on Data", "Other Error"};
    Serial.print("I2C Error on Bus ");
    Serial.print(_busId);
    Serial.print(": ");
    Serial.println(errorMsg[errorCode]);
}

HAL_I2C i2c(Wire);
void setup() {
    Serial.begin(115200);
    i2c.initialize();
    i2c.scanBus();
    i2c.testDevices();
}

void loop() {}
