#include "I2C.h"

HAL_I2C i2c(1, Wire); // You can set busId = 1 or any other meaningful identifier

void setup() {
    Serial.begin(115200);
    i2c.initialize();
    i2c.scanBus();
    i2c.testDevices();
}

void loop() {}
