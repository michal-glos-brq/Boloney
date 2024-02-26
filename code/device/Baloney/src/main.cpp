#include <Arduino.h>
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
    mpu.begin();
}

void loop() {
    mpu.read();
    mpu.printData();

    MPU6050::struct_raw_MPU data = mpu.getRawData();
    // Now you can access the raw data like this:
    // data.AcX, data.AcY, data.AcZ, etc.
}
