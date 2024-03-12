#include "MPU6050.h"

MPU6050::MPU6050() {}

bool MPU6050::begin() {
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(115200);
    return true;
}

bool MPU6050::read() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, static_cast<uint8_t>(14u), true);
    rawImuData.AcX=Wire.read()<<8|Wire.read();    
    rawImuData.AcY=Wire.read()<<8|Wire.read();
    rawImuData.AcZ=Wire.read()<<8|Wire.read();
    rawImuData.Tmp=Wire.read()<<8|Wire.read();
    rawImuData.GyX=Wire.read()<<8|Wire.read();
    rawImuData.GyY=Wire.read()<<8|Wire.read();
    rawImuData.GyZ=Wire.read()<<8|Wire.read();
    Wire.endTransmission(true);
    return true;
}

void MPU6050::printData() {
    Serial.print("AcX = "); Serial.print(rawImuData.AcX);
    Serial.print(" | AcY = "); Serial.print(rawImuData.AcY);
    Serial.print(" | AcZ = "); Serial.print(rawImuData.AcZ);
    Serial.print(" | Temp = "); Serial.print(rawImuData.Tmp/340.00+36.53);
    Serial.print(" | GyX = "); Serial.print(rawImuData.GyX);
    Serial.print(" | GyY = "); Serial.print(rawImuData.GyY);
    Serial.print(" | GyZ = "); Serial.println(rawImuData.GyZ);
}

MPU6050::struct_raw_MPU MPU6050::getRawData() {
    return rawImuData;
}