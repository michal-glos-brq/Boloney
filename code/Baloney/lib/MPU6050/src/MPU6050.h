#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    struct struct_raw_MPU {
        int16_t AcX;
        int16_t AcY;
        int16_t AcZ;
        int16_t GyX;
        int16_t GyY;
        int16_t GyZ;
        int16_t Tmp;
    };

    MPU6050();
    void begin();
    void read();
    void printData();
    struct_raw_MPU getRawData();

    

private:
    const int MPU_addr = 0x68;
    
    struct_raw_MPU rawImuData;
    
};

#endif