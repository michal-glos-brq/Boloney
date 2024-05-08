#ifndef FUSION_H
#define FUSION_H

// This file contains declarations for sensors and their fusion
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <cmath>

#include "Madgwick.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// To get the struct
#include "telemetry.h"

// More beta - less responsive, more focused on accelerometer
#define BETA 0.6f // Madgwick filter beta
// "lr" for acc and gyro low pass bias estimators
#define ALPHA_ACCX 0.05f // "lr" for acc bias estimation
#define MAX_ACCX 1.0f // Bounds for error for bias estimation

#define ALPHA_ACCY 0.05f
#define MAX_ACCY 1.0f

#define ALPHA_ACCZ 0.05f
#define MAX_ACCZ 1.0f


#define ALPHA_GYRX 0.01f
#define MAX_GYRX 0.1f

#define ALPHA_GYRY 0.01f
#define MAX_GYRY 0.1f

#define ALPHA_GYRZ 0.01f
#define MAX_GYRZ 0.1f

#define ALPHA_GRAV 0.1f // Roll averaging coefficient for low pass sensor filters for gravity magnitude

// Complementary filter coefficient
#define BAR_COEFF 0.1f // Barometer

#define CALIBRATION_PERIODS 100 // Number of periods to average for calibration
#define CALIBRATION_DELAY pdMS_TO_TICKS(10) // RTOS delay in ticks, 100 ms for example


// Sensor initialization
// #define DEBUG_SENSOR
// Sensor readings
//#define DEBUG_TELEMETRY
// Seonsor calibration debug
#define DEBUG_CALIBRATION
// Debugging sensor read timing
// #define DEBUG_SENSOR_TIMING

#define FULL_READING_PERIOD 100 // Gyro+Acc updates per full sensor reading

#define PERIOD_SENSOR_READING 10 // Period for reading sensors in ms
#define PERIOD_SENSOR_READING_S PERIOD_SENSOR_READING/1000. // Period in seconds
#define SAMPLING_FREQUENCY 1000.0f/PERIOD_SENSOR_READING // Hz

// To initialize
#define REFERENCE_PERIODS 10  // Number of periods to average for reference condition
#define REFERENCE_DELAY pdMS_TO_TICKS(100)  // RTOS delay in ticks, 100 ms for example

extern telemetryMessage currentTelemetry;
extern uint64_t id_counter;


struct Quaternion {
    float w, x, y, z;

    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,  // new w
            w * q.x + x * q.w + y * q.z - z * q.y,  // new x
            w * q.y - x * q.z + y * q.w + z * q.x,  // new y
            w * q.z + x * q.y - y * q.x + z * q.w   // new z
        );
    }

    // Conjugate of a quaternion
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
};

void setupSensors();
void otherSensorTask(void * args);
void sensorTask(void * args);
void readTelemetry();

#endif
