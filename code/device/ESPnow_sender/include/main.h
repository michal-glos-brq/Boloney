#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS_RCV {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};
#define MAC_ADDRESS_SND {0x64, 0xE8, 0x33, 0x8A, 0xA7, 0xA0};

#define BUFFER_CAPACITY 100 // 0 for electrical efficiency

#define PERIOD_TIME 1000 // ms for one comms iteration
#define MIN_WAIT_TIME 500 // ms for msg dump

#define PERIOD_SENSOR_READING 100 // For reading sensors
#define CONNECTION_TIMEOUT 5000 // TImeout to stop sending messages (dumps)


typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    uint64_t relativeTime;
    uint64_t id;
    int valid;
    double gyroscope[3];
    double accelerometer[3];
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;


esp_now_peer_info_t peerInfo;

struct telemetryMessage currentTelemetry;
telemetryMessage * telemetryArray;


MPU6050 mpu;    // I2C
Adafruit_BMP280 bmp; // I2C

MPU6050::struct_raw_MPU data;

// Mutexes for concurrent access
SemaphoreHandle_t mtx_stack;
SemaphoreHandle_t mtx_struct;
SemaphoreHandle_t mtx_sensor_acc;


// Timing stuff
uint64_t current_loop_start, last_message_received;
uint64_t current_time_telemetry, telemetry_time_tmp; // For reading and integratio of telemetry data
double telemetry_time_difference; // This is in milliseconds (dt)
long int msg_timing_diff;
// Telemetry accumulators
double position[3];
double orientation[3];

#endif