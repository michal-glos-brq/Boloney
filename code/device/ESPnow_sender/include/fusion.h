#ifndef FUSION_H
#define FUSION_H

// This file contains declarations for sensors and their fusion
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

#include <MadgwickAHRS.h>
#include <SimpleKalmanFilter.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// To get the struct
#include "telemetry.h"


#define CALIBRATION_PERIODS 100
#define CALIBRATION_DELAY pdMS_TO_TICKS(10) // RTOS delay in ticks, 100 ms for example


// #define DEBUG_SENSOR

// #define DEBUG_TELEMETRY

#define FULL_READING_PERIOD 250 // Gyro+Acc updates per full sensor reading

#define PERIOD_SENSOR_READING 10 // Period for reading sensors in ms
#define PERIOD_SENSOR_READING_S 0.01 // Period in seconds
#define SAMPLING_FREQUENCY 1000.0f/PERIOD_SENSOR_READING // Hz

extern telemetryMessage currentTelemetry;
extern uint64_t id_counter;

void setupSensors();
void sensorTask(void * args);
void readTelemetry(SemaphoreHandle_t mtx_sensor_acc, SemaphoreHandle_t mtx_stack);

#endif
