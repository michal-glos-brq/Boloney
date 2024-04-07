#ifndef FUSION_H
#define FUSION_H

// This file contains declarations for sensors and their fusion
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// To get the struct
#include "telemetry.h"


#define DEBUG_SENSOR


#define PERIOD_SENSOR_READING 100 // Period for reading sensors in ms

extern telemetryMessage currentTelemetry;
extern uint64_t id_counter;

void setupSensors();
void sensorTask(void * args);
void readTelemetry(SemaphoreHandle_t mtx_sensor_acc, SemaphoreHandle_t mtx_stack);

#endif
