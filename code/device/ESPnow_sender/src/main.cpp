
#include <main.h>

// Debug compilation (comment to turn debug off)
//#define DEBUG
#define DEBUG_COMMS
//#define DEBUG_TELEMETRY

// Timing stuff

uint64_t telemetry_time_tmp; // For reading and integratio of telemetry data
double telemetry_time_difference; // This is in milliseconds (dt)


void loop() {}

// Setup TROS periodic tasks
void setupRTOS() {
  xTaskCreatePinnedToCore(
    commsTask,   // Function to implement the task
    "Communication",         // Name of the task
    10000,                    // Stack size in words
    NULL,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    1);                      // Core where the task should run

  ///*
  xTaskCreatePinnedToCore(
    sensorTask,          // Function to implement the task
    "Sensor",                // Name of the task
    16000,                    // Stack size in words
    NULL,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    0);                      // Core where the task should run */
}

void setup() {


  // Init Serial Monitor
  Serial.begin(115200);
 
  setupSensors();
  setupComms();

  setupRTOS();
}
