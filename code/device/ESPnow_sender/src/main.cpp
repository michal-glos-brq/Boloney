
#include <main.h>

// Debug compilation (comment to turn debug off)
//#define DEBUG
#define DEBUG_COMMS
//#define DEBUG_TELEMETRY

// Mutexes for concurrent access
SemaphoreHandle_t mtx_stack;
SemaphoreHandle_t mtx_sensor_acc;

// Timing stuff

uint64_t telemetry_time_tmp; // For reading and integratio of telemetry data
double telemetry_time_difference; // This is in milliseconds (dt)


void loop() {}

// Setup TROS periodic tasks
void setupRTOS(SemaphoresComms semaphores) {
  xTaskCreatePinnedToCore(
    commsTask,   // Function to implement the task
    "Communication",         // Name of the task
    10000,                    // Stack size in words
    (void *)&semaphores,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    1);                      // Core where the task should run

  ///*
  xTaskCreatePinnedToCore(
    sensorTask,          // Function to implement the task
    "Sensor",                // Name of the task
    10000,                    // Stack size in words
    (void *) mtx_sensor_acc,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    0);                      // Core where the task should run */
}

void setup() {

  mtx_stack = xSemaphoreCreateMutex();
  mtx_sensor_acc = xSemaphoreCreateMutex();

  SemaphoresComms semaphores;
  semaphores.mtx_stack = mtx_stack;
  semaphores.mtx_sensor_acc = mtx_sensor_acc;

  if (mtx_stack == NULL || mtx_sensor_acc == NULL) {
    // Handle case where mutex creation failed
    Serial.println("Failed to create mutexes");
    return;
  }

  // Init Serial Monitor
  Serial.begin(115200);
 
  setupSensors();
  setupComms();

  setupRTOS(semaphores);
}
