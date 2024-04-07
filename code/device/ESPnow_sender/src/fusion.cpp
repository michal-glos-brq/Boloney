#include "fusion.h"

MPU6050 mpu;    // I2C
Adafruit_BMP280 bmp; // I2C
MPU6050::struct_raw_MPU data;
double position[3];
double orientation[3];
uint64_t last_telemetry_timestamp;

struct telemetryMessage dynamicTelemetry;
telemetryMessage currentTelemetry;

uint64_t id_counter = 0;

bool mpuReady = false;
bool bmpReady = false;


void setupSensors() {
  Wire.begin(8, 10); // SDA, SCL
  if(mpu.begin()){
    #ifdef DEBUG_SENSOR
    Serial.println("MPU6050 is ready!");
    #endif
    mpuReady = true;
  } else {
    #ifdef DEBUG_SENSOR
    Serial.println("MPU6050 is not ready!");
    #endif
  }

     //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (bmp.begin()) {
    #ifdef DEBUG_SENSOR
    Serial.println("BMP280 is ready!");
    #endif
    bmpReady = true;
  } else {
    #ifdef DEBUG_SENSOR
    Serial.println("BMP280 is not ready!");
    #endif
  }

  if(bmpReady){
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  dynamicTelemetry.valid = 1;
}


void sensorTask(void * args) {

  SemaphoreHandle_t mtx_sensor_acc = (SemaphoreHandle_t) args;

  while (1) {
    
    last_telemetry_timestamp = micros();
    //xSemaphoreTake(mtx_sensor_acc, portMAX_DELAY);
    if(mpuReady){
      mpu.read();
      data = mpu.getRawData();
      // Now you can access the raw data like this:
      // data.AcX, data.AcY, data.AcZ, etc.
      // Update orientation and position with basic algorithm with gravitation vector detection
      dynamicTelemetry.thermometer_stupido = data.Tmp/340.00+36.53;
      dynamicTelemetry.position[0] = data.AcX;
      dynamicTelemetry.position[1] = data.AcY;
      dynamicTelemetry.position[2] = data.AcZ;
      dynamicTelemetry.orientation[0] = data.GyX;
      dynamicTelemetry.orientation[1] = data.GyY;
      dynamicTelemetry.orientation[2] = data.GyZ;
    }
    
    if(bmpReady){
      if (bmp.takeForcedMeasurement()) {
        // can now print out the new measurements
        dynamicTelemetry.thermometer = bmp.readTemperature();
        dynamicTelemetry.barometer = bmp.readPressure();
      }
      else {
        dynamicTelemetry.thermometer = 0;
        dynamicTelemetry.barometer = 0;

        #ifdef DEBUG_SENSOR
        Serial.println("BMP280 measurement failed ...");
        #endif
      }
    }
    dynamicTelemetry.voltage = 3.7;
    dynamicTelemetry.relativeTime = last_telemetry_timestamp;
    //xSemaphoreGive(mtx_sensor_acc);

    #ifdef DEBUG_SENSOR
    mpu.printData();
    // Check the remaining stack space
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("Sensor Task Stack High Water Mark: ");
    Serial.println(uxHighWaterMark);
    #endif
    vTaskDelay(PERIOD_SENSOR_READING);  
  }
}

// This function creates telemetryMessage struct
void readTelemetry(SemaphoreHandle_t mtx_sensor_acc, SemaphoreHandle_t mtx_stack){
  // Some actual logic will have to be implemented here ...
  // message->relativeTime = micros();
  
  //xSemaphoreTake(mtx_stack, portMAX_DELAY);
  //xSemaphoreTake(mtx_sensor_acc, portMAX_DELAY);
  memcpy(&currentTelemetry, &dynamicTelemetry, sizeof(telemetryMessage));
  //xSemaphoreGive(mtx_sensor_acc);
  currentTelemetry.id = id_counter++;
  //xSemaphoreGive(mtx_stack);
}
