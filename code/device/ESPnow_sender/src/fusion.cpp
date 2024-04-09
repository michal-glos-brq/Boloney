#include "fusion.h"

MPU6050 mpu;    // I2C
Adafruit_BMP280 bmp; // I2C
MPU6050::struct_raw_MPU data;


// To initialize
#define REFERENCE_PERIODS 10  // Number of periods to average for reference condition
#define REFERENCE_DELAY pdMS_TO_TICKS(100)  // RTOS delay in ticks, 100 ms for example

// Integration magic
float acceleration[3] = {0, 0, 0};
float position[3] = {0, 0, 0};
float velocity[3] = {0, 0, 0};

float angular_accelaration[3] = {0, 0, 0};
float orientation[3] = {0, 0, 0};
float angular_velocity[3] = {0, 0, 0};

Madgwick filter;

uint64_t last_telemetry_timestamp;

struct telemetryMessage dynamicTelemetry;
telemetryMessage currentTelemetry;

uint64_t id_counter = 0;
int period_counter = 0;

bool mpuReady = false;
bool bmpReady = false;



float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


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

  filter.begin(SAMPLING_FREQUENCY);

  while (1) {

    period_counter++;    
    last_telemetry_timestamp = micros();
    //xSemaphoreTake(mtx_sensor_acc, portMAX_DELAY);
    if(mpuReady){
      mpu.read();
      data = mpu.getRawData();
      // Now you can access the raw data like this:
      // data.AcX, data.AcY, data.AcZ, etc.
      // Update orientation and position with basic algorithm with gravitation vector detection
      if (period_counter > FULL_READING_PERIOD){
        dynamicTelemetry.thermometer_stupido = data.Tmp/340.00+36.53;
      }

      dynamicTelemetry.relativeTime = last_telemetry_timestamp;

      angular_accelaration[0] = convertRawGyro(data.GyX);
      angular_accelaration[1] = convertRawGyro(data.GyY);
      angular_accelaration[2] = convertRawGyro(data.GyZ);
      acceleration[0] = convertRawAcceleration(data.AcX);
      acceleration[1] = convertRawAcceleration(data.AcY);
      acceleration[2] = convertRawAcceleration(data.AcZ);

      filter.updateIMU(angular_accelaration[0], angular_accelaration[1], angular_accelaration[2],
                    acceleration[0], acceleration[1], acceleration[2]);

      dynamicTelemetry.orientation[0] = filter.getPitch();
      dynamicTelemetry.orientation[1] = filter.getYaw();
      dynamicTelemetry.orientation[2] = filter.getRoll();

    }
    
    if (period_counter > FULL_READING_PERIOD){
      period_counter = 0;
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
    }
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
