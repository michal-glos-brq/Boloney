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

float acc_raw_bias[3] = {0, 0, 0};
float gyro_raw_bias[3] = {0, 0, 0};

Madgwick filter;

uint64_t last_telemetry_timestamp;

struct telemetryMessage dynamicTelemetry;
telemetryMessage currentTelemetry;

uint64_t id_counter = 0;
int period_counter = 0;

bool mpuReady = false;
bool bmpReady = false;

// Gyroscope Kalman Filters for each axis
SimpleKalmanFilter gyroKalmanFilterX(0.03, 1, 0.01);
SimpleKalmanFilter gyroKalmanFilterY(0.03, 1, 0.01);
SimpleKalmanFilter gyroKalmanFilterZ(0.03, 1, 0.01);

// Accelerometer Kalman Filters for each axis
SimpleKalmanFilter accelKalmanFilterX(0.03, 1, 0.01);
SimpleKalmanFilter accelKalmanFilterY(0.03, 1, 0.01);
SimpleKalmanFilter accelKalmanFilterZ(0.03, 1, 0.01);


SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);


const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;

void applyRotationMatrix(float *acc, float * orientation) {
    // Convert angles from degrees to radians
    float pitchRad = orientation[0] * M_PI / 180.0;
    float yawRad = orientation[1] * M_PI / 180.0;
    float rollRad = orientation[2] * M_PI / 180.0;

    // Pre-calculate sine and cosine of the angles
    float sinPitch = sin(pitchRad), cosPitch = cos(pitchRad);
    float sinYaw = sin(yawRad), cosYaw = cos(yawRad);
    float sinRoll = sin(rollRad), cosRoll = cos(rollRad);

    // Calculate the rotation matrix components
    float R11 = cosYaw * cosPitch;
    float R12 = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    float R13 = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    float R21 = sinYaw * cosPitch;
    float R22 = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
    float R23 = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
    float R31 = -sinPitch;
    float R32 = cosPitch * sinRoll;
    float R33 = cosPitch * cosRoll;

    // Apply the rotation matrix to the acceleration vector
    float accX_new = R11 * acc[0] + R12 * acc[1] + R13 * acc[2];
    float accY_new = R21 * acc[0] + R22 * acc[1] + R23 * acc[2];
    float accZ_new = R31 * acc[0] + R32 * acc[1] + R33 * acc[2];

    // Update the original acceleration vector
    acc[0] = accX_new;
    acc[1] = accY_new;
    acc[2] = accZ_new;
}

void estimateMeasurementUncertainty(float * accel_var_x, float * accel_var_y, float * accel_var_z, float * gyro_var_x, float * gyro_var_y, float * gyro_var_z, float * baro_var) {

  if ((!mpuReady) | (!bmpReady)) {
    exit(1);
  }

  float accX_samples[CALIBRATION_PERIODS], accY_samples[CALIBRATION_PERIODS], accZ_samples[CALIBRATION_PERIODS];
  float gyroX_samples[CALIBRATION_PERIODS], gyroY_samples[CALIBRATION_PERIODS], gyroZ_samples[CALIBRATION_PERIODS];
  float baro_samples[CALIBRATION_PERIODS];

  float acc_sums[3] = {0, 0, 0}, acc_means[3], acc_variances[3] = {0, 0, 0};
  float gyro_sums[3] = {0, 0, 0}, gyro_means[3], gyro_variances[3] = {0, 0, 0};
  float baro_sum = 0, baro_mean, baro_variance = 0;

  // Collect samples
  for (int i = 0; i < (int)CALIBRATION_PERIODS; i++) {

    mpu.read();
    data = mpu.getRawData();

    accX_samples[i] = data.AcX;
    accY_samples[i] = data.AcY;
    accZ_samples[i] = data.AcZ;

    gyroX_samples[i] = data.GyX;
    gyroY_samples[i] = data.GyY;
    gyroZ_samples[i] = data.GyZ;

    if (bmp.takeForcedMeasurement()) {
      baro_samples[i] = bmp.readPressure();
    }

    acc_sums[0] += accX_samples[i];
    acc_sums[1] += accY_samples[i];
    acc_sums[2] += accZ_samples[i];

    gyro_sums[0] += gyroX_samples[i];
    gyro_sums[1] += gyroY_samples[i];
    gyro_sums[2] += gyroZ_samples[i];

    baro_sum += baro_samples[i];

    delay(CALIBRATION_DELAY);
  }

  acc_means[0] = acc_sums[0] / CALIBRATION_PERIODS;
  acc_means[1] = acc_sums[1] / CALIBRATION_PERIODS;
  acc_means[2] = acc_sums[2] / CALIBRATION_PERIODS;

  gyro_means[0] = gyro_sums[0] / CALIBRATION_PERIODS;
  gyro_means[1] = gyro_sums[1] / CALIBRATION_PERIODS;
  gyro_means[2] = gyro_sums[2] / CALIBRATION_PERIODS;

  baro_mean = baro_sum / CALIBRATION_PERIODS;

  // Calculate variance
  for (int i = 0; i < CALIBRATION_PERIODS; i++) {
    acc_variances[0] += pow(accX_samples[i] - acc_means[0], 2);
    acc_variances[1] += pow(accY_samples[i] - acc_means[1], 2);
    acc_variances[2] += pow(accZ_samples[i] - acc_means[2], 2);

    gyro_variances[0] += pow(gyroX_samples[i] - gyro_means[0], 2);
    gyro_variances[1] += pow(gyroY_samples[i] - gyro_means[1], 2);
    gyro_variances[2] += pow(gyroZ_samples[i] - gyro_means[2], 2);

    baro_variance += pow(baro_samples[i] - baro_mean, 2);
  }
  
  acc_variances[0] /= CALIBRATION_PERIODS;
  acc_variances[1] /= CALIBRATION_PERIODS;
  acc_variances[2] /= CALIBRATION_PERIODS;

  gyro_variances[0] /= CALIBRATION_PERIODS;
  gyro_variances[1] /= CALIBRATION_PERIODS;
  gyro_variances[2] /= CALIBRATION_PERIODS;

  baro_variance /= CALIBRATION_PERIODS;

  *accel_var_x = sqrt(acc_variances[0]);
  *accel_var_y = sqrt(acc_variances[1]);
  *accel_var_z = sqrt(acc_variances[2]);

  *gyro_var_x = sqrt(gyro_variances[0]);
  *gyro_var_y = sqrt(gyro_variances[1]);
  *gyro_var_z = sqrt(gyro_variances[2]);

  *baro_var = sqrt(baro_variance);

  return;
}

void setup_kellmans() {
  float accel_var_x, accel_var_y, accel_var_z;
  float gyro_var_x, gyro_var_y, gyro_var_z;
  float baro_var;

  estimateMeasurementUncertainty(&accel_var_x, &accel_var_y, &accel_var_z, &gyro_var_x, &gyro_var_y, &gyro_var_z, &baro_var);

  // Initialize Kalman filters
  accelKalmanFilterX.setProcessNoise(accel_var_x);
  accelKalmanFilterY.setProcessNoise(accel_var_y);
  accelKalmanFilterZ.setProcessNoise(accel_var_z);

  gyroKalmanFilterX.setProcessNoise(gyro_var_x);
  gyroKalmanFilterY.setProcessNoise(gyro_var_y);
  gyroKalmanFilterZ.setProcessNoise(gyro_var_z);

  pressureKalmanFilter.setProcessNoise(baro_var);

  return;
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
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = PERIOD_SENSOR_READING;
  filter.begin(SAMPLING_FREQUENCY);

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
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

      angular_accelaration[0] = gyroKalmanFilterX.updateEstimate(data.GyX)*gyroScale;
      angular_accelaration[1] = gyroKalmanFilterY.updateEstimate(data.GyY)*gyroScale;
      angular_accelaration[2] = gyroKalmanFilterZ.updateEstimate(data.GyZ)*gyroScale;

      acceleration[0] = accelKalmanFilterX.updateEstimate(data.AcX)*accScale;
      acceleration[1] = accelKalmanFilterY.updateEstimate(data.AcY)*accScale;
      acceleration[2] = accelKalmanFilterZ.updateEstimate(data.AcZ)*accScale;

      filter.updateIMU(angular_accelaration[0], angular_accelaration[1], angular_accelaration[2],
                    acceleration[0], acceleration[1], acceleration[2]);


      dynamicTelemetry.orientation[0] = filter.getPitch();
      dynamicTelemetry.orientation[1] = filter.getYaw();
      dynamicTelemetry.orientation[2] = filter.getRoll();

      applyRotationMatrix(acceleration, dynamicTelemetry.orientation);

      // Integrate acceleration to update velocity
      velocity[0] += acceleration[0] * PERIOD_SENSOR_READING_S;
      velocity[1] += acceleration[1] * PERIOD_SENSOR_READING_S;
      velocity[2] += acceleration[2] * PERIOD_SENSOR_READING_S;

      position[0] += velocity[0] * PERIOD_SENSOR_READING_S;
      position[1] += velocity[1] * PERIOD_SENSOR_READING_S;
      position[2] += velocity[2] * PERIOD_SENSOR_READING_S;

      dynamicTelemetry.velocity[0] = velocity[0];
      dynamicTelemetry.velocity[1] = velocity[1];
      dynamicTelemetry.velocity[2] = velocity[2];

      dynamicTelemetry.position[0] = position[0];
      dynamicTelemetry.position[1] = position[1];
      dynamicTelemetry.position[2] = position[2];

      dynamicTelemetry.angularVelocity[0] = angular_accelaration[0];
      dynamicTelemetry.angularVelocity[1] = angular_accelaration[1];
      dynamicTelemetry.angularVelocity[2] = angular_accelaration[2];



      #ifdef DEBUG_TELEMETRY
      Serial.print("PosX: ");
      Serial.print(dynamicTelemetry.position[0]);
      Serial.print(" PosY: ");
      Serial.print(dynamicTelemetry.position[1]);
      Serial.print(" PosZ: ");
      Serial.print(dynamicTelemetry.position[2]);
      Serial.print(" VelX: ");
      Serial.print(velocity[0]);
      Serial.print(" VelY: ");
      Serial.print(velocity[1]);
      Serial.print(" VelZ: ");
      Serial.print(velocity[2]);

      Serial.print("Pitch: ");
      Serial.print(dynamicTelemetry.orientation[0]);
      Serial.print(" Yaw: ");
      Serial.print(dynamicTelemetry.orientation[1]);
      Serial.print(" Roll: ");
      Serial.println(dynamicTelemetry.orientation[2]);
      #endif

    }
    
    if (period_counter > FULL_READING_PERIOD){
      period_counter = 0;
      if(bmpReady){
        if (bmp.takeForcedMeasurement()) {
          // can now print out the new measurements
          dynamicTelemetry.thermometer = bmp.readTemperature();
          dynamicTelemetry.barometer =  pressureKalmanFilter.updateEstimate(bmp.readPressure());
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
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
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
