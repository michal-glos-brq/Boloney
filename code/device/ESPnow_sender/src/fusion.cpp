#include "fusion.h"

// Here we read the sensors
MPU6050 mpu;    // I2C
Adafruit_BMP280 bmp; // I2C
MPU6050::struct_raw_MPU data;

// Filter for orentation estimation
Madgwick filter;

// Timestamp for telemetry readings
uint64_t last_telemetry_timestamp;

// Telemetry message structure
struct telemetryMessage dynamicTelemetry;
telemetryMessage currentTelemetry;

// Incremet counters
uint64_t id_counter = 0;
int period_counter = 0;

// Boolean indicators of sensors OK
bool mpuReady = false;
bool bmpReady = false;

// Scaling factors for sensors, assuming default configuration
const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;

// Dynamic gravitation vectors (the magnitude is updated, the direction is obtained with orientation vector)
float gravitation[3] = {0, 0, 9.81};
float correction_gravitation[3] = {0, 0, 0};

// This is used to translate acc vectors from local to global frame of refference
float inverse_orientation[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};
float accel_bias[3] = {0, 0, 0};

// Complementary filter for barometer
float pre_corrected_baro = 0;

// Integration variables
float dt;
unsigned long integration_timestamp, tmp_timestamp;

#define clamp(x, min, max) (x < min ? min : (x > max ? max : x))

// Function to apply rotation using quaternion
void applyRotationMatrix(float *acc, float *orientation) {
    // Convert angles from degrees to radians
    float yawRad = orientation[0] * M_PI / 180.0;
    float rollRad = orientation[1] * M_PI / 180.0;
    float pitchRad = orientation[2] * M_PI / 180.0;

    // Calculate half angles for quaternion calculations
    float halfPitch = pitchRad * 0.5f;
    float halfYaw = yawRad * 0.5f;
    float halfRoll = rollRad * 0.5f;

    // Pre-calculate sine and cosine for each half angle
    float sinHalfPitch = sin(halfPitch), cosHalfPitch = cos(halfPitch);
    float sinHalfYaw = sin(halfYaw), cosHalfYaw = cos(halfYaw);
    float sinHalfRoll = sin(halfRoll), cosHalfRoll = cos(halfRoll);

    // Create quaternion from Euler angles
    Quaternion q(
        cosHalfYaw * cosHalfPitch * cosHalfRoll + sinHalfYaw * sinHalfPitch * sinHalfRoll,
        cosHalfYaw * sinHalfPitch * cosHalfRoll + sinHalfYaw * cosHalfPitch * sinHalfRoll,
        sinHalfYaw * cosHalfPitch * cosHalfRoll - cosHalfYaw * sinHalfPitch * sinHalfRoll,
        cosHalfYaw * cosHalfPitch * sinHalfRoll - sinHalfYaw * sinHalfPitch * cosHalfRoll
    );

    // Create the conjugate of the quaternion for rotation
    Quaternion q_conjugate = q.conjugate();

    // Represent the acceleration vector as a quaternion
    Quaternion acc_quaternion(0, acc[0], acc[1], acc[2]);

    // Rotate the vector using q * acc * q_conjugate
    Quaternion rotated = q * acc_quaternion * q_conjugate;

    // Update the original acceleration vector
    acc[0] = rotated.x;
    acc[1] = rotated.y;
    acc[2] = rotated.z;
}

void estimateMeasurementUncertainty() {
  // This basically obtains static bias and variance of the sensors
  if ((!mpuReady) | (!bmpReady)) {
    exit(1);
  }

  int32_t accX_samples[CALIBRATION_PERIODS], accY_samples[CALIBRATION_PERIODS], accZ_samples[CALIBRATION_PERIODS];
  int32_t gyroX_samples[CALIBRATION_PERIODS], gyroY_samples[CALIBRATION_PERIODS], gyroZ_samples[CALIBRATION_PERIODS];
  float baro_samples[CALIBRATION_PERIODS];

  int32_t acc_sums[3] = {0, 0, 0}, gyro_sums[3] = {0, 0, 0};
  float acc_means[3], acc_variances[3] = {0, 0, 0}, gyro_means[3], gyro_variances[3] = {0, 0, 0};
  float baro_sum = 0, baro_mean, baro_variance = 0;

  // Collect samples
  for (int i = 0; i < (int)CALIBRATION_PERIODS; i++) {

    mpu.read();
    data = mpu.getRawData();

    accX_samples[i] = (int32_t)data.AcX;
    accY_samples[i] = (int32_t)data.AcY;
    accZ_samples[i] = (int32_t)data.AcZ;

    gyroX_samples[i] = (int32_t)data.GyX;
    gyroY_samples[i] = (int32_t)data.GyY;
    gyroZ_samples[i] = (int32_t)data.GyZ;

    if (bmp.takeForcedMeasurement()) {
      baro_samples[i] = bmp.readPressure();
    }

    delay(CALIBRATION_DELAY);
  }

  for (int i = 0; i < (int)CALIBRATION_PERIODS; i++) {
    acc_sums[0] += accX_samples[i];
    acc_sums[1] += accY_samples[i];
    acc_sums[2] += accZ_samples[i];

    gyro_sums[0] += gyroX_samples[i];
    gyro_sums[1] += gyroY_samples[i];
    gyro_sums[2] += gyroZ_samples[i];

    baro_sum += baro_samples[i];
  }


  acc_means[0] = ((float)acc_sums[0] / (float)CALIBRATION_PERIODS * accScale);
  acc_means[1] = ((float)acc_sums[1] / (float)CALIBRATION_PERIODS * accScale);
  acc_means[2] = ((float)acc_sums[2] / (float)CALIBRATION_PERIODS * accScale);

  // I am not even sure if this is necessary, but now I worry it would stop working when removed
  // We assume the "probe" is not accelerating, hence the measurements should be equal to gravitational acceleration
  float acc_magnitude = sqrt(acc_means[0] * acc_means[0] + acc_means[1] * acc_means[1] + acc_means[2] * acc_means[2]);
  gravitation[2] = acc_magnitude;

  if (acc_magnitude == 0) {
    Serial.println("Acceleration magnitude is zero, exiting ...");
    exit(1);
  }

  float norm_acc_x = acc_means[0] / acc_magnitude;
  float norm_acc_y = acc_means[1] / acc_magnitude;
  float norm_acc_z = acc_means[2] / acc_magnitude;

 // Empirically determined, rather do not touch
 // THIS: (pitch, yaw, roll)
 // Compute orientation angles
 // The minuses here are due to axes mismatch
  dynamicTelemetry.orientation[0] = -atan2(-norm_acc_x, sqrt(norm_acc_y*norm_acc_y + norm_acc_z*norm_acc_z)) * 180.0f / M_PI;
  dynamicTelemetry.orientation[2] = -atan2(norm_acc_y, norm_acc_z) * 180.0f / M_PI;

  // Here we setup the reference point and assume orientation to the golbal system with accelerometer measurements
  // We might introduce some bias to the orientation, but it's not a big deal
  float adjusted_graviation[3] = {0, 0, acc_magnitude};


  //applyRotationMatrix(adjusted_graviation, reverse_orientation);
  // X and Y is in reverse, Z is correct
  applyRotationMatrix(adjusted_graviation, dynamicTelemetry.orientation);

  // This does not work, set quaternions
  filter.setOrientation(dynamicTelemetry.orientation[2] * M_PI / 180.0, dynamicTelemetry.orientation[0] * M_PI / 180.0, dynamicTelemetry.orientation[1] * M_PI / 180.0);


  accel_bias[0] = acc_means[0] - adjusted_graviation[0];
  accel_bias[1] = acc_means[1] - adjusted_graviation[1];
  accel_bias[2] = acc_means[2] - adjusted_graviation[2];

  gyro_means[0] = ((float)gyro_sums[0] / (float)CALIBRATION_PERIODS * gyroScale);
  gyro_means[1] = ((float)gyro_sums[1] / (float)CALIBRATION_PERIODS * gyroScale);
  gyro_means[2] = ((float)gyro_sums[2] / (float)CALIBRATION_PERIODS * gyroScale);

  gyro_bias[0] = gyro_means[0];
  gyro_bias[1] = gyro_means[1];
  gyro_bias[2] = gyro_means[2];

  baro_mean = baro_sum / CALIBRATION_PERIODS;
  dynamicTelemetry.barometer = baro_mean;

  #ifdef DEBUG_CALIBRATION
  Serial.print("AccX (mean): ");
  Serial.print(acc_means[0]);
  Serial.print(" AccY (mean): ");
  Serial.print(acc_means[1]);
  Serial.print(" AccZ (mean): ");
  Serial.println(acc_means[2]);

  Serial.print("Acc norm X: ");
  Serial.print(norm_acc_x);
  Serial.print(" Acc norm Y: ");
  Serial.print(norm_acc_y);
  Serial.print(" Acc norm Z: ");
  Serial.println(norm_acc_z);


  Serial.print("GyroX (mean): ");
  Serial.print(gyro_means[0]);
  Serial.print(" GyroY (mean): ");
  Serial.print(gyro_means[1]);
  Serial.print(" GyroZ (mean): ");
  Serial.println(gyro_means[2]);

  Serial.print("Baro (mean): ");
  Serial.println(baro_mean);

  Serial.print("Acc bias vector:");
  Serial.print("X: ");
  Serial.print(accel_bias[0]);
  Serial.print(" Y: ");
  Serial.print(accel_bias[1]);
  Serial.print(" Z: ");
  Serial.println(accel_bias[2]);

  Serial.print("Orientation angles:");
  Serial.print("Pitch: ");
  Serial.print(dynamicTelemetry.orientation[0]);
  Serial.print(" Yaw: ");
  Serial.print(dynamicTelemetry.orientation[1]);
  Serial.print(" Roll: ");
  Serial.println(dynamicTelemetry.orientation[2]);

  Serial.print("Adjusted gravity vector:");
  Serial.print("X: ");
  Serial.print(adjusted_graviation[0]);
  Serial.print(" Y: ");
  Serial.print(adjusted_graviation[1]);
  Serial.print(" Z: ");
  Serial.println(adjusted_graviation[2]);
  
  #endif

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
    // Fast raw readings
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  dynamicTelemetry.valid = 1;
}

void sensorTask(void * args) {

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = PERIOD_SENSOR_READING;
  // filter.beta = BETA;
  filter.begin(SAMPLING_FREQUENCY);

  estimateMeasurementUncertainty();

  integration_timestamp = micros();

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    period_counter++;    
    last_telemetry_timestamp = micros();

    if(mpuReady){
      mpu.read();
      data = mpu.getRawData();
      // Now you can access the raw data like this:
      // data.AcX, data.AcY, data.AcZ, etc.
      // Update orientation and position with basic algorithm with gravitation vector detection
      if (period_counter > FULL_READING_PERIOD){
        pre_corrected_baro = data.Tmp/340.00+36.53;
        dynamicTelemetry.thermometer_stupido = BAR_COEFF * dynamicTelemetry.thermometer_stupido + (1.0f - BAR_COEFF) * pre_corrected_baro;
      }

      dynamicTelemetry.relativeTime = last_telemetry_timestamp;

      // Fill in pre correction accelerometer and gyroscope values
      dynamicTelemetry.angularVelocity[0] = (data.GyX*gyroScale - gyro_bias[0]);
      dynamicTelemetry.angularVelocity[1] = (data.GyY*gyroScale - gyro_bias[1]);
      dynamicTelemetry.angularVelocity[2] = (data.GyZ*gyroScale - gyro_bias[2]);


      #ifdef DEBUG_GYRO
      // For debug, print gyro measured corrected values, gyro bias values and computed angular velocity
      Serial.print(" GyroX: ");
      Serial.print(data.GyX*gyroScale);
      Serial.print(" GyroY: ");
      Serial.print(data.GyY*gyroScale);
      Serial.print(" GyroZ: ");
      Serial.print(data.GyZ*gyroScale);
      Serial.print(" GyroX (corrected): ");
      Serial.print(dynamicTelemetry.angularVelocity[0]);
      Serial.print(" GyroY (corrected): ");
      Serial.print(dynamicTelemetry.angularVelocity[1]);
      Serial.print(" GyroZ (corrected): ");
      Serial.print(dynamicTelemetry.angularVelocity[2]);
      Serial.print(" GyroX (bias): ");
      Serial.print(gyro_bias[0]);
      Serial.print(" GyroY (bias): ");
      Serial.print(gyro_bias[1]);
      Serial.print(" GyroZ (bias): ");
      Serial.println(gyro_bias[2]);
      #endif
      #ifdef DEBUG_ACCELEROMETER
      // For debug, print accelerometer measured values and computed acceleration
      Serial.print(" AccX: ");
      Serial.print(data.AcX*accScale);
      Serial.print(" AccY: ");
      Serial.print(data.AcY*accScale);
      Serial.print(" AccZ: ");
      Serial.print(data.AcZ*accScale);
      Serial.print(" AccX (corrected): ");
      Serial.print(dynamicTelemetry.acceleration[0]);
      Serial.print(" AccY (corrected): ");
      Serial.print(dynamicTelemetry.acceleration[1]);
      Serial.print(" AccZ (corrected): ");
      Serial.print(dynamicTelemetry.acceleration[2]);
      Serial.print(" AccX (bias): ");
      Serial.print(accel_bias[0]);
      Serial.print(" AccY (bias): ");
      Serial.print(accel_bias[1]);
      Serial.print(" AccZ (bias): ");
      Serial.println(accel_bias[2]);
      #endif

      dynamicTelemetry.acceleration[0] = (data.AcX*accScale - accel_bias[0]);
      dynamicTelemetry.acceleration[1] = (data.AcY*accScale - accel_bias[1]);
      dynamicTelemetry.acceleration[2] = (data.AcZ*accScale - accel_bias[2]);

      // Least Mean Square filter for gyro bias
      gyro_bias[0] += ALPHA_GYRX * clamp(dynamicTelemetry.angularVelocity[0], -MAX_GYRX, MAX_GYRX);
      gyro_bias[1] += ALPHA_GYRY * clamp(dynamicTelemetry.angularVelocity[1], -MAX_GYRY, MAX_GYRY);
      gyro_bias[2] += ALPHA_GYRZ * clamp(dynamicTelemetry.angularVelocity[2], -MAX_GYRZ, MAX_GYRZ);

      // We have to subtract the accel vector, let's assume the old orientation vector is enough
      // Apply low pass filter to gravitational magnitude
      gravitation[2] = ALPHA_GRAV * sqrt(dynamicTelemetry.acceleration[0] * dynamicTelemetry.acceleration[0] + dynamicTelemetry.acceleration[1] * dynamicTelemetry.acceleration[1] + dynamicTelemetry.acceleration[2] * dynamicTelemetry.acceleration[2]) + (1 - ALPHA_GRAV) * gravitation[2];
      
      memcpy(correction_gravitation, gravitation, sizeof(gravitation));
      applyRotationMatrix(correction_gravitation, dynamicTelemetry.orientation);

      // Now we subtract the gravitation in order for the filter to make sense ... I really tangled myself in this, lmao
      accel_bias[0] += ALPHA_ACCX * clamp(dynamicTelemetry.acceleration[0] - correction_gravitation[0], -MAX_ACCX, MAX_ACCX);
      accel_bias[1] += ALPHA_ACCY * clamp(dynamicTelemetry.acceleration[1] - correction_gravitation[1], -MAX_ACCY, MAX_ACCY);
      accel_bias[2] += ALPHA_ACCZ * clamp(dynamicTelemetry.acceleration[2] - correction_gravitation[2], -MAX_ACCZ, MAX_ACCZ);

      // Integrate acceleration to update velocity
      tmp_timestamp = micros();
      dt = (tmp_timestamp - integration_timestamp) / 1000000.0;
      integration_timestamp = tmp_timestamp;

      // Update the orientation estimation now
      filter.updateIMU(dynamicTelemetry.angularVelocity[0], dynamicTelemetry.angularVelocity[1], dynamicTelemetry.angularVelocity[2], dynamicTelemetry.acceleration[0], dynamicTelemetry.acceleration[1], dynamicTelemetry.acceleration[2], dt);

      dynamicTelemetry.orientation[0] = filter.getPitch();
      dynamicTelemetry.orientation[1] = filter.getYaw();
      dynamicTelemetry.orientation[2] = filter.getRoll();

      // Invert the angles, so we can transform the vectors into global frame of reference
      inverse_orientation[0] = -dynamicTelemetry.orientation[0];
      inverse_orientation[1] = -dynamicTelemetry.orientation[1];
      inverse_orientation[2] = -dynamicTelemetry.orientation[2];

      applyRotationMatrix(dynamicTelemetry.acceleration, inverse_orientation);

      // We assume that it's transformed to the global frame, hence gravity would be in the z-axis
      dynamicTelemetry.acceleration[2] -= gravitation[2];


      dynamicTelemetry.velocity[0] += dynamicTelemetry.acceleration[0] * dt;
      dynamicTelemetry.velocity[1] += dynamicTelemetry.acceleration[1] * dt;
      dynamicTelemetry.velocity[2] += dynamicTelemetry.acceleration[2] * dt;

      dynamicTelemetry.position[0] += dynamicTelemetry.velocity[0] * dt;
      dynamicTelemetry.position[1] += dynamicTelemetry.velocity[1] * dt;
      dynamicTelemetry.position[2] += dynamicTelemetry.velocity[2] * dt;


      #ifdef DEBUG_TELEMETRY
      Serial.print("PosX: ");
      Serial.print(dynamicTelemetry.position[0]);
      Serial.print(" PosY: ");
      Serial.print(dynamicTelemetry.position[1]);
      Serial.print(" PosZ: ");
      Serial.print(dynamicTelemetry.position[2]);
      Serial.print(" VelX: ");
      Serial.print(dynamicTelemetry.velocity[0]);
      Serial.print(" VelY: ");
      Serial.print(dynamicTelemetry.velocity[1]);
      Serial.print(" VelZ: ");
      Serial.print(dynamicTelemetry.velocity[2]);
      Serial.print(" AccX: ");
      Serial.print(dynamicTelemetry.acceleration[0]);
      Serial.print(" AccY: ");
      Serial.print(dynamicTelemetry.acceleration[1]);
      Serial.print(" AccZ: ");
      Serial.print(dynamicTelemetry.acceleration[2]);
      Serial.print("Pitch: ");
      Serial.print(dynamicTelemetry.orientation[0]);
      Serial.print(" Yaw: ");
      Serial.print(dynamicTelemetry.orientation[1]);
      Serial.print(" Roll: ");
      Serial.print(dynamicTelemetry.orientation[2]);
      Serial.print(" AngVelX: ");
      Serial.print(dynamicTelemetry.angularVelocity[0]);
      Serial.print(" AngVelY: ");
      Serial.print(dynamicTelemetry.angularVelocity[1]);
      Serial.print(" AngVelZ: ");
      Serial.println(dynamicTelemetry.angularVelocity[2]);
      #endif

    }
    
    if (period_counter > FULL_READING_PERIOD){
      period_counter = 0;
      if(bmpReady){
        if (bmp.takeForcedMeasurement()) {
          // can now print out the new measurements
          dynamicTelemetry.thermometer = bmp.readTemperature();
           // dynamicTelemetry.barometer =  pressureKalmanFilter.updateEstimate(bmp.readPressure());
           dynamicTelemetry.barometer =  bmp.readPressure();
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

      #ifdef DEBUG_SENSOR
      mpu.printData();
      // Check the remaining stack space
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.print("Sensor Task Stack High Water Mark: ");
      Serial.println(uxHighWaterMark);
      #endif
    }
    // If debug flag set, print the elapsed time
    #ifdef DEBUG_SENSOR_TIMING
    Serial.print("Sensor task elapsed time: ");
    Serial.println(micros() - last_telemetry_timestamp);
    #endif
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// This function creates telemetryMessage struct
void readTelemetry(){
  memcpy(&currentTelemetry, &dynamicTelemetry, sizeof(telemetryMessage));
  currentTelemetry.id = id_counter++;
}
