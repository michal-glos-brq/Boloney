#include <Arduino.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

MPU6050 mpu;    // I2C
Adafruit_BMP280 bmp; // I2C

bool mpuReady = false;
bool bmpReady = false;
MPU6050::struct_raw_MPU data;

void setup() {
   Wire.begin(8, 10); // SDA, SCL
  if(mpu.begin()){
    Serial.println("MPU6050 is ready!");
    mpuReady = true;
  } else {
    Serial.println("MPU6050 is not ready!");
  }

     //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (bmp.begin()) {
    Serial.println("BMP280 is ready!");
    bmpReady = true;
  } else {
    Serial.println("BMP280 is not ready!");
  }

  if(bmpReady){
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
}

void loop() {
  if(mpuReady){
    mpu.read();
    mpu.printData();
    data = mpu.getRawData();
    // Now you can access the raw data like this:
    // data.AcX, data.AcY, data.AcZ, etc.
  }
    
  if(bmpReady){
    if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.print(" *C, ");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.print(" Pa, ");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
  } else {
    Serial.println("Forced measurement failed!");
  }
  }
    
}
