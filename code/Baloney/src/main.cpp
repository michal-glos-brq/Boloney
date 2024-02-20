#include <Arduino.h>
#include <Wire.h>
// inicializace proměnné pro určení adresy senzoru
// 0x68 nebo 0x69, dle připojení AD0
const int MPU_addr=0x68;
// inicializace proměnných, do kterých se uloží data
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup()
{
  // komunikace přes I2C sběrnici
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  // komunikace přes sériovou linku rychlostí 115200 baud
  Serial.begin(115200);
}

void loop()
{
  // zapnutí přenosu
  Wire.beginTransmission(MPU_addr);
  // zápis do registru ACCEL_XOUT_H
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // vyzvednutí dat z 14 registrů
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  // výpis surových dat z proměnných na sériovou linku
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  // přepočtení teploty dle datasheetu
  Serial.print(" | Temp = "); Serial.print(Tmp/340.00+36.53);
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(500);
}