#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <main.h>

#define SEMICOLON Serial.print(';');
#define NEWLINE Serial.println();

// MAC Address of the ESP32 devboard (cap on enable pin): 24:6F:28:25:E4:90
// MAC Address of the ESP32 devboard: C8:F0:9E:9B:32:04
// MAC Address of the ESP32 LoRa Board: 50:02:91:8A:F7:40

// Create a struct_message called myData
telemetryMessage * currentStructure = (telemetryMessage *) malloc(sizeof(telemetryMessage));

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(currentStructure, incomingData, sizeof(telemetryMessage));
  // Write the structure out as a CSV line
  Serial.print(currentStructure->relativeTime); SEMICOLON
  Serial.print(currentStructure->gyroscope[0]); SEMICOLON
  Serial.print(currentStructure->gyroscope[1]); SEMICOLON
  Serial.print(currentStructure->gyroscope[2]); SEMICOLON
  Serial.print(currentStructure->accelerometer[0]); SEMICOLON
  Serial.print(currentStructure->accelerometer[1]); SEMICOLON
  Serial.print(currentStructure->accelerometer[2]); SEMICOLON
  Serial.print(currentStructure->barometer); SEMICOLON
  Serial.print(currentStructure->thermometer); SEMICOLON
  Serial.print(currentStructure->thermometer_stupido); SEMICOLON
  Serial.print(currentStructure->voltage); NEWLINE
}

void SendHeader(){
  Serial.print("Timestamp"); SEMICOLON
  Serial.print("GyroscopeX"); SEMICOLON
  Serial.print("GyroscopeY"); SEMICOLON
  Serial.print("GyroscopeZ"); SEMICOLON
  Serial.print("AccelerometerX"); SEMICOLON
  Serial.print("AccelerometerY"); SEMICOLON
  Serial.print("AccelerometerZ"); SEMICOLON
  Serial.print("Barometer"); SEMICOLON
  Serial.print("Thermometer"); SEMICOLON
  Serial.print("ThermometerStupido"); SEMICOLON
  Serial.print("Voltage"); NEWLINE
}

void setup() {
  Serial.begin(115200);

  /*
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  */

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  SendHeader();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:
}