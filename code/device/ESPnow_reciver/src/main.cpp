#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <main.h>

uint8_t broadcastAddress[] = MAC_ADDRESS_SND;


// MAC Address of the ESP32 devboard (cap on enable pin): 24:6F:28:25:E4:90
// MAC Address of the ESP32 devboard: C8:F0:9E:9B:32:04
// MAC Address of the ESP32 LoRa Board: 50:02:91:8A:F7:40

// Create a struct_message called myData
telemetryMessage currentStructure;
telemetryAck ack;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&currentStructure, incomingData, sizeof(telemetryMessage));

  // Format the entire CSV line in one go:
  String csvLine = String(currentStructure.id) + ";" +
                   String(currentStructure.relativeTime) + ";" +
                   String(currentStructure.gyroscope[0]) + ";" +
                   String(currentStructure.gyroscope[1]) + ";" +
                   String(currentStructure.gyroscope[2]) + ";" +
                   String(currentStructure.accelerometer[0]) + ";" +
                   String(currentStructure.accelerometer[1]) + ";" +
                   String(currentStructure.accelerometer[2]) + ";" +
                   String(currentStructure.barometer) + ";" +
                   String(currentStructure.thermometer) + ";" +
                   String(currentStructure.thermometer_stupido) + ";" +
                   String(currentStructure.voltage) + "\n";

   // Print the complete line at once:
   Serial.print(csvLine); 

  // After delivery succesfull to the I2C, let's ack the message (might cause some trouble doh)
  ack.id = currentStructure.id;
  esp_now_send(broadcastAddress, (uint8_t *) &(ack), sizeof(struct_ack));
}

void SendHeader() {
  Serial.print("ID;");
  Serial.print("Timestamp;");
  Serial.print("GyroscopeX;");
  Serial.print("GyroscopeY;");
  Serial.print("GyroscopeZ;");
  Serial.print("AccelerometerX;");
  Serial.print("AccelerometerY;");
  Serial.print("AccelerometerZ;");
  Serial.print("Barometer;");
  Serial.print("Thermometer;");
  Serial.print("ThermometerStupido;");
  Serial.println("Voltage"); 
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