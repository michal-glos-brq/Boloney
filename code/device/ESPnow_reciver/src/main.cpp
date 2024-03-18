#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <mutex>
#include <main.h>

#define BUFFER_CAPACITY 100
#define SLEEP_PERIOD 66

#define DEBUG

uint8_t broadcastAddress[] = MAC_ADDRESS_SND;
int top = -1;


std::mutex mtx_stack;


// MAC Address of the ESP32 devboard (cap on enable pin): 24:6F:28:25:E4:90
// MAC Address of the ESP32 devboard: C8:F0:9E:9B:32:04
// MAC Address of the ESP32 LoRa Board: 50:02:91:8A:F7:40

// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage * message){
  return esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

void print_entry(telemetryMessage * msg) {
  // Format the entire CSV line in one go:
  String csvLine = String(msg->id) + ";" +
                   String(msg->relativeTime) + ";" +
                   String(msg->gyroscope[0]) + ";" +
                   String(msg->gyroscope[1]) + ";" +
                   String(msg->gyroscope[2]) + ";" +
                   String(msg->accelerometer[0]) + ";" +
                   String(msg->accelerometer[1]) + ";" +
                   String(msg->accelerometer[2]) + ";" +
                   String(msg->barometer) + ";" +
                   String(msg->thermometer) + ";" +
                   String(msg->thermometer_stupido) + ";" +
                   String(msg->voltage) + "\n";

   // Print the complete line at once:
   Serial.print(csvLine); 
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  mtx_stack.lock();
  if (top >= BUFFER_CAPACITY) {
    mtx_stack.unlock();
    return;
  }

  memcpy(&(telemetryArray[top]), incomingData, sizeof(telemetryMessage));
  #ifdef DEBUG
  Serial.print("Received msg: ");
  Serial.println(telemetryArray[top].id);
  #endif

  print_entry(&(telemetryArray[top++]));
  mtx_stack.unlock();

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

  mtx_stack.lock();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  #ifdef DEBUG
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  #endif

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  telemetryArray = (telemetryMessage *) calloc(BUFFER_CAPACITY, sizeof(telemetryMessage));
  currentTelemetry.valid = 0;
  SendHeader();

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return (void) 1;
  }
  mtx_stack.unlock();
}

void loop() {
  // put your main code here, to run repeatedly:
    // After delivery succesfull to the I2C, let's ack the message (might cause some trouble doh)
  
  mtx_stack.lock();
  esp_err_t err;
  while (top > 0) {
    memcpy(&currentTelemetry, &(telemetryArray[--top]), sizeof(telemetryMessage));
    err = sendMessage(&currentTelemetry);
  }
  mtx_stack.unlock();



  delay(SLEEP_PERIOD);
}