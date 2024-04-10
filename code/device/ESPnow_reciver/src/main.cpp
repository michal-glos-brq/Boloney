#include "main.h"

#define BUFFER_CAPACITY 200

// Uncomment for debug mode
// #define DEBUG

uint8_t broadcastAddress[] = MAC_ADDRESS_SND;
int top = -1;
bool activated = false;

std::mutex mtx_stack;

// MAC Address of the ESP32 devboard (cap on enable pin): 24:6F:28:25:E4:90
// MAC Address of the ESP32 devboard: C8:F0:9E:9B:32:04
// MAC Address of the ESP32 LoRa Board: 50:02:91:8A:F7:40

// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage *message)
{
  #ifdef DEBUG
    Serial.print("Sending msg: ");
    Serial.println(message->id);
  #endif
  return esp_now_send(broadcastAddress, (uint8_t *)message, sizeof(telemetryMessage));
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  #ifdef DEBUG
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
}

void print_entry(telemetryMessage *msg)
{
  // Format the entire CSV line in one go:
  String csvLine = String(msg->id) + ";" +
                   String(msg->relativeTime) + ";" +
                   String(msg->orientation[0]) + ";" +
                   String(msg->orientation[1]) + ";" +
                   String(msg->orientation[2]) + ";" +
                   String(msg->angularVelocity[0]) + ";" +
                   String(msg->angularVelocity[1]) + ";" +
                   String(msg->angularVelocity[2]) + ";" +
                   String(msg->position[0]) + ";" +
                   String(msg->position[1]) + ";" +
                   String(msg->position[2]) + ";" +
                   String(msg->velocity[0]) + ";" +
                   String(msg->velocity[1]) + ";" +
                   String(msg->velocity[2]) + ";" +
                   String(msg->barometer) + ";" +
                   String(msg->thermometer) + ";" +
                   String(msg->thermometer_stupido) + ";" +
                   String(msg->voltage) + "\n";

  // Print the complete line at once:
  Serial.print(csvLine);
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  #ifdef DEBUG
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; i++)
    {
      Serial.print(mac[i], HEX);
      if (i < 5)
        Serial.print(":");
    }
    Serial.println();
  #endif

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

void SendHeader()
{
  Serial.println("####################");
  Serial.println("New session started!");
  Serial.println("####################");
}


void setupComms() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  telemetryArray = (telemetryMessage *)calloc(BUFFER_CAPACITY, sizeof(telemetryMessage));
  currentTelemetry.valid = 0;
  SendHeader();

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return (void)1;
  }
  delay(333);
  activated = true;
}

void setup()
{
  Serial.begin(115200);

#ifdef DEBUG
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("MAC address: ");
    for (int i = 0; i < 6; i++)
    {
      Serial.print(mac[i], HEX);
      if (i < 5)
        Serial.print(":");
    }
    Serial.println();
#endif

  setupComms();

}

// Intentionally empty
void loop() {

  // Wait till activated
  while (!activated) {delay(100);};

  esp_err_t err;
  mtx_stack.lock();
  
  while (top > 0) {
    memcpy(&currentTelemetry, &(telemetryArray[--top]), sizeof(telemetryMessage));
    sendMessage(&currentTelemetry);
  }  
  
  mtx_stack.unlock();
}