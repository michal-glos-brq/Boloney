#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <main.h>

#define BUFFER_CAPACITY 480
#define PERIOD_TIME 250
#define MIN_WAIT_TIME 25

// Debug compilation (comment to turn debug off)
#define DEBUG

uint8_t broadcastAddress[] = MAC_ADDRESS_RCV;

uint64_t id_counter = 0;

uint64_t period_time_micros = PERIOD_TIME * 1000;
uint64_t min_wait_time_micros = MIN_WAIT_TIME * 1000;

uint64_t current_loop_start;

// This function creates telemetryMessage struct 
void readTelemetry(){

  // Some actual logic will have to be implemented here ...
  // message->relativeTime = micros();
  currentTelemetry.relativeTime = micros();
  currentTelemetry.id = id_counter++;
  currentTelemetry.valid = 1;
  // Mockity mock
  currentTelemetry.accelerometer[0] = 1;
  currentTelemetry.accelerometer[1] = 2;
  currentTelemetry.accelerometer[2] = 3;
  currentTelemetry.gyroscope[0] = 4;
  currentTelemetry.gyroscope[1] = 5;
  currentTelemetry.gyroscope[2] = 6;
  currentTelemetry.barometer = 7;
  currentTelemetry.thermometer = 8.0;
  currentTelemetry.thermometer_stupido = 9.0;
  currentTelemetry.voltage = 10.0;
}


// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage * message){
  return esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    #ifdef DEBUG
    Serial.print("Data sent status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    #endif
}


// This function is OnReceive callback and invalidates the message from buffer 
// with received ID (as long int) as a form of ack
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  telemetryAck * message = (telemetryAck *) incomingData;

  if (currentTelemetry.id == message->id) {
    #ifdef DEBUG
    Serial.println("Ack for the last msg.");
    #endif
    currentTelemetry.valid = 0;
    return;
  }

  #ifdef DEBUG
  Serial.print("Ack for the msg: ");
  Serial.println(message->id);
  #endif

  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].id == message->id) {
      telemetryArray[i].valid = 0;
      break;
    }
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
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
    return (void) 1;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  telemetryArray = (telemetryMessage *) calloc(BUFFER_CAPACITY, sizeof(telemetryMessage));
  currentTelemetry.valid = 0;

  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    telemetryArray[i].valid = 0;
  }

  readTelemetry();

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return (void) 1;
  }
}

void saveMessage() {
  // Saves current message to the buffer
  if (currentTelemetry.valid == 0) {
    return;
  }

  #ifdef DEBUG
  Serial.println("Warning! Message not confirmed - saving into buffer");
  #endif

  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].valid == 0) {
      // Copy currentTelemetry to the telemetry array at I place with memcpy
      memcpy(&telemetryArray[i], &currentTelemetry, sizeof(telemetryMessage));
      return;
    }
  }
  // If buffer is full, let's replace the oldest entry
  #ifdef DEBUG
  Serial.println("Warning! Buffer full - overwriting");
  #endif

  uint64_t tmpTime = telemetryArray[0].relativeTime;
  int tmpIndex = 0;
  for (int i = 1; i < BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].relativeTime < tmpTime) {
      tmpTime = telemetryArray[i].relativeTime;
      tmpIndex = i;
    }
  }
  memcpy(&telemetryArray[tmpIndex], &currentTelemetry, sizeof(telemetryMessage));
  return;
 }

void sendMessages() {
  // Try to send all valid messages (older then ) from the buffer
  #ifdef DEBUG
  Serial.print("Sending: ");
  #endif
  
  for (int i = 0; i < BUFFER_CAPACITY; i++) {

    if (telemetryArray[i].valid == 1) {
      sendMessage(&telemetryArray[i]);
      #ifdef DEBUG
      Serial.print(i);
      Serial.print(", ");
      #endif
    }
    // Check whether half time from sleep time already elapsed, if so, break and return
    if ((micros() - current_loop_start) > min_wait_time_micros) {
      break;
    }
  }
  #ifdef DEBUG
  Serial.println();
  #endif
}

void loop() {
  current_loop_start = micros();

  #ifdef DEBUG
  Serial.println("++++++++++++++++++++++++++++++++++++++++");
  Serial.print("Iteration: ");
  Serial.println(id_counter);
  Serial.print("1) Occupied indices:");
  for (int i = 0; i<BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].valid == 1) {
      Serial.print(i);
      Serial.print(", ");
    }
  }
  Serial.println();
  #endif

  // Save the previous message if not acked
  saveMessage();

  // Try to send stuff from the buffer
  sendMessages();

  // Try to send current telemetry
  readTelemetry();
  esp_now_peer_info();
  esp_err_t succ = sendMessage(&currentTelemetry);

  #ifdef DEBUG
  if (succ == ESP_OK) {
    Serial.println("Sent OK ...");
  }
  else {
    Serial.println("Sent not OK ...");
  }
  #endif

  // Sleep for the remainder of time
  delay((period_time_micros - (micros() - current_loop_start)) / 1000);
}
