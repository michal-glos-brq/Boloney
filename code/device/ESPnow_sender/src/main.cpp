#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <main.h>

uint8_t broadcastAddress[] = MAC_ADDRESS;

// This function creates telemetryMessage struct 
void readTelemetry(telemetryMessage * message){
  if(message->valid == 1){
    Serial.print("Rewriting valid message ...");
    exit(1);
  }

  // Some actual logic will have to be implemented here ...
  // message->relativeTime = micros();
  message->relativeTime = counter;
  // Mockity mock
  message->valid = 1;
  message->accelerometerX = 1;
  message->accelerometerY = 2;
  message->accelerometerZ = 3;
  message->gyroscopeX = 4;
  message->gyroscopeY = 5;
  message->gyroscopeZ = 6;
  message->barometer = 7;
  message->thermometer = 8.0;
  message->thermometer_stupido = 9.0;
  message->voltage = 10.0;
}


// This sends a sinlge message, returns the send method return value
// (success = 0, fail = 1) for further fail detection
int sendMessage(struct telemetryMessage * message){
  return (int)esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Iteration: ");
  Serial.println(counter);
  Serial.print("Failed: ");
  Serial.println(failed);
  Serial.print("Failed to save: ");
  Serial.println(failed_save);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return (void) 1;
  }

  counter = 0;
  failed = 0;
  failed_save = 0;
  telemetryArray = (telemetryMessage *)malloc(BUFFER_CAPACITY * sizeof(telemetryMessage));
  currentTelemetry.valid = 0;
  readTelemetry(&currentTelemetry);

  // Set all entries from array to NULLs
  for (int i = 0; i < BUFFER_CAPACITY; i++){
    Serial.print(i);
    telemetryArray[i].valid = 0;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

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

int saveMessage(telemetryMessage * msg){
  int i;
  Serial.print(telemetryArray[0].valid);
  Serial.print(";");
  Serial.print(telemetryArray[1].valid);
  Serial.print(";");
  Serial.print(telemetryArray[2].valid);
  Serial.print(";");
  Serial.print(telemetryArray[3].valid);
  Serial.print(";");
  Serial.print(telemetryArray[4].valid);
  Serial.print(";");
  Serial.print(telemetryArray[5].valid);
  Serial.print(";");
  Serial.print(telemetryArray[6].valid);
  for (i = 0;i < BUFFER_CAPACITY && telemetryArray[i].valid == 1; i++);
  if(i < BUFFER_CAPACITY) {
    Serial.print("Position in array: ");
    Serial.print(i);
    telemetryArray[i] = currentTelemetry;
    // memcpy(&currentTelemetry, &(telemetryArray[i]), sizeof(telemetryMessage));
    return 0;
  }
  Serial.print("What the hell");
  int ii = 0;
  uint64_t tmpTime = UINT64_MAX;
  for(i = 0; i < BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].relativeTime < tmpTime) {
      tmpTime = telemetryArray[i].relativeTime;
      ii = i;
    }
  }
  telemetryArray[ii] = currentTelemetry;
  //memcpy(&currentTelemetry, &(telemetryArray[ii]), sizeof(telemetryMessage));
  // One got flagged as invalid now
  return 1;
}

void loop() {
  currentTelemetry.valid = 0;
  readTelemetry(&currentTelemetry);

  esp_now_peer_info();
  // If send success, try to send history buffer
  if(sendMessage(&currentTelemetry)){
    // Send messages
    Serial.println("Msg succeeded\n");
  }
  else{
    Serial.println("Msg failed\n");
    failed++;
    failed_save += saveMessage(&currentTelemetry);
  }
  counter++;
  delay(SLEEP_TIME);
}
