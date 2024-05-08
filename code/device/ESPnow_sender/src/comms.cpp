#include "comms.h"


esp_now_peer_info_t peerInfo;

// Comms data structure holders
telemetryMessage * telemetryArray;

long int msg_timing_diff;
uint64_t current_comms_cycle_start, last_ack_received;

uint8_t broadcastAddress[] = MAC_ADDRESS_RCV;


void saveMessage() {
  // Saves current message to the buffer
  if (currentTelemetry.valid == 0) {
    return;
  }
  else {
    #ifdef DEBUG_COMMS
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
    #ifdef DEBUG_COMMS
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
 }

// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage * message){
  #ifdef DEBUG_MSG
  Serial.print("Sending message: ");Serial.print(message->id);Serial.print(" time: ");Serial.print(message->relativeTime);Serial.print(" acc: ");Serial.print(message->acceleration[0]);Serial.print(", ");Serial.print(message->acceleration[1]);Serial.print(", ");Serial.print(message->acceleration[2]);Serial.print(" pos: ");Serial.print(message->position[0]);Serial.print(", ");Serial.print(message->position[1]);Serial.print(", ");Serial.print(message->position[2]);Serial.print(" vel: ");Serial.print(message->velocity[0]);Serial.print(", ");Serial.print(message->velocity[1]);Serial.print(", ");Serial.print(message->velocity[2]);Serial.print(" ang: ");Serial.print(message->angularVelocity[0]);Serial.print(", ");Serial.print(message->angularVelocity[1]);Serial.print(", ");Serial.print(message->angularVelocity[2]);Serial.print(" ori: ");Serial.print(message->orientation[0]);Serial.print(", ");Serial.print(message->orientation[1]);Serial.print(", ");Serial.print(message->orientation[2]);Serial.print(" bar: ");Serial.print(message->barometer);Serial.print(" temp: ");Serial.print(message->thermometer);Serial.print(" temp2: ");Serial.print(message->thermometer_stupido);Serial.print(" volt: ");Serial.println(message->voltage);
  #endif
  return esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}

void sendMessages() {
  // Try to send all valid messages (older then ) from the buffer
  #ifdef DEBUG_COMMS
  Serial.print("2) Sending: ");
  #endif
  
  //xSemaphoreTake(semaphores.mtx_stack, portMAX_DELAY);
  for (int i = BUFFER_CAPACITY; i >= 0; i--) {
    if (telemetryArray[i].valid == 1) {
      sendMessage(&telemetryArray[i]);
      #ifdef DEBUG_COMMS
      Serial.print(telemetryArray[i].id);
      Serial.print(", ");
      #endif
    }
    // Check whether half time from sleep time already elapsed, if so, break and return
   if ((micros() - current_comms_cycle_start) > MIN_SILENT_TIME_US) {
      break;
    }
  }

  #ifdef DEBUG_COMMS
  Serial.println("FInished sendMessages ...");
  #endif
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// This function is OnReceive callback and invalidates the message from buffer 
// with received ID (as long int) as a form of ack
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  last_ack_received = micros();
  telemetryMessage * message = (telemetryMessage *) incomingData;

  if (currentTelemetry.id == message->id) {
    #ifdef DEBUG_COMMS
    Serial.print("Ack for last message: ");
    Serial.println(currentTelemetry.id);
    #endif
    currentTelemetry.valid = 0;
    return;
  }
  else {
    #ifdef DEBUG_COMMS
    Serial.print("Ack for message: ");
    Serial.println(message->id);
    #endif

    
    for (int i = 0; i < BUFFER_CAPACITY; i++) {
      if (telemetryArray[i].id == message->id) {
        telemetryArray[i].valid = 0;
        break;
      }
    }
  }
}



void commsTask(void * args) {


  while (1) {
    current_comms_cycle_start = micros();

    #ifdef DEBUG_COMMS
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("MAC address: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(mac[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.println();

    Serial.println("++++++++++++++++++++++++++++++++++++++++");
    Serial.print("Iteration: ");
    Serial.println(id_counter);
    Serial.print("1) buffered messages:");
    for (int i = 0; i<BUFFER_CAPACITY; i++) {
      if (telemetryArray[i].valid == 1) {
        Serial.print(telemetryArray[i].id);
        Serial.print(", ");
      }
    }
    Serial.println();
    #endif

    if (BUFFER_CAPACITY != 0) {
      // Save the previous message if not acked
      saveMessage();

      // Try to send stuff from the buffer
      // So even when the cappacity is set, do not try to dump all when
      // message not obtained for CONNECTION_TIMEOUT ms.
      sendMessages();
    }

    // Try to send current telemetry
    readTelemetry();

    esp_now_peer_info();  
    esp_err_t succ = sendMessage(&currentTelemetry);

    #ifdef DEBUG_COMMS
    if (succ == ESP_OK) {
      Serial.println("Sent OK ...");
    }
    else {
      Serial.println("Sent not OK ...");
    }

    // Check the remaining stack space
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("Comm Task Stack High Water Mark: ");
    Serial.println(uxHighWaterMark);
    #endif

    // Sleep for the remainder of time
    msg_timing_diff = (COMMS_PERIOD_US - (micros() - current_comms_cycle_start)) / 1000;
      
    if (msg_timing_diff > 0) {
      #ifdef DEBUG_COMMS
      Serial.print("Sleeping for: ");
      Serial.print(msg_timing_diff);
      Serial.println(" ms");
      #endif
      vTaskDelay(msg_timing_diff);
    }
    else {
      #ifdef DEBUG_COMMS
      Serial.println("Warning! Loop took too long ...");
      #endif
    }
  }
}



void setupComms() {
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

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return (void) 1;
  }
  last_ack_received = micros();
}
