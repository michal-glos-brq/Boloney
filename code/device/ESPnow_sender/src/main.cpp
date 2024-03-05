#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <main.h>


uint32_t * messagesLost = 0;

uint8_t broadcastAddress[] = MAC_ADDRESS;

esp_now_peer_info_t peerInfo;

// This function creates telemetryMessage struct 
struct telemetryMessage * readTelemetry(){
  struct telemetryMessage * dataEntry;
  dataEntry->relativeTime = micros();
  // Mockity mock
  dataEntry->accelerometer[0] = 1;
  dataEntry->accelerometer[1] = 2;
  dataEntry->accelerometer[2] = 3;
  dataEntry->gyroscope[0] = 4;
  dataEntry->gyroscope[1] = 5;
  dataEntry->gyroscope[2] = 6;
  dataEntry->barometer = 7;
  dataEntry->thermometer = 8.0;
  dataEntry->thermometer_stupido = 9.0;
  dataEntry->voltage = 10.0;
  return dataEntry;
}

// This will add data message to the array, eventually deleting oldest entry
int * pushMessage(telemetryMessage * message){
  int i = 0;
  // Here it just looks for empty space
  for (i = 0; i < BUFFER_CAPACITY; i++){
    if (telemetryArray[i] == NULL){
      telemetryArray[i] = message;
      return 0;
    }
  }
  // Now if no empty space found, we have to delete the oldest msg
  int tmpID = 0;
  uint64_t min_time = telemetryArray[i]->relativeTime;
  for (i = 1; i < BUFFER_CAPACITY; i++){
    if (telemetryArray[i]->relativeTime < min_time){
      min_time = telemetryArray[i]->relativeTime;
      tmpID = i;
    }
  }
  telemetryArray[tmpID] = message;
  return (int *) 1;
}

// This sends a sinlge message, returns the send method return value
// (success = 0, fail = 1) for further fail detection
int * sendMessage(struct telemetryMessage * message){
  return 0;
}

// Goes through the array of structs and trys to send all the structs, one after the other
// Reads the sendMessage output, and if succeeded, remove the entry from the array
void sendMessages(){
  for (int i = 0; i < BUFFER_CAPACITY; i++){
    if (telemetryArray[i] != NULL){
      if (sendMessage(telemetryArray[i])){
        telemetryArray[i] = NULL;
      }
    }
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

void loop() {
  // Read telemetry struct and put it into first free array slot
  // Conflists will yet have to be solved, maybe "least recently used"

  // strPWDcpy(myData.a, "THIS IS A CHAR");
  // myData.b = random(1,20);
  // myData.c = 1.2;
  // myData.d = false;


  // Here we just add the telemetry to the array
  // And increment messageLost in case of deleting old message (unsent)
  telemetryMessage * dato = readTelemetry();
  int * rv = pushMessage(dato);
  messagesLost += (uint32_t)*rv;


  esp_now_peer_info();
  
  sendMessages(); // The logic below will be in send Message
  // Then it will be called in loop in function sendMessages

  // // Send message via ESP-NOW
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  // delay(2000);
}
