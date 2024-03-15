#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <main.h>


uint8_t broadcastAddress[] = MAC_ADDRESS;

esp_now_peer_info_t peerInfo;

// This function creates telemetryMessage struct 
telemetryMessage * readTelemetry(){
  telemetryMessage * dataEntry = (telemetryMessage *)malloc(sizeof(telemetryMessage));
  dataEntry->relativeTime = micros();
  // Mockity mock
  dataEntry->valid = true;
  dataEntry->accelerometerX = 1;
  dataEntry->accelerometerY = 2;
  dataEntry->accelerometerZ = 3;
  dataEntry->gyroscopeX = 4;
  dataEntry->gyroscopeY = 5;
  dataEntry->gyroscopeZ = 6;
  dataEntry->barometer = 7;
  dataEntry->thermometer = 8.0;
  dataEntry->thermometer_stupido = 9.0;
  dataEntry->voltage = 10.0;
  return dataEntry;
}

// This will add data message to the array, eventually deleting oldest entry
int pushMessage(telemetryMessage * message){
  int i;
  // Here it just looks for empty space
  for (i = 0; i < BUFFER_CAPACITY; i++){
    if (!(telemetryArray[i].valid)){
      telemetryArray[i] = *message;
      return 0;
    }
  }

  // Now if no empty space found, we have to delete the oldest msg
  // Assuming all are valid ...
  int tmpID = 0;
  uint64_t min_time = telemetryArray[i].relativeTime;
  for (i = 1; i < BUFFER_CAPACITY; i++){
    if (telemetryArray[i].relativeTime < min_time){
      min_time = telemetryArray[i].relativeTime;
      tmpID = i;
    }
  }
  telemetryArray[tmpID] = *message;
  return 1;
}

// This sends a sinlge message, returns the send method return value
// (success = 0, fail = 1) for further fail detection
int sendMessage(struct telemetryMessage * message){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
     
  if (result == ESP_OK) {
    return 0;
  }
  else
  {
    return 1;
  }
}

// Goes through the array of structs and trys to send all the structs, one after the other
// Reads the sendMessage output, and if succeeded, remove the entry from the array
void sendMessages(){
  for (int i = 0; i < BUFFER_CAPACITY; i++){
    if (telemetryArray[i].valid){
      if (sendMessage(&(telemetryArray[i])) == 0){
        telemetryArray[i].valid = false;
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

  telemetryArray = (telemetryMessage *)malloc(BUFFER_CAPACITY * sizeof(telemetryMessage));

  // Set all entries from array to NULLs
  for (int i = 0; i < BUFFER_CAPACITY; i++){
    telemetryArray->valid = false;
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

  pushMessage(dato);

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
