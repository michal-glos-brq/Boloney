#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <main.h>
#include <mutex>

#define BUFFER_CAPACITY 100
#define PERIOD_TIME 2500
#define MIN_WAIT_TIME 1250 // For sending current message basically

// Debug compilation (comment to turn debug off)
#define DEBUG


// Mutex for concurrent stack access
std::mutex mtx_stack, mtx_struct, mtx_sensor_acc;

typedef struct sensorACC {
  // We will actually have to integrate ... sp basically:
    // gyroscope is orientation in space (apprx.)
    // accelerometer (with comnination of gyroscope) is position
  double gyroscope[3];
  double accelerometer[3];
  // Last timestep - neccessary for integration
  uint64_t microseconds;
} sensorACC;

sensorACC acc;

uint8_t broadcastAddress[] = MAC_ADDRESS_RCV;

uint64_t id_counter = 0;

// Timing stuff
uint64_t period_time_micros = PERIOD_TIME * 1000;
uint64_t min_wait_time_micros = MIN_WAIT_TIME * 1000;
uint64_t current_loop_start;

// This function creates telemetryMessage struct 
void readTelemetry(){
  mtx_struct.lock();
  // Some actual logic will have to be implemented here ...
  // message->relativeTime = micros();
  currentTelemetry.relativeTime = micros();
  currentTelemetry.id = id_counter++;
  currentTelemetry.valid = 1;
  // Transfer integrated data from acc and gyro
  mtx_sensor_acc.lock();
  memcpy(currentTelemetry.accelerometer, acc.accelerometer, 3 * sizeof(long));
  memcpy(currentTelemetry.gyroscope, acc.gyroscope, 3 * sizeof(long));
  mtx_sensor_acc.unlock();

  // Mockity mock
  currentTelemetry.barometer = 7;
  currentTelemetry.thermometer = 8.0;
  currentTelemetry.thermometer_stupido = 9.0;
  currentTelemetry.voltage = 10.0;
  mtx_struct.unlock();
}


// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage * message){
  return esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}


// This function is OnReceive callback and invalidates the message from buffer 
// with received ID (as long int) as a form of ack
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  telemetryMessage * message = (telemetryMessage *) incomingData;

  mtx_struct.lock();
  if (currentTelemetry.id == message->id) {
    #ifdef DEBUG
    Serial.print("Ack for last message: ");
    Serial.println(currentTelemetry.id);
    #endif
    currentTelemetry.valid = 0;
    mtx_struct.unlock();
    return;
  }
  else {
    mtx_struct.unlock();

    #ifdef DEBUG
    Serial.print("Ack for message: ");
    Serial.println(message->id);
    #endif

    mtx_stack.lock();
    for (int i = 0; i < BUFFER_CAPACITY; i++) {
      if (telemetryArray[i].id == message->id) {
        telemetryArray[i].valid = 0;
        break;
      }
    }
    mtx_stack.unlock();
  }

}

void setupReading() {
  // Setup the sensory integrators and the RTOS job
  mtx_sensor_acc.lock();
  acc.microseconds = micros();
  // Position set to [0, 0, 0]
  acc.accelerometer[0] = 0.; acc.accelerometer[1] = 0.; acc.accelerometer[2] = 0.;
  // Orientation set to [0, 0, -1] - which is basically down in Z direction
  acc.gyroscope[0] = 0.; acc.gyroscope[1] = 0.; acc.gyroscope[2] = -1.;
  mtx_sensor_acc.unlock();

  // TODO: Add RTOS task
}

void setup() {

  mtx_stack.lock();
  mtx_struct.lock();

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

  setupReading();

  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    telemetryArray[i].valid = 0;
  }

  mtx_struct.unlock();
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

  mtx_stack.unlock();
}

void saveMessage() {
  // Saves current message to the buffer
  mtx_struct.lock();
  if (currentTelemetry.valid == 0) {
    mtx_struct.unlock();
    return;
  }
  else {
    mtx_struct.unlock();
    #ifdef DEBUG
    Serial.println("Warning! Message not confirmed - saving into buffer");
    #endif

    for (int i = 0; i < BUFFER_CAPACITY; i++) {
      mtx_stack.lock();
      if (telemetryArray[i].valid == 0) {
        // Copy currentTelemetry to the telemetry array at I place with memcpy
        mtx_struct.lock();
        memcpy(&telemetryArray[i], &currentTelemetry, sizeof(telemetryMessage));
        mtx_struct.unlock();
        mtx_stack.unlock();
        return;
      }
      else {
        mtx_stack.unlock();
      }
    }
    
    // If buffer is full, let's replace the oldest entry
    #ifdef DEBUG
    Serial.println("Warning! Buffer full - overwriting");
    #endif

    mtx_stack.lock();
    uint64_t tmpTime = telemetryArray[0].relativeTime;
    int tmpIndex = 0;
    for (int i = 1; i < BUFFER_CAPACITY; i++) {
      if (telemetryArray[i].relativeTime < tmpTime) {
        tmpTime = telemetryArray[i].relativeTime;
        tmpIndex = i;
      }
    }
    mtx_struct.lock();
    memcpy(&telemetryArray[tmpIndex], &currentTelemetry, sizeof(telemetryMessage));
    mtx_struct.unlock();
    mtx_stack.unlock();
    return;
  }
 }

void sendMessages() {
  // Try to send all valid messages (older then ) from the buffer
  #ifdef DEBUG
  Serial.print("2) Sending: ");
  #endif
  
  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    mtx_stack.lock();
    if (telemetryArray[i].valid == 1) {
      sendMessage(&telemetryArray[i]);
      #ifdef DEBUG
      Serial.print(telemetryArray[i].id);
      Serial.print(", ");
      #endif
    }
    mtx_stack.unlock();
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
  Serial.print("1) buffered messages:");
  for (int i = 0; i<BUFFER_CAPACITY; i++) {
    mtx_stack.lock();
    if (telemetryArray[i].valid == 1) {
      Serial.print(telemetryArray[i].id);
      Serial.print(", ");
    }
    mtx_stack.unlock();
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
