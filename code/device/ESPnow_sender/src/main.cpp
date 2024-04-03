
#include <main.h>

// Debug compilation (comment to turn debug off)
#define DEBUG


//////////////////////////////////////////
//// Setting up some global variables ////
//////////////////////////////////////////

uint8_t broadcastAddress[] = MAC_ADDRESS_RCV;
uint64_t id_counter = 0;

bool mpuReady = false;
bool bmpReady = false;

uint64_t period_time_micros = PERIOD_TIME * 1000;
uint64_t min_wait_time_micros = MIN_WAIT_TIME * 1000;

struct telemetryMessage dynamicTelemetry;

///////////////////////////////////////////////
//// Communication functions and callbacks ////
///////////////////////////////////////////////


void saveMessage() {
  // Saves current message to the buffer
  if (currentTelemetry.valid == 0) {
    return;
  }
  else {
    #ifdef DEBUG
    Serial.println("Warning! Message not confirmed - saving into buffer");
    #endif

    //xSemaphoreTake(mtx_stack, portMAX_DELAY);

    for (int i = 0; i < BUFFER_CAPACITY; i++) {
      
      if (telemetryArray[i].valid == 0) {
        // Copy currentTelemetry to the telemetry array at I place with memcpy
        memcpy(&telemetryArray[i], &currentTelemetry, sizeof(telemetryMessage));
        //xSemaphoreGive(mtx_stack);
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
    //xSemaphoreGive(mtx_stack);
    return;
  }
 }

// This sends a sinlge message
esp_err_t sendMessage(telemetryMessage * message){
  return esp_now_send(broadcastAddress, (uint8_t *) message, sizeof(telemetryMessage));
}

void sendMessages() {
  // Try to send all valid messages (older then ) from the buffer
  #ifdef DEBUG
  Serial.print("2) Sending: ");
  #endif
  
  //xSemaphoreTake(mtx_stack, portMAX_DELAY);
  for (int i = 0; i < BUFFER_CAPACITY; i++) {
    if (telemetryArray[i].valid == 1) {
      sendMessage(&telemetryArray[i]);
      #ifdef DEBUG
      Serial.print(telemetryArray[i].id);
      Serial.print(", ");
      #endif
    }
    // Check whether half time from sleep time already elapsed, if so, break and return
    if ((micros() - current_loop_start) > min_wait_time_micros) {
      break;
    }
  }
  //xSemaphoreGive(mtx_stack);

  #ifdef DEBUG
  Serial.println("FInished sendMessages ...");
  #endif
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// This function is OnReceive callback and invalidates the message from buffer 
// with received ID (as long int) as a form of ack
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  last_message_received = micros();
  telemetryMessage * message = (telemetryMessage *) incomingData;

  //xSemaphoreTake(mtx_stack, portMAX_DELAY);
  if (currentTelemetry.id == message->id) {
    #ifdef DEBUG
    Serial.print("Ack for last message: ");
    Serial.println(currentTelemetry.id);
    #endif
    currentTelemetry.valid = 0;
    //xSemaphoreGive(mtx_stack);
    return;
  }
  else {
    #ifdef DEBUG
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
  //xSemaphoreGive(mtx_stack);
}


//////////////////////////////////////////////
//// Sensor and Telemetry msg composition ////
//////////////////////////////////////////////

// This function creates telemetryMessage struct
void readTelemetry(){
  // Some actual logic will have to be implemented here ...
  // message->relativeTime = micros();
  
  //xSemaphoreTake(mtx_stack, portMAX_DELAY);
  //xSemaphoreTake(mtx_sensor_acc, portMAX_DELAY);
  Serial.println("Reading telemetry 1 ...");
  memcpy(&currentTelemetry, &dynamicTelemetry, sizeof(telemetryMessage));
  Serial.println("Reading telemetry 2 ...");
  //xSemaphoreGive(mtx_sensor_acc);
  currentTelemetry.id = id_counter++;
  
  bmp.takeForcedMeasurement();
  Serial.println("Reading telemetry 3 ...");
  currentTelemetry.thermometer = bmp.readTemperature();
  Serial.println("Reading telemetry 4 ...");
  currentTelemetry.barometer = bmp.readPressure();
  currentTelemetry.voltage = 3.7;
  //xSemaphoreGive(mtx_stack);
}

////////////////////////
//// Periodic Tasks ////
////////////////////////

void communication_process(void * not_gonna_access_this) {
  while (1) {
    current_loop_start = micros();

    #ifdef DEBUG
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
      if (micros() - last_message_received < (CONNECTION_TIMEOUT * 1000)) {
        sendMessages();
      }
      #ifdef DEBUG
      else {
        Serial.println("Connection to GS timed out ...");
      }
      #endif
    }

    // Try to send current telemetry
    Serial.println("Starting telemetry reading ...");
    readTelemetry();
    Serial.println("Telemetry reading done ...");
    esp_now_peer_info();
  
    esp_err_t succ = sendMessage(&currentTelemetry);

    #ifdef DEBUG
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
    msg_timing_diff = (period_time_micros - (micros() - current_loop_start)) / 1000;
    if (msg_timing_diff > 0) {
      vTaskDelay(msg_timing_diff);
    }
    else {
      #ifdef DEBUG
      Serial.println("Warning! Loop took too long ...");
      #endif
    }
  }
}

void sensor_process(void * not_gonna_access_this) {

  while (1) {
    
    current_time_telemetry = micros();
    //xSemaphoreTake(mtx_sensor_acc, portMAX_DELAY);
    if(mpuReady){
      mpu.read();
      data = mpu.getRawData();
      // Now you can access the raw data like this:
      // data.AcX, data.AcY, data.AcZ, etc.
      // Update orientation and position with basic algorithm with gravitation vector detection
      dynamicTelemetry.thermometer_stupido = data.Tmp/340.00+36.53;
      dynamicTelemetry.accelerometer[0] = data.AcX;
      dynamicTelemetry.accelerometer[1] = data.AcY;
      dynamicTelemetry.accelerometer[2] = data.AcZ;
      dynamicTelemetry.gyroscope[0] = data.GyX;
      dynamicTelemetry.gyroscope[1] = data.GyY;
      dynamicTelemetry.gyroscope[2] = data.GyZ;
    }
    
    dynamicTelemetry.relativeTime = current_time_telemetry;
    
    //xSemaphoreGive(mtx_sensor_acc);

    #ifdef DEBUG
    mpu.printData();
    // Check the remaining stack space
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("Sensor Task Stack High Water Mark: ");
    Serial.println(uxHighWaterMark);
    #endif
    vTaskDelay(PERIOD_SENSOR_READING);  
  }
}

//////////////////////////
//// Setting stuff up ////
//////////////////////////

void loop() {
  // Intentionally empty
}

void setupReading() {
  Wire.begin(8, 10); // SDA, SCL
  if(mpu.begin()){
    #ifdef DEBUG
    Serial.println("MPU6050 is ready!");
    #endif
    mpuReady = true;
  } else {
    #ifdef DEBUG
    Serial.println("MPU6050 is not ready!");
    #endif
  }

     //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (bmp.begin()) {
    #ifdef DEBUG
    Serial.println("BMP280 is ready!");
    #endif
    bmpReady = true;
  } else {
    #ifdef DEBUG
    Serial.println("BMP280 is not ready!");
    #endif
  }

  if(bmpReady){
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
}

// Setup TROS periodic tasks
void setupRTOS() {
  xTaskCreatePinnedToCore(
    sensor_process,          // Function to implement the task
    "Sensor",                // Name of the task
    10000,                    // Stack size in words
    NULL,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    0);                      // Core where the task should run

  xTaskCreatePinnedToCore(
    communication_process,   // Function to implement the task
    "Communication",         // Name of the task
    10000,                    // Stack size in words
    NULL,                    // Task input parameter
    1,                       // Priority of the task
    NULL,                    // Task handle.
    1);                      // Core where the task should run
}

void setup() {

  //mtx_stack = xSemaphoreCreateMutex();
  //mtx_sensor_acc = xSemaphoreCreateMutex();

  //if (mtx_stack == NULL || mtx_sensor_acc == NULL) {
  //  // Handle case where mutex creation failed
  //  Serial.println("Failed to create mutexes");
  //  return;
  //}

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
  dynamicTelemetry.valid = 1;
  last_message_received = micros();

  setupReading();

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
  setupRTOS();
}
