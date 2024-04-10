#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
// Include mutex from cpp
#include <mutex>


// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS_RCV {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};
#define MAC_ADDRESS_SND {0x64, 0xE8, 0x33, 0x8A, 0xA7, 0xA0};


typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    uint64_t relativeTime;
    uint64_t id;
    int valid;
    float orientation[3];
    float angularVelocity[3];
    float position[3];
    float velocity[3];
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;




esp_now_peer_info_t peerInfo;

struct telemetryMessage currentTelemetry;
telemetryMessage * telemetryArray;

esp_err_t sendMessage(telemetryMessage *message);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void print_entry(telemetryMessage *msg);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void SendHeader();
void setupRTOS();
void setupComms();
void setup();
void commsTask(void * args);
void loop();

#endif