#ifndef COMMS_H
#define COMMS_H


// #define DEBUG_COMMS
// #define DEBUG_MSG

#include <esp_now.h>
#include <WiFi.h>

#include "fusion.h"

// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS_RCV {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};
#define MAC_ADDRESS_SND {0x64, 0xE8, 0x33, 0x8A, 0xA7, 0xA0};

#define BUFFER_CAPACITY 100 // 0 for electrical efficiency (Beware the max, around a thousand+)

#define COMMS_PERIOD 333 // ms for one communication cycle
#define COMMS_PERIOD_US COMMS_PERIOD * 1000 // us for one communication cycle
#define MIN_SILENT_TIME 100 // minimal time to be silent for each cycle, in ms (not starting sending new messages)
#define MIN_SILENT_TIME_US MIN_SILENT_TIME * 1000 // minimal time to be silent for each cycle, in us 

#define CONNECTION_TIMEOUT 10000 // TImeout to stop sending messages ("stack" dumps)


void saveMessage();
esp_err_t sendMessage(telemetryMessage * message);
void sendMessages();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void commsTask(void * args);
void setupComms();

#endif