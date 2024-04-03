#ifndef MAIN_H
#define MAIN_H

#include <esp_now.h>

// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS_RCV {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};
#define MAC_ADDRESS_SND {0x64, 0xE8, 0x33, 0x8A, 0xA7, 0xA0};

typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    uint64_t relativeTime;
    uint64_t id;
    int valid;
    double gyroscope[3];
    double accelerometer[3];
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;


esp_now_peer_info_t peerInfo;

struct telemetryMessage currentTelemetry;
telemetryMessage * telemetryArray;


#endif