#include <esp_now.h>

// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS_RCV {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};
#define MAC_ADDRESS_SND {0x24, 0x6F, 0x28, 0x25, 0xE4, 0x90};

typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    uint64_t relativeTime;
    uint64_t id;
    int valid;
    int gyroscope[3];
    int accelerometer[3];
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;

typedef struct telemetryAck {
    uint64_t id;
} struct_ack;


esp_now_peer_info_t peerInfo;

struct telemetryMessage currentTelemetry;
telemetryMessage * telemetryArray;
