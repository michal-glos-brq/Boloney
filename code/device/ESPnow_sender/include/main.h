#include <esp_now.h>
/*##################*/
/* Macros and stuff */
/*##################*/

// C8:F0:9E:9B:32:04 (not sure about this comment)
#define MAC_ADDRESS {0xC8, 0xF0, 0x9E, 0x9B, 0x32, 0x04};

#define BUFFER_CAPACITY 1000
#define SLEEP_TIME 2500

/*############*/
/* Datatypes */
/*############*/

// Structure example to send data
// Must match the receiver structure
typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    int valid;
    uint64_t relativeTime;
    int gyroscopeX;
    int gyroscopeY;
    int gyroscopeZ;
    int accelerometerX;
    int accelerometerY;
    int accelerometerZ;
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;

// Define the array
// For simplicity, for now, let's define it as an array full of NULLs
// So we could check easily the whole array for some data present
// If you do not understand, please write me, i promise - it makes sense :D
struct telemetryMessage * telemetryArray;
struct telemetryMessage currentTelemetry;
esp_now_peer_info_t peerInfo;

// This is for loop to compare (LRU cache)
uint64_t tmpTime;
bool connected;

uint64_t counter;
uint64_t failed;
uint64_t failed_save;

/*#################*/
/* Function header */
/*#################*/

void setup();
void loop();
struct_message * readTelemetry();
int saveMessage();
int sendMessage();
