#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

typedef struct telemetryMessage {
    // Relative time will be kept, absolute time will be noted roughly
    uint64_t relativeTime;
    uint64_t id;
    int valid;
    double orientation[3];
    double position[3];
    int barometer;
    float thermometer;
    float thermometer_stupido;
    float voltage;
} struct_message;


#endif