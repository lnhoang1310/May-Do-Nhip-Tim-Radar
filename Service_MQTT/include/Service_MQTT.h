#ifndef SERVICE_MQTT_H
#define SERVICE_MQTT_H

#include <stdbool.h>

void MQTT_Init(void);
void MQTT_Publish(const char *message, float heart_rate);

#endif
