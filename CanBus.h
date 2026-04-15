#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "MSCanBus.h"

void setupCanBus();
void parseCAN();
void sendCANMessage(uint16_t can_id, uint8_t *data, uint8_t length, const char *debugMsg);
String canSerialDebugText();

#endif
