#include "heartbeat.h"

#include "Mega.h"

#include <Arduino.h>

static const uint32_t HEARTBEAT_PERIOD =  200; // ms

uint32_t lastHeartbeat;

void heartbeat()
{
  uint32_t currentMillis = millis();

  if ((currentMillis - lastHeartbeat) >= HEARTBEAT_PERIOD)
  {
    lastHeartbeat = currentMillis;
    digitalWrite( BLUE_LED, !digitalRead(BLUE_LED) ); // toggle
  }
} // heartbeat

