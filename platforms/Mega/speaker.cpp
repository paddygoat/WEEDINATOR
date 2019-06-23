#include "speaker.h"

#include "Mega.h"

#include <Arduino.h>

uint32_t beepDuration;
uint32_t beepStart;
bool     beeping;

////////////////////////////////////////////////////////////////////////////

void beep( uint32_t duration, uint16_t freq )
{
  beeping      = true;
  beepStart    = millis();
  beepDuration = duration;
  tone( SPEAKER, freq, 0 );   // pin,pitch,duration (forever)
}

////////////////////////////////////////////////////////////////////////////

void checkBeep()
{
  if (beeping and ((millis() - beepStart) >= beepDuration)) {
    noTone( SPEAKER );
    beeping = false;
  }
}
