#include "Mega.h"

#include "compass.h"
#include "console.h"
#include "FONA.h"
#include "GPS.h"
#include "heartbeat.h"
#include "navdata.h"
#include "pixy.h"
#include "speaker.h"
#include "util.h"
#include "waypoint.h"

////////////////////////////////////////////////////////////////////////////

enum setupState_t { BEFORE_SETUP, SETUP_IN_PROGRESS, SETUP_COMPLETED };
setupState_t setupState = BEFORE_SETUP;

void setup()
{
  pinMode(BLUE_LED,OUTPUT);
  pinMode(ORANGE_LED,OUTPUT);
  pinMode(51,OUTPUT);
  pinMode(53,OUTPUT);

  digitalWrite(53,LOW);

  DEBUG_PORT.begin( DEBUG_BAUD );
  DEBUG_PORT.println( F("Mega") );

  initGPS();

  setupState = SETUP_IN_PROGRESS;

  if (usePixy)
    initPixy();

  initNavData();

  if (useCompass)
    initCompass();

  if (useFona)
    initFONA();

  // Get the initial waypoint into track
  while (!waypoint_t::track.available())
    loadWaypoints();
  waypoint_t::next();
  
  setupState = SETUP_COMPLETED;

} // setup

////////////////////////////////////////////////////////////////////////////

void yield()
{
  if (useConsole)
    checkConsole();

  if (setupState > BEFORE_SETUP) {
    heartbeat();
    checkBeep();
    checkGPS ();

    if (setupState >= SETUP_COMPLETED) {
      checkNavData();
    }
  }

} // yield

////////////////////////////////////////////////////////////////////////////

void loop()
{
  yield        ();
  pixyModule   ();
  loadWaypoints();

} // loop
