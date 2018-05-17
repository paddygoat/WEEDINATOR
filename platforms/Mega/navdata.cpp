#include "navdata.h"

#include <Arduino.h>
#include <NeoPrintToRAM.h>
#include <Wire.h>

#include "Mega.h"
#include "GPS.h"
#include "units.h"
#include "util.h"
#include "waypoint.h"

float bearingToWaypoint;  // degrees
float distanceToWaypoint; // mm

static bool newMsgA = false;
static bool newMsgB = false;

static void nextNavState();
static void compileBearDistHeadMsg();
static void compileLatLonMsg();

////////////////////////////////////////////////////////////////////////////

void initNavData()
{
  if (useI2C) {
    pinMode(I2C_REQUEST,OUTPUT);
    pinMode(I2C_RECEIVE,OUTPUT);

    Wire.begin( MEGA_I2C_ADDR );
    Wire.onRequest(sendNavData); // register event
  }
} // initNavData

////////////////////////////////////////////////////////////////////////////

         static char navData[36];
volatile static bool navDataSent = false;

void sendNavData()
{
  digitalWrite(I2C_REQUEST,HIGH);

  if (useI2C) {
    Wire.write(navData);     // as expected by master
  } else {
    DEBUG_PORT.print( F("TC275 --> ") );
    showData( navData, strlen(navData) );
  }

  navDataSent = true;

  digitalWrite(I2C_REQUEST,LOW);

} // sendNavData

////////////////////////////////////////////////////////////////////////////

void checkNavData()
{
  if (navDataSent) {
    // compile the next message
    nextNavState();
    navDataSent = false;
  }
}

enum     navDataState_t { BEARING_DIST_HEAD, LAT_LON, LAST_NAV_STATE };
const    navDataState_t FIRST_NAV_STATE = (navDataState_t) 0;
volatile navDataState_t navDataState;

static void nextNavState()
{
  navDataState = (navDataState_t) (navDataState + 1);
  if (navDataState == LAST_NAV_STATE)
    navDataState = FIRST_NAV_STATE;

  switch (navDataState) {
     case BEARING_DIST_HEAD: compileBearDistHeadMsg(); break;
     case LAT_LON          : compileLatLonMsg      (); break;
  }

} // nextNavState

////////////////////////////////////////////////////////////////////////////

void updateNavData()
{
  newMsgA = newMsgB = true;

  float range = fix.location.DistanceKm( waypoint_t::current.location );
  DEBUG_PORT.print( F("Distance km:     ") );
  DEBUG_PORT.println( range );

  distanceToWaypoint = range * MM_PER_KM;
  DEBUG_PORT.print( F("Distance mm:     ") );
  DEBUG_PORT.println( distanceToWaypoint );

  bearingToWaypoint = fix.location.BearingToDegrees( waypoint_t::current.location );
  DEBUG_PORT.print( F("Bearing:         ") );
  DEBUG_PORT.println( bearingToWaypoint );

  DEBUG_PORT.print( F("Heading:         ") );
  if (fix.valid.heading)
      DEBUG_PORT.println( fix.heading() );

  DEBUG_PORT.println();

} // updateNavData

////////////////////////////////////////////////////////////////////////////

static void compileBearDistHeadMsg()
{
  Neo::PrintToRAM msg( navData, sizeof(navData) );

  // Prevent sendNavData from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("BEAR") );
    msg.print( (int) (bearingToWaypoint * 100.0) );
    msg.print( F("DIST") );
    msg.print( (long) distanceToWaypoint );
    msg.print( F("HEAD") );
    if (fix.valid.heading)
      msg.print( fix.heading() );
    msg.terminate();
  interrupts();

  if (newMsgA) {
    newMsgA = false;
    DEBUG_PORT.print( F("msg A to send:  ") );
    DEBUG_PORT.println( navData );
  } else {
    DEBUG_PORT.print( 'A' );
  }

} // compileBearDistHeadMsg

////////////////////////////////////////////////////////////////////////////

static void compileLatLonMsg()
{
  Neo::PrintToRAM msg( navData, sizeof(navData) );

  // Prevent sendNavData from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("LOON") );
    msg.print( fix.location.lon() );
    msg.print( F("LAAT") );
    msg.print( fix.location.lat() );
    msg.terminate();
  interrupts();

  if (newMsgB) {
    newMsgB = false;
    DEBUG_PORT.print( F("msg B to send:  ") );
    DEBUG_PORT.println( navData );
  } else {
    DEBUG_PORT.print( 'B' );
  }

} // compileLatLonMsg
