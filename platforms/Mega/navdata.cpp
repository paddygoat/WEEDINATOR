#include "navdata.h"

#include <Arduino.h>
#include <NeoPrintToRAM.h>
#include <Wire.h>

#include "Mega.h"
#include "GPS.h"
#include "units.h"
#include "util.h"
#include "waypoint.h"

navData_t navData;

const size_t navData_t::MSG_SIZE = sizeof( navData_t );
 
////////////////////////////////////////////////////////////////////////////

void initNavData()
{
  if (useI2C) {
    pinMode(I2C_REQUEST,OUTPUT);
    pinMode(I2C_RECEIVE,OUTPUT);

    Wire.begin( MEGA_I2C_ADDR );
    Wire.onRequest(sendNavData); // register event
  }

  DEBUG_PORT.print( F("navData message size: ") );
  DEBUG_PORT.println( navData_t::MSG_SIZE );

} // initNavData

////////////////////////////////////////////////////////////////////////////

static uint8_t message[ navData_t::MSG_SIZE ];

void sendNavData()
{
  digitalWrite(I2C_REQUEST,HIGH);

  if (useI2C) {
    Wire.write( message, sizeof(navData) );
  } else {
    DEBUG_PORT.print( F("TC275 --> ") );
    showData( &message[0], sizeof(message) );
    DEBUG_PORT.println();
  }

  digitalWrite(I2C_REQUEST,LOW);

} // sendNavData

////////////////////////////////////////////////////////////////////////////

void updateNavData()
{
  navData.waypointID( waypoint_t::current.id );

  if (fix.valid.location) {
    navData.location( fix.location );
  } else {
    NeoGPS::Location_t nowhere( 0L, 0L );
    navData.location( nowhere );
  }
  
  if (fix.valid.location and (navData.waypointID() > 0)) {
    float distKm = fix.location.DistanceKm( waypoint_t::current.location );
    navData.distance( distKm * M_PER_KM );
    navData.bearing ( fix.location.BearingToDegrees( waypoint_t::current.location ) );
    if (fix.valid.heading)
      navData.heading( fix.heading() );     // GPS direction of travel
    else
      navData.heading( navData.bearing() ); // unknown heading, drive straight
  } else {
    navData.distance( WAYPOINT_FAR_AWAY );
    navData.bearing ( 0.0 );
    navData.heading ( 0.0 );
  }

  navData.printTo( message, sizeof(message) );

  DEBUG_PORT.print( F("Distance meters: ") );
  DEBUG_PORT.println( navData.distance() );

  DEBUG_PORT.print( F("Bearing:         ") );
  DEBUG_PORT.println( navData.bearing() );

  DEBUG_PORT.print( F("Heading:         ") );
  DEBUG_PORT.println( navData.heading() );

  DEBUG_PORT.println();

} // updateNavData

////////////////////////////////////////////////////////////////////////////

void navData_t::printTo( uint8_t *bytes, size_t len )
{
  if (len != MSG_SIZE) {
    static bool warningPrinted = false;
    if (not warningPrinted) {
      DEBUG_PORT.print( F("navData::printTo buffer size ") );
      DEBUG_PORT.print( len );
      DEBUG_PORT.print( F(" not the expected size ") );
      DEBUG_PORT.println( MSG_SIZE );
      warningPrinted = true;
    }
  } else {
    // Prevent sendNavData from getting the first half of the old
    //   response and the second half of the new message.
    noInterrupts();
      memcpy( bytes, this, MSG_SIZE );
    interrupts();

    DEBUG_PORT.print( F("msg to send :  ") );
    showData( bytes, len );
    DEBUG_PORT.println();
  }
} // printTo

////////////////////////////////////////////////////////////////////////////

void navData_t::readFrom( uint8_t *bytes, size_t len )
{
  if (len != MSG_SIZE) {
    static bool warningPrinted = false;
    if (not warningPrinted) {
      DEBUG_PORT.print( F("navData::readFrom buffer size ") );
      DEBUG_PORT.print( len );
      DEBUG_PORT.print( F(" not the expected size ") );
      DEBUG_PORT.println( MSG_SIZE );
      warningPrinted = true;
    }
  } else {
    // Prevent receiveNavData from getting the first half of the old
    //   response and the second half of the new message.
    noInterrupts();
      memcpy( this, bytes, MSG_SIZE );
    interrupts();

    DEBUG_PORT.print( F("msg received:  ") );
    showData( bytes, len );
    DEBUG_PORT.println();

    DEBUG_PORT.print( F("id      : ") );
    DEBUG_PORT.println( waypointID() );
    DEBUG_PORT.print( F("lat     : ") );
    DEBUG_PORT.println( location().lat() );
    DEBUG_PORT.print( F("lon     : ") );
    DEBUG_PORT.println( location().lon() );
    DEBUG_PORT.print( F("bearing : ") );
    DEBUG_PORT.println( bearing() );
    DEBUG_PORT.print( F("heading : ") );
    DEBUG_PORT.println( heading() );
    DEBUG_PORT.print( F("distance: ") );
    DEBUG_PORT.println( distance() );
  }
} // readFrom
