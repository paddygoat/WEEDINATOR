#include "navdata.h"

#include <Arduino.h>
#include <Wire.h>

#include "TC275.h"
#include "util.h"

navData_t navData;

const size_t navData_t::MSG_SIZE = sizeof( navData_t );
 
////////////////////////////////////////////////////////////////////////////

void initNavData()
{
  if (useI2C)
    Wire.begin();// join i2c bus (address optional for master)

  DEBUG_PORT.print( F("navData message size: ") );
  DEBUG_PORT.println( navData_t::MSG_SIZE );

} // initNavData

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
