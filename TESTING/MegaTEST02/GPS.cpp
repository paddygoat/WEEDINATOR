#include "GPS.h"

#include <Arduino.h>
#include <NMEAGPS.h>

#include "Mega.h"
#include "speaker.h"
#include "navdata.h"
#include "waypoint.h"

NMEAGPS gps;
gps_fix fix;

static gps_fix prevFix;

////////////////////////////////////////////////////////////////////////////

void initGPS()
{
  pinMode(USING_GPS_HEADING,OUTPUT);
  gpsPort.begin( GPS_BAUD );
  digitalWrite(USING_GPS_HEADING,HIGH);
}

////////////////////////////////////////////////////////////////////////////

void checkGPS()
{
  if (gps.available( gpsPort ))
  {
    prevFix = fix;
    fix     = gps.read(); // get the latest

    digitalWrite( ORANGE_LED, fix.valid.location );

    if (fix.valid.location) {
      beep( 100 );

      DEBUG_PORT.print( F("Current location: p/LAT") );
      DEBUG_PORT.print( fix.location.lat() );
      DEBUG_PORT.print( F("LONG") );
      DEBUG_PORT.println( fix.location.lon() );
      DEBUG_PORT.println();

      //readCompass();

      if (prevFix.valid.location) {
        // calculate heading from the current and previous locations
        float heading     = prevFix.location.BearingToDegrees( fix.location );
        fix.hdg.whole     = (int) heading;
        fix.hdg.frac      = (heading - (float) fix.hdg.whole) * 100.0;
        fix.valid.heading = true;
      }

      updateNavData();
      
      if (navData.distance() < WAYPOINT_DISTANCE_THRESHOLD) {
        beep( 500, 2500 ); // duration, pitch
        waypoint_t::next();
      }

    } else {
      DEBUG_PORT.write( '.' );
    }

    digitalWrite( USING_GPS_HEADING, fix.valid.heading );
  }
} // checkGPS

