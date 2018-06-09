#include "compass.h"

#include "Mega.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag( 12345 );

float autoXMax = -1000.0;
float autoXMin =  1000.0;
float autoYMax = -1000.0;
float autoYMin =  1000.0;
//unsigned long compassCount = 0;

float compass; // orientation of the platform


void initCompass()
{
  mag.enableAutoRange(true);

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    DEBUG_PORT.println( F("Ooops, no LSM303 detected ... Check your wiring!") );
    while(1);
  }

} // initCompass


void readCompass()
{
  if (not useCompass)
    return;

  //compassCount++;

  //DEBUG_PORT.println( F("####################### 1") );
  sensors_event_t magEvent;
  //DEBUG_PORT.println( F("####################### 1.5") );
  delay(100);
  mag.getEvent(&magEvent);  // This is where the problem is!!!!!!!!!!!!!!!!
  //DEBUG_PORT.println( F("####################### 2") );

  // Observed readings (integers)
  int xMin =  -19.45;
  int xMax =  23.27;
  int yMin = -49.82;
  int yMax =  -9.82;

  //DEBUG_PORT.println( F("####################### 3") );
  float xxBlob = magEvent.magnetic.x;
  float yyBlob = magEvent.magnetic.y;
  //DEBUG_PORT.println( F("####################### 4") );
  if((xxBlob!=0)&&(yyBlob!=0))
    {
    if(xxBlob>autoXMax)
      {
        autoXMax = xxBlob;
      }
    if(xxBlob<autoXMin)
      {
        autoXMin = xxBlob;
      }
    if(yyBlob>autoYMax)
      {
        autoYMax = yyBlob;
      }
    if(yyBlob<autoYMin)
      {
        autoYMin = yyBlob;
      }
    }

// Now normalise to min -50 and max 50:
  //DEBUG_PORT.println( F("####################### 5") );
  float xxx = ((magEvent.magnetic.x - xMin)*100/(xMax-xMin))-50;
  float yyy = ((magEvent.magnetic.y - yMin)*100/(yMax-yMin))-50;
  //DEBUG_PORT.println( F("####################### 6") );
  compass = atan2( yyy, xxx ) * RAD_TO_DEG;
  compass = compass + 270 +15; //Lower this value to make clockwise turn.

  DEBUG_PORT.print( F("autoXMin:  ") ); DEBUG_PORT.println(autoXMin);
  DEBUG_PORT.print( F("autoXMax:  ") ); DEBUG_PORT.println(autoXMax);
  DEBUG_PORT.print( F("autoYMin:  ") ); DEBUG_PORT.println(autoYMin);
  DEBUG_PORT.print( F("autoYMax:  ") ); DEBUG_PORT.println(autoYMax);
  DEBUG_PORT.print( F("Compass :  ") ); DEBUG_PORT.println(compass );
  DEBUG_PORT.println();

} // readCompass

