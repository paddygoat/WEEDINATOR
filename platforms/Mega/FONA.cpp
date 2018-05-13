#include "FONA.h"

#include "Mega.h"
#include "console.h"
#include "navdata.h"
#include "speaker.h"
#include "util.h"

#include <NeoPrintToRAM.h>

#include <Adafruit_FONA.h>
Adafruit_FONA fona( FONA_RST );


// Change these settings! (Network APN, username ,password)
static const char networkAPN[] PROGMEM = "pp.vodafone.co.uk";
static const char username  [] PROGMEM = "wap";
static const char password  [] PROGMEM = "wap";
static const char webAddress[] PROGMEM =
  "http://www.goatindustries.co.uk/weedinator/select";
static const char dotPhp    [] PROGMEM = ".php";

// Handy macro for passing PROGMEM char arrays to anything 
//   that expects a FLASH string, like DEBUG_PORT.print or 
//   fona.setGPRSNetworkSettings
#define CF(x) ((const __FlashStringHelper *)x)

static void turnOnGPRS();

////////////////////////////////////////////////////////////////////////////

void initFONA()
{
  fonaPort.begin( FONA_BAUD );
  if (! fona.begin( fonaPort )) {
    DEBUG_PORT.println( F("Couldn't find FONA") );
    //while (1);
  }
  DEBUG_PORT.println( F("FONA is OK") );

  uint8_t type = fona.type();
  const __FlashStringHelper *typeString;
  switch (type) {
    case FONA800L:
      typeString = F("FONA 800L");          break;
    case FONA800H:
      typeString = F("FONA 800H");          break;
    case FONA808_V1:
      typeString = F("FONA 808 (v1)");      break;
    case FONA808_V2:
      typeString = F("FONA 808 (v2)");      break;
    case FONA3G_A:
      typeString = F("FONA 3G (American)"); break;
    case FONA3G_E:
      typeString = F("FONA 3G (European)"); break;
    default: 
      typeString = F("???");                break;
  }
  DEBUG_PORT.print  ( F("Found ") );
  DEBUG_PORT.println( typeString );

  //networkStatus();   // Check the network is available. Home is good.
  DEBUG_PORT.println();

  DEBUG_PORT.println( F("Checking that GPRS is turned off to start with .........") );

  fona.setGPRSNetworkSettings( CF(networkAPN), CF(username), CF(password) );
  //delay (1000);

  //delay (1000);
  //networkStatus();   // Check the network is available. Home is good.

  turnOnGPRS();

} // initFONA

////////////////////////////////////////////////////////////////////////////

static       uint32_t lastPHPcheck         = 0;
static const uint32_t MIN_PHP_CHECK_PERIOD = 1000;

void checkWaypoint()
{
  // Don't try to get waypoints too quickly
  if (millis() - lastPHPcheck >= MIN_PHP_CHECK_PERIOD) {

    if ((waypointID < LAST_WAYPOINT_ID) &&
        (distanceToWaypoint < WAYPOINT_DISTANCE_THRESHOLD)) {

      beep( 500, 1000 ); // duration, pitch

      // Get the next waypoint
      waypointID++;
      DEBUG_PORT.print( F("New Waypoint ID ") );
      DEBUG_PORT.println( waypointID );

      // Yes, get it now.
      getWaypoint();
      lastPHPcheck = millis();
    }
  }

} // checkWaypoint

///////////////////////////////////////////////////////////////////////

void getWaypoint()
{
  DEBUG_PORT.print( F("Select php page from TC275:        ") );
  DEBUG_PORT.println( waypointID );

  // Builds the url character array:
  char url[60];
  Neo::PrintToRAM urlChars( url, sizeof(url) );
  urlChars.print( CF(webAddress) );
  urlChars.print( waypointID );
  urlChars.print( CF(dotPhp) );
  urlChars.terminate(); // add NUL terminator to this C string

  //urlChars.print( F("bollox") );
  //urlChars.print( waypointID );
  //urlChars.print( F(".php") );
  //urlChars.terminate();

  DEBUG_PORT.print( F("url for GET request:       ") );
  showData( url, strlen(url) );
  DEBUG_PORT.println();

  if (not useFona) {
    if (useConsole) {
      getConsolePHP();
    }
    return;
  }

  digitalWrite(BLUE_LED, HIGH);

  while (true) {
    // Issue GET request.  Reply will be a waypoint.
    bool     ok = false;
    uint16_t statuscode;
    uint16_t length;

    if (fona.HTTP_GET_start( url, &statuscode, &length )) {

      char receive[40];
      Neo::PrintToRAM receiveChars( receive, sizeof(receive) );

      // This is blocking, because the complete reply has not arrived yet.
      DEBUG_PORT.print( F("Raw PHP reply: '") );
      while (length > 0) {
        if (fona.available()) {
          char c = fona.read();
          if (isprint( c ))
            receiveChars.write( c ); // add to array
          showData( &c, 1 );

          length--;
        }
        yield();
      }
      receiveChars.terminate();
      DEBUG_PORT.println('\'');

      DEBUG_PORT.print( F("Lat and Lon from database (receive):     ") );
      showData( receive, receiveChars.numWritten() );
      DEBUG_PORT.println();

      ok = parseWaypoint( receive, receiveChars.numWritten() );

    } else {
       DEBUG_PORT.println( F("HTTP GET Failed!") );
    }
   
    DEBUG_PORT.println( F("\n****") );
    fona.HTTP_GET_end();

    if (ok)
      break;

    delay( 2000 );
  }

  digitalWrite(BLUE_LED, LOW);

  return;

} // getWaypoint

///////////////////////////////////////////////////////////////////////

static void turnOnGPRS()
{
  while (true) {
    DEBUG_PORT.println( F("Turning off GPRS ...") );
    if (fona.enableGPRS(false)) {
      DEBUG_PORT.println( F("GPRS turned off.") );
    } else {
      DEBUG_PORT.println( F("FAILED: GPRS not turned off") );
    }

    DEBUG_PORT.println( F("Turning on GPRS ...") );
    if (fona.enableGPRS(true)) {
      DEBUG_PORT.println( F("GPRS turned on.") );
      break;
    } else {
      DEBUG_PORT.println( F("FAILED: GPRS not turned on, retrying"));
    }

    delay( 1000 );
  }

} // turnOnGPRS
