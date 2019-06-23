#include "FONA.h"

#include "Mega.h"
#include "console.h"
#include "navdata.h"
#include "speaker.h"
#include "util.h"
#include "waypoint.h"

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
static const uint32_t MIN_PHP_CHECK_PERIOD = 3000;
static       uint16_t lastWaypointLoaded   = 0;

void loadWaypoints()
{
  // Don't try to get waypoints too quickly
  if ((millis() - lastPHPcheck >= MIN_PHP_CHECK_PERIOD) &&
      (lastWaypointLoaded      <  LAST_WAYPOINT_ID    ) &&
      waypoint_t::track.room()) {

    // Load another waypoint into the track
    DEBUG_PORT.print( F("Loading Waypoint ID ") );
    DEBUG_PORT.println( lastWaypointLoaded+1 );

    if (getWaypoint( lastWaypointLoaded+1 ))
      lastWaypointLoaded++;
    lastPHPcheck = millis();
  }

} // loadWaypoints

///////////////////////////////////////////////////////////////////////

bool getWaypoint( uint16_t id )
{
  bool idOK = false;

  DEBUG_PORT.print( F("Select php page from TC275:        ") );
  DEBUG_PORT.println( id );

  // Builds the url character array:
  char url[60];
  Neo::PrintToRAM urlChars( url, sizeof(url) );
  urlChars.print( CF(webAddress) );
  urlChars.print( id );
  urlChars.print( CF(dotPhp) );
  urlChars.terminate(); // add NUL terminator to this C string

  //urlChars.print( F("bollox") );
  //urlChars.print( id );
  //urlChars.print( F(".php") );
  //urlChars.terminate();

  DEBUG_PORT.print( F("url for GET request:       ") );
  showData( url, strlen(url) );
  DEBUG_PORT.println();

  if (not useFona) {
    if (useConsole) {
      getConsolePHP();
      return true;
    }
    return false;
  }

  digitalWrite(BLUE_LED, HIGH);

  while (true) {
    // Issue GET request.  Reply will be a waypoint.
    uint16_t statuscode;
    uint16_t length;

    bool getStarted = fona.HTTP_GET_start( url, &statuscode, &length );
    if (getStarted) {

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

      waypoint_t nextWaypoint;
      idOK = parse( nextWaypoint, receive, receiveChars.numWritten() );
      if (idOK) {
        nextWaypoint.id = id;
        waypoint_t::track.write( nextWaypoint );
      }

    } else {
       DEBUG_PORT.println( F("HTTP GET Failed!") );
    }
   
    DEBUG_PORT.println( F("\n****") );
    fona.HTTP_GET_end();

    if (getStarted)
      break;

    delay( 2000 );
  }

  digitalWrite(BLUE_LED, LOW);

  return idOK;

} // getWaypoint

////////////////////////////////////////////////////////////////////////////

bool parse( waypoint_t &waypoint, char *ptr, size_t remaining )
{
  static const char     LAT_LABEL[] PROGMEM = "LAT";
  static const size_t   LAT_LABEL_LEN       = sizeof(LAT_LABEL)-1;
  static const char     LON_LABEL[] PROGMEM = "LONG";
  static const size_t   LON_LABEL_LEN       = sizeof(LON_LABEL)-1;
               uint32_t latValue, lonValue;

  bool ok = false;

  if (remaining > 0) {
    // Skip the first character (what is it?)
    ptr++;
    remaining--;

    if (findText( LAT_LABEL, LAT_LABEL_LEN, ptr, remaining )) {

      if (parseValue( ptr, remaining, latValue )) {

        if (findText( LON_LABEL, LON_LABEL_LEN, ptr, remaining )) {

          if (parseValue( ptr, remaining, lonValue )) {

            // Set the new waypoint location
            waypoint.location.lat( latValue );
            waypoint.location.lon( lonValue );

            DEBUG_PORT.print  ( F("Location from Fona:  ") );
            DEBUG_PORT.print  ( waypoint.location.lat() );
            DEBUG_PORT.print  ( ',' );
            DEBUG_PORT.println( waypoint.location.lon() );

            //  Make sure we used all the characters
            if (remaining > 0) {
              DEBUG_PORT.print( remaining );
              DEBUG_PORT.print( F(" extra characters after lonValue: '") );
              showData( ptr, remaining );
              DEBUG_PORT.println('\'');
            }
            
            ok = true;

          } else {
            DEBUG_PORT.println( F("Invalid longitude") );
          }
        }

      } else {
        DEBUG_PORT.println( F("Invalid latitude") );
      }
    }
  } else {
    DEBUG_PORT.println( F("response too short") );
  }

  return ok;

} // parse

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
