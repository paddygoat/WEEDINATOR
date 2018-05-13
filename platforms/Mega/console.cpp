#include "console.h"

#include "Mega.h"
#include "navdata.h"

#include <Arduino.h>

static bool lineReady();

////////////////////////////////////////////////////////////////////////////
//  For testing, accept commands from the "console" to simulate:
//     *  picking a new waypoint ID
//     *  receiving a PHP response with waypoint lat/lon
//     *  displaying current message A/B

bool           gotConsolePHP = false;

void getConsolePHP()
{
  DEBUG_PORT.print( F("Enter PHP response:") );
  gotConsolePHP = false;
  while (!gotConsolePHP)
    yield();
}

//  Some variables to receive a line of characters
size_t         count     = 0;
const size_t   MAX_CHARS = 64;
char           line[ MAX_CHARS ];

void checkConsole()
{
  if (lineReady()) {
    size_t lineLen = strlen( line );

    if (lineLen > 0) {

      if (line[0] == 'w') {
        // simulate going to a new waypoint id
        distanceToWaypoint = 0;

      } else if ((line[0] == 'p') and (lineLen > 1)) {
        // simulate receiving a response to the GET request
        parseWaypoint( &line[1], lineLen-1 );
        gotConsolePHP = true;

      } else if (line[0] == 's') {
        // simulate sending messages to the TC275
        sendNavData();
      
      } else {
        DEBUG_PORT.println( F("Invalid command" ) );
      }
    }
  }
} // checkConsole

static bool lineReady()
{
  bool          ready     = false;
  const char    endMarker = '\n';

  while (DEBUG_PORT.available()) {

    char c = DEBUG_PORT.read();

    if (c != endMarker) {

      // Only save the printable characters, if there's room
      if (isprint(c) and (count < MAX_CHARS-1)) {
        line[ count++ ] = c;
      }

    } else {
      //  It's the end marker, line is completely received
      line[count] = '\0'; // terminate the string
      count       = 0;    // reset for next time
      ready       = true;
      break;
    }
  }

  return ready;

} // lineReady

