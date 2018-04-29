#include <NeoPrintToRAM.h>

#define DEBUG_PORT Serial

char url[120];

      int    zbearing          = 359.8765;
      long   zdistance         = 1234.5678;
      double compass           = 180.12345;
const char   resultA[] PROGMEM = "BEAR359DIST1234COMP180.12";

      long   zz                = -1234567890;
      long   yy                = -2134567890;
const char   resultB[] PROGMEM = "LOON-2134567890LAAT-1234567890";

void setup()
{
  DEBUG_PORT.begin( 9600 );

  characterCompileA();
  if (strcmp_P( url, resultA ) != 0)
    DEBUG_PORT.println( F("ERROR") );

  characterCompileB();
  if (strcmp_P( url, resultB ) != 0)
    DEBUG_PORT.println( F("ERROR") );
}

void loop() {}

void characterCompileA()
{
  Neo::PrintToRAM msg( url, sizeof(url) );

  // Prevent requestEvent from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("BEAR") );
    msg.print( zbearing );
    msg.print( F("DIST") );
    msg.print( zdistance );
    msg.print( F("COMP") );
    msg.print( compass );
    msg.terminate();
  interrupts();

  DEBUG_PORT.print( F("msg A to send:  ") );
  DEBUG_PORT.println( url );

} // characterCompileA


void characterCompileB()
{
  Neo::PrintToRAM msg( url, sizeof(url) );

  // Prevent requestEvent from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("LOON") );
    msg.print( yy );
    msg.print( F("LAAT") );
    msg.print( zz );
    msg.terminate();
  interrupts();

  DEBUG_PORT.print( F("msg B to send:  ") );
  DEBUG_PORT.println( url );

} // characterCompileB
