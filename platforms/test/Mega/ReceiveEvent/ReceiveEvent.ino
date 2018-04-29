//  This test program allows you to enter test messages
//    in the Serial Monitor window (press Send).
//
//  Be sure the Serial Monitor sends LF (newline) or CR+LF (CR+newline).
//
//  Some messages to try:
//
//     L
//     LAT
//     LATL
//     LAT0
//     LAT1234567890
//     LAT-1234567890
//     LAT-1234567890L
//     LAT-1234567890LON
//     LAT-1234567890LONG
//     LAT-1234567890LONGL
//     LAT-1234567890LONG-2134567098   <-- valid!
//     LAT1234567890LONG2134567098     <-- valid!
//     LAT1234567890LONG0000000000002134567098

#define DEBUG_PORT Serial

#include <Location.h>

NeoGPS::Location_t base( 532558000L, -43114000L ); // Llangefni

volatile bool fonaMsgAvailable = false;
volatile bool fonaMsgLost      = false;

#include <NeoStreamFromRAM.h>
Neo::StreamFromRAM Wire; // simulated!

//  Some variables to receive a line of characters
size_t         count     = 0;
const size_t   MAX_CHARS = 64;
char           line[ MAX_CHARS ];


void setup()
{
  DEBUG_PORT.begin( 9600 );

  fonaMsgLost = true; // simulate losing a msg
}

void loop()
{
  checkForFonaMsg();

  // Simulate receiving a Fona msg
  if (lineReady()) {
    size_t n = strlen( line );

    DEBUG_PORT.print  ( F("Parsing line '") );
    DEBUG_PORT.print  ( line );
    DEBUG_PORT.print  ( '\'' );
    DEBUG_PORT.print  ( F(", len = ") );
    DEBUG_PORT.println( n );

    Wire.write( (uint8_t *) line, n );
    receiveEvent( n );
  }
}

size_t   fonaMsgLen       = 0;
char     fonaMsg[33];

void receiveEvent(int howMany)
{
  //digitalWrite(I2C_RECEIVE,HIGH);

  if (not fonaMsgAvailable) {

    // Get the whole message
    fonaMsgLen = 0;
    while (Wire.available()) {
      char c = Wire.read();
      if (fonaMsgLen < sizeof(fonaMsg)-1)
        fonaMsg[ fonaMsgLen++ ] = c;
    }
    fonaMsg[ fonaMsgLen ] = '\0'; // NUL-terminate;

    fonaMsgAvailable = true;

  } else {
    // fonaMsgAvailable is still true, loop must not have parsed it yet.
    //   This means that this Fona message must be ignored.

    while (Wire.available())
      Wire.read();
    fonaMsgLost = true;
  }

  //digitalWrite(I2C_RECEIVE,LOW);

} // receiveEvent


void checkForFonaMsg()
{
  if (fonaMsgAvailable) {
    parseFonaMsg();

    // now that the parsing is finished, mark the message as "read"
    fonaMsgAvailable = false; 
  }

  // see if we lost a message sometime
  if (fonaMsgLost) {
    fonaMsgLost = false;
    DEBUG_PORT.println( F("FONA message lost!") );
  }

} // checkForFonaMsg

void parseFonaMsg()
{
  char   *ptr       = &fonaMsg[0]; // point to the beginning
  size_t  remaining = fonaMsgLen;

  static const char     LAT_LABEL[] PROGMEM = "LAT";
  static const size_t   LAT_LABEL_LEN       = sizeof(LAT_LABEL)-1;
  static const char     LON_LABEL[] PROGMEM = "LONG";
  static const size_t   LON_LABEL_LEN       = sizeof(LON_LABEL)-1;
               uint32_t latValue, lonValue;

  if (findText( LAT_LABEL, LAT_LABEL_LEN, ptr, remaining )) {

    if (parseValue( ptr, remaining, latValue )) {

      if (findText( LON_LABEL, LON_LABEL_LEN, ptr, remaining )) {

        if (parseValue( ptr, remaining, lonValue )) {

          // Set the new base location
          base.lat( latValue );
          base.lon( -lonValue ); // <-- until sender fixed!!

          DEBUG_PORT.print  ( F("Location from Fona:  ") );
          DEBUG_PORT.print  ( base.lat() );
          DEBUG_PORT.print  ( ',' );
          DEBUG_PORT.println( base.lon() );

          //  Make sure we used all the characters
          if (remaining > 0) {
            DEBUG_PORT.print( remaining );
            DEBUG_PORT.print( F(" extra characters after lonValue: ") );

            while (remaining-- > 0) {
              char c = *ptr++;
              char nybble = c >> 4;
              char hexDigit = (nybble < 10) ? (nybble + '0') : (nybble - 10 + 'A');
              DEBUG_PORT.print( hexDigit );

              nybble = c & 0x0F;
              hexDigit = (nybble < 10) ? (nybble + '0') : (nybble - 10 + 'A');
              DEBUG_PORT.print( hexDigit );
              DEBUG_PORT.print( ' ' );
            }
            DEBUG_PORT.println();
          }

        } else {
          DEBUG_PORT.println( F("Invalid longitude") );
        }
      }

    } else {
      DEBUG_PORT.println( F("Invalid latitude") );
    }
  }

} // parseFonaMsg


bool findText
  ( const char *   text_P, const size_t   len,
          char * & ptr   ,       size_t & remaining )
{
  bool found = false;
  
  if (remaining >= len) {

    static const int MATCHED = 0;
  
    if (memcmp_P( ptr, text_P, len ) == MATCHED) {
      found = true;
      
      ptr       += len; // Advance pointer past the label
      remaining -= len;

    } else {
      DEBUG_PORT.print  ( (const __FlashStringHelper *) text_P );
      DEBUG_PORT.println( F(" label not found") );
    }

  } else {
    DEBUG_PORT.print  ( F("Length ") );
    DEBUG_PORT.print  ( remaining );
    DEBUG_PORT.print  ( F(" too short for label ") );
    DEBUG_PORT.println( (const __FlashStringHelper *) text_P );
  }
  
  return found;

} // findText


bool parseValue( char * & ptr, size_t & remaining, uint32_t & value )
{
  bool ok = false;

  if (remaining > 0) {

    bool negative = (*ptr == '-');
    if (negative) {
      ptr++; // advance past minus sign
      remaining--;
    }

    // Interpret next digit characters
    value = 0;
    while (remaining and isDigit(*ptr)) {
      value = value * 10 + (*ptr++ - '0'); // advances pointer, too
      remaining--;
      ok = true; // at least one digit received
    }

    if (negative)
      value = -value;
  }

  return ok;

} // parseValue


bool lineReady()
{
  bool          ready     = false;
  const char    endMarker = '\n';

  while (DEBUG_PORT.available()) {

    char c = DEBUG_PORT.read();

    if (c != endMarker) {

      // Only save the printable characters, if there's room
      if (/*(' ' <= c) and */ (count < MAX_CHARS-1)) {
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