#include "util.h"

#include <ctype.h>

#include "Mega.h"

////////////////////////////////////////////////////////////////////////////

bool parseValue( char * & ptr, size_t & remaining, uint32_t & value )
{
  bool ok = false;

  if (remaining > 0) {

    bool negative = (*ptr == '-');
    if (negative) {
      ptr++; // advance past minus sign
      remaining--;
    }

    // Interpret digit characters
    value = 0;
    while (remaining and isdigit(*ptr)) {
      value = value * 10 + (*ptr++ - '0'); // advances pointer, too
      remaining--;
      ok = true; // received at least one digit
    }

    if (negative)
      value = -value;
  }

  return ok;

} // parseValue

////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////

void showData( char    *data, size_t n )
{
  for (size_t i=0; i < n; i++) {

    if (isprint(data[i]))
      DEBUG_PORT.write( data[i] );
    else if (data[i] == 0x00)
      DEBUG_PORT.print( F("<NUL>") );
    else if (data[i] == 0x0A)
      DEBUG_PORT.print( F("<LF>") );
    else if (data[i] == 0x0D)
      DEBUG_PORT.print( F("<CR>") );
    else {
      // Some other value?  Just show its HEX value.
      DEBUG_PORT.print( F("<0x") );
      if (data[i] < 0x10)
        DEBUG_PORT.print( '0' );
      DEBUG_PORT.print( data[i], HEX );
      DEBUG_PORT.print( '>' );
    }
  }
}

void showData( uint8_t *data, size_t n )
{
  for (size_t i=0; i < n; i++) {
    if (data[i] < 0x10)
      DEBUG_PORT.print( '0' );
    DEBUG_PORT.print( data[i], HEX );
    DEBUG_PORT.print( ' ' );
  }

} // showData
