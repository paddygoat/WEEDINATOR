#include "util.h"

#include <ctype.h>

#include "TC275.h"

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
