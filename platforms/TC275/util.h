#ifndef WEEDINATOR_UTIL_H
#define WEEDINATOR_UTIL_H

#include <stddef.h> // for size_t
#include <stdint.h>

extern void showData( char    *data, size_t n );
extern void showData( uint8_t *data, size_t n );

extern bool parseValue( char * & ptr, size_t & remaining, uint32_t & value );

extern bool findText
  ( const char *   text_P, const size_t   len,
          char * & ptr   ,       size_t & remaining );

#endif
