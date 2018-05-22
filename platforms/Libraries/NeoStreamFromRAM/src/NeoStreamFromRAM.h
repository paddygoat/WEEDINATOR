#ifndef NEOSTREAMFROMRAM_H
#define NEOSTREAMFROMRAM_H

#include <NeoPrintToRAM.h>

namespace Neo {

//  Copyright (C) 2018, SlashDevin
//
//  This file is part of Neo::StreamFromRAM
//
//  Neo::StreamFromRAM is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  StreamFromRAM is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Neo::StreamFromRAM.  If not, see <http://www.gnu.org/licenses/>.

class StreamFromRAM : public PrintToRAM
{
    StreamFromRAM();                                      // no default constructor
    StreamFromRAM( const StreamFromRAM & );               // no copy constructor
    StreamFromRAM & operator = ( const StreamFromRAM & ); // no assignment

  public:
    StreamFromRAM( const void *ram, size_t size )
      : PrintToRAM( (void *) ram, size )
        {}
    ~StreamFromRAM() {};

    virtual int available() { return _size - _count; };

    virtual int read()
    {
      if (_size > _count)
        return _RAM[ _count++ ];
      else
        return -1;
    }

    size_t readBytes( uint8_t *buffer, size_t length )
    {
      size_t count = 0;
      while ((length-- > 0) and available())
        buffer[ count++ ] = read();
      return count;
    }

    virtual int peek()
    {
      if (_size > _count)
        return _RAM[ _count ];
      else
        return -1;
    }

    virtual void flush() {};

    void rewind()
    {
      _count = 0;
    }

    virtual size_t write( const uint8_t *buffer, size_t size )
    {
      _RAM   = (uint8_t *) buffer;
      _size  = size;
      _count = 0;
    }

  protected:
    using PrintToRAM::write; // hide other methods
};

}; // namespace Neo

#endif
