#ifndef NEOPRINTTORAM_H
#define NEOPRINTTORAM_H

namespace Neo {

//  Copyright (C) 2018, SlashDevin
//
//  This file is part of Neo::PrintToRAM
//
//  Neo::PrintToRAM is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  PrintToRAM is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Neo::PrintToRAM.  If not, see <http://www.gnu.org/licenses/>.

class PrintToRAM : public Print
{
    PrintToRAM( const PrintToRAM & );               // no copy constructor
    PrintToRAM & operator = ( const PrintToRAM & ); // no assignment

  public:
    PrintToRAM( void *ram, size_t size )
      : _RAM( (uint8_t *) ram ), _size( size ), _count( 0 )
        {};
    ~PrintToRAM() {};

    size_t write( uint8_t b )
    {
      if (_count < _size) {
        _RAM[ _count++ ] = b;
        return 1;
      } else {
        return 0;
      }
    }

    using Print::write; // pull in all the other write methods

    virtual void flush()
    {
      _count = 0;
    }

    size_t writeTo( Print & output )
    {
      output.write( _RAM, _count );
      size_t numWritten = _count;
      flush();
      return numWritten;
    }

    size_t numWritten() const { return _count; };
    void   terminate() { write( (uint8_t) 0 ); }; // add a NUL byte for C strings

  protected:
    uint8_t *_RAM;
    size_t   _size;
    size_t   _count;
};

}; // namespace Neo

#endif
