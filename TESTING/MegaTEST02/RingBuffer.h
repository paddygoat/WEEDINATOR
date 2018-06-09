#ifndef RINGBUFFER_H
#define RINGBUFFER_H

template <class T, uint8_t N>
class RingBuffer
{
private:
  uint8_t     _head, _tail;
  uint8_t     _available;

  static const uint8_t MAX_ELEMENTS = N;

  T sample[ MAX_ELEMENTS ];

public:
  RingBuffer() { flush(); }

  void flush()
    {
      _head =
      _tail =
      _available = 0;
    };

  uint8_t available() const { return _available; };
  uint8_t room     () const { return MAX_ELEMENTS - _available; };

  T read()
    {
      T s;  // returns a default/uninitialized T if none available

      if (_available) {
        _available--;
        s = sample[ _head++ ];
        if (_head >= MAX_ELEMENTS)
          _head = 0;

      }

      return s;
    };

  bool write( const T &s )
    {
      bool room = (_available < MAX_ELEMENTS);

      if (room) {
        _available++;
        sample[ _tail++ ] = s;
        if (_tail >= MAX_ELEMENTS)
          _tail = 0;
      }

      return room;
    };

};
#endif
