#ifndef WEEDINATOR_WAYPOINT_H
#define WEEDINATOR_WAYPOINT_H

#include <Location.h>
#include "RingBuffer.h"

static const uint16_t LAST_WAYPOINT_ID = 11; // this should come from DB

class waypoint_t
{
public:

  uint16_t           id;
  NeoGPS::Location_t location;

  waypoint_t() : id( 0 ) {};

  static waypoint_t current;
  static const uint16_t WAYPOINT_LOOKAHEAD = 2;
  static RingBuffer <waypoint_t,WAYPOINT_LOOKAHEAD> track;

  static void next(); // start using the next waypoint from the track
};

#endif