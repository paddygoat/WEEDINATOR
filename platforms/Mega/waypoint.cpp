#include "waypoint.h"

#include "Mega.h"
#include "navdata.h"

waypoint_t waypoint_t::current;

RingBuffer <waypoint_t,waypoint_t::WAYPOINT_LOOKAHEAD> waypoint_t::track;

////////////////////////////////////////////////////////////////////////////

void waypoint_t::next()
{
  if (track.available()) {
    current = track.read();
    DEBUG_PORT.print( F("Going to Waypoint ID ") );
    DEBUG_PORT.println( current.id );
    updateNavData();
  } else {
    DEBUG_PORT.print( F("No more waypoints!") );
  }

} // next

