#ifndef WEEDINATOR_NAVDATA_H
#define WEEDINATOR_NAVDATA_H

#include <stdint.h>
#include "units.h"

static const float WAYPOINT_DISTANCE_THRESHOLD =     0.5; // meters
static const float WAYPOINT_FAR_AWAY           = 10000.0; // meters

extern void initNavData();
extern void updateNavData();
extern void sendNavData();

#include <Location.h>

class navData_t
{
private:
  NeoGPS::Location_t _location  ;
  uint32_t           _distance  ; // mm (meters in accessor/mutator)
  uint16_t           _waypointID;
  uint16_t           _bearing   ; // degrees * 100 (degrees in accessor/mutator)
  uint16_t           _heading   ; // degrees * 100 (degrees in accessor/mutator)
  int32_t            _panError;

public:
  navData_t()
    :
      _location  (0L,0L),
      _distance  (0),
      _waypointID(0),
      _bearing   (0),
      _heading   (0),
      _panError  (0)
    {}

  // Accessors
        uint16_t             waypointID() const { return _waypointID; }
  const NeoGPS::Location_t & location  () const { return _location  ; }
        float    bearing () const { return 0.01 * _bearing; } // degrees
        float    heading () const { return 0.01 * _heading; } // degrees
        int32_t  panError() const { return _panError      ; }
        float    distance() const { return _distance / MM_PER_M; } // m

  // Mutators
  void waypointID( const uint16_t           & id  ) { _waypointID = id ; }
  void location  ( const NeoGPS::Location_t & loc ) { _location   = loc; }
  void bearing   ( float    degrees  ) { _bearing = (degrees * 100.0    + 0.5); }
  void panError  ( uint32_t panError ) { _panError = panError; }
  void heading   ( float    degrees  ) { _heading = (degrees * 100.0    + 0.5); }
  void distance  ( float    meters   )
    {
      if (meters > WAYPOINT_FAR_AWAY)
        meters = WAYPOINT_FAR_AWAY;
      
      _distance = (meters * MM_PER_M + 0.5);
    }

  // Inter-processor messages
  void printTo ( uint8_t *bytes, size_t len );
  void readFrom( uint8_t *bytes, size_t len );

  static const size_t MSG_SIZE;
};

extern navData_t navData;

#endif
