#ifndef WEEDINATOR_NAVDATA_H
#define WEEDINATOR_NAVDATA_H

extern       int waypointID;
static const int LAST_WAYPOINT_ID = 11;

extern       float bearingToWaypoint;  // degrees
extern       float distanceToWaypoint; // mm
static const float WAYPOINT_DISTANCE_THRESHOLD = 500.0; // mm

#include <Location.h>
extern NeoGPS::Location_t waypoint;

extern bool parseWaypoint( char *ptr, size_t remaining );

extern void initNavData();
extern void sendNavData();
extern void checkNavData();
extern void updateNavData();


#endif
