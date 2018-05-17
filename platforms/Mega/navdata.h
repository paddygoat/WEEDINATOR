#ifndef WEEDINATOR_NAVDATA_H
#define WEEDINATOR_NAVDATA_H

extern       float bearingToWaypoint;  // degrees
extern       float distanceToWaypoint; // mm
static const float WAYPOINT_DISTANCE_THRESHOLD = 500.0; // mm

extern void initNavData();
extern void sendNavData();
extern void checkNavData();
extern void updateNavData();


#endif
