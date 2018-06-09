#ifndef WEEDINATOR_FONA_H
#define WEEDINATOR_FONA_H

#include <stdint.h>

extern void initFONA();
extern void loadWaypoints();
extern bool getWaypoint( uint16_t id );

#include <stddef.h>
class waypoint_t;
extern bool parse( waypoint_t &waypoint, char *ptr, size_t remaining );


#endif
