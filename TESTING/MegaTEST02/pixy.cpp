#include "pixy.h"

#include "Mega.h"
#include "compass.h"
#include "navdata.h"

#include <Arduino.h>
#include <Pixy2.h>
#include <PIDLoop.h>

#define X_CENTER         (pixy.frameWidth/2)
Pixy2 pixy;
PIDLoop headingLoop(5000, 0, 0, false);
int32_t panError; 
////////////////////////////////////////////////////////////////////////////

void initPixy()
{
  digitalWrite(PIXY_PROCESSING,HIGH);
  pixy.init();
  pixy.changeProg("line");
}

////////////////////////////////////////////////////////////////////////////

void pixyModule()
{
  if (not usePixy)
    return;
  int8_t res;
  int left, right;
  char buf[96];
  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();
  // We found the vector...
  if (res&LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    panError = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    //pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(panError);
  }
  //DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);

} // pixyModule

