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
int32_t flagValue; 
int32_t barcode;
int32_t barcodeValue;
int8_t i;
////////////////////////////////////////////////////////////////////////////

void initPixy()
{
  digitalWrite(PIXY_PROCESSING,HIGH);
  pixy.init();
  pixy.changeProg("line");
}

////////////////////////////////////////////////////////////////////////////
/* LINE_MODE_MANUAL_SELECT_VECTOR
    Normally, the line tracking algorithm will choose what it thinks is the best Vector line automatically. 
 *  Setting LINE_MODE_MANUAL_SELECT_VECTOR will prevent the line tracking algorithm from choosing the Vector automatically. 
 *  Instead, your program will need to set the Vector by calling setVector(). _MODE_MANUAL_SELECT_VECTOR
 *  uint8_t m_flags. This variable contains various flags that might be useful.
 *  uint8_t m_x0. This variable contains the x location of the tail of the Vector or line. The value ranges between 0 and frameWidth (79) 3) 
 *  int16_t m_angle . This variable contains the angle in degrees of the line.
 *  int8_t setLED(uint8_t r, uint8_t g, uint8_t b) . Sets brightness of LEDs.
*/
void pixyModule()
{
  if (not usePixy)
    return;
  int8_t res;
  int left, right;
  int r=150;             // LED
  int g=0;               // LED
  int b=0;               // LED
  char buf[96];
  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res =   pixy.line.getAllFeatures();
  //res =   pixy.line.getMainFeatures();
  // We found the vector...
  int8_t setLED(uint8_t r, uint8_t g, uint8_t b) ;
////////////////////////////////////////////////////////////////
  if (res&LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end (head) of the vector,
    // the part of the vector we're heading toward.
    panError = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;
    flagValue = (int32_t)pixy.line.vectors->m_flags;
    g=150;
    //DEBUG_PORT.print( F("Flag Value:       ") );DEBUG_PORT.println(flagValue);
    panError =  panError + 185;  // Lower value makes machine go anti clockwise.
    //pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    //headingLoop.update(panError);
  }
  else
  {
    g=0;
  }
/////////////////////////////////////////////////////////////////  
  if (res&LINE_BARCODE)
  {
    b=150;
    barcodeValue = (int32_t)pixy.line.barcodes->m_y;
    barcode = (uint8_t)pixy.line.barcodes->m_code;
  }
  else
  {
    b=0;
  }
///////////////////////////////////////////////////////////////// 
  //DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
  //DEBUG_PORT.print( F("Barcode Y position value:    ") );DEBUG_PORT.println(barcodeValue);
  //DEBUG_PORT.print( F("Barcode:    ") );DEBUG_PORT.println(barcode);
} // pixyModule

