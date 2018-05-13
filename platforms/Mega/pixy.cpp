#include "pixy.h"

#include "Mega.h"
#include "compass.h"

extern      float bearingToWaypoint;  // degrees

#include <Arduino.h>
#include <Pixy.h>
static Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

int oldSignature;  //  <-- what is this used for?  Never set, so always 0
long maxSize;
long newSize;

////////////////////////////////////////////////////////////////////////////

void initPixy()
{
  digitalWrite(PIXY_PROCESSING,HIGH);
  pixy.init();
}

////////////////////////////////////////////////////////////////////////////

void pixyModule()
{
  if (not usePixy)
    return;

  static       uint8_t frameCount         = 0;
  static const uint8_t PRINT_FRAME_PERIOD = 50;

  static uint16_t blockCount = 0;
         uint16_t blocks     = pixy.getBlocks();

  if (blocks)
  {
    digitalWrite(PIXY_PROCESSING,HIGH);
    int32_t panError  = X_CENTER - pixy.blocks[0].x;

    int32_t tiltError = pixy.blocks[0].y - Y_CENTER;

    blockCount = blocks;

    // do this (print) every so often frames because printing every
    // frame would bog down the Arduino
    frameCount++;
    if (frameCount >= PRINT_FRAME_PERIOD)
    {
      frameCount = 0;
      //DEBUG_PORT.print( F("Detected ") );
      //DEBUG_PORT.print( blocks );
      //DEBUG_PORT.println( ':' );

      for (uint16_t j=0; j<blocks; j++)
      {
        long size = pixy.blocks[j].height * pixy.blocks[j].width;
        DEBUG_PORT.print( F("No. of blocks: ") );DEBUG_PORT.println(blocks);
        DEBUG_PORT.print( F("Block no.:     ") );DEBUG_PORT.println(j+1);
        DEBUG_PORT.print( F("Size:          ") );DEBUG_PORT.println(size);
        DEBUG_PORT.print( F("Max. size:     ") );DEBUG_PORT.println(maxSize);
        DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
        DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);

        pixy.blocks[j].print();
        DEBUG_PORT.println();
      }
    }

    // Overide compass module with object recognition:
    if (panError > 300)
    {
      DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
      DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);
      //readCompass();
    }

    compass = bearingToWaypoint + panError*0.2; // <-- OK???

  } else {
    // No blocks
    digitalWrite(PIXY_PROCESSING,LOW);
  }
  //readCompass();

  int trackedBlock = 0;
  maxSize = 0;
  for (int k = 0; k < blockCount; k++)
  {
    if ((oldSignature == 0) || (pixy.blocks[k].signature == oldSignature))
    {
      newSize = pixy.blocks[k].height * pixy.blocks[k].width;

      if (newSize > maxSize)
      {
        //DEBUG_PORT.print( F("newSize:      ") );DEBUG_PORT.println(newSize);
        trackedBlock = k;
        maxSize = newSize;
        //DEBUG_PORT.print( F("maxSize:      ") );DEBUG_PORT.println(maxSize);
      }
    }
  }
} // pixyModule

