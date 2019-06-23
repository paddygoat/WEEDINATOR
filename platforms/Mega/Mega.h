#ifndef WEEDINATOR_MEGA_H
#define WEEDINATOR_MEGA_H

#include <stdint.h>

#include <HardwareSerial.h>
#define DEBUG_PORT Serial
const uint32_t DEBUG_BAUD = 115200;

#define gpsPort    Serial1

//#define SIMULATE_DEVICES  // comment this out for real system

#if defined( SIMULATE_DEVICES )

  const uint32_t GPS_BAUD   =  9600;

  const uint32_t FONA_BAUD  = DEBUG_BAUD;
  #define fonaPort   Serial

  static const bool useConsole = true;

#else

  const uint32_t GPS_BAUD   =  19200;

  #define fonaPort   Serial2
  const uint32_t FONA_BAUD  =   1200; // 4800;

  static const bool useConsole = false;

#endif

// encoder is on pin 2 and 3.
const int FONA_RST            = 5;
const int SPEAKER             = 10;
const int I2C_REQUEST         = 12;
const int I2C_RECEIVE         = 13;
const int BLUE_LED            = 37;
const int ORANGE_LED          = 39;
const int PIXY_PROCESSING     = 47;
const int USING_GPS_HEADING   = 49;

const uint8_t MEGA_I2C_ADDR   = 26;

////////////////////////////////////////////////
// Changing one of these flags to false will
//   disable the code for that device.  This is
//   handy for my testing.

static const bool usePixy    = not useConsole;
static const bool useCompass = not useConsole;
static const bool useI2C     = not useConsole;
static const bool useFona    = not useConsole;

#endif
