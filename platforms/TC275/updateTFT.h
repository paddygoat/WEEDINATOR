#ifndef WEEDINATOR_UPDATETFT_H
#define WEEDINATOR_UPDATETFT_H

#include "steering.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#define TFT_CS     40
#define TFT_RST    24  // you can also connect this to the Arduino reset
#define TFT_DC     22


extern void setupTFT();
extern void updateTFT();
extern void rectangle1();
extern void rectangle2();

#endif
