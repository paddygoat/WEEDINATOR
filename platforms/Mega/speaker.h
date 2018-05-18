#ifndef WEEDINATOR_SPEAKER_H
#define WEEDINATOR_SPEAKER_H

#include <stdint.h>

const uint16_t BEEP_FREQUENCY   = 1000; // Hz
extern void beep( uint32_t duration, uint16_t freq = BEEP_FREQUENCY );
extern void checkBeep();

#endif
