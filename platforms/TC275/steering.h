#ifndef WEEDINATOR_STEERING_H
#define WEEDINATOR_STEERING_H

#include "drive.h"

extern int rightHandLock;
extern int leftHandLock;
extern int clockW;
extern int antiClockW;
extern int actuallySteering;
extern int difference;
extern int previousFinalSteeringValue;

extern long finalSteeringValue;
extern int forwards;
extern int backwards;

extern float resultOne;
extern float resultTwo;
extern float resultThree;
extern float resultFour;

extern unsigned long previousMicrosOne;
extern unsigned long previousMicrosTwo;
extern unsigned long previousMicrosThree;
extern unsigned long previousMicrosFour;

extern int ledStateOne;   
extern int ledStateTwo; 
extern int ledStateThree;
extern int ledStateFour;

extern void turningWhenStationary();
extern void antiClockWise();
extern void clockWise();
extern void differential();

extern void changeStateOne();
extern void changeStateTwo();
extern void changeStateThree();
extern void changeStateFour();

#endif
