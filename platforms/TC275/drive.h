#ifndef WEEDINATOR_DRIVE_H
#define WEEDINATOR_DRIVE_H

#include "CNC.h"

extern int finalDriveValue;

extern unsigned long previousMicrosTimerAsyncLeftA; 
extern unsigned long previousMicrosTimerAsyncLeftB;
extern unsigned long previousMicrosTimerAsyncRightA; 
extern unsigned long previousMicrosTimerAsyncRightB;

extern unsigned long intervalTimerAsyncLeftA;
extern unsigned long intervalTimerAsyncLeftB;
extern unsigned long intervalTimerAsyncRightA;
extern unsigned long intervalTimerAsyncRightB;

extern int asyncStateRight;
extern int asyncStateLeft;

extern float intervalOne; // Right hand wheel turn speed. Bigger is faster.
extern float intervalTwo; // Left hand wheel turn speed.
extern float intervalThree; // Right hand wheel drive speed. Is this left wheel?
extern float intervalFour; // Left hand wheel drive speed.

extern float runningmaxCurrentValueFive;
extern float runningmaxCurrentValueSix;

extern long ferrets;
extern long wheelsPosition;
extern int stationary;

extern float headingDegrees;
extern float bearingDegrees;
extern long distanceMM;
extern long latitudeUblox;
extern long longitudeUblox;

extern void ampflowMotor();
extern void moveMotors();
extern void moveRightMotor();
extern void moveLeftMotor();
extern void torqueDifferential();
extern void speedDifferential();

#endif
