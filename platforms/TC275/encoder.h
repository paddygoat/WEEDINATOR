#ifndef WEEDINATOR_ENCODER_H
#define WEEDINATOR_ENCODER_H

extern bool volatile encoderDirection;
extern uint16 volatile encoderCount;
extern int currentEncoderValue;

extern void IncrEnc_Init();
extern void encoderReadings();

#endif
