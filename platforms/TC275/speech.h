#ifndef WEEDINATOR_SPEECH_H
#define WEEDINATOR_SPEECH_H

#include "steering.h"

extern int emicCount;
extern String cmd; 
extern int emicTimerInterval;
extern String text4;
extern String text1;
extern String text2;
extern String text3;
extern String text6;
extern String text7;


extern void emicSpeech1();
extern void emicIntro();
extern void emicDetect();
extern void emptyEMICport();

#endif
