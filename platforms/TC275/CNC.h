#ifndef WEEDINATOR_CNC_H
#define WEEDINATOR_CNC_H

extern float intervalCNCXOne;
extern float intervalCNCYOne;
extern float intervalCNCZOne;
extern float intervalCNCROne;
extern int speedCNCX;
extern int speedCNCY;
extern int speedCNCZ;
extern int speedCNCR;
extern unsigned long prevMicrosCNCXOne; 
extern unsigned long prevMicrosCNCYOne; 
extern unsigned long prevMicrosCNCZOne; 
extern unsigned long prevMicrosCNCROne; 
extern int XZeroingState;
extern int YZeroingState;
extern int ZZeroingState;
extern long XZeroingStepsTotal;
extern long YZeroingStepsTotal;
extern long ZZeroingStepsTotal;
extern long XZeroingStep;
extern long YZeroingStep;
extern long ZZeroingStep;
extern uint8_t LSFXRHS;          // RHS Switch open x axis.
extern uint8_t LSFXLHS;          // LHS Switch open x axis.
extern uint8_t LSBX;
extern uint8_t LSRY;          // Switch open y axis. LSRY = limit switch right y.
extern uint8_t LSLY;          // D43
extern uint8_t LSUZ;          // D41
extern uint8_t LSDZ;         // limit switch Z down.
extern int dirStateCNCX; 
extern int dirStateCNCY; 
extern int dirStateCNCZ;
extern int dirStateCNCR; 

extern int stepStateCNCX;
extern int stepStateCNCY;
extern int stepStateCNCZ;
extern int stepStateCNCR;

extern uint8_t setupStateX;           // Change to HIGH to skip CNC setups.
extern uint8_t setupStateY;
extern uint8_t setupStateZ;

extern String operationNumber;

extern bool operationZero;
extern bool operationOne;
extern bool operationTwo;
extern bool operationThree;
extern bool operationFour;
extern bool operationFive;
extern bool operationSix;
extern bool operationSeven;
extern bool operationEight;
extern bool operationNine;
extern bool operationTen;
extern bool operationEleven;
extern bool operationTwelve;
extern bool operationThirteen;
extern bool operationFourteen;
extern bool operationFifteen;
extern bool operationSixteen;
extern bool operationSeventeen;
extern bool operationEighteen;
extern bool operationNineteen;
extern bool operationTwenty;
extern bool operationTwentyOne;

extern bool operationState;
extern bool weedingBegin;
extern bool move2ColumnsForwards;
extern bool bothXAxisLimitSwitches;

extern int controlState;  // Manual or autonomous(HGH)
extern int finalCurrentSensorValueRAxis;

extern float speedTimerAsyncRightA;
extern float speedTimerAsyncRightB;
extern float speedTimerAsyncLeftA;
extern float speedTimerAsyncLeftB;

extern bool preventSteering;
extern bool barcodeReached;

extern int navState;        // Use Pixy or GPS

extern int pixyPanData;
extern int pixyBarData;
extern int pixyBarcode;

extern void CNC_TIMER_X();
extern void CNC_TIMER_Y();
extern void CNC_TIMER_Z();
extern void CNC_TIMER_R();

extern void CNC_SETUP_X();
extern void CNC_SETUP_Y();
extern void CNC_SETUP_Z();

extern void opZero();
extern void opOne();
extern void opTwo();
extern void opThree();
extern void opFour();
extern void opFive();
extern void opSix();
extern void opSeven();
extern void opEight();
extern void opNine();
extern void opTen();
extern void opEleven();
extern void opTwelve();
extern void opThirteen();
extern void opFourteen();
extern void opFifteen();


extern void moveRightMotor();
extern void moveLeftMotor();
extern void weedingBegins();
extern void weeding();
extern void move2ColumnsForward();
#endif
