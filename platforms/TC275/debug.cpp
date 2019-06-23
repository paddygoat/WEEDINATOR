#include "debug.h"
#include "encoder.h"
#include "TC275.h"       // Dont delete this !!!! 

////////////////////////////////////////////////////////////////

void debug()
{
  //DEBUG_PORT.print("Initial intervalThree= ");DEBUG_PORT.println(intervalThree);
  //int radiusRatio =3;
  //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
  //resultOne =  1/intervalThree;
  //DEBUG_PORT.print("resultOne: ");DEBUG_PORT.println(resultOne,8);
  //resultTwo = intervalOne*radiusRatio;
  //DEBUG_PORT.print("resultTwo: ");DEBUG_PORT.println(resultTwo);
  //resultThree = 1/resultTwo;
  //DEBUG_PORT.print("resultThree: ");DEBUG_PORT.println(resultThree,8);
  //resultFour = resultTwo - resultThree;
  //DEBUG_PORT.print("resultFour: ");DEBUG_PORT.println(resultFour,8);
  //intervalThree = 1/resultFour;
  //DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree,8);
    
  DEBUG_PORT.println();
  
  DEBUG_PORT.print("Encoder count:      ");DEBUG_PORT.println(encoderCount);
  DEBUG_PORT.print("Encoder direction:  ");DEBUG_PORT.println(encoderDirection);
  DEBUG_PORT.print("Current encoder value:  ");DEBUG_PORT.println(currentEncoderValue);
  
  //DEBUG_PORT.print("bothXAxisLimitSwitches:  ");DEBUG_PORT.println(bothXAxisLimitSwitches);
  //DEBUG_PORT.print("LSFXLHS:  ");DEBUG_PORT.println(LSFXLHS);
  //DEBUG_PORT.print("maxCurrentValueFive:  ");DEBUG_PORT.println(maxCurrentValueFive,2);
  //DEBUG_PORT.print("maxCurrentValueSix:  ");DEBUG_PORT.println(maxCurrentValueSix,2);
  
  //DEBUG_PORT.print("LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
  //DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
  //DEBUG_PORT.print("  samples per cycle:  ");DEBUG_PORT.println(icMax);
  //DEBUG_PORT.print("Integer latitude  UBLOX:  ");DEBUG_PORT.println(latitudeUblox);
  //DEBUG_PORT.print("Integer longitude UBLOX:  ");DEBUG_PORT.println(longitudeUblox); 
  //DEBUG_PORT.print("Bearing    = ");DEBUG_PORT.println(bearingDegrees);
  //DEBUG_PORT.print("Heading    = ");DEBUG_PORT.println(headingDegrees);
  //DEBUG_PORT.print("Way point  = ");DEBUG_PORT.println(navData.waypointID());
  //DEBUG_PORT.print("distanceMM = ");DEBUG_PORT.println(distanceMM);
  //DEBUG_PORT.println();
  DEBUG_PORT.print("ControlState value = ");DEBUG_PORT.println(controlState);
  DEBUG_PORT.print("NavState  =          ");DEBUG_PORT.println(navState);
  DEBUG_PORT.print("Final Steering Value= ");DEBUG_PORT.println(finalSteeringValue);
  DEBUG_PORT.print("Make Turn Value= ");DEBUG_PORT.println(makeTurnValue);
  //DEBUG_PORT.print("Previous steering Value= ");DEBUG_PORT.println(previousFinalSteeringValue);
  //DEBUG_PORT.print("Difference= ");DEBUG_PORT.println(difference);
  //DEBUG_PORT.print("wheelsPosition= ");DEBUG_PORT.println(wheelsPosition);
  DEBUG_PORT.print("Final Drive Value = ");DEBUG_PORT.println(finalDriveValue);
  DEBUG_PORT.print("JetsonReading = ");DEBUG_PORT.println(jetsonReading);
  //DEBUG_PORT.print("speedTimerAsyncLeftA =  ");DEBUG_PORT.println(speedTimerAsyncLeftA);
  //DEBUG_PORT.print("intervalTimerAsyncRightA =  ");DEBUG_PORT.println(intervalTimerAsyncRightA);  
  //DEBUG_PORT.print("intervalTimerAsyncLeftA =   ");DEBUG_PORT.println(intervalTimerAsyncLeftA); 
  DEBUG_PORT.print("preventSteering =   ");DEBUG_PORT.println(preventSteering);
  
  //DEBUG_PORT.print("Steering Value= ");DEBUG_PORT.println(steeringValue);
  //DEBUG_PORT.print("analogueRead A1= ");DEBUG_PORT.println(driveValue);
  //DEBUG_PORT.print("IntervalOne= ");DEBUG_PORT.println(intervalOne);
  //DEBUG_PORT.print("IntervalTwo= ");DEBUG_PORT.println(intervalTwo);
  //DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree);
  //DEBUG_PORT.print("IntervalFour= ");DEBUG_PORT.println(intervalFour);
  //DEBUG_PORT.print("velocityControlLeft= ");DEBUG_PORT.println(velocityControlLeft);
  //DEBUG_PORT.print("velocityControlRight= ");DEBUG_PORT.println(velocityControlRight); 
  //DEBUG_PORT.print("ATSDState= ");DEBUG_PORT.println(ATSDState);
  //DEBUG_PORT.print("Stationary state= ");DEBUG_PORT.println(stationary);
  //DEBUG_PORT.print("Pixy pan data  = ");DEBUG_PORT.println(pixyPanData);
  //DEBUG_PORT.print("Pixy bar data  = ");DEBUG_PORT.println(pixyBarData);
  //DEBUG_PORT.print("Pixy barcode  = ");DEBUG_PORT.println(pixyBarcode);

  DEBUG_PORT.print("move2ColumnsForwards: ");DEBUG_PORT.println(move2ColumnsForwards);
  DEBUG_PORT.print("barcodeReached: ");DEBUG_PORT.println(barcodeReached);
  //DEBUG_PORT.print("emicCount: ");DEBUG_PORT.println(emicCount);
  //DEBUG_PORT.print("  LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
  //DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
  //DEBUG_PORT.print("finalCurrentSensorValueRAxis:  ");DEBUG_PORT.println(finalCurrentSensorValueRAxis);
  //DEBUG_PORT.print("currentSensorValueRAxis:       ");DEBUG_PORT.println(currentSensorValueRAxis);  
  //DEBUG_PORT.print("runningCurrentValueRAxis:      ");DEBUG_PORT.println(runningCurrentValueRAxis);
  DEBUG_PORT.print("CNC operation:: ");DEBUG_PORT.println(operationNumber);
  
//  if(forwards==HIGH)
//    {
//    DEBUG_PORT.println("FORWARDS");   
//    }
//  if(backwards==HIGH)
//    {
//    DEBUG_PORT.println("BACKWARDS");   
//    }
//  if(rightHandLock==HIGH)
//    {
//    DEBUG_PORT.println("RIGHT HAND LOCK");   
//    }
//  if(leftHandLock==HIGH)
//    {
//    DEBUG_PORT.println("LEFT HAND LOCK");   
//    }
//  if(clockW==HIGH)
//    {
//    DEBUG_PORT.println("GOING CLOCKWISE");   
//    }
//  if(antiClockW==HIGH)
//    {
//    DEBUG_PORT.println("GOING ANTI-CLOCKWISE");   
//    }
//  DEBUG_PORT.println("");
}      
////////////////////////////////////////////////////////////////
