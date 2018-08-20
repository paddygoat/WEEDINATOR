#include "updateTFT.h"
#include "TC275.h"
#include "steering.h"

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
////////////////////////////////////////////////////////////////
void setupTFT()
{
  if (useTFT) 
  {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(0); 
    rectangle2();  
    tft.setTextColor(ILI9341_BLUE);
    tft.setTextSize(4);
    tft.setCursor(0, 20);
    tft.println("WEEDINATOR");
  }
}
void updateTFT() 
{
  if (not useTFT) {
    DEBUG_PORT.println( "TFT updated" );
    return;
  }

  tft.setRotation(0); 
  rectangle2();  
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(4);
  tft.setCursor(0, 20);
  tft.println("WEEDINATOR");
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(0, 72);
  tft.println("Heading:    "); 
  tft.setCursor(110, 72);
  tft.println(headingDegrees);
  tft.setCursor(185, 67);
  tft.println("o"); 
  tft.setCursor(0, 102);
  tft.println("Bearing:");
  tft.setCursor(110, 102);
  tft.println(bearingDegrees,2);
  tft.setCursor(185, 97);
  tft.println("o");
  tft.setCursor(0, 132);
  tft.println("Distance:        mm ");
  tft.setCursor(110, 132);
  tft.println(distanceMM);
  tft.setCursor(0, 160);
  tft.println("Waypoint:            ");
  tft.setCursor(110, 160);
  //tft.println(navData.waypointID());
  tft.setCursor(150, 160);
  tft.println(pixyBarcode);
  rectangle1();
  tft.setCursor(0, 190);
  tft.println("Pixy pan:    bar:  ");
  tft.setCursor(110, 190);
  tft.println(pixyPanData);
  tft.setCursor(210, 190);
  tft.println(pixyBarData);
    
  tft.setCursor(0, 218);
  //tft.println("Difference:     ");
  tft.setCursor(170, 218);
  //tft.println(difference);

  tft.setCursor(0, 246);
  //tft.println("Wheels Pos:     ");
  tft.setCursor(170, 246);
  //tft.println(wheelsPosition);
  
  //tft.setTextSize(1);
  //tft.setCursor(0, 274);
  //tft.println("FONA:   LAT             LON");
  //tft.setCursor(70, 274);
  //tft.println(latFona);
  //tft.setCursor(170, 274);
  //tft.println(lonFona);
  tft.setCursor(5, 215);
/////////////////////////////////////////////////////////////
  if(controlState==HIGH)
  {
    tft.println("    AUTO");
  }
  else
  {
    tft.println("   MANUAL");
  } 
  tft.setCursor(60, 215);  
  if(navState==HIGH)
  {
    tft.println("     * PIXY");
  }
  else
  {
    tft.println("     * GPS");
  }
//////////////////////////////////////////////////////////////
  tft.setTextSize(4);
  tft.setCursor(15, 240);
  //tft.println("UBLOX:  LAT             LON");
  //tft.setCursor(70, 284);
  tft.println(latitudeUblox);
  tft.setCursor(15, 280);
  tft.println(longitudeUblox);
  
  tft.setTextSize(2);    
  tft.setCursor(0, 302);
  //tft.println("Direction:     ");
  tft.setCursor(170, 302);
//  if(forwards==HIGH)
//    {
    //tft.println("FORWARDS"); 
//    }
//  if(backwards==HIGH)
//    {
    //tft.println("BACKWARDS");
//    }
  //tft.println(finalSteeringValue);

} // updateTFT
void rectangle1()
{
          tft.fillRect(0, 186, 250, 135, ILI9341_RED);
          // x,y,(from top left corner)w,h
}
void rectangle2()
{
          tft.fillRect(0, 65, 250, 118, ILI9341_RED);
          // x,y,(from top left corner)w,h
}
