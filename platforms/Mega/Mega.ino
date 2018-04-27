
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>  

#include <Pixy.h>
Pixy pixy;
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag(12345);

#include <NMEAGPS.h>
NMEAGPS gps;

NeoGPS::Location_t base( 532558000L, -43114000L ); // Llangefni

String initiator;
String dataString;
char url[120];
long x = 12345678;
long zz;
long yy;
float bearing;
int zbearing;
long zdistance;
String latString;
String lonString;
String lat;
String lon;
String a;
long latitude;
long longitude;
char c;
long result;
int sendDataState = LOW; 
unsigned long millisCalc1; 
unsigned long previousMillis1;
int ledState;
float xxBlob;
float yyBlob;
float xxx;
float yyy;
float zzz;
float Pi = 3.14159;
double compass;
float autoXMax = -1000;
float autoXMin =  1000;
float autoYMax = -1000;
float autoYMin =  1000;
unsigned long compassCount =0;
int ledBlueState = LOW;
int oldX, oldY, oldSignature;
int blockCount;
long maxSize;
long newSize;
float zheading;
float ubloxBearing;

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
}
#define DEBUG_PORT Serial
#define gpsPort    Serial1



void setup()
{
  pixy.init();
  DEBUG_PORT.begin(115200);
  gps_port.begin(19200);
  Wire.begin(26);                // join i2c bus with address #25
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  mag.enableAutoRange(true);
  /* Initialise the sensor */
  DEBUG_PORT.println("TEST2");
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    DEBUG_PORT.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  pinMode(37,OUTPUT);
  pinMode(39,OUTPUT);
  pinMode(12,OUTPUT);  
  pinMode(13,OUTPUT);  
  pinMode(49,OUTPUT);
  pinMode(51,OUTPUT);  
  pinMode(53,OUTPUT); 
  digitalWrite(49,HIGH);
  digitalWrite(47,HIGH);
  digitalWrite(53,LOW);
  tone(10,2500,1000);   // pin,pitch,duration
  delay(1000);
  digitalWrite(47,LOW);
  digitalWrite(49,LOW);
  noTone(10);
} // setup

void loop()
{

  //NeoGPS::Location_t base( latitude, longitude ); // Data from FONA module
  while (gps.available( gps_port )) 
  {
    gps_fix fix = gps.read(); // save the latest
    NeoGPS::Location_t base( latitude, longitude ); // Data from FONA module
    if (fix.valid.location)
    {
      tone(10,2500,100);   // pin,pitch,duration
      digitalWrite(39,HIGH);
      delay(100);
      noTone(10);
      float range = fix.location.DistanceKm( base );
      zheading = fix.heading();


      //float heading = prevFix.location.BearingToDegrees ( currentFix.location );
      //float bob = fix.location();
      //zheading = fix.location.BearingToDegrees ( currentFix.location );
      //prevFix = fix;

      
      DEBUG_PORT.print("Zheading:             ");DEBUG_PORT.println(zheading,2);      
      //DEBUG_PORT.print("Range:             ");DEBUG_PORT.println(range,9);  
      zdistance = range*1000*1000;  // Km to mm
      DEBUG_PORT.print("Distance mm:             ");DEBUG_PORT.println(zdistance);     
      ubloxBearing = fix.location.BearingTo( base )*57.2958; // Radians to degrees
      bearing = ubloxBearing;
      zbearing = ubloxBearing * 100; //Float to integer. zbearing is sent to TC275.
      DEBUG_PORT.print("Bearing:         ");DEBUG_PORT.println(ubloxBearing,5); 
      DEBUG_PORT.print("Compass Heading: ");DEBUG_PORT.println(compass); 
      Serial.println("");
    } 
    //else     // If there's no valid fix, machine will drive in straight line until fix recieved or pixie overides it.
    //{
    //  compass = ubloxBearing;
    //}
      // Waiting...
      //DEBUG_PORT.print( '.' );
      zz = (fix.location.lat()); // zz is a 'long integer'.
      yy = (fix.location.lon());
      //DEBUG_PORT.print("Current Ublox latitude:  ");DEBUG_PORT.println(zz);
      //DEBUG_PORT.print("Latitude from Fona:      ");DEBUG_PORT.println(latitude);
      //DEBUG_PORT.print("Current Ublox longitude: ");DEBUG_PORT.println(yy);
      //DEBUG_PORT.print("Longitude from Fona:     ");DEBUG_PORT.println(longitude);
      //DEBUG_PORT.println("");
      
      if (sendDataState == LOW) // This switches data set that is transmitted to TC275 via I2C.
      {
        characterCompileA(); // Bearing, distance and compass
        sendDataState = HIGH;
      }
      else 
      {
        characterCompileB(); // Longitude and latitude
        sendDataState = LOW;
      }
      compassModule();
  }// While GPS is available
  digitalWrite(39,LOW); // Orange LED
  blueLED();
  
///////////////////////////////////////////////////////////////////////
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError = 500; 
  int32_t tiltError = 500;
  int trackedBlock = 0;
  panError = 500;
  tiltError = 500;   
  
  blocks = pixy.getBlocks();

  if (blocks)
  {
    digitalWrite(47,HIGH);
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER; 
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    blockCount = blocks;
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0) 
    {
      //sprintf(buf, "Detected %d:\n", blocks);
      Serial.println(buf); // Empties serial buffer.
      for (j=0; j<blocks; j++)
      {
        long size = pixy.blocks[j].height * pixy.blocks[j].width;   
        Serial.print("No. of blocks: ");Serial.println(blocks);
        Serial.print("Block no.:     ");Serial.println(j+1);
        Serial.print("Size:          ");Serial.println(size);
        Serial.print("Max. size:     ");Serial.println(maxSize);
        Serial.print("PAN POS:       ");Serial.println(panError);
        Serial.print("TILT POS:      ");Serial.println(tiltError);
        //sprintf(buf, "  block %d: ", j);
        //Serial.print(buf); 
        pixy.blocks[j].print();
        Serial.println("");
      }
    }
  // Overide compass module with object recognition:
    if(panError > 300)
    {   
    Serial.print("PAN POS:       ");Serial.println(panError);
    Serial.print("TILT POS:      ");Serial.println(tiltError); 
    //compassModule();
    }
    
    compass = bearing + panError*0.2;  
     
  }// if (blocks) end
  else
  {
    digitalWrite(47,LOW);
  }
  //compassModule();
  maxSize = 0;
  for (int k = 0; k < blockCount; k++)
  {
    if ((oldSignature == 0) || (pixy.blocks[k].signature == oldSignature))
    {
      newSize = pixy.blocks[k].height * pixy.blocks[k].width;

      if (newSize > maxSize)
      {
        //Serial.print("newSize:      ");Serial.println(newSize);
        trackedBlock = k;
        maxSize = newSize;
        //Serial.print("maxSize:      ");Serial.println(maxSize);
      }
    }
  }
////////////////////////////////////////////////////////////////////////////
} // Main loop end
void blueLED()
{
  digitalWrite(37, ledBlueState);
  unsigned long currentMillis = millis();
  millisCalc1 = currentMillis - previousMillis1;
  if (millisCalc1 >= 200)                           // timer  .... 2,000
  {  
    if (ledBlueState == LOW) 
    {
      ledBlueState = HIGH;
    } 
    else 
    {
      ledBlueState = LOW;
     }
     //compassCount++;
     previousMillis1 = currentMillis;
  }
}
void requestEvent() 
{
  //Serial.println("Request event start  ");
  Wire.write(url);     // as expected by master
  digitalWrite(12,HIGH);
  delay(100);
  digitalWrite(12,LOW);
  //Serial.println("Request event end  ");
}
void receiveEvent(int howMany) // Recieves lat and long data from FONA via TC275 for calculating bearings and distances.
{
    a="";
    lat="";
    lon="";
    latString="";
    lonString="";
  //digitalWrite(13,HIGH);
  delay(100);
  //digitalWrite(13,LOW);
  //Serial.println("Recieve event start  ");
  //DEBUG_PORT.println("Here is data from TC275: "); 
  while (Wire.available())
  {
    char c = Wire.read(); // receive byte as a character
   // DEBUG_PORT.print(c); //DEBUG_PORT.print("Q");          // print the character
      if (isAlpha(c))         // analyse c for letters
      {
      //DEBUG_PORT.print("LETTER");
      a=a+c;                // string = string + character
      if (a=="LAT")
      {
       lon="";
       lat=a;
       //DEBUG_PORT.print(" Trigger word LAT detected!: ");//DEBUG_PORT.println(b);
       a="";
      } 
      if (a=="LONG")
      {
       lat="";
       lon=a;
       //DEBUG_PORT.print(" Trigger word LONG detected!: ");//DEBUG_PORT.println(d);
       a="";
      } 
    } 
      if (lat=="LAT")
      {
        if (isDigit(c))         // analyse c for numerical digit
        {
        latString=latString+c;                // string = string + character
        }
      }
      if (lon=="LONG")
      {
        if (isDigit(c))         // analyse c for numerical digit
        {
        lonString=lonString+c;                // string = string + character
        }
      }
  }    // While Loop
  result=(latString).toInt();
  if(result!=0){latitude = result;}
  result=(lonString).toInt();
  if(result!=0){longitude = result *-1;}
  //DEBUG_PORT.print("latString: ");DEBUG_PORT.println(latString);
  //DEBUG_PORT.print("lonString: ");DEBUG_PORT.println(lonString);
  //DEBUG_PORT.print("latitude integer from Fona:  ");DEBUG_PORT.println(latitude);
  //DEBUG_PORT.print("longitude integer from Fona: ");DEBUG_PORT.println(longitude);
  //DEBUG_PORT.println();
  //NeoGPS::Location_t base( latitude, longitude);
  //Serial.println("Recieve event end  ");
}
void lookForLettersAndDigits()
{
  lookForLetters();
  lookForDigits(); 
  lookForLatitude();
  lookForLongitude();
}
void lookForLetters()
{
    if (isAlpha(c))         // analyse c for letters
    {
      //DEBUG_PORT.print("LETTER");
      a=a+c;                // string = string + character
      if (a=="LAT")
      {
       lon="";
       lat=a;
       //DEBUG_PORT.print("Trigger word LAT detected!: ");//DEBUG_PORT.println(b);
       a="";
      } 
      if (a=="LON")
      {
       lat="";
       lon=a;
       //DEBUG_PORT.print("Trigger word LON detected!: ");//DEBUG_PORT.println(d);
       a="";
      } 
    } 
}
void lookForDigits()
{
  if (lat=="LAT")
  {
    if (isDigit(c))         // analyse c for numerical digit
    {
      latString=latString+c;                // string = string + character
    }
  }
  if (lon=="LON")
  {
    if (isDigit(c))         // analyse c for numerical digit
    {
      lonString=lonString+c;                // string = string + character
    }
  }
}
void lookForLatitude()
{
  if (lat=="LAT")
  {
      result=(latString).toInt();
      latitude = result;
      //DEBUG_PORT.println("");
      //DEBUG_PORT.print("Latitude: ");DEBUG_PORT.println(latitude);
  }
}
void lookForLongitude()
{
  if (lon=="LON")
  {
      result=(lonString).toInt();
      longitude = result;
      //DEBUG_PORT.println("");
      //DEBUG_PORT.print("Longitude: ");DEBUG_PORT.println(longitude);
  }
}
void characterCompileA() // For sending Ublox bearing and distance data to TC275
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is where the data is compiled into a character ....
  dataString =  initiator  + "BEAR" + zbearing + "DIST" + zdistance + "COMP" + compass;  // Limited to 32 characters for some reason !!! ..... + ",LON" + yy .... Removed.
  int n = dataString.length();
  //DEBUG_PORT.print("Data string to send:     "); DEBUG_PORT.println(dataString);   
  //DEBUG_PORT.print("Size of string:  "); DEBUG_PORT.println(n);
  // Builds the url character:
      for (int aa=0;aa<=n;aa++)                                              
      {
          url[aa] = dataString[aa];
      }
   //DEBUG_PORT.print("Character data to send:  "); DEBUG_PORT.println(url);
  // DEBUG_PORT.println("");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
}
void characterCompileB() // For sending Ublox lat and long to TC275
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is where the data is compiled into a character ....
  dataString =  initiator  + "LOON" + yy + "LAAT" + zz;  // Limited to 32 characters for some reason !!! ..... + ",LON" + yy .... Removed.
  int n = dataString.length();
  // DEBUG_PORT.print("Data string to send:     ");// DEBUG_PORT.println(dataString);   
   //DEBUG_PORT.print("Size of string:  ");// DEBUG_PORT.println(n);
  // Builds the url character:
      for (int aa=0;aa<=n;aa++)                                              
      {
          url[aa] = dataString[aa];
      }
   //DEBUG_PORT.print("Character data to send to TC275:  "); DEBUG_PORT.println(url);
   //DEBUG_PORT.println("");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
}
void compassModule()
{
  //DEBUG_PORT.println("####################### 1"); 
  sensors_event_t magEvent; 
  //DEBUG_PORT.println("####################### 1.5"); 
  delay(100);
  mag.getEvent(&magEvent);  // This is where the problem is!!!!!!!!!!!!!!!!
  //DEBUG_PORT.println("####################### 2");  
// Observed readings (integers)
int xMin =  -19.45;
int xMax =  23.27;
int yMin = -49.82;
int yMax =  -9.82;
  //DEBUG_PORT.println("####################### 3");
  xxBlob = magEvent.magnetic.x;
  yyBlob = magEvent.magnetic.y;
  //DEBUG_PORT.println("####################### 4");
if((xxBlob!=0)&&(yyBlob!=0))
  {
  if(xxBlob>autoXMax)
    {
      autoXMax = xxBlob;
    }
  if(xxBlob<autoXMin)
    {
      autoXMin = xxBlob;
    }   
  if(yyBlob>autoYMax)
    {
      autoYMax = yyBlob;
    }
  if(yyBlob<autoYMin)
    {
      autoYMin = yyBlob;
    }      
  }

// Now normalise to min -50 and max 50:
  //DEBUG_PORT.println("####################### 5");
  xxx = ((magEvent.magnetic.x - xMin)*100/(xMax-xMin))-50;
  yyy = ((magEvent.magnetic.y - yMin)*100/(yMax-yMin))-50;
  //DEBUG_PORT.println("####################### 6");
  compass = (((atan2(yyy,xxx) * 180) / Pi));
  compass = compass + 270 +15; //Lower this value to make clockwise turn.
  
  if (zheading!=0)
  {
    compass = zheading; // Overide compass with GPS derived heading
    digitalWrite(49,HIGH);
  }
  else                  // Makes the machine drive straight ahead.
  {
    compass = zbearing/100;
    digitalWrite(49,LOW);
  }
  
  DEBUG_PORT.print("autoXMin:  "); DEBUG_PORT.println(autoXMin);
  DEBUG_PORT.print("autoXMax:  "); DEBUG_PORT.println(autoXMax);
  DEBUG_PORT.print("autoYMin:  "); DEBUG_PORT.println(autoYMin);
  DEBUG_PORT.print("autoYMax:  "); DEBUG_PORT.println(autoYMax);  
  //DEBUG_PORT.print("Compass Heading: ");DEBUG_PORT.println(compass); 
  //Serial.println("");
}
