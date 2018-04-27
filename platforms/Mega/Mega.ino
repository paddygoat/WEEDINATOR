#define DEBUG_PORT Serial
#define gpsPort    Serial1
const uint32_t GPS_BAUD = 19200;


const int SPEAKER             = 10;
const int I2C_REQUEST         = 12;
const int I2C_RECEIVE         = 13;
const int BLUE_LED            = 37;
const int ORANGE_LED          = 39;
const int PIXY_PROCESSING     = 47;
const int USING_GPS_HEADING   = 49;

const uint8_t MEGA_I2C_ADDR   = 26;

const uint32_t BLUE_LED_BLINK_PERIOD =  200; // ms
const uint16_t BEEP_FREQUENCY        = 2500; // Hz


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

#include <Pixy.h>
Pixy pixy;
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag( 12345 );

#include <NMEAGPS.h>
NMEAGPS gps;

NeoGPS::Location_t base( 532558000L, -43114000L ); // Llangefni

const float MM_PER_M  = 1000.0;
const float M_PER_KM  = 1000.0;
const float MM_PER_KM = MM_PER_M * M_PER_KM;

String initiator;
String dataString;
char url[120];

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

int sendDataState = LOW;
unsigned long millisCalc1;
unsigned long previousMillis1;

double compass;
float autoXMax = -1000;
float autoXMin =  1000;
float autoYMax = -1000;
float autoYMin =  1000;
unsigned long compassCount =0;
int ledBlueState = LOW;
int oldSignature;  //  <-- what is this used for?  Never set, so always 0
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
  m_pos       = PIXY_RCS_CENTER_POS;
  m_pgain     = pgain;
  m_dgain     = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
}



void setup()
{
  pixy.init();
  DEBUG_PORT.begin(115200);
  gpsPort.begin( GPS_BAUD );

  Wire.begin( MEGA_I2C_ADDR );
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  DEBUG_PORT.println( F("TEST2") );
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    DEBUG_PORT.println( F("Ooops, no LSM303 detected ... Check your wiring!") );
    while(1);
  }

  pinMode(BLUE_LED,OUTPUT);
  pinMode(ORANGE_LED,OUTPUT);
  pinMode(I2C_REQUEST,OUTPUT);
  pinMode(I2C_RECEIVE,OUTPUT);
  pinMode(USING_GPS_HEADING,OUTPUT);
  pinMode(51,OUTPUT);
  pinMode(53,OUTPUT);
  digitalWrite(USING_GPS_HEADING,HIGH);
  digitalWrite(PIXY_PROCESSING,HIGH);
  digitalWrite(53,LOW);

  beep( 1000 );

  delay(1000);
  digitalWrite(PIXY_PROCESSING,LOW);
  digitalWrite(USING_GPS_HEADING,LOW);

} // setup

void loop()
{
  checkBeep();

  while (gps.available( gpsPort ))
  {
    gps_fix fix = gps.read(); // save the latest

    if (fix.valid.location)
    {
      beep(100);
      digitalWrite(ORANGE_LED,HIGH);

      float range = fix.location.DistanceKm( base );
      zheading = fix.heading();


      //float heading = prevFix.location.BearingToDegrees ( currentFix.location );
      //float bob = fix.location();
      //zheading = fix.location.BearingToDegrees ( currentFix.location );
      //prevFix = fix;


      DEBUG_PORT.print( F("Zheading:             ") );DEBUG_PORT.println(zheading,2);
      //DEBUG_PORT.print( F("Range:             ") );DEBUG_PORT.println(range,9);
      zdistance = range * MM_PER_KM;
      DEBUG_PORT.print( F("Distance mm:             ") );DEBUG_PORT.println(zdistance);
      ubloxBearing = fix.location.BearingToDegrees( base );
      bearing = ubloxBearing;
      zbearing = ubloxBearing * 100; //Float to integer. zbearing is sent to TC275.
      DEBUG_PORT.print( F("Bearing:         ") );DEBUG_PORT.println(ubloxBearing,5);
      DEBUG_PORT.print( F("Compass Heading: ") );DEBUG_PORT.println(compass);
      DEBUG_PORT.println();
    }
    //else     // If there's no valid fix, machine will drive in straight line until fix recieved or pixie overides it.
    //{
    //  compass = ubloxBearing;
    //}
      // Waiting...
      //DEBUG_PORT.print( '.' );
      zz = (fix.location.lat()); // zz is a 'long integer'.
      yy = (fix.location.lon());
      //DEBUG_PORT.print( F("Current Ublox latitude:  ") );DEBUG_PORT.println(zz);
      //DEBUG_PORT.print( F("Latitude from Fona:      ") );DEBUG_PORT.println(latitude);
      //DEBUG_PORT.print( F("Current Ublox longitude: ") );DEBUG_PORT.println(yy);
      //DEBUG_PORT.print( F("Longitude from Fona:     ") );DEBUG_PORT.println(longitude);
      //DEBUG_PORT.println();

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

  digitalWrite( ORANGE_LED, LOW );
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
    digitalWrite(PIXY_PROCESSING,HIGH);
    panError  = X_CENTER - pixy.blocks[0].x;
    panLoop.update(panError);

    tiltError = pixy.blocks[0].y - Y_CENTER;
    tiltLoop.update(tiltError);

    blockCount = blocks;
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      //sprintf(buf, "Detected %d:\n", blocks);
      DEBUG_PORT.println(buf); // Empties serial buffer.
      for (j=0; j<blocks; j++)
      {
        long size = pixy.blocks[j].height * pixy.blocks[j].width;
        DEBUG_PORT.print( F("No. of blocks: ") );DEBUG_PORT.println(blocks);
        DEBUG_PORT.print( F("Block no.:     ") );DEBUG_PORT.println(j+1);
        DEBUG_PORT.print( F("Size:          ") );DEBUG_PORT.println(size);
        DEBUG_PORT.print( F("Max. size:     ") );DEBUG_PORT.println(maxSize);
        DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
        DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);
        //sprintf(buf, "  block %d: ", j);
        //DEBUG_PORT.print(buf);
        pixy.blocks[j].print();
        DEBUG_PORT.println();
      }
    }
  // Overide compass module with object recognition:
    if(panError > 300)
    {
    DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
    DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);
    //compassModule();
    }

    compass = bearing + panError*0.2;

  }// if (blocks) end
  else
  {
    digitalWrite(PIXY_PROCESSING,LOW);
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
        //DEBUG_PORT.print( F("newSize:      ") );DEBUG_PORT.println(newSize);
        trackedBlock = k;
        maxSize = newSize;
        //DEBUG_PORT.print( F("maxSize:      ") );DEBUG_PORT.println(maxSize);
      }
    }
  }
////////////////////////////////////////////////////////////////////////////
} // Main loop end
void blueLED()
{
  digitalWrite(BLUE_LED, ledBlueState);
  unsigned long currentMillis = millis();
  millisCalc1 = currentMillis - previousMillis1;
  if (millisCalc1 >= BLUE_LED_BLINK_PERIOD)
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
  //DEBUG_PORT.println( F("Request event start  ") );
  Wire.write(url);     // as expected by master
  digitalWrite(I2C_REQUEST,HIGH);
  delay(100);
  digitalWrite(I2C_REQUEST,LOW);
  //DEBUG_PORT.println( F("Request event end  ") );
}


void receiveEvent(int howMany) // Recieves lat and long data from FONA via TC275 for calculating bearings and distances.
{
  //digitalWrite(I2C_RECEIVE,HIGH);
    a="";
    lat="";
    lon="";
    latString="";
    lonString="";
  delay(100);
  //DEBUG_PORT.println( F("Recieve event start  ") );
  //DEBUG_PORT.println( F("Here is data from TC275: ") );
  while (Wire.available())
  {
    char c = Wire.read(); // receive byte as a character
   // DEBUG_PORT.print(c); //DEBUG_PORT.print( 'Q' );          // print the character
      if (isAlpha(c))         // analyse c for letters
      {
      //DEBUG_PORT.print( F("LETTER") );
      a=a+c;                // string = string + character
      if (a=="LAT")
      {
       lon="";
       lat=a;
       //DEBUG_PORT.print( F(" Trigger word LAT detected!: ") );//DEBUG_PORT.println(b);
       a="";
      }
      if (a=="LONG")
      {
       lat="";
       lon=a;
       //DEBUG_PORT.print( F(" Trigger word LONG detected!: ") );//DEBUG_PORT.println(d);
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

  long result = latString.toInt();
  if(result!=0) { latitude = result; }

  result = lonString.toInt();
  if(result!=0) { longitude = -result; } // <--  Why negative?  Sent wrong?

  if ((latitude != 0) and (longitude != 0)) {
    // Received a new base location  
    base.lat( latitude );
    base.lon( longitude );
  }

  //DEBUG_PORT.print( F("latString: ") );DEBUG_PORT.println(latString);
  //DEBUG_PORT.print( F("lonString: ") );DEBUG_PORT.println(lonString);
  //DEBUG_PORT.print( F("latitude integer from Fona:  ") );DEBUG_PORT.println(latitude);
  //DEBUG_PORT.print( F("longitude integer from Fona: ") );DEBUG_PORT.println(longitude);
  //DEBUG_PORT.println();
  //DEBUG_PORT.println( F("Recieve event end  ") );
  //digitalWrite(I2C_RECEIVE,LOW);
}

void characterCompileA() // For sending Ublox bearing and distance data to TC275
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is where the data is compiled into a character ....
  dataString =  initiator  + "BEAR" + zbearing + "DIST" + zdistance + "COMP" + compass;  // Limited to 32 characters for some reason !!! ..... + ",LON" + yy .... Removed.
  int n = dataString.length();
  //DEBUG_PORT.print( F("Data string to send:     ") ); DEBUG_PORT.println(dataString);
  //DEBUG_PORT.print( F("Size of string:  ") ); DEBUG_PORT.println(n);
  // Builds the url character:
      for (int aa=0;aa<=n;aa++)
      {
          url[aa] = dataString[aa];
      }
   //DEBUG_PORT.print( F("Character data to send:  ") ); DEBUG_PORT.println(url);
  // DEBUG_PORT.println();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void characterCompileB() // For sending Ublox lat and long to TC275
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is where the data is compiled into a character ....
  dataString =  initiator  + "LOON" + yy + "LAAT" + zz;  // Limited to 32 characters for some reason !!! ..... + ",LON" + yy .... Removed.
  int n = dataString.length();
  // DEBUG_PORT.print( F("Data string to send:     ") );// DEBUG_PORT.println(dataString);
   //DEBUG_PORT.print( F("Size of string:  ") );// DEBUG_PORT.println(n);
  // Builds the url character:
      for (int aa=0;aa<=n;aa++)
      {
          url[aa] = dataString[aa];
      }
   //DEBUG_PORT.print( F("Character data to send to TC275:  ") ); DEBUG_PORT.println(url);
   //DEBUG_PORT.println();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
void compassModule()
{
  //DEBUG_PORT.println( F("####################### 1") );
  sensors_event_t magEvent;
  //DEBUG_PORT.println( F("####################### 1.5") );
  delay(100);
  mag.getEvent(&magEvent);  // This is where the problem is!!!!!!!!!!!!!!!!
  //DEBUG_PORT.println( F("####################### 2") );

  // Observed readings (integers)
  int xMin =  -19.45;
  int xMax =  23.27;
  int yMin = -49.82;
  int yMax =  -9.82;

  //DEBUG_PORT.println( F("####################### 3") );
  float xxBlob = magEvent.magnetic.x;
  float yyBlob = magEvent.magnetic.y;
  //DEBUG_PORT.println( F("####################### 4") );
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
  //DEBUG_PORT.println( F("####################### 5") );
  float xxx = ((magEvent.magnetic.x - xMin)*100/(xMax-xMin))-50;
  float yyy = ((magEvent.magnetic.y - yMin)*100/(yMax-yMin))-50;
  //DEBUG_PORT.println( F("####################### 6") );
  compass = atan2( yyy, xxx ) * RAD_TO_DEG;
  compass = compass + 270 +15; //Lower this value to make clockwise turn.

  if (zheading!=0)
  {
    compass = zheading; // Overide compass with GPS derived heading
    digitalWrite(USING_GPS_HEADING,HIGH);
  }
  else                  // Makes the machine drive straight ahead.
  {
    compass = zbearing/100;
    digitalWrite(USING_GPS_HEADING,LOW);
  }

  DEBUG_PORT.print( F("autoXMin:  ") ); DEBUG_PORT.println(autoXMin);
  DEBUG_PORT.print( F("autoXMax:  ") ); DEBUG_PORT.println(autoXMax);
  DEBUG_PORT.print( F("autoYMin:  ") ); DEBUG_PORT.println(autoYMin);
  DEBUG_PORT.print( F("autoYMax:  ") ); DEBUG_PORT.println(autoYMax);
  //DEBUG_PORT.print( F("Compass Heading: ") );DEBUG_PORT.println(compass);
  //DEBUG_PORT.println();
}

uint32_t beepDuration;
uint32_t beepStart;
bool     beeping;

void beep( uint32_t duration )
{
  beeping      = true;
  beepStart    = millis();
  beepDuration = duration;
  tone( SPEAKER, BEEP_FREQUENCY, 0 );   // pin,pitch,duration (forever)
}

void checkBeep()
{
  if (beeping and ((millis() - beepStart) >= beepDuration)) {
    noTone( SPEAKER );
    beeping = false;
  }
}
