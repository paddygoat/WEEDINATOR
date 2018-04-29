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

const uint32_t HEARTBEAT_PERIOD =  200; // ms
const uint16_t BEEP_FREQUENCY   = 2500; // Hz

////////////////////////////////////////////////
// Changing one of these flags to false will
//   disable the code for that device.  This is
//   handy for my testing.

static const bool usePixy    = true;
static const bool useCompass = true;
static const bool useI2C     = true;


#include <Wire.h>
volatile bool   fonaMsgAvailable = false;
volatile bool   fonaMsgLost      = false;
         size_t fonaMsgLen       = 0;
         char   fonaMsg[33];



#include <Pixy.h>
Pixy pixy;
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)


#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag( 12345 );

#include <NMEAGPS.h>
NMEAGPS gps;

NeoGPS::Location_t base( 532558000L, -43114000L ); // Llangefni

const float MM_PER_M  = 1000.0;
const float M_PER_KM  = 1000.0;
const float MM_PER_KM = MM_PER_M * M_PER_KM;


#include <NeoPrintToRAM.h>
char url[120];

long zz;
long yy;
float bearing;
int zbearing;
long zdistance;

int sendDataState = LOW;

double compass;

int oldSignature;  //  <-- what is this used for?  Never set, so always 0
long maxSize;
long newSize;
float zheading;
float ubloxBearing;

////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////

void setup()
{
  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println( F("Mega") );

  gpsPort.begin( GPS_BAUD );

  if (usePixy)
    pixy.init();

  if (useI2C) {
    Wire.begin( MEGA_I2C_ADDR );
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent); // register event
  }

  if (useCompass) {
    mag.enableAutoRange(true);

    if(!mag.begin())
    {
      /* There was a problem detecting the LSM303 ... check your connections */
      DEBUG_PORT.println( F("Ooops, no LSM303 detected ... Check your wiring!") );
      while(1);
    }
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
  heartbeat      ();
  checkBeep      ();
  checkGPS       ();
  pixyModule     ();
  checkForFonaMsg();

} // loop

////////////////////////////////////////////////////////////////////////////

void checkGPS()
{
  if (gps.available( gpsPort ))
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


      DEBUG_PORT.print( F("Zheading:             ") );
      DEBUG_PORT.println(zheading,2);
      
      zdistance = range * MM_PER_KM;
      DEBUG_PORT.print( F("Distance mm:             ") );
      DEBUG_PORT.println(zdistance);

      ubloxBearing = fix.location.BearingToDegrees( base );
      bearing = ubloxBearing;
      zbearing = ubloxBearing * 100; //Float to integer. zbearing is sent to TC275.
      
      DEBUG_PORT.print( F("Bearing:         ") );
      DEBUG_PORT.println(ubloxBearing,5);
      DEBUG_PORT.print( F("Compass Heading: ") );
      DEBUG_PORT.println(compass);
      DEBUG_PORT.println();

      zz = (fix.location.lat()); // zz is a 'long integer'.
      yy = (fix.location.lon());

      //DEBUG_PORT.print( F("Current Ublox latitude:  ") );
      //DEBUG_PORT.println(zz);
      //DEBUG_PORT.print( F("Latitude from Fona:      ") );
      //DEBUG_PORT.println( base.lat() );
      //DEBUG_PORT.print( F("Current Ublox longitude: ") );
      //DEBUG_PORT.println(yy);
      //DEBUG_PORT.print( F("Longitude from Fona:     ") );
      //DEBUG_PORT.println( base.lon() );
      //DEBUG_PORT.println();

      // This switches data set that is transmitted to TC275 via I2C.
      if (sendDataState == LOW)
      {
        characterCompileA(); // Bearing, distance and compass
        sendDataState = HIGH;
      }
      else
      {
        characterCompileB(); // Longitude and latitude
        sendDataState = LOW;
      }

    } else {
      // No valid location, machine will drive in straight 
      //    line until fix received or pixie overrides it.
      digitalWrite( ORANGE_LED, LOW );

      //compass = ubloxBearing;

      // Waiting...
      //DEBUG_PORT.print( '.' );
    }

    compassModule();

  }
} // checkGPS

////////////////////////////////////////////////////////////////////////////

void pixyModule()
{
  if (not usePixy)
    return;

  static       uint8_t frameCount         = 0;
  static const uint8_t PRINT_FRAME_PERIOD = 50;

  static uint16_t blockCount = 0;
         uint16_t blocks     = pixy.getBlocks();

  if (blocks)
  {
    digitalWrite(PIXY_PROCESSING,HIGH);
    int32_t panError  = X_CENTER - pixy.blocks[0].x;
    panLoop.update(panError);

    int32_t tiltError = pixy.blocks[0].y - Y_CENTER;
    tiltLoop.update(tiltError);

    blockCount = blocks;

    // do this (print) every so often frames because printing every
    // frame would bog down the Arduino
    frameCount++;
    if (frameCount >= PRINT_FRAME_PERIOD)
    {
      frameCount = 0;
      //DEBUG_PORT.print( F("Detected ") );
      //DEBUG_PORT.print( blocks );
      //DEBUG_PORT.println( ':' );

      for (uint16_t j=0; j<blocks; j++)
      {
        long size = pixy.blocks[j].height * pixy.blocks[j].width;
        DEBUG_PORT.print( F("No. of blocks: ") );DEBUG_PORT.println(blocks);
        DEBUG_PORT.print( F("Block no.:     ") );DEBUG_PORT.println(j+1);
        DEBUG_PORT.print( F("Size:          ") );DEBUG_PORT.println(size);
        DEBUG_PORT.print( F("Max. size:     ") );DEBUG_PORT.println(maxSize);
        DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
        DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);

        pixy.blocks[j].print();
        DEBUG_PORT.println();
      }
    }

    // Overide compass module with object recognition:
    if (panError > 300)
    {
      DEBUG_PORT.print( F("PAN POS:       ") );DEBUG_PORT.println(panError);
      DEBUG_PORT.print( F("TILT POS:      ") );DEBUG_PORT.println(tiltError);
      //compassModule();
    }

    compass = bearing + panError*0.2;

  } else {
    // No blocks
    digitalWrite(PIXY_PROCESSING,LOW);
  }
  //compassModule();

  int trackedBlock = 0;
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
} // pixyModule

////////////////////////////////////////////////////////////////////////////

void checkForFonaMsg()
{
  if (fonaMsgAvailable) {
    parseFonaMsg();

    // now that the parsing is finished, mark the message as "read"
    fonaMsgAvailable = false; 
  }

  // see if we lost a message sometime
  if (fonaMsgLost) {
    fonaMsgLost = false;
    DEBUG_PORT.println( F("FONA message lost!") );
  }

} // checkForFonaMsg


void parseFonaMsg()
{
  char   *ptr       = &fonaMsg[0]; // point to the beginning
  size_t  remaining = fonaMsgLen;

  static const char     LAT_LABEL[] PROGMEM = "LAT";
  static const size_t   LAT_LABEL_LEN       = sizeof(LAT_LABEL)-1;
  static const char     LON_LABEL[] PROGMEM = "LONG";
  static const size_t   LON_LABEL_LEN       = sizeof(LON_LABEL)-1;
               uint32_t latValue, lonValue;

  if (findText( LAT_LABEL, LAT_LABEL_LEN, ptr, remaining )) {

    if (parseValue( ptr, remaining, latValue )) {

      if (findText( LON_LABEL, LON_LABEL_LEN, ptr, remaining )) {

        if (parseValue( ptr, remaining, lonValue )) {

          // Set the new base location
          base.lat( latValue );
          base.lon( lonValue );

          DEBUG_PORT.print  ( F("Location from Fona:  ") );
          DEBUG_PORT.print  ( base.lat() );
          DEBUG_PORT.print  ( ',' );
          DEBUG_PORT.println( base.lon() );

          //  Make sure we used all the characters
          if (remaining > 0) {
            DEBUG_PORT.print( remaining );
            DEBUG_PORT.print( F(" extra characters after lonValue: '") );
            showData( ptr, remaining );
            DEBUG_PORT.println('\'');
          }

        } else {
          DEBUG_PORT.println( F("Invalid longitude") );
        }
      }

    } else {
      DEBUG_PORT.println( F("Invalid latitude") );
    }
  }

} // parseFonaMsg


void showData( char *data, size_t n )
{
  for (size_t i=0; i < n; i++) {

    if (isprint(data[i]))
      DEBUG_PORT.print( data[i] );
    else if (data[i] == 0x0D)
      DEBUG_PORT.print( F("<CR>") );
    else if (data[i] == 0x0A)
      DEBUG_PORT.print( F("<LF>") );
    else {
      // Some other value?  Just show its HEX value.
      DEBUG_PORT.print( F("<0x") );
      if (data[i] < 0x10)
        DEBUG_PORT.print( '0' );
      DEBUG_PORT.print( data[i], HEX );
      DEBUG_PORT.print( '>' );
    }
  }

} // showData


bool findText
  ( const char *   text_P, const size_t   len,
          char * & ptr   ,       size_t & remaining )
{
  bool found = false;
  
  if (remaining >= len) {

    static const int MATCHED = 0;
  
    if (memcmp_P( ptr, text_P, len ) == MATCHED) {
      found = true;
      
      ptr       += len; // Advance pointer past the label
      remaining -= len;

    } else {
      DEBUG_PORT.print  ( (const __FlashStringHelper *) text_P );
      DEBUG_PORT.println( F(" label not found") );
    }

  } else {
    DEBUG_PORT.print  ( F("Length ") );
    DEBUG_PORT.print  ( remaining );
    DEBUG_PORT.print  ( F(" too short for label ") );
    DEBUG_PORT.println( (const __FlashStringHelper *) text_P );
  }
  
  return found;

} // findText


bool parseValue( char * & ptr, size_t & remaining, uint32_t & value )
{
  bool ok = false;

  if (remaining > 0) {

    bool negative = (*ptr == '-');
    if (negative) {
      ptr++; // advance past minus sign
      remaining--;
    }

    // Interpret digit characters
    value = 0;
    while (remaining and isDigit(*ptr)) {
      value = value * 10 + (*ptr++ - '0'); // advances pointer, too
      remaining--;
      ok = true; // received at least one digit
    }

    if (negative)
      value = -value;
  }

  return ok;

} // parseValue

////////////////////////////////////////////////////////////////////////////

uint32_t lastHeartbeat;

void heartbeat()
{
  uint32_t currentMillis = millis();

  if ((currentMillis - lastHeartbeat) >= HEARTBEAT_PERIOD)
  {
    lastHeartbeat = currentMillis;
    digitalWrite( BLUE_LED, !digitalRead(BLUE_LED) ); // toggle
  }
} // heartbeat

////////////////////////////////////////////////////////////////////////////

void requestEvent()
{
  digitalWrite(I2C_REQUEST,HIGH);
  //DEBUG_PORT.println( F("Request event start  ") );
  Wire.write(url);     // as expected by master
  //delay(100);
  //DEBUG_PORT.println( F("Request event end  ") );
  digitalWrite(I2C_REQUEST,LOW);

} // requestEvent

////////////////////////////////////////////////////////////////////////////

// Receive lat and long data from FONA via TC275 for 
//   calculating bearings and distances.

void receiveEvent(int howMany)
{
  //digitalWrite(I2C_RECEIVE,HIGH);

  if (not fonaMsgAvailable) {

    // Get the whole message
    fonaMsgLen = 0;
    while (Wire.available()) {
      char c = Wire.read();
      // If it's a printable char (not CR/LF) and there's room, save it.
      if ((c >= ' ') and (fonaMsgLen < sizeof(fonaMsg)-1))
        fonaMsg[ fonaMsgLen++ ] = c;
    }
    fonaMsg[ fonaMsgLen ] = '\0'; // NUL-terminate;

    fonaMsgAvailable = true;

  } else {
    // fonaMsgAvailable is still true, loop must not have parsed it yet.
    //   This means that this Fona message must be ignored.

    while (Wire.available())
      Wire.read();
    fonaMsgLost = true;
  }

  //digitalWrite(I2C_RECEIVE,LOW);

} // receiveEvent

////////////////////////////////////////////////////////////////////////////

// For sending Ublox bearing and distance data to TC275

void characterCompileA()
{
  Neo::PrintToRAM msg( url, sizeof(url) );

  // Prevent requestEvent from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("BEAR") );
    msg.print( zbearing );
    msg.print( F("DIST") );
    msg.print( zdistance );
    msg.print( F("COMP") );
    msg.print( compass );
    msg.terminate();
  interrupts();

  //DEBUG_PORT.print( F("msg A to send:  ") );
  //DEBUG_PORT.println( url );

} // characterCompileA

////////////////////////////////////////////////////////////////////////////

// For sending Ublox lat and long to TC275

void characterCompileB()
{
  Neo::PrintToRAM msg( url, sizeof(url) );

  // Prevent requestEvent from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("LOON") );
    msg.print( yy );
    msg.print( F("LAAT") );
    msg.print( zz );
    msg.terminate();
  interrupts();

  //DEBUG_PORT.print( F("msg B to send:  ") );
  //DEBUG_PORT.println( url );

} // characterCompileB

////////////////////////////////////////////////////////////////////////////

float autoXMax = -1000.0;
float autoXMin =  1000.0;
float autoYMax = -1000.0;
float autoYMin =  1000.0;
//unsigned long compassCount = 0;

void compassModule()
{
  if (not useCompass)
    return;

  //compassCount++;

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

} // compassModule

////////////////////////////////////////////////////////////////////////////

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
