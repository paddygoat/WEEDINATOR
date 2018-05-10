#define DEBUG_PORT Serial
const uint32_t DEBUG_BAUD = 115200;

#define gpsPort    Serial1

//#define SIMULATE_DEVICES  // comment this out for real system

#if defined( SIMULATE_DEVICES )

  const uint32_t GPS_BAUD   =  9600;

  const uint32_t FONA_BAUD  = DEBUG_BAUD;
  #define fonaPort   Serial

  static const bool useConsole = true;

#else

  const uint32_t GPS_BAUD   =  19200;

  #define fonaPort   Serial2
  const uint32_t FONA_BAUD  =   1200; // 4800;

  static const bool useConsole = false;

#endif

const int FONA_RST            = 5;
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
void beep( uint32_t duration, uint16_t freq = BEEP_FREQUENCY ); // forward decl

////////////////////////////////////////////////
// Changing one of these flags to false will
//   disable the code for that device.  This is
//   handy for my testing.

static const bool usePixy    = not useConsole;
static const bool useCompass = not useConsole;
static const bool useI2C     = not useConsole;
static const bool useFona    = not useConsole;


#include <Wire.h>

volatile int newWaypointID = 0;
         int waypointID    = 0;
bool newMsgA = false;
bool newMsgB = false;

static void disableInterrupts() { cli(); }
static void enableInterrupts()  { sei(); }


#include <Adafruit_FONA.h>
Adafruit_FONA fona( FONA_RST );


// Change these settings! (Network APN, username ,password)
const char networkAPN[] PROGMEM = "pp.vodafone.co.uk";
const char username  [] PROGMEM = "wap";
const char password  [] PROGMEM = "wap";
const char webAddress[] PROGMEM =
  "http://www.goatindustries.co.uk/weedinator/select";
const char dotPhp    [] PROGMEM = ".php";

// Handy macro for passing PROGMEM char arrays to anything 
//   that expects a FLASH string, like DEBUG_PORT.print or 
//   fona.setGPRSNetworkSettings
#define CF(x) ((const __FlashStringHelper *)x)


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
gps_fix fix, prevFix;

NeoGPS::Location_t waypoint( 532558000L, -43114000L ); // Llangefni

const float MM_PER_M  = 1000.0;
const float M_PER_KM  = 1000.0;
const float MM_PER_KM = MM_PER_M * M_PER_KM;


#include <NeoPrintToRAM.h>

float bearingToWaypoint;  // degrees
float distanceToWaypoint; // mm

float compass; // orientation of the platform

int oldSignature;  //  <-- what is this used for?  Never set, so always 0
long maxSize;
long newSize;


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
  DEBUG_PORT.begin( DEBUG_BAUD );
  DEBUG_PORT.println( F("Mega") );

  gpsPort.begin( GPS_BAUD );

  if (usePixy)
    pixy.init();

  if (useI2C) {
    Wire.begin( MEGA_I2C_ADDR );
    Wire.onReceive(receiveNewWaypoint); // register event
    Wire.onRequest(sendNavData); // register event
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

  initFona();

} // setup


void yield()
{
  heartbeat    ();
  checkBeep    ();
  checkGPS     ();
  checkNavData ();
  checkConsole ();

} // yield


void loop()
{
  yield        ();
  pixyModule   ();
  checkWaypoint();

} // loop

////////////////////////////////////////////////////////////////////////////
//  For testing, accept commands from the "console" to simulate:
//     *  receiving a new waypoint ID from the TC275
//     *  receiving a PHP response with waypoint lat/lon
//     *  displaying current message A/B

//  Some variables to receive a line of characters
size_t         count     = 0;
const size_t   MAX_CHARS = 64;
char           line[ MAX_CHARS ];

void checkConsole()
{
  if (useConsole) {

    if (lineReady()) {
      size_t lineLen = strlen( line );

      if (lineLen > 0) {

        if ((line[0] == 'w') and (lineLen > 1)) {
          // simulate receiving a waypoint id from the TC275
          newWaypointID = atoi( &line[1] );

        } else if ((line[0] == 'p') and (lineLen > 1)) {
          // simulate receiving a response to the GET request
          parseWaypoint( &line[1], lineLen-1 );

        } else if (line[0] == 's') {
          // simulate sending messages to the TC275
          sendNavData();
        
        } else {
          DEBUG_PORT.println( F("Invalid command" ) );
        }
      }
    }
  }
} // checkConsole

bool lineReady()
{
  bool          ready     = false;
  const char    endMarker = '\n';

  while (DEBUG_PORT.available()) {

    char c = DEBUG_PORT.read();

    if (c != endMarker) {

      // Only save the printable characters, if there's room
      if (isprint(c) and (count < MAX_CHARS-1)) {
        line[ count++ ] = c;
      }

    } else {
      //  It's the end marker, line is completely received
      line[count] = '\0'; // terminate the string
      count       = 0;    // reset for next time
      ready       = true;
      break;
    }
  }

  return ready;

} // lineReady

////////////////////////////////////////////////////////////////////////////

void checkGPS()
{
  if (gps.available( gpsPort ))
  {
    prevFix = fix;
    fix     = gps.read(); // get the latest

    digitalWrite( ORANGE_LED, fix.valid.location );

    if (fix.valid.location) {
      beep(100);

      float range = fix.location.DistanceKm( waypoint );
      DEBUG_PORT.print( F("Distance km:             ") );
      DEBUG_PORT.println( range );

      distanceToWaypoint = range * MM_PER_KM;
      DEBUG_PORT.print( F("Distance mm:             ") );
      DEBUG_PORT.println( distanceToWaypoint );

      bearingToWaypoint = fix.location.BearingToDegrees( waypoint );
      DEBUG_PORT.print( F("Bearing:         ") );
      DEBUG_PORT.println( bearingToWaypoint );
      DEBUG_PORT.println();

      //DEBUG_PORT.print( F("Current GPS latitude:  ") );
      //DEBUG_PORT.println( fix.location.lat() );
      //DEBUG_PORT.print( F("Waypoint Latitude from Fona:      ") );
      //DEBUG_PORT.println( waypoint.lat() );
      //DEBUG_PORT.print( F("Current GPS longitude: ") );
      //DEBUG_PORT.println( fix.location.lon() );
      //DEBUG_PORT.print( F("Waypoint Longitude from Fona:     ") );
      //DEBUG_PORT.println( waypoint.lon() );
      //DEBUG_PORT.println();

      //readCompass();

      if (prevFix.valid.location) {
        // calculate heading from the current and previous locations
        float heading = prevFix.location.BearingToDegrees( fix.location );
        fix.hdg.whole = (int) heading;
        fix.hdg.frac  = (heading - (float) fix.hdg.whole) * 100.0;
        fix.valid.heading = true;
      }

      updateNavData();
      newMsgA = newMsgB = true;

    } else {
      DEBUG_PORT.write( '.' );
    }

    digitalWrite( USING_GPS_HEADING, fix.valid.heading );

    if (fix.valid.heading) {
      DEBUG_PORT.print( F("heading:              ") );
      DEBUG_PORT.println( fix.heading() );
    }
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
      //readCompass();
    }

    compass = bearingToWaypoint + panError*0.2; // <-- OK???

  } else {
    // No blocks
    digitalWrite(PIXY_PROCESSING,LOW);
  }
  //readCompass();

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

      uint32_t lastPHP              = 0;
const uint32_t MIN_PHP_CHECK_PERIOD = 1000;

void checkWaypoint()
{
  // Don't try to get waypoints too quickly
  if (millis() - lastPHP >= MIN_PHP_CHECK_PERIOD) {

    // Was a new waypoint requested?
    disableInterrupts();
      int safeID = newWaypointID;
    enableInterrupts();

    if (waypointID != safeID) {

      waypointID = safeID;
      DEBUG_PORT.print( F("New Waypoint ID ") );
      DEBUG_PORT.println( waypointID );

      // Yes, get it now.
      getWaypoint();
      lastPHP = millis();
    }
  }

} // checkWaypoint

///////////////////////////////////////////////////////////////////////

void getWaypoint()
{
  DEBUG_PORT.print( F("Select php page from TC275:        ") );
  DEBUG_PORT.println( waypointID );

  // Builds the url character array:
  char url[60];
  Neo::PrintToRAM urlChars( url, sizeof(url) );
  urlChars.print( CF(webAddress) );
  urlChars.print( waypointID );
  urlChars.print( CF(dotPhp) );
  urlChars.terminate(); // add NUL terminator to this C string

  //urlChars.print( F("bollox") );
  //urlChars.print( waypointID );
  //urlChars.print( F(".php") );
  //urlChars.terminate();

  DEBUG_PORT.print( F("url for GET request:       ") );
  showData( url, strlen(url) );
  DEBUG_PORT.println();

  if (not useFona)
    return;

  digitalWrite(BLUE_LED, HIGH);

  // Issue GET request.  Reply will be a waypoint ID.
  uint16_t statuscode;
  uint16_t length;

  if (fona.HTTP_GET_start( url, &statuscode, &length )) {

    char receive[40];
    Neo::PrintToRAM receiveChars( receive, sizeof(receive) );

    // This is blocking, because the complete reply has not arrived yet.
    DEBUG_PORT.print( F("Raw PHP reply: '") );
    while (length > 0) {
      if (fona.available()) {
        char c = fona.read();
        if (isprint( c ))
          receiveChars.write( c ); // add to array
        showData( &c, 1 );

        length--;
      }
      yield();
    }
    receiveChars.terminate();
    DEBUG_PORT.println('\'');

    DEBUG_PORT.print( F("Lat and Lon from database (receive):     ") );
    showData( receive, receiveChars.numWritten() );
    DEBUG_PORT.println();

    parseWaypoint( receive, receiveChars.numWritten() );

  } else {
     DEBUG_PORT.println( F("HTTP GET Failed!") );
  }
 
  DEBUG_PORT.println( F("\n****") );
  fona.HTTP_GET_end();

  digitalWrite(BLUE_LED, LOW);

  beep(1000);

} // getWaypoint

////////////////////////////////////////////////////////////////////////////

void parseWaypoint( char *ptr, size_t remaining )
{
  static const char     LAT_LABEL[] PROGMEM = "LAT";
  static const size_t   LAT_LABEL_LEN       = sizeof(LAT_LABEL)-1;
  static const char     LON_LABEL[] PROGMEM = "LONG";
  static const size_t   LON_LABEL_LEN       = sizeof(LON_LABEL)-1;
               uint32_t latValue, lonValue;

  if (remaining > 0) {
    // Skip the first character (what is it?)
    ptr++;
    remaining--;

    if (findText( LAT_LABEL, LAT_LABEL_LEN, ptr, remaining )) {

      if (parseValue( ptr, remaining, latValue )) {

        if (findText( LON_LABEL, LON_LABEL_LEN, ptr, remaining )) {

          if (parseValue( ptr, remaining, lonValue )) {

            // Set the new waypoint location
            waypoint.lat( latValue );
            waypoint.lon( lonValue );

            DEBUG_PORT.print  ( F("Location from Fona:  ") );
            DEBUG_PORT.print  ( waypoint.lat() );
            DEBUG_PORT.print  ( ',' );
            DEBUG_PORT.println( waypoint.lon() );

            //  Make sure we used all the characters
            if (remaining > 0) {
              DEBUG_PORT.print( remaining );
              DEBUG_PORT.print( F(" extra characters after lonValue: '") );
              showData( ptr, remaining );
              DEBUG_PORT.println('\'');
            }

            updateNavData();
            newMsgA = newMsgB = true;

          } else {
            DEBUG_PORT.println( F("Invalid longitude") );
          }
        }

      } else {
        DEBUG_PORT.println( F("Invalid latitude") );
      }
    }
  } else {
    DEBUG_PORT.println( F("response too short") );
  }

} // parseWaypoint

///////////////////////////////////////////////////////////////////////

void turnOnGPRS()
{
  while (true) {
    DEBUG_PORT.println( F("Turning off GPRS ...") );
    if (fona.enableGPRS(false)) {
      DEBUG_PORT.println( F("GPRS turned off.") );
    } else {
      DEBUG_PORT.println( F("FAILED: GPRS not turned off") );
    }

    DEBUG_PORT.println( F("Turning on GPRS ...") );
    if (fona.enableGPRS(true)) {
      DEBUG_PORT.println( F("GPRS turned on.") );
      break;
    } else {
      DEBUG_PORT.println( F("FAILED: GPRS not turned on, retrying"));
    }

    delay( 1000 );
  }

} // turnOnGPRS

////////////////////////////////////////////////////////////////////////////

void initFona()
{
  if (not useFona)
    return;

  fonaPort.begin( FONA_BAUD );
  if (! fona.begin( fonaPort )) {
    DEBUG_PORT.println( F("Couldn't find FONA") );
    //while (1);
  }
  DEBUG_PORT.println( F("FONA is OK") );

  uint8_t type = fona.type();
  const __FlashStringHelper *typeString;
  switch (type) {
    case FONA800L:
      typeString = F("FONA 800L");          break;
    case FONA800H:
      typeString = F("FONA 800H");          break;
    case FONA808_V1:
      typeString = F("FONA 808 (v1)");      break;
    case FONA808_V2:
      typeString = F("FONA 808 (v2)");      break;
    case FONA3G_A:
      typeString = F("FONA 3G (American)"); break;
    case FONA3G_E:
      typeString = F("FONA 3G (European)"); break;
    default: 
      typeString = F("???");                break;
  }
  DEBUG_PORT.print  ( F("Found ") );
  DEBUG_PORT.println( typeString );

  //networkStatus();   // Check the network is available. Home is good.
  DEBUG_PORT.println();

  DEBUG_PORT.println( F("Checking that GPRS is turned off to start with .........") );

  fona.setGPRSNetworkSettings( CF(networkAPN), CF(username), CF(password) );
  //delay (1000);

  //delay (1000);
  //networkStatus();   // Check the network is available. Home is good.

  turnOnGPRS();

} // initFona

////////////////////////////////////////////////////////////////////////////

void showData( char *data, size_t n )
{
  for (size_t i=0; i < n; i++) {

    if (isprint(data[i]))
      DEBUG_PORT.print( data[i] );
    else if (data[i] == 0x00)
      DEBUG_PORT.print( F("<NUL>") );
    else if (data[i] == 0x0A)
      DEBUG_PORT.print( F("<LF>") );
    else if (data[i] == 0x0D)
      DEBUG_PORT.print( F("<CR>") );
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

void receiveNewWaypoint(int howMany)
{
  while (Wire.available()) {
    char c = Wire.read();
    if (isdigit(c)) {
      newWaypointID = c - '0'; // E.g., char '5' to integer value 5
    }
  }
} // receiveNewWaypoint

////////////////////////////////////////////////////////////////////////////

         char navData[120];
volatile bool navDataSent = false;

void sendNavData()
{
  digitalWrite(I2C_REQUEST,HIGH);

  if (useI2C) {
    Wire.write(navData);     // as expected by master
  } else {
    DEBUG_PORT.print( F("TC275 --> ") );
    showData( navData, strlen(navData) );
  }

  navDataSent = true;

  digitalWrite(I2C_REQUEST,LOW);

} // sendNavData

////////////////////////////////////////////////////////////////////////////

void checkNavData()
{
  if (navDataSent) {
    // compile the next message
    nextNavState();
    updateNavData();
    navDataSent = false;
  }
}

enum     navDataState_t { BEARING_DIST_HEAD, LAT_LON, LAST_NAV_STATE };
const    navDataState_t FIRST_NAV_STATE = (navDataState_t) 0;
volatile navDataState_t navDataState;

void nextNavState()
{
  navDataState = (navDataState_t) (navDataState + 1);
  if (navDataState == LAST_NAV_STATE)
    navDataState = FIRST_NAV_STATE;

} // nextNavState

void updateNavData()
{
  switch (navDataState) {
    case BEARING_DIST_HEAD: compileBearDistHeadMsg(); break;
    case LAT_LON          : compileLatLonMsg      (); break;
  }

} // updateNavData

////////////////////////////////////////////////////////////////////////////

void compileBearDistHeadMsg()
{
  Neo::PrintToRAM msg( navData, sizeof(navData) );

  // Prevent sendNavData from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("BEAR") );
    msg.print( (int) (bearingToWaypoint * 100.0) );
    msg.print( F("DIST") );
    msg.print( (long) distanceToWaypoint );
    msg.print( F("HEAD") );
    if (fix.valid.heading)
      msg.print( fix.heading() );
    msg.terminate();
  interrupts();

  if (newMsgA) {
    newMsgA = false;
    DEBUG_PORT.print( F("msg A to send:  ") );
    DEBUG_PORT.println( navData );
  } else {
    DEBUG_PORT.print( 'A' );
  }

} // compileBearDistHeadMsg

////////////////////////////////////////////////////////////////////////////

void compileLatLonMsg()
{
  Neo::PrintToRAM msg( navData, sizeof(navData) );

  // Prevent sendNavData from getting the first half of the old
  //   response and the second half of the new response.
  noInterrupts();
    msg.print( F("LOON") );
    msg.print( fix.location.lon() );
    msg.print( F("LAAT") );
    msg.print( fix.location.lat() );
    msg.terminate();
  interrupts();

  if (newMsgB) {
    newMsgB = false;
    DEBUG_PORT.print( F("msg B to send:  ") );
    DEBUG_PORT.println( navData );
  } else {
    DEBUG_PORT.print( 'B' );
  }

} // compileLatLonMsg

////////////////////////////////////////////////////////////////////////////

float autoXMax = -1000.0;
float autoXMin =  1000.0;
float autoYMax = -1000.0;
float autoYMin =  1000.0;
//unsigned long compassCount = 0;

void readCompass()
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

  DEBUG_PORT.print( F("autoXMin:  ") ); DEBUG_PORT.println(autoXMin);
  DEBUG_PORT.print( F("autoXMax:  ") ); DEBUG_PORT.println(autoXMax);
  DEBUG_PORT.print( F("autoYMin:  ") ); DEBUG_PORT.println(autoYMin);
  DEBUG_PORT.print( F("autoYMax:  ") ); DEBUG_PORT.println(autoYMax);
  DEBUG_PORT.print( F("Compass :  ") ); DEBUG_PORT.println(compass );
  DEBUG_PORT.println();

} // readCompass

////////////////////////////////////////////////////////////////////////////

uint32_t beepDuration;
uint32_t beepStart;
bool     beeping;

void beep( uint32_t duration, uint16_t freq )
{
  beeping      = true;
  beepStart    = millis();
  beepDuration = duration;
  tone( SPEAKER, freq, 0 );   // pin,pitch,duration (forever)
}

void checkBeep()
{
  if (beeping and ((millis() - beepStart) >= beepDuration)) {
    noTone( SPEAKER );
    beeping = false;
  }
}
