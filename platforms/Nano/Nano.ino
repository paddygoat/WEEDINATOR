#define DEBUG_PORT Serial

#include <Wire.h>

#include <Adafruit_FONA.h>

// For testing on a Mega, just use Serial (type the AT command responses)
#if defined(ARDUINO_AVR_MEGA2560)
  #define fonaPort Serial
  const uint32_t FONA_BAUD = 115200;

// For a real Nano, use SoftwareSerial to talk to a real Fona module
#elif defined(ARDUINO_AVR_NANO)
  #include <SoftwareSerial.h>
  const int FONA_RX = 3;
  const int FONA_TX = 4;
  SoftwareSerial fonaPort( FONA_TX, FONA_RX );
  const uint32_t FONA_BAUD = 4800;

#else
  #error Board not supported!
#endif

const int FONA_RST     = 5;
const int SPEAKER      = 6;
const int ORANGE_LED   = 11;
const int BLUE_LED     = 12;
const int LED_PIN      = 13;
const int I2C_ACTIVITY = LED_PIN;

const int      HEARTBEAT_LED    = ORANGE_LED;
const uint32_t HEARTBEAT_PERIOD =  200; // ms
const uint16_t BEEP_FREQUENCY   =  750; // Hz

// Connect Nano A5 to Maga SCL
// Connect Nano A4 to Mega SDA

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

#include <NeoPrintToRAM.h>


char rabbits[40] = "";


volatile int phpPage         = 0;
         int previousPhpPage = 0;


Adafruit_FONA fona( FONA_RST );


///////////////////////////////////////////////////////////////////////

void setup()
{
  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println( F("FONA basic test\r\n"
                        "Initialising....(May take 3 seconds)") );

  Wire.begin(43);                // join i2c bus with address #43
  Wire.onReceive( receiveEvent ); // register event
  Wire.onRequest( requestEvent ); // register event

  pinMode( BLUE_LED, OUTPUT );
  digitalWrite( BLUE_LED, LOW );
  pinMode( ORANGE_LED, OUTPUT );
  digitalWrite( ORANGE_LED, LOW );
  pinMode( I2C_ACTIVITY, OUTPUT );
  digitalWrite( I2C_ACTIVITY, LOW );

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

  // Turn off GPRS:
  if (!fona.enableGPRS(false))
    DEBUG_PORT.println( F("FAILED: GPRS not turned off") );
  else
    DEBUG_PORT.println( F("GPRS turned off") );

  //delay (1000);
  //networkStatus();   // Check the network is available. Home is good.

  turnOnGPRS();
  turnOnGPRS();  // <-- why twice?

  beep( 500 );

  DEBUG_PORT.println( F("Let's now do some work with the database  ...........") );

} // setup

///////////////////////////////////////////////////////////////////////

void loop()
{
  heartbeat();
  checkBeep();
  checkPHP ();
}

///////////////////////////////////////////////////////////////////////

      uint32_t lastPHP              = 0;
const uint32_t MIN_PHP_CHECK_PERIOD = 1000;

void checkPHP()
{
  // Don't try to get waypoints too quickly
  if (millis() - lastPHP >= MIN_PHP_CHECK_PERIOD) {

    // Was a new waypoint requested?
    if (previousPhpPage != phpPage) {

      // Yes, get it now.
      receiveData();
      lastPHP         = millis();
      previousPhpPage = phpPage;
    }
  }

} // checkPHP

///////////////////////////////////////////////////////////////////////

void receiveEvent(int howMany)
{
  while (Wire.available()) {
    char c = Wire.read();
    if (isdigit(c)) {
      phpPage = c - '0'; // E.g., char '5' to integer value 5
    }
  }
} // receiveEvent

///////////////////////////////////////////////////////////////////////

void requestEvent()
{
  digitalWrite(I2C_ACTIVITY, HIGH);

  Wire.write( rabbits, strlen( rabbits ) + 1 ); // includes NUL terminator

  digitalWrite(I2C_ACTIVITY, LOW);

} // requestEvent

///////////////////////////////////////////////////////////////////////

void receiveData()
{
  DEBUG_PORT.print( F("Select php page from TC275:        ") );
  DEBUG_PORT.println( phpPage );

  // Builds the url character array:
  char url[60];
  Neo::PrintToRAM urlChars( url, sizeof(url) );
  urlChars.print( CF(webAddress) );
  urlChars.print( phpPage );
  urlChars.print( CF(dotPhp) );
  urlChars.terminate(); // add NUL terminator to this C string

  //urlChars.print( F("bollox") );
  //urlChars.print( phpPage );
  //urlChars.print( F(".php") );
  //urlChars.terminate();

  DEBUG_PORT.print( F("url for GET request:       ") );
  showData( url, strlen(url) );
  DEBUG_PORT.println();

  digitalWrite(BLUE_LED, HIGH);

  // Issue GET request.  Reply will be a waypoint.
  uint16_t statuscode;
  uint16_t length;

  if (fona.HTTP_GET_start( url, &statuscode, &length )) {

    char receive[40];
    Neo::PrintToRAM receiveChars( receive, sizeof(receive) );

    // This is blocking, because the complete reply has not arrived yet.
    while (length > 0) {
      if (fona.available()) {
        char c = fona.read();
        if (isprint( c ))
          receiveChars.write( c ); // add to array
        showData( &c, 1 );

        length--;
      }
    }
    receiveChars.terminate();

    DEBUG_PORT.print( F("Lat and Lon from database (receive):     ") );
    showData( receive, strlen(receive) );
    DEBUG_PORT.println();

    // Builds the message to send with lat and long coordinates from database.
    length = receiveChars.numWritten();
    if (length > 0) {
      // skip the first character?
      memcpy( rabbits, &receive[1], length-1 ); // Include the NUL terminator
    } else {
      // Empty message
      rabbits[0] = '\0';
    }
    
    DEBUG_PORT.print( F("Character data to be requested by TC275 (rabbits):  ") );
    showData( rabbits, strlen(rabbits)+1 );
    DEBUG_PORT.println();

  } else {
     DEBUG_PORT.println( F("HTTP GET Failed!") );
  }
 
  DEBUG_PORT.println( F("\n****") );
  fona.HTTP_GET_end();

  digitalWrite(BLUE_LED, LOW);

  beep(1000);        //pin,pitch,duration
}

///////////////////////////////////////////////////////////////////////

void turnOnGPRS()
{
  DEBUG_PORT.println( F("Now attempting to turn on GPRS .........") );

  if (fona.enableGPRS(true)) {
    digitalWrite(ORANGE_LED, HIGH);
    //DEBUG_PORT.println( F("Wait for 10 seconds to make sure GPRS is on ..........." ));
    //delay (10000);
  } else {
    //DEBUG_PORT.println(("No - Failed to turn on"));
  }

} // turnOnGPRS

////////////////////////////////////////////////////////////////////////////

uint32_t lastHeartbeat;

void heartbeat()
{
  uint32_t currentMillis = millis();

  if ((currentMillis - lastHeartbeat) >= HEARTBEAT_PERIOD)
  {
    lastHeartbeat = currentMillis;
    digitalWrite( HEARTBEAT_LED, !digitalRead(HEARTBEAT_LED) ); // toggle
  }
} // heartbeat

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
