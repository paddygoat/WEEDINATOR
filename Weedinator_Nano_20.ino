#include <Wire.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#define FONA_RX 3
#define FONA_TX 4
#define FONA_RST 5
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
// Connect Nano A5 to Maga SCL
// Connect Nano A4 to Mega SDA

// this is a large buffer for replies
char replybuffer[255];
String initiator;
String dataString = "#################################"; //Assign 33 bytes.
char url[60] = "############################################################";
const char webAddress[55] = "http://www.goatindustries.co.uk/weedinator/select";
const char dotPhp[5] = ".php";
int stringLength =0;
int w=10;
int n=0;
int p=0;
int i=0;
char rabbits[40];
String a="";
int k;
int z=0;
String recieve = "#################################"; //Assign 33 bytes.
String cString = "1";
int phpPage =0;
int previousPhpPage =0;
int ledState;
int ledPin=13;
char c;

//Connect Rx on Fona to Tx1 on mega or due:
//HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{ 
  tone(6,750,500);
  delay(500);
  noTone(6);
  Wire.begin(43);                // join i2c bus with address #43
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(12, OUTPUT);         // Blue LED
  digitalWrite(12, LOW);
  pinMode(11, OUTPUT);         // Orange LED
  digitalWrite(11, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (!Serial);       // Use this for initiating program by opening of serial monitor. Delete in normal operation.
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initialising....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
//   while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
  }
  //networkStatus();   // Check the network is available. Home is good.
  
        Serial.println("");
        Serial.println("Checking that GPRS is turned off to start with .........");
  fona.setGPRSNetworkSettings(F("pp.vodafone.co.uk"), F("wap"), F("wap"));   // Change these settings! (Network APN, username ,password)
//delay (1000);
        // Turn off GPRS:
        if (!fona.enableGPRS(false))
        Serial.println(F("No - Failed to turn off"));
        Serial.println("If the line above says OK, then GPRS has just been turned off");
//delay (1000);
    //networkStatus();   // Check the network is available. Home is good.
    turnOnGPRS();
    turnOnGPRS();
    delay (10000);  // Allow time for TC275 to transmit to this NANO before going to loop.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   Serial.println("Let's now do some work with the database  ...........");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() // Do not use millis commands - not enough memory! Need at least 77% dynamic memory.
{
  cString = c;
  phpPage = (cString).toInt();
  if(phpPage!=previousPhpPage)   // New waypoint required by TC275
  {
    recieveData();
    delay(100);
    previousPhpPage = phpPage;
  }
  digitalWrite(11, HIGH);
  delay(100);
  digitalWrite(11, LOW);
  delay(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
// End of main loop.
void receiveEvent(int howMany) // Recieves data about which /select.php file to read next
{
  while (Wire.available())
  {
    c = Wire.read(); // receive byte as a character, eg 7
    Serial.print("c:                ");Serial.print(c);Serial.println(" (recieved from TC275)");
  }
}
void requestEvent() 
{ 
  characterCompile();
  Serial.println("Request event start  ");
  //Wire.write(rabbits[1]);
  Wire.write(rabbits[2]);
  Wire.write(rabbits[3]); // as expected by master
  Wire.write(rabbits[4]);
  Wire.write(rabbits[5]);
  Wire.write(rabbits[6]);
  Wire.write(rabbits[7]);
  Wire.write(rabbits[8]);
  Wire.write(rabbits[9]);
  Wire.write(rabbits[10]);
  Wire.write(rabbits[11]);
  Wire.write(rabbits[12]);  
  Wire.write(rabbits[13]);
  Wire.write(rabbits[14]);
  Wire.write(rabbits[15]);
  Wire.write(rabbits[16]);
  Wire.write(rabbits[17]);
  Wire.write(rabbits[18]);
  Wire.write(rabbits[19]);
  Wire.write(rabbits[20]);
  Wire.write(rabbits[21]); 
  Wire.write(rabbits[22]);
  Wire.write(rabbits[23]); 
  Wire.write(rabbits[24]);
  Wire.write(rabbits[25]); 
  Wire.write(rabbits[26]);
  Wire.write(rabbits[27]);
  Serial.println("Request event end  ");
  //    for (int aa=2;aa<=n;aa++)   // For some reason this does not work.                                           
  //    {
  //        Wire.write(rabbits);
  //    }
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
}
void characterCompile()
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        BME280Temperature
// This is where the data is compiled into a character ....
  Serial.print("Lat and Lon recieved from database (recieve):     ");Serial.println(recieve);  
  //dataString =  initiator  + recieve;  // Limited to 32 characters for some reason !!!
  int n =  31;                                                      // dataString.length();
  //Serial.print("Data string to compile into char[] rabbits:     ");Serial.println(dataString);   
  //Serial.print("Size of string:  ");Serial.println(n);
  // Builds the rabbits character, which is lat and long coordinates from database.
      for (int aa=0;aa<=n;aa++)                                              
      {
        rabbits[aa] = recieve[aa];  // char = string
      }
  Serial.print("Character data to be requested by TC275 (rabbits):  ");Serial.println(rabbits);
  Serial.println("");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
}
void recieveData()
{
////////////////////////////////////////////////////////////////////////////////////////
  Serial.print("Select php url from TC275:        ");Serial.println(cString);
  dataString = "";
  dataString =  initiator + webAddress + cString + dotPhp;
  //dataString =  "bollox" + cString + ".php";
  Serial.print("url string compiled (dataString):   ");Serial.println(dataString);
  // Builds the url character:
  n = dataString.length()-1;
  for (int aa=0;aa<=n;aa++)                                              
  {
    url[aa]="Z";
  }
  p = dataString.length()-1;
  for (int bb=0;bb<=p;bb++)                                              
  {
    url[bb] = dataString[bb];     // Character = String
  }
  Serial.print("url character compiled (url):       ");Serial.println(url);
//////////////////////////////////////////////////////////////////////////////////////// 
  recieve="";
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(100);
  // read website URL
  uint16_t statuscode;
  int16_t length;
  //char url[80] = "http://www.goatindustries.co.uk/weedinator/select7.php";
  if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) 
  {
     Serial.println("Failed!");
     //break;
  }
  while (length > 0) 
  {
    while (fona.available()) 
    {
      char ccc = fona.read();
      recieve=recieve+ccc;                      // string = string + character 
      // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = ccc; 
      #else
        Serial.write(ccc);
      #endif
        length--;
         if (! length);
     }
   }
   Serial.println(F("\n****"));
   fona.HTTP_GET_end();
   tone(6,750,1000);        //pin,pitch,duration
   delay(1000);
   noTone(6);
}
//////////////////////////////////////////////////////////////////////////////////
void turnOnGPRS()
{    
        //delay (10000);
        Serial.println("Now attempting to turn on GPRS .........");
     //   if (!fona.enableGPRS(true))
     //   Serial.println(F("No - Failed to turn on"));
     //   Serial.println("GPRS is on if the line above shows 'OK'");
     //   Serial.println("Wait for 10 seconds to make sure GPRS is on ...........");        
     //   delay (10000);
        if (fona.enableGPRS(true))   
        digitalWrite(11, HIGH);                  // Orange LED

    //    Serial.println(("No - Failed to turn on"));
}
