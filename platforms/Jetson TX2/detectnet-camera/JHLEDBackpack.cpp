//#include <JHLEDBackpack.h>

static const uint8_t numbertable[] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* a */
    0x7C, /* b */
    0x39, /* C */
    0x5E, /* d */
    0x79, /* E */
    0x71, /* F */
};


HT16K33::HT16K33(int address) {
   kI2CBus = 1 ;           // Default Gen2 I2C bus for Jetson TK1, J3A1:18 & J3A1:20
   kI2CAddress = address ; // Defaults to 0x70 for HT16K33 ; jumper settable
   error = 0 ;
   position = 0 ;
}

HT16K33::~HT16K33() {
   closeHT16K33() ;
}

bool HT16K33::openHT16K33()
{
   char fileNameBuffer[32];
   sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
   kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
   if (kI2CFileDescriptor < 0) {
       // Could not open the file
      error = errno ;
      return false ;
   }
   if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
       // Could not open the device on the bus
       error = errno ;
       return false ;
   }
   return true ;
}

void HT16K33::closeHT16K33()
{
   if (kI2CFileDescriptor > 0) {
       close(kI2CFileDescriptor);
       // WARNING - This is not quite right, need to check for error first
       kI2CFileDescriptor = -1 ;
   }
}


int HT16K33::i2cwrite(int writeValue) {
    // printf("Wrote: 0x%02X  \n",writeValue) ;
    int toReturn = i2c_smbus_write_byte(kI2CFileDescriptor, writeValue);
    if (toReturn < 0) {
        printf("HT16K33 Write error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;

}

void HT16K33::setBrightness(uint8_t b) {
  if (b > 15) b = 15;
  i2cwrite(HT16K33_CMD_BRIGHTNESS | b);
}

void HT16K33::blinkRate(uint8_t b) {
   if (b > 3) b = 0; // turn off if not sure

  i2cwrite(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
}


void HT16K33::begin() {
    i2cwrite(0x21);  // turn on oscillator
    blinkRate(HT16K33_BLINK_OFF);

    setBrightness(15); // max brightness
   }

void HT16K33::end() {
    i2cwrite(0x20) ;  // turn off the oscillator
}

int HT16K33::writeDisplay(void) {
  int toReturn = i2c_smbus_write_i2c_block_data(kI2CFileDescriptor,0x00,10,(__u8*)displayBuffer) ;
  if (toReturn < 0) {
      printf("HT16K33 Write error: %d",errno) ;
      error = errno ;
      toReturn = -1 ;
  }
  return toReturn ;

}

void HT16K33::clear(void) {
  for (uint8_t i=0; i< sizeof(displayBuffer); i++) {
    displayBuffer[i] = 0;
  }
}



void HT16K33::print(unsigned long n, int base)
{
  if (base == 0) write(n);
  else printNumber(n, base);
}

void HT16K33::print(char c, int base)
{
  print((long) c, base);
}

void HT16K33::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void HT16K33::print(int n, int base)
{
  print((long) n, base);
}

void HT16K33::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void  HT16K33::println(void) {
  position = 0;
}

void  HT16K33::println(char c, int base)
{
  print(c, base);
  println();
}

void  HT16K33::println(unsigned char b, int base)
{
  print(b, base);
  println();
}

void  HT16K33::println(int n, int base)
{
  print(n, base);
  println();
}

void  HT16K33::println(unsigned int n, int base)
{
  print(n, base);
  println();
}

void  HT16K33::println(long n, int base)
{
  print(n, base);
  println();
}

void  HT16K33::println(unsigned long n, int base)
{
  print(n, base);
  println();
}

void  HT16K33::println(double n, int digits)
{
  print(n, digits);
  println();
}

void  HT16K33::print(double n, int digits)
{
  printFloat(n, digits);
}


size_t HT16K33::write(uint8_t c) {

  uint8_t r = 0;

  if (c == '\n') position = 0;
  if (c == '\r') position = 0;

  if ((c >= '0') && (c <= '9')) {
    writeDigitNum(position, c-'0');
    r = 1;
  }

  position++;
  if (position == 2) position++;

  return r;
}

void HT16K33::writeDigitRaw(uint8_t d, uint8_t bitmask) {
  if (d > 4) return;
  displayBuffer[d] = bitmask;
}

void HT16K33::drawColon(bool state) {
  if (state)
    displayBuffer[2] = 0x2;
  else
    displayBuffer[2] = 0;
}

void HT16K33::writeColon(void) {

    i2cwrite((uint8_t)0x04); // start at address $02

    i2cwrite(displayBuffer[2] & 0xFF);
    i2cwrite(displayBuffer[2] >> 8);
}

void HT16K33::writeDigitNum(uint8_t d, uint8_t num, bool dot) {
  if (d > 4) return;

  writeDigitRaw(d, numbertable[num] | (dot << 7));
}

void HT16K33::print(long n, int base)
{
  printNumber(n, base);
}

void HT16K33::printNumber(long n, uint8_t base)
{
    printFloat(n, 0, base);
}

void HT16K33::printFloat(double n, uint8_t fracDigits, uint8_t base)
{
  uint8_t numericDigits = 4;   // available digits on display
  bool isNegative = false;  // true if the number is negative

  // is the number negative?
  if(n < 0) {
    isNegative = true;  // need to draw sign later
    --numericDigits;    // the sign will take up one digit
    n *= -1;            // pretend the number is positive
  }

  // calculate the factor required to shift all fractional digits
  // into the integer part of the number
  double toIntFactor = 1.0;
  for(int i = 0; i < fracDigits; ++i) toIntFactor *= base;

  // create integer containing digits to display by applying
  // shifting factor and rounding adjustment
  uint32_t displayNumber = n * toIntFactor + 0.5;

  // calculate upper bound on displayNumber given
  // available digits on display
  uint32_t tooBig = 1;
  for(int i = 0; i < numericDigits; ++i) tooBig *= base;

  // if displayNumber is too large, try fewer fractional digits
  while(displayNumber >= tooBig) {
    --fracDigits;
    toIntFactor /= base;
    displayNumber = n * toIntFactor + 0.5;
  }

  // did toIntFactor shift the decimal off the display?
  if (toIntFactor < 1) {
    printError();
  } else {
    // otherwise, display the number
    int8_t displayPos = 4;

    if (displayNumber)  //if displayNumber is not 0
    {
      for(uint8_t i = 0; displayNumber || i <= fracDigits; ++i) {
        bool displayDecimal = (fracDigits != 0 && i == fracDigits);
        writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
        if(displayPos == 2) writeDigitRaw(displayPos--, 0x00);
        displayNumber /= base;
      }
    }
    else {
      writeDigitNum(displayPos--, 0, false);
    }

    // display negative sign if negative
    if(isNegative) writeDigitRaw(displayPos--, 0x40);

    // clear remaining display positions
    while(displayPos >= 0) writeDigitRaw(displayPos--, 0x00);
  }
}

void HT16K33::printError(void) {
  for(uint8_t i = 0; i < SEVENSEG_DIGITS; ++i) {
    writeDigitRaw(i, (i == 2 ? 0x00 : 0x40));
  }
}
