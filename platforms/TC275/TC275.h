#ifndef WEEDINATOR_TC275_H
#define WEEDINATOR_TC275_H

#include <stdint.h>

#include <HardwareSerial.h>
#define DEBUG_PORT SerialASC
const uint32_t DEBUG_BAUD = 115200;

const uint8_t MEGA_I2C_ADDR   = 26;

#ifdef ARDUINO_ARCH_AVR
  #define SIMULATE_DEVICES
  #define WEEDINATOR_SINGLE_CORE

  #define SerialASC Serial

  #define StartOfUninitialised_LMURam_Variables
  #define EndOfUninitialised_LMURam_Variables
  #define StartOfInitialised_LMURam_Variables
  #define EndOfInitialised_LMURam_Variables
  #define StartOfUninitialised_CPU1_Variables
  #define EndOfUninitialised_CPU1_Variables
  #define StartOfInitialised_CPU1_Variables
  #define EndOfInitialised_CPU1_Variables
#endif

#if defined( SIMULATE_DEVICES )

  #define emicPort Serial
  const uint32_t EMIC_BAUD = DEBUG_BAUD;

  static const bool useConsole = true;

#else

  #define emicPort Serial1
  const uint32_t EMIC_BAUD = 9600;

  static const bool useConsole = false;

#endif

static const bool useI2C     = not useConsole;
static const bool useTFT     = not useConsole;

#endif
