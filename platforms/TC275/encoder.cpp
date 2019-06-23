#include "IfxGpt12_reg.h"
#include "encoder.h"
#include "TC275.h"

volatile uint8 Port00_Sample = 0;
uint32 volatile PinIntFunc2_var = 0;
uint32 volatile PinIntFunc3_var = 0;
bool volatile encoderDirection;
uint16 volatile encoderCount;
////////////////////////////////////////////////////////////////
void encoderReadings()
{
  encoderDirection = GPT120_T3CON.B.T3RDIR;
  /* Get position count from Timer3 */
  encoderCount = (uint16)GPT120_T3.U;
}
void IncrEnc_Init(void)
{

   /* Clear Einit protection */
     IfxScuWdt_clearCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());

     /* If GTM is disabled then enable it */
     while(GPT120_CLC.B.DISS == 1u)
     {

      GPT120_CLC.B.DISR = 0u;

     }

     /* Set Einit protection */
     IfxScuWdt_setCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());

     /* Set P2.6/7/8 to input mode */
     P02_IOCR4.B.PC6 = 0x00u; /* Input mode IOP_U_A, ShieldBuddy pin D8 */
     P02_IOCR4.B.PC7 = 0x00u; /* Input mode IOP_U_B, ShieldBuddy pin D9 */
     P02_IOCR8.B.PC8 = 0x00u; /* Input mode T4IN, , ShieldBuddy pin D51 */

     /* Configure pin inputs to GPT12 */
     GPT120_PISEL.U = 0x00u; /* Set all pin input options to default (T3INA, T3EUDA, T4INA) */

     /* Set up GPT120 Encoder Input Mode */
     GPT120_T3CON.U = 0x0000u;    /* Default configuration */
     GPT120_T3CON.B.BPS1 = 0x00u;   /* T3 runs at 2.5MHz   */
     GPT120_T3CON.B.T3M = 0x7u;   /* Interrupt on edge detect mode (not used) */
     GPT120_T3CON.B.T3UD = 1u;
     GPT120_T3CON.B.T3I = 3u; /* Any transition (rising or falling edge) on any Tx input (TxIN or TxEUD). */

     GPT120_T4CON.B.CLRT3EN = 1u; /* Reset T3 using mechanical zero position input (T4IN, P2.8) */

     /* Start encoder position monitoring */
     GPT120_T3CON.B.T3R = 1u ;
 }
