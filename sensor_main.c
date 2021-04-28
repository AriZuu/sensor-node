/*
 * Copyright (c) 2006-2013, Ari Suutari <ari@stonepile.fi>.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <picoos.h>
#include <picoos-u.h>
#include <stdbool.h>

#include <string.h>
#ifdef ONEWIRE
#include <picoos-ow.h>
#include <temp10.h>
#endif

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "mrfi_defs.h"

#include "sensor_msg.h"
#include "calib_data.h"

#define MAIN_STACK_SIZE 260
#define IDLE_STACK_SIZE 60
#define MEAS_TIMER_SECS (5 * 60)

#ifdef MRFI_CC430

// Olimex MSP430-CCRF

#include <HAL_TLV.h>

#define     LED_RED               BIT0
#define     LED_GREEN             0 // not present
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT

#endif

#ifdef MRFI_CC2500

// TI ez430-RF2500

#define     LED_RED               BIT0
#define     LED_GREEN             BIT1
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT

#ifndef KWH
#define KWH
#endif

#endif

#if __GNUC__ == 4
extern int __infod[];
#else
int __attribute__((section(".infoD"))) __infod[];
#endif
unsigned char rf_freqoffset;

#if NOSCFG_FEATURE_CONOUT == 1
#define DEBUG_nosPrint(a) nosPrint(a)
#define DEBUG_nosPrintf(args...) nosPrintf(args)
#else
#define DEBUG_nosPrint(a) do{ } while ( false )
#define DEBUG_nosPrintf(args...) do{ } while ( false )
#endif

static POSSEMA_t adcSem;
static POSSEMA_t timerSem;
static POSTIMER_t mainTimer;

#ifdef MRFI_CC2500
void radioGpioInterrupt(void);
#endif

#ifdef KWH
void pulseInterrupt(void);
#endif

void adcInterrupt(void);

static void ledBlink(int led, int cnt);
static bool joinAp(void);
static void readSensors(bool);

#ifdef ONEWIRE
static void readOneWire(void);
#endif

static int sendPacket(void);
static void mainTask(void *arg);

#ifdef __CC430F5137__

float mref, nref;
struct s_TLV_ADC_Cal_Data * pADCCAL;
unsigned char bADCCAL_bytes;
struct s_TLV_Die_Record* die;

#else

volatile unsigned int * tempOffset = (unsigned int *) 0x10F4; // Temperature offset set at production

#endif

#define MISSES_IN_A_ROW  3

static linkID_t sLinkID1 = 0;
static volatile unsigned int adcResult;
static SensorMsg msg;
static SensorMsgData* msgData;

#define MSG_HAS_ROOM (msgData - msg.data < MSG_MAX_SENSOR_VALUES)

static volatile uint16_t pulses = 0;

extern unsigned int _end[];

static void ledBlink(int led, int cnt)
{
  int i;

  if (led == 0)
    led = LED_RED;

  for (i = 0; i < cnt; i++) {

    if (i > 0)
      posTaskSleep(MS(1000));

    LED_OUT |= led;
    uosSpinUSecs(100);
    LED_OUT &= ~led;
  }
}

static unsigned int adcConvert(unsigned int chan, unsigned int sht, unsigned int ref)
{
#ifdef __CC430F5137__

  REFCTL0 |= REFMSTR + ref + REFON; // Enable internal reference (1.5V or 2.5V)

  // Initialize ADC12_A
  ADC12CTL0 = sht + ADC12ON; // Set sample time
  ADC12CTL1 = ADC12SHP; // Enable sample timer
  ADC12MCTL0 = ADC12SREF_1 + chan; // ADC input channel
  ADC12IE = 0x001; // ADC_IFG upon conv result-ADCMEMO

  __delay_cycles(128 * PORTCFG_CPU_CLOCK_MHZ);

  ADC12CTL0 |= ADC12ENC;

  // Sampling and conversion start
  ADC12CTL0 |= ADC12SC;

  posSemaGet(adcSem);

  ADC12CTL0 &= ~(ADC12ENC | ADC12SC | sht);
  ADC12CTL0 &= ~ADC12ON;

  // Shut down reference voltage
  REFCTL0 &= ~(REFMSTR + ref + REFON);

  ADC12IE = 0;

#else

  ADC10CTL1 = chan + ADC10DIV_4; // ADC10CLK/5
  ADC10CTL0 = SREF_1 + ref + sht + REFON + ADC10ON + ADC10IE + ADC10SR;
  __delay_cycles(128 * PORTCFG_CPU_CLOCK_MHZ);
  ADC10CTL0 |= ENC + ADC10SC;// Sampling and conversion start

  posSemaGet(adcSem);

  ADC10CTL0 &= ~ENC;
  ADC10CTL0 &= ~(REFON + ADC10ON);// turn off A/D to save power

#endif

  return adcResult;
}

static bool joinAp()
{
  uint8_t misses;

  /* Keep trying to join (a side effect of successful initialization) until
   * successful. Toggle LEDS to indicate that joining has not occurred.
   */

  misses = 0;
  while (SMPL_SUCCESS != SMPL_Init(0) && misses < 3) {

    ledBlink(LED_RED, 1);
    posTaskSleep(MS(3000));
    ++misses;
  }

  if (misses < 3) {

    DEBUG_nosPrint("Joined\n");

    /* Keep trying to link... */
    misses = 0;
    while (SMPL_SUCCESS != SMPL_Link(&sLinkID1) && misses < 10) {

      ledBlink(LED_RED, 2);
      posTaskSleep(MS(3000));
      ++misses;
    }

    if (misses < 10) {

      DEBUG_nosPrint("Linked\n");
      ledBlink(LED_GREEN, 2);
      return 1;
    }
  }

  return 0;
}

static void saveData(uint8_t id, uint8_t type, int16_t value)
{
  if (MSG_HAS_ROOM) {

    msgData->id = id;
    msgData->type = type;
    msgData->value = value;
    msgData++;
    DEBUG_nosPrint(" OK\n");
  }
  else
    DEBUG_nosPrint(" Full\n");

}

#ifdef ONEWIRE

#if NOSCFG_FEATURE_CONOUT == 1
static void sensorAddressStr(uint8_t* serialNum)
{
  int i;

  for (i = 7; i >= 0; i--) {

    nosPrintf("%X", (int)serialNum[i]);

    if (i > 0)
      nosPrint("-");
   }
}
#endif

static void readOneWire()
{
  uchar   rslt;
  int     sens;
  float   temp;
  int     portNum = 0;
  uint8_t serialNum[8];
 // char buf[40];

  if (!owAcquire(portNum, NULL)) {

    DEBUG_nosPrint("owAcquire failed\n");
    return;
  }

  //posTaskSleep(MS(100));
  rslt = owFirst(portNum, TRUE, FALSE);

  sens = 0;
  while (rslt) {

    owSerialNum(portNum, serialNum, TRUE);

#if NOSCFG_FEATURE_CONOUT == 1
    nosPrint("1-Wire ");
    sensorAddressStr(serialNum);
#endif

    ReadTemperature(0, serialNum, &temp);
    DEBUG_nosPrintf(" Temp=%d.%d oC", (int)temp, (int)(temp * 10) % 10);
    saveData(sens + 1, SENSOR_TEMPERATURE, temp * 10);

    ++sens;
    rslt = owNext(portNum, TRUE, FALSE);
  }

  owRelease(portNum);
}

#endif

static void readSensors(bool pulseValid)
{
  int degC, volt;
  int results[2];
  /* get radio ready...awakens in idle state */

#ifdef __CC430F5137__

  results[0] = adcConvert(ADC12INCH_10, ADC12SHT0_8, REFVSEL_0);
  results[1] = adcConvert(ADC12INCH_11, ADC12SHT0_10, REFVSEL_1);

  degC = (results[0] - nref) / mref * 10;

  // Convert ADC value to "x.xx V"
  // Ideally we have A11=0->AVCC=0V ... A11=4095(2^12-1)->AVCC=4V
  // --> (A11/4095)*4V=AVCC --> AVCC=(A11*4)/4095
  volt = (results[1] * 2 * 2) / 41;
  volt = volt / 10;

#else

  volatile long temp;

  results[0] = adcConvert(INCH_10, ADC10SHT_3, 0);
  results[1] = adcConvert(INCH_11, ADC10SHT_3, REF2_5V);

  // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278
  // the temperature is transmitted as an integer where 32.1 = 321
  // hence 4230 instead of 423
  temp = results[0];
  degC = ((temp - 673) * 4230) / 1024;

  if ((*tempOffset) != 0xFFFF)
  {
    degC += (*tempOffset);
  }

  temp = results[1];
  volt = (temp * 25) / 512;

#endif

  DEBUG_nosPrintf("Batt %d.%d", volt / 10, volt % 10);
  saveData(0, SENSOR_BATTVOLTAGE, volt);

  DEBUG_nosPrintf("Temp %d.%d", degC / 10, degC % 10);
  saveData(0, SENSOR_TEMPERATURE, degC);

#ifdef KWH

  uint16_t wh;
  POS_LOCKFLAGS;

  POS_SCHED_LOCK;
  wh = pulses;
  pulses = 0;
  POS_SCHED_UNLOCK;

  DEBUG_nosPrintf("Energy %d Wh",  (int)wh);
  saveData(0, SENSOR_PULSES, wh);

  DEBUG_nosPrint("Power");
  saveData(0, SENSOR_POWER,  wh * 3600L / MEAS_TIMER_SECS);

#endif
}

static int sendPacket()
{
  uint8_t misses;
  uint8_t noAck;
  smplStatus_t rc;

  noAck = 0;

  /* Try sending message MISSES_IN_A_ROW times looking for ack */
  for (misses = 0; misses < MISSES_IN_A_ROW; ++misses) {

    if (SMPL_SUCCESS == (rc = SMPL_SendOpt(sLinkID1, (uint8_t*) &msg, sizeof(msg), SMPL_TXOPTION_ACKREQ))) {

      return SMPL_SUCCESS;
    }

    if (SMPL_NO_ACK == rc) {

      /* Count ack failures. Could also fail becuase of CCA and
       * we don't want to scan in this case.
       */
      noAck++;
    }
  }

  if (MISSES_IN_A_ROW == noAck) {

    DEBUG_nosPrint("AP lost.\n");
    ledBlink(LED_RED, 5);

    SMPL_Ioctl(IOCTL_OBJ_CONNOBJ, IOCTL_ACT_DELETE, &sLinkID1);
    return SMPL_NO_JOIN;
  }

  return SMPL_NO_ACK;

}
static void mainTask(void *memstart)
{
  bool joined;
  bool lastAcked;
  smplStatus_t rc;
  uint8_t pass;

  LED_DIR |= LED_RED;
  LED_OUT &= ~LED_RED;

#ifdef MRFI_CC2500
  LED_DIR |= LED_GREEN;
  LED_OUT &= ~LED_GREEN;
#endif

#ifdef MRFI_CC430

  uint16_t actlevel;

  actlevel = (PMMCTL0 & PMMCOREV_3);     // Get actual VCore
  if (actlevel < PMMCOREV_2)
    portSetVCore(PMMCOREV_2); // oli 3

  WDTCTL = WDTPW + WDTIS__128M + WDTSSEL__ACLK + WDTCNTCL;

#endif

#if NOSCFG_FEATURE_CONOUT == 1
  nosPrint("Sensor start\n");
  uosBootDiag();
#endif

#ifdef __CC430F5137__
  if (__infod[0] == CALIB_DATA_FINGERPRINT)
    rf_freqoffset = __infod[1];
#endif

  memset(&msg, '\0', sizeof(msg));
#ifdef __CC430F5137__
  // TLV access Function Call
  Get_TLV_Info(TLV_ADCCAL, 0, &bADCCAL_bytes, (unsigned int **) &pADCCAL);

  if (pADCCAL->adc_ref15_30_temp == pADCCAL->adc_ref20_30_temp
      || pADCCAL->adc_ref15_30_temp == pADCCAL->adc_ref25_30_temp) { // Calibration data wrong, see device errata datasheet

    mref = ((float) (pADCCAL->adc_ref15_85_temp - 2198)) / (85 - 30);
    DEBUG_nosPrint("This chip has calibration data error.\n");
  }
  else {

    mref = ((float) (pADCCAL->adc_ref15_85_temp - pADCCAL->adc_ref15_30_temp)) / (85 - 30);
  }

  nref = pADCCAL->adc_ref15_85_temp - mref * 85;

  Get_TLV_Info(TLV_DIERECORD, 0, &bADCCAL_bytes, (unsigned int**) &die);
  if (die != NULL) {

    addr_t myAddr;

    myAddr.addr[0] = die->die_y_position & 0xff;
    myAddr.addr[1] = die->die_x_position & 0xff;
    myAddr.addr[2] = die->wafer_id & 0xff;
    myAddr.addr[3] = die->wafer_id >> 8;

    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &myAddr);
  }

#endif

  msg.pktVersion = 3;
  adcSem = posSemaCreate(0);
  timerSem = posSemaCreate(0);
  mainTimer = posTimerCreate();

  P_ASSERT("mainSema", adcSem != NULL && timerSem != NULL && mainTimer != NULL);

#ifdef _DBG
  posTimerSet(mainTimer, timerSem, MS(10000), MS(10000L));
#else
  posTimerSet(mainTimer, timerSem, MS(10000), MS(MEAS_TIMER_SECS * 1000L));
#endif
  posTimerStart(mainTimer);

#ifdef KWH

// Enable interrupts on KWH meter pulses.
// Sensor consistst of PT331C phototransistor and 220 kOhm pull-up 
// resistor connected to P2.5.

#ifdef MRFI_CC430
  P2IE |= BIT5;
#endif

#ifdef MRFI_CC2500
  P1IE |= BIT2;
#endif

#endif

  joined = false;
  lastAcked = false;
  msg.seq = 0;

  ledBlink(LED_GREEN, 5);

  joined = joinAp(); // Initialize radio
  SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);

  pass = 0;
  while (true) {

    lastAcked = false;

    msg.seq++;
    msgData = msg.data;

    // First two passes occur faster than normal,
    // this causes pulse measurements to be invalid.
    readSensors(pass >= 2);
#ifdef ONEWIRE
    readOneWire();
#endif

    // Clear tail of the packet.
    while (MSG_HAS_ROOM) {

      memset(msgData, '\0', sizeof (SensorMsgData));
      ++msgData;
    }

    if (pass < 2)
      ++pass;

    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);

    if (!joined) {

      joined = joinAp();
    }

    if (joined) {

      rc = sendPacket();
      if (rc == SMPL_SUCCESS)
        lastAcked = true;
      else if (rc == SMPL_NO_JOIN)
        joined = false;
    }

    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);

    if (lastAcked) {

#ifdef MRFI_CC430
      WDTCTL = WDTPW + WDTIS__128M + WDTSSEL__ACLK + WDTCNTCL;
#endif
      ledBlink(LED_GREEN, 1);
    }

    uosResourceDiag();
    posSemaGet(timerSem);
  }
}

void PORT_NAKED
#ifdef __CC430F5137__
__attribute__((interrupt(ADC12_VECTOR)))
#else
__attribute__((interrupt(ADC10_VECTOR)))
#endif
adcInterrupt()
{
  portSaveContext();
  c_pos_intEnter();

#ifdef __CC430F5137__

  if (ADC12IV == 6) {

    adcResult = ADC12MEM0;
    posSemaSignal(adcSem);

  }

#else

  adcResult = ADC10MEM;
  posSemaSignal(adcSem);

#endif

  c_pos_intExit();
  portRestoreContext();
}

#ifdef MRFI_CC2500
void PORT_NAKED __attribute__((interrupt(PORT2_VECTOR)))
radioGpioInterrupt()
{
  portSaveContext();
  c_pos_intEnter();

  /*
   *  This ISR is easily replaced.  The new ISR must simply
   *  include the following function call.
   */
  MRFI_GpioIsr();
  c_pos_intExit();
  portRestoreContext();

}
#endif

#ifdef KWH
#ifdef MRFI_CC430
void __attribute__((interrupt(PORT2_VECTOR))) pulseInterrupt()
{
  switch (P2IV)
  {
    case 12:
    ++pulses;
    break;
  }
}
#endif

#ifdef MRFI_CC2500
void __attribute__((interrupt(PORT1_VECTOR))) pulseInterrupt()
{
  if (P1IFG & BIT2) {

    P1IFG &= ~BIT2;
    ++pulses;
  }
}

#endif
#endif

/*
 * Initialize board pins.
 */

static void initPins(void)
{

  // To save power, set unused IO pins to output.

#ifdef MRFI_CC2500

//  P1.0 LED
//  P1.1 LED
//  P1.2 SW
//  P1.3-P1.7 free

  P1OUT = BIT2;
  P1SEL = 0;
  P1DIR = BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
  P1REN = BIT2;

//  P2.0-2.5 free
//  P2.6-2.7 radio

  P2OUT = 0;
  P2SEL = 0;
  P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;

//  P3.0-3.3 radio
//  P3.4-3.5 RS232
//  P3.6-3.7 free

  P3OUT = 0;
  P3SEL = 0;
  P3DIR = BIT6 + BIT7;

//  P4.0-4.7 free

  P4OUT = 0;
  P4SEL = 0;
  P4DIR = 0xff;

#endif

#ifdef MRFI_CC430

  // PULL-UP   P5.0 XIN
  // PULL-DOWN P5.1 XOUT

  // OUT       P0.1 LED
  // OUT       P1.1 button
  // PULL-DOWN P1.2-1.4 unused
  // PULL-DOWN P1.5-1.6 rs232
  // IN        P1.7 STE, on jo pull-up

  P1REN = BIT2 + BIT3 + BIT4;

#if NOSCFG_FEATURE_CONOUT == 0 && NOSCFG_FEATURE_CONIN == 0
  P1REN |= BIT5 + BIT6;                     // rx/rx pins not used
#endif

  // IN        P2.0-2.1 SCL / SDA

  P2DIR = 0;
  P2SEL = 0;
#ifdef KWH
  P2REN = 0xff & ~(BIT0 + BIT1 + BIT5); // P2.5 = Pulse meter
#else
#ifdef ONEWIRE
  P2REN = 0xff & ~(BIT0 + BIT1 + BIT6 + BIT7); // P2.6-7 OneWire interface
  P2DS |= BIT6 + BIT7;
#else
  P2REN = 0xff & ~(BIT0 + BIT1);
#endif
#endif

  P3REN = 0xff;

#if PORTCFG_XT1_HZ == 0

  // If XT1 is not in use, pull up pins.

  P5DIR = 0;
  P5SEL = 0;
  P5OUT = BIT0;
  P5REN = BIT0 + BIT1;

#endif

#ifndef _DBG
  // Pull down JTAG pins.
  PJDIR = 0;
  PJOUT = 0;
  PJREN = 0xFF;
#endif
#endif

#if PORTCFG_XT1_HZ > 0

#if defined(__cc430x513x)

  P5OUT = 0x00;
  P5SEL |= BIT1 + BIT0;

  UCSCTL6 &= (~XTS);        // Select Low-frequency mode.
  UCSCTL6 |= XCAP_3;        // Internal load cap.

#endif

#endif

#if NOSCFG_FEATURE_CONOUT == 1 || NOSCFG_FEATURE_CONIN == 1

/*
 * Program * TX & RX pin for usart use.
 */

  UCA0CTL1 |= UCSWRST;

#if defined(__MSP430F2274__)

  P3DIR |= BIT4;                            // Set P3.4 as TX output
  P3SEL |= 0x30;            // P3.4,5 = USCI_A0 TXD/RXD

#elif defined(__CC430F5137__)

  P1DIR |= BIT6;                            // Set P1.6 as TX output
  P1SEL |= BIT5 + BIT6;                     // Select P1.5 & P1.6 to UART function

#endif
#endif
}

int main(int argc, char **argv)
{
  initPins();
  uosInit();
  nosInit(mainTask, NULL, 10, MAIN_STACK_SIZE, IDLE_STACK_SIZE);
  return 0;
}
