/*
 * IRremote
 * Version 0.11 August, 2009
 *
 * Ported to C for use in ATTiny, 2014 Matt Mets
 *
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
 * Modified  by Mitra Ardron <mitra@mitra.biz> 
 * Added Sanyo and Mitsubishi controllers
 * Modified Sony to spot the repeat codes that some Sony's send
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 * LG added by Darryl Smith (based on the JVC protocol)
 */

#include "irremote.h"
#include "irremoteint.h"

// Provides ISR
#include <avr/interrupt.h>


//// Arduino Shim //////
#include <avr/io.h>
#include <util/delay.h>

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

//// Arduino Shim //////

volatile decode_results_t results;
volatile irparams_t irparams;

// Private functions
long decodeNEC();
int compare(unsigned int oldval, unsigned int newval);




// These versions of MATCH, MATCH_MARK, and MATCH_SPACE are only for debugging.
// To use them, set DEBUG in IRremoteInt.h
// Normally macros are used for efficiency
#ifdef DEBUG
int MATCH(int measured, int desired) {
  Serial.print("Testing: ");
  Serial.print(TICKS_LOW(desired), DEC);
  Serial.print(" <= ");
  Serial.print(measured, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired), DEC);
  return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}

int MATCH_MARK(int measured_ticks, int desired_us) {
  Serial.print("Testing mark ");
  Serial.print(measured_ticks * USECPERTICK, DEC);
  Serial.print(" vs ");
  Serial.print(desired_us, DEC);
  Serial.print(": ");
  Serial.print(TICKS_LOW(desired_us + MARK_EXCESS), DEC);
  Serial.print(" <= ");
  Serial.print(measured_ticks, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired_us + MARK_EXCESS), DEC);
  return measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS);
}

int MATCH_SPACE(int measured_ticks, int desired_us) {
  Serial.print("Testing space ");
  Serial.print(measured_ticks * USECPERTICK, DEC);
  Serial.print(" vs ");
  Serial.print(desired_us, DEC);
  Serial.print(": ");
  Serial.print(TICKS_LOW(desired_us - MARK_EXCESS), DEC);
  Serial.print(" <= ");
  Serial.print(measured_ticks, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired_us - MARK_EXCESS), DEC);
  return measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS);
}
#else
inline uint8_t MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
inline uint8_t MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
inline uint8_t MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}
// Debugging versions are in IRremote.cpp
#endif


// initialization
void enableIRIn() {
  cli();
  // setup pulse clock timer interrupt
  //Prescale /8 (16M/8 = 0.5 microseconds per tick)
  // Therefore, the timer interval can range from 0.5 to 128 microseconds
  // depending on the reset value (255 to 0)
  TIMER_CONFIG_NORMAL();

  //Timer2 Overflow Interrupt Enable
  TIMER_ENABLE_INTR;

  TIMER_RESET;

  sei();  // enable interrupts

  // initialize state machine variables
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
  irparams.timer = 0;

  // set pin modes
  //pinMode(irparams.recvpin, INPUT);
}

void disableIRIn() {
    TIMER_DISABLE_INTR;
}

// TIMER2 interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts

uint8_t irdata;
ISR(TIMER_INTR_NAME)
{
  TIMER_RESET;

  irdata = ((PINB & _BV(PIN_IR_DATA))?SPACE:MARK);

  irparams.timer++; // One more 50us tick
  if (irparams.rawlen >= RAWBUF) {
    // Buffer overflow
    irparams.rcvstate = STATE_STOP;
  }
  switch(irparams.rcvstate) {
  case STATE_IDLE: // In the middle of a gap
    if (irdata == MARK) {
      if (irparams.timer < GAP_TICKS) {
        // Not big enough to be a gap.
        irparams.timer = 0;
      } 
      else {
        // gap just ended, record duration and start recording transmission
        irparams.rawlen = 0;
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
    }
    break;
  case STATE_MARK: // timing MARK
    if (irdata == SPACE) {   // MARK ended, record time
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == MARK) { // SPACE just ended, record it
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_MARK;
    } 
    else { // SPACE
      if (irparams.timer > GAP_TICKS) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        irparams.rcvstate = STATE_STOP;
      } 
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == MARK) { // reset gap timer
      irparams.timer = 0;
    }
    break;
  }

#if 0
  if (irparams.blinkflag) {
    if (irdata == MARK) {
      BLINKLED_ON();  // turn pin 13 LED on
    } 
    else {
      BLINKLED_OFF();  // turn pin 13 LED off
    }
  }
#endif
}

void resumeIR() {
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
}



// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int decodeIR() {
  results.rawbuf = irparams.rawbuf;
  results.rawlen = irparams.rawlen;
  if (irparams.rcvstate != STATE_STOP) {
    return ERR;
  }

  cli();
  if (decodeNEC()) {
    sei();
    return DECODED;
  }
  sei();

  // Throw away and start over
  resumeIR();
  return ERR;
}

// NECs have a repeat only 4 items long
long decodeNEC() {
//  TODO: Delete me?
//  pulseOut(0xA1);
//  pulseOut(0xA2);
//  pulseOut(irparams.rawlen);
//
//  for(uint8_t pos=0; pos < irparams.rawlen; pos++) {
//    pulseOut(results.rawbuf[pos]);
//  }

  long data = 0;
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(results.rawbuf[offset], NEC_HDR_MARK)) {
    return ERR;
  }
  offset++;

  // Check for repeat
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(results.rawbuf[offset], NEC_RPT_SPACE) &&
    MATCH_MARK(results.rawbuf[offset+1], NEC_BIT_MARK)) {
    results.bits = 0;
    results.value = REPEAT;
    return DECODED;
  }

//  pulseOut(0xEE);
//  pulseOut(0x00);
//  pulseOut(irparams.rawlen);

  if (irparams.rawlen < 2 * NEC_BITS + 4) {
    return ERR;
  }

  // Initial space  
  if (!MATCH_SPACE(results.rawbuf[offset], NEC_HDR_SPACE)) {
    return ERR;
  }
  offset++;

  for (int i = 0; i < NEC_BITS; i++) {
    if (!MATCH_MARK(results.rawbuf[offset], NEC_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results.rawbuf[offset], NEC_ONE_SPACE)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_SPACE(results.rawbuf[offset], NEC_ZERO_SPACE)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
  }
  // Success
  results.bits = NEC_BITS;
  results.value = data;
  return DECODED;
}

