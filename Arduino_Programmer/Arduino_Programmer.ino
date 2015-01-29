/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
#include "crc.h"

const uint8_t PIN_IR_OUT = 3;
const uint8_t PIN_IR_GND = 4;

const uint8_t DEFAULT_SENSITIVITY = 0;
const uint8_t DEFAULT_COUNTS = 255;
const uint8_t DEFAULT_BPM = 40;

IRsend irsend;

void setup()
{
  Serial.begin(9600);
  
  pinMode(PIN_IR_GND, OUTPUT);
  digitalWrite(PIN_IR_GND, LOW);
}

// Update the orb heartbeat
// @param bpm: New heartbeat rate, in BPM
// @param counts: Number of times the heartbeat should be repeated when the orb is activated
// @param sensitivity: Sensitivity
void sendHeartbeatParameters(uint8_t bpm, uint8_t counts, uint8_t sensitivity) {
  
  // Convert the heart rate from BPM to tenths-of-a-millisecond/sample
  uint8_t rate = uint8_t(10000*60.0/bpm/497*7);
  
  resetCRC();
  updateCRC(rate);
  updateCRC(counts);
  updateCRC(sensitivity);
  
  long data = 0;
  data |= ((long)rate)        << 24;
  data |= ((long)counts)      << 16;
  data |= ((long)sensitivity) << 8;
  data |= (getCRC());

  irsend.sendNEC(data, 32); // NEC code
}

void loop() {
  sendHeartbeatParameters(DEFAULT_BPM, DEFAULT_COUNTS, DEFAULT_SENSITIVITY);
  delay(500);
}
