/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
#include "crc.h"

const uint8_t DEFAULT_SENSITIVITY = 255;
const uint8_t DEFAULT_COUNTS = 10;

IRsend irsend;

void setup()
{
  Serial.begin(9600);
}

// Update the orb heartbeat
// @param rate: New heartbeat rate, in BPM
// @param counts: Number of times the heartbeat should be repeated when the orb is activated
// @param sensitivity: Sensitivity
void sendHeartbeatParameters(uint8_t rate, uint8_t counts, uint8_t sensitivity) {
  
  // Convert the heart rate from BPM to the system value
  rate = uint8_t(100000*60.0/bpm/500);
  
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
  
  sendHeartbeatParameters(60, DEFAULT_COUNTS, DEFAULT_SENSITIVITY);
  delay(150);
}
