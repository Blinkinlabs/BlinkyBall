// Show what the blinkyball LED output looks like

#include "ekg_data.h"
#include <util/delay.h>


uint8_t heartbeatReps = 6;  // Number of times to play the loop
uint8_t heartbeatSpeed = 0;

const uint8_t ledPin = 11;  // OC2A


// note: minimum bpm is 48 to avoid overflow
void setSpeedFromBPM(uint8_t bpm) {
  heartbeatSpeed = uint8_t(100000*60.0/bpm/500);
}

void setup() {
  pinMode(ledPin, OUTPUT);
  
  setSpeedFromBPM(48);
  
  // Configure Timer2 to run at a higher frequency
  TCCR2B &= 0xF8;
  TCCR2B |= _BV(CS20);
}


inline void setLEDs(uint8_t val) {
  analogWrite(ledPin, val);  
}


// Play back one loop of the heartbeat signal
inline void playEKG() {
    uint8_t position = 0;
    uint8_t count = 0;

    for(position = 0; position < EKG_DATA_LENGTH; position++) {
        setLEDs(pgm_read_byte(&ekgData[position]));

        // make a delay
        // Note that the system is heavily loaded by the IR sensor,
        // so this number has to be tuned.
        for(count = 0; count < heartbeatSpeed; count++) {
            _delay_us(10);
        }
    }
}

void loop() {
  // Play the EKG back, checking for an IR reception
  for(uint8_t loopCount = heartbeatReps; loopCount > 0; loopCount--) {
      // For the first 22/50th of the heartbeat, play the EKG sample
      playEKG();
      setLEDs(0);
  
      for(int irLoop = 20; irLoop < (500 - EKG_DATA_LENGTH); irLoop++) {
          for(uint8_t count = 0; count < heartbeatSpeed; count++) {
              _delay_us(10);
          }
      }
  }
}
