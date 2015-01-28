#ifndef BLINKYBALL_H_
#define BLINKYBALL_H_

// Define pins
#define PIN_LED_ON      PB1     // LED control pin; pulled low externally
#define PIN_WAKEUP      PB2     // Wakeup pin; pulled high externally
#define PIN_IR_POWER    PB4     // IR Power supply
#define PIN_IR_DATA     PB3     // IR Data input

#define PIN_UNUSED      PB0     // Unused pin; should configure as pull-up

// System parameters
#define DEBOUNCE_COUNT_DEFAULT  15
#define HEARTBEAT_REPS_DEFAULT  6
#define HEARTBEAT_SPEED_DEFAULT 200

// EEPROM data addresses
#define MAGIC_HEADER_ADDRESS    0
#define DEBOUNCE_COUNT_ADDRESS  1
#define HEARTBEAT_REPS_ADDRESS  2
#define HEARTBEAT_SPEED_ADDRESS 3

// Magic header to determine if the EEPROM was written properly
#define  MAGIC_HEADER_VALUE     0xDE


#define bitSet(reg, bit) reg |= (1<<bit)
#define bitClear(reg, bit) reg &= ~(1<<bit)

#define setLEDs(value) OCR1A = value

#include <avr/io.h>
#include <util/delay.h>

void pulseOut(uint8_t data);
uint8_t measureBattery();

#endif
