#ifndef BLINKYBALL_H_
#define BLINKYBALL_H_

// Define pins
#define PIN_LED_ON      PB1     // LED control pin; pulled low externally
#define PIN_WAKEUP      PB2     // Wakeup pin; pulled high externally
#define PIN_IR_POWER    PB4     // IR Power supply
#define PIN_IR_DATA     PB3     // IR Data input

#define PIN_UNUSED      PB0     // Unused pin; should configure as pull-up

// System parameters
#define IR_MONITOR_TIME         30  // Amount of time to stay awake for IR reception, in 10ths of a second

#define DEBOUNCE_COUNT_DEFAULT   15 // Wakeup sensitivity, in counts (higher is less sensitive)
#define REPEAT_COUNT_DEFAULT      1 // Repeat count in 7-pulse units
#define HEARTBEAT_SPEED_DEFAULT 140 // Heartbeat speed in units

#define REPEAT_COUNT_MAXIMUM      4 // Maximum repeat counts

// EEPROM data addresses
#define MAGIC_HEADER_ADDRESS    0
#define DEBOUNCE_COUNT_ADDRESS  1
#define REPEAT_COUNT_ADDRESS    2
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
