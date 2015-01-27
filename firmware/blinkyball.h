#ifndef BLINKYBALL_H_
#define BLINKYBALL_H_

// Define pins
#define PIN_LED_ON      PB1     // LED control pin; pulled low externally
#define PIN_WAKEUP      PB2     // Wakeup pin; pulled high externally
#define PIN_IR_POWER    PB4     // IR Power supply
#define PIN_IR_DATA     PB3     // IR Data input

#define PIN_UNUSED      PB0     // Unused pin; should configure as pull-up

#define bitSet(reg, bit) reg |= (1<<bit)
#define bitClear(reg, bit) reg &= ~(1<<bit)

#include <avr/io.h>
#include <util/delay.h>

inline void pulseOut(uint8_t data) {
    const uint8_t BIT_DELAY = 104; // ~9600 baud

    // start bit
    bitClear(PORTB, PIN_UNUSED);
    _delay_us(BIT_DELAY);

    // data bits, lsb first
    for(uint8_t bit = 0; bit < 8; bit++) {
        if((data >> bit) & 1) {
            bitSet(PORTB, PIN_UNUSED);
        }
        else {
            bitClear(PORTB, PIN_UNUSED);
        }
        _delay_us(BIT_DELAY);
    }

    // Stop bits
    bitSet(PORTB, PIN_UNUSED);
    _delay_us(BIT_DELAY);
    _delay_us(BIT_DELAY);
}

#endif
