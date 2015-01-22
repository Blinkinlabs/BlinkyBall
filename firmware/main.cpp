/* Name: main.c
 * Heartbeat ball
 *
 * Copyright 2015 Blinkinlabs, LLC
 * Author: Matt Mets
 *
 * Based loosely on the ATTiny example by Markus Konrad
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "ekg_data.h"
//#include "irremote.h"

// Define pins

#define PIN_LED_ON   PB1

#define PIN_WAKEUP   PB2
#define PIN_IR_POWER PB4
#define PIN_IR_DATA  PB3


// System parameters

// Bounce sensitivity, in interrupt counts. Increase to decrease sensitivity
//#define DEBOUNCE_COUNT 20
#define DEBOUNCE_COUNT 2

// Number of heartbeats played during each on-time
#define HEARTBEAT_REPS 6

#define bitSet(reg, bit) reg |= (1<<bit)
#define bitClear(reg, bit) reg &= ~(1<<bit)

// long delay function
void long_delay_ms(uint16_t ms) {
    for(ms /= 10; ms>0; ms--) _delay_ms(10);
}


// Counter for interrupt-based debounce routine
volatile uint8_t interrupt_count = 0;

void sleep()
{
    interrupt_count = 0;

    bitSet(GIMSK, INT0);    // Enable INT0

    sleep_enable(); // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei(); // Enable interrupts
    sleep_cpu(); // SLEEP
 
    sleep_disable(); // Clear SE bit

    // Wait a small amount of time to accrue debounce information   
    _delay_ms(1);

    cli(); // Disable interrupts
    bitClear(GIMSK, INT0);      // Disable INT0
    sei(); // Enable interrupts
}

ISR(INT0_vect) {
    interrupt_count++;
}

void setLEDs(uint8_t value) {
//    OCR0B = value;
    OCR1A = value;
}

void playEKG() {
    int count;
    int position;

    for(count = 0; count < HEARTBEAT_REPS; count++) {
        for(position = 0; position < EKG_DATA_LENGTH; position++) {
            setLEDs(pgm_read_byte(&ekgData[position]));

            // make a delay
            _delay_ms(2);
        }
    }
}

void solidOn() {
    setLEDs(255);
    long_delay_ms(1000);
}

// program entry point
// Kinda sad that the production devices will only run this once T_T
int main(void) {

    bitSet(ACSR, ACD);          // Disable the analog comparitor
    bitClear(ADCSRA, ADEN);     // Disable the ADC
    // Note: Brown out detection disabled by fuse setting
    // Note: Watchdog disabled by fuse setting

    // Disable clicks to peripherals that we aren't using
    // (This saves power in run mode)
    PRR |= _BV(PRUSI) | _BV(PRADC);

    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));      //INT0 on low level // TODO: This is the default?
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Set up timer1 to do PWM output to the LEDs (OC1A)
    
    OCR1C = 0xFF;
    TCCR1 = _BV(PWM1A) | _BV(COM1A0) | _BV(CS10);

//    // Set up Timer0 to do PWM output to the LEDs (OC0A, OC0B)
//    // Use phase correct PWM mode
//    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
//    TCCR0B = _BV(CS00);

    // Main loop- enable the LED outputs, run the pattern, then disable the outputs and sleep.
    for(;;){
//        // Turn off the LED outputs before disabling the pins
//        setLEDs(0);
//        _delay_ms(1);
//
//        // Set all I/O pins to input, and disable pull-up resistors
//        // TODO: Enable pullups for unused I/O
//        PORTB = 0;
//        DDRB = 0;
//
//        // Go to sleep
//        sleep();
//
//        // Do a quick debounce check, to discard small shakes
//        if(interrupt_count > DEBOUNCE_COUNT) {

            // Set the LED pin as a output, and enable the IR receiver
            DDRB = _BV(PIN_LED_ON) | _BV(PIN_IR_POWER);
            PORTB = _BV(PIN_IR_POWER);
            //PORTB = 0;

            playEKG();
            //solidOn();

//        }
    }
    
    return 0;   /* never reached */
}
