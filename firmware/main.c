/* Name: main.c
 * This is just an example for AVR-C with ATtiny microcontrollers. The code
 * implements digital switching of two pins, letting two connected LEDs blink.
 * It is part of the following instructable: 
 *
 * Author: Markus Konrad
 * Copyright 2013. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the mkonrad.net nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "ekg_data.h"

// Define pins

#define PIN_LED_TOP  PB1
#define PIN_LED_BOT  PB0

#define PIN_WAKEUP   PB2
#define PIN_IR_DATA  PB4

// Define helper macros

// write digital "high" to pin <pn> on port <prt>
#define DIGIWRITE_H(prt, pn) prt |= _BV(pn)

// write digital "low" to pin <pn> on port <prt>
#define DIGIWRITE_L(prt, pn) prt &= ~_BV(pn)

#define bitSet(reg, bit) reg |= (1<<bit)
#define bitClear(reg, bit) reg &= ~(1<<bit)

// Define long delay function

void long_delay_ms(uint16_t ms) {
    for(ms /= 10; ms>0; ms--) _delay_ms(10);
}

void flash() {
    static uint8_t toggle = 0;

    // alternate between the LEDs to let them blink
    if(toggle) {
        DIGIWRITE_L(PORTB, PIN_LED_TOP);
        DIGIWRITE_H(PORTB, PIN_LED_BOT);
    }
    else {
        DIGIWRITE_H(PORTB, PIN_LED_TOP);
        DIGIWRITE_L(PORTB, PIN_LED_BOT);
    }
    
    // alternave the toggle variable
    toggle = !toggle;

}

void sleep()
{
    bitSet(GIMSK, INT0);    // Enable INT0
    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));      //INT0 on low level

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable(); // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)

    sei(); // Enable interrupts
    sleep_cpu(); // SLEEP
  
    cli(); // Disable interrupts
    bitClear(GIMSK, INT0);      // Disable INT0
    sleep_disable(); // Clear SE bit
  
    sei(); // Enable interrupts
}

// TODO: Drop this foolishness.
volatile int interrupt_count = 0;

ISR(INT0_vect) {
    interrupt_count++;
}

void setLEDs(uint8_t value) {
    OCR0A = value;
    OCR0B = value;
}

void playEKG() {
    int i;

    for(i = 0; i < EKG_DATA_LENGTH; i++) {
        setLEDs(pgm_read_byte(&ekgData[i]));

        // make a delay
        _delay_ms(10);
    }
}

void solidOn() {
    int i;
    setLEDs(255);
    long_delay_ms(4000);
}

// program entry point
int main(void) {

    // TODO: Make sure all unused peripherals are disabled!
    bitSet(ACSR, ACD);          // Disable the analog comparitor
    bitClear(ADCSRA, ADEN);     // Disable the ADC
    // Note: Brown out detection disabled by fuse setting
    // Note: Watchdog disabled by fuse setting

    // Also disable clicks to peripherals that we aren't using
    PRR |= _BV(PRTIM1) | _BV(PRUSI) | _BV(PRADC);

    // Set up Timer0 to do PWM output to the LEDs (OC0A, OC0B)
    // Use phase correct PWM mode
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR0B = _BV(CS00);
    OCR0A = 0;
    OCR0B = 0;
    

    // main loop - cycle for 5 seconds, then sleep
    for(;;){
        // Set the LED pins as outputs
        DDRB |= _BV(PIN_LED_TOP) | _BV(PIN_LED_BOT);
    
        // Set all outputs to low and disable pull-up resistors
        // TODO: Configure unused pins as pullups
        PORTB = 0;

        //playEKG();
        solidOn();

        // Clear the transistors before disabling the ports
        // TODO: Prevent drift on portb from enabling the transistor??
        setLEDs(0);
        _delay_ms(1);

        PORTB = 0;
        DDRB = 0;

        // Go back to sleep
        // TODO: Don't sleep if bouncing is still going on
        sleep();
    }
    
    return 0;   /* never reached */
}
