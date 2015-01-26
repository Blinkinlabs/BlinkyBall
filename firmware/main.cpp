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
#include <avr/eeprom.h>
#include "ekg_data.h"
#include "irremote.h"

// Define pins
#define PIN_LED_ON      PB1     // LED control pin; pulled low externally
#define PIN_WAKEUP      PB2     // Wakeup pin; pulled high externally
#define PIN_IR_POWER    PB4     // IR Power supply
#define PIN_IR_DATA     PB3     // IR Data input

#define PIN_UNUSED      PB0     // Unused pin; should configure as pull-up


// System parameters
#define DEBOUNCE_COUNT_DEFAULT 15
#define HEARTBEAT_REPS_DEFAULT 6
#define HEARTBEAT_SPEED_DEFAULT 200

// EEPROM data addresses
const uint8_t MAGIC_HEADER_ADDRESS    = 0;
const uint8_t DEBOUNCE_COUNT_ADDRESS  = 1;
const uint8_t HEARTBEAT_REPS_ADDRESS  = 2;
const uint8_t HEARTBEAT_SPEED_ADDRESS = 3;

// Magic header to determine if the EEPROM was written properly
const uint8_t MAGIC_HEADER_VALUE = 0xDE;


// Bounce sensitivity, in interrupt counts. Increase to decrease sensitivity
volatile uint8_t debounceCount;

// Number of heartbeats played during each on-time
volatile uint8_t heartbeatReps;

// Heartbeat speed, in 10ths of a microsecond per sample
// To convert from BPM to this constant, use the following formula:
// ((60/BPM/SAMPLES)) × 100000
// where SAMPLES is equal to the heartbeat sample count (500)
// Examples:
// 60bpm: 200
// 100bpm: 120
volatile uint8_t heartbeatSpeed;

// Counter for interrupt-based debounce routine
volatile uint8_t interruptCount;

#define bitSet(reg, bit) reg |= (1<<bit)
#define bitClear(reg, bit) reg &= ~(1<<bit)

// long delay function
void long_delay_ms(uint16_t ms) {
    for(ms /= 10; ms>0; ms--) _delay_ms(10);
}

// IR Receiver 
IRrecv irrecv(PIN_IR_DATA);
decode_results results;


void sleep()
{
    interruptCount = 0;

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
    interruptCount++;
}

void setLEDs(uint8_t value) {
    OCR1A = value;
}

// Store the configuration rates in EEPROM
void saveRates() {
    eeprom_write_byte((uint8_t*)DEBOUNCE_COUNT_ADDRESS,   DEBOUNCE_COUNT_DEFAULT);
    eeprom_write_byte((uint8_t*)HEARTBEAT_REPS_ADDRESS,   HEARTBEAT_REPS_DEFAULT);
    eeprom_write_byte((uint8_t*)HEARTBEAT_SPEED_ADDRESS,  HEARTBEAT_SPEED_DEFAULT);
    eeprom_write_byte((uint8_t*)MAGIC_HEADER_ADDRESS,     MAGIC_HEADER_VALUE);
}

// Load the configuration rates from EEPROM, or create from default if they weren't
// already initialized
void loadRates() {
    // If the EEPROM wasn't initialized, do so now
    if(eeprom_read_byte((uint8_t*)MAGIC_HEADER_ADDRESS) != MAGIC_HEADER_VALUE) {
        debounceCount = DEBOUNCE_COUNT_DEFAULT;
        heartbeatReps = HEARTBEAT_REPS_DEFAULT;
        heartbeatSpeed = HEARTBEAT_SPEED_DEFAULT;
        saveRates();
    }

    debounceCount =  eeprom_read_byte((uint8_t*)DEBOUNCE_COUNT_ADDRESS);
    heartbeatReps =  eeprom_read_byte((uint8_t*)HEARTBEAT_REPS_ADDRESS);
    heartbeatSpeed = eeprom_read_byte((uint8_t*)HEARTBEAT_SPEED_ADDRESS);
}

void playEKG() {
    int position;

    for(position = 0; position < EKG_DATA_LENGTH; position++) {
        setLEDs(pgm_read_byte(&ekgData[position]));

        // make a delay
        for(uint8_t count = 0; count < heartbeatSpeed; count++) {
            _delay_us(9);  // Allow 1uS for loop setup
        }
    }
}

// program entry point
// Kinda sad that the production devices will only run this once T_T
int main(void) {

    // Change the clock to 8MHz
    asm volatile (
        "st Z,%1" "\n\t"
        "st Z,%2"
        : :
        "z" (&CLKPR),
        "r" ((uint8_t) (1<<CLKPCE)),
        "r" ((uint8_t) 0)  // new CLKPR value
    );

    bitSet(ACSR, ACD);          // Disable the analog comparitor
    bitClear(ADCSRA, ADEN);     // Disable the ADC
    // Note: Brown out detection disabled by fuse setting
    // Note: Watchdog disabled by fuse setting

    // Disable clicks to peripherals that we aren't using
    // (This saves power in run mode)
    PRR |= _BV(PRUSI) | _BV(PRADC);

    // Configure the sleep mode and wakeup interrupt
    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));    // INT0 on low level
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // Use power down mode for sleep

    // Set up timer1 to do PWM output to the LEDs (OC1A)
    OCR1C = 0xFF;
    TCCR1 = _BV(PWM1A) | _BV(COM1A0) | _BV(CS10);


    loadRates();

    // Main loop. The first thing that we do is turn off peripherals that we don't need,
    // then we go to sleep. When woken up, run a quick debounce routine to determine if we
    // should go into LED display mode, at at the completion go back to sleep.
    for(;;){

        // Disable the IR timer
        irrecv.disableIRIn();

        // Turn off the LED outputs before disabling the pins
        setLEDs(0);
        _delay_ms(1);

        // Set all I/O pins to input, and pull-up resistors for floating pins
        DDRB = 0;
        PORTB = _BV(PIN_UNUSED);

        // Go to sleep
        sleep();

        // Do a quick debounce check, to discard small shakes
        if(interruptCount <= debounceCount) {
            continue;
        }

        // Set the LED pin as a output, and enable the IR receiver
        DDRB = _BV(PIN_LED_ON) | _BV(PIN_IR_POWER);
        PORTB = _BV(PIN_UNUSED) | _BV(PIN_IR_POWER);

        // Turn on the IR Receiver
        irrecv.enableIRIn(); // Start the receiver

        // Play the EKG back, checking for an IR reception
        for(uint8_t loopCount = 0; loopCount < heartbeatReps; loopCount++) {
            playEKG();

            if (irrecv.decode(&results)) {
                for(uint8_t i = 0; i < 50; i++) {
                    setLEDs(i%2==0?255:0);
                    _delay_ms(10);
                }

                irrecv.resume(); // Receive the next value
            }
        }
    }
    
    return 0;   /* never reached */
}
