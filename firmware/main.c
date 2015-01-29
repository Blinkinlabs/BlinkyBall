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
#include "IRremote.h"
#include "blinkyball.h"
#include "crc.h"
#include "seven_cycles_corrected.h"

// Bounce sensitivity, in interrupt counts. Increase to decrease sensitivity
volatile uint8_t debounceCount;

// Heartbeat speed, in 10ths of a millisecond per sample
// To convert from BPM to this constant, use the following formula:
// ((60/BPM/SAMPLES*7)) × 10000
// or : 8458/bpm
// where SAMPLES is equal to the heartbeat sample count for seven cycles (497)
// Examples:
// 60bpm: 140
// 150bpm: 56
volatile uint8_t heartbeatSpeed;

// Number of times the 7-beat pattern is repeated
volatile uint8_t repeatCount;

// Counter for interrupt-based debounce routine
volatile uint8_t interruptCount;

// IR Receiver 
extern decode_results_t results;

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

// Validate the configuration rates. Do this when they are loaded and when they are changed.
inline void validateRates() {
    if(repeatCount > REPEAT_COUNT_MAXIMUM) {
        repeatCount = REPEAT_COUNT_MAXIMUM;
    }
}

// Store the configuration rates in EEPROM
inline void saveRates() {
    eeprom_write_byte((uint8_t*)DEBOUNCE_COUNT_ADDRESS,   DEBOUNCE_COUNT_DEFAULT);
    eeprom_write_byte((uint8_t*)REPEAT_COUNT_ADDRESS,     REPEAT_COUNT_DEFAULT);
    eeprom_write_byte((uint8_t*)HEARTBEAT_SPEED_ADDRESS,  HEARTBEAT_SPEED_DEFAULT);
    eeprom_write_byte((uint8_t*)MAGIC_HEADER_ADDRESS,     MAGIC_HEADER_VALUE);
}

// Load the configuration rates from EEPROM, or create from default if they weren't
// already initialized
inline void loadRates() {
    // If the EEPROM wasn't initialized, do so now
    if(eeprom_read_byte((uint8_t*)MAGIC_HEADER_ADDRESS) != MAGIC_HEADER_VALUE) {
        debounceCount  = DEBOUNCE_COUNT_DEFAULT;
        repeatCount    = REPEAT_COUNT_DEFAULT;
        heartbeatSpeed = HEARTBEAT_SPEED_DEFAULT;
        saveRates();
    }

    debounceCount  = eeprom_read_byte((uint8_t*)DEBOUNCE_COUNT_ADDRESS);
    repeatCount    = eeprom_read_byte((uint8_t*)REPEAT_COUNT_ADDRESS);
    heartbeatSpeed = eeprom_read_byte((uint8_t*)HEARTBEAT_SPEED_ADDRESS);
}

// Play back one loop of the heartbeat signal
// Duration: nominal 7 seconds at 60 BPM
void playEKG() {
    uint8_t sample;

    for(uint16_t loop = 0; loop < repeatCount; loop++) {
        for(uint16_t position = 0; position < EKG_DATA_LENGTH; position++) {
            sample = pgm_read_byte(&ekgData[position]);

            setLEDs(sample);

            // make a delay based on the heartbeat speed
            for(uint16_t count = 0; count < heartbeatSpeed; count++) {
                _delay_us(100);
            }
        }
    }

    setLEDs(0);
}

// Listen for an IR command
// Duration: 4+ seconds
void monitorIR() {
    setLEDs(0); // Make sure that the LEDs are off to prevent interference
    enableIRIn(); // Start the receiver
    
    for(uint32_t irLoop = 0; irLoop < IR_MONITOR_TIME; irLoop++) {
        _delay_ms(90);

        // If we didn't get an NEC command, bail
        if (!decodeIR()) {
            continue;
        }

        uint8_t speed =         (results.value >> 24) & 0xFF;
        uint8_t repeats =       (results.value >> 16) & 0xFF;
        uint8_t sensitivity =   (results.value >>  8) & 0xFF;

        resetCRC();
        updateCRC(speed);
        updateCRC(repeats);
        updateCRC(sensitivity);

        // If the CRC is valid, store the results.
        if(getCRC() == (results.value & 0xFF)) {
            // Update the rates both in working memory and in the EEPROM
            heartbeatSpeed = speed;
            repeatCount = repeats;
            debounceCount = sensitivity;
            validateRates();
            saveRates();

            // Flash the LEDs to indicate IR reception
            for(uint8_t i = 0; i < 5; i++) {
                setLEDs(i%2==0?20:0);
                _delay_ms(2);
            }
            setLEDs(0);

            // And reset our timeout counter
            irLoop = 0; 
        }
        
        resumeIR(); // Receive the next value
        
    }
}


// program entry point
// Kinda sad that the production devices will only run this once T_T
int main(void) {

    // Change the clock to 8MHz
    __asm__ volatile (
        "st Z,%1" "\n\t"
        "st Z,%2"
        : :
        "z" (&CLKPR),
        "r" ((uint8_t) (1<<CLKPCE)),
        "r" ((uint8_t) 1)  // new CLKPR value 0=8MHz, 1=4MHz, 2=2MHz, 3=1MHz)
    );

    bitSet(ACSR, ACD);          // Disable the analog comparitor
    bitClear(ADCSRA, ADEN);     // Disable the ADC
    // Note: Brown out detection disabled by fuse setting
    // Note: Watchdog disabled by fuse setting

    // Disable clicks to peripherals that we aren't using
    // (This saves power in run mode)
    PRR |= _BV(PRUSI) | _BV(ADC);

    // Configure the sleep mode and wakeup interrupt
    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));    // INT0 on low level
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // Use power down mode for sleep

    // Set up timer1 to do PWM output to the LEDs (OC1A)
    OCR1C = 0xFF;
    TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(CS10);

    // Load configuration from EEPROM and validate it
    loadRates();
    validateRates();

    // Main loop. The first thing that we do is turn off peripherals that we don't need,
    // then we go to sleep. When woken up, run a quick debounce routine to determine if we
    // should go into LED display mode, and at the completion go back to sleep.
    for(;;){

        // Disable the IR timer
        disableIRIn();

        // Turn off the LED outputs before disabling the pins
        setLEDs(0);
        _delay_ms(1);

        // Set all I/O pins to input, and pull-up resistors for floating pins
        DDRB = 0;
        PORTB = _BV(PIN_UNUSED);

        // Go to sleep
        sleep();

        // Do a quick debounce check, to discard small shakes
        if(interruptCount < debounceCount) {
            continue;
        }

        // Set the LED pin as a output, and enable the IR receiver
        DDRB = _BV(PIN_LED_ON) | _BV(PIN_IR_POWER);
        PORTB = _BV(PIN_UNUSED) | _BV(PIN_IR_POWER);

#ifdef TIMING_CHECK
        bitSet(DDRB, PIN_UNUSED);
        bitClear(PORTB, PIN_UNUSED);
#endif

        // First play the heartbeat pattern back
#ifdef TIMING_CHECK
        bitSet(PORTB, PIN_UNUSED);
#endif
        playEKG();
#ifdef TIMING_CHECK
        bitClear(PORTB, PIN_UNUSED);
#endif

        // Finally, monitor the IR input
#ifdef TIMING_CHECK
        bitSet(PORTB, PIN_UNUSED);
#endif
        monitorIR();

#ifdef TIMING_CHECK
        bitClear(PORTB, PIN_UNUSED);
#endif
    }
    
    return 0;   /* never reached */
}
