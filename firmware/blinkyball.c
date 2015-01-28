#include "blinkyball.h"

#if 0
void pulseOut(uint8_t data) {
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

#if 0
// Meausre the battery voltage under load
// @return Battery voltage, in counts (255=5V, 127=2.5V, etc)
uint8_t measureBattery() {
    // Select Vcc as voltage reference, Vbg as input, and left-justify result
    ADMUX = _BV(ADLAR) | _BV(MUX3) | _BV(MUX2);

    // Turn on the ADC, disable interrupts, and set the prescaler to /32
    ADCSRA = _BV(ADEN) | _BV(ADPS2);
   
    // Turn on the LEDs
    setLEDs(255);

    // Wait 1ms as per the datasheet recommendation before sampling Vbg
    _delay_ms(1);

    // Trigger a conversion
    ADCSRA |= _BV(ADSC);

    // Wait for conversion to finish
    while(ADCSRA & _BV(ADSC)) {}

    // Turn off the LEDs
    setLEDs(0);

    // Sample ADCH
    uint8_t measured = ADCH;

    // Disable ADC
    ADCSRA = 0;

    // The measured value is equal to:
    // measured = Vbg/Vcc*counts
    // so Vcc = Vbg*counts/measured
    // to put it in counts (0-255), multiply by 255/5V:
    // Vcc = Vbg*counts/measured*255/5
    // Vbg is 1.1V nominally, so this can be reduced to:
    // 14305 / measured
    return 14305 / measured;
}
#endif
