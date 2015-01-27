#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

// Reset the CRC engine
void resetCRC();

// Update the CRC engine
void updateCRC(uint8_t data);

// Get the current value of the CRC engine
uint8_t getCRC();

#endif
