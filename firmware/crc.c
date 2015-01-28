#include "crc.h"

uint8_t crc;

void resetCRC() {
  crc = 0;
}

uint8_t getCRC() {
  return crc;
}

void updateCRC(uint8_t data) {
  uint8_t i;

  crc = crc ^ data;
  for (i = 0; i < 8; i++)
  {
    if(crc & 0x01) {
      crc = (crc >> 1) ^ 0x8C;
    }
    else {
      crc >>= 1;
    }
  }
}

