// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.
// CRC-CCIT Implementation based on work by Francesco Sacchi

#ifndef CRC_CCIT_H
#define CRC_CCIT_H

#include <stdint.h>
#include <avr/pgmspace.h>

#define CRC_CCIT_INIT_VAL ((uint16_t)0xFFFF)

extern const uint16_t crc_ccit_table[256];

inline uint16_t update_crc_ccit(uint8_t c, uint16_t prev_crc) {
  return (prev_crc >> 8) ^ pgm_read_word(&crc_ccit_table[(prev_crc ^ c) & 0xff]);
}


#endif
