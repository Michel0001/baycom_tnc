// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.

#ifndef AFSK_H
#define AFSK_H

#include <Arduino.h>
#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "FIFO.h"
#include "HDLC.h"

#define CPU_FREQ F_CPU

#define CONFIG_AFSK_RX_BUFLEN 64
#define CONFIG_AFSK_TX_BUFLEN 64

#define BITRATE    1200
#define BIT_TIME 1000000 / BITRATE
#define MAX_BIT_TIME 1.05 * BIT_TIME
#define MIN_BIT_TIME 0.95 * BIT_TIME
#define BIT_STUFF_LEN 5

#define DCD_MIN_COUNT 6

typedef struct Hdlc
{
  uint8_t demodulatedBits;
  uint8_t bitIndex;
  uint8_t currentByte;
  bool receiving;
  bool dcd;
  uint8_t dcd_count;
} Hdlc;

typedef struct Afsk
{
  // General values
  Hdlc hdlc;                              // We need a link control structure
  uint16_t preambleLength;                // Length of sync preamble
  uint16_t tailLength;                    // Length of transmission tail

  // Modulation values
  uint8_t currentOutputByte;              // Current byte to be modulated
  uint8_t txBit;                          // Mask of current modulated bit
  bool bitStuff;                          // Whether bitstuffing is allowed
  uint8_t bitstuffCount;                  // Counter for bit-stuffing
  bool txOutput;                           // State of TX output

  FIFOBuffer txFifo;                      // FIFO for transmit data
  uint8_t txBuf[CONFIG_AFSK_TX_BUFLEN];   // Actial data storage for said FIFO

  volatile bool sending;                  // Set when modem is sending

  // Demodulation values
  FIFOBuffer rxFifo;                      // FIFO for received data
  uint8_t rxBuf[CONFIG_AFSK_RX_BUFLEN];   // Actual data storage for said FIFO

  uint32_t ts_last;
  uint32_t pllBitTime;
  bool dcd;

  volatile int status;                    // Status of the modem, 0 means OK

} Afsk;

void AFSK_init(Afsk *afsk);
void AFSK_transmit(char *buffer, size_t size);
void AFSK_poll(Afsk *afsk);

void afsk_putchar(char c);
int afsk_getchar(void);

#endif
