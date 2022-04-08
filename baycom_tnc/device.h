// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.

#ifndef DEVICE_CONFIGURATION
#define DEVICE_CONFIGURATION

#define m328p  0x01
#define m1284p 0x02
#define m644p  0x03

// CPU settings
#ifndef F_CPU
#define F_CPU 16000000
#endif

// Port settings
#define TCM3105_RX_PIN 2
#define TCM3105_TX_PIN 3
#define PTT_PIN 4
#define DCD_LED_PIN 13

// Default parameter
#define CONFIG_AFSK_PREAMBLE_LEN 150UL
#define CONFIG_AFSK_TRAILER_LEN 50UL
#endif
