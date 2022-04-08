// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.

#include "AFSK.h"
#include "AX25.h"
#include "KISS.h"

Afsk modem;
AX25Ctx AX25;

static void ax25_callback(struct AX25Ctx *ctx) {
  kiss_messageCallback(ctx);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    uint8_t sbyte = (uint8_t)Serial.read();
    kiss_serialCallback(sbyte);
  }
}

void setup(void) {
  cli();
  // Computer KISS
  Serial.begin(9600);
  AFSK_init(&modem);
  ax25_init(&AX25, ax25_callback);
  kiss_init(&AX25, &modem);

  // Initialize modem
  modem.ts_last = micros();
  sei();
}

void loop(void) {
  ax25_poll(&AX25);
}
