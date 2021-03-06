// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/microaprs
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "KISS.h"

static uint8_t serialBuffer[AX25_MAX_FRAME_LEN]; // Buffer for holding incoming serial data
AX25Ctx *ax25ctx;
Afsk *channel;
size_t frame_len;
bool IN_FRAME;
bool ESCAPE;
bool FLOWCONTROL;

uint8_t command = CMD_UNKNOWN;
unsigned long custom_preamble = CONFIG_AFSK_PREAMBLE_LEN;
unsigned long custom_tail = CONFIG_AFSK_TRAILER_LEN;

unsigned long slotTime = 200;
uint8_t p = 63;

void kiss_init(AX25Ctx *ax25, Afsk *afsk) {
  ax25ctx = ax25;
  channel = afsk;
  FLOWCONTROL = false;
}

void kiss_messageCallback(AX25Ctx *ctx) {
  Serial.write(FEND);
  Serial.write(0x00);
  for (unsigned i = 0; i < ctx->frame_len - 2; i++) {
    uint8_t b = ctx->buf[i];
    if (b == FEND) {
      Serial.write(FESC);
      Serial.write(TFEND);
    } else if (b == FESC) {
      Serial.write(FESC);
      Serial.write(TFESC);
    } else {
      Serial.write(b);
    }
  }
  Serial.write(FEND);
}

void kiss_csma(AX25Ctx *ctx, uint8_t *buf, size_t len) {
  bool sent = false;
  while (!sent) {
    if (!channel->hdlc.receiving) {
      uint8_t tp = rand() & 0xFF;
      if (tp < p) {
        ax25_sendRaw(ctx, buf, len);
        sent = true;
      } else {
        delay(slotTime);
      }
    } else {
      while (!sent && channel->hdlc.receiving) {
        // Continously poll the modem for data
        // while waiting, so we don't overrun
        // receive buffers
        ax25_poll(ax25ctx);

        if (channel->status != 0) {
          // If an overflow or other error
          // occurs, we'll back off and drop
          // this packet silently.
          channel->status = 0;
          sent = true;
        }
      }
    }
  }

  if (FLOWCONTROL) {
    while (!ctx->ready_for_data) {
      /* Wait */
    }
    Serial.write(FEND);
    Serial.write(CMD_READY);
    Serial.write(0x01);
    Serial.write(FEND);
  }
}

void kiss_serialCallback(uint8_t sbyte) {
  if (IN_FRAME && sbyte == FEND && command == CMD_DATA) {
    IN_FRAME = false;
    kiss_csma(ax25ctx, serialBuffer, frame_len);
  } else if (sbyte == FEND) {
    IN_FRAME = true;
    command = CMD_UNKNOWN;
    frame_len = 0;
  } else if (IN_FRAME && frame_len < AX25_MAX_FRAME_LEN) {
    // Have a look at the command byte first
    if (frame_len == 0 && command == CMD_UNKNOWN) {
      // MicroModem supports only one HDLC port, so we
      // strip off the port nibble of the command byte
      sbyte = sbyte & 0x0F;
      command = sbyte;
    } else if (command == CMD_DATA) {
      if (sbyte == FESC) {
        ESCAPE = true;
      } else {
        if (ESCAPE) {
          if (sbyte == TFEND) sbyte = FEND;
          if (sbyte == TFESC) sbyte = FESC;
          ESCAPE = false;
        }
        serialBuffer[frame_len++] = sbyte;
      }
    } else if (command == CMD_TXDELAY) {
      custom_preamble = sbyte * 10UL;
    } else if (command == CMD_TXTAIL) {
      custom_tail = sbyte * 10UL;
    } else if (command == CMD_SLOTTIME) {
      slotTime = sbyte * 10UL;
    } else if (command == CMD_P) {
      p = sbyte;
    } else if (command == CMD_READY) {
      if (sbyte == 0x00) {
        FLOWCONTROL = false;
      } else {
        FLOWCONTROL = true;
      }
    }

  }
}
