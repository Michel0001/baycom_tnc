// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.
// Based on work by Francesco Sacchi

#include "Arduino.h"
#include <string.h>
#include <ctype.h>
#include "AX25.h"
#include "HDLC.h"
#include "CRC-CCIT.h"
#include "AFSK.h"

void ax25_init(AX25Ctx *ctx, ax25_callback_t hook) {
  memset(ctx, 0, sizeof(*ctx));
  ctx->hook = hook;
  ctx->crc_in = ctx->crc_out = CRC_CCIT_INIT_VAL;
  ctx->ready_for_data = true;
}

static void ax25_decode(AX25Ctx *ctx) {
  if (ctx->hook) {
    ctx->hook(ctx);
  }
}

void ax25_poll(AX25Ctx *ctx) {
  int c;

  while ((c = afsk_getchar()) != EOF) {
    if (!ctx->escape && c == HDLC_FLAG) {
      if (ctx->frame_len >= AX25_MIN_FRAME_LEN) {
        if (ctx->crc_in == AX25_CRC_CORRECT) {
          ax25_decode(ctx);
        }
      }
      ctx->sync = true;
      ctx->crc_in = CRC_CCIT_INIT_VAL;
      ctx->frame_len = 0;
      continue;
    }

    if (!ctx->escape && c == HDLC_RESET) {
      ctx->sync = false;
      continue;
    }

    if (!ctx->escape && c == AX25_ESC) {
      ctx->escape = true;
      continue;
    }

    if (ctx->sync) {
      if (ctx->frame_len < AX25_MAX_FRAME_LEN) {
        ctx->buf[ctx->frame_len++] = c;
        ctx->crc_in = update_crc_ccit(c, ctx->crc_in);
      } else {
        ctx->sync = false;
      }
    }
    ctx->escape = false;
  }
}

static void ax25_putchar(AX25Ctx *ctx, uint8_t c)
{
  if (c == HDLC_FLAG || c == HDLC_RESET || c == AX25_ESC) afsk_putchar(AX25_ESC);
  ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
  afsk_putchar(c);
}

void ax25_sendRaw(AX25Ctx *ctx, void *_buf, size_t len) {
  ctx->ready_for_data = false;
  ctx->crc_out = CRC_CCIT_INIT_VAL;
  afsk_putchar(HDLC_FLAG);
  const uint8_t *buf = (const uint8_t *)_buf;
  while (len--) ax25_putchar(ctx, *buf++);

  uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
  uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
  ax25_putchar(ctx, crcl);
  ax25_putchar(ctx, crch);

  afsk_putchar(HDLC_FLAG);
  ctx->ready_for_data = true;
}
