// Based on

// Copyright Mark Qvist / unsigned.io
// https://unsigned.io/projects/libaprs/
// https://unsigned.io/projects/microaprs/
//
// Licensed under GPL-3.0. For full info,
// read the LICENSE file.

// AND

// G7TAJ BayCom Packet KISS driver for RasPi - Copyright � 2019
// Original BayCom code Copyright � 2001-2019 John Wiseman G8BPQ

#include <string.h>
#include "AFSK.h"
#include "Arduino.h"

extern unsigned long custom_preamble;
extern unsigned long custom_tail;

Afsk *AFSK_modem;

// Forward declerations
int afsk_getchar(void);
void afsk_putchar(char c);
void AFSK_rx_isr();

void AFSK_hw_init(void) {
  // TX interrupt
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 1200 Hz increments
  OCR1A = CPU_FREQ / BITRATE - 1; // = 16000000 / 1200 - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // disable timer compare interrupt
  TIMSK1 &= ~(1 << OCIE1A);
  digitalWrite(TCM3105_TX_PIN, AFSK_modem->txOutput);
  pinMode(TCM3105_TX_PIN, OUTPUT);

  // RX interrupt
  pinMode(TCM3105_RX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TCM3105_RX_PIN), AFSK_rx_isr, CHANGE);

  // DCD Led
  pinMode(DCD_LED_PIN, OUTPUT);
  digitalWrite(DCD_LED_PIN, LOW);

  // PTT
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
}

void AFSK_init(Afsk *afsk) {
  // Allocate modem struct memory
  memset(afsk, 0, sizeof(*afsk));
  AFSK_modem = afsk;
  AFSK_modem->pllBitTime = BIT_TIME;
  AFSK_modem->txOutput = 0;
  // Initialise FIFO buffers
  fifo_init(&afsk->rxFifo, afsk->rxBuf, sizeof(afsk->rxBuf));
  fifo_init(&afsk->txFifo, afsk->txBuf, sizeof(afsk->txBuf));

  AFSK_hw_init();
}

static void AFSK_txStart(Afsk *afsk) {
  if (!afsk->sending) {
    afsk->bitstuffCount = 0;
    afsk->sending = true;
    digitalWrite(DCD_LED_PIN, HIGH);
    afsk->preambleLength = (custom_preamble * 1000) / (8 * BIT_TIME);
    digitalWrite(PTT_PIN, HIGH);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    afsk->tailLength = (custom_tail * 1000) / (8 * BIT_TIME);
  }
}

void afsk_putchar(char c) {
  AFSK_txStart(AFSK_modem);
  while (fifo_isfull_locked(&AFSK_modem->txFifo)) {
    /* Wait */
  }
  fifo_push_locked(&AFSK_modem->txFifo, c);
}

int afsk_getchar(void) {
  if (fifo_isempty_locked(&AFSK_modem->rxFifo)) {
    return EOF;
  } else {
    return fifo_pop_locked(&AFSK_modem->rxFifo);
  }
}

void AFSK_transmit(char *buffer, size_t size) {
  fifo_flush(&AFSK_modem->txFifo);
  int i = 0;
  while (size--) {
    afsk_putchar(buffer[i++]);
  }
}

static bool hdlcParse(Hdlc *hdlc, bool bit, FIFOBuffer *fifo) {
  // Initialise a return value. We start with the
  // assumption that all is going to end well :)
  bool ret = true;

  // Bitshift our byte of demodulated bits to
  // the left by one bit, to make room for the
  // next incoming bit
  hdlc->demodulatedBits <<= 1;
  // And then put the newest bit from the
  // demodulator into the byte.
  hdlc->demodulatedBits |= bit ? 1 : 0;

  // Now we'll look at the last 8 received bits, and
  // check if we have received a HDLC flag (01111110)
  if (hdlc->demodulatedBits == HDLC_FLAG) {
    // If we have, check that our output buffer is
    // not full.
    if (!fifo_isfull(fifo)) {
      // If it isn't, we'll push the HDLC_FLAG into
      // the buffer and indicate that we are now
      // receiving data. For bling we also turn
      // on the RX LED.
      fifo_push(fifo, HDLC_FLAG);
      hdlc->receiving = true;
      if (hdlc->dcd_count < DCD_MIN_COUNT) {
        hdlc->dcd = false;
        hdlc->dcd_count++;
      } else {
        hdlc->dcd = true;
      }
    } else {
      // If the buffer is full, we have a problem
      // and abort by setting the return value to
      // false and stopping the here.

      ret = false;
      hdlc->receiving = false;
      hdlc->dcd = false;
      hdlc->dcd_count = 0;
    }

    // Everytime we receive a HDLC_FLAG, we reset the
    // storage for our current incoming byte and bit
    // position in that byte. This effectively
    // synchronises our parsing to  the start and end
    // of the received bytes.
    hdlc->currentByte = 0;
    hdlc->bitIndex = 0;
    return ret;
  }

  // Check if we have received a RESET flag (01111111)
  // In this comparison we also detect when no transmission
  // (or silence) is taking place, and the demodulator
  // returns an endless stream of zeroes. Due to the NRZ
  // coding, the actual bits send to this function will
  // be an endless stream of ones, which this AND operation
  // will also detect.
  if ((hdlc->demodulatedBits & HDLC_RESET) == HDLC_RESET) {
    // If we have, something probably went wrong at the
    // transmitting end, and we abort the reception.
    hdlc->receiving = false;
    hdlc->dcd = false;
    hdlc->dcd_count = 0;
    return ret;
  }

  // Check the DCD status and set RX LED appropriately
  if (hdlc->dcd) {
    digitalWrite(DCD_LED_PIN, HIGH);
  } else {
    digitalWrite(DCD_LED_PIN, LOW);
  }

  // If we have not yet seen a HDLC_FLAG indicating that
  // a transmission is actually taking place, don't bother
  // with anything.
  if (!hdlc->receiving) {
    hdlc->dcd = false;
    hdlc->dcd_count = 0;
    return ret;
  }

  // First check if what we are seeing is a stuffed bit.
  // Since the different HDLC control characters like
  // HDLC_FLAG, HDLC_RESET and such could also occur in
  // a normal data stream, we employ a method known as
  // "bit stuffing". All control characters have more than
  // 5 ones in a row, so if the transmitting party detects
  // this sequence in the _data_ to be transmitted, it inserts
  // a zero to avoid the receiving party interpreting it as
  // a control character. Therefore, if we detect such a
  // "stuffed bit", we simply ignore it and wait for the
  // next bit to come in.
  //
  // We do the detection by applying an AND bit-mask to the
  // stream of demodulated bits. This mask is 00111111 (0x3f)
  // if the result of the operation is 00111110 (0x3e), we
  // have detected a stuffed bit.
  if ((hdlc->demodulatedBits & 0x3f) == 0x3e)
    return ret;

  // If we have an actual 1 bit, push this to the current byte
  // If it's a zero, we don't need to do anything, since the
  // bit is initialized to zero when we bitshifted earlier.
  if (hdlc->demodulatedBits & 0x01)
    hdlc->currentByte |= 0x80;

  // Increment the bitIndex and check if we have a complete byte
  if (++hdlc->bitIndex >= 8) {
    // If we have a HDLC control character, put a AX.25 escape
    // in the received data. We know we need to do this,
    // because at this point we must have already seen a HDLC
    // flag, meaning that this control character is the result
    // of a bitstuffed byte that is equal to said control
    // character, but is actually part of the data stream.
    // By inserting the escape character, we tell the protocol
    // layer that this is not an actual control character, but
    // data.
    if ((hdlc->currentByte == HDLC_FLAG ||
         hdlc->currentByte == HDLC_RESET ||
         hdlc->currentByte == AX25_ESC)) {
      // We also need to check that our received data buffer
      // is not full before putting more data in
      if (!fifo_isfull(fifo)) {
        fifo_push(fifo, AX25_ESC);
      } else {
        // If it is, abort and return false
        hdlc->receiving = false;
        hdlc->dcd = false;
        hdlc->dcd_count = 0;
        digitalWrite(DCD_LED_PIN, LOW);
        ret = false;
      }
    }

    // Push the actual byte to the received data FIFO,
    // if it isn't full.
    if (!fifo_isfull(fifo)) {
      fifo_push(fifo, hdlc->currentByte);
    } else {
      // If it is, well, you know by now!
      hdlc->receiving = false;
      hdlc->dcd = false;
      hdlc->dcd_count = 0;
      digitalWrite(DCD_LED_PIN, LOW);
      ret = false;
    }

    // Wipe received byte and reset bit index to 0
    hdlc->currentByte = 0;
    hdlc->bitIndex = 0;

  } else {
    // We don't have a full byte yet, bitshift the byte
    // to make room for the next bit
    hdlc->currentByte >>= 1;
  }

  return ret;
}

void AFSK_rx_isr() {
  uint32_t ts_current = micros();
  // Interval is time since previous input transition
  uint32_t interval = ts_current - AFSK_modem->ts_last;
  AFSK_modem->ts_last = ts_current;

  uint32_t bitTime = AFSK_modem->pllBitTime;
  uint32_t reasonableError = bitTime * 20 / 100;     // 20% max
  uint32_t bits = 0;

  if (interval < bitTime - reasonableError)
    return;                                         // Short sample - assume noise for now
  // at some point try to ignore short spikes

  if (interval > (bitTime + reasonableError) * 7) {
    interval = bitTime * 10;                        // Long idles - 9 ones is plenty to abort and reset frame
  }

  while (interval > bitTime + reasonableError) {
    if (!hdlcParse(&AFSK_modem->hdlc, 1, &AFSK_modem->rxFifo)) {
      AFSK_modem->status |= 1;
      if (fifo_isfull(&AFSK_modem->rxFifo)) {
        fifo_flush(&AFSK_modem->rxFifo);
        AFSK_modem->status = 0;
      }
    }
    interval -= bitTime;
    bits++;
  }

  if (!hdlcParse(&AFSK_modem->hdlc, 0, &AFSK_modem->rxFifo)) {
    AFSK_modem->status |= 1;
    if (fifo_isfull(&AFSK_modem->rxFifo)) {
      fifo_flush(&AFSK_modem->rxFifo);
      AFSK_modem->status = 0;
    }
  }
  bits++;

  interval -= bitTime;

  interval /= bits;								   // Error per bit

  // Use any residue to converge DPLL, but make sure not spurious transition before updating DPLL
  // Can be between - (BitTime - Reasonable) and Reasonable. Can't be greater, or we would have extraced another 1 bit

  if ((interval + reasonableError) < 0) {
    // Do we try to ignore spurious? Maybe once synced.
    // for now, ignore, but dont use to converge dpll
    return;
  }

  if (interval) {
    AFSK_modem->pllBitTime += interval / 16;
    if (AFSK_modem->pllBitTime > MAX_BIT_TIME) {
      AFSK_modem->pllBitTime = MAX_BIT_TIME;
    } else if (AFSK_modem->pllBitTime < MIN_BIT_TIME) {
      AFSK_modem->pllBitTime = MIN_BIT_TIME;
    }
  }
}

// TX interrupt
ISR(TIMER1_COMPA_vect) {
  if (AFSK_modem->txBit == 0) {
    if (fifo_isempty(&AFSK_modem->txFifo) && AFSK_modem->tailLength == 0) {
      // disable timer compare interrupt
      TIMSK1 &= ~(1 << OCIE1A);
      digitalWrite(PTT_PIN, LOW);
      AFSK_modem->sending = false;
      digitalWrite(DCD_LED_PIN, LOW);
      return 0;
    } else {
      if (!AFSK_modem->bitStuff) AFSK_modem->bitstuffCount = 0;
      AFSK_modem->bitStuff = true;
      if (AFSK_modem->preambleLength == 0) {
        if (fifo_isempty(&AFSK_modem->txFifo)) {
          AFSK_modem->tailLength--;
          AFSK_modem->currentOutputByte = HDLC_FLAG;
        } else {
          AFSK_modem->currentOutputByte = fifo_pop(&AFSK_modem->txFifo);
        }
      } else {
        AFSK_modem->preambleLength--;
        AFSK_modem->currentOutputByte = HDLC_FLAG;
      }
      if (AFSK_modem->currentOutputByte == AX25_ESC) {
        if (fifo_isempty(&AFSK_modem->txFifo)) {
          // disable timer compare interrupt
          TIMSK1 &= ~(1 << OCIE1A);
          digitalWrite(PTT_PIN, LOW);
          AFSK_modem->sending = false;
          digitalWrite(DCD_LED_PIN, LOW);
          return 0;
        } else {
          AFSK_modem->currentOutputByte = fifo_pop(&AFSK_modem->txFifo);
        }
      } else if (AFSK_modem->currentOutputByte == HDLC_FLAG || AFSK_modem->currentOutputByte == HDLC_RESET) {
        AFSK_modem->bitStuff = false;
      }
    }
    AFSK_modem->txBit = 0x01;
  }

  if (AFSK_modem->bitStuff && AFSK_modem->bitstuffCount >= BIT_STUFF_LEN) {
    AFSK_modem->bitstuffCount = 0;
    AFSK_modem->txOutput = !AFSK_modem->txOutput;
    digitalWrite(TCM3105_TX_PIN, AFSK_modem->txOutput);
  } else {
    if (AFSK_modem->currentOutputByte & AFSK_modem->txBit) {
      AFSK_modem->bitstuffCount++;
    } else {
      AFSK_modem->bitstuffCount = 0;
      AFSK_modem->txOutput = !AFSK_modem->txOutput;
      digitalWrite(TCM3105_TX_PIN, AFSK_modem->txOutput);
    }
    AFSK_modem->txBit <<= 1;
  }
}
