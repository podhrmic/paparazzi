/*
 * Copyright (C) 2014 Michal Podhradsky, Senman, podhrad@pdx.edu
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

 /**
 * @file kitemill.c
 *
 * Module for datalogging and command reception
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "modules/loggers/kitemill.h"

static inline void kitemill_cksum(uint16_t packet_length, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );
static inline void kitemill_datalength(uint16_t idx, uint8_t *buf);

// Buffers
static uint8_t kitemill_txbuf[KITEMILL_BUFFER_SIZE];
static uint8_t kitemill_rxbuf[KITEMILL_BUFFER_SIZE];

struct KitemillLogger logger;

/**
 * Init
 */
void kitemill_init(void){
  memset(&kitemill_rxbuf, 0x00, sizeof(kitemill_rxbuf));
  memset(&kitemill_txbuf, 0x00, sizeof(kitemill_txbuf));

  logger.msg_available = FALSE;
  logger.rxcounter = 0;
  logger.txcounter = 0;
}

/**
 * Parse received char
 */
void kitemill_parse(uint8_t c){
  // TBD
}

/**
 * Parse whole message
 */
void kitemill_read_message(void){
  // TBD
}

/**
 * Event func
 */
void kitemill_event(void){
/*
    if (LoggerBuffer()) {
      ReadLoggerBuffer();
    }
    if (logger.msg_available) {
      kitemill_read_message();
    }
    logger.msg_available = FALSE;
*/
}

/**
 * Periodic
 */
void kitemill_periodic(void){
  static uint8_t cksum0, cksum1;
  static uint16_t idx;

  kitemill_txbuf[0] = KITEMILL_MSG0;
  kitemill_txbuf[1] = KITEMILL_MSG1;

  cksum0 = 0;
  cksum1 = 0;
  idx = KITEMILL_DATA_IDX;


// DEBUG
  kitemill_txbuf[idx] = 'H';
  idx++;
  kitemill_txbuf[idx] = 'E';
  idx++;
  kitemill_txbuf[idx] = 'L';
  idx++;
  kitemill_txbuf[idx] = 'L';
  idx++;
  kitemill_txbuf[idx] = 'O';
  idx++;
  kitemill_txbuf[idx] = 0x20;
  idx++;
  kitemill_txbuf[idx] = 'W';
  idx++;
  kitemill_txbuf[idx] = 'O';
  idx++;
  kitemill_txbuf[idx] = 'R';
  idx++;
  kitemill_txbuf[idx] = 'L';
  idx++;
  kitemill_txbuf[idx] = 'D';
  idx++;


  // fill in data length, uint16
  kitemill_datalength(idx-KITEMILL_DATA_IDX, kitemill_txbuf);

  // calculate checksum & send
  kitemill_cksum(idx, kitemill_txbuf, &cksum0, &cksum1);
  kitemill_txbuf[idx] = cksum0;
  idx++;
  kitemill_txbuf[idx] = cksum1;
  idx++;


  // transmit
  for (uint16_t k = 0; k<idx; k++){
    LoggerLink(Transmit(kitemill_txbuf[k]));
  }
}

/**
 * Calculate checksum
 *
 * @param size - data to be checksumed (it is effectively header+data)
 * @param buf - pointer to the buffer
 * @param cksum0 - first byte of cksum
 * @param cksum1 - second byte of cksum
 */
void kitemill_cksum(uint16_t size, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 ) {
  static uint8_t c0, c1;
  static uint16_t i;//, size;
  c0 = 0;
  c1 = 0;
  //size = packet_length - 2;

  //Start at byte five so the header is not part of the checksum
  for ( i = 5; i < size; i++ ) {
  	c0 += (uint8_t)buf[i];
    	c1 += c0;
  }

  //DEBUG
  c0 = '\r';
  c1 = '\n';

  *cksum0 = c0;
  *cksum1 = c1;
}

/**
 * Fill in data length (uint16)
 */
static inline void kitemill_datalength(uint16_t idx, uint8_t *buf){
  for (uint8_t i = 0; i<2; i++){
    buf[KITEMILL_SIZE_IDX+i] = 0xFF&(idx>>(i*8));
  }
}
