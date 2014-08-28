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

#define DEBUG 0

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
    if (LoggerBuffer()) {
      ReadLoggerBuffer();
    }
    if (logger.msg_available) {
      kitemill_read_message();
    }
    logger.msg_available = FALSE;
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

  kitemill_txbuf[2] = 0;
  kitemill_txbuf[3] = 0;
  idx = KITEMILL_DATA_IDX;


  // DEBUG
if (DEBUG) {
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
}
else {
  static float timestamp;

  // System time [s]
// TODO // Float doesnt return proper values
// FIXME
  timestamp = get_sys_time_usec()/1000000.0;//get_sys_time_float(); 
  memcpy(&kitemill_txbuf[idx], &timestamp, sizeof(float));
  idx += sizeof(float);

  // Number of actuators, uint8_t
  kitemill_txbuf[idx] = (uint8_t) ACTUATORS_NB;
  idx++;

  // Actuators value, int16
  memcpy(&kitemill_txbuf[idx], actuators, ACTUATORS_NB*sizeof(int16_t));
  idx += ACTUATORS_NB*sizeof(uint16_t);


  //Baro, float, hPa
  memcpy(&kitemill_txbuf[idx], &ms5611_fbaroms, sizeof(float));
  idx += sizeof(float);


  //Attitude, float, [rads]
  struct FloatEulers* att = stateGetNedToBodyEulers_f();
  // phi
  memcpy(&kitemill_txbuf[idx], &(att->phi), sizeof(float));
  idx += sizeof(float);
  //psi
  memcpy(&kitemill_txbuf[idx], &(att->psi), sizeof(float));
  idx += sizeof(float);
  //theta
  memcpy(&kitemill_txbuf[idx], &(att->theta), sizeof(float));
  idx += sizeof(float);


  // Accel (imu-frame), float, [m/s^-2]
  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel)
  memcpy(&kitemill_txbuf[idx], &accel_float.x, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &accel_float.y, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &accel_float.z, sizeof(float));
  idx += sizeof(float);


  // Rates (imu frame), float, [rad/s]
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);
  memcpy(&kitemill_txbuf[idx], &gyro_float.p, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &gyro_float.q, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &gyro_float.r, sizeof(float));
  idx += sizeof(float);


  // Mag (imu frame), float [!]
  struct FloatVect3 mag_float;
  MAGS_FLOAT_OF_BFP(mag_float, imu.mag);
  memcpy(&kitemill_txbuf[idx], &mag_float.x, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &mag_float.y, sizeof(float));
  idx += sizeof(float);
  memcpy(&kitemill_txbuf[idx], &mag_float.z, sizeof(float));
  idx += sizeof(float);


  // GPS data
  //GPS ECEFCORD position, int32
  for (uint8_t i = 0;i<4;i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_pos.x>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_pos.y>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_pos.z>>(i*8));
    idx++;
  }
  //GPS LLA, int32
  for (uint8_t i = 0;i<4;i++){
    kitemill_txbuf[idx] = 0xFF&(gps.lla_pos.lat>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.lla_pos.lon>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.lla_pos.alt>>(i*8));
    idx++;
  }
  //GPS hmsl, int32
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.hmsl>>(i*8));
    idx++;
  }
  //GPS ECEFCORD speed, int32
  for (uint8_t i = 0;i<4;i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_vel.x>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_vel.y>>(i*8));
    idx++;
  }
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.ecef_vel.z>>(i*8));
    idx++;
  }
  //GPS pacc, uint32
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.pacc>>(i*8));
    idx++;
  }
  //GPS sacc, uint32
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.sacc>>(i*8));
    idx++;
  }
  //GPS tow, uint32
  for (uint8_t i = 0; i<4; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.tow>>(i*8));
    idx++;
  }
  //GPS pdop, uint16
  for (uint8_t i = 0; i<2; i++){
    kitemill_txbuf[idx] = 0xFF&(gps.pdop>>(i*8));
    idx++;
  }
  //GPS fix, uint8
  kitemill_txbuf[idx] = gps.fix;
  idx++;

  //GPS num_sv, uint8
  kitemill_txbuf[idx] = gps.num_sv;
  idx++;
}

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
  for ( i = KITEMILL_DATA_IDX; i < size; i++ ) {
  	c0 += (uint8_t)buf[i];
    	c1 += c0;
  }

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
