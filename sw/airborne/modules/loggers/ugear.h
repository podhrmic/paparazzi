/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 */
 /**
 * @file ugear_blackbox_uart.h
 *
 * Module for UGEAR and datalogging
 *
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef UGEAR_H
#define UGEAR_H

#define UGEAR_MSG0 0x93
#define UGEAR_MSG1 0xE0

#define UGEAR_HEADER_GPS 0x42
#define UGEAR_HEADER_ATTITUDE 0x69
#define UGEAR_HEADER_RADIO 0x13
#define UGEAR_HEADER_STATUS 0x66
#define UGEAR_HEADER_DEBUG 0xAA

/* Message size is the size of data only */
#define UGEAR_MESSAGE_SIZE_ATTITUDE 50//44//50//193
#define UGEAR_MESSAGE_SIZE_GPS 62//56//62
#define UGEAR_MESSAGE_SIZE_RADIO 18//18
#define UGEAR_MESSAGE_SIZE_STATUS 289
#define UGEAR_MESSAGE_SIZE_DEBUG 46
#define UGEAR_MESSAGE_SIZE_STATUS_BYTE 38

#define UGEAR_DATA_INDEX 4

/* General includes for all boards (AgNav, AgPt fixedwing, AgPt rotorcraft */
#include "mcu_periph/uart.h"
#include "subsystems/gps.h"
#include "mcu_periph/sys_time_arch.h"
#include "state.h"
#include "led.h"

extern  uint32_t ugear_counter_status;
extern  uint32_t ugear_counter_gps;
extern  uint32_t ugear_counter_radio;
extern  uint32_t ugear_counter_debug;

void ugear_init(void);
void ugear_periodic(void); // main periodic
void ugear_debug(void); // debug
void ugear_send_gps(void); // GPS data, ~4Hz
void ugear_send_radio(void); // radio commands, ~55Hz
void ugear_send_attitude(void); // attitude from AgNav, 500Hz
void ugear_send_status(void); // AgPt status for logging, 512Hz
void send_buf(uint16_t size,uint8_t *_buf);
void ugear_cksum(uint16_t packet_length, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );
#endif /* UGEAR_H */
