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
 * @file xgear.h
 *
 * Module for Xgear datalogging and block smashing code
 *
 * Meant for RT Paparazzi only
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef XGEAR_H
#define XGEAR_H

#ifndef USE_CHIBIOS_RTOS
#error "Xgear module is for RT Paparazzi only!"
#endif

/* General includes for all boards (AgNav, AgPt fixedwing, AgPt rotorcraft */
#include "mcu_periph/uart.h"
#include "subsystems/gps.h"
#include "subsystems/imu.h"
#include "state.h"

/*
 * Message headers
 */
#define XGEAR_MSG0 0x93
#define XGEAR_MSG1 0xE0

#define XGEAR_HEADER_GPS 0x42
#define XGEAR_HEADER_ATTITUDE 0x69
#define XGEAR_HEADER_RADIO 0x13
#define XGEAR_HEADER_STATUS 0x66
#define XGEAR_HEADER_DEBUG 0xAA
#define XGEAR_HEADER_CODEBLOCK 0xBB

/*
 * Size of whole packet, including checksum
 *
 * This is the number that defines how many bytes
 * are going to be sent. This is also number that goes
 * to the header.
 *
 */
#define XGEAR_PACKET_SIZE_ATTITUDE 50
#define XGEAR_PACKET_SIZE_GPS 62
#define XGEAR_PACKET_SIZE_RADIO 18

#define XGEAR_PACKET_SIZE_STATUS 289
/* This number goes to message header, as MSG_SIZE is over 255 bytes */
#define XGEAR_PACKET_SIZE_STATUS_BYTE 38

#define XGEAR_PACKET_SIZE_DEBUG 46
#define XGEAR_PACKET_SIZE_CODEBLOCK 64

/* Data offset */
#define XGEAR_DATA_INDEX 4

/*
 * General functions
 */
void xgear_init(void);
void xgear_periodic(void); // main periodic
void xgear_cksum(uint16_t packet_length, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );

/*
 * Debug functions
 */
void xgear_send_debug(void); // debug

/*
 * AggiePilot functions
 */
void xgear_send_gps(void); // GPS data, ~4Hz
void xgear_send_radio(void); // radio commands, ~55Hz
void xgear_send_status(void); // AgPt status for logging, 512Hz
void xgear_send_codeblock(void); // Block smashing code, ~1Hz

/*
 * AggieNav functions
 */
void xgear_send_imu(void); // attitude from AgNav, 500Hz

#endif /* UGEAR_H */
