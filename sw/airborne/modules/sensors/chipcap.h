/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi

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

/** @file modules/sensors/airspeed_ms45xx.h
 *  Airspeed driver for the MS45xx
 */

#ifndef CHIPCAP_H
#define CHIPCAP_H

#include "std.h"
#include "mcu_periph/i2c.h"

/** Default I2C device
 */
#ifndef CHIPCAP_I2C_DEV
#define CHIPCAP_I2C_DEV i2c1
#endif

/** Sensor I2C slave address (defaults 0x50) */
#define CHIPCAP_I2C_ADDR 0x50

enum chipcap_stat{
  CHIPCAP_UNINIT,
  CHIPCAP_INIT,
  CHIPCAP_READ_DF,
  CHIPCAP_WAIT
};

extern uint16_t chipcap_eeprom;
extern uint8_t chipcap_trans_status;
extern uint8_t chipcap_status;
extern uint8_t chipcap_response;

extern uint16_t rh_data;
extern uint16_t tmp_data;

extern float rh;
extern float chipcap_tmp;

extern void chipcap_init(void);
extern void chipcap_periodic(void);
extern void chipcap_event(void);

#endif


