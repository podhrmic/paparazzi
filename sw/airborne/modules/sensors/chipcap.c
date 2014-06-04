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

/** @file modules/sensors/airspeed_ms45xx.c
 *  Airspeed driver for the MS45xx
 */


#include "modules/sensors/chipcap.h"

#include "mcu_periph/sys_time_arch.h"

uint8_t chipcap_response;
uint16_t chipcap_eeprom;
struct i2c_transaction chipcap_trans;
uint8_t chipcap_trans_status;
uint8_t chipcap_status;

uint16_t rh_data;
uint16_t tmp_data;
float rh;
float chipcap_tmp;

void chipcap_init(void) {
  rh = 0;
  chipcap_tmp = 0;

/*
  chipcap_trans.buf[0] = 0x16;
  chipcap_trans.buf[1] = 0x00;
  chipcap_trans.buf[2] = 0x00;
  chipcap_status =  CHIPCAP_UNINIT;
  chipcap_trans_status = chipcap_trans.status;
  i2c_transmit(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 3);
*/
}

void chipcap_periodic(void) {
i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 4);

/*
  switch (chipcap_status) {
    case CHIPCAP_UNINIT:
      // send cmd mode request again
      chipcap_trans.buf[0] = 0xA0;
      chipcap_trans.buf[1] = 0x00;
      chipcap_trans.buf[2] = 0x00;
      i2c_transmit(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 3);
      //i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 1);
      break;
    case CHIPCAP_INIT:
      // EEprom read
      chipcap_trans.buf[0] = 0x1E;
      chipcap_trans.buf[1] = 0x00;
      chipcap_trans.buf[2] = 0x00;
      i2c_transmit(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 3);
      break;
    case CHIPCAP_READ_DF:
      // read the EEPROM
      chipcap_trans.buf[0] = 0x00;
      chipcap_trans.buf[1] = 0x00;
      chipcap_trans.buf[2] = 0x00;
      //i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 3);
      i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 4);
      break;
    default:
      break;
  }
*/

}

void chipcap_event(void) {
  // Check if transaction is succesfull 
  if(chipcap_trans.status == I2CTransSuccess) {
    chipcap_response = (chipcap_trans.buf[0] >> 6);

    rh_data = (uint16_t)(((chipcap_trans.buf[0] << 8) | chipcap_trans.buf[1]) & 0x3FFF);
    rh = (float)rh_data/16384.0*100.0;

    tmp_data = (uint16_t) ( (chipcap_trans.buf[2] << 6) | (chipcap_trans.buf[3]>>2) );
    chipcap_tmp = ((float)tmp_data/16384.0*165.0)-40.0;


/*
    switch (chipcap_status) {
      case CHIPCAP_UNINIT:
        chipcap_response = chipcap_trans.buf[0];
        //chipcap_trans.buf[0] = 0x00;
        i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 1);
        chipcap_status = CHIPCAP_INIT;
        break;
      case CHIPCAP_INIT:
        chipcap_response = chipcap_trans.buf[0];
        //chipcap_trans.buf[0] = 0x00;
        i2c_receive(&CHIPCAP_I2C_DEV, &chipcap_trans, CHIPCAP_I2C_ADDR, 3);
        chipcap_status = CHIPCAP_READ_DF;
        break;
      case CHIPCAP_READ_DF:
        // read the EEPROM
        chipcap_response = chipcap_trans.buf[0];
        chipcap_eeprom = (uint16_t)((chipcap_trans.buf[1] << 8) | chipcap_trans.buf[2]);
        //chipcap_status = CHIPCAP_WRITE_CMD;
        //chipcap_eeprom = (uint16_t)((chipcap_trans.buf[1] << 8) | chipcap_trans.buf[2]);
        //chipcap_status = CHIPCAP_INIT;
        break;
      default:
        break;
    }
*/

  }
  chipcap_trans.status = I2CTransDone;
  chipcap_trans_status = chipcap_trans.status;
}

