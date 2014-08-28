/*
 * driver for ADC AD7997 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#include "modules/adcs/ad7998.h"

struct i2c_transaction ad7998_trans;
uint8_t ad7998_status;
uint8_t ad7998_trans_status;


float ad7998_ch1;
float ad7998_ch2;
float ad7998_ch3;
float ad7998_ch4;
float ad7998_ch5;
float ad7998_ch6;
float ad7998_ch7;
float ad7998_ch8;

float ad7998_channels[8];


uint16_t ad7998_data;
uint16_t ad7998_raw;
uint8_t ad7998_ch_id;
uint8_t ad7998_flag;
float ad7998_voltage;

uint8_t ad7998_addr;


/**
 * Increments channel address for the address pointer register
 */
void increment_channel(uint8_t* addr) {
  *addr += 0x10;
  if (*addr == 0) *addr = 0x80;
}


/**
 * Init variables
 */
void ad7998_init(void) {
  ad7998_ch1 = 0;
  ad7998_ch2 = 0;
  ad7998_ch3 = 0;
  ad7998_ch4 = 0;
  ad7998_ch5 = 0;
  ad7998_ch6 = 0;
  ad7998_ch7 = 0;
  ad7998_ch8 = 0;

  ad7998_data = 0;
  ad7998_raw = 0;
  ad7998_ch_id = 0;
  ad7998_flag = 0;
  ad7998_voltage = 0;


  memset(ad7998_channels,0,8);
  ad7998_trans.status = I2CTransDone;
  ad7998_trans_status = ad7998_trans.status;

  ad7998_addr = 0x80;
  ad7998_status = AD7998_INIT;
}


/**
 * Periodic request for data
 */
void ad7998_periodic(void){
/*
  if(ad7998_trans.status == I2CTransDone) {
    switch (ad7998_status) {
      case AD7998_INIT:
        // request data
        ad7998_trans.buf[0] = 0x80;
        i2c_transmit(&AD7998_I2C_DEV, &ad7998_trans, AD7998_ADDR, 1);
        break;
      default:
        break;
    }
  }
*/
        ad7998_trans.buf[0] = ad7998_addr;
        //i2c_transmit(&AD7998_I2C_DEV, &ad7998_trans, AD7998_ADDR, 1);
        i2c_transceive(&AD7998_I2C_DEV, &ad7998_trans, AD7998_ADDR, 1, 2);
        increment_channel(&ad7998_addr);
}


/**
 * Process results
 */
void ad7998_event(void){
  if(ad7998_trans.status == I2CTransSuccess) {
/*
    switch (ad7998_status) {
      case AD7998_REQ:
        // conversion requested, read results
        ad7998_trans.buf[0] = 0;
        ad7998_trans.buf[1] = 0;
        i2c_receive(&AD7998_I2C_DEV, &ad7998_trans, AD7998_ADDR, 2);
        ad7998_status = AD7998_READ;
        break;
      case AD7998_READ:
        // data acquired
        ad7998_data = (uint16_t) (ad7998_trans.buf[0] << 8 | ad7998_trans.buf[1]);
        ad7998_raw = ad7998_data & 0xFFF;
        ad7998_ch_id = (ad7998_trans.buf[0] >> 4) & 0x7;
        ad7998_flag = ad7998_trans.buf[0] >> 7;
        ad7998_voltage = (float)ad7998_raw*AD7998_MULT;
        ad7998_channels[0] = (float)ad7998_data;//voltage;

        //ad7998_addr = 0x80;//increment_channel(ad7998_addr);
        ad7998_status = AD7998_REQ;
        break;
      default:
        break;
    }
*/
    ad7998_data = (uint16_t) (ad7998_trans.buf[0] << 8 | ad7998_trans.buf[1]);
    ad7998_raw = ad7998_data & 0xFFF;
    ad7998_ch_id = (ad7998_trans.buf[0] >> 4) & 0x7;
    ad7998_flag = ad7998_trans.buf[0] >> 7;
    ad7998_voltage = (float)ad7998_raw*AD7998_MULT;
    ad7998_channels[ad7998_ch_id] = ad7998_voltage;

  ad7998_ch1 = 0;
  ad7998_ch2 = 0;
  ad7998_ch3 = 0;
  ad7998_ch4 = 0;
  ad7998_ch5 = 0;
  ad7998_ch6 = 0;
  ad7998_ch7 = 0;
  ad7998_ch8 = 0;

    // make trans done
    ad7998_trans.status = I2CTransDone;
    ad7998_trans_status = ad7998_trans.status;
  }
  else {
    if(ad7998_trans.status == I2CTransFailed) {
      ad7998_trans.status = I2CTransDone;
      ad7998_trans_status = ad7998_trans.status;
    }
  }
}

