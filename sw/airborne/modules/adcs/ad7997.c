/*
 * driver for ADC AD7997 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#include "modules/adcs/ad7997.h"

struct i2c_transaction ad7997_trans_u1;
struct i2c_transaction ad7997_trans_u2;
uint8_t ad7997_status_u1 = AD7997_UNINIT;
uint8_t ad7997_status_u2 = AD7997_UNINIT;
uint16_t u1_ports[] = {0,0,0,0,0,0,0,0};
uint16_t u2_ports[] = {0,0,0,0,0,0,0,0};
uint16_t u_ports[16] = {0x00};
uint8_t ad7997_trans_status_u1;
uint8_t ad7997_trans_status_u2;

static uint8_t startup_delay;


void ad7997_init(void) {
  startup_delay = AD7997_STARTUP_DELAY;

  // U1
  //configure MODE 2 - Sequence operation
  ad7997_trans_u1.buf[0] = AD7997_CONFIG_REG_ADDR;
  ad7997_trans_u1.buf[1] = AD7997_U1_CONFIG_DATA_0;
  ad7997_trans_u1.buf[2] = AD7997_U1_CONFIG_DATA_1;
  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u1, AD7997_U1_ADDR, 3);

/*
  // U2
  //configure MODE 2 - Sequence operation
  ad7997_trans_u2.buf[0] = AD7997_CONFIG_REG_ADDR;
  ad7997_trans_u2.buf[1] = AD7997_U2_CONFIG_DATA_0;
  ad7997_trans_u2.buf[2] = AD7997_U2_CONFIG_DATA_1;
  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u2, AD7997_U2_ADDR, 3);
*/
}




void ad7997_periodic(void){
  if(startup_delay > 0) {
    --startup_delay;
    return;
  }

/*
  switch (ad7997_status_u2) {
    case AD7997_UNINIT:
      // uninit, send again
      // Only if the transaction is complete
      if(ad7997_trans_u2.status == I2CTransDone) {
        // U2
        //configure MODE 2 - Sequence operation
        ad7997_trans_u2.buf[0] = AD7997_CONFIG_REG_ADDR;
        ad7997_trans_u2.buf[1] = AD7997_U2_CONFIG_DATA_0;
        ad7997_trans_u2.buf[2] = AD7997_U2_CONFIG_DATA_1;
        i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u2, AD7997_U2_ADDR, 3);
      }
      break;
    case AD7997_INIT:
      //init, request read
      // Only if the transaction is complete
      if(ad7997_trans_u2.status == I2CTransDone) {
        ad7997_trans_u2.buf[0] = AD7997_READ_SEQUENTIAL;
        i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u2, AD7997_U2_ADDR, 1);
      }
      break;
    case AD7997_READING:
      // reading, request data
      // Only if the transaction is complete
      if(ad7997_trans_u2.status == I2CTransDone) {
        i2c_receive(&AD7997_I2C_DEV, &ad7997_trans_u2, AD7997_U2_ADDR, 16);
      }
      break;
    default:
      break;
  }
*/
  switch (ad7997_status_u1) {
    case AD7997_UNINIT:
      // uninit, send again
      // Only if the transaction is complete
      if(ad7997_trans_u1.status == I2CTransDone) {
        // U2
        //configure MODE 2 - Sequence operation
        ad7997_trans_u1.buf[0] = AD7997_CONFIG_REG_ADDR;
        ad7997_trans_u1.buf[1] = AD7997_U1_CONFIG_DATA_0;
        ad7997_trans_u1.buf[2] = AD7997_U1_CONFIG_DATA_1;
        i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u1, AD7997_U1_ADDR, 3);
      }
      break;
    case AD7997_INIT:
      //init, request read
      // Only if the transaction is complete
      if(ad7997_trans_u1.status == I2CTransDone) {
        ad7997_trans_u1.buf[0] = AD7997_READ_SEQUENTIAL;
        i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_u1, AD7997_U1_ADDR, 1);
      }
      break;
    case AD7997_READING:
      // reading, request data
      // Only if the transaction is complete
      if(ad7997_trans_u1.status == I2CTransDone) {
        i2c_receive(&AD7997_I2C_DEV, &ad7997_trans_u1, AD7997_U1_ADDR, 16);
      }
      break;
    default:
      break;
  }
}

void ad7997_event(void){
  // U1
  // if last trans was success
  if(ad7997_trans_u1.status == I2CTransSuccess) {
    switch (ad7997_status_u1) {
      case AD7997_UNINIT:
        // was uninit, now init
        ad7997_status_u1 = AD7997_INIT;
        break;
      case AD7997_INIT:
        // we requested reading
        ad7997_status_u1 = AD7997_READING;
        break;
      case AD7997_READING:
        // we requested read
        u_ports[8] = (ad7997_trans_u1.buf[0] << 8 | ad7997_trans_u1.buf[1]) & 0xFFC;
        u_ports[9] = (ad7997_trans_u1.buf[2] << 8 | ad7997_trans_u1.buf[3]) & 0xFFC;
        u_ports[10] = (ad7997_trans_u1.buf[4] << 8 | ad7997_trans_u1.buf[5]) & 0xFFC;
        u_ports[11] = (ad7997_trans_u1.buf[6] << 8 | ad7997_trans_u1.buf[7]) & 0xFFC;
        u_ports[12] = (ad7997_trans_u1.buf[8] << 8 | ad7997_trans_u1.buf[9]) & 0xFFC;
        u_ports[13] = (ad7997_trans_u1.buf[10] << 8 | ad7997_trans_u1.buf[11]) & 0xFFC;
        u_ports[14] = (ad7997_trans_u1.buf[12] << 8 | ad7997_trans_u1.buf[13]) & 0xFFC;
        u_ports[15] = (ad7997_trans_u1.buf[14] << 8 | ad7997_trans_u1.buf[15]) & 0xFFC;
        ad7997_status_u1 = AD7997_INIT;
        break;
      default:
        break;
    }
    // make trans done
    ad7997_trans_u1.status = I2CTransDone;
    ad7997_trans_status_u1 = ad7997_trans_u1.status;
  }
  else {
    if(ad7997_trans_u1.status == I2CTransFailed) {
      ad7997_trans_u1.status = I2CTransDone;
      ad7997_trans_status_u1 = ad7997_trans_u1.status;
    }
  }

/*
  // U2
  // if last trans was success
  if(ad7997_trans_u2.status == I2CTransSuccess) {
    switch (ad7997_status_u2) {
      case AD7997_UNINIT:
        // was uninit, now init
        ad7997_status_u2 = AD7997_INIT;
        break;
      case AD7997_INIT:
        // we requested reading
        ad7997_status_u2 = AD7997_READING;
        break;
      case AD7997_READING:
        // we requested read
        u_ports[0] = (ad7997_trans_u2.buf[0] << 8 | ad7997_trans_u2.buf[1]) & 0xFFC;
        u_ports[1] = (ad7997_trans_u2.buf[2] << 8 | ad7997_trans_u2.buf[3]) & 0xFFC;
        u_ports[2] = (ad7997_trans_u2.buf[4] << 8 | ad7997_trans_u2.buf[5]) & 0xFFC;
        u_ports[3] = (ad7997_trans_u2.buf[6] << 8 | ad7997_trans_u2.buf[7]) & 0xFFC;
        u_ports[4] = (ad7997_trans_u2.buf[8] << 8 | ad7997_trans_u2.buf[9]) & 0xFFC;
        u_ports[5] = (ad7997_trans_u2.buf[10] << 8 | ad7997_trans_u2.buf[11]) & 0xFFC;
        u_ports[6] = (ad7997_trans_u2.buf[12] << 8 | ad7997_trans_u2.buf[13]) & 0xFFC;
        u_ports[7] = (ad7997_trans_u2.buf[14] << 8 | ad7997_trans_u2.buf[15]) & 0xFFC;
        ad7997_status_u2 = AD7997_INIT;
        break;
      default:
        break;
    }
    // make trans done
    ad7997_trans_u2.status = I2CTransDone;
    ad7997_trans_status_u2 = ad7997_trans_u2.status;
  }
  else {
    if(ad7997_trans_u2.status == I2CTransFailed) {
      ad7997_trans_u2.status = I2CTransDone;
      ad7997_trans_status_u2 = ad7997_trans_u2.status;
    }
  }
*/
}

