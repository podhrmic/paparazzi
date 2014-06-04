/*
 * driver for ADC AD7997 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#ifndef AD7997_H
#define AD7997_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#ifndef AD7997_I2C_DEV
#define AD7997_I2C_DEV i2c1
#endif

#ifndef AD7997_STARTUP_DELAY
#define AD7997_STARTUP_DELAY 0
#endif

#define AD7997_U1_ADDR 0x44// u1 airspeed etc
#define AD7997_U2_ADDR 0x44// u2 servo current

/* Config register addres */
#define AD7997_CONFIG_REG_ADDR 0x2

// For all channels + filter on I2C
#define AD7997_U1_CONFIG_DATA_0 0x0 // MSB
#define AD7997_U1_CONFIG_DATA_1 0xF8// LSB

// For all channels + filter on I2C
#define AD7997_U2_CONFIG_DATA_0 0xF // MSB
#define AD7997_U2_CONFIG_DATA_1 0xF8// LSB

/* Initiate conversion and sequential read */
#define AD7997_READ_SEQUENTIAL 0x70 // Address pointer byte


enum ad7997_stat{
  AD7997_UNINIT,
  AD7997_INIT,
  AD7997_READING
};

void ad7997_init(void);
void ad7997_event(void);
void ad7997_periodic(void);

//void read_ad7997_u2(void);
//void read_ad7997_u1(void);
//void read_ad7997_debug(void);


extern uint8_t ad7997_status_u1;
extern uint8_t ad7997_status_u2;
extern uint8_t ad7997_trans_status_u1;
extern uint8_t ad7997_trans_status_u2;
extern uint16_t u1_ports[];
extern uint16_t u2_ports[];
extern uint16_t u_ports[];

#endif /* AD7997_H */
