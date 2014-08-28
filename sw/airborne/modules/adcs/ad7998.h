/*
 * driver for ADC AD7998 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#ifndef AD7998_H
#define AD7998_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#ifndef AD7998_I2C_DEV
#define AD7998_I2C_DEV i2c1
#endif

#define AD7998_ADDR 0x40// normally 0x20 

#define AD7998_READ_CH1 0x80
#define AD7998_READ_CH2 0x90
#define AD7998_READ_CH3 0xA0
#define AD7998_READ_CH4 0xB0
#define AD7998_READ_CH5 0xC0
#define AD7998_READ_CH6 0xD0
#define AD7998_READ_CH7 0xE0
#define AD7998_READ_CH8 0xF0

#define AD7998_VREF 5.0 // V
#define AD7998_RES 12 // bit resolution
#define AD7998_MULT AD7998_VREF/(1<<AD7998_RES)

enum ad7998_stat{
  AD7998_UNINIT,
  AD7998_INIT,
  AD7998_REQ,
  AD7998_READ
};

void ad7998_init(void);
void ad7998_event(void);
void ad7998_periodic(void);
void increment_channel(uint8_t *addr);


extern uint8_t ad7998_status;
extern uint8_t ad7998_trans_status;


extern float ad7998_ch1;
extern float ad7998_ch2;
extern float ad7998_ch3;
extern float ad7998_ch4;
extern float ad7998_ch5;
extern float ad7998_ch6;
extern float ad7998_ch7;
extern float ad7998_ch8;
extern float ad7998_channels[];

extern uint16_t ad7998_data;
extern uint16_t ad7998_raw;
extern uint8_t ad7998_ch_id;
extern uint8_t ad7998_flag;
extern float ad7998_voltage;

#endif /* AD7998_H */
