/*
 * driver for ADC AD7997 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#ifndef AD7997_H
#define AD7997_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "subsystems/electrical.h"
#include "mcu_periph/sys_time.h"

#ifndef AD7997_I2C_DEV
#define AD7997_I2C_DEV i2c2
#endif

/* address is 01000110 (AS=GND), 01001000 (AS=VDD), 01000000 (AS=FLOAT) */
#define AD7997_BUS_ADDR 0x46
#define AD7997_BALANCER_ADDR 0x40
// 0x44 for combiner board -> AD7998-0 (OK)
// 0x40 on the Power Board -> AD7997-1 float
// 0x46 on the Power Board -> AD7997-1 GND (OK)

/* Config register addres */
#define AD7997_CONFIG_REG_ADDR 0x2

/* Configuration for MODE 2 - sequential read, see AD7997 datasheet */
//activate CH1,3+ filter on I2C line
#define AD7997_BUS_CONFIG_DATA_0 0x0 // first byte
#define AD7997_BUS_CONFIG_DATA_1 0x58// second byte

// For all channels + filter on I2C
#define AD7997_BALANCER_CONFIG_DATA_0 0xF // first byte
#define AD7997_BALANCER_CONFIG_DATA_1 0xF8// second byte

// For channel 1 and 2
//#define AD7997_BALANCER_CONFIG_DATA_0 0x0 // first byte
//#define AD7997_BALANCER_CONFIG_DATA_1 0x18// second byte

/* Initiate conversion and sequential read */
#define AD7997_READ_SEQUENTIAL 0x70

#define BATTERY_CELL_NB 4

enum ad7997_stat{
  AD7997_UNINIT,
  AD7997_INIT,
  AD7997_READING
};

void init_ad7997(void);
void read_ad7997_balancer(void);
void read_ad7997_bus(void);
void read_ad7997_debug(void);

extern uint16_t bus_current, bus_voltage, test;
extern float measured_current, charge_integrated, charge_remaining;
extern uint8_t ad7997_trans_status_bus;
extern uint8_t ad7997_trans_status_balancer;
extern uint16_t balancer_ports[];

#endif /* AD7997_H */
