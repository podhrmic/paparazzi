/*
 * driver for ADC AD7997 on a custom made board
 * 2012, Utah State University, http://aggieair.usu.edu/
 */

#include "modules/adcs/ad7997.h"

struct i2c_transaction ad7997_trans_bus;
struct i2c_transaction ad7997_trans_balancer;
uint8_t ad7997_status_bus = AD7997_UNINIT;
uint8_t ad7997_status_balancer = AD7997_UNINIT;
uint16_t bus_current, bus_voltage;
uint16_t balancer_ports[] = {0,0,0,0,0,0,0,0};
uint8_t ad7997_trans_status_bus;
uint8_t ad7997_trans_status_balancer;
float measured_current, charge_integrated, charge_remaining;

float soc_values[] = {100,88,77,70,60,52,38,28,18,8,0};
float voltage_values[] = {16.7, 16.2, 15.9, 15.7, 15.5, 15.4, 15.17, 15, 14.87, 14.12,0};
float soc_offset = 0;

void init_ad7997(void) {
  bus_current = 0;
  bus_voltage = 0;
  measured_current = 0.0;
  charge_integrated = 0.0;
  charge_remaining = 100.0;

  // ADC 1: BALANCER
  //configure MODE 2 - Sequence operation
  ad7997_trans_balancer.buf[0] = AD7997_CONFIG_REG_ADDR;
  ad7997_trans_balancer.buf[1] = AD7997_BALANCER_CONFIG_DATA_0;
  ad7997_trans_balancer.buf[2] = AD7997_BALANCER_CONFIG_DATA_1;
  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_balancer, AD7997_BALANCER_ADDR, 3);
  ad7997_status_balancer = AD7997_INIT;
  ad7997_trans_status_balancer = ad7997_trans_balancer.status;

  // ADC 2: BUS
  //configure MODE 2 - Sequence operation
  ad7997_trans_bus.buf[0] = AD7997_CONFIG_REG_ADDR;
  ad7997_trans_bus.buf[1] = AD7997_BUS_CONFIG_DATA_0;
  ad7997_trans_bus.buf[2] = AD7997_BUS_CONFIG_DATA_1;
  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_bus, AD7997_BUS_ADDR, 3);
  ad7997_status_bus = AD7997_INIT;
  ad7997_trans_status_bus = ad7997_trans_bus.status;
}

void read_ad7997_debug(void) {
  ad7997_trans_bus.buf[0] = AD7997_CONFIG_REG_ADDR;
  ad7997_trans_bus.buf[1] = AD7997_BUS_CONFIG_DATA_0;
  ad7997_trans_bus.buf[2] = AD7997_BUS_CONFIG_DATA_1;
  i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_bus, AD7997_BUS_ADDR, 3);
}

void read_ad7997_bus(void) {
    uint16_t time_sec = sys_time.nb_sec;
    if ((time_sec >= 4) && (time_sec < 5)) {
      // initializing actual SOC
      uint8_t soc_index = 0;
      while (electrical.vsupply < voltage_values[soc_index]*10.0) {
        soc_index++;
      }
      if (soc_index == 0) {
        // soc is 100% as the voltage is above 16.7
      }
      else {
        if (soc_index == 10) {
          soc_offset = 99; // empty battery? no voltage?
        }
        else {
        // we are somewhere between
        float k = voltage_values[soc_index];
        float a = (voltage_values[soc_index-1] - voltage_values[soc_index])/(soc_values[soc_index-1]-soc_values[soc_index]);
        soc_offset = 100 - (((electrical.vsupply/10.0)-k)/a  + soc_values[soc_index]);
        }
      }
    }

    charge_integrated += measured_current/10; // [C]
    charge_remaining = (100*(79200-charge_integrated)/79200)-soc_offset; // [%] - this is for 2 Max Amps batteries

   if (ad7997_trans_bus.status == I2CTransSuccess) {
        switch (ad7997_status_bus) {
          case AD7997_INIT:
              ad7997_trans_bus.buf[0] = AD7997_READ_SEQUENTIAL;
              i2c_transceive(&AD7997_I2C_DEV, &ad7997_trans_bus, AD7997_BUS_ADDR, 1, 4);
              ad7997_status_bus = AD7997_READING;
              break;
          case AD7997_READING:
              bus_current = (ad7997_trans_bus.buf[0] << 8 | ad7997_trans_bus.buf[1]) & 0xFFC;
              bus_voltage = (ad7997_trans_bus.buf[2] << 8 | ad7997_trans_bus.buf[3]) & 0xFFC;
              electrical.vsupply = bus_voltage*5/100;//0.049198035;
              static bool_t test;
              if (electrical.vsupply < 10) {
                test = FALSE;
              }
              else {
                test = TRUE;
              }
              chDbgAssert(test, "read_ad7997_bus(), #66", "vsupply too low");
              //measured_current = ((float)bus_current-493)/17; // [A]
              measured_current = ((float)bus_current-487)/17; // [A]
              if (measured_current < 0) {
                measured_current = 0;
              }

              ad7997_status_bus = AD7997_INIT;
              break;
          default:
              break;
        }
     }
  else {
          electrical.vsupply = -1;
          //configure MODE 2 - Sequence operation
          ad7997_trans_bus.buf[0] = AD7997_CONFIG_REG_ADDR;
          ad7997_trans_bus.buf[1] = AD7997_BUS_CONFIG_DATA_0;
          ad7997_trans_bus.buf[2] = AD7997_BUS_CONFIG_DATA_1;
          i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_bus, AD7997_BUS_ADDR, 3);
      }
  ad7997_trans_status_bus = ad7997_trans_bus.status;
}

void read_ad7997_balancer(void) {
   if (ad7997_trans_balancer.status == I2CTransSuccess) {
        switch (ad7997_status_balancer) {
          case AD7997_INIT:
              ad7997_trans_balancer.buf[0] = AD7997_READ_SEQUENTIAL;
              i2c_transceive(&AD7997_I2C_DEV, &ad7997_trans_balancer, AD7997_BALANCER_ADDR, 1, 16);
              //i2c_transceive(&AD7997_I2C_DEV, &ad7997_trans_balancer, AD7997_BALANCER_ADDR, 1, 2);
              ad7997_status_balancer = AD7997_READING;
              break;
          case AD7997_READING:

              balancer_ports[0] = (ad7997_trans_balancer.buf[0] << 8 | ad7997_trans_balancer.buf[1]) & 0xFFC;
              balancer_ports[1] = (ad7997_trans_balancer.buf[2] << 8 | ad7997_trans_balancer.buf[3]) & 0xFFC;
              balancer_ports[2] = (ad7997_trans_balancer.buf[4] << 8 | ad7997_trans_balancer.buf[5]) & 0xFFC;
              balancer_ports[3] = (ad7997_trans_balancer.buf[6] << 8 | ad7997_trans_balancer.buf[7]) & 0xFFC;
              balancer_ports[4] = (ad7997_trans_balancer.buf[8] << 8 | ad7997_trans_balancer.buf[9]) & 0xFFC;
              balancer_ports[5] = (ad7997_trans_balancer.buf[10] << 8 | ad7997_trans_balancer.buf[11]) & 0xFFC;
              balancer_ports[6] = (ad7997_trans_balancer.buf[12] << 8 | ad7997_trans_balancer.buf[13]) & 0xFFC;
              balancer_ports[7] = (ad7997_trans_balancer.buf[14] << 8 | ad7997_trans_balancer.buf[15]) & 0xFFC;

              /*
              * READ CHANNEL NUMBERS *
              balancer_ports[0] = (ad7997_trans_balancer.buf[0] & 0x70) >> 4;
              balancer_ports[1] = (ad7997_trans_balancer.buf[2] & 0x70) >> 4;
              balancer_ports[2] = (ad7997_trans_balancer.buf[4] & 0x70) >> 4;
              balancer_ports[3] = (ad7997_trans_balancer.buf[6] & 0x70) >> 4;
              balancer_ports[4] = (ad7997_trans_balancer.buf[8] & 0x70) >> 4;
              balancer_ports[5] = (ad7997_trans_balancer.buf[10] & 0x70) >> 4;
              balancer_ports[6] = (ad7997_trans_balancer.buf[12] & 0x70) >> 4;
              balancer_ports[7] = (ad7997_trans_balancer.buf[14] & 0x70) >> 4;
              */
              ad7997_status_balancer = AD7997_INIT;
              break;
          default:
              break;
        }
     }
  else {
          //configure MODE 2 - Sequence operation
          balancer_ports[0] = -1;
          ad7997_trans_balancer.buf[0] = AD7997_CONFIG_REG_ADDR;
          ad7997_trans_balancer.buf[1] = AD7997_BALANCER_CONFIG_DATA_0;
          ad7997_trans_balancer.buf[2] = AD7997_BALANCER_CONFIG_DATA_1;
          i2c_transmit(&AD7997_I2C_DEV, &ad7997_trans_balancer, AD7997_BALANCER_ADDR, 3);
      }
  ad7997_trans_status_balancer = ad7997_trans_balancer.status;
}
