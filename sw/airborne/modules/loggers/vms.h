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
 *
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef VMS_H
#define VMS_H

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "std.h"

#define __VMSLink(dev, _x) dev##_x
#define _VMSLink(dev, _x)  __VMSLink(dev, _x)
#define VMSLink(_x) _VMSLink(VMS_LINK, _x)

#define VMSBuffer() VMSLink(ChAvailable())

/*
 * Message headers
 */
#define VMS_MSG0 0x21
#define VMS_MSG1 0x3F
#define VMS_BUFFER_SIZE 128
#define VMS_HEADER_SIZE 2

extern struct VMS_packet VMS_packet;
extern struct VMS_data bms;

extern void vms_packet_read_message(void);
extern void vms_packet_parse(uint8_t c);


extern float VMS_freq;
extern uint16_t VMS_chksm;
extern uint16_t VMS_calcsm;
extern uint32_t VMS_time;

struct VMS_packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[VMS_BUFFER_SIZE];
  uint8_t  status;
  uint8_t  msg_idx;
  uint16_t datalength;
};

enum VMSMsgStatus {
  VMSHeader0,
  VMSHeader1,
  VMSDatalength,
  VMSData
};

struct VMS_data {
	uint16_t up_time;
	int16_t	dc_current;
	int16_t motor_speed;
	int16_t dc_voltage;
	int16_t torque_fb;
	uint16_t counter;
	struct VMS_packet vms_packet;
	int8_t min_cell_temp;
	int8_t max_cell_temp;
	uint16_t min_cell_volt;
	uint16_t max_cell_volt;
};


void vms_init(void);
//void vms_event(void);

static inline void ReadVMSBuffer(void) {
  while (VMSLink(ChAvailable()) && !bms.vms_packet.msg_available)
    vms_packet_parse(VMSLink(Getch()));
}

static inline void vms_event(void) {
  if (VMSBuffer()) {
    ReadVMSBuffer();
  }
  if (bms.vms_packet.msg_available) {
    vms_packet_read_message();
    bms.vms_packet.msg_available = FALSE;
  }
}

/**
 * Calculates the 16-bit CRC for the given ASCII or binary message.
 * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
 * and ending at the end of payload.
 */
static inline unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
	uint8_t c0, c1;
	uint16_t i;//, size;
	c0 = 0;
	c1 = 0;

	//Start at byte three so MSG0 and MSG1 is not part of the checksum
	for ( i = 0; i < length; i++ ) {
		c0 += (uint8_t)data[i];
		c1 += c0;
	}
	return (unsigned short)(c0 << 8| c1);
	/*
  unsigned int i;
  unsigned short crc = 0;
  for (i=0; i<length; i++){
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
  */
}

/*
static inline bool_t GX3_verify_chk(volatile uint8_t *buff_add) {
  uint16_t i,chk_calc;
  chk_calc = 0;
  for (i=0;i<GX3_MSG_LEN-2;i++) {
    chk_calc += (uint8_t)*buff_add++;
  }
  return (chk_calc == ( (((uint16_t)*buff_add)<<8) + (uint8_t)*(buff_add+1) ));
}
*/

/**
 * Verify checksum
 */
static inline bool vms_verify_chk(unsigned char data[], unsigned int length, uint16_t* calc_chk, uint16_t* rec_chk) {
  unsigned short calc_crc = calculateCRC(data, length);
  unsigned short rec_crc = (unsigned short) (data[length] << 8 | data[length+1]);
  *calc_chk = (uint16_t) calc_crc;
  *rec_chk = (uint16_t) rec_crc;
  if (calc_crc == rec_crc) {
    return 1;
  }
  else {
    return 0;
  }
}

#endif /* VMS */
