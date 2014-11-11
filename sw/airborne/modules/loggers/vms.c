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
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "modules/loggers/vms.h"
#include "subsystems/electrical.h"


struct VMS_packet vms_packet;
struct VMS_data bms;

uint32_t VMS_time;
uint32_t VMS_ltime;
uint16_t VMS_chksm;
uint16_t VMS_calcsm;
float VMS_freq;



void vms_init(void){
	// initialize things to zero
	VMS_chksm = 0;
	VMS_calcsm = 0;
	VMS_time = 0;
	VMS_ltime = 0;
	VMS_freq = 0;
}

/*
 * Read received packet
 */
void vms_packet_read_message(void) {
	uint16_t idx = VMS_HEADER_SIZE;
	// LSB first
	//uptime uint16_t
	bms.up_time = (uint16_t)(bms.vms_packet.msg_buf[idx+1] << 8 | bms.vms_packet.msg_buf[idx]);
	idx = idx + 2;

	// DC current - dc_current, int16_t
	bms.dc_current = (uint16_t)(bms.vms_packet.msg_buf[idx+1] << 8 | bms.vms_packet.msg_buf[idx]);
	idx = idx + 2;

	// RPM - motor_speed, int16_t
	bms.motor_speed = (uint16_t)(bms.vms_packet.msg_buf[idx+1] << 8 | bms.vms_packet.msg_buf[idx]);
	idx = idx + 2;

	// DC Voltage - dc_voltage, int16_t
	bms.dc_voltage = (uint16_t)(bms.vms_packet.msg_buf[idx+1] << 8 | bms.vms_packet.msg_buf[idx]);
	idx = idx + 2;

	// torque_fb - estimated motor torque, int16_t
	bms.torque_fb = (uint16_t)(bms.vms_packet.msg_buf[idx+1] << 8 | bms.vms_packet.msg_buf[idx]);

	//electrical current
	electrical.vsupply = bms.dc_voltage;
	electrical.current = bms.dc_current*100;
}

/*
 *  Packet Collection & state machine
 */
void vms_packet_parse( uint8_t c) {
	switch (bms.vms_packet.status) {
	case VMSHeader0:
		// sync the header
		bms.vms_packet.msg_idx = 0;
		if (c == VMS_MSG0) {
			bms.vms_packet.status = VMSHeader1;
		} else {
			bms.vms_packet.hdr_error++;
		}
		break;
	case VMSHeader1:
		// sync the header
		if (c == VMS_MSG1) {
			bms.vms_packet.status = VMSDatalength;
		} else {
			bms.vms_packet.hdr_error++;
			bms.vms_packet.status = VMSHeader0;
		}
		break;
	case VMSDatalength:
		// read datalength data
		bms.vms_packet.msg_buf[bms.vms_packet.msg_idx] = c;
		bms.vms_packet.msg_idx++;
		if (bms.vms_packet.msg_idx == 2) {
			bms.vms_packet.datalength = 12;// 10b data, 2b datalength, 2b header, 2b checksum - total 12+2
			bms.vms_packet.status = VMSData;
		}
		break;
	case VMSData:
		bms.vms_packet.msg_buf[bms.vms_packet.msg_idx] =  c;
		bms.vms_packet.msg_idx++;
		if (bms.vms_packet.msg_idx == (bms.vms_packet.datalength+2)) {
			if (vms_verify_chk(bms.vms_packet.msg_buf,bms.vms_packet.datalength, &VMS_calcsm, &VMS_chksm)) {
				bms.vms_packet.msg_available = true;
				bms.counter++;
			} else {
				bms.vms_packet.msg_available = false;
				bms.vms_packet.chksm_error++;
			}
			bms.vms_packet.status = VMSHeader0;
		}
		break;
	default:;
	bms.vms_packet.status = VMSHeader0;
	bms.vms_packet.msg_idx = 0;
	break;
	}
}


