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
 */
 /**
 * @file xgear.c
 *
 * Module for Xgear datalogging and block smashing code
 *
 * Meant for RT Paparazzi only - Rotorcraft version
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "modules/loggers/xgear.h"

static uint8_t XGEAR_GPS_buf[XGEAR_PACKET_SIZE_GPS];
static uint8_t XGEAR_IMU_buf[XGEAR_PACKET_SIZE_ATTITUDE];
static uint8_t XGEAR_RADIO_buf[XGEAR_PACKET_SIZE_RADIO];
static uint8_t XGEAR_STATUS_buf[XGEAR_PACKET_SIZE_STATUS];
static uint8_t XGEAR_DEBUG_buf[XGEAR_PACKET_SIZE_DEBUG];
static uint8_t XGEAR_CODEBLCOK_buf[XGEAR_PACKET_SIZE_CODEBLOCK];

// Thread defines
static __attribute__((noreturn)) msg_t thd_xgear_rx(void *arg);
//static __attribute__((noreturn)) msg_t thd_xgear_tx(void *arg);

// Thread working area defines
#define CH_THREAD_AREA_XGEAR_TX 512
#define CH_THREAD_AREA_XGEAR_RX 512

// Working area initialization
static WORKING_AREA(wa_thd_xgear_rx, CH_THREAD_AREA_XGEAR_RX);
//static WORKING_AREA(wa_thd_xgear_tx, CH_THREAD_AREA_XGEAR_TX);

/**
 * XGEAR RX data callback
 *
 * For block smashing code - this function is called once
 * full packet is received
 */
void xgear_msg_ready(void)
{
  //dummy for now
  // DOWNLINK_SEND_BLOCKSMASHING_CODE_MESSAGE(arg1...argX);
}

/**
 * Xgear parse message
 */
void xgear_parse_msg(uint8_t c) {
  static uint8_t counter;
  counter++;
  (void) c;// Dummy
  if (counter == XGEAR_PACKET_SIZE_CODEBLOCK) {
    xgear_msg_ready();
    counter = 0;
  }
}

/**
 * RX thread
 */
__attribute__((noreturn)) msg_t thd_xgear_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("module_xgear_rx");
  EventListener elXgearRxData;
  flagsmask_t rx_flags;
  chEvtRegisterMask((EventSource *)chnGetEventSource((SerialDriver*)XGEAR_PORT.reg_addr), &elXgearRxData, EVENT_MASK(1));
  while (TRUE) {
     chEvtWaitOneTimeout(EVENT_MASK(1), S2ST(1));
     rx_flags = chEvtGetAndClearFlags(&elXgearRxData);
     uart_receive_buffer(&XGEAR_PORT, rx_flags, &xgear_parse_msg);
     // xgear_parse_msg takes care of calling the other function once new message is available
  }
}

/**
 * TX thread
 */
 /*
__attribute__((noreturn)) msg_t thd_xgear_tx(void *arg)
{
  chRegSetThreadName("module_xgear_tx");
  (void) arg;

  EventListener elXgearRadioData, elXgearGpsData;
  chEvtRegister(&eventGpsData, &elXgearGpsData, EVT_GPS_DATA);
  chEvtRegister(&eventRadioData, &elXgearRadioData, EVT_RADIO_DATA);
  //chEvtRegister(&eventPpmFrame, &elXgearRadioData, EVT_PPM_FRAME);
  eventmask_t xgear_mask;

  while (TRUE)
  {
    xgear_mask = chEvtWaitAny(EVENT_MASK(EVT_GPS_DATA | EVT_RADIO_DATA));
    if (xgear_mask & EVT_GPS_DATA) {
      xgear_send_gps();
    }
    if (xgear_mask & EVT_RADIO_DATA) {
      xgear_send_radio();
    }
  }
}
*/


/**
 * Initialize Xgear module
 *
 * Message Buffer Headers never change
 */
void xgear_init(void) {
	// IMU packet
	XGEAR_IMU_buf[0] = XGEAR_MSG0;
	XGEAR_IMU_buf[1] = XGEAR_MSG1;
  XGEAR_IMU_buf[2] = XGEAR_HEADER_ATTITUDE;
  XGEAR_IMU_buf[3] = XGEAR_PACKET_SIZE_ATTITUDE;

  // GPS packet
	XGEAR_GPS_buf[0] = XGEAR_MSG0;
	XGEAR_GPS_buf[1] = XGEAR_MSG1;
  XGEAR_GPS_buf[2] = XGEAR_HEADER_GPS;
  XGEAR_GPS_buf[3] = XGEAR_PACKET_SIZE_GPS;

  // Radio packet
  XGEAR_RADIO_buf[0] = XGEAR_MSG0;
  XGEAR_RADIO_buf[1] = XGEAR_MSG1;
  XGEAR_RADIO_buf[2] = XGEAR_HEADER_RADIO;
  XGEAR_RADIO_buf[3] = XGEAR_PACKET_SIZE_RADIO;

  // Status packet
  XGEAR_STATUS_buf[0] = XGEAR_MSG0;
  XGEAR_STATUS_buf[1] = XGEAR_MSG1;
  XGEAR_STATUS_buf[2] = XGEAR_HEADER_STATUS;
  XGEAR_STATUS_buf[3] = XGEAR_PACKET_SIZE_STATUS_BYTE;

  // Debug packet
  memset(&XGEAR_DEBUG_buf, 0xAA, sizeof(XGEAR_DEBUG_buf));
  XGEAR_DEBUG_buf[0] = XGEAR_MSG0;
  XGEAR_DEBUG_buf[1] = XGEAR_MSG1;
  XGEAR_DEBUG_buf[2] = XGEAR_HEADER_DEBUG;
  XGEAR_DEBUG_buf[3] = XGEAR_PACKET_SIZE_DEBUG;

  // Blocksmashing packet
  // TODO: maybe we don't need header?
  XGEAR_CODEBLCOK_buf[0] = XGEAR_MSG0;
  XGEAR_CODEBLCOK_buf[1] = XGEAR_MSG1;
  XGEAR_CODEBLCOK_buf[2] = XGEAR_HEADER_CODEBLOCK;
  XGEAR_CODEBLCOK_buf[3] = XGEAR_PACKET_SIZE_CODEBLOCK;

  //Init threads
  chThdCreateStatic(wa_thd_xgear_rx, sizeof(wa_thd_xgear_rx), NORMALPRIO, thd_xgear_rx, NULL);
  //chThdCreateStatic(wa_thd_xgear_tx, sizeof(wa_thd_xgear_tx), NORMALPRIO, thd_xgear_tx, NULL);
}


/**
 * Periodic Xgear function
 *
 * Calls xgear_send_imu() (attitude data, meant for AggieNav only)
 * and xgear_send_status() (status for datalogging, for AggiePilot
 * only) at periodic frequency (defined in xgear.xml)
 *
 * Other data are sent from a separate thread
 */
void xgear_periodic(void) {
#ifdef XGEAR_AGGIENAV
    xgear_send_imu();
#else
    xgear_send_status();
#endif
}

/**
 * Send debug data
 */
void xgear_send_debug(void) {
  static uint8_t cksum0, cksum1, debug_index;
  cksum0 = 0;
  cksum1 = 0;
  debug_index = XGEAR_DATA_INDEX;
  static float timestamp;

  // System time [s]
  timestamp = (float)chTimeNow()/CH_FREQUENCY;
  memcpy(&XGEAR_DEBUG_buf[debug_index], &timestamp, sizeof(float));
  debug_index += sizeof(float);

  // calculate checksum & send
  xgear_cksum(XGEAR_PACKET_SIZE_DEBUG, (uint8_t *)XGEAR_DEBUG_buf, &cksum0, &cksum1);
  XGEAR_DEBUG_buf[XGEAR_PACKET_SIZE_DEBUG-2] = cksum0;
  XGEAR_DEBUG_buf[XGEAR_PACKET_SIZE_DEBUG-1] = cksum1;
  uart_transmit_buffer(&XGEAR_PORT, XGEAR_DEBUG_buf, (size_t) XGEAR_PACKET_SIZE_DEBUG);
}


/**
 * Send GPS data
 */
void xgear_send_gps(void) {
  static uint8_t cksum0, cksum1, gps_index;
  cksum0 = 0;
  cksum1 = 0;
  gps_index = XGEAR_DATA_INDEX;

  // TODO: Mutex
  //GPS ECEFCORD position, int32
  for (uint8_t i = 0;i<4;i++){
	  XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_pos.x>>(i*8));
	  gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_pos.y>>(i*8));
    gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_pos.z>>(i*8));
    gps_index++;
  }
  //GPS LLA, int32
  for (uint8_t i = 0;i<4;i++){
	  XGEAR_GPS_buf[gps_index] = 0xFF&(gps.lla_pos.lat>>(i*8));
	  gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.lla_pos.lon>>(i*8));
    gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.lla_pos.alt>>(i*8));
    gps_index++;
  }
  //GPS hmsl, int32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.hmsl>>(i*8));
    gps_index++;
  }
  //GPS ECEFCORD speed, int32
  for (uint8_t i = 0;i<4;i++){
	  XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_vel.x>>(i*8));
	  gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_vel.y>>(i*8));
    gps_index++;
  }
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.ecef_vel.z>>(i*8));
    gps_index++;
  }
  //GPS pacc, uint32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.pacc>>(i*8));
    gps_index++;
  }
  //GPS sacc, uint32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.sacc>>(i*8));
    gps_index++;
  }
  //GPS tow, uint32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.tow>>(i*8));
    gps_index++;
  }
  //GPS pdop, uint16
  for (uint8_t i = 0; i<2; i++){
    XGEAR_GPS_buf[gps_index] = 0xFF&(gps.pdop>>(i*8));
    gps_index++;
  }
  //GPS num_sv, uint8
  XGEAR_GPS_buf[gps_index] = gps.fix;
  gps_index++;

  //GPS num_sv, uint8
  XGEAR_GPS_buf[gps_index] = gps.num_sv;
  gps_index++;
  // TODO: Mutex

  // calculate checksum & send
  xgear_cksum(XGEAR_PACKET_SIZE_GPS, (uint8_t *)XGEAR_GPS_buf, &cksum0, &cksum1);
  XGEAR_GPS_buf[XGEAR_PACKET_SIZE_GPS-2] = cksum0;
  XGEAR_GPS_buf[XGEAR_PACKET_SIZE_GPS-1] = cksum1;
  uart_transmit_buffer(&XGEAR_PORT, XGEAR_GPS_buf, (size_t) XGEAR_PACKET_SIZE_GPS);
}

/**
 * Send imu/attitude data
 */
void xgear_send_imu(void) {
  static uint8_t cksum0, cksum1, imu_index;
  cksum0 = 0;
  cksum1 = 0;
  imu_index = XGEAR_DATA_INDEX;
  static float timestamp;

  // TODO: Mutex

  // System time [s]
  timestamp = (float)chTimeNow()/CH_FREQUENCY;
  memcpy(&XGEAR_IMU_buf[imu_index], &timestamp, sizeof(float));
  imu_index += sizeof(float);

  // Acceleration, in imu coordinate system
  for (uint8_t i = 0;i<4;i++){
    XGEAR_IMU_buf[imu_index] = 0xFF&(imu.accel.x>>(i*8));
    imu_index++;
  }
  for (uint8_t i = 0;i<4;i++){
    XGEAR_IMU_buf[imu_index] = 0xFF&(imu.accel.y>>(i*8));
    imu_index++;
  }
  for (uint8_t i = 0;i<4;i++){
    XGEAR_IMU_buf[imu_index] = 0xFF&(imu.accel.z>>(i*8));
    imu_index++;
  }

  // Angular rates, body frame
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetBodyRates_f()->p), sizeof(float));
  imu_index += sizeof(float);
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetBodyRates_f()->q), sizeof(float));
  imu_index += sizeof(float);
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetBodyRates_f()->r), sizeof(float));
  imu_index += sizeof(float);

  // Attitude, eulers, body frame
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetNedToBodyEulers_f()->phi), sizeof(float));
  imu_index += sizeof(float);
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetNedToBodyEulers_f()->theta), sizeof(float));
  imu_index += sizeof(float);
  memcpy(&XGEAR_IMU_buf[imu_index], &(stateGetNedToBodyEulers_f()->psi), sizeof(float));
  imu_index += sizeof(float);
  // TODO: Mutex

  // calculate checksum & send
  xgear_cksum(XGEAR_PACKET_SIZE_ATTITUDE, (uint8_t *)XGEAR_IMU_buf, &cksum0, &cksum1 );
  XGEAR_IMU_buf[XGEAR_PACKET_SIZE_ATTITUDE-2] = cksum0;
  XGEAR_IMU_buf[XGEAR_PACKET_SIZE_ATTITUDE-1] = cksum1;
  uart_transmit_buffer(&XGEAR_PORT, XGEAR_IMU_buf, (size_t) XGEAR_PACKET_SIZE_ATTITUDE);
}

/**
 * Send radio values
 */
void xgear_send_radio(void) {
  uint8_t cksum0, cksum1;
  // Radio channel values, int16_t
  memcpy(&XGEAR_RADIO_buf[XGEAR_DATA_INDEX], radio_control.values, RADIO_CONTROL_NB_CHANNEL*sizeof(int16_t));
  // calculate checksum & send
  xgear_cksum(XGEAR_PACKET_SIZE_RADIO, (uint8_t *)XGEAR_RADIO_buf, &cksum0, &cksum1 );
  XGEAR_RADIO_buf[XGEAR_PACKET_SIZE_RADIO-2] = cksum0;
  XGEAR_RADIO_buf[XGEAR_PACKET_SIZE_RADIO-1] = cksum1;
  uart_transmit_buffer(&XGEAR_PORT, XGEAR_RADIO_buf, (size_t) XGEAR_PACKET_SIZE_RADIO);
}


/**
 * Sens AP status
 *
 * TODO: FIXME later
 */
void xgear_send_status(void) {
  uint8_t cksum0, cksum1;
  uint8_t xgear_index = XGEAR_DATA_INDEX;

  /*
  // TIMING
  // AggieNav GX3 Time (float)  [s]
  memcpy(&XGEAR_STATUS_buf[xgear_index], &AGGIENAV_time, sizeof(float));
  xgear_index += sizeof(float);
  // AggieNav Sys Time (float) [s]
  memcpy(&XGEAR_STATUS_buf[xgear_index], &AGGIENAV_systime, sizeof(float));
  xgear_index += sizeof(float);
  // AggiePilot Sys Time (float) [s]
  aggiePilot_systime = (float)(get_sys_time_usec()/1000000.0);
  memcpy(&XGEAR_STATUS_buf[xgear_index], &aggiePilot_systime, sizeof(float));
  xgear_index += sizeof(float);

  // COMMANDS
  //Commands, int32, roll, pitch, yaw, thrust
  memcpy(&XGEAR_STATUS_buf[xgear_index], stabilization_cmd, 4*sizeof(int32_t));
  xgear_index += 4*sizeof(int32_t);
  // Actuators PWM commands (6 of them), int32_t
  memcpy(&XGEAR_STATUS_buf[xgear_index], actuators_pwm_values, ACTUATORS_NB*sizeof(int32_t));
  xgear_index += ACTUATORS_NB*sizeof(int32_t);

  // POWER
  // Measured current, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &measured_current, sizeof(float));
  xgear_index += sizeof(float);
  // State of Charge, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &charge_remaining, sizeof(float));
  xgear_index += sizeof(float);

  // Vsupply, uint16, Decivolts
  for (uint8_t i = 0; i<2; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(electrical.vsupply>>(i*8));
    xgear_index++;
  }

  // ROTORCRAFT_FP aka H_LOOP
  // Rotorcraft position North, int32
  int32_t tmp = (stateGetPositionEnu_i()->x);
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(tmp>>(i*8));
    xgear_index++;
  }
  // Rotorcraft position East, int32
  tmp = (stateGetPositionEnu_i()->y);
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(tmp>>(i*8));
    xgear_index++;
  }
  // Rotorcraft velocity North, int32
  tmp = (stateGetSpeedEnu_i()->x);
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(tmp>>(i*8));
    xgear_index++;
  }
  // Rotorcraft velocity East, int32
  tmp = (stateGetSpeedEnu_i()->y);
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(tmp>>(i*8));
    xgear_index++;
  }
  // Rotorcraft flight time, uin16
  for (uint8_t i = 0; i<2; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(autopilot_flight_time>>(i*8));
    xgear_index++;
  }
  // Rotorcraft carrot east, int32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(guidance_h_pos_sp.y>>(i*8));
    xgear_index++;
  }
  // Rotorcraft carrot north, int32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(guidance_h_pos_sp.x>>(i*8));
    xgear_index++;
  }

  // ROTORCRAFT_STATUS
  //AP mode, uint8
  XGEAR_STATUS_buf[xgear_index] = autopilot_mode;
  xgear_index++;
  //AP_motors_on, uint8
  XGEAR_STATUS_buf[xgear_index] = autopilot_motors_on;
  xgear_index++;
  //AP_in_flight, uint8
  XGEAR_STATUS_buf[xgear_index] = autopilot_in_flight;
  xgear_index++;
  //RC_status, uint8
  XGEAR_STATUS_buf[xgear_index] = radio_control.status;
  xgear_index++;
  //RC_frame_rate, uint8
  XGEAR_STATUS_buf[xgear_index] = radio_control.frame_rate;
  xgear_index++;

  //SONAR
  // sonar measure, uint16
  for (uint8_t i = 0; i<2; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(sonar_meas>>(i*8));
    xgear_index++;
  }
  //sonar distance, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &sonar_distance, sizeof(float));
  xgear_index += sizeof(float);

  // STAB_ATTITUDE_FLOAT
  //est_p, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetBodyRates_f()->p), sizeof(float));
  xgear_index += sizeof(float);
  //est_q, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetBodyRates_f()->q), sizeof(float));
  xgear_index += sizeof(float);
  //est_r, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetBodyRates_f()->r), sizeof(float));
  xgear_index += sizeof(float);
  //est_phi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetNedToBodyEulers_f()->phi), sizeof(float));
  xgear_index += sizeof(float);
  //est_theta, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetNedToBodyEulers_f()->theta), sizeof(float));
  xgear_index += sizeof(float);
  //est_psi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &(stateGetNedToBodyEulers_f()->psi), sizeof(float));
  xgear_index += sizeof(float);
  //ref_phi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stab_att_ref_euler.phi, sizeof(float));
  xgear_index += sizeof(float);
  //ref_theta, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stab_att_ref_euler.theta, sizeof(float));
  xgear_index += sizeof(float);
  //ref_psi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stab_att_ref_euler.psi, sizeof(float));
  xgear_index += sizeof(float);
  //sum_err_phi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_sum_err.phi, sizeof(float));
  xgear_index += sizeof(float);
  //sum_err_theta, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_sum_err.theta, sizeof(float));
  xgear_index += sizeof(float);
  //sum_err_psi, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_sum_err.psi, sizeof(float));
  xgear_index += sizeof(float);
  //feedback roll, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_fb_cmd[COMMAND_ROLL], sizeof(float));
  xgear_index += sizeof(float);
  //feedback pitch, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_fb_cmd[COMMAND_PITCH], sizeof(float));
  xgear_index += sizeof(float);
  //feedback yaw, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_fb_cmd[COMMAND_YAW], sizeof(float));
  xgear_index += sizeof(float);
  //feedforward roll, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_ff_cmd[COMMAND_ROLL], sizeof(float));
  xgear_index += sizeof(float);
  //feeforward pitch, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_ff_cmd[COMMAND_PITCH], sizeof(float));
  xgear_index += sizeof(float);
  //feedforward yaw, f
  memcpy(&XGEAR_STATUS_buf[xgear_index], &stabilization_att_ff_cmd[COMMAND_YAW], sizeof(float));
  xgear_index += sizeof(float);

  //BARO RAW
  // Baro absolute, int32
  for (uint8_t i = 0; i<4; i++){
    XGEAR_STATUS_buf[xgear_index] = 0xFF&(baro.absolute>>(i*8));
    xgear_index++;
  }

  //ACCELERATION
  // Acceleration x, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &imuf.accel.x, sizeof(float));
  xgear_index += sizeof(float);
  // Acceleration x, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &imuf.accel.y, sizeof(float));
  xgear_index += sizeof(float);
  // Acceleration x, float
  memcpy(&XGEAR_STATUS_buf[xgear_index], &imuf.accel.z, sizeof(float));
  xgear_index += sizeof(float);
*/
  
  //DUMMY
  xgear_index++;

  // calculate checksum & send
  xgear_cksum(XGEAR_PACKET_SIZE_STATUS, (uint8_t *)XGEAR_STATUS_buf, &cksum0, &cksum1 );
  XGEAR_STATUS_buf[XGEAR_PACKET_SIZE_STATUS-2] = cksum0;
  XGEAR_STATUS_buf[XGEAR_PACKET_SIZE_STATUS-1] = cksum1;
  uart_transmit_buffer(&XGEAR_PORT, XGEAR_STATUS_buf, (size_t) XGEAR_PACKET_SIZE_STATUS);
}


/**
 * Calculate checksum
 */
void xgear_cksum(uint16_t packet_length, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 ) {
  static uint8_t c0, c1;
  static uint16_t i, size;
  c0 = 0;
  c1 = 0;
  i = 0;
  size = packet_length - 2;

  for ( i = 0; i < size; i++ ) {
  	c0 += (uint8_t)buf[i];
    	c1 += c0;
  }
  *cksum0 = c0;
  *cksum1 = c1;
}
