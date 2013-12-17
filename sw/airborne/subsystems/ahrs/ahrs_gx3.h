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
 * @file ahrs_gx3.h
 *
 * Driver for Microstrain GX3 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See GX3 datasheet for configuration options.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AHRS_GX3_H
#define AHRS_GX3_H

#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "mcu_periph/uart.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "state.h"
#include "led.h"

#ifdef ImuScaleGyro
#undef ImuScaleGyro
#endif
#define ImuScaleGyro(_imu) {}

#ifdef ImuScaleAccel
#undef ImuScaleAccel
#endif
#define ImuScaleAccel(_imu) {}

#ifdef ImuScaleMag
#undef ImuScaleMag
#endif
#define ImuScaleMag(_imu) {}

#define GX3_MAX_PAYLOAD 128
#define GX3_MSG_LEN 67
#define GX3_HEADER 0xC8
#define GX3_MIN_FREQ 300

#ifdef USE_CHIBIOS_RTOS
#define GX3_QUEUE_SIZE 5
#define CH_THREAD_AREA_AHRS 1024
extern Mutex ahrs_mutex_flag;
extern Mutex states_mutex_flag;
extern __attribute__((noreturn)) msg_t thd_ahrs(void *arg);
#endif

#define IMU_GX3_LONG_DELAY 8000000

extern void gx3_packet_read_message(void);
extern uint8_t gx3_packet_parse(uint8_t c);

struct GX3Packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[GX3_MAX_PAYLOAD];
  uint8_t  status;
  uint8_t  msg_idx;
};

#ifdef USE_CHIBIOS_RTOS
struct GX3Queue {
  uint8_t front;
  uint8_t rear;
  uint8_t queue_buf[GX3_QUEUE_SIZE][GX3_MSG_LEN+1];
  uint8_t status;
};
#endif

enum GX3PacketStatus {
  GX3PacketWaiting,
  GX3PacketReading,
  GX3PacketFull,
};

enum GX3Status {
  GX3Uninit,
  GX3Running
};

//AHRS
struct AhrsFloatQuat {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  float mag_offset;                    ///< Difference between true and magnetic north

  struct GX3Packet gx3_packet;       ///< Packet struct
  enum GX3Status gx3_status;          ///< GX3 status
  float gx3_freq;                     ///< data frequency
  uint16_t gx3_chksm;                 ///< aux variable for checksum
  uint32_t gx3_time;                  ///< GX3 time stamp
  uint32_t gx3_ltime;                 ///< aux time stamp

#ifdef USE_CHIBIOS_RTOS
  uint32_t ch_ltime;
  uint32_t ch_time;
  float ch_freq;

  uint8_t gx3_data_buffer[GX3_MSG_LEN+1];
  struct GX3Queue queue;
  uint16_t freq_err;
#endif

  struct FloatVect3 gx3_accel;        ///< measured acceleration in IMU frame
  struct FloatRates gx3_rate;         ///< measured angular rates in IMU frame
  struct FloatRMat  gx3_rmat;         ///< measured attitude in IMU frame (rotational matrix)
};

extern struct AhrsFloatQuat ahrs_impl;


#ifndef USE_CHIBIOS_RTOS
static inline void ReadGX3Buffer(void) {
  while (uart_char_available(&GX3_PORT) && !ahrs_impl.gx3_packet.msg_available)
    gx3_packet_parse(uart_getch(&GX3_PORT));
}

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  if (uart_char_available(&GX3_PORT)) {
    ReadGX3Buffer();
  }
  if (ahrs_impl.gx3_packet.msg_available) {
    gx3_packet_read_message();
    _gyro_handler();
    _accel_handler();
    _mag_handler();
    ahrs_impl.gx3_packet.msg_available = FALSE;
  }
}
#endif

extern struct FloatEulers ltp_to_body_eulers;

#ifdef AHRS_UPDATE_FW_ESTIMATOR
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif

#endif /* AHRS_GX3_H*/
