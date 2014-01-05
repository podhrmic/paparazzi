/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef TELEMETRY_H
#define TELEMETRY_H



#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "subsystems/electrical.h"

#include "generated/periodic_telemetry.h"
#include "generated/airframe.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"

#if USE_GPS
#include "subsystems/gps.h"
#endif

#if RADIO_CONTROL
#include "subsystems/radio_control.h"
#endif


/********************************************************
 *
 *                  COMMANDS
 *
 ******************************************************/
#define PERIODIC_SEND_COMMANDS(_trans, _dev) DOWNLINK_SEND_COMMANDS(_trans, _dev, COMMANDS_NB, commands)

/********************************************************
 *
 *                  ACTUATORS
 *
 ******************************************************/
#ifdef ACTUATORS
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) DOWNLINK_SEND_ACTUATORS(_trans, _dev, ACTUATORS_NB, actuators)
#else
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) {}
#endif

/********************************************************
 *
 *                  RC / PPM
 *
 ******************************************************/
#ifdef RADIO_CONTROL
#define PERIODIC_SEND_RC(_trans, _dev) DOWNLINK_SEND_RC(_trans, _dev, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#else /* ! RADIO_CONTROL */
#define PERIODIC_SEND_RC(_trans, _dev) {}
#endif

#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM(_trans, _dev) {                             \
    uint16_t ppm_pulses_usec[RADIO_CONTROL_NB_CHANNEL];        \
    for (int i=0;i<RADIO_CONTROL_NB_CHANNEL;i++)               \
      ppm_pulses_usec[i] = USEC_OF_RC_PPM_TICKS(ppm_pulses[i]); \
    DOWNLINK_SEND_PPM(_trans, _dev,                                   \
                      &radio_control.frame_rate,               \
                      PPM_NB_CHANNEL,                          \
                      ppm_pulses_usec);                        \
  }  
#else
#define PERIODIC_SEND_PPM(_trans, _dev) {}
#endif

/********************************************************
 *
 *                  FBW_STATUS
 *
 ******************************************************/
#if RADIO_CONTROL
#define PERIODIC_SEND_FBW_STATUS(_trans, _dev) {        \
  DOWNLINK_SEND_FBW_STATUS(_trans, _dev,                \
                           &(radio_control.status),     \
                           &(radio_control.frame_rate), \
                           &autopilot_mode,             \
                           &electrical.vsupply,         \
                           &electrical.current);        \
  }
#else
#define PERIODIC_SEND_FBW_STATUS(_trans, _dev) {        \
  int16_t foo = -42;                                    \
  DOWNLINK_SEND_FBW_STATUS(_trans, _dev,                \
                           &(foo),                      \
                           &(foo),                      \
                           &autopilot_mode,             \
                           &electrical.vsupply,         \
                           &electrical.current);        \
  }
#endif


/********************************************************
 *
 *                  PPRZ_MODE
 *
 ******************************************************/
#define PERIODIC_SEND_PPRZ_MODE(_trans, _dev) {                         \
    uint8_t rc_settings_mode_none = 0;                                  \
    uint8_t dummy_mode = 0;                                             \
    uint8_t dummy_mcu_status = 0;                                       \
    DOWNLINK_SEND_PPRZ_MODE(_trans, _dev,                               \
                            &autopilot_mode,                            \
                            &dummy_mode,                                \
                            &lateral_mode,                              \
                            &horizontal_mode,                           \
                            &rc_settings_mode_none,                     \
                            &dummy_mcu_status);                         \
  }

/********************************************************
 *
 *                  NAVIGATION
 *
 ******************************************************/
#define PERIODIC_SEND_NAVIGATION(_trans, _dev) { \
    uint8_t _circle_count = NavCircleCount(); \
    struct EnuCoor_f* pos = stateGetPositionEnu_f(); \
    DOWNLINK_SEND_NAVIGATION(_trans, _dev, &nav_block, &nav_stage, &(pos->x), &(pos->y), &dist2_to_wp, &dist2_to_home, &_circle_count, &nav_oval_count); \
    }
#define PERIODIC_SEND_NAVIGATION_REF(_trans, _dev)  DOWNLINK_SEND_NAVIGATION_REF(_trans, _dev, &nav_utm_east0, &nav_utm_north0, &nav_utm_zone0);

/********************************************************
 *
 *                  ATTITUDE
 *
 ******************************************************/
#define PERIODIC_SEND_ATTITUDE(_trans, _dev) { \
    struct FloatEulers* att = stateGetNedToBodyEulers_f(); \
    DOWNLINK_SEND_ATTITUDE(_trans, _dev, &(att->phi), &(att->psi), &(att->theta)); \
}

/********************************************************
 *
 *                  ESTIMATOR
 *
 ******************************************************/
#define PERIODIC_SEND_ESTIMATOR(_trans, _dev) DOWNLINK_SEND_ESTIMATOR(_trans, _dev, &(stateGetPositionUtm_f()->alt), &(stateGetSpeedEnu_f()->z))

/********************************************************
 *
 *                  ESTIMATOR
 *
 ******************************************************/
#define PERIODIC_SEND_ENERGY(_trans, _dev) {                    \
  const int16_t energy_dummy = 123;                                     \
  const float vsup = ((float)electrical.vsupply) / 10.0f;       \
  const float curs = ((float) electrical.current)/1000.0f;      \
  const float power = vsup * curs;                              \
  DOWNLINK_SEND_ENERGY(_trans, _dev, &vsup, &curs, &energy_dummy, &power);\
  }


/********************************************************
 *
 *                  WP MOVED
 *
 ******************************************************/
/*
#define PERIODIC_SEND_WP_MOVED(_trans, _dev) {					\
    static uint8_t i;							\
    i++; if (i >= nb_waypoint) i = 0;					\
    DOWNLINK_SEND_WP_MOVED_ENU(_trans, _dev,					\
                   &i,					\
                   &(waypoints[i].x),			\
                   &(waypoints[i].y),			\
                   &(waypoints[i].z));			\
  }
*/
#define DownlinkSendWp(_trans, _dev, i) {	     \
  float x = nav_utm_east0 +  waypoints[i].x; \
  float y = nav_utm_north0 + waypoints[i].y; \
  DOWNLINK_SEND_WP_MOVED(_trans, _dev, &i, &x, &y, &(waypoints[i].a),&nav_utm_zone0); \
}
#define PERIODIC_SEND_WP_MOVED(_trans, _dev) { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DownlinkSendWp(_trans, _dev, i);	    \
}


/********************************************************
 *
 *              CALIATION / CIRCLE / SEGMENT / SURVEY
 *
 ******************************************************/
#define PERIODIC_SEND_CALIBRATION(_trans, _dev) DOWNLINK_SEND_CALIBRATION(_trans, _dev, &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode)

#define PERIODIC_SEND_CIRCLE(_trans, _dev) if (nav_in_circle) { DOWNLINK_SEND_CIRCLE(_trans, _dev, &nav_circle_x, &nav_circle_y, &nav_circle_radius); }

#define PERIODIC_SEND_SEGMENT(_trans, _dev) if (nav_in_segment) { DOWNLINK_SEND_SEGMENT(_trans, _dev, &nav_segment_x_1, &nav_segment_y_1, &nav_segment_x_2, &nav_segment_y_2); }

#define PERIODIC_SEND_SURVEY(_trans, _dev) { \
  if (nav_survey_active) \
    DOWNLINK_SEND_SURVEY(_trans, _dev, &nav_survey_east, &nav_survey_north, &nav_survey_west, &nav_survey_south); \
  }

/********************************************************
 *
 *                  IMU ACCEL/GYRO
 *
 ******************************************************/
#define PERIODIC_SEND_IMU_GYRO_SCALED(_trans, _dev) {		\
    DOWNLINK_SEND_IMU_GYRO_SCALED(_trans, _dev,			\
                 &imu.gyro.p,		\
                 &imu.gyro.q,		\
                 &imu.gyro.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_SCALED(_trans, _dev) {			\
    DOWNLINK_SEND_IMU_ACCEL_SCALED(_trans, _dev,				\
                  &imu.accel.x,		\
                  &imu.accel.y,		\
                  &imu.accel.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_SCALED(_trans, _dev) {			\
    DOWNLINK_SEND_IMU_MAG_SCALED(_trans, _dev,				\
                &imu.mag.x,			\
                &imu.mag.y,			\
                &imu.mag.z);			\
  }

#define PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_GYRO_RAW(_trans, _dev,					\
                   &imu.gyro_unscaled.p,		\
                   &imu.gyro_unscaled.q,		\
                   &imu.gyro_unscaled.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_ACCEL_RAW(_trans, _dev,					\
                &imu.accel_unscaled.x,		\
                &imu.accel_unscaled.y,		\
                &imu.accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_MAG_RAW(_trans, _dev,					\
                  &imu.mag_unscaled.x,			\
                  &imu.mag_unscaled.y,			\
                  &imu.mag_unscaled.z);		\
  }


/********************************************************
 *
 *                  STATE FILTER
 *
 ******************************************************/
// FIXME: legacy bullshit (who uses infrared>)  
#if USE_INFRARED
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) {                             \
  uint16_t contrast = abs(infrared.roll) + abs(infrared.pitch) + abs(infrared.top);   \
  uint8_t mde = 3;                                                                    \
  if (contrast < 50) mde = 7;                                                         \
  DOWNLINK_SEND_STATE_FILTER_STATUS(_trans, _dev, &mde, &contrast);                   \
  }
#elif USE_IMU && USE_AHRS
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) {                             \
        uint8_t ahrs_timeout_counter_dummy = 0;                                       \
        uint8_t mde = 3;                                                              \
        if (ahrs.status == AHRS_UNINIT) mde = 2;                                      \
        if (ahrs_timeout_counter_dummy > 10) mde = 5;                                 \
        uint16_t val = 0;                                                             \
        DOWNLINK_SEND_STATE_FILTER_STATUS(_trans, _dev, &mde, &val);                  \
        }
#else
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) {}
#endif


/********************************************************
 *
 *                  BATTERY
 *
 ******************************************************/
#define PERIODIC_SEND_BAT(_trans, _dev) {                      \
    int16_t amps = (int16_t) (electrical.current/10);				\
    { float e = -42;                      \
        DOWNLINK_SEND_BAT(_trans, _dev,                        \
                          &v_ctl_throttle_slewed,       \
                          &electrical.vsupply,                     \
                          &amps,                        \
                          &autopilot_flight_time,       \
                          &kill_throttle,				\
                          &block_time,					\
                          &stage_time,					\
                          &e);                          \
      };                                               \
  }


/********************************************************
 *
 *                  BATTERY MONITOR
 *
 ******************************************************/
#ifdef USE_BATTERY_MONITOR
#define PERIODIC_SEND_BATTERY_MONITOR(_trans, _dev) DOWNLINK_SEND_BATTERY_MONITOR(_trans, _dev,\
    &bus_current,                                                                              \
    &bus_voltage,                                                                              \
    &measured_current,                                                                              \
    &charge_integrated,                                                                              \
    &charge_remaining,                                                                              \
    &ad7997_trans_status_bus,                                                                              \
    &ad7997_trans_status_balancer,                                                                              \
    2*BATTERY_CELL_NB, balancer_ports);
#else
#define PERIODIC_SEND_BATTERY_MONITOR(_trans, _dev) {}
#endif



/********************************************************
 *
 *                  GPS
 *
 ******************************************************/
#if USE_GPS
#define PERIODIC_SEND_GPS(_trans, _dev) {                               \
    static uint8_t i;                                                   \
    int16_t climb = -gps.ned_vel.z;                                     \
    int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));             \
    DOWNLINK_SEND_GPS(_trans, _dev,                                     \
                      &gps.fix,                                         \
                      &gps.utm_pos.east,                                \
                      &gps.utm_pos.north,                               \
                      &course, &gps.hmsl,                               \
                      &gps.gspeed,                                      \
                      &climb,                                           \
                      &gps.week,                                        \
                      &gps.tow,                                         \
                      &gps.utm_pos.zone,                                \
                      &i);                                              \
    if ((gps.fix != GPS_FIX_3D) && (i >= gps.nb_channels)) i = 0;       \
    if (i >= gps.nb_channels * 2) i = 0;                                \
    if (i < gps.nb_channels && ((gps.fix != GPS_FIX_3D) || (gps.svinfos[i].cno > 0))) { \
      DOWNLINK_SEND_SVINFO(_trans, _dev,                                \
                           &i,                                          \
                           &gps.svinfos[i].svid,                        \
                           &gps.svinfos[i].flags,                       \
                           &gps.svinfos[i].qi,                          \
                           &gps.svinfos[i].cno,                         \
                           &gps.svinfos[i].elev,                        \
                           &gps.svinfos[i].azim);                       \
    }                                                                   \
    i++;                                                                \
  }
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {   \
  DOWNLINK_SEND_GPS_INT( _trans, _dev,          \
                         &gps.ecef_pos.x,       \
                         &gps.ecef_pos.y,       \
                         &gps.ecef_pos.z,       \
                         &gps.lla_pos.lat,      \
                         &gps.lla_pos.lon,      \
                         &gps.lla_pos.alt,      \
                         &gps.hmsl,             \
                         &gps.ecef_vel.x,       \
                         &gps.ecef_vel.y,       \
                         &gps.ecef_vel.z,       \
                         &gps.pacc,             \
                         &gps.sacc,             \
                         &gps.tow,              \
                         &gps.pdop,             \
                         &gps.num_sv,           \
                         &gps.fix);             \
  }
#else
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {}
#define PERIODIC_SEND_GPS(_trans, _dev) {}
#endif

#if USE_GPS || defined SITL
#define PERIODIC_SEND_GPS_SOL(_trans, _dev) DOWNLINK_SEND_GPS_SOL(_trans, _dev, &gps.pacc, &gps.sacc, &gps.pdop, &gps.num_sv)
#else
#define PERIODIC_SEND_GPS_SOL(_trans, _dev) {}
#endif



/********************************************************
 *
 *                  CHIBIOS INFO
 *
 ******************************************************/
#ifdef USE_CHIBIOS_RTOS
#include "mcu_periph/sys_time_arch.h"
#define PERIODIC_SEND_CHIBIOS_INFO(_trans, _dev) {  \
  static uint32_t time_now = 0;                     \
  time_now = chTimeNow()/CH_FREQUENCY;              \
  DOWNLINK_SEND_CHIBIOS_INFO(_trans, _dev,          \
	&core_free_memory,                                \
	&time_now,                                        \
	&thread_counter,                                  \
	&cpu_frequency,                                   \
	&electrical.cpu_temp);                            \
  }
#else
#define PERIODIC_SEND_CHIBIOS_INFO(_trans, _dev) {}
#endif


/********************************************************
 *
 *                  DOWNLINK/VALUE/ALIVE
 *
 ******************************************************/
#define PERIODIC_SEND_DOWNLINK(_trans, _dev) { \
  static uint16_t last; \
  uint16_t rate = (downlink_nb_bytes - last) / PERIOD_DOWNLINK_Main_0; \
  last = downlink_nb_bytes; \
  DOWNLINK_SEND_DOWNLINK(_trans, _dev, &downlink_nb_ovrn, &rate, &downlink_nb_msgs); \
}

#include "generated/settings.h"
#define PERIODIC_SEND_DL_VALUE(_trans, _dev) PeriodicSendDlValue(_trans, _dev)

#define PERIODIC_SEND_ALIVE(_trans, _dev)  DOWNLINK_SEND_ALIVE(_trans, _dev, 16, MD5SUM);



/********************************************************
 *
 *                  I2C
 *
 ******************************************************/
#ifdef USE_I2C0
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {                       \
    uint16_t i2c0_queue_full_cnt        = i2c0.errors->queue_full_cnt;  \
    uint16_t i2c0_ack_fail_cnt          = i2c0.errors->ack_fail_cnt;    \
    uint16_t i2c0_miss_start_stop_cnt   = i2c0.errors->miss_start_stop_cnt; \
    uint16_t i2c0_arb_lost_cnt          = i2c0.errors->arb_lost_cnt;    \
    uint16_t i2c0_over_under_cnt        = i2c0.errors->over_under_cnt;  \
    uint16_t i2c0_pec_recep_cnt         = i2c0.errors->pec_recep_cnt;   \
    uint16_t i2c0_timeout_tlow_cnt      = i2c0.errors->timeout_tlow_cnt; \
    uint16_t i2c0_smbus_alert_cnt       = i2c0.errors->smbus_alert_cnt; \
    uint16_t i2c0_unexpected_event_cnt  = i2c0.errors->unexpected_event_cnt; \
    uint32_t i2c0_last_unexpected_event = i2c0.errors->last_unexpected_event; \
    const uint8_t _bus0 = 0;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c0_queue_full_cnt,                      \
                             &i2c0_ack_fail_cnt,                        \
                             &i2c0_miss_start_stop_cnt,                 \
                             &i2c0_arb_lost_cnt,                        \
                             &i2c0_over_under_cnt,                      \
                             &i2c0_pec_recep_cnt,                       \
                             &i2c0_timeout_tlow_cnt,                    \
                             &i2c0_smbus_alert_cnt,                     \
                             &i2c0_unexpected_event_cnt,                \
                             &i2c0_last_unexpected_event,               \
                             &_bus0);                                   \
  }
#else
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C1
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {                       \
    uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;  \
    uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;    \
    uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt; \
    uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;    \
    uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;  \
    uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;   \
    uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt; \
    uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt; \
    uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt; \
    uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event; \
    const uint8_t _bus1 = 1;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c1_queue_full_cnt,                      \
                             &i2c1_ack_fail_cnt,                        \
                             &i2c1_miss_start_stop_cnt,                 \
                             &i2c1_arb_lost_cnt,                        \
                             &i2c1_over_under_cnt,                      \
                             &i2c1_pec_recep_cnt,                       \
                             &i2c1_timeout_tlow_cnt,                    \
                             &i2c1_smbus_alert_cnt,                     \
                             &i2c1_unexpected_event_cnt,                \
                             &i2c1_last_unexpected_event,               \
                             &_bus1);                                   \
  }
#else
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C2
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {                       \
    uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;  \
    uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;    \
    uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt; \
    uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;    \
    uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;  \
    uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;   \
    uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt; \
    uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt; \
    uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt; \
    uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event; \
    const uint8_t _bus2 = 2;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c2_queue_full_cnt,                      \
                             &i2c2_ack_fail_cnt,                        \
                             &i2c2_miss_start_stop_cnt,                 \
                             &i2c2_arb_lost_cnt,                        \
                             &i2c2_over_under_cnt,                      \
                             &i2c2_pec_recep_cnt,                       \
                             &i2c2_timeout_tlow_cnt,                    \
                             &i2c2_smbus_alert_cnt,                     \
                             &i2c2_unexpected_event_cnt,                \
                             &i2c2_last_unexpected_event,               \
                             &_bus2);                                   \
  }
#else
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C3
#define PERIODIC_SEND_I2C3_ERRORS(_trans, _dev) {                       \
    uint16_t i2c3_queue_full_cnt        = i2c3.errors->queue_full_cnt;  \
    uint16_t i2c3_ack_fail_cnt          = i2c3.errors->ack_fail_cnt;    \
    uint16_t i2c3_miss_start_stop_cnt   = i2c3.errors->miss_start_stop_cnt; \
    uint16_t i2c3_arb_lost_cnt          = i2c3.errors->arb_lost_cnt;    \
    uint16_t i2c3_over_under_cnt        = i2c3.errors->over_under_cnt;  \
    uint16_t i2c3_pec_recep_cnt         = i2c3.errors->pec_recep_cnt;   \
    uint16_t i2c3_timeout_tlow_cnt      = i2c3.errors->timeout_tlow_cnt; \
    uint16_t i2c3_smbus_alert_cnt       = i2c3.errors->smbus_alert_cnt; \
    uint16_t i2c3_unexpected_event_cnt  = i2c3.errors->unexpected_event_cnt; \
    uint32_t i2c3_last_unexpected_event = i2c3.errors->last_unexpected_event; \
    const uint8_t _bus3 = 3;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c3_queue_full_cnt,                      \
                             &i2c3_ack_fail_cnt,                        \
                             &i2c3_miss_start_stop_cnt,                 \
                             &i2c3_arb_lost_cnt,                        \
                             &i2c3_over_under_cnt,                      \
                             &i2c3_pec_recep_cnt,                       \
                             &i2c3_timeout_tlow_cnt,                    \
                             &i2c3_smbus_alert_cnt,                     \
                             &i2c3_unexpected_event_cnt,                \
                             &i2c3_last_unexpected_event,               \
                             &_bus3);                                   \
  }
#else
#define PERIODIC_SEND_I2C3_ERRORS(_trans, _dev) {}
#endif

#ifndef SITL
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {        \
    static uint8_t _i2c_nb_cnt = 0;                     \
    switch (_i2c_nb_cnt) {                              \
      case 0:                                           \
        PERIODIC_SEND_I2C0_ERRORS(_trans, _dev); break; \
      case 1:                                           \
        PERIODIC_SEND_I2C1_ERRORS(_trans, _dev); break; \
      case 2:                                           \
        PERIODIC_SEND_I2C2_ERRORS(_trans, _dev); break; \
      case 3:                                           \
        PERIODIC_SEND_I2C3_ERRORS(_trans, _dev); break; \
      default:                                          \
        break;                                          \
    }                                                   \
    _i2c_nb_cnt++;                                      \
    if (_i2c_nb_cnt == 3)                               \
      _i2c_nb_cnt = 0;                                  \
  }
#else
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {}
#endif


/********************************************************
 *
 *                  UART ERRORS
 *
 ******************************************************/
#ifdef USE_UART1
#define PERIODIC_SEND_UART1_ERRORS(_trans, _dev) {   \
    const uint8_t _bus1 = 1;                         \
    uint16_t ore = uart1.ore;                        \
    uint16_t ne_err = uart1.ne_err;                  \
    uint16_t fe_err = uart1.fe_err;                  \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &ore,                   \
                             &ne_err,                \
                             &fe_err,                \
                             &_bus1);                \
  }
#else
#define PERIODIC_SEND_UART1_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART2
#define PERIODIC_SEND_UART2_ERRORS(_trans, _dev) {   \
    const uint8_t _bus2 = 2;                         \
    uint16_t ore = uart2.ore;                        \
    uint16_t ne_err = uart2.ne_err;                  \
    uint16_t fe_err = uart2.fe_err;                  \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &ore,                   \
                             &ne_err,                \
                             &fe_err,                \
                             &_bus2);                \
  }
#else
#define PERIODIC_SEND_UART2_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART3
#define PERIODIC_SEND_UART3_ERRORS(_trans, _dev) {   \
    const uint8_t _bus3 = 3;                         \
    uint16_t ore = uart3.ore;                        \
    uint16_t ne_err = uart3.ne_err;                  \
    uint16_t fe_err = uart3.fe_err;                  \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &ore,                   \
                             &ne_err,                \
                             &fe_err,                \
                             &_bus3);                \
  }
#else
#define PERIODIC_SEND_UART3_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART5
#define PERIODIC_SEND_UART5_ERRORS(_trans, _dev) {   \
    const uint8_t _bus5 = 5;                         \
    uint16_t ore = uart5.ore;                        \
    uint16_t ne_err = uart5.ne_err;                  \
    uint16_t fe_err = uart5.fe_err;                  \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &ore,                   \
                             &ne_err,                \
                             &fe_err,                \
                             &_bus5);                \
  }
#else
#define PERIODIC_SEND_UART5_ERRORS(_trans, _dev) {}
#endif


#ifndef SITL
#define PERIODIC_SEND_UART_ERRORS(_trans, _dev) {           \
    static uint8_t uart_nb_cnt = 0;                         \
    switch (uart_nb_cnt) {                                  \
      case 0:                                               \
        PERIODIC_SEND_UART1_ERRORS(_trans, _dev); break;    \
      case 1:                                               \
        PERIODIC_SEND_UART2_ERRORS(_trans, _dev); break;    \
      case 2:                                               \
        PERIODIC_SEND_UART3_ERRORS(_trans, _dev); break;    \
      case 3:                                               \
        PERIODIC_SEND_UART5_ERRORS(_trans, _dev); break;    \
      default: break;                                       \
    }                                                       \
    uart_nb_cnt++;                                          \
    if (uart_nb_cnt == 4)                                   \
      uart_nb_cnt = 0;                                      \
  }
#else
#define PERIODIC_SEND_UART_ERRORS(_trans, _dev) {}
#endif

#endif /* TELEMETRY_H */
