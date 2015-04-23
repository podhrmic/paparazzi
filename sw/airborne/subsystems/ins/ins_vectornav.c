/*
 * Copyright (C) 2014 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ins_vectornav.c
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/ins/ins_vectornav.h"

#include "subsystems/abi.h"

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"



#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif


#if USE_CHIBIOS_RTOS
  Mutex ins_get_data_flag;
#endif

static inline void ins_set_pacc(void);
static inline void ins_set_sacc(void);
static inline void ins_check_status(void);

struct InsInt ins_impl;

uint16_t calc_chk;
uint16_t rec_chk;
uint16_t counter;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_impl.ltp_pos.x, &ins_impl.ltp_pos.y, &ins_impl.ltp_pos.z,
                    &ins_impl.ltp_speed.x, &ins_impl.ltp_speed.y, &ins_impl.ltp_speed.z,
                    &ins_impl.ltp_accel.x, &ins_impl.ltp_accel.y, &ins_impl.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_impl.baro_z, &ins_impl.ltp_pos.z, &ins_impl.ltp_speed.z, &ins_impl.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_impl.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_impl.ltp_def.ecef.x, &ins_impl.ltp_def.ecef.y, &ins_impl.ltp_def.ecef.z,
                          &ins_impl.ltp_def.lla.lat, &ins_impl.ltp_def.lla.lon, &ins_impl.ltp_def.lla.alt,
                          &ins_impl.ltp_def.hmsl, &ins_impl.qfe);
  }
}

static void send_vn_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VECTORNAV_INFO(trans, dev, AC_ID,
      &ins_impl.timestamp,
      &ins_impl.vn_packet.chksm_error,
      &ins_impl.vn_packet.hdr_error,
      &counter,
      &ins_impl.mode,
      &ins_impl.err,
      &ins_impl.YprU.phi,
      &ins_impl.YprU.theta,
      &ins_impl.YprU.psi);
}

static void send_vn_msg(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t vnstatus = (uint16_t)ins_impl.mode;
  struct FloatEulers* attitude;
  attitude = stateGetNedToBodyEulers_f();
  pprz_msg_send_VECTORNAV_MSG(trans, dev, AC_ID,
      &ins_impl.timestamp,
      &attitude->phi,
      &attitude->theta,
      &attitude->psi,
      &imuf.gyro.p,
      &imuf.gyro.q,
      &imuf.gyro.r,
      &ins_impl.pos_lla[0],
      &ins_impl.pos_lla[1],
      &ins_impl.pos_lla[2],
      &ins_impl.vel_ned.x,
      &ins_impl.vel_ned.y,
      &ins_impl.vel_ned.z,
      &imuf.accel.x,
      &imuf.accel.y,
      &imuf.accel.z,
      &gps.num_sv,
      &gps.fix,
      &ins_impl.posU[0],
      &ins_impl.posU[1],
      &ins_impl.posU[2],
      &ins_impl.velU,
      &ins_impl.ltp_accel_f.x,
      &ins_impl.ltp_accel_f.y,
      &ins_impl.ltp_accel_f.z,
      &ins_impl.YprU.phi,
      &ins_impl.YprU.theta,
      &ins_impl.YprU.psi,
      &vnstatus
      );
}
#endif


/** INS initialization. Called at startup.
 *  Needs to be implemented by each INS algorithm.
 */
void ins_init(void) {
  ins.status = INS_UNINIT;

#if USE_CHIBIOS_RTOS
  chMtxInit(&ins_get_data_flag);
#endif /* USE_CHIBIOS_RTOS */

  // Initialize variables
  ins_impl.vn_status = VNNotTracking;
  ins_impl.vn_time = get_sys_time_float();

  // Initialize packet
  ins_impl.vn_packet.status = VNMsgSync;
  ins_impl.vn_packet.msg_idx = 0;
  ins_impl.vn_packet.msg_available = FALSE;
  ins_impl.vn_packet.chksm_error = 0;
  ins_impl.vn_packet.hdr_error = 0;
  ins_impl.vn_packet.overrun_error = 0;
  ins_impl.vn_packet.noise_error = 0;
  ins_impl.vn_packet.framing_error = 0;

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_impl.ltp_initialized = TRUE;
#else
  TODO("Warning: USE_INS_NAV_INIT set to FALSE, untested behavior!");
  ins_impl.ltp_initialized  = FALSE;
#endif

  INT32_VECT3_ZERO(ins_impl.ltp_pos);
  INT32_VECT3_ZERO(ins_impl.ltp_speed);
  INT32_VECT3_ZERO(ins_impl.ltp_accel);

  FLOAT_VECT3_ZERO(ins_impl.vel_ned);
  FLOAT_VECT3_ZERO(ins_impl.lin_accel);
  FLOAT_VECT3_ZERO(ins_impl.vel_body);

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "INS", send_ins);
  register_periodic_telemetry(DefaultPeriodic, "INS_Z", send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_INFO", send_vn_info);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_MSG", send_vn_msg);
#endif
}

/** INS periodic call.
 *  Needs to be implemented by each INS algorithm.
 */
void ins_periodic(void) {
  ins_impl.vn_ltime = get_sys_time_float();//chTimeNow();
  ins_impl.vn_freq =  1.0 / ((float)(ins_impl.vn_ltime - ins_impl.vn_time));
  // check when was last time we had a packet (using frequency)
  /*
  if (ins_impl.vn_freq < VN_MIN_FREQ) {
    ins_impl.vn_freq = 0.0;
    ins.status = INS_UNINIT;
  }
  */


  //if (ins.status == INS_RUNNING) {
    // update internal states for telemetry purposes
      ins_impl.ltp_pos = *stateGetPositionNed_i();
      ins_impl.ltp_speed = *stateGetSpeedNed_i();
      ins_impl.ltp_accel = *stateGetAccelNed_i();
 // }
}

/**
 * Convert yaw, pitch, and roll data from VectorNav
 * to correct attitude
 * yaw(0), pitch(1), roll(2) -> phi, theta, psi
 * [deg] -> rad
 */
static inline void ins_yawPitchRoll_to_Attitude(struct FloatEulers* vn_attitude) {
  static struct FloatEulers att_rad;
  att_rad.phi = (vn_attitude->psi)*DEG_TO_RAD;
  att_rad.theta = (vn_attitude->theta)*DEG_TO_RAD;
  att_rad.psi = (vn_attitude->phi)*DEG_TO_RAD;

  vn_attitude->phi = att_rad.phi;
  vn_attitude->theta = att_rad.theta;
  vn_attitude->psi = att_rad.psi;
}

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Needs to be implemented by each INS algorithm.
 */
void ins_propagate(float dt) {
  (void) dt;

  // Acceleration [m/s^2]
  ACCELS_BFP_OF_REAL(imu.accel, imuf.accel); // for backwards compatibility with fixed point interface

  // Rates [rad/s]
  static struct FloatRates body_rate;
  RATES_BFP_OF_REAL(imu.gyro, imuf.gyro ); // for backwards compatibility with fixed point interface
  FLOAT_RMAT_RATEMULT(body_rate, imuf.body_to_imu.rmat_f, imuf.gyro); // compute body rates
  stateSetBodyRates_f(&body_rate);   // Set state [rad/s]

  // Attitude [deg]
  ins_yawPitchRoll_to_Attitude(&ins_impl.attitude); // convert to correct units and axis [rad]
  static struct FloatQuat imu_quat; // convert from euler to quat
  FLOAT_QUAT_OF_EULERS(imu_quat, ins_impl.attitude);
  static struct FloatRMat imu_rmat; // convert from quat to rmat
  FLOAT_RMAT_OF_QUAT(imu_rmat, imu_quat);
  static struct FloatRMat ltp_to_body_rmat; // rotate to body frame
  static struct FloatRMat* body_to_imu_rmat; // rotate to body frame
  body_to_imu_rmat = orientationGetRMat_f(&(imuf.body_to_imu));
  FLOAT_RMAT_COMP(ltp_to_body_rmat, imu_rmat, *body_to_imu_rmat);
  stateSetNedToBodyRMat_f(&ltp_to_body_rmat); // set body states [rad]


  // NED (LTP) velocity [m/s]
  // North east down (NED), also known as local tangent plane (LTP),
  // is a geographical coordinate system for representing state vectors that is commonly used in aviation.
  // It consists of three numbers: one represents the position along the northern axis,
  // one along the eastern axis, and one represents vertical position. Down is chosen as opposed to
  // up in order to comply with the right-hand rule.
  // The origin of this coordinate system is usually chosen to be the aircraft's center of gravity.
  // x = North
  // y = East
  // z = Down
  stateSetSpeedNed_f(&ins_impl.vel_ned); // set state
  NED_BFP_OF_REAL(gps.ned_vel, ins_impl.vel_ned); //gps vel ned is in cm/s
  // climb = -gps.ned_vel.z;

  // FIXME: this should rotate from body fram to NED, shouldnt it?
  // NED (LTP) acceleration [m/s^2]
  static struct FloatVect3 accel_meas_ltp;// first we need to rotate linear acceleration from imu-frame to body-frame
  RMAT_VECT3_TRANSP_MUL(accel_meas_ltp, imuf.body_to_imu.rmat_f, ins_impl.lin_accel);
  static struct NedCoor_f ltp_accel; // assign to NedCoord_f struct
  VECT3_ASSIGN(ltp_accel, accel_meas_ltp.x, accel_meas_ltp.y, accel_meas_ltp.z);
  stateSetAccelNed_f(&ltp_accel); // then set the states
  ins_impl.ltp_accel_f = ltp_accel;

  // LLA position [rad, rad, m]
  // convert from deg to rad, and from double to float
  ins_impl.lla_pos.lat = ((float)ins_impl.pos_lla[0])*DEG_TO_RAD; // ins_impl.pos_lla[0] = lat
  ins_impl.lla_pos.lon = ((float)ins_impl.pos_lla[1])*DEG_TO_RAD; // ins_impl.pos_lla[1] = lon
  ins_impl.lla_pos.alt = ((float)ins_impl.pos_lla[2]); // ins_impl.pos_lla[2] = alt
  stateSetPositionLla_f(&ins_impl.lla_pos); // then set the states

  // fill in gps variables
  LLA_BFP_OF_REAL(gps.lla_pos, ins_impl.lla_pos);

  // fill in GPS message variables (ECEF is needed for correct display of AC in GCS map)
  ecef_of_lla_i(&gps.ecef_pos, &gps.lla_pos);


  // ECEF velocity
  /*
  struct LtpDef_f def;
  ltp_def_from_lla_f(&def, &ins_impl.lla_pos);
  struct EcefCoor_f ecef_vel;
  ecef_of_ned_point_f(&ecef_vel, &def, &ins_impl.vel_ned);
  struct EcefCoor_f ecef_vel_cm;
  ECEF_BFP_OF_REAL(gps.ecef_vel, ecef_vel_cm);
  //stateSetSpeedEcef_i(&gps.ecef_vel);
  */

/*
  struct EcefCoor_i *ecef_vel;
  ecef_vel =  stateGetSpeedEcef_i();

  gps.ecef_vel.x = (ecef_vel->x)/100.0;
  gps.ecef_vel.y = (ecef_vel->y);
  gps.ecef_vel.z = (ecef_vel->z);
  */


  //FIXME: do we need utm?
  // plus it doent reallt work
  // GPS UTM
  // Computes from (lat, long) in the referenced UTM zone
  /*
  struct LlaCoor_f lla_f;
  lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
  lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
  struct UtmCoor_f utm_f;
  utm_f.zone = NAV_UTM_ZONE0;//nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  gps.utm_pos.east = utm_f.east*100;
  gps.utm_pos.north = utm_f.north*100;
  gps.utm_pos.alt = utm_f.alt*1000;
  gps.utm_pos.zone = NAV_UTM_ZONE0;//nav_utm_zone0;
  */



  float speed = sqrt(ins_impl.vel_ned.x*ins_impl.vel_ned.x + ins_impl.vel_ned.y*ins_impl.vel_ned.y);
  gps.gspeed = ((uint16_t)(speed*100));

  /*
   * In order to correctly display altitude at GCS
   * we have to equal altitude and HMSL
   * VN-200 does NOT output Heigh above Mean Sea Level,
   * just altitude above reference elipsoid (although in
   * different coordinate systems).
   *
   * The actual HMSL may differ significantly (tens of meters)
   * from Altitude above Elipsoid, so I am not sure if it is OK
   * for flighs (have to be tested).
   */
  gps.hmsl = gps.lla_pos.alt;

  // set course
  //gps.course = (int32_t)(1e7*(atan2(ins_impl.vel_ned.y,ins_impl.vel_ned.x)));

  // set position uncertainty
  ins_set_pacc();

  // set velocity uncertainty
  ins_set_sacc();

  // check GPS status
  if (gps.fix == GPS_FIX_3D) {
    gps.last_msg_time = sys_time.nb_sec;
  }

  // read INS status
  ins_check_status();
}

/**
 * Check INS status
 */
static inline void ins_check_status(void) {
  ins_impl.mode = (uint8_t)(ins_impl.ins_status & 0x03);
  ins_impl.err = (uint8_t)((ins_impl.ins_status >> 3) & 0x0F);
}

/**
 * Set speed (velocity) uncertainty (NED)
 */
static inline void ins_set_sacc(void){
  // from GPS subsystem:
  //uint32_t sacc;                 ///< speed accuracy in cm/s
  gps.sacc = (uint32_t)(ins_impl.velU*100);
}

/**
 * Find maximum uncertainty (NED)
 */
static inline void ins_set_pacc(void){
  float pacc = ins_impl.posU[0]; // in meters
  if (ins_impl.posU[1] > pacc) {
    pacc = ins_impl.posU[1];
  }
  if (ins_impl.posU[2] > pacc) {
    pacc = ins_impl.posU[2];
  }

  // GPS Subsystem
  //uint32_t pacc;                 ///< position accuracy in cm
  gps.pacc = (uint32_t)(pacc*100);
}


/** initialize the local origin (ltp_def) from flight plan position */
void ins_init_origin_from_flightplan(void) {
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  //llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  // FIXME: Hack to get correct altitude in the flight plan
  llh_nav0.alt = NAV_ALT0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_impl.ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);
}


/**
 * Read received packet
 */
void vn_packet_read_message(void) {
  ins_impl.vn_time = get_sys_time_float();

  uint16_t idx = VN_HEADER_SIZE;

  // Timestamp [nanoseconds] since startup
  static uint64_t nanostamp = 0;
  memcpy(&nanostamp, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);

  // Timestamp [s]
  ins_impl.timestamp = ((float)nanostamp/1000000000); // [nanoseconds to seconds]

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  memcpy(&ins_impl.attitude, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // Rates (imu frame), float, [rad/s]
  memcpy(&imuf.gyro, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //Pos LLA, double,[deg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  memcpy(&ins_impl.pos_lla, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(double));
  idx += 3*sizeof(double);

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  memcpy(&ins_impl.vel_ned, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // Accel (imu-frame), float, [m/s^-2]
  memcpy(&imuf.accel, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // tow (in nanoseconds), uint64
  static uint64_t tow = 0;
  memcpy(&tow, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);
  tow = tow / 1000000; // nanoseconds to miliseconds
  gps.tow = (uint32_t) tow;

  //num sats, uint8
  gps.num_sv = ins_impl.vn_packet.msg_buf[idx];
  idx++;

  //gps fix, uint8
  gps.fix = ins_impl.vn_packet.msg_buf[idx];
  idx++;

  //posU, float[3]
  memcpy(&ins_impl.posU, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //velU, float
  memcpy(&ins_impl.velU, &ins_impl.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  //linear acceleration imu-body frame, float [m/s^2]
  //The estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The
  //acceleration measurement has been bias compensated by the onboard INS filter, and the gravity
  //component has been removed using the current gravity reference vector model. This measurement is
  //attitude dependent, since the attitude solution is required to map the gravity reference vector (known
  //in the inertial NED frame), into the body frame so that it can be removed from the measurement. If the
  //device is stationary and the onboard INS filter is tracking, the measurement nominally will read 0 in all
  //three axes.
  memcpy(&ins_impl.lin_accel, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //YprU, float[3]
  memcpy(&ins_impl.YprU, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //instatus, uint16
  memcpy(&ins_impl.ins_status, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint16_t));
  idx += sizeof(uint16_t);

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  memcpy(&ins_impl.vel_body, &ins_impl.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  //FIXME
  ins_propagate(0.1);

  ins.status = INS_RUNNING;
}


/*
 *  Packet Collection & state machine
 */
void vn_packet_parse( uint8_t c) {
  switch (ins_impl.vn_packet.status) {
  case VNMsgSync:
    // sync the header
    ins_impl.vn_packet.msg_idx = 0;
    if (c == VN_SYNC) {
      ins_impl.vn_packet.status = VNMsgHeader;
    } else {
      ins_impl.vn_packet.hdr_error++;
    }
    break;
  case VNMsgHeader:
    // read header data (we expect 0x39)
    if (c == VN_OUTPUT_GROUP) {
      // increment idx and save current byte for checksum
      ins_impl.vn_packet.status = VNMsgGroup;
      ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] = c;
      ins_impl.vn_packet.msg_idx++;
    }
    else {
      ins_impl.vn_packet.hdr_error++;
      ins_impl.vn_packet.status = VNMsgSync;
    }
    break;
    break;
  case VNMsgGroup:
    // read header data
    ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] = c;
    ins_impl.vn_packet.msg_idx++;
    if (ins_impl.vn_packet.msg_idx == VN_GROUP_BYTES) {
      ins_impl.vn_packet.datalength = VN_PAYLOAD_SIZE+VN_HEADER_SIZE;
      ins_impl.vn_packet.status = VNMsgData;
    }
    break;
  case VNMsgData:
    ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] =  c;
    ins_impl.vn_packet.msg_idx++;
    if (ins_impl.vn_packet.msg_idx == (ins_impl.vn_packet.datalength+2)) {
      if (verify_chk(ins_impl.vn_packet.msg_buf,ins_impl.vn_packet.datalength, &calc_chk, &rec_chk)) {
        ins_impl.vn_packet.msg_available = true;
        counter++;
      } else {
        ins_impl.vn_packet.msg_available = false;
        ins_impl.vn_packet.chksm_error++;
      }
      ins_impl.vn_packet.status = VNMsgSync;
    }
    break;
  default:;
    ins_impl.vn_packet.status = VNMsgSync;
    ins_impl.vn_packet.msg_idx = 0;
    break;
  }
}

#if USE_CHIBIOS_RTOS
/*
 *  INS Data Receiving Thread
 *
 *  Reads data from VectorNav
 *  Assume Vectornav already configured for a custom message
 */
__attribute__((noreturn)) msg_t thd_ins(void *args __attribute__((unused)))
{
  chRegSetThreadName("pprz_ins");

  static msg_t charbuf;
  while (TRUE) {
    charbuf = sdGet((SerialDriver*)VN_PORT.reg_addr);
    vn_packet_parse(charbuf);
    if (ins_impl.vn_packet.msg_available) {
    	chMtxLock(&ins_get_data_flag);
      vn_packet_read_message();
      ins.status = INS_RUNNING;
      chMtxUnlock();
      ins_impl.vn_packet.msg_available = FALSE;
    }
  }
}
#endif /* USE_CHIBIOS_RTOS */

