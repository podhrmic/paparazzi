/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/autopilot.h
 *
 * Template for autopilot (basically copies VTOL)
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

/*
 * AP modes are defined here
 */
#define AP_MODE_1          0
#define AP_MODE_2          1
#define AP_MODE_Z_HOLD
#define AP_MODE_ATTITUDE
#define AP_MODE_NAV
#define AP_MODE_N		    2


#define AP_MODE_MANUAL
#define AP_MODE_AUTO1
#define AP_MODE_AUTO2

/************ AP variables ********************/
/*
 * current ap mode
 */
extern uint8_t autopilot_mode;

/*
 * new ap mode (for switching)
 */
extern uint8_t autopilot_mode_auto2;

/*
 * motors on indicator
 */
extern bool_t autopilot_motors_on;

/*
 * in flight indicator
 */
extern bool_t autopilot_in_flight;

/*
 * kill indicator
 */
extern bool_t kill_throttle;

/*
 * a neat way how to switch on/off RC commands
 *
 * probably nicer than #ifdef RADIO_CONTORL. For Simpack
 * imagine you defined this false and updating commands will be
 * trigerred from uplink
 */
extern bool_t autopilot_rc;


/************ AP functions ********************/
/*
 * Initialize autopilot, structs, mutexes etc.
 */
extern void autopilot_init(void);

/*
 * periodic tasks, including control/navigation loops
 */
extern void autopilot_periodic(void);

/*
 * what to do once RC frame is received
 */
extern void autopilot_on_rc_frame(void);

/*
 * mode change
 */
extern void autopilot_set_mode(uint8_t new_autopilot_mode);

/*
 * routine for starting motors
 *
 * can check additional stuff, i.e. AHRS aligned etc.
 */
extern void autopilot_set_motors_on(bool_t motors_on);

/*
 * routine for in flight checking
 */
extern void autopilot_check_in_flight(bool_t motors_on);

/*
 * autolanding variables
 */
extern bool_t autopilot_ground_detected;
extern bool_t autopilot_detect_ground_once;

/*
 * flight time
 */
extern uint32_t autopilot_flight_time;

/*
 * define radio thresholds
 */
#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

/*
 * Mode from channel value, given the thresholds
 */
#define AP_MODE_OF_PPRZ(_rc, _mode) {               \
    if      (_rc > THRESHOLD_2_PPRZ)                 \
      _mode = autopilot_mode_auto2;                 \
    else if (_rc > THRESHOLD_1_PPRZ)					\
      _mode = MODE_AUTO1;                           \
    else                                            \
      _mode = MODE_MANUAL;                          \
  }

/*
 * if needed to kill throttle from ground station
 *
 * or potentially from RC channel
 */
#define autopilot_KillThrottle(_kill) { \
    if (_kill)                          \
      autopilot_set_motors_on(FALSE);   \
    else                                \
      autopilot_set_motors_on(TRUE);    \
  }

/*
 * some power switch stuff here
 */
// TODO?

/*
 * set autopilot commands
 *
 * can be actually fbw commands if manual mode used
 */
#define SetAutopilotCommands(_cmd, _in_flight,  _motor_on) {}


/*
 * Ground detection based on vertical acceleration.
 *
 * this is for VTOL (looking for a big bump), for fixedwing it might be similar?
 */
static inline void DetectGroundEvent(void) {
  if (autopilot_mode == AP_MODE_FAILSAFE || autopilot_detect_ground_once) {
    struct NedCoor_f* accel = stateGetAccelNed_f();
    if (accel->z < -THRESHOLD_GROUND_DETECT ||
        accel->z > THRESHOLD_GROUND_DETECT) {
      autopilot_ground_detected = TRUE;
      autopilot_detect_ground_once = FALSE;
    }
  }
}
#endif /* AUTOPILOT_H */
