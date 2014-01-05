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
 * OK
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "std.h"

#include "led.h"

#include "generated/airframe.h"
#include "subsystems/ins.h"

/*
 * AP modes are defined here
 */
#define AP_MODE_FAILSAFE          0
#define AP_MODE_KILL              1
//#define AP_MODE_RATE_DIRECT       2
#define AP_MODE_ATTITUDE_DIRECT   3
//#define AP_MODE_RATE_RC_CLIMB     4
//#define AP_MODE_ATTITUDE_RC_CLIMB 5
//#define AP_MODE_ATTITUDE_CLIMB    6
//#define AP_MODE_RATE_Z_HOLD       7
//#define AP_MODE_ATTITUDE_Z_HOLD   8
//#define AP_MODE_HOVER_DIRECT      9
//#define AP_MODE_HOVER_CLIMB       10
//#define AP_MODE_HOVER_Z_HOLD      11
#define AP_MODE_NAV               12
#define AP_MODE_RC_DIRECT         13	// Safety Pilot Direct Commands
//#define AP_MODE_CARE_FREE_DIRECT  14
//#define AP_MODE_FORWARD           15


/** Default RC mode.
 */
#ifndef MODE_MANUAL
#define MODE_MANUAL AP_MODE_RC_DIRECT
#endif
#ifndef MODE_AUTO1
#define MODE_AUTO1 AP_MODE_ATTITUDE_DIRECT
#endif
#ifndef MODE_AUTO2
#define MODE_AUTO2 AP_MODE_NAV
#endif

// FIXME, move to control/stabilization
// the same as GUIDANCE_H/V_MODE
#define LATERAL_MODE_MANUAL    0
#define LATERAL_MODE_ROLL_RATE 1
#define LATERAL_MODE_ROLL      2
#define LATERAL_MODE_COURSE    3
#define LATERAL_MODE_NB        4
extern uint8_t lateral_mode;

// FIXME Needed for generated/settings.h to work (see fixedwing_basic_rt.xml)
#define autopilot_ResetFlightTimeAndLaunch(_) { \
  autopilot_flight_time = 0; launch = FALSE; \
}

// FIXME define proper stabilization loops -> from Guidance
extern void guidance_h_init(void);
extern void guidance_h_mode_changed(uint8_t new_mode);
extern void guidance_h_run(bool_t in_flight);

extern void guidance_v_init(void);
extern void guidance_v_mode_changed(uint8_t new_mode);
extern void guidance_v_run(bool_t in_flight);

/** vertical speed setpoint in meter/s (input).
 *  fixed point representation: Q12.19
 *  accuracy 0.0000019, range +/-4096
 */
extern int32_t guidance_v_zd_sp;

/************ AP variables ********************/

// FIXME required by datalink.c
extern bool_t launch;

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

// TODO k cemu to je
extern bool_t autopilot_power_switch;


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
#ifdef POWER_SWITCH_LED
#define autopilot_SetPowerSwitch(_v) {          \
    autopilot_power_switch = _v;                \
    if (_v) { LED_OFF(POWER_SWITCH_LED); }      \
    else { LED_ON(POWER_SWITCH_LED); }          \
  }
#else
#define autopilot_SetPowerSwitch(_v) {		\
    autopilot_power_switch = _v;		\
  }
#endif

/*
 * set autopilot commands
 *
 * can be actually fbw commands if manual mode used
 * TODO: some heuristics for motor on/in flight?
 */
#define SetAutopilotCommands(_cmd, _in_flight, _motor_on) { \
  int i; \
  for(i = 0; i < COMMANDS_NB; i++) commands[i] = _cmd[i]; \
}
// SetCommandsFromRC generated, probably not so useful...

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif

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
