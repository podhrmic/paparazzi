/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot, general definition.
 *
 */

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"
#include "subsystems/gps.h"
#include "subsystems/commands.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "led.h"

/*
 * AP modes
 */
uint8_t  autopilot_mode;
uint8_t  autopilot_mode_auto2;

// in flight
bool_t   autopilot_in_flight;

// aux in flight counter
uint32_t autopilot_in_flight_counter;

// in flight time
uint16_t autopilot_flight_time;

bool_t   autopilot_motors_on;
bool_t   kill_throttle;

bool_t   autopilot_rc;
//bool_t   autopilot_power_switch; //bullshit

bool_t   autopilot_ground_detected;
bool_t   autopilot_detect_ground_once;

/*
 * dimensionless number, basically saying how many loops of
 * autopilot_check_in_flight() are needed to consider AP in flight
 */
#define AUTOPILOT_IN_FLIGHT_TIME_THRESHOLD    20

/** minimum vertical speed for in_flight condition in m/s */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_SPEED
#define AUTOPILOT_IN_FLIGHT_MIN_SPEED 0.2
#endif

/** minimum vertical acceleration for in_flight condition in m/s^2 */
// Maybe smaller for fixedwing?
#ifndef AUTOPILOT_IN_FLIGHT_MIN_ACCEL
#define AUTOPILOT_IN_FLIGHT_MIN_ACCEL 2.0
#endif

/** minimum thrust for in_flight condition in pprz_t units */
// Can be zero for fixedwing obviously
#ifndef AUTOPILOT_IN_FLIGHT_MIN_THRUST
#define AUTOPILOT_IN_FLIGHT_MIN_THRUST 500
#endif

/*
 * This can be handy if we want to get rid of
 * annoying is_ahrs_aligned() checks before flight
 * Useful for fixedwing as it lessens refactoring
 */
#ifndef AUTOPILOT_DISABLE_AHRS_KILL
#include "subsystems/ahrs.h"
static inline int ahrs_is_aligned(void) {
  return (ahrs.status == AHRS_RUNNING);
}
#else
PRINT_CONFIG_MSG("Using AUTOPILOT_DISABLE_AHRS_KILL")
static inline int ahrs_is_aligned(void) {
  return TRUE;
}
#endif

/*
 * For fixeding -> throttle for arming
 *
 * For VTOL -> yaw for arming
 */
#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
#include "autopilot_arming_switch.h"
PRINT_CONFIG_MSG("Using kill switch for motor arming")
#elif USE_THROTTLE_FOR_MOTOR_ARMING
#include "autopilot_arming_throttle.h"
PRINT_CONFIG_MSG("Using throttle for motor arming")
#else
#include "autopilot_arming_yaw.h"
PRINT_CONFIG_MSG("Using 2 sec yaw for motor arming")
#endif

/*
 * Probably keep the same for both fixedwing/VTOL
 *
 * What it means that it always starts in KILL, so it will
 * never accidentaly start motors
 */
#ifndef MODE_STARTUP
#define MODE_STARTUP AP_MODE_KILL
PRINT_CONFIG_MSG("Using default AP_MODE_KILL as MODE_STARTUP")
#endif

/*
 * init function
 *
 * TODO: add mutexes and other goodness
 */
void autopilot_init(void) {
  /*
   * mode is finally set at end of init if MODE_STARTUP is not KILL
   * -> more general
   */
  autopilot_mode = AP_MODE_KILL;
  autopilot_motors_on = FALSE;
  kill_throttle = ! autopilot_motors_on;
  autopilot_in_flight = FALSE;
  autopilot_in_flight_counter = 0;
  autopilot_mode_auto2 = MODE_AUTO2;
  autopilot_ground_detected = FALSE;
  autopilot_detect_ground_once = FALSE;
  autopilot_flight_time = 0;
  autopilot_rc = TRUE;

  autopilot_arming_init(); // for arming motors

  nav_init();
  /*
   * Maybe renaming it into something more appropriate?
   *
   * h_att_loop_xxx and v_att_loop_xxx
   */
  guidance_h_init();
  guidance_v_init();
  stabilization_init(); // can be empty for fixedwing I assume

  /* set startup mode, propagates through to guidance h/v */
  autopilot_set_mode(MODE_STARTUP);
}

/*
 * Effectively Main Autopilot function
 */
void autopilot_periodic(void) {
  // probably better to use a prescaler than an independent thread
  RunOnceEvery(NAV_PRESCALER, nav_periodic_task());


  /* If in FAILSAFE mode and either already not in_flight anymore
   * or just "detected" ground, go to KILL mode.
   */
  if (autopilot_mode == AP_MODE_FAILSAFE) {
    if (!autopilot_in_flight)
      autopilot_set_mode(AP_MODE_KILL);

// NOTE: probably good enough for now, imagine langing gear landing etc.
#if FAILSAFE_GROUND_DETECT
INFO("Using FAILSAFE_GROUND_DETECT: KILL")
    if (autopilot_ground_detected)
      autopilot_set_mode(AP_MODE_KILL);
#endif
  }

  /* Reset ground detection _after_ running flight plan
   */
  if (!autopilot_in_flight || autopilot_ground_detected) {
    autopilot_ground_detected = FALSE;
    autopilot_detect_ground_once = FALSE;
  }

  /* Set fixed "failsafe" commands from airframe file if in KILL mode.
   * If in FAILSAFE mode, run normal loops with failsafe attitude and
   * downwards velocity setpoints. <-makes sense
   */
  if (autopilot_mode == AP_MODE_KILL) {
    SetCommands(commands_failsafe);
  }
  else {
	/*
	 * The core of the whole thing -> actual control loops
	 */
    guidance_v_run( autopilot_in_flight );
    guidance_h_run( autopilot_in_flight );
    SetAuropilotCommands(stabilization_cmd, autopilot_in_flight, autopilot_motors_on);
  }

}

/*
 * Update mode -> this is platform specific
 *
 * My attempt for fixedwing - check if it makes sense (i.e. separate
 * horizontal and vertical mode/loop - it probably still does
 */
void autopilot_set_mode(uint8_t new_autopilot_mode) {

  /* force kill mode as long as AHRS is not aligned */
  if (!ahrs_is_aligned())
    new_autopilot_mode = AP_MODE_KILL;

  if (new_autopilot_mode != autopilot_mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        stabilization_attitude_set_failsafe_setpoint();
        guidance_h_mode_changed(GUIDANCE_H_MODE_AUTO2);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_in_flight = FALSE;
        autopilot_in_flight_counter = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
        break;
      case AP_MODE_MANUAL:
        guidance_h_mode_changed(GUIDANCE_H_MODE_MANUAL);
        break;
      case AP_MODE_AUTO1:
        guidance_h_mode_changed(GUIDANCE_H_MODE_AUTO1);
        break;
      case AP_MODE_AUTO2:
        guidance_h_mode_changed(GUIDANCE_H_MODE_AUTO2);
        break;
      default:
        break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        guidance_v_mode_changed(GUIDANCE_V_MODE_AUTO2);
        //guidance_v_zd_sp = SPEED_BFP_OF_REAL(0.5);
        // setpoints are already set in "horizontal mode"
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_motors_on(FALSE);
        stabilization_cmd[COMMAND_THRUST] = 0;
        guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
        break;
      case AP_MODE_MANUAL:
        guidance_v_mode_changed(GUIDANCE_V_MODE_MANUAL);
        break;
      case AP_MODE_AUTO1:
        guidance_v_mode_changed(GUIDANCE_V_MODE_AUTO1);
        break;
      case AP_MODE_AUTO2:
        guidance_v_mode_changed(GUIDANCE_V_MODE_AUTO2);
        break;
      default:
        break;
    }
    autopilot_mode = new_autopilot_mode;
  }

}

/*
 * Check in flight procedure - probably general enough for fixedwing too
 */
void autopilot_check_in_flight(bool_t motors_on) {
  if (autopilot_in_flight) {
    if (autopilot_in_flight_counter > 0) {
      /* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
      if ((stabilization_cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
          (abs(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
          (abs(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL))
      {
        autopilot_in_flight_counter--;
        if (autopilot_in_flight_counter == 0) {
          autopilot_in_flight = FALSE;
        }
      }
      else {  /* thrust, speed or accel not above min threshold, reset counter */
        autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
      }
    }
  }
  else { /* currently not in flight */
    if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
        motors_on)
    {
      /* if thrust above min threshold, assume in_flight.
       * Don't check for velocity and acceleration above threshold here...
       */
      if (stabilization_cmd[COMMAND_THRUST] > AUTOPILOT_IN_FLIGHT_MIN_THRUST) {
        autopilot_in_flight_counter++;
        if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
          autopilot_in_flight = TRUE;
      }
      else { /* currently not in_flight and thrust below threshold, reset counter */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

/*
 * set_motors_on
 *
 * General enough too
 */
void autopilot_set_motors_on(bool_t motors_on) {
  if (ahrs_is_aligned() && motors_on)
    autopilot_motors_on = TRUE;
  else
    autopilot_motors_on = FALSE;
  kill_throttle = ! autopilot_motors_on;
  autopilot_arming_set(autopilot_motors_on);
}

/*
 * Ap specific - what happens after receiving RC frame
 *
 * Double check the logic though
 *
 * Note that NAV == AUTO_2
 * 		and  ATT_DIRECT == AUTO_1
 * 		and  RC_DIRECT == MANUAL (can be even for VTOL, but not recommended)
 */
void autopilot_on_rc_frame(void) {

  if (kill_switch_is_on())
    autopilot_set_mode(AP_MODE_KILL);
  else {
    uint8_t new_autopilot_mode = 0;
    AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode);
    /* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
    if (!(new_autopilot_mode == AP_MODE_NAV
#if USE_GPS
          && GpsIsLost()
#endif
       ))
      autopilot_set_mode(new_autopilot_mode);
  }

  /* if not in FAILSAFE mode check motor and in_flight status, read RC */
  if (autopilot_mode > AP_MODE_FAILSAFE) {

    /* if there are some commands that should always be set from RC, do it */
	 // NOTE: this is typically YAW/FLAPS for fixedwing, empty for VTOL
#ifdef SetAutoCommandsFromRC
    SetAutoCommandsFromRC(commands, radio_control.values);
#endif

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    if (autopilot_mode != AP_MODE_NAV) {
      SetCommandsFromRC(commands, radio_control.values);
    }
#endif

    /* an arming sequence is used to start/stop motors */
    autopilot_arming_check_motors_on();
    kill_throttle = ! autopilot_motors_on;

    /*
     * Update setpoints from RC
     */
    guidance_v_read_rc();
    guidance_h_read_rc(autopilot_in_flight);
  }

}
