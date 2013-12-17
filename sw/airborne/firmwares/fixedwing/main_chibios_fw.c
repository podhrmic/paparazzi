/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

/*
 * Paparazzi includes
 */
#define MODULES_C

#define ABI_C

#include <math.h>

#include "firmwares/fixedwing/main_chibios_fw.h"

// Ap includes:
#include "mcu.h"
#include "mcu_periph/sys_time.h"

// Sensors
#if USE_GPS
#include "subsystems/gps.h"
#endif
#if USE_IMU
#include "subsystems/imu.h"
#endif
#if USE_AHRS
#include "subsystems/ahrs.h"
#endif
#if USE_AHRS_ALIGNER
#include "subsystems/ahrs/ahrs_aligner.h"
#endif
#include "subsystems/air_data.h"
#if USE_BARO_BOARD
#include "subsystems/sensors/baro.h"
#endif
#include "subsystems/ins.h"

// autopilot & control
#include "state.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include CTRL_TYPE_H
#include "subsystems/nav.h"
#include "generated/flight_plan.h"
#ifdef TRAFFIC_INFO
#include "subsystems/navigation/traffic_info.h"
#endif

// datalink & telemetry
#include "subsystems/datalink/datalink.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/xbee.h"
#include "subsystems/datalink/w5100.h"
#include "firmwares/fixedwing/ap_downlink.h"

// modules & settings
#include "generated/modules.h"
#include "generated/settings.h"
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
#include "rc_settings.h"
#endif
#include "subsystems/abi.h"

#include "led.h"
// End AP includes

// Fbw includes
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/radio_control.h"

#include "fbw_downlink.h"
#include "paparazzi.h"
#include "mcu_periph/i2c.h"
// End Fbw includes

/*
 * FBW Init
 */

/** Trim commands for roll, pitch and yaw.
 * These are updated from the trim commands in ap_state via inter_mcu
 */
pprz_t command_roll_trim;
pprz_t command_pitch_trim;
pprz_t command_yaw_trim;

volatile uint8_t fbw_new_actuators = 0;

uint8_t fbw_mode;
/*
 * End FBW Init
 */

/*
 * AP Init
 */
/* Default trim commands for roll, pitch and yaw */
#ifndef COMMAND_ROLL_TRIM
#define COMMAND_ROLL_TRIM 0
#endif

#ifndef COMMAND_PITCH_TRIM
#define COMMAND_PITCH_TRIM 0
#endif

#ifndef COMMAND_YAW_TRIM
#define COMMAND_YAW_TRIM 0
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)
PRINT_CONFIG_VAR(NAVIGATION_FREQUENCY)
PRINT_CONFIG_VAR(CONTROL_FREQUENCY)

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 40
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#ifndef FAILSAFE_FREQUENCY
#define FAILSAFE_FREQUENCY 20
#endif
PRINT_CONFIG_VAR(FAILSAFE_FREQUENCY)

#ifndef ELECTRICAL_PERIODIC_FREQ
#define ELECTRICAL_PERIODIC_FREQ 10
#endif
PRINT_CONFIG_VAR(ELECTRICAL_PERIODIC_FREQ)

#ifndef RADIO_CONTROL_FREQ
#define RADIO_CONTROL_FREQ 60
#endif
PRINT_CONFIG_VAR(RADIO_CONTROL_FREQ)

#if USE_AHRS && USE_IMU

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif
PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
#ifndef AHRS_CORRECT_FREQUENCY
#define AHRS_CORRECT_FREQUENCY PERIODIC_FREQUENCY
#endif
PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)

static inline void on_gyro_event( void );
static inline void on_accel_event( void );
static inline void on_mag_event( void );
volatile uint8_t ahrs_timeout_counter = 0;

#endif // USE_AHRS && USE_IMU

#if USE_GPS
static WORKING_AREA(wa_thd_gps_rx, CH_THREAD_AREA_GPS_RX);
static void on_gps_solution( void );
#endif

static uint8_t  mcu1_status;

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static uint8_t  mcu1_ppm_cpt;
#endif

/** Supply current in milliAmpere.
 * This the ap copy of the measurement from fbw
 */
static int32_t current; // milliAmpere

/*
 * End AP Init
 */

/*
 * Thread Area Definitions
 */
#define CH_THREAD_AREA_HEARTBEAT 256
#define CH_THREAD_AREA_FAILSAFE 256
#define CH_THREAD_AREA_ELECTRICAL 256
#define CH_THREAD_AREA_RADIO_CONTROL 256
#define CH_THREAD_AREA_RADIO_EVENT 512

#define CH_THREAD_AREA_NAVIGATION 512

/*
 * Thread Area Initialization
 */
static WORKING_AREA(wa_thd_heartbeat, CH_THREAD_AREA_HEARTBEAT);
static WORKING_AREA(wa_thd_failsafe, CH_THREAD_AREA_FAILSAFE);
static WORKING_AREA(wa_thd_electrical, CH_THREAD_AREA_ELECTRICAL);
static WORKING_AREA(wa_thd_radio_control, CH_THREAD_AREA_RADIO_CONTROL);
static WORKING_AREA(wa_thd_radio_event, CH_THREAD_AREA_RADIO_EVENT);

static WORKING_AREA(wa_thd_nav, CH_THREAD_AREA_NAVIGATION);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
static __attribute__((noreturn)) msg_t thd_failsafe(void *arg);
static __attribute__((noreturn)) msg_t thd_electrical(void *arg);
static __attribute__((noreturn)) msg_t thd_radio_control(void *arg);
static __attribute__((noreturn)) msg_t thd_radio_event(void *arg);
static __attribute__((noreturn)) msg_t thd_navigation_periodic(void *arg);

/*
 * Static Auxilliary Functions Definitions
 */
static inline void failsafe_check(void);

#ifdef RADIO_CONTROL
static void handle_rc_frame( void ) {
  fbw_mode = FBW_MODE_OF_PPRZ(radio_control.values[RADIO_MODE]);
  if (fbw_mode == FBW_MODE_MANUAL)
  {
    SetCommandsFromRC(commands, radio_control.values);
    fbw_new_actuators = 1;
  }
}
#endif

/**
 * HeartBeat & System Info
 *
 * Blinks LED and logs the cpu usage and other system info
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  chRegSetThreadName("pprz_heartbeat");
  (void) arg;
  systime_t time = chTimeNow();
  static uint32_t last_idle_counter = 0;
  static uint32_t last_nb_sec = 0;

  while (TRUE)
  {
    time += S2ST(1);
    LED_TOGGLE(SYS_TIME_LED);
    sys_time.nb_sec++;

    monitor_task();

    core_free_memory = chCoreStatus();
    thread_counter = 0;

    Thread *tp;
    tp = chRegFirstThread();
    do
    {
      thread_counter++;
      if (tp ==chSysGetIdleThread())
      {
        idle_counter =  (uint32_t)tp->p_time;
      }
      tp = chRegNextThread(tp);
    }
    while (tp != NULL);

    cpu_counter = (idle_counter-last_idle_counter)/(sys_time.nb_sec-last_nb_sec);
    cpu_frequency = (1 - (float)cpu_counter/CH_FREQUENCY)*100;

    last_idle_counter = idle_counter;
    last_nb_sec = sys_time.nb_sec;

    chThdSleepUntil(time);
  }
}

/**
 * Failsafe Thread
 *
 * Replaces failsafe_periodic()
 *
 * TODO: ChibiOS/RT failsafe check (hypervisor thread)
 */
static __attribute__((noreturn)) msg_t thd_failsafe(void *arg)
{
  chRegSetThreadName("pprz_failsafe");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/FAILSAFE_FREQUENCY);
    failsafe_check();
    chThdSleepUntil(time);
  }
}

/*
 * Electrical Periodic Thread
 *
 * Calls electrical_periodic()
 */
static __attribute__((noreturn)) msg_t thd_electrical(void *arg)
{
  chRegSetThreadName("pprz_electrical");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/ELECTRICAL_PERIODIC_FREQ);
    electrical_periodic();
    vsupply = electrical.vsupply;
    chThdSleepUntil(time);
  }
}

/*
 * Radio Control Periodic Thread
 *
 * Calls radio_control_periodic()
 */
static __attribute__((noreturn)) msg_t thd_radio_control(void *arg)
{
  chRegSetThreadName("pprz_radio_control");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/RADIO_CONTROL_FREQ);
    radio_control_periodic_task();
    chThdSleepUntil(time);
  }
}


/**
 * Navigation periodic
 */
static __attribute__((noreturn)) msg_t thd_navigation_periodic(void *arg)
{
  chRegSetThreadName("pprz_navigation_periodic");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/NAVIGATION_FREQUENCY);
    navigation_task();
    chThdSleepUntil(time);
  }
}

/**
 * Radio Control Event Thread
 *
 * Waits for EVT_PPM_FRAME event flag to be broadcasted,
 * then executes RadioControlEvent()
 *
 * @note: It is a nice example how to use event listeners.
 * Optionally after the frame is processed, another event can be
 * broadcasted, so it is possible to chain data processing (i.e. in AHRS)
 * Maybe a similar structure can be used for GPS events etc.
 *
 * after receiving EVT_PPM_FRAM and processing it, we can call
 * chEvtBroadcastFlags(&initializedEventSource, SOME_DEFINED_EVENT);
 * to propagate event further
 */
static __attribute__((noreturn)) msg_t thd_radio_event(void *arg)
{
  chRegSetThreadName("pprz_radio_event");
  (void) arg;

  EventListener elRadioEvt;
  chEvtRegister(&eventPpmFrame, &elRadioEvt, EVT_PPM_FRAME);
  flagsmask_t rc_flags;

  while (TRUE)
  {
    chEvtWaitOne(EVENT_MASK(EVT_PPM_FRAME));
    rc_flags = chEvtGetAndClearFlags(&elRadioEvt);
    if (rc_flags & EVT_PPM_FRAME)
    {
      RadioControlEvent(handle_rc_frame);
        //chEvtBroadcastFlags(&initializedEventSource, SOME_DEFINED_EVENT);
    }
  }
}

#if USE_BARO_BOARD
#define CH_THREAD_AREA_BARO 512
static WORKING_AREA(wa_thd_baro, CH_THREAD_AREA_BARO);
static __attribute__((noreturn)) msg_t thd_baro(void *arg);

/**
 * Baro thread
 */
static __attribute__((noreturn)) msg_t thd_baro(void *arg)
{
  chRegSetThreadName("pprz_baro");
  (void) arg;

  baro_init();

  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/BARO_PERIODIC_FREQUENCY);
    baro_periodic();
    chThdSleepUntil(time);
  }
}
#endif /* USE_BARO_BOARD */

#ifdef DOWNLINK
#define CH_THREAD_AREA_DOWNLINK_TX 1024
#define CH_THREAD_AREA_DOWNLINK_RX 1024
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg);
__attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg);

/**
 *  Telemetry TX thread
 *
 *  Replaces telemetryPeriodic()
 */
static WORKING_AREA(wa_thd_telemetry_tx, 1024);
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_tx");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/TELEMETRY_FREQUENCY);
    PeriodicSendAp(DefaultChannel, DefaultDevice);
    PeriodicSendFbw(DefaultChannel,DefaultDevice);
    chThdSleepUntil(time);
  }
}

/**
 *  Telemetry RX thread
 *
 *  Replaces DatalinkEvent()
 *
 *  @note: assumes PprziDwonlink for now
 *  @todo General definition for different links
 */
static WORKING_AREA(wa_thd_telemetry_rx, 1024);
__attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_rx");
  (void) arg;
  EventListener elTelemetryRx;
  flagsmask_t flags;
  chEvtRegisterMask((EventSource *)chnGetEventSource((SerialDriver*)DOWNLINK_PORT.reg_addr), &elTelemetryRx, EVENT_MASK(1));
  while (TRUE)
  {
    chEvtWaitOneTimeout(EVENT_MASK(1), S2ST(1));
    flags = chEvtGetAndClearFlags(&elTelemetryRx);
    ch_uart_receive_downlink(DOWNLINK_PORT, flags, parse_pprz, &pprz_tp);
    if (pprz_tp.trans.msg_received)
    {
      pprz_parse_payload(&(pprz_tp));
      pprz_tp.trans.msg_received = FALSE;
      dl_parse_msg();
      dl_msg_available = FALSE;
    }
  }
}
#endif /* DOWNLINK */


#ifdef MODULES_C
/**
 * Modules periodic tasks
 */
static WORKING_AREA(wa_thd_modules_periodic, 1024);
__attribute__((noreturn)) msg_t thd_modules_periodic(void *arg)
{
  chRegSetThreadName("pprz_modules_periodic");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += MS2ST(1000/MODULES_FREQUENCY);
    modules_periodic_task();
    chThdSleepUntil(time);
  }
}
#endif /* MODULES_C */

/**
 * Paparazzi failsafe thread
 */
static inline void failsafe_check(void)
{
  // TODO
#ifdef INTER_MCU
  inter_mcu_periodic_task();
  if (fbw_mode == FBW_MODE_AUTO && !ap_ok)
  {
    fbw_mode = FBW_MODE_FAILSAFE;
    SetCommands(commands_failsafe);
    fbw_new_actuators = 1;
  }
#endif
}


/**
 * Fly-by-wire init
 */
void init_fbw( void ) {

  mcu_init();

  electrical_init();

#ifdef ACTUATORS
  actuators_init();
  /* Load the failsafe defaults */
  SetCommands(commands_failsafe);
  fbw_new_actuators = 1;
#endif
#ifdef RADIO_CONTROL
  radio_control_init();
#endif

  fbw_mode = FBW_MODE_FAILSAFE;
}

/**
 * Autopilot iniy
 */
void init_ap( void ) {
  /****** initialize and reset state interface ********/
  stateInit();

  /************* Sensors initialization ***************/
#if USE_GPS
  gps_init();
#endif

#if USE_IMU
  imu_init();
#if USE_IMU_FLOAT
  imu_float_init();
#endif
#endif

#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

#if USE_AHRS
  ahrs_init();
#endif

  air_data_init();
#if USE_BARO_BOARD
  baro_init();
#endif

  ins_init();

  /************ Internal status ***************/
  autopilot_init();
  h_ctl_init();
  v_ctl_init();
  nav_init();

  modules_init();

  settings_init();

  /** - start interrupt task */
  mcu_int_enable();

#if defined DATALINK
#if DATALINK == XBEE
  xbee_init();
#endif
#if DATALINK == W5100
  w5100_init();
#endif
#endif /* DATALINK */

  /************ Multi-uavs status ***************/

#ifdef TRAFFIC_INFO
  traffic_info_init();
#endif

  /* set initial trim values.
   * these are passed to fbw via inter_mcu.
   */
  ap_state->command_roll_trim = COMMAND_ROLL_TRIM;
  ap_state->command_pitch_trim = COMMAND_PITCH_TRIM;
  ap_state->command_yaw_trim = COMMAND_YAW_TRIM;
}


/**
 * Init threads
 */
void init_threads(void) {
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), IDLEPRIO, thd_heartbeat, NULL);
  chThdCreateStatic(wa_thd_electrical, sizeof(wa_thd_electrical), LOWPRIO, thd_electrical, NULL);
  chThdCreateStatic(wa_thd_radio_control, sizeof(wa_thd_radio_control), NORMALPRIO, thd_radio_control, NULL);
  chThdCreateStatic(wa_thd_radio_event, sizeof(wa_thd_radio_event), NORMALPRIO, thd_radio_event, NULL);

  chThdCreateStatic(wa_thd_nav, sizeof(wa_thd_nav), NORMALPRIO, thd_navigation_periodic, NULL);

#if USE_BARO_BOARD
  chThdCreateStatic(wa_thd_baro, sizeof(wa_thd_baro),LOWPRIO, thd_baro, NULL);
#endif

#ifdef DOWNLINK
  chThdCreateStatic(wa_thd_telemetry_tx, sizeof(wa_thd_telemetry_tx),NORMALPRIO, thd_telemetry_tx, NULL);
  chThdCreateStatic(wa_thd_telemetry_rx, sizeof(wa_thd_telemetry_rx),NORMALPRIO, thd_telemetry_rx, NULL);
#endif

#ifdef USE_GPS
  chThdCreateStatic(wa_thd_gps_rx, sizeof(wa_thd_gps_rx),NORMALPRIO, thd_gps_rx, &on_gps_solution);
#endif

#ifdef MODULES_C
  chThdCreateStatic(wa_thd_modules_periodic, sizeof(wa_thd_modules_periodic),LOWPRIO, thd_modules_periodic, NULL);
#endif

  chThdCreateStatic(wa_thd_failsafe, sizeof(wa_thd_failsafe), HIGHPRIO, thd_failsafe, NULL);
}


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Paparazzi initialization
   */
  init_fbw();
  init_ap();

  init_threads();

  chThdSetPriority (HIGHPRIO);

  chThdSleep(MS2ST(100));
  systime_t main_time = chTimeNow();

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
    main_time += US2ST(1000000/PERIODIC_FREQUENCY);
    sensors_task();
    ImuEvent(on_gyro_event, on_accel_event, on_mag_event);
    attitude_loop();
    if (inter_mcu_received_fbw) {
      /* receive radio control task from fbw */
      inter_mcu_received_fbw = FALSE;
      telecommand_task();
    }
    event_task_fbw();
    chThdSleepUntil(main_time);
  }
  return 0;
}


/**
 * Gps callback
 */
#if USE_GPS
static void on_gps_solution( void ) {
  ins_update_gps();
#if USE_AHRS
  ahrs_update_gps();
#endif
#ifdef GPS_TRIGGERED_FUNCTION
  GPS_TRIGGERED_FUNCTION();
#endif
}
#endif


/*
 * AHRS/IMU related stuff
 */
#if USE_AHRS
#if USE_IMU
static inline void on_accel_event( void ) {
}

static inline void on_gyro_event( void ) {

  ahrs_timeout_counter = 0;

#if USE_AHRS_ALIGNER
  // Run aligner on raw data as it also makes averages.
  if (ahrs.status == AHRS_UNINIT) {
    ImuScaleGyro(imu);
    ImuScaleAccel(imu);
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
    return;
  }
#endif

#if PERIODIC_FREQUENCY == 60
  ImuScaleGyro(imu);
  ImuScaleAccel(imu);

  ahrs_propagate();
  ahrs_update_accel();

#else //PERIODIC_FREQUENCY
  static uint8_t _reduced_propagation_rate = 0;
  static uint8_t _reduced_correction_rate = 0;
  static struct Int32Vect3 acc_avg;
  static struct Int32Rates gyr_avg;

  RATES_ADD(gyr_avg, imu.gyro_unscaled);
  VECT3_ADD(acc_avg, imu.accel_unscaled);

  _reduced_propagation_rate++;
  if (_reduced_propagation_rate < (PERIODIC_FREQUENCY / AHRS_PROPAGATE_FREQUENCY))
  {
    return;
  }
  else
  {
    _reduced_propagation_rate = 0;

    RATES_SDIV(imu.gyro_unscaled, gyr_avg, (PERIODIC_FREQUENCY / AHRS_PROPAGATE_FREQUENCY) );
    INT_RATES_ZERO(gyr_avg);

    ImuScaleGyro(imu);

    ahrs_propagate();

    _reduced_correction_rate++;
    if (_reduced_correction_rate >= (AHRS_PROPAGATE_FREQUENCY / AHRS_CORRECT_FREQUENCY))
    {
      _reduced_correction_rate = 0;
      VECT3_SDIV(imu.accel_unscaled, acc_avg, (PERIODIC_FREQUENCY / AHRS_CORRECT_FREQUENCY) );
      INT_VECT3_ZERO(acc_avg);
      ImuScaleAccel(imu);
      ahrs_update_accel();
    }
  }
#endif //PERIODIC_FREQUENCY


}

static inline void on_mag_event(void)
{
#if USE_MAGNETOMETER
  ImuScaleMag(imu);
  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag();
  }
#endif
}

#endif // USE_IMU

#endif // USE_AHRS


/** Maximum time allowed for low battery level before going into kill mode */
#define LOW_BATTERY_DELAY 5

/** Maximum distance from HOME waypoint before going into kill mode */
#ifndef KILL_MODE_DISTANCE
#define KILL_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

/** Define minimal speed for takeoff in m/s */
#define MIN_SPEED_FOR_TAKEOFF 5.

/** monitor stuff run at 1Hz */
void monitor_task( void ) {
  if (autopilot_flight_time)
    autopilot_flight_time++;
#if defined DATALINK || defined SITL
  datalink_time++;
#endif

  static uint8_t t = 0;
  if (vsupply < CATASTROPHIC_BAT_LEVEL*10)
    t++;
  else
    t = 0;
  kill_throttle |= (t >= LOW_BATTERY_DELAY);
  kill_throttle |= launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));

  if (!autopilot_flight_time &&
      *stateGetHorizontalSpeedNorm_f() > MIN_SPEED_FOR_TAKEOFF) {
    autopilot_flight_time = 1;
    launch = TRUE; /* Not set in non auto launch */
    uint16_t time_sec = sys_time.nb_sec;
    DOWNLINK_SEND_TAKEOFF(DefaultChannel, DefaultDevice, &time_sec);
  }
}

/** Run at PERIODIC_FREQUENCY (60Hz if not defined) */
void sensors_task( void ) {
#if USE_IMU
  imu_periodic();

#if USE_AHRS
  if (ahrs_timeout_counter < 255)
    ahrs_timeout_counter ++;
#endif // USE_AHRS
#endif // USE_IMU

  //FIXME: this is just a kludge
#if USE_AHRS && defined SITL && !USE_NPS
  ahrs_propagate();
#endif

#if USE_BARO_BOARD
  baro_periodic();
#endif

  ins_periodic();
}


#ifdef FAILSAFE_DELAY_WITHOUT_GPS
#define GpsTimeoutError (sys_time.nb_sec - gps.last_fix_time > FAILSAFE_DELAY_WITHOUT_GPS)
#endif

/**
 *  Compute desired_course
 */
void navigation_task( void ) {
#if defined FAILSAFE_DELAY_WITHOUT_GPS
  /** This section is used for the failsafe of GPS */
  static uint8_t last_pprz_mode;

  /** If aircraft is launched and is in autonomus mode, go into
      PPRZ_MODE_GPS_OUT_OF_ORDER mode (Failsafe) if we lost the GPS */
  if (launch) {
    if (GpsTimeoutError) {
      if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME) {
        last_pprz_mode = pprz_mode;
        pprz_mode = PPRZ_MODE_GPS_OUT_OF_ORDER;
        PERIODIC_SEND_PPRZ_MODE(DefaultChannel, DefaultDevice);
        gps_lost = TRUE;
      }
    } else if (gps_lost) { /* GPS is ok */
      /** If aircraft was in failsafe mode, come back in previous mode */
      pprz_mode = last_pprz_mode;
      gps_lost = FALSE;
      PERIODIC_SEND_PPRZ_MODE(DefaultChannel, DefaultDevice);
    }
  }
#endif /* GPS && FAILSAFE_DELAY_WITHOUT_GPS */

  common_nav_periodic_task_4Hz();
  if (pprz_mode == PPRZ_MODE_HOME)
    nav_home();
  else if (pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER)
    nav_without_gps();
  else
    nav_periodic_task();

#ifdef TCAS
  CallTCAS();
#endif

#ifndef PERIOD_NAVIGATION_0 // If not sent periodically (in default 0 mode)
  SEND_NAVIGATION(DefaultChannel, DefaultDevice);
#endif

  SEND_CAM(DefaultChannel, DefaultDevice);

  /* The nav task computes only nav_altitude. However, we are interested
     by desired_altitude (= nav_alt+alt_shift) in any case.
     So we always run the altitude control loop */
  if (v_ctl_mode == V_CTL_MODE_AUTO_ALT)
    v_ctl_altitude_loop();

  if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME
            || pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER) {
#ifdef H_CTL_RATE_LOOP
    /* Be sure to be in attitude mode, not roll */
    h_ctl_auto1_rate = FALSE;
#endif
    if (lateral_mode >=LATERAL_MODE_COURSE)
      h_ctl_course_loop(); /* aka compute nav_desired_roll */

    // climb_loop(); //4Hz
  }
  energy += ((float)current) / 3600.0f * 0.25f; // mAh = mA * dt (4Hz -> hours)
}


#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
volatile uint8_t new_ins_attitude = 0;
#endif

/**
 * Attitude loop
 */
void attitude_loop( void ) {

#if USE_INFRARED
  ahrs_update_infrared();
#endif /* USE_INFRARED */

  if (pprz_mode >= PPRZ_MODE_AUTO2)
  {
    if (v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE) {
      v_ctl_throttle_setpoint = nav_throttle_setpoint;
      v_ctl_pitch_setpoint = nav_pitch;
    }
    else if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB)
    {
      v_ctl_climb_loop();
    }

#if defined V_CTL_THROTTLE_IDLE
    Bound(v_ctl_throttle_setpoint, TRIM_PPRZ(V_CTL_THROTTLE_IDLE*MAX_PPRZ), MAX_PPRZ);
#endif

#ifdef V_CTL_POWER_CTL_BAT_NOMINAL
    if (vsupply > 0.) {
      v_ctl_throttle_setpoint *= 10. * V_CTL_POWER_CTL_BAT_NOMINAL / (float)vsupply;
      v_ctl_throttle_setpoint = TRIM_UPPRZ(v_ctl_throttle_setpoint);
    }
#endif

    h_ctl_pitch_setpoint = v_ctl_pitch_setpoint; // Copy the pitch setpoint from the guidance to the stabilization control
    Bound(h_ctl_pitch_setpoint, H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
    if (kill_throttle || (!autopilot_flight_time && !launch))
      v_ctl_throttle_setpoint = 0;
  }

  h_ctl_attitude_loop(); /* Set  h_ctl_aileron_setpoint & h_ctl_elevator_setpoint */
  v_ctl_throttle_slew();
  ap_state->commands[COMMAND_THROTTLE] = v_ctl_throttle_slewed;
  ap_state->commands[COMMAND_ROLL] = -h_ctl_aileron_setpoint;

  ap_state->commands[COMMAND_PITCH] = h_ctl_elevator_setpoint;

#if defined MCU_SPI_LINK || defined MCU_UART_LINK
  link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
  /**Directly set the flag indicating to FBW that shared buffer is available*/
  inter_mcu_received_ap = TRUE;
#endif

}


/**
 * Fbw event
 */

void event_task_fbw( void) {
#ifdef INTER_MCU
  if (inter_mcu_received_ap) {
    inter_mcu_received_ap = FALSE;
    inter_mcu_event_task();
    command_roll_trim = ap_state->command_roll_trim;
    command_pitch_trim = ap_state->command_pitch_trim;
    command_yaw_trim = ap_state->command_yaw_trim;
#ifndef OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP
    if (ap_ok && fbw_mode == FBW_MODE_FAILSAFE) {
      fbw_mode = FBW_MODE_AUTO;
    }
#endif
    if (fbw_mode == FBW_MODE_AUTO) {
      SetCommands(ap_state->commands);
    }
#ifdef SetApOnlyCommands
    else
    {
      SetApOnlyCommands(ap_state->commands);
    }
#endif
    fbw_new_actuators = 1;

#ifdef SINGLE_MCU
    inter_mcu_fill_fbw_state();
#endif /**Else the buffer is filled even if the last receive was not correct */
  }

#ifdef ACTUATORS
  if (fbw_new_actuators > 0)
  {
    pprz_t trimmed_commands[COMMANDS_NB];
    int i;
    for(i = 0; i < COMMANDS_NB; i++) trimmed_commands[i] = commands[i];

    #ifdef COMMAND_ROLL
    trimmed_commands[COMMAND_ROLL] += ChopAbs(command_roll_trim, MAX_PPRZ/10);
    #endif
    #ifdef COMMAND_PITCH
    trimmed_commands[COMMAND_PITCH] += ChopAbs(command_pitch_trim, MAX_PPRZ/10);
    #endif
    #ifdef COMMAND_YAW
    trimmed_commands[COMMAND_YAW] += ChopAbs(command_yaw_trim, MAX_PPRZ);
    #endif

    SetActuatorsFromCommands(trimmed_commands, autopilot_mode);
    fbw_new_actuators = 0;
    #if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE
    if (crash == 1)
    {
      for (;;) {}
    }
    #endif

  }
#endif
#endif /* INTER_MCU */
}

/** Update paparazzi mode.
 */
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static inline uint8_t pprz_mode_update( void ) {
  if ((pprz_mode != PPRZ_MODE_HOME &&
       pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER)
#ifdef UNLOCKED_HOME_MODE
      || TRUE
#endif
      ) {
#ifndef RADIO_AUTO_MODE
    return ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE]));
#else
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    /* If RADIO_AUTO_MODE is enabled mode swithing will be seperated between two switches/channels
     * RADIO_MODE will switch between PPRZ_MODE_MANUAL and any PPRZ_MODE_AUTO mode selected by RADIO_AUTO_MODE.
     *
     * This is mainly a cludge for entry level radios with no three-way switch but two available two-way switches which can be used.
     */
    if(PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE]) == PPRZ_MODE_MANUAL) {
      /* RADIO_MODE in MANUAL position */
      return ModeUpdate(pprz_mode, PPRZ_MODE_MANUAL);
    } else {
      /* RADIO_MODE not in MANUAL position.
       * Select AUTO mode bassed on RADIO_AUTO_MODE channel
       */
      return ModeUpdate(pprz_mode, (fbw_state->channels[RADIO_AUTO_MODE] > THRESHOLD2) ? PPRZ_MODE_AUTO2 : PPRZ_MODE_AUTO1);
    }
#endif // RADIO_AUTO_MODE
  } else
    return FALSE;
}
#else // not RADIO_CONTROL
static inline uint8_t pprz_mode_update( void ) {
  return FALSE;
}
#endif

/** Send back uncontrolled channels.
 */
static inline void copy_from_to_fbw ( void ) {
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(ap_state->commands, fbw_state->channels);
#elif defined RADIO_YAW && defined COMMAND_YAW
  ap_state->commands[COMMAND_YAW] = fbw_state->channels[RADIO_YAW];
#endif
}

static inline uint8_t mcu1_status_update( void ) {
  uint8_t new_status = fbw_state->status;
  if (mcu1_status != new_status) {
    bool_t changed = ((mcu1_status&MASK_FBW_CHANGED) != (new_status&MASK_FBW_CHANGED));
    mcu1_status = new_status;
    return changed;
  }
  return FALSE;
}


/** mode to enter when RC is lost in PPRZ_MODE_MANUAL or PPRZ_MODE_AUTO1 */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE PPRZ_MODE_HOME
#endif

/**
 * Function to be called when a message from FBW is available
 */
static inline void telecommand_task( void ) {
  uint8_t mode_changed = FALSE;
  copy_from_to_fbw();

  uint8_t really_lost = bit_is_set(fbw_state->status, STATUS_RADIO_REALLY_LOST) && (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_MANUAL);
  if (pprz_mode != PPRZ_MODE_HOME && pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER && launch) {
    if  (too_far_from_home) {
      pprz_mode = PPRZ_MODE_HOME;
      mode_changed = TRUE;
    }
    if  (really_lost) {
      pprz_mode = RC_LOST_MODE;
      mode_changed = TRUE;
    }
  }
  if (bit_is_set(fbw_state->status, AVERAGED_CHANNELS_SENT)) {
    bool_t pprz_mode_changed = pprz_mode_update();
    mode_changed |= pprz_mode_changed;
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
    bool_t calib_mode_changed = RcSettingsModeUpdate(fbw_state->channels);
    rc_settings(calib_mode_changed || pprz_mode_changed);
    mode_changed |= calib_mode_changed;
#endif
  }
  mode_changed |= mcu1_status_update();
  if ( mode_changed )
    PERIODIC_SEND_PPRZ_MODE(DefaultChannel, DefaultDevice);

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
  /** In AUTO1 mode, compute roll setpoint and pitch setpoint from
   * \a RADIO_ROLL and \a RADIO_PITCH \n
   */
  if (pprz_mode == PPRZ_MODE_AUTO1) {
    /** Roll is bounded between [-AUTO1_MAX_ROLL;AUTO1_MAX_ROLL] */
    h_ctl_roll_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_ROLL], 0., AUTO1_MAX_ROLL);

    /** Pitch is bounded between [-AUTO1_MAX_PITCH;AUTO1_MAX_PITCH] */
    h_ctl_pitch_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_PITCH], 0., AUTO1_MAX_PITCH);
  } /** Else asynchronously set by \a h_ctl_course_loop() */

  /** In AUTO1, throttle comes from RADIO_THROTTLE
      In MANUAL, the value is copied to get it in the telemetry */
  if (pprz_mode == PPRZ_MODE_MANUAL || pprz_mode == PPRZ_MODE_AUTO1) {
    v_ctl_throttle_setpoint = fbw_state->channels[RADIO_THROTTLE];
  }
  /** else asynchronously set by v_ctl_climb_loop(); */

  mcu1_ppm_cpt = fbw_state->ppm_cpt;
#endif // RADIO_CONTROL


  vsupply = fbw_state->vsupply;
  current = fbw_state->current;

#ifdef RADIO_CONTROL
  if (!autopilot_flight_time) {
    if (pprz_mode == PPRZ_MODE_AUTO2 && fbw_state->channels[RADIO_THROTTLE] > THROTTLE_THRESHOLD_TAKEOFF) {
      launch = TRUE;
    }
  }
#endif
}
