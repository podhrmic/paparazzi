/*
 * Copyright (C) 2013-2015 Gautier Hattenberger, Alexandre Bustico
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
 * @file firmwares/fixedwing/main_chibios.c
 */

/*
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

#include "led.h"
#include "mcu_periph/sys_time.h"

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_CFG_ST_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_CFG_ST_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_CFG_ST_FREQUENCY
#elif  CH_CFG_ST_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_CFG_ST_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

#ifdef FBW
#include "firmwares/fixedwing/main_fbw.h"
#define Fbw(f) f ## _fbw()
#else
#define Fbw(f)
#endif

#ifdef AP
#include "firmwares/fixedwing/main_ap.h"
#define Ap(f) f ## _ap()
#else
#define Ap(f)
#endif


/*
 * Telemetry defines
 */
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_chibios_info(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t time_now = 0;
  time_now = chVTGetSystemTime()/CH_CFG_ST_FREQUENCY;

  pprz_msg_send_CHIBIOS_INFO(trans, dev, AC_ID,
                    &core_free_memory,
                    &time_now,
                    &thread_counter,
                    &cpu_frequency);
}
#endif

/*
 * Heartbeat thread
 */
static void thd_heartbeat(void *arg);
static THD_WORKING_AREA(wa_thd_heartbeat, 2048);

/*
 * PPRZ thread
 */
static void thd_pprz(void *arg);
static THD_WORKING_AREA(wa_thd_pprz, 4096);
thread_t *pprzThdPtr = NULL;

/**
 * Main function
 */
int main(void)
{
  // Init
  Fbw(init);
  Ap(init);

  // ????
  PWR->CSR &= ~PWR_CSR_BRE;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

  // Create threads
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat),
      NORMALPRIO, thd_heartbeat, NULL);

  chThdSleepMilliseconds(100);

  pprzThdPtr = chThdCreateStatic(wa_thd_pprz, sizeof(wa_thd_pprz),
      NORMALPRIO+1, thd_pprz, NULL);

  // Main loop, do nothing
  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
  return 0;
}


/**
 * HeartBeat & System Info
 *
 * Blinks LED and logs the cpu usage and other system info
 */
static void thd_heartbeat(void *arg)
{
  chRegSetThreadName("pprz_heartbeat");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  static uint32_t last_idle_counter = 0;
  static uint32_t last_nb_sec = 0;

  while (TRUE)
  {
    time += S2ST(1);
    core_free_memory = chCoreGetStatusX();
    thread_counter = 0;

    thread_t *tp;
    tp = chRegFirstThread();
    do
    {
      thread_counter++;
      if (tp ==chSysGetIdleThreadX())
      {
        // only if CH_DBG_THREADS_PROFILING == TRUE
        idle_counter =  (uint32_t)tp->p_time;
      }
      tp = chRegNextThread(tp);
    }
    while (tp != NULL);

    cpu_counter = (idle_counter-last_idle_counter)/(sys_time.nb_sec-last_nb_sec);
    cpu_frequency = (1 - (float)cpu_counter/CH_CFG_ST_FREQUENCY)*100;

    last_idle_counter = idle_counter;
    last_nb_sec = sys_time.nb_sec;

    chThdSleepUntil(time);
  }
}

/*
 * PPRZ thread
 *
 * Call PPRZ periodic and event functions
 */
static void thd_pprz(void *arg)
{
  /*
     To be compatible with rtos architecture, each of this 4 workers should
     be implemented in differents threads, each of them waiting for job to be done:
     periodic task should sleep, and event task should wait for event
     */
  (void) arg;
  chRegSetThreadName("pprz big loop");

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "CHIBIOS_INFO", send_chibios_info);
#endif

  while (!chThdShouldTerminateX()) {
    Fbw(handle_periodic_tasks);
    Ap(handle_periodic_tasks);
    Fbw(event_task);
    Ap(event_task);
    chThdSleepMilliseconds(1);
  }

}
