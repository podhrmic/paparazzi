/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file main_chibios.c
 * Main file for ChibiOS/RT Paparazzi fixedwing
 *
 * Includes both Paparazzi and ChibiOS files, threads are static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */
#include "firmwares/fixedwing/main_chibios.h"
#include "firmwares/fixedwing/main_threads.h"




/********** PERIODIC MESSAGES ************************************************/
#if PERIODIC_TELEMETRY
static void send_chibios_info(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t time_now = 0;
  time_now = chVTGetSystemTime()/CH_CFG_ST_FREQUENCY;

  // Mutex guard
  chMtxLock(&mtx_sys_time);

  pprz_msg_send_CHIBIOS_INFO(trans, dev, AC_ID,
                    &core_free_memory,
                    &time_now,
                    &thread_counter,
                    &cpu_frequency);

  // Mutex guard
  chMtxUnlock(&mtx_sys_time);
}
#endif


/**
 * Main loop
 *
 * Initializes system (both chibios and paparazzi),
 * then turns into main thread - main_periodic()
 */
int main(void)
{
  init_fbw();

#if DOWNLINK
  downlink_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "CHIBIOS_INFO", send_chibios_info);
#endif

  /*
   * Creates threads
   */
  spawn_threads();

  while (TRUE)
  {
    sys_time_ssleep(1);
  }

  return TRUE;
}

