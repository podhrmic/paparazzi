/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
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
 * Includes both Paparazzi and ChibiOs files, threads aare static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */

/*
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

/*
 * Paparazzi includes
 */
//#include "led.h"
//#include "mcu.h"

/**
 * Main loop
 *
 * Initializes system (both chibios and paparazzi),
 * then turns into main thread - main_periodic()
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  //chThdSleep(MS2ST(1500));
  //systime_t main_time = chTimeNow();
  while (TRUE)
  {

    palClearPad(GPIOA, 8);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOA, 8);
    chThdSleepMilliseconds(100);
  }

  return TRUE;
}

