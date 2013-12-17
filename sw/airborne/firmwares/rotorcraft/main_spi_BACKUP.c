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
 *
 * @brief Main file for ChibiOS/RT Paparazzi
 * @details Includes both Paparazzi and ChibiOs files
 * 			Threads are static, the memory allocation is
 * 			just approximate at this point. Eventually
 * 			most of the variables should be static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */

/**
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

/**
 * Paparazzi includes
 */
#include "led.h"
//#include "mcu.h"




/*
 * Thread definitions
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
static __attribute__((noreturn)) msg_t thd_spi_test(void *arg);


static void spicb(SPIDriver *spip);

/*
 * SPI2 configuration structure.
 * Speed 21MHz, CPHA=0, CPOL=0, 16bits frames, MSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOA.
 */
static const SPIConfig spi_baro_cfg = {
  spicb,
  GPIOC,  /* HW dependent part.*/
  13,
  SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static const SPIConfig spi_aspirin_cfg = {
  spicb,
  GPIOB,  /* HW dependent part.*/
  12,
  SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/*
 * SPI end transfer callback.
 */
static void spicb(SPIDriver *spip) {

  /* On transfer end just releases the slave select line.*/
  chSysLockFromIsr();
  spiUnselectI(spip);
  chSysUnlockFromIsr();
}

/*
 * HeartBeat & System Info
 */
static WORKING_AREA(wa_thd_heartbeat, 128);
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  chRegSetThreadName("pprz heartbeat");
  (void) arg;
  systime_t time = chTimeNow();     // Current time
  while (TRUE) {
    time += MS2ST(1000);            // Next deadline, sleep for one sec.
    LED_TOGGLE(SYS_TIME_LED);
    chThdSleepUntil(time);
  }
}






static WORKING_AREA(wa_thd_spi_test, 128);
static __attribute__((noreturn)) msg_t thd_spi_test(void *arg)
{
  chRegSetThreadName("pprz_spi_test");
  (void) arg;
  systime_t time = chTimeNow();
  static  uint8_t tx_buf[1];
  tx_buf[0] = 0xAA;
  static uint8_t spi_buf[10] = { 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0x11, '\r' };
  while (TRUE) {
    time += MS2ST(10);
    LED_TOGGLE(2);
    //imu_periodic();
    //spi_submit(&SPID2, &tx_buf);
    spiStart(&SPID2, &spi_baro_cfg);
    spiSelect(&SPID2);
    spiStartSend(&SPID2, sizeof(tx_buf), tx_buf);
    chThdSleepUntil(time);
  }
}

/*
 * Main loop
 * Initializes system (both chibios and paparazzi), then goes to sleep.
 * Eventually Main thread will be turned into Idle thread.
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
  mcu_init();



  /*
   * Thread initialization
   */
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), IDLEPRIO, thd_heartbeat, NULL);
  chThdCreateStatic(wa_thd_spi_test, sizeof(wa_thd_spi_test),NORMALPRIO, thd_spi_test, NULL);


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop. Eventually we want to modify main to an idle thread.
   */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
  return 1;
}
