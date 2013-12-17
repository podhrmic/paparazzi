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

#include "ch.h"
#include "hal.h"

static void spicb(SPIDriver *spip);

/*
 *
 *  Regular GPIO driven LEDs
 *
 */
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i

#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)

#define LED_ON(i) palClearPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_OFF(i) palSetPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_TOGGLE(i) palTogglePad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_PERIODIC() {}

/*
 * SPI2 configuration structure.
 * Speed 21MHz, CPHA=0, CPOL=0, 16bits frames, MSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOA.
 */
static const SPIConfig spi_baro_cfg = {
  spicb,
  GPIOC,  /* HW dependent part.*/
  13,
  //SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA
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


static WORKING_AREA(waHeartBeat, 128);
static msg_t heartBeat(void *param) {

  systime_t time = chTimeNow();
  while (TRUE) {
    time += MS2ST(1000);
    LED_TOGGLE(1);
	chThdSleepUntil(time);
  }
}

/*
 * SPI Thread
 */
static WORKING_AREA(waSpiTest, 128);
static __attribute__((noreturn)) msg_t thd_spiTest(void *arg)
{
  chRegSetThreadName("pprz spiTest");
  (void) arg;
  systime_t time = chTimeNow();     // T0
  //uint8_t spi_buf[10] = { 0x36, 0x80, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, '\r' };
  static uint8_t spi_buf[10] = { 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0x11, '\r' };
  static uint8_t baro_buf[1];
  baro_buf[0] = 0xAA;
  while (TRUE) {
    time += MS2ST(10);            // Next deadline, sleep for one sec.
    LED_TOGGLE(4);
    /* SPI code */
    spiStart(&SPID2, &spi_baro_cfg);
    spiSelect(&SPID2);
    spiStartSend(&SPID2, sizeof(baro_buf), baro_buf);

	chThdSleepUntil(time);
  }
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
  //LUFTBOOT Hack
  SCB_VTOR = 0x000002000;





  /*
   * Creates the heartbeat
   */
  chThdCreateStatic(waHeartBeat, sizeof(waHeartBeat), NORMALPRIO, heartBeat, NULL);

  /*
   * SPI
   */
  chThdCreateStatic(waSpiTest, sizeof(waSpiTest),NORMALPRIO, thd_spiTest, NULL);

  while (TRUE) {
      chThdSleepMilliseconds(1000);
  }
}

