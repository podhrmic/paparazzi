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

#define XSTR(x) STR(x)
#define STR(x) #x
#pragma message "The value of STM32_TIMCLK1: " XSTR(STM32_TIMCLK1)
#pragma message "The value of STM32_TIMCLK2: " XSTR(STM32_TIMCLK2)

#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i

#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)

#define LED_ON(i) palClearPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_OFF(i) palSetPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_TOGGLE(i) palTogglePad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_PERIODIC() {}


static void pwmpcb(PWMDriver *pwmp) {
  (void)pwmp;
}

static void pwmc1cb(PWMDriver *pwmp) {
  (void)pwmp;
  LED_TOGGLE(3);
}

static PWMConfig pwmcfg = {
  1000000,                                    /* 10kHz PWM clock frequency.   */
  3000,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
#if STM32_PWM_USE_ADVANCED
  0
#endif
};

icucnt_t last_width, last_period;

static void icuwidthcb(ICUDriver *icup) {
  LED_TOGGLE(4);
  last_width = icuGetWidth(icup);
}

static void icuperiodcb(ICUDriver *icup) {
  LED_TOGGLE(3);
  last_period = icuGetPeriod(icup);
}

static void icuoverflowcb(ICUDriver *icup) {
  (void)icup;
  LED_TOGGLE(2);
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_LOW,
  1000000,                                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  NULL,
  NULL,
  ICU_CHANNEL_3,
  0
};

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

#if LUFTBOOT
  PRINT_CONFIG_MSG("We are running luftboot, the interrupt vector is being relocated.")
  SCB_VTOR = 0x00002000;
#endif  /*
   * Initializes the PWM driver 1 and ICU driver 4.
   */
  //pwmStart(&PWMD5, &pwmcfg);
  //palSetPadMode(IOPORT1, 8, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  icuStart(&ICUD1, &icucfg);
  icuEnable(&ICUD1);
  //icuObjectInit(&ICUD1);
  //ICUD1.tim = STM32_TIM1;

/*
  rccEnableTIM1(FALSE);
  rccResetTIM1();
  nvicEnableVector(STM32_TIM1_UP_NUMBER,
                   CORTEX_PRIORITY_MASK(STM32_ICU_TIM1_IRQ_PRIORITY));
  nvicEnableVector(STM32_TIM1_CC_NUMBER,
                   CORTEX_PRIORITY_MASK(STM32_ICU_TIM1_IRQ_PRIORITY));
  uint32_t clock;
  uint32_t frequency = 1000000;
  clock = STM32_TIMCLK2;

  stm32_tim_t *tim;
  tim = STM32_TIM1;
  tim->CR1 &= ~(STM32_TIM_CR1_CKD_MASK | STM32_TIM_CR1_CMS_MASK | STM32_TIM_CR1_DIR);
  tim->CR1 |= STM32_TIM_CR1_CKD(0) | STM32_TIM_CR1_CMS(0);

  tim->ARR   = 0xFFFF;
  uint32_t psc;
  psc = (clock / frequency) - 1;
  tim->PSC  = (uint16_t)psc;
  tim->CCMR2 &= ~(STM32_TIM_CCMR2_CC3S_MASK);
  tim->CCMR2 |= (STM32_TIM_CCMR2_CC3S(5) | STM32_TIM_CCMR2_IC3PSC(0) | STM32_TIM_CCMR2_IC3F(0));
  tim->DIER |= (STM32_TIM_DIER_CC3IE | STM32_TIM_DIER_UIE);
  tim->CCER |= STM32_TIM_CCER_CC3E;
  tim->CR1 |= STM32_TIM_CR1_CEN;
  */



  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
    LED_TOGGLE(1);
    chThdSleepMilliseconds(500);
    LED_TOGGLE(2);
    chThdSleepMilliseconds(100);
    LED_TOGGLE(3);
    chThdSleepMilliseconds(100);
  }
  return 0;
}
