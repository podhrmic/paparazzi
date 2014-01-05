/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> [rt_paparazzi] update 0.3.1
 * @file arch/chibios/mcu_periph/spi_arch.c
 * Implementation of SPI interface for ChibiOS arch
 *
 * Only Master mode is allowed in ChibiOS.
<<<<<<< HEAD
=======
 * @brief chibios arch dependant implementation of SPI interface
 * @note Assume SPI master for now
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
>>>>>>> [rt_paparazzi] update 0.3.1
 */
#include "mcu_periph/spi.h"

#if SPI_SLAVE
#error "ChibiOS operates only in SPI_MASTER mode"
#endif

#if USE_SPI0
#error "ChibiOS architectures don't have SPI0"
#endif
#if USE_SPI1
void spi1_arch_init(void) {
  spi1.reg_addr = &SPID1;
}
#endif
#if USE_SPI2
void spi2_arch_init(void) {
  spi2.reg_addr = &SPID2;
}
#endif
#if USE_SPI3
void spi3_arch_init(void) {
  spi3.reg_addr = &SPID3;
}
#endif

/**
 * Resolve slave port
 *
 * Given the slave number and the board config file, returns the right
 * port (i.e. GPIOC)
 *
 * @param[in] slave index number of a slave
 */
static inline ioportid_t spi_resolve_slave_port(uint8_t slave) {
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      return SPI_SELECT_SLAVE0_PORT;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      return SPI_SELECT_SLAVE1_PORT;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      return SPI_SELECT_SLAVE2_PORT;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      return SPI_SELECT_SLAVE3_PORT;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      return SPI_SELECT_SLAVE4_PORT;
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      return SPI_SELECT_SLAVE5_PORT;
      break;
#endif //USE_SPI_SLAVE5
    default:
      return 0;
      break;
  }
}

/**
<<<<<<< HEAD
<<<<<<< HEAD
 * Resolve slave pin
=======
 * Resolve slave port
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
 * Resolve slave pin
>>>>>>> [rt_paparazzi] update 0.3.1
 *
 * Given the slave number and the board config file, returns the right
 * pin (i.e. 12)
 *
 * @param[in] slave index number of a slave
 */
static inline uint16_t spi_resolve_slave_pin(uint8_t slave) {
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      return SPI_SELECT_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      return SPI_SELECT_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      return SPI_SELECT_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      return SPI_SELECT_SLAVE3_PIN;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      return SPI_SELECT_SLAVE4_PIN;
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      return SPI_SELECT_SLAVE5_PIN;
      break;
#endif //USE_SPI_SLAVE5
    default:
      return 0;
      break;
  }
}

/**
 * Resolve CR1
 *
 * Given the transaction settings, returns the right configuration of
 * SPIx_CR1 register.
 *
<<<<<<< HEAD
<<<<<<< HEAD
 * This function is currently architecture dependent (for STM32F1xx only)
 * TODO: implement for STM32F4 and possible other architectures too
 *
=======
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
 * This function is currently architecture dependent (for STM32F1xx only)
 * TODO: implement for STM32F4 and possible other architectures too
 *
>>>>>>> [rt_paparazzi] update 0.3.1
 * @param[in] t pointer to a @p spi_transaction struct
 */
static inline uint16_t spi_resolve_CR1(struct spi_transaction* t){
  uint16_t CR1 = 0;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  /// The settings are architecture dependent
  /// TODO: Now for STM32F1xx only
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
>>>>>>> [rt_paparazzi] update 0.3.1
#ifdef __STM32F10x_H
  if (t->dss == SPIDss16bit) {
    CR1 |= SPI_CR1_DFF;
  }
  if (t->bitorder == SPILSBFirst) {
    CR1 |= SPI_CR1_LSBFIRST;
  }
  if (t->cpha == SPICphaEdge2) {
    CR1 |= SPI_CR1_CPHA;
  }
  if (t->cpol == SPICpolIdleHigh) {
    CR1 |= SPI_CR1_CPOL;
  }

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> [rt_paparazzi] update 0.3.1
  switch (t->cdiv) {
    case SPIDiv2://000
      break;
    case SPIDiv4://001
      CR1 |= SPI_CR1_BR_0;
      break;
    case SPIDiv8://010
      CR1 |= SPI_CR1_BR_1;
      break;
    case SPIDiv16://011
      CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
      break;
    case SPIDiv32://100
      CR1 |= SPI_CR1_BR_2;
      break;
    case SPIDiv64://101
      CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
      break;
    case SPIDiv128://110
      CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
      break;
    case SPIDiv256://111
      CR1 |= SPI_CR1_BR;
      break;
    default:
      break;
  }
<<<<<<< HEAD
#endif
  return CR1;
=======
   switch (t->cdiv) {
     case SPIDiv2://000
       break;
     case SPIDiv4://001
       CR1 |= SPI_CR1_BR_0;
       break;
     case SPIDiv8://010
       CR1 |= SPI_CR1_BR_1;
       break;
     case SPIDiv16://011
       CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
       break;
     case SPIDiv32://100
       CR1 |= SPI_CR1_BR_2;
       break;
     case SPIDiv64://101
       CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
       break;
     case SPIDiv128://110
       CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
       break;
     case SPIDiv256://111
       CR1 |= SPI_CR1_BR;
       break;
     default:
       break;
   }
#endif
   return CR1;
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
#endif
  return CR1;
>>>>>>> [rt_paparazzi] update 0.3.1
}


/**
 * Submit SPI transaction
 *
 * Interafces Paparazzi SPI code with ChibiOS SPI driver.
 * The transaction length is max(rx,tx), before and after
 * callbacks are called accordingly.
 *
 * ChibiOS doesn't provide error checking for the SPI transactions,
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> [rt_paparazzi] update 0.3.1
 * since all spi functions are return void. The SPI transaction is
 * synchronous, so we always assume success if the transaction finishes.
 *
 * There is no explicit timeout on SPI transaction.
 * TODO: Timeout on SPI trans and error detection.
<<<<<<< HEAD
=======
 * since all spi functions are return void. The API is asynchronous.
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
>>>>>>> [rt_paparazzi] update 0.3.1
 *
 * @param[in] p pointer to a @p spi_periph struct
 * @param[in] t pointer to a @p spi_transaction struct
 */
bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  SPIConfig spi_cfg = {
<<<<<<< HEAD
<<<<<<< HEAD
    NULL, // no callback
=======
    NULL, /// no callback
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
    NULL, // no callback
>>>>>>> [rt_paparazzi] Documentation fixes and cleanup
    spi_resolve_slave_port(t->slave_idx),
    spi_resolve_slave_pin(t->slave_idx),
    spi_resolve_CR1(t)
#ifdef __STM32F4xx_H
    ,spi_resolve_CR2(t)
#endif
  };

  // find max transaction length
  static size_t t_length;
  if (t->input_length >= t->output_length) {
    t_length = (size_t)t->input_length;
  }
  else {
    t_length = (size_t)t->output_length;
  }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // Acquire exclusive access to the SPI bus
  spiAcquireBus((SPIDriver*)p->reg_addr);

  // Configure SPI bus with the current slave select pin
  spiStart((SPIDriver*)p->reg_addr, &spi_cfg);
  spiSelect((SPIDriver*)p->reg_addr);

  // Run the callback after selecting the slave
=======
  /// Acquire exclusive access to the spi bus
=======
  /// Acquire exclusive access to the SPI bus
>>>>>>> [rt_paparazzi] update 0.3.1
=======
  // Acquire exclusive access to the SPI bus
>>>>>>> [rt_paparazzi] Documentation fixes and cleanup
  spiAcquireBus((SPIDriver*)p->reg_addr);

  // Configure SPI bus with the current slave select pin
  spiStart((SPIDriver*)p->reg_addr, &spi_cfg);
  spiSelect((SPIDriver*)p->reg_addr);

<<<<<<< HEAD
<<<<<<< HEAD
  /// Run the callback AFTER selecting the slave
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
  /// Run the callback after selecting the slave
>>>>>>> [rt_paparazzi] update 0.3.1
=======
  // Run the callback after selecting the slave
>>>>>>> [rt_paparazzi] Documentation fixes and cleanup
  if (t->before_cb != 0) {
    t->before_cb(t);
  }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // Start synchronous data transfer
  spiExchange((SPIDriver*)p->reg_addr, t_length, t->output_buf, t->input_buf);

  // Unselect the slave
  spiUnselect((SPIDriver*)p->reg_addr);

  // Release the exclusive access to the bus
  spiReleaseBus((SPIDriver*)p->reg_addr);

  // Report the transaction as success
  t->status = SPITransSuccess;

  /*
   * Run the callback after deselecting the slave
   * to avoid recursion and/or concurency over the bus
   */
=======
  /// Start asynchronous data transfer
=======
  /// Start synchronous data transfer
>>>>>>> [rt_paparazzi] update 0.3.1
=======
  // Start synchronous data transfer
>>>>>>> [rt_paparazzi] Documentation fixes and cleanup
  spiExchange((SPIDriver*)p->reg_addr, t_length, t->output_buf, t->input_buf);

  // Unselect the slave
  spiUnselect((SPIDriver*)p->reg_addr);

  // Release the exclusive access to the bus
  spiReleaseBus((SPIDriver*)p->reg_addr);

  // Report the transaction as success
  t->status = SPITransSuccess;

<<<<<<< HEAD
  /// Run the callback AFTER deselecting the slave
  /// to avoid recursion and/or concurency over the bus
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
  /*
   * Run the callback after deselecting the slave
   * to avoid recursion and/or concurency over the bus
   */
>>>>>>> [rt_paparazzi] update 0.3.1
  if (t->after_cb != 0) {
    t->after_cb(t);
  }
  return TRUE;
}



/**
 * spi_slave_select() function
 *
 * Empty, for paparazzi compatibility only
 */
<<<<<<< HEAD
<<<<<<< HEAD
void spi_slave_select(uint8_t slave __attribute__((unused))) {}
=======
void spi_slave_select(uint8_t slave) {
  (void) slave;
}
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
void spi_slave_select(uint8_t slave __attribute__((unused))) {}
>>>>>>> [rt_paparazzi] update 0.3.1

/**
 * spi_slave_unselect() function
 *
 * Empty, for paparazzi compatibility only
 */
<<<<<<< HEAD
<<<<<<< HEAD
void spi_slave_unselect(uint8_t slave __attribute__((unused))) {}
=======
void spi_slave_unselect(uint8_t slave) {
  (void) slave;
}
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
void spi_slave_unselect(uint8_t slave __attribute__((unused))) {}
>>>>>>> [rt_paparazzi] update 0.3.1

/**
 * spi_lock() function
 *
 * Empty, for paparazzi compatibility only
 */
<<<<<<< HEAD
<<<<<<< HEAD
bool_t spi_lock(struct spi_periph* p __attribute__((unused)), uint8_t slave __attribute__((unused))) {
=======
bool_t spi_lock(struct spi_periph* p, uint8_t slave) {
  (void) slave;
  (void) p;
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
bool_t spi_lock(struct spi_periph* p __attribute__((unused)), uint8_t slave __attribute__((unused))) {
>>>>>>> [rt_paparazzi] update 0.3.1
  return TRUE;
}

/**
 * spi_resume() function
 *
 * Empty, for paparazzi compatibility only
 */
<<<<<<< HEAD
<<<<<<< HEAD
bool_t spi_resume(struct spi_periph* p __attribute__((unused)), uint8_t slave __attribute__((unused))) {
=======
bool_t spi_resume(struct spi_periph* p, uint8_t slave) {
  (void) slave;
  (void) p;
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
bool_t spi_resume(struct spi_periph* p __attribute__((unused)), uint8_t slave __attribute__((unused))) {
>>>>>>> [rt_paparazzi] update 0.3.1
  return TRUE;
}

/**
 * spi_init_slaves() function
 *
 * Empty, for paparazzi compatibility only
 */
void spi_init_slaves(void) {}
