/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_aspirin_2_spi.c
 * Driver for the Aspirin v2.x IMU using SPI for the MPU6000.
 */

#include "subsystems/imu.h"
#include "mcu_periph/spi.h"
#include "subsystems/imu/boogie_board.h"

#include "led.h"

static void spicb(SPIDriver *spip);

//struct Boogie_Spi boogie_struct;
struct spi_transaction spi_trans_bg;
static uint8_t baro_buf = 0xA5;
static uint8_t rx_buf[1];

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

void imu_impl_init(void)
{
  spi_trans_bg.output_buf = &baro_buf;
  spi_trans_bg.input_buf = rx_buf;
  spi_trans_bg.spi_cfg = spi_baro_cfg;
}


void imu_periodic(void)
{
  spi_submit(&ASPIRIN_2_SPI_DEV, &spi_trans_bg);
}

