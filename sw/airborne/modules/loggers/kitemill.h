/*
 * Copyright (C) 2014 Michal Podhradsky, Senman, podhrad@pdx.edu
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
 *
 */

/**
 * Serial data transfer to Overo
 *
 */
#ifndef KITEMILL_H
#define KITEMILL_H

#include "mcu_periph/uart.h"

#define __KitemillLink(dev, _x) dev##_x
#define _KitemillLink(dev, _x)  __KitemillLink(dev, _x)
#define KitemillLink(_x) _KitemillLink(KITEMILL_LINK, _x)

#define KitemillBuffer() KitemillLink(ChAvailable())

// Pressure
#include "modules/sensors/baro_ms5611_spi.h"
// GPS
#include "subsystems/gps.h"
// Mag
#include "subsystems/imu.h"
// Attitude, body rates, accel
#include "state.h"
//Servo commands
#include "subsystems/actuators.h"

/*
 * Message headers
 */
#define KITEMILL_MSG0 0x21
#define KITEMILL_MSG1 0x3F

/*
 * Buffer size
 *
 * Note that the size should be larger than the largest packet
 */
#define KITEMILL_BUFFER_SIZE 1024

/* Data offset */
#define KITEMILL_SIZE_IDX 2
#define KITEMILL_DATA_IDX 4

/*
 * This part is used by the autopilot to read data from a uart
 */
#define __LoggerLink(dev, _x) dev##_x
#define _LoggerLink(dev, _x)  __LoggerLink(dev, _x)
#define LoggerLink(_x) _LoggerLink(KITEMILL_LINK, _x)

#define LoggerBuffer() LoggerLink(ChAvailable())

#define ReadLoggerBuffer() {					\
    while (LoggerLink(ChAvailable())&&!logger.msg_available)	\
      kitemill_parse(LoggerLink(Getch()));			\
  }

/*
 * General functions
 */
void kitemill_init(void);
void kitemill_periodic(void);
void kitemill_event(void);

void kitemill_parse(uint8_t c);
void kitemill_read_message(void);


struct KitemillLogger {
  bool_t msg_available;
  uint32_t rxcounter;
  uint32_t txcounter;
};

extern struct KitemillLogger logger;

#endif /* KITEMILL_H */
