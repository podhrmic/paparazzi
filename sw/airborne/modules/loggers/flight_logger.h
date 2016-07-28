/*
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/loggers/flight_logger.h"
 * @author Michal Podhradsky <http://github.com/podhrmic>
 * Send telemetry messages over a serial port to external logger.
 */

#ifndef FLIGHT_LOGGER_H
#define FLIGHT_LOGGER_H

/** Init function
 */
extern void flight_logger_init(void);

/** Periodic function
 *
 * should be called at TELEMETRY_FREQUENCY
 */
extern void flight_logger_periodic(void);

#endif /* FLIGHT_LOGGER_H */

