/*
 * Copyright (C) 2014 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
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
 * @file imu_transparemt.h
 *
 * Transparent IMU subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef IMU_TRANSPARENT_H
#define IMU_TRANSPARENT_H

#include "subsystems/imu.h"

/* include dummy sensitivity data so it compiles without errors*/
#include "subsystems/imu/imu_mpu60x0_defaults.h"

#ifdef ImuScaleGyro
#undef ImuScaleGyro
#endif
#define ImuScaleGyro(_imu) {}

#ifdef ImuScaleAccel
#undef ImuScaleAccel
#endif
#define ImuScaleAccel(_imu) {}

#ifdef ImuScaleMag
#undef ImuScaleMag
#endif
#define ImuScaleMag(_imu) {}

static inline void ImuEvent(
    void (* _gyro_handler)(void) __attribute__((unused)),
    void (* _accel_handler)(void) __attribute__((unused)),
    void (* _mag_handler)(void) __attribute__((unused)) ) {}

#endif /* IMU_TRANSPARENT_H*/
