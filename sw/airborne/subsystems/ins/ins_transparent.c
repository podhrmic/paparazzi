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
 * @file ins_transparemt.c
 *
 * Transparent INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/ins/ins_transparent.h"

struct InsInt ins_impl;

void ins_init(void) {
  ins.status = INS_RUNNING;

  ins_impl.ltp_initialized  = FALSE;
  ins_impl.baro_initialized = FALSE;

  INT32_VECT3_ZERO(ins_impl.ltp_pos);
  INT32_VECT3_ZERO(ins_impl.ltp_speed);
  INT32_VECT3_ZERO(ins_impl.ltp_accel);
}
