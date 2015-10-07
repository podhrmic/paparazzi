/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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

#ifndef CHIBIOS_STUB_H
#define CHIBIOS_STUB_H
#include <stdint.h>

extern void chibios_chSysLock (void);
extern void chibios_chSysUnlock (void);
extern void chibios_chSysLockFromISR (void);
extern void chibios_chSysUnlockFromISR (void);
extern void chibios_chSysDisable (void);
extern void chibios_chSysEnable (void);
extern void chibios_chRegSetThreadName (const char* thdName);
extern void chibios_chThdSleepMilliseconds(uint32_t ms);
extern void chibios_chThdSleepMicroseconds(uint32_t us);
extern uint32_t chibios_chTimeNow(void);


#endif
