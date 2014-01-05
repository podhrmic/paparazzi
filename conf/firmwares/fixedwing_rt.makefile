# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2010 The Paparazzi Team
#
# modified by AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
# Utah State University, http://aggieair.usu.edu/
#
# Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
# Calvin Coopmans (c.r.coopmans@ieee.org)
# 2013
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared
CFG_FIXEDWING_RT=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/fixedwing

SRC_BOARD=boards/$(BOARD)
SRC_FIRMWARE=firmwares/fixedwing
SRC_SUBSYSTEMS=subsystems

SRC_ARCH=arch/$(ARCH)

FIXEDWING_INC = -I$(SRC_FIRMWARE) -I$(SRC_BOARD)

#we are using rt fixedwing firmware (unlike rt_rotorcraft), with chibios arch
$(TARGET).CFLAGS += -DUSE_CHIBIOS_RTOS
$(TARGET).CFLAGS += $(FIXEDWING_INC)
$(TARGET).CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -DPERIPHERALS_AUTO_INIT

#
# Main (Using RTOS)
#
ns_srcs	+= $(SRC_FIRMWARE)/main_chibios.c
$(TARGET).srcs 	+= mcu.c
$(TARGET).srcs 	+= $(SRC_ARCH)/mcu_arch.c

#
# Math functions
#
$(TARGET).srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c

#
# LEDs
#
ns_CFLAGS 		+= -DUSE_LED


# frequency of main periodic
PERIODIC_FREQUENCY ?= 60
$(TARGET).CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)

#
# Sys-time
#
$(TARGET).CFLAGS += -DUSE_SYS_TIME
$(TARGET).srcs   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifneq ($(SYS_TIME_LED),none)
  ns_CFLAGS 	+= -DSYS_TIME_LED=$(SYS_TIME_LED)
endif

#
# Telemetry / Datalink
#
ap.srcs += subsystems/settings.c

#
# I2C
#
ifeq ($(TARGET), ap)
  $(TARGET).srcs += mcu_periph/i2c_pprz.c
  $(TARGET).srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
endif

#
# Air data
#
ap.srcs += subsystems/air_data.c

#
# Baro
#
include $(CFG_SHARED)/baro_board.makefile

#
# Analog Backend
#
ns_CFLAGS += -DUSE_ADC
ns_srcs 	+= $(SRC_ARCH)/mcu_periph/adc_arch.c
ns_srcs   += subsystems/electrical.c

#
# UARTS
#
ns_srcs 		+= mcu_periph/uart_pprz.c
ns_srcs 		+= $(SRC_ARCH)/mcu_periph/uart_arch.c


#
# AP
#
ap.srcs += subsystems/commands.c
ap.srcs += subsystems/actuators.c
ap.srcs	+= state.c
ap.srcs += $(SRC_FIRMWARE)/autopilot.c

ap.srcs += $(SRC_FIRMWARE)/stabilization.c

# Navigation
# we have subsystem "navigation.makefile"
#ap.CFLAGS += -DUSE_NAVIGATION
#ap.srcs += $(SRC_FIRMWARE)/navigation.c
#ap.srcs += subsystems/navigation/common_flight_plan.c

#
# Maybe?
#
# ahrs frequencies if configured
#ifdef AHRS_PROPAGATE_FREQUENCY
#ap_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
#endif
#ifdef AHRS_CORRECT_FREQUENCY
#ap_CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)
#endif


#
# No-Sim parameters
#
ap.CFLAGS 		+= $(ns_CFLAGS)
ap.srcs 		+= $(ns_srcs)

