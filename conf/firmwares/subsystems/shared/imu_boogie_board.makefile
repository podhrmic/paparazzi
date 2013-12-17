# Hey Emacs, this is a -*- makefile -*-
#
# Common part for Aspirin IMU v2.1 and v2.2
#
#
ifndef SPI_INCLUDED

SPI_INCLUDED = 1

#generic spi master driver
SPI_CFLAGS = -DUSE_SPI -DSPI_MASTER
SPI_SRCS = mcu_periph/spi_pprzi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

ap.CFLAGS += $(SPI_CFLAGS)
ap.srcs += $(SPI_SRCS)

endif

ifeq ($(TARGET), ap)
  IMU_ASPIRIN_2_CFLAGS  = -DUSE_IMU
endif

IMU_ASPIRIN_2_CFLAGS += -DIMU_TYPE_H=\"imu/boogie_board.h\"
IMU_ASPIRIN_2_SRCS    = $(SRC_SUBSYSTEMS)/imu.c
IMU_ASPIRIN_2_SRCS   += $(SRC_SUBSYSTEMS)/imu/boogie_board.c
#IMU_ASPIRIN_2_SRCS   += peripherals/mpu60x0.c
#IMU_ASPIRIN_2_SRCS   += peripherals/mpu60x0_spi.c

include $(CFG_SHARED)/spi_master.makefile

#
# SPI device and slave select defaults
#
# Slave select configuration
# SLAVE2 is on PB12 (NSS) (MPU600 CS)
ASPIRIN_2_SPI_DEV ?= spi2
ASPIRIN_2_SPI_SLAVE_IDX ?= SPI_SLAVE2

ifeq ($(TARGET), ap)
ifndef ASPIRIN_2_SPI_DEV
$(error Error: ASPIRIN_2_SPI_DEV not configured!)
endif
ifndef ASPIRIN_2_SPI_SLAVE_IDX
$(error Error: ASPIRIN_2_SPI_SLAVE_IDX not configured!)
endif
endif

ASPIRIN_2_SPI_DEV_UPPER=$(shell echo $(ASPIRIN_2_SPI_DEV) | tr a-z A-Z)
ASPIRIN_2_SPI_DEV_LOWER=$(shell echo $(ASPIRIN_2_SPI_DEV) | tr A-Z a-z)

IMU_ASPIRIN_2_CFLAGS += -DUSE_$(ASPIRIN_2_SPI_DEV_UPPER)
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_DEV=$(ASPIRIN_2_SPI_DEV_LOWER)

IMU_ASPIRIN_2_CFLAGS += -DUSE_$(ASPIRIN_2_SPI_SLAVE_IDX)
IMU_ASPIRIN_2_CFLAGS += -DASPIRIN_2_SPI_SLAVE_IDX=$(ASPIRIN_2_SPI_SLAVE_IDX)

#
# Baro is connected via SPI, so additionally specify the slave select line for it,
# so that it will be unselected at init (baro CS line high)
#
# SLAVE3 is on PC13, which is the baro CS
IMU_ASPIRIN_2_CFLAGS += -DUSE_SPI_SLAVE3

ap.CFLAGS += $(IMU_ASPIRIN_2_CFLAGS)
ap.srcs   += $(IMU_ASPIRIN_2_SRCS)
