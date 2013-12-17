# Rotorcraft IMU module for GX3
# ChibiiOS DMA driver
# 2013, Utah State University, http://aggieair.usu.edu/

GX3_PORT ?= UARTD3
GX3_BAUD ?= B921600

IMU_GX3_CFLAGS  = -DUSE_AHRS
IMU_GX3_CFLAGS += -DUSE_IMU
IMU_GX3_CFLAGS += -DUSE_IMU_FLOAT
IMU_GX3_CFLAGS += -DUSE_GX3

IMU_GX3_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_gx3.h\"
IMU_GX3_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
IMU_GX3_SRCS   += subsystems/imu/imu_gx3.c


IMU_GX3_CFLAGS += -DUSE_$(GX3_PORT) -D$(GX3_PORT)_BAUD=$(GX3_BAUD)
IMU_GX3_CFLAGS += -DUSE_GX3

ap.CFLAGS += $(IMU_GX3_CFLAGS)
ap.srcs += $(IMU_GX3_SRCS)
