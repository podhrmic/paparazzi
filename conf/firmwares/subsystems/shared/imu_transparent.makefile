# IMU Transparent
# 2014, Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
# Utah State University, http://aggieair.usu.edu/

IMU_CFLAGS += -DUSE_IMU
IMU_CFLAGS += -DUSE_IMU_FLOAT

IMU_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_transparent.h\"
IMU_SRCS   += $(SRC_SUBSYSTEMS)/imu.c
IMU_SRCS   += subsystems/imu/imu_transparent.c

ap.CFLAGS += $(IMU_CFLAGS)
ap.srcs += $(IMU_SRCS)

# add nps flags
