# Hey Emacs, this is a -*- makefile -*-

# Auto generated fixed wing control loops


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_generated.c $(SRC_FIRMWARE)/guidance/guidance_v.c modules/simulink/roll_loop/roll_loop.c

$(TARGET).CFLAGS += -DCTRL_TYPE_H=\"firmwares/fixedwing/guidance/guidance_v.h\"
#$(TARGET).CFLAGS += -DCTRL_VERTICAL_LANDING=1
