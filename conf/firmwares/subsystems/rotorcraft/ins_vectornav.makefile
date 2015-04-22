#
# simple with float vertical and horizontal filters for INS
# Universal Serial Driver
#

VN_PORT ?= UART2
VN_BAUD ?= B921600

VN_CFLAGS += -DUSE_$(VN_PORT) -D$(VN_PORT)_BAUD=$(VN_BAUD)

# The commented stuff might be needed for older versions of paparazzi
# which are using macros to access serial ports
# tested on 5.2 and works fine like this
#ifneq (,$(findstring USE_CHIBIOS_RTOS,$($(TARGET).CFLAGS)))
VN_PORT_LOWER=$(shell echo $(VN_PORT) | tr A-Z a-z)
VN_CFLAGS += -DVN_PORT=$(VN_PORT_LOWER)
#else
#VN_CFLAGS += -DVN_PORT=$(VN_PORT)
#endif


VN_CFLAGS += -DINS_TYPE_H=\"subsystems/ins/ins_vectornav.h\"
VN_srcs += $(SRC_SUBSYSTEMS)/ins.c
VN_srcs += $(SRC_SUBSYSTEMS)/ins/ins_vectornav.c

ap.CFLAGS += $(VN_CFLAGS)
ap.srcs += $(VN_srcs)

# add targets for nps...
