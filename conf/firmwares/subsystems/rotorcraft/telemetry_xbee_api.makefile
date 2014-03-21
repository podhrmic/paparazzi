# XBee modems in API mode
#
# Expected from board file or overriden as xml param :
#
# MODEM_PORT
# MODEM_BAUD
#

ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD) -DXBEE_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(MODEM_PORT) -DXBEE_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=XBeeTransport -DDATALINK=XBEE -DDefaultPeriodic='&telemetry_Main'

ifneq (,$(findstring USE_CHIBIOS_RTOS,$($(TARGET).CFLAGS)))
MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
ap.CFLAGS += -DDOWNLINK_PORT=$(MODEM_PORT_LOWER)
endif

ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/xbee.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/rotorcraft_telemetry.c
