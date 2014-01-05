# Hey Emacs, this is a -*- makefile -*-

ifndef SPI_INCLUDED

SPI_INCLUDED = 1

#generic spi master driver
SPI_CFLAGS = -DUSE_SPI -DSPI_MASTER
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
SPI_SRCS = mcu_periph/spi_pprz.c $(SRC_ARCH)/mcu_periph/spi_arch.c
=======
SPI_SRCS = mcu_periph/spi_pprzi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
>>>>>>> [rt_paparazzi] update 0.3.1.
=======
SPI_SRCS = mcu_periph/spi_pprz.c $(SRC_ARCH)/mcu_periph/spi_arch.c
>>>>>>> [rt_paparazzi] update 0.3.1
=======
SPI_SRCS = mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
>>>>>>> [rt_paparazzi] remove "_pprz" suffix in some makefiles to be compatible with master version again

ap.CFLAGS += $(SPI_CFLAGS)
ap.srcs += $(SPI_SRCS)

endif
