#ifndef BARO_MS56111_SPI_H
#define BARO_MS56111_SPI_H

extern float ms5611_fbaroms, ms5611_ftempms;

void baro_ms5611_init(void);
void baro_ms5611_periodic(void);
void baro_ms5611_event(void);

#endif
