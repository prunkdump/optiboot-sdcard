#ifndef SPI_H
#define SPI_H

/* SPI CONFIG */
/* No interrupts, SPI enabled, MSB, Master, Mode 0, DIV4 */   
//#define SPI_SPCR_CONFIG 0b01010000
#define SPI_SPCR_CONFIG _BV(SPE) | _BV(MSTR)

#include <inttypes.h>
#include <avr/io.h>

void SPI_begin(void);
uint8_t SPI_transfer(uint8_t data);

#endif
