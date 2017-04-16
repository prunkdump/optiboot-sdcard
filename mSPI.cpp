/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "mSPI.h"

#include <inttypes.h>
#include <avr/io.h>

void SPI_begin(void) {

  /* set hardware spi SS pin to output */
  /* but do not set to high as it is not used */
  DDRB |= _BV(2);
  
  /* enable SPI */
  SPCR = SPI_SPCR_CONFIG;

  /* Set clock and MOSI to output */
  DDRB |= _BV(5); //SCK
  DDRB |= _BV(3); //MOSI
}
 
uint8_t SPI_transfer(uint8_t data) {

    SPDR = data;
    /*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
     */
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    return SPDR;
}
