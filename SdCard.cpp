/* Arduino FAT16 Library
 * Copyright (C) 2008 by William Greiman
 *
 * This file is part of the Arduino FAT16 Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with the Arduino Fat16 Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "SdCard.h"

//#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>

#include "Fat16Config.h"
#include "mSPI.h"


/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))



//------------------------------------------------------------------------------
#if 0//////////////////////////////////////////////////////////////////////////////////////////////////
// r1 status values
uint8_t const R1_READY_STATE = 0;
uint8_t const R1_IDLE_STATE  = 1;
// start data token for read or write
uint8_t const DATA_START_BLOCK = 0XFE;
// data response tokens for write block
uint8_t const DATA_RES_MASK        = 0X1F;
uint8_t const DATA_RES_ACCEPTED    = 0X05;
uint8_t const DATA_RES_CRC_ERROR   = 0X0B;
uint8_t const DATA_RES_WRITE_ERROR = 0X0D;
#endif////////////////////////////////////////////////////////////////////////////////////////////////////
//
// stop compiler from inlining where speed optimization is not required
//#define STATIC_NOINLINE static __attribute__((noinline))

uint8_t spiReceive(void) {
  
  return SPI_transfer(0xff);
}

bool waitNotBusy(void) {
  uint16_t count = 0;
  while (spiReceive() != 0xff) {
    if ( count >= SD_MAX_TRANSFERTS ) return false;
  }
  return true;
}

void chipSelectHigh(void) {
  
  SD_SS_PORT_REG |= _BV(SD_SS_PIN);
}

void chipSelectLow(void) {

  SD_SS_PORT_REG &= ! _BV(SD_SS_PIN);
}

uint8_t cardCommand(uint8_t cmd, uint32_t arg) {

  /* wait not busy */
  if (cmd != CMD0) {
    waitNotBusy();
  }

  // send command
  SPI_transfer(cmd | 0x40);

  // send argument
  uint8_t *pa = reinterpret_cast<uint8_t *>(&arg);
  for (int8_t i = 3; i >= 0; i--) {
    SPI_transfer(pa[i]);
  }

  // send CRC - correct for CMD0 with arg zero or CMD8 with arg 0X1AA
  SPI_transfer(cmd == CMD0 ? 0X95 : 0X87);

  // discard first fill byte to avoid MISO pull-up problem.
  spiReceive();

  // there are 1-8 fill bytes before response.  fill bytes should be 0XFF.
  uint8_t status;
  for (uint8_t i = 0; ((status = spiReceive()) & 0X80) && i < 10; i++) {
  }
  return status;
}

uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
  
  cardCommand(CMD55, 0);
  return cardCommand(cmd, arg);
}



uint8_t SdCard_begin(void) {

  uint8_t status;

  // 16-bit init start time allows over a minute
  uint16_t count = 0;
  uint32_t arg;
  uint8_t cardType;

  // initialize SPI bus and chip select pin.
  DDRB = _BV(2) | _BV(3) | _BV(5); //SS, MOSI, SCK
  SD_SS_DDR_REG |= _BV(SD_SS_PIN); //SD SS as OUTPUT
  chipSelectHigh();

  // set SCK rate for initialization commands.
  SPCR = SPI_SPCR_CONFIG | SPI_SCK_INIT_DIVISOR;
  SPSR = 0x01; //enable SPI2X
  chipSelectLow();
  
  // must supply min of 74 clock cycles with CS high.
  chipSelectHigh();
  for (uint8_t i = 0; i < 10; i++) {
    spiReceive();
  }
  
  // command to go idle in SPI mode
  chipSelectLow();
  while (cardCommand(CMD0, 0) != R1_IDLE_STATE) {
    count++;
    if( count > SD_MAX_CMD0_TRIES ) {
      goto fail;
    }
  }

  // check SD version
  if (cardCommand(CMD8, 0x1AA) == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE)) {
    cardType = CARD_TYPE_SDV1;
  } else {
    for (uint8_t i = 0; i < 4; i++) {
      status = spiReceive();
    }
    if (status == 0XAA) {
      cardType = CARD_TYPE_SDV2;
    } else {
      goto fail;
    }
  }

  // initialize card and send host supports SDHC if SD2
  arg = cardType == CARD_TYPE_SDV2 ? 0X40000000 : 0;

  count = 0;
  while (cardAcmd(ACMD41, arg) != R1_READY_STATE) {
    count++;
    if( count > SD_MAX_COMMANDS ) {
      goto fail;
    }
  }
    
  // if SD2 read OCR register to check for SDHC card
  if (cardType == CARD_TYPE_SDV2) {
    if (cardCommand(CMD58, 0)) {
      goto fail;
    }
    if ((spiReceive() & 0XC0) == 0XC0) {
      cardType = CARD_TYPE_SDHC;
    }
    // Discard rest of ocr - contains allowed voltage range.
    for (uint8_t i = 0; i < 3; i++) {
      spiReceive();
    }
  }

  /* reset SPI clock */
  SPCR = SPI_SPCR_CONFIG;
  chipSelectHigh();
  spiReceive();
  return cardType;

 fail:
  chipSelectHigh();
  spiReceive();
  return 0;
}


//------------------------------------------------------------------------------
/**
 * Reads a 512 byte block from a storage device.
 *
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdCard_readBlock(uint32_t blockNumber, uint8_t cardType) {

  uint8_t status;
  uint16_t count;

  /* start SPI */
  chipSelectLow();
  
  /* get block number */
  if (cardType != CARD_TYPE_SDHC) {
    blockNumber <<= 9;
  }
  if (cardCommand(CMD17, blockNumber)) {
    goto fail;
  }
  
  /********/
  /* read */
  /********/
  // wait for start block token
  count = 0;
  while ((status = spiReceive()) == 0XFF) {
    count++;
    if (count > SD_MAX_TRANSFERTS) {
      goto fail;
    }
  }
  if (status != DATA_START_BLOCK) {
    goto fail;
  }
  // transfer data
  for (int i = 0; i < 512; i++) {
    buff[i] = spiReceive();
  }
  
  // discard crc
  spiReceive();
  spiReceive();

  // ok
  chipSelectHigh();
  spiReceive();
  return true;

fail:
  chipSelectHigh();
  spiReceive();
  return false;
}

//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to a storage device.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdCard_writeBlock(uint32_t blockNumber, uint8_t cardType) {

  uint8_t status;
 
  /* start SPI */
  chipSelectLow();
    
  /* set block number */
  if (cardType != CARD_TYPE_SDHC) {
    blockNumber <<= 9;
  }
  if (cardCommand(CMD24, blockNumber)) {
    goto fail;
  }

  /*********/
  /* write */
  /*********/
  SPI_transfer(DATA_START_BLOCK);
  for (int i = 0; i < 512; i++) {
    SPI_transfer(buff[i]);
  }
  spiReceive();
  spiReceive();
    
  status = spiReceive();
  if ((status & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    goto fail;
  }

  /**********************/
  /* flush cache buffer */
  /**********************/
  if ( !waitNotBusy() ) {
    goto fail;
  }
  
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiReceive()) {
    goto fail;
  }

  // ok
  chipSelectHigh();
  spiReceive();
  return true;
  
 fail:
  chipSelectHigh();
  spiReceive();
  return false;
}
