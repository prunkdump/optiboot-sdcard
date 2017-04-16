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


bool waitForToken(uint8_t token) {
  uint16_t count = 0;
  while (SPI_transfer(0xff) != token) {
    count++;
    if( count >= SD_MAX_TRANSFERTS ) return false;
  }
  return true;
}

void chipSelectHigh(void) {
  
  SD_SS_PORT_REG |= _BV(SD_SS_PIN);
  SPI_transfer(0Xff);
}

void chipSelectLow(void) {

  SD_SS_PORT_REG &= ! _BV(SD_SS_PIN);
}


uint8_t cardCommand(uint8_t cmd, uint32_t arg) {

  // select card
  chipSelectLow();
  
  // wait if busy
  waitForToken(0xff);

  // send command
  SPI_transfer(cmd | 0x40);

  // send argument
  uint8_t *pa = (uint8_t *)(&arg);
  for (int8_t i = 3; i >= 0; i--) {
    SPI_transfer(pa[i]);
  }
  
  // send CRC - correct for CMD0 with arg zero or CMD8 with arg 0X1AA
  SPI_transfer(cmd == CMD0 ? 0X95 : 0X87);

  // skip stuff byte for stop read
  if (cmd == CMD12) {
    SPI_transfer(0xff);
  }

  // wait for response
  uint8_t status;
  for (uint8_t i = 0; ((status = SPI_transfer(0xff)) & 0X80) && i != 0XFF; i++) {
  }
  
  return status;
}


uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
  
  cardCommand(CMD55, 0);
  return cardCommand(cmd, arg);
}

bool SdCard_begin(uint8_t* cardType) {

  uint8_t status;
  uint32_t arg;
  uint16_t count;

  // initialize chip select pin.
  SD_SS_DDR_REG |= _BV(SD_SS_PIN); //OUTPUT 
  SD_SS_PORT_REG |= _BV(SD_SS_PIN); //HIGH

  /* init SPI */
  SPI_begin();

  /* set slow SPI clock */
  uint8_t save_SPCR = SPCR;
  SPCR |= SPI_SCK_INIT_DIVISOR;

  // toggle chip select 
  chipSelectLow();
  chipSelectHigh();

  // must supply min of 74 clock cycles with CS high.
  for (uint8_t i = 0; i < 10; i++) {
    SPI_transfer(0xff);
  }
  
  // command to go idle in SPI mode
  count = 0;
  while (cardCommand(CMD0, 0) != R1_IDLE_STATE) {
    count++;
    if( count > SD_MAX_COMMANDS ) {
      goto fail;
    }
  }

  // check SD version
  count = 0;
  while (1) {
    count++;
    if (cardCommand(CMD8, 0x1AA) == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE)) {
      *cardType = CARD_TYPE_SDV1;
      break;
    }
    for (uint8_t i = 0; i < 4; i++) {
      status = SPI_transfer(0xff);
    }
    if (status == 0XAA) {
      *cardType = CARD_TYPE_SDV2;
      break;
    }
    if ( count > SD_MAX_COMMANDS ) {
      goto fail;
    }
  }

    
  // initialize card and send host supports SDHC if SD2
  arg = *cardType == CARD_TYPE_SDV2 ? 0X40000000 : 0;

  count = 0;
  while (cardAcmd(ACMD41, arg) != R1_READY_STATE) {
    count++;
    // check for timeout
    if (count > SD_MAX_COMMANDS) {
      goto fail;
    }
  }
    
  // if SD2 read OCR register to check for SDHC card
  if (*cardType == CARD_TYPE_SDV2) {
    if (cardCommand(CMD58, 0)) {
      goto fail;
    }
    if ((SPI_transfer(0xff) & 0XC0) == 0XC0) {
      *cardType = CARD_TYPE_SDHC;
    }
    // Discard rest of ocr - contains allowed voltage range.
    for (uint8_t i = 0; i < 3; i++) {
      SPI_transfer(0xff);
    }
  }
  
  chipSelectHigh();

  /* restore SPI clock */
  SPCR = save_SPCR;
  return true;

 fail:
  chipSelectHigh();
  return false;
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
bool SdCard_readBlock(uint32_t blockNumber, uint8_t* dst, uint8_t cardType) {

  uint8_t status;
  uint16_t count;
  
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
  while ((status = SPI_transfer(0xff)) == 0XFF) {
    count++;
    if (count > SD_MAX_TRANSFERTS) {
      goto fail;
    }
  }
  if (status != DATA_START_BLOCK) {
    goto fail;
  }
  // transfer data
  for (uint16_t i = 0; i < 512; i++) {
    dst[i] = SPI_transfer(0XFF);
  }
  
  // discard crc
  SPI_transfer(0XFF);
  SPI_transfer(0XFF);

  // ok
  chipSelectHigh();
  return true;

fail:
  chipSelectHigh();
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
bool SdCard_writeBlock(uint32_t blockNumber, const uint8_t* src, uint8_t cardType) {

  uint8_t status;
  
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
  SPI_transfer(0xff);
  SPI_transfer(0xff);
  
  SPI_transfer(DATA_START_BLOCK);
  for (uint16_t i = 0; i < 512; i++) {
    SPI_transfer(src[i]);
  }
  SPI_transfer(0xff);
  SPI_transfer(0xff);
  
  status = SPI_transfer(0xff);
  if ((status & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    goto fail;
  }

  // ok
  chipSelectHigh();
  return true;
  
 fail:
  chipSelectHigh();
  return false;
}
