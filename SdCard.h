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
#ifndef SdCard_h
#define SdCard_h

//#include <inttypes.h>
#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <avr/io.h>

#include "mSPI.h"

/********************/
/* CHIP SELECT PIN  */
/********************/

/* !!! don't works !!! */

/* the default SS pin is PB2 */
#ifndef SD_SS_PORT
#define SD_SS_PORT B
#endif

/* !!! default changed !!! */
#ifndef SD_SS_PIN
#define SD_SS_PIN 0
#endif

/* build register name */
#define SD_SS_PORT_REG PORTC
#define SD_SS_DDR_REG DDRC

/* init at DIV128 */
#define SPI_SCK_INIT_DIVISOR 0x11


//------------------------------------------------------------------------------
// SD operation timeouts

#define SD_MAX_TRANSFERTS UINT16_MAX
#define SD_MAX_COMMANDS 1000


//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
#define CMD0 0X00
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
#define CMD8 0X08
/** SEND_CSD - read the Card Specific Data (CSD register) */
#define CMD9 0X09
/** SEND_CID - read the card identification information (CID register) */
#define CMD10 0X0A
/** STOP_TRANSMISSION - end multiple block read sequence */
#define CMD12 0X0C
/** SEND_STATUS - read the card status register */
#define CMD13 0X0D
/** READ_SINGLE_BLOCK - read a single data block from the card */
#define CMD17 0X11
/** READ_MULTIPLE_BLOCK - read a multiple data blocks from the card */
#define CMD18 0X12
/** WRITE_BLOCK - write a single data block to the card */
#define CMD24 0X18
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
#define CMD25 0X19
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
#define CMD32 0X20
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
#define CMD33 0X21
/** ERASE - erase all previously selected blocks */
#define CMD38 0X26
/** APP_CMD - escape for application specific command */
#define CMD55 0X37
/** READ_OCR - read the OCR register of a card */
#define CMD58 0X3A
/** CRC_ON_OFF - enable or disable CRC checking */
#define CMD59 0X3B
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
#define ACMD23 0X17
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
#define ACMD41 0X29
//------------------------------------------------------------------------------
/** status for card in the ready state */
#define R1_READY_STATE 0X00
/** status for card in the idle state */
#define R1_IDLE_STATE 0X01
/** status bit for illegal command */
#define R1_ILLEGAL_COMMAND 0X04
/** start data token for read or write single block*/
#define DATA_START_BLOCK 0XFE
/** stop token for write multiple blocks*/
#define STOP_TRAN_TOKEN 0XFD
/** start data token for write multiple blocks*/
#define WRITE_MULTIPLE_TOKEN 0XFC
/** mask for data response tokens after a write block operation */
#define DATA_RES_MASK 0X1F
/** write data accepted token */
#define DATA_RES_ACCEPTED 0X05
//------------------------------------------------------------------------------
/** sd card type v1 */
#define CARD_TYPE_SDV1 0x01
/** sd card type v2 */
#define CARD_TYPE_SDV2 0x02
/** sd card type sdhc */
#define CARD_TYPE_SDHC 0x03

//------------------------------------------------------------------------------

bool SdCard_begin(uint8_t* cardType);
bool SdCard_readBlock(uint32_t block, uint8_t* dst, uint8_t cardType);
bool SdCard_writeBlock(uint32_t block, const uint8_t* src, uint8_t cardType);

#endif  // SdCard_h
