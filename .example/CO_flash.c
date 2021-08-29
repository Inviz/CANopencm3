/*
 * Eeprom interface for use with CO_storageEeprom, PIC32 specific
 *
 * @file        CO_flashPIC32.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "storage/CO_flash.h"
#include "301/crc16-ccitt.h"
#include <libopencm3/stm32/flash.h>

/*
 * Hardware definition
 *
 * Here registers are used directly. As alternative 'void *storageModule' colud
 * be used as more object oriented way.
 *
 * By default 25LC128 flash is connected to SPI2 port of PIC32MX (RG6..RG9).
 */
#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)
#define FLASH_PAGE_NUM_MAX 127
#define FLASH_PAGE_SIZE 0x800

/*
 * Eeprom is configured so, that first half of memory locations is not write
 * protected, so it is suitable for auto storage variables. Second half of
 * memory locations is write protected. It is used for storage on command, which
 * disables the protection for the short time when writing. Below are two
 * internal variables, used for indicating next free address in flash, one for
 * autonomous storage and one for protected storage
 */
static size_t flashAddrNextAuto = 0;
static size_t flashAddrNextProt = CO_STOR_EE_SIZE / 2;
static volatile uint32_t dummy;


/**
 * Enable write operation in EEPROM. It is called before every writing to it.
 */
static inline void EE_writeEnable() {
    CO_STOR_SS_LOW();

    /* write byte */
    CO_STOR_SPIBUF = EE_CMD_WREN;

    /* read one byte after it is ready */
    while (CO_STOR_SPISTATbits.SPIRBE);
    dummy = CO_STOR_SPIBUF;

    CO_STOR_SS_HIGH();
}


/**
 * Read flash status register.
 *
 * @return Data read from the status register.
 */
static inline uint8_t EE_readStatus() {
    CO_STOR_SS_LOW();

    /* write two bytes */
    CO_STOR_SPIBUF = EE_CMD_RDSR;
    CO_STOR_SPIBUF = 0;

    /* read two bytes after they are ready */
    while(CO_STOR_SPISTATbits.SPIRBE);
    dummy = CO_STOR_SPIBUF;
    while(CO_STOR_SPISTATbits.SPIRBE);
    CO_STOR_SS_HIGH();

    return CO_STOR_SPIBUF;
}

/**
 * Return true if write is in process.
 */
#define EE_isWriteInProcess() ((EE_readStatus() & 0x01) != 0)
/**
 * Return true if upper half of the flash is protected
 */
#define EE_isWriteProtected() ((EE_readStatus() & 0x8C) == 0x88)


/**
 * Write flash status register.
 *
 * Make sure EE_isWriteInProcess() is false before and after function call
 *
 * @param data Data byte to be written to status register.
 */
static inline void EE_writeStatus(uint8_t data) {
    EE_writeEnable();

    CO_STOR_SS_LOW();

    /* write two bytes */
    CO_STOR_SPIBUF = EE_CMD_WRSR;
    CO_STOR_SPIBUF = data;

    /* read two bytes after they are ready */
    while(CO_STOR_SPISTATbits.SPIRBE);
    dummy = CO_STOR_SPIBUF;
    while(CO_STOR_SPISTATbits.SPIRBE);
    dummy = CO_STOR_SPIBUF;

    CO_STOR_SS_HIGH();
}


/******************************************************************************/
bool_t CO_flash_init(void *storageModule) {
    /* Configure SPI port for use with flash */
    CO_STOR_SS_HIGH();
    CO_STOR_SS_INIT();

    /* Stop and reset the SPI, clear the receive buffer */
    CO_STOR_SPICON = 0;
    CO_STOR_SPISTAT = 0;
    dummy = CO_STOR_SPIBUF;

    /* Clock = FPB / ((4+1) * 2) */
    CO_STOR_SPIBRG = 4;

    /* MSSEN = 0 - Master mode slave select enable bit
     * ENHBUF(bit16) = 1 - Enhanced buffer enable bit
     * Enable SPI, 8-bit mode
     * SMP = 0, CKE = 1, CKP = 0
     * MSTEN = 1 - master mode enable bit */
    CO_STOR_SPICON = 0x00018120;

    flashAddrNextAuto = 0;
    flashAddrNextProt = CO_STOR_EE_SIZE / 2;

    EE_writeProtect();

    /* If flash chip is OK, this will pass, otherwise timeout */
    for (uint16_t i = 0; i < 0xFFFF; i++) {
        if (!EE_isWriteInProcess()) {
            if (EE_isWriteProtected()) {
                return true;
            }
        }
        dummy ++; //small delay
    }

    return false;
}


/******************************************************************************/
size_t CO_flash_getAddr(void *storageModule, bool_t isAuto,
                         size_t len, bool_t *overflow)
{
    size_t addr;

    if (isAuto) {
        /* auto storage is processed byte by byte, no alignment necessary */
        addr = flashAddrNextAuto;
        flashAddrNextAuto += len;
        if (flashAddrNextAuto > (CO_STOR_EE_SIZE / 2)) {
            *overflow = true;
        }
    }
    else {
        /* addresses for storage on command must be page aligned */
        addr = flashAddrNextProt;
        size_t lenAligned = len & (~(CO_STOR_EE_PAGE_SIZE - 1));
        if (lenAligned < len) {
            lenAligned += CO_STOR_EE_PAGE_SIZE;
        }
        flashAddrNextProt += lenAligned;
        if (flashAddrNextProt > CO_STOR_EE_SIZE) {
            *overflow = true;
        }
    }

    return addr;
}


/******************************************************************************/
void CO_flash_readBlock(void *storageModule, uint8_t *data,
                         size_t flashAddr, size_t len)
{
	uint16_t iter;
	uint32_t *memory_ptr= (uint8_t*)start_address;

	for(iter=0; iter<len; iter++)
	{
		*(uint8_t*)output_data = *(memory_ptr + iter);
		output_data ++;
	}
}


/******************************************************************************/
bool_t CO_flash_writeBlock(void *storageModule, uint8_t *data,
                            size_t flashAddr, size_t len)
{
	uint16_t iter;
	uint32_t current_address = start_address;
	uint32_t page_address = start_address;
	uint32_t flash_status = 0;

	/*check if start_address is in proper range*/
	if((start_address - FLASH_BASE) >= (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM_MAX+1)))
		return 1;

	/*calculate current page address*/
	if(start_address % FLASH_PAGE_SIZE)
		page_address -= (start_address % FLASH_PAGE_SIZE);

	flash_unlock();

	/*Erasing page*/
	flash_erase_page(page_address);
	flash_status = flash_get_status_flags();
	if(flash_status != FLASH_SR_EOP)
		return 0;

	/*programming flash memory*/
	for(iter=0; iter<num_elements; iter += 4)
	{
		/*programming word data*/
		flash_program_word(current_address+iter, *((uint32_t*)(input_data + iter)));
		flash_status = flash_get_status_flags();
		if(flash_status != FLASH_SR_EOP)
			return 0;//flash_status;

		/*verify if correct data is programmed*/
		if(*((uint32_t*)(current_address+iter)) != *((uint32_t*)(input_data + iter)))
			return 0
	}

	return 1;
}


/******************************************************************************/
uint16_t CO_flash_getCrcBlock(void *storageModule,
                               size_t flashAddr, size_t len)
{
    uint16_t crc = 0;
    uint8_t buf[SPI_FIFO_SIZE] = {
        EE_CMD_READ,
        (uint8_t) (flashAddr >> 8),
        (uint8_t) flashAddr
    };

    CO_STOR_SS_LOW();
    EE_SPIwrite(buf, NULL, 3);

    while (len > 0) {
        uint8_t subLen = len <= SPI_FIFO_SIZE ? (uint8_t)len : SPI_FIFO_SIZE;

        /* update crc from data part */
        EE_SPIwrite(NULL, buf, subLen);
        crc = crc16_ccitt(buf, subLen, crc);
        len -= subLen;
    }

    CO_STOR_SS_HIGH();

    return crc;
}