/**
  * @file fat_sd.c
  * @author Grant Wilk
  * @created 2/22/2020
  * @modified 2/22/2020
  * @desc 
  */

# include "user_diskio_sd.h"
# include "uart_print.h"

# define DEFAULT_TIMEOUT 1000 // loops or operations

extern SPI_HandleTypeDef hspi2;
DSTATUS         status = STA_NOINIT;

/*
 * SD Card Parameters
 */
SD_CardType     sd_type;
SD_CardCapacity sd_capacity;

/*
 * DISK I/O Functions
 */

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_SD_initialize(BYTE pdrv) {

    sd_response_t sd_response;

    // check valid drive access
    if (pdrv) return STA_NOINIT;

    // check if card is in socket
    if (status & STA_NODISK) return status;

    // send CMD0 (set idle state)
    sd_response = sd_send_command(CMD0, NULL_ARG, CMD0_CRC);

    // verify SD card is in idle state
    if (sd_response.status & STATUS_IDLE) {
        uart_printf("Set SD State:\t\t\tIdle\n");
    } else {
        uart_printf("Set SD State:\t\t\tNOT IDLE\n");
        return STA_NOINIT;
    }

    // send CMD8 (card type check, voltage check, check pattern)
    sd_response = sd_send_command(CMD8, 0x1AA, CMD8_CRC);

    // check SD card type (version)
    if (sd_response.status & STATUS_ILLEGAL_CMD) {
        uart_printf("Card Type:\t\t\t\tSDv1\n");
        sd_type = CT_SDv1;

    } else {
        uart_printf("Card Type:\t\t\t\tSDv2\n");
        sd_type = CT_SDv2;
    }

    // if card is SDv2, check voltage and check pattern
    if (sd_type == CT_SDv2) {
        // verify SD card is receiving proper voltage
        if ((sd_response.data & CMD8_VOLTAGE_ACCEPT)) {
            uart_printf("Check Voltage:\t\t\tOK\n");
        } else {
            uart_printf("Check Voltage:\t\t\tNOT OK\n");
            return STA_NOINIT;
        }
        // verify SD card check pattern
        if ((sd_response.data & CMD8_CHECK_PATTERN) == 0xAA) {
            uart_printf("Check Pattern:\t\t\tOK\n");
        } else {
            uart_printf("Check Pattern:\t\t\tNOT OK\n");
            return STA_NOINIT;
        }
    }

    // send CMD58 (read OCR)
    sd_response = sd_send_command(CMD58, NULL_ARG, NULL_CRC);

    // verify that our 3.3V voltage voltage is supported by the card
    if (sd_response.data & CMD58_3_3V_SUPPORT) {
        uart_printf("Check 3.3V Support:\t\tOK\n");
    } else {
        uart_printf("Check 3.3V Support:\t\tNOT OK\n");
        return STA_NOINIT;
    }

    // if SDv1, check command valid
    if (sd_type == CT_SDv1 && sd_response.status & STATUS_ILLEGAL_CMD) {
        uart_printf("Card Not SD Memory!\n");
        return STA_NOINIT;
    }

    // send CMD55 followed by ACMD41 until it leaves idle state
    uart_printf("Waiting for Ready:");
    uint32_t timeout = DEFAULT_TIMEOUT;
    do {

        // CMD55 tells the card it will receive an application command (ACMD41)
        sd_send_command(CMD55, NULL_ARG, NULL_CRC);

        // ACMD41 starts the cards initialization process
        sd_response = sd_send_command(ACMD41, (1 << 30), NULL_CRC);

        // if SDv1, check command valid
        if (sd_type == CT_SDv1 && sd_response.status & STATUS_ILLEGAL_CMD) {
            uart_printf("Card Not SD Memory!\n");
            return STA_NOINIT;
        }

    } while (sd_response.status & STATUS_IDLE && timeout--);

    // check ready of timeout
    if (timeout) {
        uart_printf("\t\tReady\n");
    } else {
        uart_printf("\t\tTimed Out!\n");
        return STA_NOINIT;
    }

    // check card capacity
    if (sd_type == CT_SDv1) {
        uart_printf("Card Capacity:\t\t\tStandard SD\n");
        sd_capacity = CC_STANDARD;
    } else {
        // send CMD58 (read OCR)
        sd_response = sd_send_command(CMD58, NULL_ARG, NULL_CRC);
        // check card CCS
        if (sd_response.data & CMD58_CCS) {
            uart_printf("Card Capacity:\t\t\tSDXC/SDHC\n");
            sd_capacity = CC_SDHC_SDXC;
        } else {
            uart_printf("Card Capacity:\t\t\tStandard SD\n");
            sd_capacity = CC_STANDARD;
        }
    }

    // if SDv1, set block size = 512 bytes
    if (sd_type == CT_SDv1) {
        sd_response = sd_send_command(CMD16, 512, NULL_CRC);
        // check block size set successful
        if (sd_response.status == 0) {
            uart_printf("Block Size Set:\t\t512\n");
        } else {
            uart_printf("Block Size Set: FAILED\n");
        }
    }

    // deselect the card
    spi_deselect();

    // clear card no init flag
    status &= ~(STA_NOINIT);

    // return drive status
    return status;

}

/**
  * Gets the disk status
  * @param pdrv - physical drive number
  * @retval operation status
  */
DSTATUS USER_SD_status(BYTE pdrv) {

    // check valid drive
    if (pdrv) return STA_NOINIT;

    // return disk status
    return status;

}

/**
 * Reads sectors from the SD card
 * @param pdrv - physical drive number
 * @param buff - pointer to the buffer that should store the received blocks
 * @param sector - sector address formatted in LBA (logical block address)
 * @param count - number of sectors to read
 * @return operation result
 */
DRESULT USER_SD_read(BYTE pdrv, BYTE *buff, uint32_t sector, uint32_t count) {

    // check valid drive access
    if (pdrv) return RES_PARERR;

    // check valid count
    if (!count) return RES_PARERR;

    // check if drive is ready
    if (status & STA_NOINIT) return RES_NOTRDY;

    // read blocks
    int blocks_read;

    if (count == 1) {
        blocks_read = sd_read_block(buff, sector);
    } else {
        blocks_read = sd_read_multi_block(buff, sector, count);
    }

    // blocks read should equal count
    return (blocks_read == count) ? RES_OK : RES_ERROR;

}

/**
  * Writes sectors to the SD card
  * @param pdrv - physical drive number
  * @param buff - pointer to the buffer containing blocks to be transmitted
  * @param sector - sector address formatted in LBA (logical block address)
  * @param count - number of sectors to write
  * @return operation result
  */
DRESULT USER_SD_write(BYTE pdrv, const BYTE *buff, uint32_t sector, uint32_t count) {

    // check valid drive access
    if (pdrv) return RES_PARERR;

    // check valid count
    if (!count) return RES_PARERR;

    // check if drive is ready
    if (status & STA_NOINIT) return RES_NOTRDY;

    // check if drive is write-protected
    if (status & STA_PROTECT) return RES_WRPRT;

    // if SDv1, convert LBA to byte address (multiply by 512)
    if (sd_type == CT_SDv1) sector *= 512;

    // write blocks
    int blocks_written = 0;

    if (count == 1) {
        blocks_written = sd_write_block(buff, sector);
    } else {
        blocks_written = sd_write_multi_block(buff, sector, count);
    }

    // blocks written should equal count
    return (blocks_written == count) ? RES_OK : RES_ERROR;

}

/**
 * Miscellaneous drive controls other than data read/write
 * @param pdrv - physical drive number
 * @param cmd - control command code
 * @param buff - pointer to control data
 * @return operation result
 */
DRESULT USER_SD_ioctrl(BYTE pdrv, BYTE cmd, void * buff) {

    DRESULT res;
    BYTE n;
    BYTE csd[16];
    DWORD *dp, st, ed, csize;

    sd_response_t sd_response;
    BYTE rx_status;


    // check valid drive
    if (pdrv) return RES_PARERR;

    // check drive ready
    if (status & STA_NOINIT) return RES_NOTRDY;

    // default result to error state
    res = RES_ERROR;

    switch (cmd) {

        // wait for end of the card's internal write process
        case CTRL_SYNC:
            if (spi_select()) res = RES_OK;
            break;

        // get the drive capacity in units of sectors
        case GET_SECTOR_COUNT:

            // send CMD9 (send CSD) and receive a partial block
            sd_response = sd_send_command(CMD9, 0, NULL_CRC);
            rx_status = spi_receive_block(csd, 16);

            // if we receive a valid response and a partial block
            if (sd_response.status == 0 && rx_status) {

                // get CSD version
                BYTE csd_version = csd[0] & 0b11 ;

                // CSD v1.XX
                if (csd_version == 0) {
                    // how is this different from CSDv2?
                    res = RES_OK;
                }

                // CSD v2.XX
                else if (csd_version == 1) {

                    // get device size (C_SIZE)
                    csize = (csd[7] >> 6) | (csd[8] << 2) | ((csd[9] & 0b11) << 10);
                    // TODO: give it to the buffer ... somehow
                    res = RES_OK;
                }

                else {
                    uart_printf("Invalid CSD version!\n");
                    res = RES_ERROR;
                }

            }

            break;

        // get erase block size in units of sectors
        case GET_BLOCK_SIZE:
            // don't need to implement for my uses
            break;

        // erase a block of sectors (used when _USER_ERASE == 1)
        case CTRL_TRIM:
            // don't need to implement for my uses
            break;

        // set result to parameter error
        default:
            res = RES_PARERR;
    }

    return res;

}

/*
 * SD Helper Functions
 */

/**
 * Sends an SD command to the SD card
 * @param cmd - command index
 * @param arg - 4-byte command argument
 * @param crc - cyclic redundancy check value
 * @return - the SD card's response
 */
static sd_response_t sd_send_command(BYTE cmd, uint32_t arg, BYTE crc) {

    // print the sent command
    // // uart_printf("Send Command (CMD%d):\t0x%02X / 0x%08X / 0x%02X\n", cmd, cmd, arg, crc | 0x01);

    BYTE txData;
    sd_response_t sd_response;

    // if we're not reading or writing, select the SD card
    if (cmd != CMD17 && cmd != CMD18 && cmd != CMD24 && cmd != CMD25) {
        spi_deselect();
        spi_select();
    }

    // transmit start bit + command index
    txData = (0x40 | cmd);
    spi_tx(&txData);

    // transmit first byte (MSB)
    txData = (BYTE) (arg >> 24);
    spi_tx(&txData);

    // transmit second byte
    txData = (BYTE) (arg >> 16);
    spi_tx(&txData);

    // transmit third byte
    txData = (BYTE) (arg >> 8);
    spi_tx(&txData);

    // transmit fourth byte (LSB)
    txData = (BYTE) arg;
    spi_tx(&txData);

    // transmit CRC with stop bit
    txData = crc | 0x01;
    spi_tx(&txData);

    // get the SD card's response
    sd_response = sd_get_response(cmd);

    // if we're not reading or writing, deselect the SD card
    if (cmd != CMD17 && cmd != CMD18 &&     /* read commands */
        cmd != CMD24 && cmd != CMD25 &&     /* write commands */
        cmd != CMD9  && cmd != CMD10        /* misc r/w commands */
    ) {
        spi_deselect();
    }

    // return the SD card's response
    return sd_response;

}

/**
 * Receives the full 40-bit (5-byte) response from a command
 * @param cmd - command index
 * @param rxBuf - a pointer to the 40-bit (5-byte) receive buffer
 * @return the SD card's response
 */
static sd_response_t sd_get_response(BYTE cmd) {

    BYTE rxData;
    sd_response_t sd_response;

    // wait until we receive the status byte, then store it in the SD response
    uint32_t timeout = DEFAULT_TIMEOUT;
    do {
        spi_rx(&rxData);
    } while (rxData & 0x80 && timeout--);

    sd_response.status = rxData;

    // default retval to zero for most commands
    sd_response.data = 0;

    // if command issues an R7 response, gather the response bytes and reassign retval
    if (cmd == CMD8 || cmd == CMD58) {

        // union for easily converting BYTE[] to uint32_t
        union {
            BYTE rxArr[4];
            uint32_t rxInt;
        } retval;

        // grab sequential bytes from SPI
        for (int i = 4; i; --i) {
            spi_rx(&rxData);
            retval.rxArr[i - 1] = rxData;
        }

        // assign integer component of union as retval
        sd_response.data = retval.rxInt;
    }

    // print the SD card's response
    // // uart_printf("Received Response:\t\t0x%02X / 0x%08X\n", sd_response.status, sd_response.data);

    // return the SD card's response
    return sd_response;

}

/**
 * Reads a block from the SD card
 * @param rxBuf - pointer to the buffer where the received block should be stored
 * @param block_index - the address of the block to read
 * @return the number of blocks read
 */
static int sd_read_block(BYTE * rxBuf, uint32_t block_index) {

    sd_response_t sd_response;

    // select chip
    spi_deselect();
    spi_select();

    // send CMD17 (single block read)
    sd_response = sd_send_command(CMD17, block_index, NULL_CRC);

    // if we don't receive a null status, return 0
    if (sd_response.status) return 0;

    // receive a block from the SPI
    spi_receive_block(rxBuf, BLOCK_SIZE);

    // deselect chip
    spi_deselect();

    // dump the received bytes
    // mydump((uint8_t *) rxBuf, BLOCK_SIZE);

    return 1;

}

/**
 * Reads multiple blocks from the SD card
 * @param rxBuf - pointer to the buffer where the received blocks should be stored
 * @param block_index - the starting address of the blocks to read
 * @param count - the number of blocks to read
 * @return the number of blocks read
 */
static int sd_read_multi_block(BYTE * rxBuf, uint32_t block_index, uint32_t count) {

    uint32_t blocks_read;

    // read the blocks one-by-one
    for (blocks_read = 0; blocks_read < count; blocks_read++) {

        uint32_t rxBufOffset = blocks_read * BLOCK_SIZE;
        uint32_t blockOffset = blocks_read;

        sd_read_block(rxBuf + rxBufOffset, block_index + blockOffset);
    }

    // return the number of blocks read
    return blocks_read;

}

/**
 * Writes a single block to the SD card
 * @param txBuf - pointer to the buffer containing the block to be transmitted
 * @param block_index - the address of the block to write to
 * @return the number of blocks written
 */
static int sd_write_block(const BYTE * txBuf, uint32_t block_index) {

    sd_response_t sd_response;

    // select chip
    spi_deselect();
    spi_select();

    // send CMD24 (single block write)
    sd_response = sd_send_command(CMD24, block_index, NULL_CRC);

    // if we don't receive a null status, return 0
    if (sd_response.status) return 0;

    // transmit a block
    spi_transmit_block(txBuf, BLOCK_SIZE);

    // uart_dump((char *) txBlock, BLOCK_SIZE);

    // deselect the chip
    spi_deselect();

    // return the number of blocks written
    return 1;

}

/**
 * Writes multiple blocks to the SD card
 * @param txBuf - pointer to the buffer that the blocks should be transmitted from
 * @param block_index - the starting address of the blocks to write to
 * @param count - the number of blocks to write
 * @return the number of blocks written
 */
static int sd_write_multi_block(const BYTE *txBuf, uint32_t block_index, uint32_t count) {

    uint32_t blocks_written;
    uint32_t tx_buf_offset;

    // write the blocks one-by-one
    for (blocks_written = 0; blocks_written < count; blocks_written++) {
        tx_buf_offset = blocks_written * BLOCK_SIZE;
        sd_write_block(txBuf + tx_buf_offset, block_index + blocks_written);
    }

    // return the number of blocks written
    return blocks_written;

}

/*
 * SPI RX/TX Functions
 */

/**
 * Transmits a byte of data through the SPI
 * @param txData - a pointer to the byte to transmit
 * @return the transmit status from the SPI
 */
static BYTE spi_tx(BYTE *txData) {
    return HAL_SPI_Transmit(&hspi2, txData, 1, 1000);
}

/**
 * Transmits multiple bytes of data from the SPI
 * @param txBuf - a pointer to the transmit buffer
 * @param size - the number of bytes to transmit
 * @return the transmit status from the SPI
 */
static BYTE spi_tx_multi(BYTE *txBuf, int size) {

    int count;
    BYTE txStatus = 0;

    BYTE *txAddr;
    BYTE txData;

    // transmit the bytes one-by-one, stop if an error occurs
    for (count = 0; count < size && !txStatus; count++) {
        txAddr = txBuf + count;
        txData = *txAddr;
        txStatus = HAL_SPI_Transmit(&hspi2, &txData, 1, 1000);
    }

    // uart_printf("cnt: %d\n", count);

    return txStatus;
}

/**
 * Receives a byte of data from the SPI
 * @param rxData - a pointer to the receive byte
 * @return the receive status from the SPI
 */
static BYTE spi_rx(BYTE *rxData) {
    BYTE txData = 0xFF;
    return HAL_SPI_TransmitReceive(&hspi2, &txData, rxData, 1, 1000);
}

/**
 * Receives multiple bytes of data from the SPI
 * @param rxBuf - a pointer to the receive buffer
 * @param size - the number of bytes to receive
 * @return the receive status from the SPI
 */
static BYTE spi_rx_multi(BYTE *rxBuf, int size) {

    int count;
    BYTE rxStatus = 0;
    BYTE txData = 0xFF;

    // read the bytes one-by-one, stop if an error occurs
    for (count = 0; count < size && !rxStatus; count++) {
        rxStatus = HAL_SPI_TransmitReceive(&hspi2, &txData, rxBuf + count, 1, 1000);
    }

    return rxStatus;
}

/**
 * Selects the SD card and waits until it is ready to receive a command
 * @return 1 if the card is ready, 0 if the ready check times out
 */
static int spi_select(void) {

    // set CS low
    SD_CS_LOW();

    // wait until card not busy
    return spi_wait_ready();
}

/**
 * Deselects the SD card
 */
static void spi_deselect(void) {

    // set CS high
    SD_CS_HIGH();

    // dummy clock to force D0 hi-z for multi-slave SPI
    BYTE txData = 0xFF;
    spi_tx(&txData);

}

/**
 * Waits until the SPI MISO line is ready
 * @return 1 if the card is ready, 0 if the ready check times out
 */
static int spi_wait_ready(void) {
    BYTE rxData;
    uint32_t timeout = DEFAULT_TIMEOUT;
    do {
        spi_rx(&rxData);
    } while (rxData != 0xFF && timeout--);
    return timeout;
}

/**
 * Receives a block of specifiable size from the SPI
 * @param rxBuf - the buffer to store the received block
 * @param size - the size of the block in units of bytes
 * @return the receive status from the SPI
 */
static int spi_receive_block(BYTE * rxBuf, uint32_t size) {

    BYTE rxData;

    // wait until we receive the start token or timeout
    uint32_t timeout = DEFAULT_TIMEOUT;
    do {
        spi_rx(&rxData);
    } while (rxData != R_W_START_TOKEN && timeout--);

    // if we timeout, return error
    if (!timeout) return 0;

    // read bytes
    spi_rx_multi(rxBuf, size);

    // read block CRC, ignore it
    spi_rx_multi(NULL, BLOCK_CRC_SIZE);

    // return OK
    return 1;

}

/**
 * Receives a block of specifiable size from the SPI
 * @param rxBuf - the buffer to store the received block
 * @param size - the size of the block in units of bytes
 * @return the receive status from the SPI
 */
static int spi_transmit_block(const BYTE * txBuf, uint32_t size) {

    BYTE rxData;
    BYTE txData;

    // transmit the start token
    txData = R_W_START_TOKEN;
    spi_tx(&txData);

    // transmit the block
    spi_tx_multi((BYTE *) txBuf, size);

    // receive data until we receive a valid data response token (in form 0bxxx0xxx1)
    uint32_t timeout = DEFAULT_TIMEOUT;
    do {
        spi_rx(&rxData);
    } while (!((rxData & 0b1 || rxData & (0b1 << 4))) && timeout--);

    // if we timeout, return error
    if (!timeout) return 0;

    // extract status (bits 1 thru 3)
    BYTE data_response_status = (rxData >> 1) & 0b111;

    // check for errors (0b010 is the SD card's data accepted response)
    if (data_response_status != 0b010) return 0;

    // return OK
    return 1;

}