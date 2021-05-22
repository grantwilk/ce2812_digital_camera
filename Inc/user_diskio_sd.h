/**
  * @file user_diskio_sd.h
  * @author Grant Wilk
  * @created 2/22/2020
  * @modified 2/22/2020
  * @desc 
  */

# include "ff_gen_drv.h"

#ifndef CE2812_WK09_LAB_SD_USER_DISKIO_SD_H
#define CE2812_WK09_LAB_SD_USER_DISKIO_SD_H
#endif

/*
 * SD Chip Select Pin and Macros
 */
# define SD_CS_GPIO_PORT    GPIOB           /* The GPIO port the SD CS pin is on */
# define SD_CS_GPIO_PIN     GPIO_PIN_12     /* The pin number of the SD CS pin*/

# define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN, GPIO_PIN_SET)    /* Sets the SD CS pin */
# define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN, GPIO_PIN_RESET)  /* Clears the SD CS pin */

/*
 * SD Card Commands
 */

# define CMD0       0       /* GO_IDLE_STATE */
# define CMD1       1       /* SEND_OP_COND (MMC) */
# define ACMD41     41      /* SEND_OP_COND (SDC) */
# define CMD8       8       /* SEND_IF_COND */
# define CMD9       9       /* SEND_CSD */
# define CMD10      10      /* SEND_CID */
# define CMD12      12      /* STOP_TRANSMISSION */
# define ACMD13     13      /* SD_STATUS (SDC) */
# define CMD16      16      /* SET_BLOCKLEN */
# define CMD17      17      /* READ_SINGLE_BLOCK */
# define CMD18      18      /* READ_MULTIPLE_BLOCK */
# define CMD23      23      /* SET_BLOCK_COUNT (MMC) */
# define ACMD23     23      /* SET_WR_BLK_ERASE_COUNT (SDC) */
# define CMD24      24      /* WRITE_BLOCK */
# define CMD25      25      /* WRITE_MULTIPLE_BLOCK */
# define CMD32      32      /* ERASE_ER_BLK_START */
# define CMD33      33      /* ERASE_ER_BLK_END */
# define CMD38      38      /* ERASE */
# define CMD55      55      /* APP_CMD */
# define CMD58      58      /* READ_OCR */

/*
 * SD Card Argument Values
 */

# define NULL_ARG   0x00    /* null argument */

/*
 * SD Card Cyclical Response Check Values
 */

# define NULL_CRC   0x00    /* Default CRC */
# define CMD0_CRC   0x94    /* CRC for CMD0 */
# define CMD8_CRC   0x86    /* CRC for CMD8 */

/*
 * SD Command Response Fields
 */

# define CMD8_CMD_VERSION       (0xFULL << 28)      /* command version mask */
# define CMD8_VOLTAGE_ACCEPT    (0xFULL << 8)       /* voltage accepted mask */
# define CMD8_CHECK_PATTERN     (0xFFULL << 0)      /* check pattern mask */

# define CMD58_2_7V_SUPPORT     (0b01 << 15)    /* card supports 2.7 volts flag */
# define CMD58_3_0V_SUPPORT     (0b11 << 17)    /* card supports 3.0 volts flag */
# define CMD58_3_3V_SUPPORT     (0b11 << 20)    /* card supports 3.3 volts flag */
# define CMD58_3_6V_SUPPORT     (0b01 << 23)    /* card supports 3.6 volts flag */
# define CMD58_CCS              (1 << 30)       /* card capacity status */

/*
 * SD Status Response Flags
 */

# define NULL_STATUS                0x00        /* null status */
# define STATUS_IDLE                (1 << 0)    /* in idle state */
# define STATUS_ERASE_RESET         (1 << 1)    /* erase reset */
# define STATUS_ILLEGAL_CMD         (1 << 2)    /* illegal command */
# define STATUS_CRC_ERROR           (1 << 3)    /* CRC error */
# define STATUS_ERASE_SEQ_ERROR     (1 << 4)    /* erase sequence error */
# define STATUS_ADDRESS_ERROR       (1 << 5)    /* address error */
# define STATUS_PARAMETER_ERROR     (1 << 6)    /* parameter error */

/*
 * SD Read/Write Values
 */

# define R_W_START_TOKEN    0xFE    /* read/write start token */
# define BLOCK_SIZE         512     /* 512 bytes */
# define BLOCK_CRC_SIZE     2       /* 2 bytes */

/**
 * SD Card Type Enum
 */
 typedef enum {
   CT_SDv1,
   CT_SDv2
 } SD_CardType;

 /**
  * SD Card Capacity Enum
  */
  typedef enum {
      CC_STANDARD,
      CC_SDHC_SDXC
  } SD_CardCapacity;

/**
 * SD Command Response Structure
 */
typedef struct {
    BYTE status;
    uint32_t data;
} sd_response_t;

/*
 * DISK I/O Functions
 */

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_SD_initialize(BYTE pdrv);

/**
  * Gets the disk status
  * @param pdrv - physical drive number
  * @retval operation status
  */
DSTATUS USER_SD_status(BYTE pdrv);

/**
 * Reads sectors from the SD card
 * @param pdrv - physical drive number
 * @param buff - pointer to the buffer for storing read data
 * @param sector - sector address in LBA (logical block address)
 * @param count - number of sectors to read
 * @return operation result
 */
DRESULT USER_SD_read(BYTE pdrv, BYTE *buff, uint32_t sector, uint32_t count);

/**
  * Writes sectors to the SD card
  * @param pdrv - physical drive number
  * @param buff - pointer to the buffer containing blocks to be transmitted
  * @param sector - sector address formatted in LBA (logical block address)
  * @param count - number of sectors to write
  * @return operation result
  */
DRESULT USER_SD_write(BYTE pdrv, const BYTE *buff, uint32_t sector, uint32_t count);

/**
 * Miscellaneous drive controls other than data read/write
 * @param pdrv - physical drive number
 * @param cmd - control command code
 * @param buff - pointer to control data
 * @return operation result
 */
DRESULT USER_SD_ioctrl(BYTE pdrv, BYTE cmd, void * buff);

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
static sd_response_t sd_send_command(BYTE cmd, uint32_t arg, BYTE crc);

/**
 * Receives the full 40-bit (5-byte) response from a command
 * @param cmd - command index
 * @param rxBuf - a pointer to the 40-bit (5-byte) receive buffer
 * @return the SD card's response
 */
static sd_response_t sd_get_response(BYTE cmd);

/**
 * Reads a single block from the SD card
 * @param rxBuf - pointer to the buffer where the received block should be stored
 * @param block_index - the address of the block to read
 * @return the number of blocks read
 */
static int sd_read_block(BYTE *rxBuf, uint32_t block_index);

/**
 * Reads multiple blocks from the SD card
 * @param rxBuf - pointer to the buffer where the received blocks should be stored
 * @param block_index - the starting address of the blocks to read
 * @param count - the number of blocks to read
 * @return the number of blocks read
 */
static int sd_read_multi_block(BYTE * rxBuf, uint32_t block_index, uint32_t count);

/**
 * Writes a single block to the SD card
 * @param txBuf - pointer to the buffer containing the block to be transmitted
 * @param block_index - the address of the block to write to
 * @return the number of blocks written
 */
static int sd_write_block(const BYTE * txBuf, uint32_t block_index);

/**
 * Writes multiple blocks to the SD card
 * @param txBuf - pointer to the buffer containing the blocks to be transmitted
 * @param block_index - the starting address of the blocks to write to
 * @param count - the number of blocks to write
 * @return the number of blocks written
 */
static int sd_write_multi_block(const BYTE * txBuf, uint32_t block_index, uint32_t count);

/*
 * SPI RX/TX Functions
 */

/**
 * Transmits a byte of data through the SPI
 * @param txData - a pointer to the byte to transmit
 * @return the transmit status from the SPI
 */
static BYTE spi_tx(BYTE *txData);

/**
 * Transmits multiple bytes of data from the SPI
 * @param txBuf - a pointer to the transmit buffer
 * @param size - the number of bytes to transmit
 * @return the transmit status from the SPI
 */
static BYTE spi_tx_multi(BYTE *txBuf, int size);

/**
 * Receives a byte of data from the SPI
 * @param rxData - a pointer to the receive byte
 * @return the receive status from the SPI
 */
static BYTE spi_rx(BYTE *rxData);

/**
 * Receives multiple bytes of data from the SPI
 * @param rxBuf - a pointer to the receive buffer
 * @param size - the number of bytes to receive
 * @return the receive status from the SPI
 */
static BYTE spi_rx_multi(BYTE *rxBuf, int size);

/**
 * Selects the SD card
 * @return 1 if the card is ready, 0 if the ready check times out
 */
static int spi_select(void);

/**
 * Deselects the SD card
 */
static void spi_deselect(void);

/**
 * Waits until the SPI MISO line is ready
 * @return 1 if the card is ready, 0 if the ready check times out
 */
static int spi_wait_ready(void);

/**
 *
 * @param rxBuf - the buffer to store the received block
 * @param size - the size of the block in units of bytes
 * @return the receive status from the SPI
 */
static int spi_receive_block(BYTE * rxBuf, uint32_t size);

/**
 * Receives a block of specifiable size from the SPI
 * @param rxBuf - the buffer to store the received block
 * @param size - the size of the block in units of bytes
 * @return the receive status from the SPI
 */
static int spi_transmit_block(const BYTE * txBuf, uint32_t size);