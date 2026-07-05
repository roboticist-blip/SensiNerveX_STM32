/**
 * @file    SDCard.h
 * @brief   SDIO block-device driver for the on-board microSD socket.
 *
 * Target hardware: WeAct Studio STM32F405RGT6 core board.
 * Peripheral:       SDIO (STM32F405 has a single SDIO controller, not SDMMC).
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Config.h"


typedef enum {
    SDCARD_OK              = 0,
    SDCARD_ERR_INIT        = 1,   
    SDCARD_ERR_TIMEOUT     = 2,   
    SDCARD_ERR_READ        = 3,
    SDCARD_ERR_WRITE       = 4,
    SDCARD_ERR_NOT_PRESENT = 5,   
    SDCARD_ERR_WIDEBUS     = 6,   
} SDCard_Status_t;

/**
 * @brief  Configure SDIO GPIO/clock and bring the card into data-transfer
 *         state (identification, CSD/CID readout, wide-bus negotiation).
 *         Safe to call again after SDCard_DeInit() for hot-swap recovery.
 * @return SDCARD_OK on success.
 */
SDCard_Status_t SDCard_Init(void);

/**
 * @brief  Release the SDIO peripheral and GPIO. Used before re-init on
 *         card-detect events or after repeated I/O failures.
 */
void SDCard_DeInit(void);

/**
 * @brief  Optional physical card-detect check.
 *         Returns 1 if SD_DETECT_GPIO_ENABLE is 0 (no detect pin wired —
 *         assume present) so the rest of the stack behaves identically
 *         whether or not a detect switch exists.
 */
uint8_t SDCard_IsPresent(void);

/**
 * @brief  Blocking multi-block read.
 * @param  dst          Destination buffer, must be big enough for
 *                       count * SDCARD_BLOCK_SIZE bytes, word-aligned.
 * @param  start_block  LBA of first 512-byte block.
 * @param  count        Number of blocks to read.
 */
SDCard_Status_t SDCard_ReadBlocks(uint8_t *dst, uint32_t start_block, uint32_t count);

/**
 * @brief  Blocking multi-block write.
 */
SDCard_Status_t SDCard_WriteBlocks(const uint8_t *src, uint32_t start_block, uint32_t count);

/**
 * @brief  Total capacity of the mounted card, in 512-byte blocks.
 */
uint32_t SDCard_GetBlockCount(void);

/**
 * @brief  Human-readable card type + capacity, for boot-time logging.
 */
void SDCard_PrintInfo(void);

/**
 * @brief  Poll the card's internal state machine until it reports
 *         TRANSFER (idle, ready for the next command) or timeout.
 *         FatFs calls this indirectly through diskio.c after every
 *         write to avoid returning "success" before the card has
 *         actually finished programming its NAND.
 */
SDCard_Status_t SDCard_WaitReady(uint32_t timeout_ms);

#endif /* SDCARD_H */
