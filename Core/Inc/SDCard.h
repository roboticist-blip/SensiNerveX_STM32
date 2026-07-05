/**
 * @file    SDCard.h
 * @brief   SDIO block-device driver for the on-board microSD socket.
 *
 * Target hardware: WeAct Studio STM32F405RGT6 core board.
 * Peripheral:       SDIO (STM32F405 has a single SDIO controller, not SDMMC).
 *
 * Pin map (fixed by silicon — cannot be remapped on F405):
 *
 *   Signal      MCU Pin   AF        Notes
 *   ----------  --------  --------  --------------------------------
 *   SDIO_D0     PC8       AF12      Data line 0 (also used for 1-bit mode)
 *   SDIO_D1     PC9       AF12      Data line 1 (4-bit mode only)
 *   SDIO_D2     PC10      AF12      Data line 2 (4-bit mode only)
 *   SDIO_D3     PC11      AF12      Data line 3 (4-bit mode only)
 *   SDIO_CK     PC12      AF12      Clock
 *   SDIO_CMD    PD2       AF12      Command
 *   SD_DETECT   see Config.h        Optional card-detect GPIO (active low)
 *
 * All data/clock/cmd lines need external pull-ups (10 kΩ) to 3.3 V; most
 * microSD breakout boards and the WeAct socket already provide these.
 *
 * This module is a thin, testable wrapper around HAL_SD. It intentionally
 * exposes a small, storage-agnostic block API (init/read/write/status) so
 * that the FatFs glue layer (diskio.c) — and therefore every module above
 * it — never has to know about SDIO/HAL_SD directly. That indirection is
 * what lets the storage backend be swapped later (e.g. SPI-mode SD, external
 * NOR flash) without touching FatFs or the application layer.
 *
 * @author  SensiNerveX Project
 * @version 1.0.0
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Config.h"

/* =========================================================================
 * STATUS CODES
 * ========================================================================= */

typedef enum {
    SDCARD_OK              = 0,
    SDCARD_ERR_INIT        = 1,   /**< HAL_SD_Init / InitCard failed        */
    SDCARD_ERR_TIMEOUT     = 2,   /**< Card did not reach TRANSFER state    */
    SDCARD_ERR_READ        = 3,
    SDCARD_ERR_WRITE       = 4,
    SDCARD_ERR_NOT_PRESENT = 5,   /**< Card-detect pin reports no card      */
    SDCARD_ERR_WIDEBUS     = 6,   /**< 4-bit bus width negotiation failed   */
} SDCard_Status_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

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
