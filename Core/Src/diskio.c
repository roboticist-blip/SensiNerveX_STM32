/**
 * @file    diskio.c
 * @brief   FatFs <-> SDCard.c glue layer.
 *
 * This is the only file that bridges the third-party FatFs core (ff.c,
 * added separately — see ffconf.h) and our SDIO driver. Keeping the
 * translation in one small file means FatFs, SDCard.c, and everything
 * above them (DataLogger.c) can each be tested/replaced independently —
 * the scalability goal called out in Config.h's module-framework section.
 *
 * Single physical drive (pdrv == 0 always): the microSD card.
 */

#include "diskio.h"
#include "SDCard.h"
#include "Config.h"

#define SD_DRIVE_NUM   0U

static volatile DSTATUS s_status = STA_NOINIT;

DSTATUS disk_initialize(uint8_t pdrv)
{
    if (pdrv != SD_DRIVE_NUM) {
        return STA_NOINIT;
    }

    if (!SDCard_IsPresent()) {
        s_status = STA_NOINIT | STA_NODISK;
        return s_status;
    }

    SDCard_Status_t ret = SDCard_Init();
    s_status = (ret == SDCARD_OK) ? 0U : STA_NOINIT;
    return s_status;
}

DSTATUS disk_status(uint8_t pdrv)
{
    if (pdrv != SD_DRIVE_NUM) {
        return STA_NOINIT;
    }
    if (!SDCard_IsPresent()) {
        s_status |= STA_NODISK;
    }
    return s_status;
}

DRESULT disk_read(uint8_t pdrv, uint8_t *buff, uint32_t sector, uint32_t count)
{
    if (pdrv != SD_DRIVE_NUM) {
        return RES_PARERR;
    }
    if (s_status & STA_NOINIT) {
        return RES_NOTRDY;
    }
    return (SDCard_ReadBlocks(buff, sector, count) == SDCARD_OK) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(uint8_t pdrv, const uint8_t *buff, uint32_t sector, uint32_t count)
{
    if (pdrv != SD_DRIVE_NUM) {
        return RES_PARERR;
    }
    if (s_status & STA_NOINIT) {
        return RES_NOTRDY;
    }
    /* Read-only enforcement (FF_FS_READONLY) is handled inside ff.c itself —
     * it never calls disk_write() at all in that configuration, so no
     * duplicate check is needed here. */
    return (SDCard_WriteBlocks(buff, sector, count) == SDCARD_OK) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(uint8_t pdrv, uint8_t cmd, void *buff)
{
    if (pdrv != SD_DRIVE_NUM) {
        return RES_PARERR;
    }
    if (s_status & STA_NOINIT) {
        return RES_NOTRDY;
    }

    switch (cmd) {
        case CTRL_SYNC:
            /* SDCard_WriteBlocks() is already blocking-until-ready, so
             * there is no write cache to flush here. */
            return RES_OK;

        case GET_SECTOR_COUNT:
            *(uint32_t *)buff = SDCard_GetBlockCount();
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(uint16_t *)buff = 512U;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(uint32_t *)buff = 1U;   /* erase-block size in sectors — unknown, report 1 */
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

uint32_t get_fattime(void)
{
    /* No RTC on this board. Fixed timestamp: 2026-01-01 00:00:00.
     * FAT time packing: bit31:25=year-1980, 24:21=month, 20:16=day,
     *                    15:11=hour, 10:5=minute, 4:0=second/2 */
    return ((uint32_t)(2026 - 1980) << 25) | (1UL << 21) | (1UL << 16);
}
