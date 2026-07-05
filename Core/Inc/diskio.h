/**
 * @file    diskio.h
 * @brief   FatFs low-level disk I/O interface — implemented in diskio.c
 *          on top of SDCard.h. This is the standard interface FatFs
 *          expects from any block device; ff.c calls these functions
 *          and never touches SDCard.h/HAL_SD directly.
 *
 * NOTE: This header intentionally mirrors the interface the FatFs core
 * (ff.c) #includes as "diskio.h". If you add the official FatFs source
 * (see ffconf.h header comment) and it ships its own diskio.h template,
 * keep *this* version — it's already wired to SDCard.c.
 */

#ifndef DISKIO_H
#define DISKIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Status of Disk Functions */
typedef uint8_t DSTATUS;

/* Disk Status Bits (DSTATUS) */
#define STA_NOINIT      0x01   /* Drive not initialized */
#define STA_NODISK      0x02   /* No medium in the drive */
#define STA_PROTECT     0x04   /* Write protected */

/* Results of Disk Functions */
typedef enum {
    RES_OK = 0,     /* 0: Successful */
    RES_ERROR,      /* 1: R/W Error */
    RES_WRPRT,      /* 2: Write Protected */
    RES_NOTRDY,     /* 3: Not Ready */
    RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;

/* Generic command codes for disk_ioctl() */
#define CTRL_SYNC           0  /* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT    1  /* Get media size */
#define GET_SECTOR_SIZE     2  /* Get sector size */
#define GET_BLOCK_SIZE      3  /* Get erase block size */
#define CTRL_TRIM           4  /* Inform device that the data on the block of sectors is no longer used */

DSTATUS disk_initialize(uint8_t pdrv);
DSTATUS disk_status(uint8_t pdrv);
DRESULT disk_read(uint8_t pdrv, uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT disk_write(uint8_t pdrv, const uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT disk_ioctl(uint8_t pdrv, uint8_t cmd, void *buff);

/** Required by FatFs when FF_FS_NORTC == 0; returns a packed FAT timestamp.
 *  This board has no RTC wired, so we return a fixed epoch (see Config.h /
 *  ffconf.h FF_NORTC_* — set this to the firmware build date if you care
 *  about accurate file timestamps; it doesn't affect log content). */
uint32_t get_fattime(void);

#ifdef __cplusplus
}
#endif

#endif /* DISKIO_H */
