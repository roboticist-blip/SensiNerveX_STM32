/**
 * @file    diskio.h
 * @brief   FatFs low-level disk I/O interface — implemented in diskio.c
 *          on top of SDCard.h. This is the standard interface FatFs
 *          expects from any block device; ff.c calls these functions
 *          and never touches SDCard.h/HAL_SD directly.
 *
 */

#ifndef DISKIO_H
#define DISKIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef uint8_t DSTATUS;

#define STA_NOINIT      0x01   // Drive not initialized 
#define STA_NODISK      0x02   // No medium in the drive
#define STA_PROTECT     0x04   // Write protected

typedef enum {
    RES_OK = 0,     // 0: Successful
    RES_ERROR,      // 1: R/W Error 
    RES_WRPRT,      // 2: Write Protected 
    RES_NOTRDY,     // 3: Not Ready 
    RES_PARERR      // 4: Invalid Parameter 
} DRESULT;

#define CTRL_SYNC           0  
#define GET_SECTOR_COUNT    1  
#define GET_SECTOR_SIZE     2  
#define GET_BLOCK_SIZE      3  
#define CTRL_TRIM           4  

DSTATUS disk_initialize(uint8_t pdrv);
DSTATUS disk_status(uint8_t pdrv);
DRESULT disk_read(uint8_t pdrv, uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT disk_write(uint8_t pdrv, const uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT disk_ioctl(uint8_t pdrv, uint8_t cmd, void *buff);

uint32_t get_fattime(void);

#ifdef __cplusplus
}
#endif

#endif /* DISKIO_H */
