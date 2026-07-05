/**
 * @file    ffconf.h
 * @brief   Project-specific FatFs configuration.
 *
 * IMPORTANT — this header configures FatFs but does NOT include it.
 * You still need to add the FatFs core engine itself (ff.c, ff.h,
 * ffunicode.c) to the project — it is third-party middleware, not part
 * of the ST HAL package, so it isn't in Drivers/. Get it one of two ways:
 *
 *   1. STM32CubeMX: add the "FATFS" middleware to this project, choose
 *      "User defined" custom disk driver, and let CubeMX drop ff.c/ff.h
 *      into a Middlewares/Third_Party/FatFs/ folder. Point the Makefile/
 *      CMakeLists at that path (see the notes added to both build files).
 *   2. Download FatFs directly (ChaN's generic FAT filesystem module,
 *      BSD-style license) and drop ff.c/ff.h/ffunicode.c next to diskio.c.
 *
 * This file matches FatFs R0.14/R0.15 option names. diskio.c and
 * DataLogger.c only use the generic f_open/f_write/f_close/f_mount API,
 * so either release works without further changes.
 *
 * Sizing choices below are deliberately minimal (no LFN, no exFAT) to
 * keep flash/RAM cost low on the F405 (1 MB flash / 192 KB RAM total,
 * most of which is already committed to the NN weights + FL buffers —
 * see README "Memory Budget"). Revisit if you outgrow 8.3 filenames.
 *
 * @author  SensiNerveX Project
 * @version 1.0.0
 */

#ifndef FFCONF_H
#define FFCONF_H

#define FFCONF_DEF 63463  /* Revision ID matching FatFs R0.14b/R0.15 conventions */

/* ------------------- Function Configurations ------------------- */
#define FF_FS_READONLY     0
#define FF_FS_MINIMIZE      0
#define FF_USE_STRFUNC      1   /* f_gets/f_putc/f_puts/f_printf — used by DataLogger CSV writer */
#define FF_USE_FIND         0
#define FF_USE_MKFS         1   /* allow one-time f_mkfs() if a fresh card arrives unformatted */
#define FF_USE_FASTSEEK     0
#define FF_USE_EXPAND       0
#define FF_USE_CHMOD        0
#define FF_USE_LABEL        0
#define FF_USE_FORWARD      0

/* ------------------- Namespace / LFN Configurations ------------------- */
#define FF_CODE_PAGE        437  /* US — only ASCII filenames are used ("LOGS/SNX_00001.CSV") */
#define FF_USE_LFN          0    /* 8.3 short names only: saves ~1 KB stack per open + flash */
#define FF_MAX_LFN          255
#define FF_LFN_UNICODE      0
#define FF_LFN_BUF          255
#define FF_SFN_BUF          12
#define FF_FS_RPATH         0    /* no relative paths / chdir — keeps footprint down */

/* ------------------- Drive/Volume Configurations ------------------- */
#define FF_VOLUMES          1    /* single physical drive: the microSD card */
#define FF_STR_VOLUME_ID    0
#define FF_MULTI_PARTITION  0
#define FF_MIN_SS           512
#define FF_MAX_SS           512  /* SD blocks are always 512 B — fixed sector size avoids
                                     the extra RAM FatFs reserves for variable sector sizes */
#define FF_LBA64            0    /* cards used here are well under 2 TB */
#define FF_MIN_GPT          0x10000000
#define FF_USE_TRIM         0

/* ------------------- System Configurations ------------------- */
#define FF_FS_TINY          1    /* share the sector buffer between FS object and file objects —
                                     saves 512 B per open FIL, worthwhile given NN weight pressure */
#define FF_FS_EXFAT         0    /* exFAT adds ~5 KB flash for no benefit on cards this small */
#define FF_FS_NORTC         0
#define FF_NORTC_MON        1
#define FF_NORTC_MDAY       1
#define FF_NORTC_YEAR       2026
#define FF_FS_NOFSINFO      0
#define FF_FS_LOCK          4    /* max simultaneously open objects — DataLogger keeps at most
                                     one file open, so this just guards against programming errors */
#define FF_FS_REENTRANT     0    /* single-threaded bare-metal main loop, no RTOS — see
                                     DataLogger.h for the cooperative-queue pattern used instead */

#endif /* FFCONF_H */
