/**
 * @file    DataLogger.c
 * @brief   DataLogger implementation. See DataLogger.h for the design
 *          rationale (queue decoupling, AppModule_t wiring).
 */

#include "DataLogger.h"
#include "Utils.h"
#include <string.h>
#include <stdio.h>

#if (SD_LOGGING_ENABLE)

#include "ff.h"   /* FatFs core — added separately, see ffconf.h header comment */

/* =========================================================================
 * STATE
 * ========================================================================= */

typedef struct {
    char    text[SD_LOG_MAX_ROW_LEN];
    uint8_t len;
} DLog_Row_t;

static FATFS               s_fatfs;
static FIL                 s_file;
static uint8_t             s_mounted        = 0U;
static uint8_t             s_file_open      = 0U;
static uint32_t            s_last_flush_ms  = 0U;

/* Single-producer (main loop), single-consumer (Tick, also main loop) ring
 * buffer. Because both sides run on the same thread (bare-metal, no RTOS),
 * no locking is needed — the "concurrency" this solves is purely about
 * bounding blocking-call latency, not about race conditions. */
static DLog_Row_t          s_queue[SD_LOG_QUEUE_DEPTH];
static volatile uint8_t    s_q_head = 0U;   /* next slot to write (producer) */
static volatile uint8_t    s_q_tail = 0U;   /* next slot to read  (consumer) */
static volatile uint8_t    s_q_count = 0U;

static DataLogger_Stats_t  s_stats = {0};

static void DataLogger_ModuleInit(void);
static void DataLogger_ModuleTick(void);
static uint8_t DataLogger_Mount(void);
static uint8_t DataLogger_OpenSessionFile(void);
static uint8_t DataLogger_Enqueue(const char *text, int len);

AppModule_t g_datalogger_module = {
    .name = "DataLogger",
    .init = DataLogger_ModuleInit,
    .tick = DataLogger_ModuleTick,
};

/* =========================================================================
 * MOUNT / SESSION FILE
 * ========================================================================= */

static uint8_t DataLogger_Mount(void)
{
    FRESULT fr = f_mount(&s_fatfs, "", 1);   /* 1 = mount now, not lazily */

    if (fr == FR_NO_FILESYSTEM) {
        /* Card is physically fine (SDCard_PrintInfo already succeeded by
         * this point) but has no FAT boot sector FatFs recognizes — either
         * genuinely blank, or formatted as something this FatFs release
         * can't parse. _USE_MKFS is now enabled in the vendored
         * Middlewares/Third_Party/FatFs/src/ffconf.h for exactly this case
         * (that file — NOT Core/Inc/ffconf.h — is the one that actually
         * governs ff.c: ff.h's #include "ffconf.h" resolves same-directory
         * first, so the vendored copy always wins. Core/Inc/ffconf.h is
         * unused dead weight at this point; edit the vendored one instead
         * if you need to change any FatFs setting).
         *
         * NOTE: this vendored FatFs release predates the MKFS_PARM-struct
         * f_mkfs() API — it uses the older 5-argument signature
         * (path, opt, au, work, len), not (path, const MKFS_PARM*, work,
         * len). Using the wrong signature here would fail to compile
         * against this specific ff.h.
         */
        LOG_ERR("DataLogger: no FAT filesystem found on card — formatting");

        /* Work buffer must be >= the FatFs sector size — this vendored
         * config's _MAX_SS is 512 (see ffconf.h); hardcoded here rather
         * than via a macro since the macro name itself differs between
         * FatFs API generations (_MAX_SS here vs FF_MAX_SS elsewhere). */
        static uint8_t s_mkfs_work[512];

        FRESULT mkfs_fr = f_mkfs("", FM_FAT | FM_SFD, 0, s_mkfs_work, sizeof(s_mkfs_work));
        if (mkfs_fr != FR_OK) {
            LOG_ERR("DataLogger: f_mkfs failed (FRESULT=%d) — card may be faulty/write-protected",
                     (int)mkfs_fr);
            return 0U;
        }

        LOG_INF("DataLogger: format complete, remounting");
        fr = f_mount(&s_fatfs, "", 1);
    }

    if (fr != FR_OK) {
        LOG_ERR("DataLogger: f_mount failed (FRESULT=%d)", (int)fr);
        return 0U;
    }

    /* Ensure the log directory exists; FR_EXIST just means it's already there. */
    fr = f_mkdir(SD_LOG_DIR);
    if (fr != FR_OK && fr != FR_EXIST) {
        LOG_ERR("DataLogger: f_mkdir(%s) failed (FRESULT=%d)", SD_LOG_DIR, (int)fr);
        return 0U;
    }

    s_mounted = 1U;
    return 1U;
}

/**
 * @brief  Find the next unused SNX_NNNNN.CSV name in SD_LOG_DIR so a fresh
 *         boot never overwrites a previous session's log, and open it with
 *         a one-line CSV header.
 */
static uint8_t DataLogger_OpenSessionFile(void)
{
    char path[48];
    FILINFO fno;

    for (uint32_t idx = 1U; idx <= 99999U; idx++) {
        snprintf(path, sizeof(path), "%s/%s_%05lu.CSV", SD_LOG_DIR, SD_LOG_FILE_PREFIX, idx);
        FRESULT fr = f_stat(path, &fno);
        if (fr == FR_NO_FILE) {
            FRESULT fr = f_open(&s_file, path, FA_WRITE | FA_CREATE_NEW);
            if (fr != FR_OK) {
                LOG_ERR("DataLogger: f_open(%s) failed (FRESULT=%d)", path, (int)fr);
                return 0U;
            }
            LOG_INF("DataLogger: session file %s opened", path);

            static const char header[] =
                "type,seq_or_round,millis,pitch_or_loss,roll_or_samples,"
                "pred_or_srv,p_normal,p_imbalance,p_looseness,label\r\n";
            UINT written = 0U;
            f_write(&s_file, header, (UINT)(sizeof(header) - 1U), &written);
            f_sync(&s_file);

            s_file_open = 1U;
            return 1U;
        }
    }

    LOG_ERR("DataLogger: no free session filename in %s (99999 exhausted)", SD_LOG_DIR);
    return 0U;
}

/* =========================================================================
 * QUEUE
 * ========================================================================= */

static uint8_t DataLogger_Enqueue(const char *text, int len)
{
    if (s_q_count >= SD_LOG_QUEUE_DEPTH) {
        s_stats.rows_dropped++;
        return 0U;
    }
    if (len < 0) len = 0;
    if ((uint32_t)len >= SD_LOG_MAX_ROW_LEN) len = (int)SD_LOG_MAX_ROW_LEN - 1;

    DLog_Row_t *slot = &s_queue[s_q_head];
    memcpy(slot->text, text, (size_t)len);
    slot->text[len] = '\0';
    slot->len = (uint8_t)len;

    s_q_head = (uint8_t)((s_q_head + 1U) % SD_LOG_QUEUE_DEPTH);
    s_q_count++;
    return 1U;
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

DataLogger_Status_t DataLogger_LogWindow(uint32_t seq, uint32_t millis,
                                          float pitch, float roll,
                                          uint8_t pred,
                                          float p_normal, float p_imbalance, float p_looseness,
                                          uint8_t label)
{
    if (!s_mounted || !s_file_open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "W,%lu,%lu,%.4f,%.4f,%u,%.4f,%.4f,%.4f,%u\r\n",
        (unsigned long)seq, (unsigned long)millis,
        (double)pitch, (double)roll, pred,
        (double)p_normal, (double)p_imbalance, (double)p_looseness, label);

    return DataLogger_Enqueue(row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

DataLogger_Status_t DataLogger_LogFLRound(uint32_t round, uint32_t millis,
                                           float mean_loss, uint32_t total_samples,
                                           uint8_t server_connected)
{
    if (!s_mounted || !s_file_open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "FL,%lu,%lu,%.6f,%lu,%u,,,,\r\n",
        (unsigned long)round, (unsigned long)millis,
        (double)mean_loss, (unsigned long)total_samples, server_connected);

    return DataLogger_Enqueue(row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

void DataLogger_Flush(void)
{
    if (s_file_open) {
        f_sync(&s_file);
        s_last_flush_ms = Utils_GetMillis();
    }
}

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out == NULL) return;
    *out = s_stats;
    out->mounted = s_mounted && s_file_open;
}

/* =========================================================================
 * APPMODULE HOOKS
 * ========================================================================= */

static void DataLogger_ModuleInit(void)
{
    for (uint8_t attempt = 0U; attempt < SD_MOUNT_RETRY_COUNT; attempt++) {
        if (DataLogger_Mount() && DataLogger_OpenSessionFile()) {
            s_last_flush_ms = Utils_GetMillis();
            return;
        }
        LOG_ERR("DataLogger: mount/open attempt %u/%u failed, retrying...",
                attempt + 1U, SD_MOUNT_RETRY_COUNT);
        s_stats.remount_count++;
        HAL_Delay(50U);
    }
    LOG_ERR("DataLogger: giving up after %u attempts — logging disabled for this boot",
             SD_MOUNT_RETRY_COUNT);
}

/**
 * @brief  Drain at most one queued row per call, then flush on a timer.
 *         Bounds worst-case per-tick SD latency to one row write.
 */
static void DataLogger_ModuleTick(void)
{
    if (!s_file_open) return;

    if (s_q_count > 0U) {
        DLog_Row_t *slot = &s_queue[s_q_tail];
        UINT written = 0U;
        FRESULT fr = f_write(&s_file, slot->text, slot->len, &written);

        if (fr != FR_OK || written != slot->len) {
            LOG_ERR("DataLogger: f_write failed (FRESULT=%d)", (int)fr);
            s_stats.write_errors++;
            /* Card likely wedged — force a remount next tick rather than
             * spinning on a permanently failing write. */
            s_file_open = 0U;
            s_mounted   = 0U;
            f_close(&s_file);
            DataLogger_ModuleInit();
        } else {
            s_stats.rows_written++;
            s_q_tail  = (uint8_t)((s_q_tail + 1U) % SD_LOG_QUEUE_DEPTH);
            s_q_count--;
        }
    }

    uint32_t now = Utils_GetMillis();
    if ((now - s_last_flush_ms) >= SD_LOG_FLUSH_INTERVAL_MS) {
        DataLogger_Flush();
    }
}

#else /* SD_LOGGING_ENABLE == 0 — fully compiled out, zero footprint */

static void DataLogger_ModuleInit(void) { }
static void DataLogger_ModuleTick(void) { }

AppModule_t g_datalogger_module = {
    .name = "DataLogger(disabled)",
    .init = DataLogger_ModuleInit,
    .tick = DataLogger_ModuleTick,
};

DataLogger_Status_t DataLogger_LogWindow(uint32_t seq, uint32_t millis,
                                          float pitch, float roll, uint8_t pred,
                                          float p_normal, float p_imbalance, float p_looseness,
                                          uint8_t label)
{
    (void)seq; (void)millis; (void)pitch; (void)roll; (void)pred;
    (void)p_normal; (void)p_imbalance; (void)p_looseness; (void)label;
    return DLOG_DISABLED;
}

DataLogger_Status_t DataLogger_LogFLRound(uint32_t round, uint32_t millis,
                                           float mean_loss, uint32_t total_samples,
                                           uint8_t server_connected)
{
    (void)round; (void)millis; (void)mean_loss; (void)total_samples; (void)server_connected;
    return DLOG_DISABLED;
}

void DataLogger_Flush(void) { }

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out) memset(out, 0, sizeof(*out));
}

#endif /* SD_LOGGING_ENABLE */