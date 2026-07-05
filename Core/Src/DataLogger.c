/**
 * @file    DataLogger.c
 * @brief   DataLogger implementation. See DataLogger.h for the two-file
 *          design rationale (async-queued LOG file vs synchronous DATA
 *          file), and AppModule.h for the AppModule_t wiring.
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
static uint8_t             s_mounted        = 0U;   /* one filesystem, shared by both files */

static FIL                 s_log_file;
static uint8_t             s_log_file_open  = 0U;
static uint32_t            s_last_flush_ms  = 0U;

static FIL                 s_data_file;
static uint8_t             s_data_file_open = 0U;

/* Single-producer (main loop), single-consumer (Tick, also main loop) ring
 * buffer for the LOG file only. Because both sides run on the same thread
 * (bare-metal, no RTOS), no locking is needed — the "concurrency" this
 * solves is purely about bounding blocking-call latency, not about race
 * conditions. The DATA file deliberately has no queue — see
 * DataLogger_LogRawWindow() below for why. */
static DLog_Row_t          s_queue[SD_LOG_QUEUE_DEPTH];
static volatile uint8_t    s_q_head = 0U;   /* next slot to write (producer) */
static volatile uint8_t    s_q_tail = 0U;   /* next slot to read  (consumer) */
static volatile uint8_t    s_q_count = 0U;

static DataLogger_Stats_t  s_stats = {0};

static void DataLogger_ModuleInit(void);
static void DataLogger_ModuleTick(void);
static uint8_t DataLogger_Mount(void);
static uint8_t DataLogger_OpenIndexedFile(const char *dir, const char *prefix, FIL *file);
static uint8_t DataLogger_Enqueue(const char *text, int len);
static void DataLogger_WriteLogHeader(void);
static void DataLogger_WriteDataHeader(void);

AppModule_t g_datalogger_module = {
    .name = "DataLogger",
    .init = DataLogger_ModuleInit,
    .tick = DataLogger_ModuleTick,
};

/* =========================================================================
 * MOUNT
 * ========================================================================= */

static uint8_t DataLogger_Mount(void)
{
    FRESULT fr = f_mount(&s_fatfs, "", 1);   /* 1 = mount now, not lazily */

    if (fr == FR_NO_FILESYSTEM) {
        /* Card is physically fine but has no FAT boot sector FatFs
         * recognizes — either genuinely blank, or formatted as something
         * this FatFs release can't parse. _USE_MKFS is enabled in the
         * vendored Middlewares/Third_Party/FatFs/src/ffconf.h for exactly
         * this case (that file — NOT Core/Inc/ffconf.h — is the one that
         * actually governs ff.c: ff.h's #include "ffconf.h" resolves
         * same-directory first, so the vendored copy always wins).
         *
         * NOTE: this vendored FatFs release predates the MKFS_PARM-struct
         * f_mkfs() API — it uses the older 5-argument signature
         * (path, opt, au, work, len). */
        LOG_ERR("DataLogger: no FAT filesystem found on card — formatting");

        /* Work buffer must be >= the FatFs sector size — this vendored
         * config's _MAX_SS is 512 (see ffconf.h). */
        static uint8_t s_mkfs_work[512];

        /* FM_FAT (no FM_SFD): standard MBR-partitioned layout rather than
         * a super-floppy boot sector — the far more commonly tested
         * combination in this old FatFs release. */
        FRESULT mkfs_fr = f_mkfs("", FM_FAT, 0, s_mkfs_work, sizeof(s_mkfs_work));
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

    /* Ensure both directories exist; FR_EXIST just means already there. */
    fr = f_mkdir(SD_LOG_DIR);
    if (fr != FR_OK && fr != FR_EXIST) {
        LOG_ERR("DataLogger: f_mkdir(%s) failed (FRESULT=%d)", SD_LOG_DIR, (int)fr);
        return 0U;
    }
    fr = f_mkdir(SD_DATA_DIR);
    if (fr != FR_OK && fr != FR_EXIST) {
        LOG_ERR("DataLogger: f_mkdir(%s) failed (FRESULT=%d)", SD_DATA_DIR, (int)fr);
        return 0U;
    }

    s_mounted = 1U;
    return 1U;
}

/* =========================================================================
 * SESSION FILES
 * ========================================================================= */

/**
 * @brief  Find the next unused PREFIXNNNNN.CSV name in dir and open it
 *         for writing (FA_CREATE_NEW — never overwrites an existing
 *         session). Header rows are written by the caller afterward,
 *         since LOG and DATA files need different headers.
 */
static uint8_t DataLogger_OpenIndexedFile(const char *dir, const char *prefix, FIL *file)
{
    char path[48];
    FILINFO fno;

    for (uint32_t idx = 1U; idx <= 99999U; idx++) {
        /* NOTE: _USE_LFN is 0 in the vendored FatFs config, so filenames
         * are limited to 8.3 short names — max 8 chars before the dot.
         * "PREFIX" + 5 digits must be <= 8 chars (no separator budget);
         * enforced at compile time in Config.h for both prefixes. */
        snprintf(path, sizeof(path), "%s/%s%05lu.CSV", dir, prefix, idx);
        FRESULT fr = f_stat(path, &fno);
        if (fr == FR_NO_FILE) {
            fr = f_open(file, path, FA_WRITE | FA_CREATE_NEW);
            if (fr != FR_OK) {
                LOG_ERR("DataLogger: f_open(%s) failed (FRESULT=%d)", path, (int)fr);
                return 0U;
            }
            LOG_INF("DataLogger: session file %s opened", path);
            return 1U;
        }
    }

    LOG_ERR("DataLogger: no free session filename in %s (99999 exhausted)", dir);
    return 0U;
}

static void DataLogger_WriteLogHeader(void)
{
    UINT written = 0U;

    /* Metadata preamble — '#' prefix so any CSV reader (pandas, Excel,
     * etc.) treats these as comments to skip, while still capturing the
     * hyperparameters a paper's methods section needs, right next to the
     * data they describe. Pulled from Config.h compile-time constants,
     * so this can never drift out of sync with the firmware that produced
     * the accompanying rows. */
    char meta[SD_LOG_MAX_ROW_LEN];
    int n;

    n = snprintf(meta, sizeof(meta), "# FedVibroSense STM32F405 run metadata\r\n");
    f_write(&s_log_file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# nn_input=%u nn_hidden=%u nn_output=%u\r\n",
                 (unsigned)NN_INPUT_SIZE, (unsigned)NN_HIDDEN_SIZE, (unsigned)NN_OUTPUT_SIZE);
    f_write(&s_log_file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# feature_vector_size=%u sample_rate_hz=%u feature_window_s=%u\r\n",
                 (unsigned)FEATURE_VECTOR_SIZE, (unsigned)IMU_SAMPLE_RATE_HZ,
                 (unsigned)FEATURE_WINDOW_SECONDS);
    f_write(&s_log_file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# nn_learning_rate=%.6f fl_local_epochs=%u fl_standalone=%u\r\n",
                 (double)NN_LEARNING_RATE, (unsigned)FL_LOCAL_EPOCHS, (unsigned)FL_STANDALONE_MODE);
    f_write(&s_log_file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta),
                 "type,seq_or_round,millis,pitch_or_loss,roll_or_samples,"
                 "pred_or_srv,p_normal,p_imbalance,p_looseness,label\r\n");
    f_write(&s_log_file, meta, (UINT)n, &written);

    f_sync(&s_log_file);
}

static void DataLogger_WriteDataHeader(void)
{
    /* Fixed prefix is only written once at file-open, so a slightly
     * larger transient stack buffer here (unlike the per-row hot path
     * below) costs nothing — no RAM budget concern for something that
     * doesn't persist. The per-feature loop still uses a small buffer,
     * since that's the part that would matter if ever changed to run
     * more than once per session. */
    char header[64];
    char field[16];
    UINT written = 0U;
    int n;

    n = snprintf(header, sizeof(header), "seq,millis,label,pred,p_normal,p_imbalance,p_looseness");
    f_write(&s_data_file, header, (UINT)n, &written);

    for (uint16_t i = 0U; i < FEATURE_VECTOR_SIZE; i++) {
        n = snprintf(field, sizeof(field), ",f%u", (unsigned)i);
        f_write(&s_data_file, field, (UINT)n, &written);
    }

    f_write(&s_data_file, "\r\n", 2U, &written);
    f_sync(&s_data_file);
}

/* =========================================================================
 * LOG-FILE QUEUE (async)
 * ========================================================================= */

static uint8_t DataLogger_Enqueue(const char *text, int len)
{
    if (s_q_count >= SD_LOG_QUEUE_DEPTH) {
        s_stats.log_rows_dropped++;
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
 * PUBLIC API — LOG FILE (async, queued)
 * ========================================================================= */

DataLogger_Status_t DataLogger_LogWindow(uint32_t seq, uint32_t millis,
                                          float pitch, float roll,
                                          uint8_t pred,
                                          float p_normal, float p_imbalance, float p_looseness,
                                          uint8_t label)
{
    if (!s_mounted || !s_log_file_open) return DLOG_ERR_MOUNT;

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
    if (!s_mounted || !s_log_file_open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "FL,%lu,%lu,%.6f,%lu,%u,,,,\r\n",
        (unsigned long)round, (unsigned long)millis,
        (double)mean_loss, (unsigned long)total_samples, server_connected);

    return DataLogger_Enqueue(row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

/* =========================================================================
 * PUBLIC API — DATA FILE (synchronous, unqueued)
 * ========================================================================= */

/**
 * Why this one is synchronous while LogWindow()/LogFLRound() aren't:
 *
 * A full row here is seq+millis+label+pred+3 probabilities+500 floats —
 * as text, roughly 4 KB. The LOG file's queue design (fixed-size RAM
 * slots, SD_LOG_QUEUE_DEPTH deep) doesn't scale to that: even a queue
 * depth of just 2 at 4 KB/slot is 8 KB, and this MCU's SRAM1 is already
 * near its budget (see README "Memory Budget"). The obvious-looking
 * workaround — park that queue in CCM RAM, which is otherwise entirely
 * unused — doesn't actually work here: HAL_SD's DMA engine cannot address
 * CCM on the STM32F4, so a write source buffer living in CCM would fail
 * or silently corrupt, not just cost RAM.
 *
 * Given that, this writes directly and blocks: seq/millis/label/pred/
 * probabilities as one small field, then each of the 500 feature values
 * one at a time, all through a single reused ~16-byte stack buffer — no
 * large buffer anywhere, ever. This is deliberately NOT built via this
 * old FatFs release's f_printf(): its %f is unimplemented (it silently
 * treats unrecognized format characters as literal pass-through text —
 * i.e. "%.4f" would print the literal character 'f' into the file, not
 * the float value). snprintf() from the toolchain's own libc is used
 * instead, which does support floats correctly.
 *
 * Blocking here is an acceptable tradeoff because this is only called
 * once per feature window (1 Hz in this project), not at IMU sample
 * rate (100 Hz) — a several-millisecond SD write once a second doesn't
 * meaningfully perturb the 10 ms IMU sample timing the way it would if
 * called from the hot path.
 */
DataLogger_Status_t DataLogger_LogRawWindow(uint32_t seq, uint32_t millis,
                                             uint8_t label, uint8_t pred,
                                             float p_normal, float p_imbalance, float p_looseness,
                                             const float *feature_vec, uint16_t feature_len)
{
    if (!s_mounted || !s_data_file_open) return DLOG_ERR_MOUNT;
    if (feature_vec == NULL) return DLOG_ERR_WRITE;

    /* 48 bytes: safely covers %f's full decimal expansion for any finite
     * float (worst case ~40 chars for FLT_MAX) plus a comma and null —
     * not just the ~8 chars a normal [0,1] softmax probability needs.
     * Cheap: this is stack-only and reused across every field/feature
     * write, never persisted. */
    char field[48];
    UINT written = 0U;
    int n;
    FRESULT fr;

    /* Written as separate small fields rather than one combined snprintf
     * — the header write above already hit a real truncation bug from
     * combining fields into a too-small buffer; splitting removes any
     * need to reason about worst-case combined width (e.g. a NaN or
     * out-of-range probability expanding %f far past the ~8 chars a
     * normal softmax output takes). */
    n = snprintf(field, sizeof(field), "%lu,", (unsigned long)seq);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    n = snprintf(field, sizeof(field), "%lu,", (unsigned long)millis);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    n = snprintf(field, sizeof(field), "%u,%u,", label, pred);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    n = snprintf(field, sizeof(field), "%.6f,", (double)p_normal);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    n = snprintf(field, sizeof(field), "%.6f,", (double)p_imbalance);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    n = snprintf(field, sizeof(field), "%.6f", (double)p_looseness);
    fr = f_write(&s_data_file, field, (UINT)n, &written);
    if (fr != FR_OK) goto write_failed;

    for (uint16_t i = 0U; i < feature_len; i++) {
        n = snprintf(field, sizeof(field), ",%.6f", (double)feature_vec[i]);
        fr = f_write(&s_data_file, field, (UINT)n, &written);
        if (fr != FR_OK) goto write_failed;
    }

    fr = f_write(&s_data_file, "\r\n", 2U, &written);
    if (fr != FR_OK) goto write_failed;

    s_stats.data_rows_written++;
    return DLOG_OK;

write_failed:
    LOG_ERR("DataLogger: DATA file write failed (FRESULT=%d)", (int)fr);
    s_stats.data_write_errors++;
    return DLOG_ERR_WRITE;
}

/* =========================================================================
 * SHARED API
 * ========================================================================= */

void DataLogger_Flush(void)
{
    if (s_log_file_open) {
        f_sync(&s_log_file);
        s_last_flush_ms = Utils_GetMillis();
    }
    if (s_data_file_open) {
        f_sync(&s_data_file);
    }
}

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out == NULL) return;
    *out = s_stats;
    out->log_mounted  = s_mounted && s_log_file_open;
    out->data_mounted = s_mounted && s_data_file_open;
}

/* =========================================================================
 * APPMODULE HOOKS
 * ========================================================================= */

static void DataLogger_ModuleInit(void)
{
    for (uint8_t attempt = 0U; attempt < SD_MOUNT_RETRY_COUNT; attempt++) {
        if (DataLogger_Mount() &&
            DataLogger_OpenIndexedFile(SD_LOG_DIR, SD_LOG_FILE_PREFIX, &s_log_file) &&
            DataLogger_OpenIndexedFile(SD_DATA_DIR, SD_DATA_FILE_PREFIX, &s_data_file)) {

            s_log_file_open  = 1U;
            s_data_file_open = 1U;
            DataLogger_WriteLogHeader();
            DataLogger_WriteDataHeader();
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
 * @brief  Drain at most one queued LOG row per call, then flush both
 *         files on a timer. Bounds worst-case per-tick SD stall from the
 *         LOG file's queue to one row's worth of I/O. The DATA file has
 *         no queue to drain here — DataLogger_LogRawWindow() already
 *         wrote (and effectively flushed, via its own blocking f_write
 *         calls) by the time this runs.
 */
static void DataLogger_ModuleTick(void)
{
    if (!s_log_file_open) return;

    if (s_q_count > 0U) {
        DLog_Row_t *slot = &s_queue[s_q_tail];
        UINT written = 0U;
        FRESULT fr = f_write(&s_log_file, slot->text, slot->len, &written);

        if (fr != FR_OK || written != slot->len) {
            LOG_ERR("DataLogger: LOG file write failed (FRESULT=%d)", (int)fr);
            s_stats.log_write_errors++;
            /* Card likely wedged — force a remount next tick rather than
             * spinning on a permanently failing write. */
            s_log_file_open  = 0U;
            s_data_file_open = 0U;
            s_mounted        = 0U;
            f_close(&s_log_file);
            f_close(&s_data_file);
            DataLogger_ModuleInit();
        } else {
            s_stats.log_rows_written++;
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

DataLogger_Status_t DataLogger_LogRawWindow(uint32_t seq, uint32_t millis,
                                             uint8_t label, uint8_t pred,
                                             float p_normal, float p_imbalance, float p_looseness,
                                             const float *feature_vec, uint16_t feature_len)
{
    (void)seq; (void)millis; (void)label; (void)pred;
    (void)p_normal; (void)p_imbalance; (void)p_looseness;
    (void)feature_vec; (void)feature_len;
    return DLOG_DISABLED;
}

void DataLogger_Flush(void) { }

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out) memset(out, 0, sizeof(*out));
}

#endif /* SD_LOGGING_ENABLE */