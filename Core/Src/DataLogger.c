/**
 * @file    DataLogger.c
 * @brief   DataLogger implementation. See DataLogger.h for the three-file
 *          design rationale, and AppModule.h for the AppModule_t wiring.
 */

#include "DataLogger.h"
#include "Utils.h"
#include <string.h>
#include <stdio.h>

#if (SD_LOGGING_ENABLE)

#include "ff.h"   /* FatFs core — added separately, see ffconf.h header comment */

/* =========================================================================
 * GENERIC ASYNC-QUEUED FILE
 *
 * LOG and IMU are structurally identical: small text rows, produced
 * faster than the SD card can always keep up with, drained one row per
 * main-loop tick. Rather than copy-paste the queue/drain/error-recovery
 * logic a second time for IMU, both are instances of this one type,
 * differing only in queue depth (IMU fills ~100x faster than LOG) and
 * which physical file they're bound to.
 * ========================================================================= */

typedef struct {
    char    text[SD_LOG_MAX_ROW_LEN];
    uint8_t len;
} DLog_Row_t;

typedef struct {
    FIL         file;
    uint8_t     open;
    DLog_Row_t *queue;      /* points at a statically-allocated, per-instance array */
    uint8_t     depth;
    volatile uint8_t head;  /* next slot to write (producer) */
    volatile uint8_t tail;  /* next slot to read  (consumer) */
    volatile uint8_t count;
    uint32_t    rows_written;
    uint32_t    rows_dropped;
    uint32_t    write_errors;
    const char *dir;
    const char *prefix;
} QueuedFile_t;

static DLog_Row_t s_log_queue_buf[SD_LOG_QUEUE_DEPTH];
static DLog_Row_t s_imu_queue_buf[SD_IMU_QUEUE_DEPTH];

static QueuedFile_t s_log_qf = {
    .queue = s_log_queue_buf, .depth = SD_LOG_QUEUE_DEPTH,
    .dir = SD_LOG_DIR, .prefix = SD_LOG_FILE_PREFIX,
};
static QueuedFile_t s_imu_qf = {
    .queue = s_imu_queue_buf, .depth = SD_IMU_QUEUE_DEPTH,
    .dir = SD_IMU_DIR, .prefix = SD_IMU_FILE_PREFIX,
};

/**
 * @brief  Non-blocking: copy text into the next free ring slot. O(len).
 */
static uint8_t QF_Enqueue(QueuedFile_t *qf, const char *text, int len)
{
    if (qf->count >= qf->depth) {
        qf->rows_dropped++;
        return 0U;
    }
    if (len < 0) len = 0;
    if ((uint32_t)len >= SD_LOG_MAX_ROW_LEN) len = (int)SD_LOG_MAX_ROW_LEN - 1;

    DLog_Row_t *slot = &qf->queue[qf->head];
    memcpy(slot->text, text, (size_t)len);
    slot->text[len] = '\0';
    slot->len = (uint8_t)len;

    qf->head = (uint8_t)((qf->head + 1U) % qf->depth);
    qf->count++;
    return 1U;
}

/**
 * @brief  Drain at most one row to the SD card.
 * @return 1 if a write failed (caller should trigger a full remount —
 *         the failure almost certainly isn't specific to this file,
 *         since all files share one physical card/filesystem), else 0.
 */
static uint8_t QF_DrainOne(QueuedFile_t *qf)
{
    if (!qf->open || qf->count == 0U) return 0U;

    DLog_Row_t *slot = &qf->queue[qf->tail];
    UINT written = 0U;
    FRESULT fr = f_write(&qf->file, slot->text, slot->len, &written);

    if (fr != FR_OK || written != slot->len) {
        LOG_ERR("DataLogger: %s/%s write failed (FRESULT=%d)", qf->dir, qf->prefix, (int)fr);
        qf->write_errors++;
        return 1U;
    }

    qf->rows_written++;
    qf->tail  = (uint8_t)((qf->tail + 1U) % qf->depth);
    qf->count--;
    return 0U;
}

/* =========================================================================
 * SHARED STATE
 * ========================================================================= */

static FATFS               s_fatfs;
static uint8_t             s_mounted        = 0U;   /* one filesystem, shared by all three files */
static uint32_t            s_last_flush_ms  = 0U;

static FIL                 s_data_file;
static uint8_t             s_data_file_open = 0U;
static uint32_t            s_data_rows_written = 0U;
static uint32_t            s_data_write_errors = 0U;

static uint32_t            s_remount_count  = 0U;

static void DataLogger_ModuleInit(void);
static void DataLogger_ModuleTick(void);
static uint8_t DataLogger_Mount(void);
static uint8_t DataLogger_OpenIndexedFile(const char *dir, const char *prefix, FIL *file);
static void DataLogger_WriteLogHeader(void);
static void DataLogger_WriteDataHeader(void);
static void DataLogger_WriteImuHeader(void);

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
         * recognizes. _USE_MKFS is enabled in the vendored
         * Middlewares/Third_Party/FatFs/src/ffconf.h for exactly this
         * case (that file — NOT Core/Inc/ffconf.h — is the one that
         * actually governs ff.c: ff.h's #include "ffconf.h" resolves
         * same-directory first, so the vendored copy always wins).
         *
         * NOTE: this vendored FatFs release predates the MKFS_PARM-struct
         * f_mkfs() API — it uses the older 5-argument signature
         * (path, opt, au, work, len). */
        LOG_ERR("DataLogger: no FAT filesystem found on card — formatting");

        static uint8_t s_mkfs_work[512];   /* _MAX_SS in the vendored ffconf.h */

        /* FM_FAT (no FM_SFD): standard MBR-partitioned layout — the far
         * more commonly tested combination in this old FatFs release. */
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

    /* Ensure all three directories exist; FR_EXIST just means already there. */
    const char *dirs[3] = { SD_LOG_DIR, SD_DATA_DIR, SD_IMU_DIR };
    for (uint8_t i = 0U; i < 3U; i++) {
        fr = f_mkdir(dirs[i]);
        if (fr != FR_OK && fr != FR_EXIST) {
            LOG_ERR("DataLogger: f_mkdir(%s) failed (FRESULT=%d)", dirs[i], (int)fr);
            return 0U;
        }
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
 *         since each file needs a different header.
 */
static uint8_t DataLogger_OpenIndexedFile(const char *dir, const char *prefix, FIL *file)
{
    char path[48];
    FILINFO fno;

    for (uint32_t idx = 1U; idx <= 99999U; idx++) {
        /* NOTE: _USE_LFN is 0 in the vendored FatFs config, so filenames
         * are limited to 8.3 short names — max 8 chars before the dot.
         * "PREFIX" + 5 digits must be <= 8 chars; enforced at compile
         * time in Config.h for every prefix used here. */
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
    /* Metadata preamble — '#' prefix so any CSV reader (pandas, Excel,
     * etc.) treats these as comments to skip, while still capturing the
     * hyperparameters a paper's methods section needs. Pulled from
     * Config.h compile-time constants, so this can never drift out of
     * sync with the firmware that produced the accompanying rows. */
    char meta[SD_LOG_MAX_ROW_LEN];
    UINT written = 0U;
    int n;

    n = snprintf(meta, sizeof(meta), "# FedVibroSense STM32F405 run metadata\r\n");
    f_write(&s_log_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# nn_input=%u nn_hidden=%u nn_output=%u\r\n",
                 (unsigned)NN_INPUT_SIZE, (unsigned)NN_HIDDEN_SIZE, (unsigned)NN_OUTPUT_SIZE);
    f_write(&s_log_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# feature_vector_size=%u sample_rate_hz=%u feature_window_s=%u\r\n",
                 (unsigned)FEATURE_VECTOR_SIZE, (unsigned)IMU_SAMPLE_RATE_HZ,
                 (unsigned)FEATURE_WINDOW_SECONDS);
    f_write(&s_log_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# nn_learning_rate=%.6f fl_local_epochs=%u fl_standalone=%u\r\n",
                 (double)NN_LEARNING_RATE, (unsigned)FL_LOCAL_EPOCHS, (unsigned)FL_STANDALONE_MODE);
    f_write(&s_log_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta),
                 "type,seq_or_round,millis,pitch_or_loss,roll_or_samples,"
                 "pred_or_srv,p_normal,p_imbalance,p_looseness,label\r\n");
    f_write(&s_log_qf.file, meta, (UINT)n, &written);

    f_sync(&s_log_qf.file);
}

static void DataLogger_WriteDataHeader(void)
{
    /* Fixed prefix is only written once at file-open, so a slightly
     * larger transient stack buffer here (unlike the per-row hot path)
     * costs nothing — no RAM budget concern for something that doesn't
     * persist. */
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

static void DataLogger_WriteImuHeader(void)
{
    char meta[SD_LOG_MAX_ROW_LEN];
    UINT written = 0U;
    int n;

    n = snprintf(meta, sizeof(meta), "# Raw IMU stream, sample_rate_hz=%u\r\n",
                 (unsigned)IMU_SAMPLE_RATE_HZ);
    f_write(&s_imu_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# units: ax,ay,az [m/s^2] gx,gy,gz [deg/s, bias-compensated]\r\n");
    f_write(&s_imu_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# label = ground truth at capture time\r\n");
    f_write(&s_imu_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "# pred = most recent window prediction, updated once per "
                                      "second, NOT per row\r\n");
    f_write(&s_imu_qf.file, meta, (UINT)n, &written);

    n = snprintf(meta, sizeof(meta), "seq,micros,ax,ay,az,gx,gy,gz,label,pred\r\n");
    f_write(&s_imu_qf.file, meta, (UINT)n, &written);

    f_sync(&s_imu_qf.file);
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
    if (!s_mounted || !s_log_qf.open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "W,%lu,%lu,%.4f,%.4f,%u,%.4f,%.4f,%.4f,%u\r\n",
        (unsigned long)seq, (unsigned long)millis,
        (double)pitch, (double)roll, pred,
        (double)p_normal, (double)p_imbalance, (double)p_looseness, label);

    return QF_Enqueue(&s_log_qf, row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

DataLogger_Status_t DataLogger_LogFLRound(uint32_t round, uint32_t millis,
                                           float mean_loss, uint32_t total_samples,
                                           uint8_t server_connected)
{
    if (!s_mounted || !s_log_qf.open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "FL,%lu,%lu,%.6f,%lu,%u,,,,\r\n",
        (unsigned long)round, (unsigned long)millis,
        (double)mean_loss, (unsigned long)total_samples, server_connected);

    return QF_Enqueue(&s_log_qf, row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

/* =========================================================================
 * PUBLIC API — IMU FILE (async, queued — see DataLogger.h for why this
 * one must never block, unlike DATA below)
 * ========================================================================= */

DataLogger_Status_t DataLogger_LogRawIMU(uint32_t seq, uint32_t micros,
                                          float ax, float ay, float az,
                                          float gx, float gy, float gz,
                                          uint8_t label, uint8_t pred)
{
    if (!s_mounted || !s_imu_qf.open) return DLOG_ERR_MOUNT;

    char row[SD_LOG_MAX_ROW_LEN];
    int n = snprintf(row, sizeof(row),
        "%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u\r\n",
        (unsigned long)seq, (unsigned long)micros,
        (double)ax, (double)ay, (double)az,
        (double)gx, (double)gy, (double)gz,
        label, pred);

    return QF_Enqueue(&s_imu_qf, row, n) ? DLOG_OK : DLOG_ERR_QUEUE_FULL;
}

/* =========================================================================
 * PUBLIC API — DATA FILE (synchronous, unqueued)
 * ========================================================================= */

/**
 * Why this one is synchronous while LogWindow()/LogFLRound()/LogRawIMU()
 * aren't:
 *
 * A full row here is seq+millis+label+pred+3 probabilities+500 floats —
 * as text, roughly 4 KB. The queue design used by LOG/IMU (fixed-size RAM
 * slots) doesn't scale to that: even a queue depth of just 2 at 4 KB/slot
 * is 8 KB, and this MCU's SRAM1 is already near its budget (see README
 * "Memory Budget"). The obvious-looking workaround — park that queue in
 * CCM RAM, which is otherwise entirely unused — doesn't actually work
 * here: HAL_SD's DMA engine cannot address CCM on the STM32F4, so a write
 * source buffer living in CCM would fail or silently corrupt, not just
 * cost RAM.
 *
 * Given that, this writes directly and blocks: seq/millis/label/pred/
 * probabilities as small individual fields, then each of the 500 feature
 * values one at a time, all through a single reused ~48-byte stack
 * buffer — no large buffer anywhere, ever. This is deliberately NOT
 * built via this old FatFs release's f_printf(): its %f is unimplemented
 * (it silently treats unrecognized format characters as literal
 * pass-through text — i.e. "%.4f" would print the literal character 'f'
 * into the file, not the float value). snprintf() from the toolchain's
 * own libc is used instead, which does support floats correctly. Fields
 * are written individually rather than combined into one snprintf call,
 * since a combined buffer's worst case (a stray NaN/FLT_MAX blowing up
 * %f's decimal expansion) isn't something to eyeball when it's this
 * cheap to just not combine them — a lesson from a real truncation bug
 * caught by the compiler while writing the header below.
 *
 * Blocking here is an acceptable tradeoff because this is only called
 * once per feature window (1 Hz in this project), not at IMU sample
 * rate (100 Hz, see DataLogger_LogRawIMU() above) — a several-millisecond
 * SD write once a second doesn't meaningfully perturb the 10 ms IMU
 * sample timing the way it would on the hot path.
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
     * not just the ~8 chars a normal [0,1] softmax probability needs. */
    char field[48];
    UINT written = 0U;
    int n;
    FRESULT fr;

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

    s_data_rows_written++;
    return DLOG_OK;

write_failed:
    LOG_ERR("DataLogger: DATA file write failed (FRESULT=%d)", (int)fr);
    s_data_write_errors++;
    return DLOG_ERR_WRITE;
}

/* =========================================================================
 * SHARED API
 * ========================================================================= */

void DataLogger_Flush(void)
{
    if (s_log_qf.open)  { f_sync(&s_log_qf.file); s_last_flush_ms = Utils_GetMillis(); }
    if (s_imu_qf.open)  { f_sync(&s_imu_qf.file); }
    if (s_data_file_open) { f_sync(&s_data_file); }
}

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out == NULL) return;

    out->log_rows_written  = s_log_qf.rows_written;
    out->log_rows_dropped  = s_log_qf.rows_dropped;
    out->log_write_errors  = s_log_qf.write_errors;
    out->imu_rows_written  = s_imu_qf.rows_written;
    out->imu_rows_dropped  = s_imu_qf.rows_dropped;
    out->imu_write_errors  = s_imu_qf.write_errors;
    out->data_rows_written = s_data_rows_written;
    out->data_write_errors = s_data_write_errors;
    out->remount_count     = s_remount_count;
    out->log_mounted       = s_mounted && s_log_qf.open;
    out->imu_mounted       = s_mounted && s_imu_qf.open;
    out->data_mounted      = s_mounted && s_data_file_open;
}

/* =========================================================================
 * APPMODULE HOOKS
 * ========================================================================= */

static void DataLogger_ModuleInit(void)
{
    for (uint8_t attempt = 0U; attempt < SD_MOUNT_RETRY_COUNT; attempt++) {
        if (DataLogger_Mount() &&
            DataLogger_OpenIndexedFile(SD_LOG_DIR,  SD_LOG_FILE_PREFIX,  &s_log_qf.file) &&
            DataLogger_OpenIndexedFile(SD_DATA_DIR, SD_DATA_FILE_PREFIX, &s_data_file) &&
            DataLogger_OpenIndexedFile(SD_IMU_DIR,  SD_IMU_FILE_PREFIX,  &s_imu_qf.file)) {

            s_log_qf.open    = 1U;
            s_data_file_open = 1U;
            s_imu_qf.open    = 1U;
            DataLogger_WriteLogHeader();
            DataLogger_WriteDataHeader();
            DataLogger_WriteImuHeader();
            s_last_flush_ms = Utils_GetMillis();
            return;
        }
        LOG_ERR("DataLogger: mount/open attempt %u/%u failed, retrying...",
                attempt + 1U, SD_MOUNT_RETRY_COUNT);
        s_remount_count++;
        HAL_Delay(50U);
    }
    LOG_ERR("DataLogger: giving up after %u attempts — logging disabled for this boot",
             SD_MOUNT_RETRY_COUNT);
}

/**
 * @brief  Drain at most one row from each of the LOG and IMU queues per
 *         call, then flush all three files on a timer. Since the main
 *         loop iterates far faster than either queue's production rate
 *         under normal conditions, one row per queue per tick keeps both
 *         queues near-empty; queue depth (SD_LOG_QUEUE_DEPTH /
 *         SD_IMU_QUEUE_DEPTH) only matters as a shock absorber during an
 *         actual SD stall. The DATA file has no queue to drain here —
 *         DataLogger_LogRawWindow() already wrote it synchronously.
 */
static void DataLogger_ModuleTick(void)
{
    if (!s_log_qf.open) return;

    uint8_t log_failed = QF_DrainOne(&s_log_qf);
    uint8_t imu_failed = QF_DrainOne(&s_imu_qf);

    if (log_failed || imu_failed) {
        /* Card likely wedged — all three files share one filesystem, so
         * a failure on any one of them means a full remount, not a
         * per-file retry. */
        s_log_qf.open     = 0U;
        s_imu_qf.open     = 0U;
        s_data_file_open  = 0U;
        s_mounted         = 0U;
        f_close(&s_log_qf.file);
        f_close(&s_imu_qf.file);
        f_close(&s_data_file);
        DataLogger_ModuleInit();
        return;
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

DataLogger_Status_t DataLogger_LogRawIMU(uint32_t seq, uint32_t micros,
                                          float ax, float ay, float az,
                                          float gx, float gy, float gz,
                                          uint8_t label, uint8_t pred)
{
    (void)seq; (void)micros; (void)ax; (void)ay; (void)az;
    (void)gx; (void)gy; (void)gz; (void)label; (void)pred;
    return DLOG_DISABLED;
}

void DataLogger_Flush(void) { }

void DataLogger_GetStats(DataLogger_Stats_t *out)
{
    if (out) memset(out, 0, sizeof(*out));
}

#endif /* SD_LOGGING_ENABLE */