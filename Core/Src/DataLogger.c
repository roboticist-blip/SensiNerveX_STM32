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

#include "ff.h"  
typedef struct {
    char    text[SD_LOG_MAX_ROW_LEN];
    uint8_t len;
} DLog_Row_t;

static FATFS               s_fatfs;
static uint8_t             s_mounted        = 0U;   
static FIL                 s_log_file;
static uint8_t             s_log_file_open  = 0U;
static uint32_t            s_last_flush_ms  = 0U;
static FIL                 s_data_file;
static uint8_t             s_data_file_open = 0U;


static DLog_Row_t          s_queue[SD_LOG_QUEUE_DEPTH];
static volatile uint8_t    s_q_head = 0U;   
static volatile uint8_t    s_q_tail = 0U;   
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

static uint8_t DataLogger_Mount(void)
{
    FRESULT fr = f_mount(&s_fatfs, "", 1);   

    if (fr == FR_NO_FILESYSTEM) {

        LOG_ERR("DataLogger: no FAT filesystem found on card — formatting");
        static uint8_t s_mkfs_work[512];

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

DataLogger_Status_t DataLogger_LogRawWindow(uint32_t seq, uint32_t millis,
                                             uint8_t label, uint8_t pred,
                                             float p_normal, float p_imbalance, float p_looseness,
                                             const float *feature_vec, uint16_t feature_len)
{
    if (!s_mounted || !s_data_file_open) return DLOG_ERR_MOUNT;
    if (feature_vec == NULL) return DLOG_ERR_WRITE;

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

    s_stats.data_rows_written++;
    return DLOG_OK;

write_failed:
    LOG_ERR("DataLogger: DATA file write failed (FRESULT=%d)", (int)fr);
    s_stats.data_write_errors++;
    return DLOG_ERR_WRITE;
}

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

#else 
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