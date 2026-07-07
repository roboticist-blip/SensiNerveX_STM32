/**
 * @file    DataLogger.h
 * @brief   Application-level SD logging service.
 *
 * This is the ONLY module that main.c and FederatedClient.c should talk
 * to for persistence. It owns the FatFs FATFS/FIL objects, session-file
 * naming, periodic flush policy, and mount-failure recovery.
 *
 * THREE independent files are written per session, in three separate
 * directories, with independently auto-incrementing session numbers:
 *
 *   LOGS/SNXNNNNN.CSV — operational log (DataLogger_LogWindow /
 *                        LogFLRound). Small rows (~150 bytes), async
 *                        queue, ~1 Hz production rate.
 *
 *   DATA/DATNNNNN.CSV — raw dataset (DataLogger_LogRawWindow). One row
 *                        per inference window: seq, millis, label, pred,
 *                        class probabilities, and the FULL
 *                        FEATURE_VECTOR_SIZE-length derived feature
 *                        vector. Written SYNCHRONOUSLY (blocking),
 *                        ~1 Hz — see the rationale in DataLogger.c above
 *                        DataLogger_LogRawWindow().
 *
 *   IMU/IMUNNNNN.CSV  — raw sensor stream (DataLogger_LogRawIMU). One
 *                        row per 100 Hz IMU sample: ax,ay,az,gx,gy,gz,
 *                        tagged with the current ground-truth label and
 *                        the most recent model prediction (see the
 *                        IMPORTANT note on DataLogger_LogRawIMU() below
 *                        about what "most recent" means at 100 Hz vs a
 *                        1 Hz prediction rate). Async queue, same
 *                        mechanism as the LOG file — 100 Hz absolutely
 *                        cannot go through a blocking write (see
 *                        Mod_ImuSample_Tick in main.c, and the "Why the
 *                        LOG/IMU files use a queue" note below) — but
 *                        needs a deeper queue than the LOG file's since
 *                        it fills ~100x faster (SD_IMU_QUEUE_DEPTH).
 *
 * Why the LOG and IMU files use a queue in front of the SD card, while
 * DATA does not:
 *   HAL_SD_WriteBlocks() and FatFs f_write()/f_sync() are blocking and
 *   can take several milliseconds (worse on a slow/aging card). The 100 Hz
 *   IMU sampling loop cannot tolerate that jitter on every single log
 *   line. So DataLogger_LogWindow()/LogFLRound()/LogRawIMU() just enqueue
 *   a pre-formatted row into a small RAM ring buffer (O(µs), never
 *   touches the SD card) and DataLogger_ModuleTick() — called once per
 *   main-loop iteration — drains at most one row per queue per call.
 *
 * This module conforms to the generic AppModule_t interface (AppModule.h)
 * so it plugs into main.c's module table exactly like every other
 * subsystem.
 *
 * @author  SensiNerveX Project
 * @version 1.2.0
 */

#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <stdint.h>
#include "Config.h"
#include "AppModule.h"

typedef enum {
    DLOG_OK             = 0,
    DLOG_ERR_MOUNT      = 1,
    DLOG_ERR_OPEN       = 2,
    DLOG_ERR_WRITE      = 3,
    DLOG_ERR_QUEUE_FULL = 4,   /**< Non-fatal: row dropped, counted in stats */
    DLOG_DISABLED       = 5,   /**< SD_LOGGING_ENABLE == 0 — all calls are no-ops */
} DataLogger_Status_t;

typedef struct {
    uint32_t log_rows_written;
    uint32_t log_rows_dropped;   /**< Queue was full — sign the SD card can't keep up */
    uint32_t log_write_errors;
    uint32_t data_rows_written;
    uint32_t data_write_errors;
    uint32_t imu_rows_written;
    uint32_t imu_rows_dropped;   /**< Queue was full — at 100 Hz this is the one to
                                       watch; non-zero means real IMU samples are
                                       missing from the raw stream, not just delayed */
    uint32_t imu_write_errors;
    uint32_t remount_count;      /**< Times the card was re-initialized after an error */
    uint8_t  log_mounted;
    uint8_t  data_mounted;
    uint8_t  imu_mounted;
} DataLogger_Stats_t;

/**
 * @brief  Register this module with the AppModule table (main.c calls
 *         AppModule_Register(&g_datalogger_module) once at boot).
 *         Init() mounts the filesystem and opens/creates both session
 *         files; Tick() drains one queued LOG row per call.
 */
extern AppModule_t g_datalogger_module;

/**
 * @brief  Enqueue one inference-window row to the LOG file.
 *         Format: "W,seq,millis,pitch,roll,pred,p_normal,p_imbalance,p_looseness,label\r\n"
 *         Never blocks on SD I/O — returns immediately.
 * @return DLOG_OK, or DLOG_ERR_QUEUE_FULL if the ring buffer is saturated
 *         (in which case this row is dropped and log_rows_dropped increments).
 */
DataLogger_Status_t DataLogger_LogWindow(uint32_t seq, uint32_t millis,
                                          float pitch, float roll,
                                          uint8_t pred,
                                          float p_normal, float p_imbalance, float p_looseness,
                                          uint8_t label);

/**
 * @brief  Enqueue one FL round-summary row to the LOG file.
 *         Format: "FL,round,millis,mean_loss,total_samples,server_connected\r\n"
 */
DataLogger_Status_t DataLogger_LogFLRound(uint32_t round, uint32_t millis,
                                           float mean_loss, uint32_t total_samples,
                                           uint8_t server_connected);

/**
 * @brief  Write one full raw-dataset row to the DATA file: ground-truth
 *         label, the model's own prediction/confidence at capture time,
 *         and the complete feature_len-length raw feature vector.
 *         BLOCKING — see the design note in DataLogger.h's header comment
 *         and in DataLogger.c above this function's implementation for
 *         why this one is synchronous while LogWindow()/LogFLRound()
 *         aren't. Safe to call once per second (the window cadence this
 *         project uses); not safe to call at IMU sample rate.
 * @param  feature_vec  Pointer to a feature_len-length float array.
 * @param  feature_len  Must equal FEATURE_VECTOR_SIZE for header/data
 *                       column counts to line up; passed explicitly
 *                       rather than hardcoded so a future feature-vector
 *                       resize doesn't silently desync the two.
 */
DataLogger_Status_t DataLogger_LogRawWindow(uint32_t seq, uint32_t millis,
                                             uint8_t label, uint8_t pred,
                                             float p_normal, float p_imbalance, float p_looseness,
                                             const float *feature_vec, uint16_t feature_len);

/**
 * @brief  Enqueue one raw IMU sample row to the IMU file — the 6-axis
 *         sensor reading (post-scaling, pre-feature-extraction), tagged
 *         with the current ground-truth label and the most recent
 *         model prediction. Called from Mod_ImuSample_Tick in main.c,
 *         i.e. at the full 100 Hz IMU sample rate.
 *
 *         IMPORTANT: "pred" here is NOT a fresh per-sample prediction —
 *         the model only predicts once per 1-second feature window, not
 *         per raw sample. Every IMU row between two window completions
 *         carries that same most-recent prediction (main.c tracks it in
 *         s_latest_pred, updated once per window). If you need to know
 *         exactly which IMU rows a given prediction is/isn't based on,
 *         cross-reference against the DATA file's "seq" column (the
 *         window that produced a given "pred" covers the ~100 preceding
 *         IMU rows at IMU_SAMPLE_RATE_HZ, not just the one row it
 *         happens to be printed next to).
 *
 *         Format: "seq,micros,ax,ay,az,gx,gy,gz,label,pred\r\n"
 *         Never blocks on SD I/O — returns immediately. This MUST stay
 *         non-blocking: it's called from the same tick that does the
 *         actual IMU read, and that tick's timing is what everything
 *         else in this project (feature windows, FL submission cadence)
 *         is built on top of.
 * @return DLOG_OK, or DLOG_ERR_QUEUE_FULL if the ring buffer is
 *         saturated (row dropped, imu_rows_dropped increments — check
 *         this after a run if the raw stream looks like it has gaps).
 */
DataLogger_Status_t DataLogger_LogRawIMU(uint32_t seq, uint32_t micros,
                                          float ax, float ay, float az,
                                          float gx, float gy, float gz,
                                          uint8_t label, uint8_t pred);

/**
 * @brief  Force-flush both open files to the card immediately (used by
 *         the 'f' UART debug command and before a controlled power-down).
 *         Blocking.
 */
void DataLogger_Flush(void);

/**
 * @brief  Snapshot of counters for the 's'/'w'-style status commands.
 */
void DataLogger_GetStats(DataLogger_Stats_t *out);

#endif /* DATALOGGER_H */