/**
 * @file    DataLogger.h
 * @brief   Application-level SD logging service.
 *
 * This is the ONLY module that main.c and FederatedClient.c should talk
 * to for persistence. It owns the FatFs FATFS/FIL objects, session-file
 * naming, periodic flush policy, and mount-failure recovery.
 *
 * TWO independent files are written per session, in two separate
 * directories, with independently auto-incrementing session numbers:
 *
 *   LOGS/SNXNNNNN.CSV  — operational log (DataLogger_LogWindow /
 *                         LogFLRound). Small rows (~150 bytes), written
 *                         via the async RAM queue described below, since
 *                         these get produced at up to 1 Hz continuously
 *                         and must never stall the caller.
 *
 *   DATA/DATNNNNN.CSV  — raw dataset (DataLogger_LogRawWindow). One row
 *                         per inference window: seq, millis, label, pred,
 *                         class probabilities, and the FULL
 *                         FEATURE_VECTOR_SIZE-length raw feature vector —
 *                         everything needed to retrain or re-derive
 *                         results externally for publication. Written
 *                         SYNCHRONOUSLY (blocking), deliberately NOT
 *                         through the async queue — see the rationale in
 *                         DataLogger.c above DataLogger_LogRawWindow().
 *                         In short: a ~4 KB/row queue at any usable depth
 *                         doesn't fit this MCU's RAM budget, and the one
 *                         apparent workaround (parking the queue buffer
 *                         in CCM RAM, which is otherwise unused) doesn't
 *                         work here because HAL_SD's DMA engine cannot
 *                         address CCM on the F4 — so the buffer would
 *                         have to live in normal SRAM regardless. Given
 *                         that, a blocking write once per second (the
 *                         window cadence) is simpler and no less safe
 *                         than an async queue that gains nothing.
 *
 * Why the LOG file still uses a queue in front of the SD card:
 *   HAL_SD_WriteBlocks() and FatFs f_write()/f_sync() are blocking and
 *   can take several milliseconds (worse on a slow/aging card). The 100 Hz
 *   IMU sampling loop cannot tolerate that jitter on every single log
 *   line. So DataLogger_LogWindow()/LogFLRound() just enqueue a
 *   pre-formatted row into a small RAM ring buffer (O(µs), never touches
 *   the SD card) and DataLogger_ModuleTick() — called once per main-loop
 *   iteration — drains at most one row per call.
 *
 * This module conforms to the generic AppModule_t interface (AppModule.h)
 * so it plugs into main.c's module table exactly like every other
 * subsystem.
 *
 * @author  SensiNerveX Project
 * @version 1.1.0
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
    uint32_t remount_count;      /**< Times the card was re-initialized after an error */
    uint8_t  log_mounted;
    uint8_t  data_mounted;
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