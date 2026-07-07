/**
 * @file    DataLogger.h
 * @brief   Application-level SD logging service.
 *
 * This is the ONLY module that main.c and FederatedClient.c should talk
 * to for persistence. It owns the FatFs FATFS/FIL objects, session-file
 * naming, periodic flush policy, and mount-failure recovery.
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
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
    DLOG_ERR_QUEUE_FULL = 4,   
    DLOG_DISABLED       = 5,   
} DataLogger_Status_t;

typedef struct {
    uint32_t log_rows_written;
    uint32_t log_rows_dropped;   
    uint32_t log_write_errors;
    uint32_t data_rows_written;
    uint32_t data_write_errors;
    uint32_t imu_rows_written;
    uint32_t imu_rows_dropped;   
    uint32_t imu_write_errors;
    uint32_t remount_count;      
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