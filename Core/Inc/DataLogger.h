/**
 * @file    DataLogger.h
 * @brief   Application-level SD logging service.
 *
 * This is the ONLY module that main.c and FederatedClient.c should talk
 * to for persistence. It owns the FatFs FATFS/FIL objects, session-file
 * naming, periodic flush policy, and mount-failure recovery — none of
 * that leaks into main.c, which only ever calls DataLogger_LogWindow(),
 * DataLogger_LogFLRound(), and DataLogger_Tick().
 *
 * Why a queue in front of the SD card:
 *   HAL_SD_WriteBlocks() and FatFs f_write()/f_sync() are blocking and
 *   can take several milliseconds (worse on a slow/aging card). The 100 Hz
 *   IMU sampling loop and the FL FSM cannot tolerate that jitter. So
 *   producers (App main loop) only ever push a pre-formatted row into a
 *   small RAM ring buffer (DataLogger_LogWindow/LogFLRound — O(µs), never
 *   touches the SD card) and DataLogger_Tick() — called once per main-loop
 *   iteration, same as every other module — drains at most one row per
 *   call and writes/flushes it. This bounds the worst-case SD stall seen
 *   by any single loop iteration to one row's worth of I/O.
 *
 * This module conforms to the generic AppModule_t interface (AppModule.h)
 * so it plugs into main.c's module table exactly like every other
 * subsystem — that's the "scalability" half of this change: a future
 * second storage sink (e.g. an SPI flash black-box recorder) is another
 * AppModule_t, not another set of hand-wired calls in main.c.
 *
 * @author  SensiNerveX Project
 * @version 1.0.0
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
    uint32_t rows_written;
    uint32_t rows_dropped;     /**< Queue was full — sign the SD card can't keep up */
    uint32_t write_errors;
    uint32_t remount_count;    /**< Times the card was re-initialized after an error */
    uint8_t  mounted;
} DataLogger_Stats_t;

/**
 * @brief  Register this module with the AppModule table (main.c calls
 *         AppModule_Register(&g_datalogger_module) once at boot).
 *         Init() mounts the filesystem and opens/creates the session
 *         log file; Tick() drains one queued row per call.
 */
extern AppModule_t g_datalogger_module;

/**
 * @brief  Enqueue one inference-window row for logging.
 *         Format: "seq,millis,pitch,roll,pred,p_normal,p_imbalance,p_looseness,label\n"
 *         Never blocks on SD I/O — returns immediately.
 * @return DLOG_OK, or DLOG_ERR_QUEUE_FULL if the ring buffer is saturated
 *         (in which case this row is dropped and rows_dropped increments).
 */
DataLogger_Status_t DataLogger_LogWindow(uint32_t seq, uint32_t millis,
                                          float pitch, float roll,
                                          uint8_t pred,
                                          float p_normal, float p_imbalance, float p_looseness,
                                          uint8_t label);

/**
 * @brief  Enqueue one FL round-summary row.
 *         Format: "FL,round,millis,mean_loss,total_samples,server_connected\n"
 */
DataLogger_Status_t DataLogger_LogFLRound(uint32_t round, uint32_t millis,
                                           float mean_loss, uint32_t total_samples,
                                           uint8_t server_connected);

/**
 * @brief  Force-flush the currently open file to the card immediately
 *         (used by the 'f' UART debug command and before a controlled
 *         power-down). Blocking.
 */
void DataLogger_Flush(void);

/**
 * @brief  Snapshot of counters for the 's'/'w'-style status commands.
 */
void DataLogger_GetStats(DataLogger_Stats_t *out);

#endif /* DATALOGGER_H */
