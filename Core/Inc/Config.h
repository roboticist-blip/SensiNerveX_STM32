/* Config.h — central configuration for FedVibroSense */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* FEDERATED LEARNING MODE — set to 1 to run locally without a server */
#define FL_STANDALONE_MODE         1U
#define FL_MAX_RETRIES             3U

/* =========================================================================
 * SYSTEM CLOCK & TIMING
 * ========================================================================= */

#define SYS_CORE_CLOCK_HZ          168000000UL
#define IMU_SAMPLE_RATE_HZ         100U
#define IMU_SAMPLE_PERIOD_S        (1.0f / IMU_SAMPLE_RATE_HZ)
#define FEATURE_WINDOW_SECONDS     1U
#define FEATURE_WINDOW_SAMPLES     (IMU_SAMPLE_RATE_HZ * FEATURE_WINDOW_SECONDS)

/* =========================================================================
 * FEATURE EXTRACTION
 * ========================================================================= */

#define FEATURE_VECTOR_SIZE        500U
#define IMU_RAW_AXES               6U
#define ZSCORE_EPSILON             1e-7f

/* =========================================================================
 * COMPLEMENTARY FILTER
 * ========================================================================= */

#define CF_ALPHA                   0.98f
#define GRAVITY_MSS                9.80665f

/* =========================================================================
 * NEURAL NETWORK
 * ========================================================================= */

#define NN_INPUT_SIZE              FEATURE_VECTOR_SIZE
#define NN_HIDDEN_SIZE             16U
#define NN_OUTPUT_SIZE             3U
#define NN_LEARNING_RATE           0.01f
#define NN_XAVIER_SCALE_HIDDEN     0.06324555f
#define NN_XAVIER_SCALE_OUTPUT     0.35355339f
#define NN_SOFTMAX_TEMP            1.0f
#define NN_GRAD_CLIP               5.0f

/**
 * Number of labeled windows collected before one local training round.
 * Lower = faster rounds but noisier gradients.
 * Higher = smoother gradients but more time before first upload.
 * Recommended: 10 for research, 5 for fast iteration during development.
 */
#define FL_LOCAL_EPOCHS            10U

/* =========================================================================
 * FEDERATED LEARNING PACKET
 * ========================================================================= */

#define FL_WEIGHT_COUNT            (NN_INPUT_SIZE * NN_HIDDEN_SIZE + \
                                    NN_HIDDEN_SIZE + \
                                    NN_HIDDEN_SIZE * NN_OUTPUT_SIZE + \
                                    NN_OUTPUT_SIZE)
#define FL_PACKET_MAGIC_UPLOAD     0xFE01U
#define FL_PACKET_MAGIC_DOWNLOAD   0xFE02U
#define FL_CRC16_POLY              0x1021U
#define FL_CRC16_INIT              0xFFFFU

/* =========================================================================
 * UART
 * ========================================================================= */

#define UART_DEBUG_BAUD            115200U
#define UART_FL_BAUD               921600U
#define UART_TX_TIMEOUT_MS         100U
#define UART_RX_TIMEOUT_MS         5000U
#define UART_MAX_PAYLOAD_BYTES     (FL_WEIGHT_COUNT * 4U + 16U)

/* =========================================================================
 * I2C / MPU6050
 * ========================================================================= */

#define MPU6050_I2C_ADDR           0x68U
#define MPU6050_I2C_ADDR_SHIFTED   (MPU6050_I2C_ADDR << 1)
#define I2C_TIMEOUT_MS             10U
#define MPU6050_ACCEL_FS           0U
#define MPU6050_ACCEL_SENSITIVITY  16384.0f
#define MPU6050_GYRO_FS            0U
#define MPU6050_GYRO_SENSITIVITY   131.0f
#define MPU6050_CALIBRATION_SAMPLES 200U
#define MPU6050_DLPF_CFG           3U

/* =========================================================================
 * MEMORY & BUFFERS
 * ========================================================================= */

#define IMU_RING_BUFFER_DEPTH      128U
#define DEBUG_LOG_BUF_SIZE         128U
#define LOSS_HISTORY_LEN           16U

/* =========================================================================
 * CLASS LABELS
 * ========================================================================= */

#define CLASS_NORMAL               0U
#define CLASS_IMBALANCE            1U
#define CLASS_LOOSENESS            2U

/* =========================================================================
 * DEBUG LEVEL (0=off, 1=errors only, 2=info, 3=verbose)
 * ========================================================================= */

#define DEBUG_LEVEL                2U

/* =========================================================================
 * COMPILE-TIME ASSERTIONS
 * ========================================================================= */

_Static_assert(FEATURE_VECTOR_SIZE == 500U, "FEATURE_VECTOR_SIZE must be 500");
_Static_assert(FL_LOCAL_EPOCHS > 0U, "FL_LOCAL_EPOCHS must be >= 1");
_Static_assert(IMU_RING_BUFFER_DEPTH >= FEATURE_WINDOW_SAMPLES,
    "Ring buffer must hold at least one full window");

#endif /* CONFIG_H */
