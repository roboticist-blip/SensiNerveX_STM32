/*
 * @file    MPU6050.h
 * @brief   MPU-6050 IMU driver for STM32F405 (HAL I2C)
 *
 * Provides:
 *  - Register-level initialization and configuration
 *  - Raw 16-bit burst read (accel + temp + gyro in one transaction)
 *  - Gyroscope bias calibration (static, device-at-rest)
 *  - Scaled output in physical units (m/s², °/s)
 *  - Interrupt-based data-ready signaling (optional EXTI path)
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Config.h"

#define MPU6050_REG_SELF_TEST_X    0x0DU
#define MPU6050_REG_SELF_TEST_Y    0x0EU
#define MPU6050_REG_SELF_TEST_Z    0x0FU
#define MPU6050_REG_SELF_TEST_A    0x10U
#define MPU6050_REG_SMPLRT_DIV     0x19U
#define MPU6050_REG_CONFIG         0x1AU
#define MPU6050_REG_GYRO_CONFIG    0x1BU
#define MPU6050_REG_ACCEL_CONFIG   0x1CU
#define MPU6050_REG_FIFO_EN        0x23U
#define MPU6050_REG_INT_PIN_CFG    0x37U
#define MPU6050_REG_INT_ENABLE     0x38U
#define MPU6050_REG_INT_STATUS     0x3AU
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU
#define MPU6050_REG_ACCEL_XOUT_L   0x3CU
#define MPU6050_REG_ACCEL_YOUT_H   0x3DU
#define MPU6050_REG_ACCEL_YOUT_L   0x3EU
#define MPU6050_REG_ACCEL_ZOUT_H   0x3FU
#define MPU6050_REG_ACCEL_ZOUT_L   0x40U
#define MPU6050_REG_TEMP_OUT_H     0x41U
#define MPU6050_REG_TEMP_OUT_L     0x42U
#define MPU6050_REG_GYRO_XOUT_H    0x43U
#define MPU6050_REG_GYRO_XOUT_L    0x44U
#define MPU6050_REG_GYRO_YOUT_H    0x45U
#define MPU6050_REG_GYRO_YOUT_L    0x46U
#define MPU6050_REG_GYRO_ZOUT_H    0x47U
#define MPU6050_REG_GYRO_ZOUT_L    0x48U
#define MPU6050_REG_USER_CTRL      0x6AU
#define MPU6050_REG_PWR_MGMT_1     0x6BU
#define MPU6050_REG_PWR_MGMT_2     0x6CU
#define MPU6050_REG_WHO_AM_I       0x75U

#define MPU6050_WHO_AM_I_VAL       0x68U

/*
 * @brief Raw sensor output (unscaled 16-bit ADC counts).
 */
typedef struct {
    int16_t accel_x;   //< Accelerometer X raw count 
    int16_t accel_y;   //< Accelerometer Y raw count 
    int16_t accel_z;   //< Accelerometer Z raw count 
    int16_t temp;      //< Temperature raw count 
    int16_t gyro_x;    //< Gyroscope X raw count 
    int16_t gyro_y;    //< Gyroscope Y raw count 
    int16_t gyro_z;    //< Gyroscope Z raw count 
} MPU6050_RawData_t;

/*
 * @brief Scaled sensor output in physical units.
 */
typedef struct {
    float ax;   //< Acceleration X [m/s²] 
    float ay;   //< Acceleration Y [m/s²] 
    float az;   //< Acceleration Z [m/s²] 
    float gx;   //< Angular velocity X [°/s], bias-compensated 
    float gy;   //< Angular velocity Y [°/s], bias-compensated 
    float gz;   //< Angular velocity Z [°/s], bias-compensated 
    float temp_c; //< Temperature [°C] 
} MPU6050_Data_t;

/*
 * @brief Driver handle — owns I2C reference and calibration state.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      //< Pointer to HAL I2C handle 
    uint8_t  dev_addr;            //< 8-bit shifted I2C address 
    float    gyro_bias_x;         //< Gyro bias X [°/s] from calibration 
    float    gyro_bias_y;         //< Gyro bias Y [°/s] from calibration 
    float    gyro_bias_z;         //< Gyro bias Z [°/s] from calibration 
    float    accel_bias_x;        //< Accel bias X [m/s²] (optional) 
    float    accel_bias_y;        //< Accel bias Y [m/s²] (optional) 
    uint8_t  initialized;         //< 1 if MPU6050_Init() succeeded 
    uint8_t  data_ready_flag;     //< Set by EXTI ISR if interrupt mode used 
} MPU6050_Handle_t;

typedef enum {
    MPU6050_OK          = 0,
    MPU6050_ERR_I2C     = 1,
    MPU6050_ERR_WHO_AM_I = 2,
    MPU6050_ERR_TIMEOUT = 3,
    MPU6050_ERR_NOTINIT = 4,
} MPU6050_Status_t;

/*
 * @brief  Initialize the MPU-6050 and verify device identity.
 *
 * Sequence:
 *  1. WHO_AM_I check (aborts on mismatch)
 *  2. Device reset → 100 ms delay
 *  3. Clock source = PLL with X-axis gyroscope reference
 *  4. Sample rate divider set for IMU_SAMPLE_RATE_HZ
 *  5. DLPF configured per MPU6050_DLPF_CFG
 *  6. Accel FS and Gyro FS set per Config.h
 *  7. Data Ready interrupt enabled (active low, push-pull)
 *
 * @param  hnd   Pointer to caller-allocated MPU6050_Handle_t
 * @param  hi2c  Pointer to HAL I2C handle (must be initialized by CubeMX)
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hnd, I2C_HandleTypeDef *hi2c);

/*
 * @brief  Calibrate gyroscope bias (device must be perfectly still).
 *
 * Averages MPU6050_CALIBRATION_SAMPLES readings and stores the mean
 * in hnd->gyro_bias_{x,y,z}.  Also performs a sanity check on accel Z
 * to confirm the device is horizontal.
 *
 * @param  hnd  Pointer to initialized MPU6050_Handle_t
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_Calibrate(MPU6050_Handle_t *hnd);

/*
 * @brief  Burst-read all 14 bytes (accel + temp + gyro) in one I2C transaction.
 *
 * This is the primary data-acquisition call executed at IMU_SAMPLE_RATE_HZ.
 * One 14-byte burst is more efficient than 7 individual register reads.
 *
 * @param  hnd   Initialized handle
 * @param  raw   Output raw data
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Handle_t *hnd, MPU6050_RawData_t *raw);

/*
 * @brief  Convert raw counts to physical units and apply bias compensation.
 *
 * Accelerometer: raw / ACCEL_SENSITIVITY * GRAVITY_MSS → m/s²
 * Gyroscope:     raw / GYRO_SENSITIVITY - bias → °/s
 * Temperature:   raw / 340.0 + 36.53 → °C
 *
 * @param  hnd   Initialized handle (bias fields must be set by Calibrate)
 * @param  raw   Raw input
 * @param  out   Scaled output
 */
void MPU6050_ConvertToPhysical(const MPU6050_Handle_t *hnd,
                                const MPU6050_RawData_t *raw,
                                MPU6050_Data_t *out);

/*
 * @brief  Combined read + convert in one call (convenience wrapper).
 * @param  hnd   Initialized handle
 * @param  out   Scaled output
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadScaled(MPU6050_Handle_t *hnd, MPU6050_Data_t *out);

/*
 * @brief  Read a single register byte.
 * @param  hnd   Initialized handle
 * @param  reg   Register address
 * @param  data  Output byte
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadReg(MPU6050_Handle_t *hnd, uint8_t reg, uint8_t *data);

/*
 * @brief  Write a single register byte.
 * @param  hnd   Initialized handle
 * @param  reg   Register address
 * @param  data  Byte to write
 * @return MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *hnd, uint8_t reg, uint8_t data);

/*
 * @brief  Clear the data_ready_flag (called after reading data in ISR mode).
 * @param  hnd  Handle
 */
static inline void MPU6050_ClearDataReady(MPU6050_Handle_t *hnd)
{
    hnd->data_ready_flag = 0U;
}

/*
 * @brief  Check if new data is available.
 * @param  hnd  Handle
 * @return 1 if data_ready_flag is set, 0 otherwise
 */
static inline uint8_t MPU6050_IsDataReady(const MPU6050_Handle_t *hnd)
{
    return hnd->data_ready_flag;
}

/*
 * @brief  EXTI callback — call from HAL_GPIO_EXTI_Callback when INT fires.
 *         Sets data_ready_flag in the handle for polling-based consumers.
 * @param  hnd  Handle
 */
void MPU6050_DataReadyISR(MPU6050_Handle_t *hnd);

#endif /* MPU6050_H */
