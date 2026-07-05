/**
 * @file    ComplementaryFilter.h
 * @brief   Complementary filter for pitch/roll estimation from IMU data
 *
 * Fuses accelerometer angle (low-frequency accurate) with gyroscope
 * integration (high-frequency accurate) using:
 *
 *   angle[k] = α × (angle[k-1] + ω × dt) + (1-α) × θ_acc
 *
 * where:
 *   α   = CF_ALPHA (0.98)  — gyro trust weight
 *   ω   = gyroscope rate [°/s]
 *   dt  = sample period [s]
 *   θ_acc = atan2-derived angle from accelerometer
 *
 * Benefits over Kalman filter:
 *  - Zero dynamic memory
 *  - Deterministic O(1) execution per sample
 *  - Sufficient for vibration feature extraction (absolute accuracy < 1°)
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

#include <stdint.h>
#include "Config.h"
#include "MPU6050.h"

/**
 * @brief Complementary filter state (one instance per IMU axis pair).
 */
typedef struct {
    float pitch;          // Current pitch estimate [°] 
    float roll;           // Current roll estimate [°] 
    float alpha;          // Gyro trust weight (CF_ALPHA) 
    float dt;             // Sample period [s] (IMU_SAMPLE_PERIOD_S) 
    uint8_t initialized;  // 1 after first sample processed 
} CF_State_t;

/**
 * @brief  Filtered orientation and derived vibration features per sample.
 */
typedef struct {
    float pitch;           //< Filtered pitch [°]
    float roll;            //< Filtered roll  [°] 
    float gyro_mag;        //< Gyroscope vector magnitude [°/s] 
    float accel_mag;       //< Accelerometer vector magnitude [m/s²] 
    float delta_gyro_mag;  //< Change in gyro_mag from previous sample 
} CF_Output_t;

/**
 * @brief  Initialize the complementary filter state.
 *
 * @param  state  Pointer to caller-allocated CF_State_t
 */
void CF_Init(CF_State_t *state);

/**
 * @brief  Reset filter state (useful when node changes orientation).
 * @param  state  Filter state
 */
void CF_Reset(CF_State_t *state);

/**
 * @brief  Process one IMU sample through the complementary filter.
 *
 * Steps:
 *  1. Compute accel-derived pitch and roll via atan2f
 *  2. On first call: seed angle estimates from accelerometer
 *  3. Integrate gyroscope to predict new angle
 *  4. Fuse prediction with accel measurement using alpha
 *  5. Compute gyro_mag, accel_mag, delta_gyro_mag
 *
 * Execution is O(1) with fixed FPU operations; no dynamic allocation.
 *
 * @param  state   Filter state (updated in-place)
 * @param  imu     Scaled IMU sample from MPU6050_ReadScaled()
 * @param  out     Filtered output written here
 */
void CF_Update(CF_State_t *state, const MPU6050_Data_t *imu, CF_Output_t *out);

/**
 * @brief  Compute pitch angle from accelerometer [°].
 *         pitch = atan2(ay, sqrt(ax² + az²)) * 180/π
 * @param  ax, ay, az  Accelerations in m/s²
 * @return float pitch in degrees
 */
float CF_AccelPitch(float ax, float ay, float az);

/**
 * @brief  Compute roll angle from accelerometer [°].
 *         roll = atan2(ax, sqrt(ay² + az²)) * 180/π
 * @param  ax, ay, az  Accelerations in m/s²
 * @return float roll in degrees
 */
float CF_AccelRoll(float ax, float ay, float az);

#endif /* COMPLEMENTARYFILTER_H */
