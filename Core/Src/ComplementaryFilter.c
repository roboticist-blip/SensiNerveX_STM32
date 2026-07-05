/**
 * @file    ComplementaryFilter.c
 * @brief   Complementary filter implementation for pitch/roll estimation
 *
 * Fuses accelerometer and gyroscope data to produce stable orientation
 * estimates for vibration feature extraction.
 *
 * Mathematical background:
 *
 *  The complementary filter exploits the frequency domain complementarity
 *  of the two sensors:
 *    - Accelerometer: accurate at low frequencies (DC and quasi-static tilt)
 *                     but noisy at high frequencies (vibration, shock)
 *    - Gyroscope:     accurate at high frequencies (fast rotation)
 *                     but drifts at low frequencies (integration error)
 *
 *  High-pass filter on gyro:   θ_gyro[k] = α × (θ[k-1] + ω × dt)
 *  Low-pass filter on accel:   θ_acc[k]  = (1-α) × arctan2(...)
 *  Fused:                      θ[k] = θ_gyro[k] + θ_acc[k]
 *
 *  Time constant: τ = α × dt / (1 - α)
 *  With α=0.98, dt=0.01 s: τ = 0.49 s
 *  Cross-over frequency: fc = 1/(2πτ) ≈ 0.32 Hz
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#include "ComplementaryFilter.h"
#include "Utils.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

#define RAD_TO_DEG  (180.0f / (float)M_PI)
#define DEG_TO_RAD  ((float)M_PI / 180.0f)

/**
 * @brief  Initialize complementary filter state.
 *
 * The 'initialized' flag is cleared so the first call to CF_Update()
 * seeds the angle estimates purely from the accelerometer, avoiding
 * a large transient at startup.
 */
void CF_Init(CF_State_t *state)
{
    state->pitch       = 0.0f;
    state->roll        = 0.0f;
    state->alpha       = CF_ALPHA;
    state->dt          = IMU_SAMPLE_PERIOD_S;
    state->initialized = 0U;
}

/**
 * @brief  Reset the filter (e.g., after mounting orientation change).
 */
void CF_Reset(CF_State_t *state)
{
    state->pitch       = 0.0f;
    state->roll        = 0.0f;
    state->initialized = 0U;
}


/**
 * @brief  Derive pitch from accelerometer readings.
 *
 * Uses the 4-quadrant atan2 formulation:
 *   pitch = atan2(ay, sqrt(ax² + az²)) × (180/π)
 *
 * This definition keeps pitch in [-90°, +90°] and is insensitive to
 * sign changes in az (avoids flip at ±90° of tilt).
 *
 * @param  ax, ay, az  Acceleration components in m/s²
 * @return pitch in degrees
 */
float CF_AccelPitch(float ax, float ay, float az)
{
    float denom = sqrtf(ax * ax + az * az);
    if (denom < 1e-6f) denom = 1e-6f;
    return atan2f(ay, denom) * RAD_TO_DEG;
}

/**
 * @brief  Derive roll from accelerometer readings.
 *
 *   roll = atan2(ax, sqrt(ay² + az²)) × (180/π)
 *
 * @param  ax, ay, az  Acceleration components in m/s²
 * @return roll in degrees
 */
float CF_AccelRoll(float ax, float ay, float az)
{
    float denom = sqrtf(ay * ay + az * az);
    if (denom < 1e-6f) denom = 1e-6f;
    return atan2f(ax, denom) * RAD_TO_DEG;
}

/**
 * @brief  Process one IMU sample through the complementary filter.
 *
 * Execution profile on Cortex-M4 @ 168 MHz (approximate):
 *   atan2f:          ~30 µs (uses FPU atan2 instruction via libm)
 *   sqrtf:           ~1 µs  (hardware VSQRT.F32)
 *   filter math:     ~1 µs  (all VMAC/VADD FPU ops)
 *   Total:           ~32 µs per call → 3.2% CPU at 100 Hz
 *
 * @param  state   Complementary filter state (updated in-place)
 * @param  imu     Scaled IMU sample (m/s² and °/s)
 * @param  out     Computed pitch, roll, and vibration magnitudes
 */
void CF_Update(CF_State_t *state, const MPU6050_Data_t *imu, CF_Output_t *out)
{
    float pitch_acc = CF_AccelPitch(imu->ax, imu->ay, imu->az);
    float roll_acc  = CF_AccelRoll (imu->ax, imu->ay, imu->az);

    if (!state->initialized) {
        state->pitch       = pitch_acc;
        state->roll        = roll_acc;
        state->initialized = 1U;
    } else {
        float pitch_gyro = state->pitch + imu->gx * state->dt;
        float roll_gyro  = state->roll  + imu->gy * state->dt;

        state->pitch = state->alpha * pitch_gyro
                     + (1.0f - state->alpha) * pitch_acc;
        state->roll  = state->alpha * roll_gyro
                     + (1.0f - state->alpha) * roll_acc;
    }

    float gx = imu->gx, gy = imu->gy, gz = imu->gz;
    float gyro_mag_new = sqrtf(gx * gx + gy * gy + gz * gz);


    float ax = imu->ax, ay = imu->ay, az = imu->az;
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);

    float prev_gyro_mag = out->gyro_mag;  
    float delta_gyro_mag = gyro_mag_new - prev_gyro_mag;

    out->pitch          = state->pitch;
    out->roll           = state->roll;
    out->gyro_mag       = gyro_mag_new;
    out->accel_mag      = accel_mag;
    out->delta_gyro_mag = delta_gyro_mag;
}
