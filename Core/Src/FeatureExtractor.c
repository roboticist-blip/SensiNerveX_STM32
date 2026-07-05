/**
 * @file    FeatureExtractor.c
 * @brief   Ring-buffer IMU accumulation and 500-float feature vector construction
 *
 * Design decisions:
 *
 *  Ring buffer:
 *    - Power-of-2 depth (IMU_RING_BUFFER_DEPTH = 128) allows masking
 *      instead of modulo: idx = head & (DEPTH - 1)
 *    - Stores 5 floats per slot (pitch, roll, gyro_mag, accel_mag, delta_gyro)
 *    - No dynamic allocation; buffer lives inside FE_State_t (BSS segment)
 *
 *  Feature vector construction:
 *    - Extracts the last FEATURE_WINDOW_SAMPLES (100) entries from the ring
 *    - Lays them out channel-major: all 100 pitch values, then all 100 roll, etc.
 *    - Applies Z-score normalization per channel segment after extraction
 *
 *  Z-score normalization:
 *    - Mean and unbiased std computed in a single pass (Welford's algorithm)
 *    - If std < ZSCORE_EPSILON, segment is zeroed (flat signal → no information)
 *    - This is critical: raw IMU values span very different ranges
 *      (pitch: ±180°, accel_mag: ~9.8 m/s²) — normalization equalizes them
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#include "FeatureExtractor.h"
#include "Utils.h"
#include <string.h>
#include <math.h>

#define CH_PITCH        0U
#define CH_ROLL         1U
#define CH_GYRO_MAG     2U
#define CH_ACCEL_MAG    3U
#define CH_DELTA_GYRO   4U
#define NUM_CHANNELS    5U

void FE_Init(FE_State_t *fe)
{
    memset(fe, 0, sizeof(FE_State_t));
}

/**
 * @brief  Insert one CF_Output_t into the ring buffer.
 *
 * The ring buffer uses a power-of-2 depth mask for O(1) indexing:
 *   write_idx = (head) & (IMU_RING_BUFFER_DEPTH - 1)
 *
 * head is an unbounded uint32_t — never reset, wraps naturally.
 * count saturates at IMU_RING_BUFFER_DEPTH to track fill level.
 */
void FE_Push(FE_State_t *fe, const CF_Output_t *out)
{
    uint32_t idx = fe->ring.head & (IMU_RING_BUFFER_DEPTH - 1U);

    fe->ring.buf[idx][CH_PITCH]      = out->pitch;
    fe->ring.buf[idx][CH_ROLL]       = out->roll;
    fe->ring.buf[idx][CH_GYRO_MAG]   = out->gyro_mag;
    fe->ring.buf[idx][CH_ACCEL_MAG]  = out->accel_mag;
    fe->ring.buf[idx][CH_DELTA_GYRO] = out->delta_gyro_mag;

    fe->ring.head++;

    if (fe->ring.count < IMU_RING_BUFFER_DEPTH) {
        fe->ring.count++;
    }

    if (fe->ring.count >= FEATURE_WINDOW_SAMPLES) {
        fe->window_ready = 1U;
    }
}


/**
 * @brief  Normalize a float segment to zero mean and unit variance.
 *
 * Uses Welford's numerically stable one-pass mean and variance algorithm:
 *   mean   = sum(v) / n
 *   var    = sum((v - mean)²) / (n - 1)   [sample variance]
 *   std    = sqrt(var)
 *   v[i]  -= (v[i] - mean) / (std + eps)
 *
 * Why in-place two-pass instead of Welford single-pass?
 *   The array is small (100 elements, 400 bytes) and already in cache.
 *   Two cache-hot passes are faster and simpler than Welford's M2 accumulator.
 */
void FE_ZScoreNormalize(float *v, uint32_t len)
{
    if (len == 0U) return;

    // Pass 1: compute mean 
    float sum = 0.0f;
    for (uint32_t i = 0U; i < len; i++) {
        sum += v[i];
    }
    float mean = sum / (float)len;

    // Pass 2: compute variance (sum of squared deviations)
    float var_sum = 0.0f;
    for (uint32_t i = 0U; i < len; i++) {
        float diff = v[i] - mean;
        var_sum += diff * diff;
    }

    float std = sqrtf(var_sum / (float)(len > 1U ? len - 1U : 1U));

    if (std < ZSCORE_EPSILON) {
        memset(v, 0, len * sizeof(float));
        return;
    }

    float inv_std = 1.0f / (std + ZSCORE_EPSILON);
    for (uint32_t i = 0U; i < len; i++) {
        v[i] = (v[i] - mean) * inv_std;
    }
}

/**
 * @brief  Extract and normalize 500-float feature vector from ring buffer.
 *
 * The ring buffer is a circular array with 'head' pointing to the next
 * write position.  To read the last N samples in chronological order:
 *
 *   oldest = (head - FEATURE_WINDOW_SAMPLES) & mask
 *   sample[0] is at oldest
 *   sample[N-1] is at (head - 1) & mask
 *
 * We lay out channel-major (all pitch values first, then roll, etc.)
 * because the neural network's W1 matrix accesses contiguous input regions
 * during dot products → better cache behavior than interleaved layout.
 *
 * Memory writes:
 *   feature[0   ..  99] = pitch[0..99]
 *   feature[100 .. 199] = roll[0..99]
 *   feature[200 .. 299] = gyro_mag[0..99]
 *   feature[300 .. 399] = accel_mag[0..99]
 *   feature[400 .. 499] = delta_gyro_mag[0..99]
 */
uint8_t FE_BuildFeatureVector(const FE_State_t *fe,
                               float feature[FEATURE_VECTOR_SIZE])
{
    if (!fe->window_ready) return 0U;

    const uint32_t mask = IMU_RING_BUFFER_DEPTH - 1U;
    const uint32_t N    = FEATURE_WINDOW_SAMPLES;

    uint32_t start = (fe->ring.head - N) & mask;

    for (uint32_t s = 0U; s < N; s++) {
        uint32_t ring_idx = (start + s) & mask;

        feature[s + 0U * N] = fe->ring.buf[ring_idx][CH_PITCH];
        feature[s + 1U * N] = fe->ring.buf[ring_idx][CH_ROLL];
        feature[s + 2U * N] = fe->ring.buf[ring_idx][CH_GYRO_MAG];
        feature[s + 3U * N] = fe->ring.buf[ring_idx][CH_ACCEL_MAG];
        feature[s + 4U * N] = fe->ring.buf[ring_idx][CH_DELTA_GYRO];
    }

    for (uint32_t ch = 0U; ch < NUM_CHANNELS; ch++) {
        FE_ZScoreNormalize(&feature[ch * N], N);
    }

    return 1U;
}
