/**
 * @file    FeatureExtractor.h
 * @brief   Ring-buffer accumulation and feature vector construction
 *
 * Architecture:
 *
 *   CF_Output_t (per sample)
 *        │
 *        ▼
 *   Ring buffer [IMU_RING_BUFFER_DEPTH × 5 floats]
 *        │   (circular, no malloc, power-of-2 mask addressing)
 *        ▼
 *   FE_BuildFeatureVector()
 *        │   copies 5 channels × FEATURE_WINDOW_SAMPLES = 500 floats
 *        ▼
 *   Z-score normalization (per-channel mean/std over the window)
 *        │
 *        ▼
 *   float feature_vector[FEATURE_VECTOR_SIZE]  → NeuralNetwork input
 *
 * Memory budget:
 *   Ring buffer:  128 × 5 × 4 = 2560 bytes (static)
 *   Feature vec:  500 × 4     = 2000 bytes (static, caller-provided)
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include <stdint.h>
#include "Config.h"
#include "ComplementaryFilter.h"

/**
 * @brief  Ring buffer storing the last IMU_RING_BUFFER_DEPTH filter outputs.
 *
 * Each slot stores 5 floats:
 *   [0] pitch [°]
 *   [1] roll  [°]
 *   [2] gyro_mag [°/s]
 *   [3] accel_mag [m/s²]
 *   [4] delta_gyro_mag [°/s]
 *
 * Addressing uses power-of-2 mask (depth - 1) to avoid modulo overhead.
 */
typedef struct {
    float    buf[IMU_RING_BUFFER_DEPTH][5];  
    uint32_t head;                           
    uint32_t count;                         
} FE_RingBuffer_t;

/**
 * @brief  Feature extractor state — owns ring buffer and window statistics.
 */
typedef struct {
    FE_RingBuffer_t ring;             // Circular IMU data store
    uint8_t         window_ready;     //1 when >= FEATURE_WINDOW_SAMPLES present
} FE_State_t;

/**
 * @brief  Initialize the feature extractor state.
 *         Zeroes ring buffer and resets counters.
 * @param  fe  Pointer to caller-allocated FE_State_t
 */
void FE_Init(FE_State_t *fe);

/**
 * @brief  Push one complementary filter output into the ring buffer.
 *
 *
 * @param  fe   Feature extractor state
 * @param  out  CF_Output_t from the current filter step
 */
void FE_Push(FE_State_t *fe, const CF_Output_t *out);

/**
 * @brief  Build a 500-float normalized feature vector from the ring buffer.
 *
 * Must only be called when fe->window_ready == 1.
 *
 * Process:
 *  1. Copy the 100 most recent samples for each of 5 channels
 *     into the target array (channel-major order):
 *       feature[0..99]   = pitch[0..99]
 *       feature[100..199]= roll[0..99]
 *       feature[200..299]= gyro_mag[0..99]
 *       feature[300..399]= accel_mag[0..99]
 *       feature[400..499]= delta_gyro_mag[0..99]
 *  2. For each channel segment: compute mean and std, then z-score normalize.
 *
 * @param  fe       Feature extractor state
 * @param  feature  Output array of FEATURE_VECTOR_SIZE floats (caller-owned)
 * @return 1 on success, 0 if window not ready
 */
uint8_t FE_BuildFeatureVector(const FE_State_t *fe,
                               float feature[FEATURE_VECTOR_SIZE]);

/**
 * @brief  Z-score normalize a segment of the feature vector in place.
 *
 * Computes mean and unbiased std over [start, start+len).
 * Divides each element by (std + ZSCORE_EPSILON).
 * If std < ZSCORE_EPSILON, segment is zeroed (flat signal → no vibration info).
 *
 * @param  v    Float array (in-place)
 * @param  len  Segment length
 */
void FE_ZScoreNormalize(float *v, uint32_t len);

/**
 * @brief  Query whether a full feature window is available.
 * @param  fe  Feature extractor state
 * @return 1 if ready, 0 otherwise
 */
static inline uint8_t FE_IsWindowReady(const FE_State_t *fe)
{
    return fe->window_ready;
}

#endif /* FEATUREEXTRACTOR_H */
