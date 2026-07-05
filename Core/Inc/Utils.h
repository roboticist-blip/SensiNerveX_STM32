/**
 * @file    Utils.h
 * @brief   Utility macros, math helpers, and debug infrastructure
 *
 * Provides:
 *  - Debug logging macros (level-gated, UART-backed)
 *  - Timing measurement utilities
 *  - Inline math helpers (min/max/clamp, fast inverse sqrt)
 *  - CRC-16 computation
 *  - Memory footprint reporting
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef UTILS_H
#define UTILS_H
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Config.h"


#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart2;   // Debug UART 
extern UART_HandleTypeDef huart1;   // FL communication UART 
extern TIM_HandleTypeDef  htim2;    // Free-running microsecond timer 



extern char _utils_log_buf[DEBUG_LOG_BUF_SIZE];

/**
 * @brief Internal log helper — do not call directly; use LOG_* macros.
 */
void Utils_LogWrite(const char *buf, uint16_t len);

#if (DEBUG_LEVEL >= 1)
#define LOG_ERR(fmt, ...) \
    do { \
        int _n = snprintf(_utils_log_buf, DEBUG_LOG_BUF_SIZE, \
                          "[ERR] " fmt "\r\n", ##__VA_ARGS__); \
        if (_n > 0) Utils_LogWrite(_utils_log_buf, (uint16_t)_n); \
    } while(0)
#else
#define LOG_ERR(fmt, ...) do {} while(0)
#endif

#if (DEBUG_LEVEL >= 2)
#define LOG_INF(fmt, ...) \
    do { \
        int _n = snprintf(_utils_log_buf, DEBUG_LOG_BUF_SIZE, \
                          "[INF] " fmt "\r\n", ##__VA_ARGS__); \
        if (_n > 0) Utils_LogWrite(_utils_log_buf, (uint16_t)_n); \
    } while(0)
#else
#define LOG_INF(fmt, ...) do {} while(0)
#endif

#if (DEBUG_LEVEL >= 3)
#define LOG_VRB(fmt, ...) \
    do { \
        int _n = snprintf(_utils_log_buf, DEBUG_LOG_BUF_SIZE, \
                          "[VRB] " fmt "\r\n", ##__VA_ARGS__); \
        if (_n > 0) Utils_LogWrite(_utils_log_buf, (uint16_t)_n); \
    } while(0)
#else
#define LOG_VRB(fmt, ...) do {} while(0)
#endif

/**
 * @brief  Initialize the microsecond timer (call once in main before use).
 *         Configures TIM2 as a free-running 32-bit microsecond counter.
 */
void Utils_TimerInit(void);

/**
 * @brief  Return current timestamp in microseconds.
 *         Wraps after ~71.5 minutes.
 * @return uint32_t microseconds
 */
static inline uint32_t Utils_GetMicros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief  Return current timestamp in milliseconds via HAL tick.
 * @return uint32_t milliseconds since boot
 */
static inline uint32_t Utils_GetMillis(void)
{
    return HAL_GetTick();
}

/**
 * @brief  Compute elapsed microseconds between start and now.
 *         Handles 32-bit wraparound correctly.
 * @param  start_us  Timestamp from Utils_GetMicros()
 * @return uint32_t  Elapsed microseconds
 */
static inline uint32_t Utils_ElapsedMicros(uint32_t start_us)
{
    return Utils_GetMicros() - start_us;   /* unsigned subtraction wraps correctly */
}

/** Clamp x to [lo, hi] */
static inline float Utils_ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/** Absolute value of float (avoids linking fabsf in some toolchains) */
static inline float Utils_AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

/** Integer power of 2 check */
static inline uint8_t Utils_IsPow2(uint32_t v)
{
    return (v != 0U) && ((v & (v - 1U)) == 0U);
}

/**
 * @brief  Compute 1/sqrt(x) using Quake III fast approximation
 *         followed by one Newton-Raphson refinement step.
 *         Accurate to < 0.175% — sufficient for magnitude normalization.
 * @param  x  Input (must be > 0)
 * @return Approximation of 1/sqrt(x)
 */
static inline float Utils_FastInvSqrt(float x)
{
    union { float f; uint32_t i; } conv = { .f = x };
    conv.i = 0x5F3759DFU - (conv.i >> 1U);
    float y = conv.f;
    y *= 1.5f - (x * 0.5f * y * y);   /* Newton-Raphson step */
    return y;
}

/**
 * @brief  Compute vector L2 norm of length n using FPU.
 * @param  v  Pointer to float array
 * @param  n  Length
 * @return float L2 norm
 */
float Utils_VecNorm(const float *v, uint32_t n);

/**
 * @brief  Clip all elements of vector to [-clip, +clip].
 * @param  v     Pointer to float array
 * @param  n     Length
 * @param  clip  Clip magnitude
 */
void Utils_VecClip(float *v, uint32_t n, float clip);

/**
 * @brief  Compute CRC-16/CCITT over a byte buffer.
 * @param  data  Pointer to data
 * @param  len   Number of bytes
 * @return uint16_t CRC value
 */
uint16_t Utils_CRC16(const uint8_t *data, uint32_t len);


/**
 * @brief  Estimate remaining heap/stack gap by walking the stack.
 *         Requires the linker script to export _estack and _end symbols.
 *         Prints result via LOG_INF.
 */
void Utils_PrintMemoryStats(void);

/**
 * @brief  Seed the internal LCG PRNG.
 * @param  seed  Non-zero seed value
 */
void Utils_SeedRNG(uint32_t seed);

/**
 * @brief  Return next pseudo-random float in [-1, +1].
 * @return float in [-1.0, +1.0]
 */
float Utils_RandF(void);

#endif /* UTILS_H */
