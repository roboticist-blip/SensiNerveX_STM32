/**
 * @file    Serialization.h
 * @brief   Model weight serialization / deserialization for FL communication
 *
 * Packet format (binary, little-endian):
 *
 *  ┌──────────┬──────────┬────────────┬───────────────────┬──────────┐
 *  │ Magic    │ Version  │ Num Weights│ float32 payload   │ CRC-16   │
 *  │ 2 bytes  │ 1 byte   │ 2 bytes    │ N × 4 bytes       │ 2 bytes  │
 *  └──────────┴──────────┴────────────┴───────────────────┴──────────┘
 *
 *  Magic:       0xFE01 (upload) or 0xFE02 (download)
 *  Version:     Protocol version = 0x01
 *  Num Weights: FL_WEIGHT_COUNT (8067) as uint16_t
 *  Payload:     W1 | b1 | W2 | b2 in row-major order (all float32 LE)
 *  CRC-16:      CRC-16/CCITT over [Magic..Payload]
 *
 * Total packet size: 2+1+2 + 8067×4 + 2 = 32275 bytes
 *
 * At 921600 baud UART: ~8-bit framing → ~352 ms transfer time.
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <stdint.h>
#include "Config.h"
#include "NeuralNetwork.h"


#define SER_PROTOCOL_VERSION     0x01U
#define SER_HEADER_SIZE          5U       // magic(2) + ver(1) + nweights(2) 
#define SER_CRC_SIZE             2U
#define SER_PAYLOAD_BYTES        (FL_WEIGHT_COUNT * 4U)
#define SER_TOTAL_PACKET_BYTES   (SER_HEADER_SIZE + SER_PAYLOAD_BYTES + SER_CRC_SIZE)

/* =========================================================================
 * SERIALIZATION BUFFER
 *
 * The transmit buffer is statically allocated here and shared between
 * upload and download operations (never simultaneously).
 * Size = SER_TOTAL_PACKET_BYTES
 * ========================================================================= */

extern uint8_t ser_packet_buf[SER_TOTAL_PACKET_BYTES];
typedef enum {
    SER_OK              = 0,
    SER_ERR_MAGIC       = 1,   //< Wrong magic bytes in received packet 
    SER_ERR_VERSION     = 2,   //< Unsupported protocol version 
    SER_ERR_LENGTH      = 3,   //< Payload length mismatch 
    SER_ERR_CRC         = 4,   //< CRC-16 mismatch 
    SER_ERR_BUFFER      = 5,   //< Buffer too small 
} SER_Status_t;


/**
 * @brief  Serialize NN weights into the binary FL packet format.
 *
 * Packs W1, b1, W2, b2 (row-major float32 little-endian) into
 * ser_packet_buf with header and CRC-16 appended.
 *
 * Weight order inside payload:
 *   [0 .. 7999]  W1[0..499][0..15]  (row-major)
 *   [8000..8015] b1[0..15]
 *   [8016..8063] W2[0..15][0..2]   (row-major)
 *   [8064..8066] b2[0..2]
 *
 * @param  nn      Source neural network
 * @param  magic   FL_PACKET_MAGIC_UPLOAD or FL_PACKET_MAGIC_DOWNLOAD
 * @param  out_buf Output buffer (must be >= SER_TOTAL_PACKET_BYTES)
 * @param  buf_len Size of out_buf in bytes
 * @return SER_Status_t
 */
SER_Status_t SER_SerializeWeights(const NN_Handle_t *nn,
                                   uint16_t magic,
                                   uint8_t *out_buf,
                                   uint32_t buf_len);

/**
 * @brief  Deserialize a received FL packet and apply weights to NN.
 *
 * Validates magic, version, weight count, and CRC before writing
 * any weights into the network. If validation fails, NN state is
 * left unchanged.
 *
 * @param  nn       Destination neural network
 * @param  in_buf   Received byte buffer
 * @param  buf_len  Number of bytes in in_buf
 * @return SER_Status_t
 */
SER_Status_t SER_DeserializeWeights(NN_Handle_t *nn,
                                     const uint8_t *in_buf,
                                     uint32_t buf_len);

/**
 * @brief  Pack a single float32 into a byte buffer at offset (little-endian).
 * @param  buf     Destination buffer
 * @param  offset  Byte offset to write at
 * @param  val     Float value
 */
static inline void SER_WriteFloat(uint8_t *buf, uint32_t offset, float val)
{
    /* Use memcpy-safe union to avoid strict-aliasing UB */
    union { float f; uint8_t b[4]; } u;
    u.f = val;
    buf[offset + 0] = u.b[0];
    buf[offset + 1] = u.b[1];
    buf[offset + 2] = u.b[2];
    buf[offset + 3] = u.b[3];
}

/**
 * @brief  Read a single float32 from a byte buffer at offset (little-endian).
 * @param  buf     Source buffer
 * @param  offset  Byte offset to read from
 * @return float value
 */
static inline float SER_ReadFloat(const uint8_t *buf, uint32_t offset)
{
    union { float f; uint8_t b[4]; } u;
    u.b[0] = buf[offset + 0];
    u.b[1] = buf[offset + 1];
    u.b[2] = buf[offset + 2];
    u.b[3] = buf[offset + 3];
    return u.f;
}

/**
 * @brief  Encode a uint16_t into a buffer at offset (little-endian).
 */
static inline void SER_WriteU16(uint8_t *buf, uint32_t offset, uint16_t val)
{
    buf[offset + 0] = (uint8_t)(val & 0xFFU);
    buf[offset + 1] = (uint8_t)(val >> 8U);
}

/**
 * @brief  Decode a uint16_t from a buffer at offset (little-endian).
 */
static inline uint16_t SER_ReadU16(const uint8_t *buf, uint32_t offset)
{
    return (uint16_t)(buf[offset] | ((uint16_t)buf[offset + 1] << 8U));
}

/**
 * @brief  Compute and verify the CRC-16 over a packet buffer.
 * @param  buf  Pointer to complete packet (header + payload)
 * @param  len  Length of header + payload (excluding CRC bytes)
 * @return uint16_t CRC value
 */
uint16_t SER_ComputeCRC(const uint8_t *buf, uint32_t len);

#endif /* SERIALIZATION_H */
