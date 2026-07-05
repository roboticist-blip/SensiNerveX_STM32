/**
 * @file    Serialization.c
 * @brief   FL model weight binary serialization / deserialization
 *
 * Packet layout:
 *   Offset   Size  Field
 *   ------   ----  -----
 *   0        2     Magic (LE uint16)
 *   2        1     Version (uint8 = 0x01)
 *   3        2     NumWeights (LE uint16 = FL_WEIGHT_COUNT = 8067)
 *   5        N×4   Float32 weights (W1 | b1 | W2 | b2), little-endian
 *   5+N×4    2     CRC-16/CCITT over bytes [0 .. 5+N×4-1]
 *
 *   N = FL_WEIGHT_COUNT = 8067
 *   Total = 5 + 32268 + 2 = 32275 bytes
 *
 * @author  SensiNerveX Project
 * @version 1.0.0
 */

#include "Serialization.h"
#include "Utils.h"
#include <string.h>

/* 
 * STATIC PACKET BUFFER
 *
 * Placed in BSS — zero-initialized by startup code.
 * Shared between upload and download (never simultaneous).
 */

uint8_t ser_packet_buf[SER_TOTAL_PACKET_BYTES];

uint16_t SER_ComputeCRC(const uint8_t *buf, uint32_t len)
{
    return Utils_CRC16(buf, len);
}

/**
 * @brief  Pack neural network weights into the FL binary packet format.
 *
 * Weight serialization order:
 *   1. W1[0..499][0..15]  = 8000 float32 values, row-major
 *   2. b1[0..15]          =   16 float32 values
 *   3. W2[0..15][0..2]    =   48 float32 values, row-major
 *   4. b2[0..2]           =    3 float32 values
 *   Total = 8067 float32 = 32268 bytes
 *
 * Header:
 *   [0..1] magic   (LE)
 *   [2]    version = SER_PROTOCOL_VERSION
 *   [3..4] num_weights = FL_WEIGHT_COUNT (LE)
 *
 * Footer (after payload):
 *   [5+N*4 .. 5+N*4+1] CRC-16 over [0 .. 5+N*4-1]
 */
SER_Status_t SER_SerializeWeights(const NN_Handle_t *nn,
                                   uint16_t magic,
                                   uint8_t *out_buf,
                                   uint32_t buf_len)
{
    if (buf_len < SER_TOTAL_PACKET_BYTES) {
        LOG_ERR("SER: buffer too small (%lu < %u)", buf_len, SER_TOTAL_PACKET_BYTES);
        return SER_ERR_BUFFER;
    }

    uint32_t offset = 0U;

    SER_WriteU16(out_buf, offset, magic);
    offset += 2U;

    out_buf[offset++] = SER_PROTOCOL_VERSION;

    SER_WriteU16(out_buf, offset, (uint16_t)FL_WEIGHT_COUNT);
    offset += 2U;

    for (uint32_t i = 0U; i < NN_INPUT_SIZE * NN_HIDDEN_SIZE; i++) {
        SER_WriteFloat(out_buf, offset, nn->W1[i]);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_HIDDEN_SIZE; i++) {
        SER_WriteFloat(out_buf, offset, nn->b1[i]);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_HIDDEN_SIZE * NN_OUTPUT_SIZE; i++) {
        SER_WriteFloat(out_buf, offset, nn->W2[i]);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_OUTPUT_SIZE; i++) {
        SER_WriteFloat(out_buf, offset, nn->b2[i]);
        offset += 4U;
    }

    uint16_t crc = SER_ComputeCRC(out_buf, offset);
    SER_WriteU16(out_buf, offset, crc);
    offset += 2U;

    LOG_INF("SER: serialized %lu bytes, magic=0x%04X, CRC=0x%04X",
            offset, magic, crc);

    return SER_OK;
}

/**
 * @brief  Validate and unpack a received FL packet into the neural network.
 *
 * Validation sequence (fail-fast, NN unchanged on any error):
 *   1. Buffer length check
 *   2. Magic bytes
 *   3. Protocol version
 *   4. Weight count
 *   5. CRC-16 integrity
 *
 * Only after all checks pass do we write to nn->W1, b1, W2, b2.
 */
SER_Status_t SER_DeserializeWeights(NN_Handle_t *nn,
                                     const uint8_t *in_buf,
                                     uint32_t buf_len)
{
    if (buf_len < SER_TOTAL_PACKET_BYTES) {
        LOG_ERR("SER: rx packet too short (%lu < %u)", buf_len, SER_TOTAL_PACKET_BYTES);
        return SER_ERR_BUFFER;
    }

    uint32_t offset = 0U;

    uint16_t rx_magic = SER_ReadU16(in_buf, offset);
    offset += 2U;
    if (rx_magic != FL_PACKET_MAGIC_DOWNLOAD) {
        LOG_ERR("SER: bad magic 0x%04X (expected 0x%04X)", rx_magic,
                FL_PACKET_MAGIC_DOWNLOAD);
        return SER_ERR_MAGIC;
    }

    uint8_t rx_ver = in_buf[offset++];
    if (rx_ver != SER_PROTOCOL_VERSION) {
        LOG_ERR("SER: unsupported version 0x%02X", rx_ver);
        return SER_ERR_VERSION;
    }

    uint16_t rx_nweights = SER_ReadU16(in_buf, offset);
    offset += 2U;
    if (rx_nweights != (uint16_t)FL_WEIGHT_COUNT) {
        LOG_ERR("SER: weight count mismatch (%u vs %u)", rx_nweights,
                (unsigned)FL_WEIGHT_COUNT);
        return SER_ERR_LENGTH;
    }

    uint32_t payload_end = SER_HEADER_SIZE + SER_PAYLOAD_BYTES;
    uint16_t rx_crc      = SER_ReadU16(in_buf, payload_end);
    uint16_t calc_crc    = SER_ComputeCRC(in_buf, payload_end);

    if (rx_crc != calc_crc) {
        LOG_ERR("SER: CRC mismatch (rx=0x%04X, calc=0x%04X)", rx_crc, calc_crc);
        return SER_ERR_CRC;
    }

    for (uint32_t i = 0U; i < NN_INPUT_SIZE * NN_HIDDEN_SIZE; i++) {
        nn->W1[i] = SER_ReadFloat(in_buf, offset);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_HIDDEN_SIZE; i++) {
        nn->b1[i] = SER_ReadFloat(in_buf, offset);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_HIDDEN_SIZE * NN_OUTPUT_SIZE; i++) {
        nn->W2[i] = SER_ReadFloat(in_buf, offset);
        offset += 4U;
    }

    for (uint32_t i = 0U; i < NN_OUTPUT_SIZE; i++) {
        nn->b2[i] = SER_ReadFloat(in_buf, offset);
        offset += 4U;
    }

    LOG_INF("SER: deserialized global model OK, CRC=0x%04X", calc_crc);
    return SER_OK;
}
