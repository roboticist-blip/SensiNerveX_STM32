/* FederatedClient.c — FL client FSM implementation */
#include "FederatedClient.h"
#include "Utils.h"
#include <string.h>

/* ACK sequence expected from aggregation server */
static const uint8_t _ack_seq[2] = { FLC_ACK_BYTE0, FLC_ACK_BYTE1 };

/* State string helper */

const char *FLC_StateStr(FLC_State_t state)
{
    switch (state) {
        case FLC_STATE_IDLE:        return "IDLE";
        case FLC_STATE_COLLECTING:  return "COLLECTING";
        case FLC_STATE_TRAINING:    return "TRAINING";
        case FLC_STATE_UPLOADING:   return "UPLOADING";
        case FLC_STATE_DOWNLOADING: return "DOWNLOADING";
        case FLC_STATE_ERROR:       return "ERROR";
        default:                    return "UNKNOWN";
    }
}

/* =========================================================================
 * INITIALIZATION
 * ========================================================================= */

void FLC_Init(FLC_Handle_t *flc, NN_Handle_t *nn, UART_HandleTypeDef *huart)
{
    memset(flc, 0, sizeof(FLC_Handle_t));
    flc->nn           = nn;
    flc->huart_fl     = huart;
    flc->state        = FLC_STATE_IDLE;
    flc->upload_fails = 0U;

#if FL_STANDALONE_MODE
    LOG_INF("FL client initialized — STANDALONE MODE (no server required)");
    LOG_INF("  Training locally, FL_LOCAL_EPOCHS=%u", FL_LOCAL_EPOCHS);
#else
    LOG_INF("FL client initialized — CONNECTED MODE (server required on UART1)");
    LOG_INF("  FL_LOCAL_EPOCHS=%u, FL_MAX_RETRIES=%u", FL_LOCAL_EPOCHS, FL_MAX_RETRIES);
#endif
}

/* =========================================================================
 * SAMPLE SUBMISSION
 * ========================================================================= */

uint8_t FLC_SubmitSample(FLC_Handle_t *flc,
                          const float feature[FEATURE_VECTOR_SIZE],
                          uint8_t label)
{
    if (flc->state != FLC_STATE_IDLE &&
        flc->state != FLC_STATE_COLLECTING) {
        return 0U;
    }

    if (flc->train_buf.count >= FL_LOCAL_EPOCHS) {
        LOG_ERR("FL: train buffer full, ignoring sample");
        return 0U;
    }

    if (flc->state == FLC_STATE_IDLE) {
        flc->state = FLC_STATE_COLLECTING;
        LOG_INF("FL: IDLE → COLLECTING");
    }

    uint8_t idx = flc->train_buf.count;
    memcpy(flc->train_buf.features[idx], feature,
           FEATURE_VECTOR_SIZE * sizeof(float));
    flc->train_buf.labels[idx] = label;
    flc->train_buf.count++;

    LOG_INF("FL: buffered sample %u/%u, label=%u",
            flc->train_buf.count, FL_LOCAL_EPOCHS, label);

    if (flc->train_buf.count >= FL_LOCAL_EPOCHS) {
        flc->state = FLC_STATE_TRAINING;
        LOG_INF("FL: COLLECTING → TRAINING (%u samples ready)", FL_LOCAL_EPOCHS);
    }

    return 1U;
}

/* =========================================================================
 * FSM TICK
 * ========================================================================= */

void FLC_Tick(FLC_Handle_t *flc)
{
    switch (flc->state) {
        case FLC_STATE_IDLE:
        case FLC_STATE_COLLECTING:
            break;

        case FLC_STATE_TRAINING:
            FLC_DoTraining(flc);
            break;

        case FLC_STATE_UPLOADING:
#if FL_STANDALONE_MODE
            /* ── STANDALONE: skip upload entirely, go straight to IDLE ── */
            LOG_INF("FL: [STANDALONE] skipping upload, continuing locally");
            flc->round_count++;
            memset(&flc->train_buf, 0, sizeof(FLC_TrainingBuffer_t));
            flc->state = FLC_STATE_IDLE;
            FLC_PrintStatus(flc);
#else
            FLC_DoUpload(flc);
#endif
            break;

        case FLC_STATE_DOWNLOADING:
#if FL_STANDALONE_MODE
            /* Should never reach here in standalone mode */
            flc->state = FLC_STATE_IDLE;
#else
            FLC_DoDownload(flc);
#endif
            break;

        case FLC_STATE_ERROR:
            LOG_ERR("FL: error code %u — resetting to IDLE", flc->error_code);
            FLC_Reset(flc);
            break;

        default:
            LOG_ERR("FL: unknown state %u", (unsigned)flc->state);
            flc->state = FLC_STATE_ERROR;
            break;
    }
}

/* =========================================================================
 * TRAINING PHASE
 * ========================================================================= */

void FLC_DoTraining(FLC_Handle_t *flc)
{
    LOG_INF("FL: === LOCAL TRAINING START (round %lu) ===",
            flc->round_count + 1UL);

    float total_loss = 0.0f;

    for (uint8_t s = 0U; s < flc->train_buf.count; s++) {
        float loss = NN_TrainStep(flc->nn,
                                  flc->train_buf.features[s],
                                  flc->train_buf.labels[s]);
        total_loss += loss;
        LOG_INF("FL: train step %u/%u label=%u loss=%.4f",
                s + 1U, flc->train_buf.count,
                flc->train_buf.labels[s], (double)loss);
    }

    flc->last_round_loss = total_loss / (float)flc->train_buf.count;
    flc->total_samples  += flc->train_buf.count;

    LOG_INF("FL: training done, mean_loss=%.4f, total_samples=%lu",
            (double)flc->last_round_loss, flc->total_samples);

#if FL_STANDALONE_MODE
    /* In standalone mode, training IS the complete round */
    LOG_INF("FL: TRAINING → UPLOADING (standalone: will skip)");
#else
    LOG_INF("FL: TRAINING → UPLOADING");
#endif

    flc->state = FLC_STATE_UPLOADING;
}

/* =========================================================================
 * UPLOAD PHASE  (only compiled/used in connected mode)
 * ========================================================================= */

#if !FL_STANDALONE_MODE

void FLC_DoUpload(FLC_Handle_t *flc)
{
    LOG_INF("FL: UPLOADING weights (%u bytes)...", SER_TOTAL_PACKET_BYTES);

    /* Serialize weights */
    SER_Status_t ser_status = SER_SerializeWeights(
        flc->nn,
        FL_PACKET_MAGIC_UPLOAD,
        ser_packet_buf,
        SER_TOTAL_PACKET_BYTES
    );

    if (ser_status != SER_OK) {
        LOG_ERR("FL: serialization failed, code=%u", (unsigned)ser_status);
        flc->error_code = (uint8_t)ser_status;
        flc->state = FLC_STATE_ERROR;
        return;
    }

    /* Transmit over UART1 */
    HAL_StatusTypeDef tx_ret = HAL_UART_Transmit(
        flc->huart_fl,
        ser_packet_buf,
        SER_TOTAL_PACKET_BYTES,
        10000U   /* 10 s TX timeout — generous for 32 KB at 921600 baud */
    );

    if (tx_ret != HAL_OK) {
        LOG_ERR("FL: UART TX failed, HAL=%d", (int)tx_ret);
        flc->upload_fails++;
        _FLC_HandleUploadFailure(flc);
        return;
    }

    LOG_INF("FL: upload TX complete, waiting for ACK...");

    /* Wait for 2-byte ACK */
    uint8_t ack_buf[2] = {0U, 0U};
    HAL_StatusTypeDef rx_ret = HAL_UART_Receive(
        flc->huart_fl,
        ack_buf,
        2U,
        UART_RX_TIMEOUT_MS
    );

    if (rx_ret != HAL_OK) {
        LOG_ERR("FL: ACK timeout (HAL=%d) — is fed_server.py running on RPi?",
                (int)rx_ret);
        flc->upload_fails++;
        _FLC_HandleUploadFailure(flc);
        return;
    }

    if (ack_buf[0] != _ack_seq[0] || ack_buf[1] != _ack_seq[1]) {
        LOG_ERR("FL: bad ACK bytes [0x%02X 0x%02X]", ack_buf[0], ack_buf[1]);
        flc->upload_fails++;
        _FLC_HandleUploadFailure(flc);
        return;
    }

    /* ACK received — proceed to download */
    flc->upload_fails     = 0U;
    flc->server_connected = 1U;
    LOG_INF("FL: ACK received — UPLOADING → DOWNLOADING");
    flc->state = FLC_STATE_DOWNLOADING;
}

/* =========================================================================
 * DOWNLOAD PHASE
 * ========================================================================= */

void FLC_DoDownload(FLC_Handle_t *flc)
{
    LOG_INF("FL: DOWNLOADING global model (%u bytes)...",
            SER_TOTAL_PACKET_BYTES);

    HAL_StatusTypeDef rx_ret = HAL_UART_Receive(
        flc->huart_fl,
        ser_packet_buf,
        SER_TOTAL_PACKET_BYTES,
        UART_RX_TIMEOUT_MS * 100U
    );

    if (rx_ret != HAL_OK) {
        LOG_ERR("FL: download RX timeout (HAL=%d)", (int)rx_ret);
        flc->error_code = 0xFCU;
        flc->state = FLC_STATE_ERROR;
        return;
    }

    SER_Status_t deser_status = SER_DeserializeWeights(
        flc->nn,
        ser_packet_buf,
        SER_TOTAL_PACKET_BYTES
    );

    if (deser_status != SER_OK) {
        LOG_ERR("FL: deserialization failed, code=%u", (unsigned)deser_status);
        flc->error_code = (uint8_t)deser_status;
        flc->state = FLC_STATE_ERROR;
        return;
    }

    flc->round_count++;
    memset(&flc->train_buf, 0, sizeof(FLC_TrainingBuffer_t));
    flc->server_connected = 1U;

    LOG_INF("FL: === ROUND %lu COMPLETE (with server) ===", flc->round_count);
    flc->state = FLC_STATE_IDLE;
    FLC_PrintStatus(flc);
}

/* =========================================================================
 * UPLOAD FAILURE HANDLER — retry logic
 * ========================================================================= */

void _FLC_HandleUploadFailure(FLC_Handle_t *flc)
{
    flc->server_connected = 0U;

    if (flc->upload_fails >= FL_MAX_RETRIES) {
        LOG_ERR("FL: upload failed %u/%u times — switching to local-only mode",
                flc->upload_fails, FL_MAX_RETRIES);
        LOG_ERR("FL: weights are trained locally, continuing data collection");
        /* Don't discard training — keep weights, reset buffer, go IDLE */
        flc->upload_fails = 0U;
        flc->round_count++;   /* Count as a local-only round */
        memset(&flc->train_buf, 0, sizeof(FLC_TrainingBuffer_t));
        flc->state = FLC_STATE_IDLE;
        FLC_PrintStatus(flc);
    } else {
        LOG_ERR("FL: retry %u/%u — will attempt upload again next tick",
                flc->upload_fails, FL_MAX_RETRIES);
        /* Stay in UPLOADING state to retry immediately next FLC_Tick() call */
        /* But first, delay briefly to avoid hammering the UART */
        HAL_Delay(500U);
    }
}

#endif /* !FL_STANDALONE_MODE */

/* =========================================================================
 * UTILITY
 * ========================================================================= */

void FLC_TriggerUpload(FLC_Handle_t *flc)
{
    if (flc->state == FLC_STATE_COLLECTING ||
        flc->state == FLC_STATE_IDLE) {
        LOG_INF("FL: manual upload triggered");
        flc->state = FLC_STATE_UPLOADING;
    }
}

void FLC_Reset(FLC_Handle_t *flc)
{
    memset(&flc->train_buf, 0, sizeof(FLC_TrainingBuffer_t));
    flc->state        = FLC_STATE_IDLE;
    flc->error_code   = 0U;
    flc->upload_fails = 0U;
    LOG_INF("FL: reset to IDLE");
}

void FLC_PrintStatus(const FLC_Handle_t *flc)
{
    LOG_INF("FL Status: state=%s round=%lu samples=%lu loss=%.4f server=%u fails=%u",
            FLC_StateStr(flc->state),
            flc->round_count,
            flc->total_samples,
            (double)flc->last_round_loss,
            flc->server_connected,
            flc->upload_fails);
    NN_PrintWeightStats(flc->nn);
}
