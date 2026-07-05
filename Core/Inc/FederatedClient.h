#ifndef FEDERATEDCLIENT_H
#define FEDERATEDCLIENT_H

#include <stdint.h>
#include "Config.h"
#include "NeuralNetwork.h"
#include "Serialization.h"
#include "FeatureExtractor.h"
#include "stm32f4xx_hal.h"

typedef enum {
    FLC_STATE_IDLE        = 0,
    FLC_STATE_COLLECTING  = 1,
    FLC_STATE_TRAINING    = 2,
    FLC_STATE_UPLOADING   = 3,
    FLC_STATE_DOWNLOADING = 4,
    FLC_STATE_ERROR       = 5,
} FLC_State_t;

#define FLC_ACK_BYTE0   0xACU
#define FLC_ACK_BYTE1   0xACU

typedef struct {
    float   features[FL_LOCAL_EPOCHS][FEATURE_VECTOR_SIZE];
    uint8_t labels[FL_LOCAL_EPOCHS];
    uint8_t count;
} FLC_TrainingBuffer_t;

typedef struct {
    FLC_State_t           state;
    NN_Handle_t           *nn;
    UART_HandleTypeDef    *huart_fl;
    FLC_TrainingBuffer_t  train_buf;
    uint32_t              round_count;
    uint32_t              total_samples;
    float                 last_round_loss;
    uint8_t               server_connected;
    uint8_t               error_code;
    uint8_t               upload_fails;   
} FLC_Handle_t;

void    FLC_Init(FLC_Handle_t *flc, NN_Handle_t *nn, UART_HandleTypeDef *huart);
uint8_t FLC_SubmitSample(FLC_Handle_t *flc,
                          const float feature[FEATURE_VECTOR_SIZE],
                          uint8_t label);
void    FLC_Tick(FLC_Handle_t *flc);
void    FLC_TriggerUpload(FLC_Handle_t *flc);
void    FLC_Reset(FLC_Handle_t *flc);
const char *FLC_StateStr(FLC_State_t state);
void    FLC_PrintStatus(const FLC_Handle_t *flc);

void    FLC_DoTraining(FLC_Handle_t *flc);
void    FLC_DoUpload(FLC_Handle_t *flc);
void    FLC_DoDownload(FLC_Handle_t *flc);
void    _FLC_HandleUploadFailure(FLC_Handle_t *flc);

#endif /* FEDERATEDCLIENT_H */
