/**
 * @file    NeuralNetwork.h
 * @brief   Lightweight 3-layer feedforward neural network for STM32F405
 *
 * Architecture:
 *   Input (500) → Dense+ReLU (16) → Dense+Softmax (3)
 *
 * Implementation strategy:
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │  All weight/activation/gradient matrices are statically allocated.  │
 *  │  Matrices are stored in row-major flattened arrays.                 │
 *  │  No external ML framework dependency.                               │
 *  │  FPU-accelerated float32 arithmetic throughout.                     │
 *  └─────────────────────────────────────────────────────────────────────┘
 *
 * Memory layout (approximate):
 *   W1   [500×16]  = 8000 × 4 bytes = 32000 bytes
 *   b1   [16]      =   16 × 4 bytes =    64 bytes
 *   W2   [16×3]    =   48 × 4 bytes =   192 bytes
 *   b2   [3]       =    3 × 4 bytes =    12 bytes
 *   a1   [16]      activations layer 1
 *   a2   [3]       activations layer 2 (softmax probabilities)
 *   dW1, db1, dW2, db2  gradient buffers (same sizes as weights)
 *   delta1 [16], delta2 [3]  backprop error signals
 *   TOTAL weights: ~32268 bytes = ~31.5 KB
 *
 * Note: W1 dominates RAM. At 500×16 float32 this is unavoidable without
 * quantization. STM32F405 has 192 KB SRAM — well within budget.
 *
 * Training algorithm: Mini-batch SGD (batch size = 1 for online learning)
 * Loss function: Categorical cross-entropy
 * Initialization: Xavier/Glorot uniform
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <stdint.h>
#include "Config.h"

/*
 * INDEX MACRO — row-major matrix element access
 * MAT(ptr, row, col, ncols) expands to ptr[row * ncols + col]
 */
#define MAT(ptr, row, col, ncols)  ((ptr)[(row) * (ncols) + (col)])

/* 
 * NEURAL NETWORK HANDLE
 *
 * All buffers are declared inside the struct to keep everything
 * in a single statically-allocated object (place in BSS or SRAM2).
 */

typedef struct {
    float W1[NN_INPUT_SIZE  * NN_HIDDEN_SIZE];  //< Layer 1 weights [I×H] 
    float b1[NN_HIDDEN_SIZE];                   //< Layer 1 biases  [H]   
    float W2[NN_HIDDEN_SIZE * NN_OUTPUT_SIZE];  //< Layer 2 weights [H×O]
    float b2[NN_OUTPUT_SIZE];                   //< Layer 2 biases  [O]   

    float a1[NN_HIDDEN_SIZE];    //< Post-ReLU hidden activations 
    float a2[NN_OUTPUT_SIZE];    //< Post-Softmax output probabilities 
    float z1[NN_HIDDEN_SIZE];    //< Pre-ReLU linear combination
    float z2[NN_OUTPUT_SIZE];    //< Pre-Softmax linear combination

    float dW1[NN_INPUT_SIZE  * NN_HIDDEN_SIZE]; //< Gradient w.r.t. W1 
    float db1[NN_HIDDEN_SIZE];                  //< Gradient w.r.t. b1 
    float dW2[NN_HIDDEN_SIZE * NN_OUTPUT_SIZE]; //< Gradient w.r.t. W2 
    float db2[NN_OUTPUT_SIZE];                  //< Gradient w.r.t. b2 

    float delta2[NN_OUTPUT_SIZE];  //< Output layer error delta 
    float delta1[NN_HIDDEN_SIZE];  //< Hidden layer error delta 

    float learning_rate;           //< SGD step size

    float last_loss;               //< Cross-entropy loss from last forward 
    uint32_t train_step_count;     //< Total gradient steps taken 
    float loss_history[LOSS_HISTORY_LEN];  //< Rolling loss log 
    uint8_t loss_history_idx;              //< Circular index into loss_history 
} NN_Handle_t;

/**
 * @brief  Initialize the neural network: Xavier weight init, zero biases.
 *
 * Xavier uniform initialization:
 *   W1 ~ Uniform(-sqrt(6/(I+H)), +sqrt(6/(I+H)))
 *   W2 ~ Uniform(-sqrt(6/(H+O)), +sqrt(6/(H+O)))
 *   Biases initialized to zero.
 *
 * Uses Utils_RandF() seeded deterministically; call Utils_SeedRNG() first
 * if reproducible training is required.
 *
 * @param  nn   Pointer to caller-allocated NN_Handle_t
 */
void NN_Init(NN_Handle_t *nn);

/**
 * @brief  Forward propagation — compute class probabilities from input.
 *
 * Layer 1: z1 = W1 × x + b1,  a1 = ReLU(z1)
 * Layer 2: z2 = W2 × a1 + b2, a2 = Softmax(z2)
 *
 * Result is in nn->a2[0..2]: probability distribution over 3 classes.
 *
 * Complexity: O(I×H + H×O) = O(8048) FP multiplications
 * On Cortex-M4 @ 168 MHz ≈ ~50 µs per forward pass.
 *
 * @param  nn    Neural network handle
 * @param  x     Input feature vector [NN_INPUT_SIZE]
 */
void NN_Forward(NN_Handle_t *nn, const float *x);

/**
 * @brief  Backward propagation — compute gradients from one training sample.
 *
 * Uses categorical cross-entropy loss with softmax output.
 * The combined softmax + CE derivative simplifies to:
 *   delta2 = a2 - y_onehot
 *
 * Then backpropagates through W2, applies ReLU derivative, then W1.
 * Gradient clipping is applied to prevent exploding gradients.
 *
 * @param  nn     Neural network handle (forward pass must have run first)
 * @param  x      Input feature vector [NN_INPUT_SIZE] (same as forward)
 * @param  label  Ground-truth class index (0, 1, or 2)
 */
void NN_Backward(NN_Handle_t *nn, const float *x, uint8_t label);

/**
 * @brief  Apply accumulated gradients to weights via SGD.
 *
 *   W -= learning_rate × dW
 *   b -= learning_rate × db
 *
 * @param  nn  Neural network handle
 */
void NN_UpdateWeights(NN_Handle_t *nn);

/**
 * @brief  Convenience: forward + backward + weight update in one call.
 *
 * @param  nn     Neural network handle
 * @param  x      Input feature vector [NN_INPUT_SIZE]
 * @param  label  Ground-truth class index
 * @return float  Cross-entropy loss for this sample
 */
float NN_TrainStep(NN_Handle_t *nn, const float *x, uint8_t label);

/**
 * @brief  Return the predicted class index (argmax of a2).
 * @param  nn  Neural network handle (forward must have run)
 * @return uint8_t  Predicted class (0, 1, or 2)
 */
uint8_t NN_Predict(const NN_Handle_t *nn);

/**
 * @brief  Compute categorical cross-entropy: -sum(y * log(a2 + eps)).
 * @param  nn     Neural network handle (a2 must be populated)
 * @param  label  Ground-truth class index
 * @return float  Cross-entropy loss (>= 0)
 */
float NN_CrossEntropyLoss(const NN_Handle_t *nn, uint8_t label);

/**
 * @brief  Compute moving average loss over loss_history buffer.
 * @param  nn  Neural network handle
 * @return float  Average of last LOSS_HISTORY_LEN training losses
 */
float NN_MovingAvgLoss(const NN_Handle_t *nn);

/**
 * @brief  ReLU activation applied element-wise in-place.
 * @param  v  Float array
 * @param  n  Length
 */
void NN_ReLU(float *v, uint32_t n);

/**
 * @brief  Softmax activation applied element-wise in-place.
 *         Uses numerically stable max-subtraction before exp.
 * @param  v  Float array (modified in-place)
 * @param  n  Length
 */
void NN_Softmax(float *v, uint32_t n);

/**
 * @brief  Print weight statistics (min, max, mean) to UART for debugging.
 * @param  nn  Neural network handle
 */
void NN_PrintWeightStats(const NN_Handle_t *nn);

#endif /* NEURALNETWORK_H */
