/**
 * @file    NeuralNetwork.c
 * @brief   3-layer feedforward neural network with backpropagation — pure C
 *
 * Mathematical formulation:
 *
 *  Forward pass:
 *    z1 = W1 · x + b1            [H-vector, H=16]
 *    a1 = ReLU(z1)               [H-vector]
 *    z2 = W2 · a1 + b2           [O-vector, O=3]
 *    a2 = Softmax(z2)            [O-vector, probability simplex]
 *    L  = -Σ y_k log(a2_k)      [scalar, cross-entropy]
 *
 *  Backward pass (combined Softmax + CE derivative):
 *    δ2 = a2 - y_onehot          [O-vector, clean gradient]
 *    dW2 = a1ᵀ · δ2              [H×O matrix]
 *    db2 = δ2
 *    δ1 = (W2 · δ2) ⊙ ReLU'(z1) [H-vector, ⊙ = element-wise]
 *    dW1 = xᵀ · δ1               [I×H matrix]
 *    db1 = δ1
 *
 *  Weight update (SGD):
 *    W  ← W - η × dW
 *    b  ← b - η × db
 *
 * Xavier uniform initialization:
 *    W ~ Uniform(-limit, +limit)
 *    limit = sqrt(6 / (fan_in + fan_out))
 *
 * Numerical stability measures:
 *    - Softmax uses max-subtraction before exp (prevents overflow)
 *    - CE loss uses log(a2 + ε) to prevent log(0)
 *    - Gradient clipping to NN_GRAD_CLIP prevents exploding gradients
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#include "NeuralNetwork.h"
#include "Utils.h"
#include <string.h>
#include <math.h>

#define CE_EPS  1e-7f

/**
 * @brief  ReLU: max(0, x) applied element-wise in-place.
 *
 * On Cortex-M4 with FPU, VMAXNM.F32 handles this in one instruction
 * per element when compiled with -O2 and appropriate FPCSR flags.
 */
void NN_ReLU(float *v, uint32_t n)
{
    for (uint32_t i = 0U; i < n; i++) {
        v[i] = (v[i] > 0.0f) ? v[i] : 0.0f;
    }
}

/**
 * @brief  Numerically stable Softmax applied in-place.
 *
 * Steps:
 *  1. Subtract max value to prevent exp() overflow:
 *     v[i] -= max(v)
 *  2. Compute exp(v[i]) for each element
 *  3. Normalize by sum to get probabilities
 *
 * After softmax: all v[i] ∈ (0, 1), sum(v) = 1.0
 */
void NN_Softmax(float *v, uint32_t n)
{
    float max_val = v[0];
    for (uint32_t i = 1U; i < n; i++) {
        if (v[i] > max_val) max_val = v[i];
    }

    float sum_exp = 0.0f;
    for (uint32_t i = 0U; i < n; i++) {
        v[i] = expf(v[i] - max_val);
        sum_exp += v[i];
    }

    float inv_sum = 1.0f / (sum_exp + CE_EPS);
    for (uint32_t i = 0U; i < n; i++) {
        v[i] *= inv_sum;
    }
}


/**
 * @brief  Xavier uniform weight initialization.
 *
 * Xavier uniform limit for layer with fan_in and fan_out:
 *   limit = sqrt(6 / (fan_in + fan_out))
 *
 * Weights are sampled from Uniform(-limit, +limit) using Utils_RandF()
 * which returns values in [-1, +1], so we scale by limit.
 *
 * Biases are initialized to zero (standard practice for feedforward nets).
 *
 * Counter is zeroed; loss history is cleared.
 */
void NN_Init(NN_Handle_t *nn)
{
    nn->learning_rate   = NN_LEARNING_RATE;
    nn->last_loss       = 0.0f;
    nn->train_step_count = 0U;
    nn->loss_history_idx = 0U;

    memset(nn->loss_history, 0, sizeof(nn->loss_history));
    memset(nn->b1, 0, sizeof(nn->b1));
    memset(nn->b2, 0, sizeof(nn->b2));

    // Xavier uniform for W1: fan_in=500, fan_out=16
    float limit_w1 = sqrtf(6.0f / (float)(NN_INPUT_SIZE + NN_HIDDEN_SIZE));
    for (uint32_t i = 0U; i < NN_INPUT_SIZE * NN_HIDDEN_SIZE; i++) {
        nn->W1[i] = Utils_RandF() * limit_w1;
    }

    // Xavier uniform for W2: fan_in=16, fan_out=3 
    float limit_w2 = sqrtf(6.0f / (float)(NN_HIDDEN_SIZE + NN_OUTPUT_SIZE));
    for (uint32_t i = 0U; i < NN_HIDDEN_SIZE * NN_OUTPUT_SIZE; i++) {
        nn->W2[i] = Utils_RandF() * limit_w2;
    }

    LOG_INF("NN init: I=%u H=%u O=%u, W1_limit=%.4f, W2_limit=%.4f",
            NN_INPUT_SIZE, NN_HIDDEN_SIZE, NN_OUTPUT_SIZE,
            (double)limit_w1, (double)limit_w2);
}

/**
 * @brief  Forward pass through the two-layer network.
 *
 * Layer 1 GEMV (General Matrix-Vector multiply):
 *   z1[j] = Σ_i W1[j][i] × x[i] + b1[j]    for j in [0, H)
 *
 * Using row-major storage: W1[j*I + i] = W1[j][i]
 * The inner loop over I=500 is the hot path — compiler autovectorizes
 * with FPU VFMA.F32 pairs when compiled at -O2.
 *
 * Layer 2 GEMV:
 *   z2[k] = Σ_j W2[k][j] × a1[j] + b2[k]    for k in [0, O)
 *
 * This is only H×O = 48 operations — trivial.
 */
void NN_Forward(NN_Handle_t *nn, const float *x)
{
    uint32_t i, j, k;

    // Layer 1: z1 = W1 × x + b1, a1 = ReLU(z1)
    for (j = 0U; j < NN_HIDDEN_SIZE; j++) {
        float acc = nn->b1[j];
        const float *w1_row = &nn->W1[j * NN_INPUT_SIZE];
        // Critical inner loop: I=500 multiply-accumulate operations
        for (i = 0U; i < NN_INPUT_SIZE; i++) {
            acc += w1_row[i] * x[i];
        }
        nn->z1[j] = acc;
        nn->a1[j] = (acc > 0.0f) ? acc : 0.0f; 
    }

    // Layer 2: z2 = W2 × a1 + b2, a2 = Softmax(z2)
    for (k = 0U; k < NN_OUTPUT_SIZE; k++) {
        float acc = nn->b2[k];
        const float *w2_row = &nn->W2[k * NN_HIDDEN_SIZE];
        for (j = 0U; j < NN_HIDDEN_SIZE; j++) {
            acc += w2_row[j] * nn->a1[j];
        }
        nn->z2[k] = acc;
        nn->a2[k] = acc;   /* Softmax applied below */
    }

    NN_Softmax(nn->a2, NN_OUTPUT_SIZE);
}

/**
 * @brief  Categorical cross-entropy: L = -log(a2[label] + ε)
 *
 * For one-hot targets this reduces to the log probability of the true class.
 * The epsilon prevents log(0) in the (rare) case of saturated softmax.
 */
float NN_CrossEntropyLoss(const NN_Handle_t *nn, uint8_t label)
{
    float p = nn->a2[label];
    if (p < CE_EPS) p = CE_EPS;
    return -logf(p);
}


/**
 * @brief  Backpropagate gradients through both layers.
 *
 * Step 1 — Output layer delta (softmax + CE combined derivative):
 *   δ2[k] = a2[k] - 1_{k == label}
 *   This elegant result comes from d/dz2 of CE(Softmax(z2)):
 *   The Jacobian of softmax combines with CE derivative to give this simple form.
 *
 * Step 2 — W2 and b2 gradients (outer product):
 *   dW2[k][j] = a1[j] × δ2[k]
 *   db2[k]    = δ2[k]
 *
 * Step 3 — Hidden layer delta (backprop through W2, then ReLU gate):
 *   δ1_raw[j] = Σ_k W2[k][j] × δ2[k]      (backprop through W2)
 *   δ1[j]     = δ1_raw[j] × (z1[j] > 0)   (ReLU derivative: 1 if z1>0 else 0)
 *
 * Step 4 — W1 and b1 gradients:
 *   dW1[j][i] = x[i] × δ1[j]
 *   db1[j]    = δ1[j]
 *
 * Step 5 — Gradient clipping on dW1 (the largest gradient tensor):
 *   clip dW1 elements to [-NN_GRAD_CLIP, +NN_GRAD_CLIP]
 */
void NN_Backward(NN_Handle_t *nn, const float *x, uint8_t label)
{
    uint32_t i, j, k;

    // Step 1: Output layer delta
    for (k = 0U; k < NN_OUTPUT_SIZE; k++) {
        nn->delta2[k] = nn->a2[k];
    }
    nn->delta2[label] -= 1.0f;   

    // Step 2: W2 and b2 gradients
    for (k = 0U; k < NN_OUTPUT_SIZE; k++) {
        nn->db2[k] = nn->delta2[k];
        float *dw2_row = &nn->dW2[k * NN_HIDDEN_SIZE];
        for (j = 0U; j < NN_HIDDEN_SIZE; j++) {
            dw2_row[j] = nn->a1[j] * nn->delta2[k];
        }
    }

    // Step 3: Hidden layer delta
    for (j = 0U; j < NN_HIDDEN_SIZE; j++) {
        float d = 0.0f;
        for (k = 0U; k < NN_OUTPUT_SIZE; k++) {
            //W2 is stored row-major: W2[k][j] = W2[k * H + j]
            d += nn->W2[k * NN_HIDDEN_SIZE + j] * nn->delta2[k];
        }
        // ReLU derivative: gate by pre-activation z1 
        nn->delta1[j] = (nn->z1[j] > 0.0f) ? d : 0.0f;
    }

    // Step 4: W1 and b1 gradients
    for (j = 0U; j < NN_HIDDEN_SIZE; j++) {
        nn->db1[j] = nn->delta1[j];
        float *dw1_row = &nn->dW1[j * NN_INPUT_SIZE];
        float  d1j     = nn->delta1[j];
        // Critical inner loop: I=500 multiply-store operations
        for (i = 0U; i < NN_INPUT_SIZE; i++) {
            dw1_row[i] = x[i] * d1j;
        }
    }

    // Step 5: Gradient clipping on dW1 (the large gradient tensor)
    Utils_VecClip(nn->dW1, NN_INPUT_SIZE * NN_HIDDEN_SIZE, NN_GRAD_CLIP);
}

/**
 * @brief  Apply SGD weight updates: W -= lr × dW, b -= lr × db
 *
 * The learning rate is applied element-wise across all four parameter tensors.
 * For regularization (L2 weight decay), add: W[i] -= lambda × W[i] here.
 */
void NN_UpdateWeights(NN_Handle_t *nn)
{
    uint32_t i;
    float lr = nn->learning_rate;

    for (i = 0U; i < NN_INPUT_SIZE * NN_HIDDEN_SIZE; i++) {
        nn->W1[i] -= lr * nn->dW1[i];
    }

    for (i = 0U; i < NN_HIDDEN_SIZE; i++) {
        nn->b1[i] -= lr * nn->db1[i];
    }

    for (i = 0U; i < NN_HIDDEN_SIZE * NN_OUTPUT_SIZE; i++) {
        nn->W2[i] -= lr * nn->dW2[i];
    }

    for (i = 0U; i < NN_OUTPUT_SIZE; i++) {
        nn->b2[i] -= lr * nn->db2[i];
    }

    nn->train_step_count++;
}

/**
 * @brief  Forward → Backward → Update in one call.  Returns loss.
 */
float NN_TrainStep(NN_Handle_t *nn, const float *x, uint8_t label)
{
    NN_Forward(nn, x);
    float loss = NN_CrossEntropyLoss(nn, label);

    nn->loss_history[nn->loss_history_idx] = loss;
    nn->loss_history_idx = (uint8_t)((nn->loss_history_idx + 1U) % LOSS_HISTORY_LEN);
    nn->last_loss = loss;

    NN_Backward(nn, x, label);
    NN_UpdateWeights(nn);

    return loss;
}

/**
 * @brief  argmax(a2) — returns predicted class index.
 */
uint8_t NN_Predict(const NN_Handle_t *nn)
{
    uint8_t best_idx = 0U;
    float   best_val = nn->a2[0];
    for (uint8_t k = 1U; k < NN_OUTPUT_SIZE; k++) {
        if (nn->a2[k] > best_val) {
            best_val = nn->a2[k];
            best_idx = k;
        }
    }
    return best_idx;
}

float NN_MovingAvgLoss(const NN_Handle_t *nn)
{
    float sum = 0.0f;
    for (uint8_t i = 0U; i < LOSS_HISTORY_LEN; i++) {
        sum += nn->loss_history[i];
    }
    return sum / (float)LOSS_HISTORY_LEN;
}

void NN_PrintWeightStats(const NN_Handle_t *nn)
{
    float min_w1 = nn->W1[0], max_w1 = nn->W1[0], sum_w1 = 0.0f;
    for (uint32_t i = 0U; i < NN_INPUT_SIZE * NN_HIDDEN_SIZE; i++) {
        float w = nn->W1[i];
        if (w < min_w1) min_w1 = w;
        if (w > max_w1) max_w1 = w;
        sum_w1 += w;
    }
    float mean_w1 = sum_w1 / (float)(NN_INPUT_SIZE * NN_HIDDEN_SIZE);

    LOG_INF("NN WeightStats W1: min=%.4f max=%.4f mean=%.4f",
            (double)min_w1, (double)max_w1, (double)mean_w1);
    LOG_INF("NN step=%lu, last_loss=%.4f, avg_loss=%.4f",
            nn->train_step_count, (double)nn->last_loss,
            (double)NN_MovingAvgLoss(nn));
    LOG_INF("NN output: P(normal)=%.3f P(imbal)=%.3f P(loose)=%.3f",
            (double)nn->a2[0], (double)nn->a2[1], (double)nn->a2[2]);
}
