/**
 * @file    main.c
 * @brief   FedVibroSense — STM32F405 Federated Learning Vibration Client
 *
 * Application entry point and main control loop.
 *
 * Hardware:
 *   MCU:    STM32F405RGT6 @ 168 MHz, Cortex-M4F
 *   Sensor: MPU-6050 on I2C1 (PB8=SCL, PB9=SDA)
 *   Debug:  UART2 (PA2=TX, PA3=RX) @ 115200 baud
 *   FL:     UART1 (PA9=TX, PA10=RX) @ 921600 baud
 *   Label:  PC13 button (active low) cycles class: normal→imbalance→looseness
 *   Status: PA4/5/6 LEDs = class indicator, PA7 = training indicator
 *
 * Main loop execution (100 Hz):
 *
 *  Every 10 ms (100 Hz interrupt via TIM3):
 *    1. Read MPU-6050 (burst I2C)
 *    2. Update complementary filter → CF_Output_t
 *    3. Push to feature ring buffer
 *
 *  Every 1 s (when ring buffer has FEATURE_WINDOW_SAMPLES):
 *    4. Build 500-float feature vector (with Z-score normalization)
 *    5. Run NN inference → predicted class
 *    6. If labeled mode: submit (feature, label) to FL client
 *    7. Run FLC_Tick() to advance FL FSM
 *
 *  UART label injection:
 *    Receive single byte: '0'=normal, '1'=imbalance, '2'=looseness, 'u'=upload
 *
 * =========================================================================
 * CUBEMX-GENERATED FILES (DO NOT EDIT MANUALLY):
 *   Core/Src/stm32f4xx_hal_msp.c    — MSP init callbacks
 *   Core/Src/stm32f4xx_it.c         — IRQ handlers
 *   Core/Src/system_stm32f4xx.c     — SystemClock_Config stub
 *   Core/Inc/stm32f4xx_hal_conf.h   — HAL module enables
 *   Core/Inc/stm32f4xx_it.h
 *   Core/Inc/main.h                 — HAL handle externs
 *   Drivers/STM32F4xx_HAL_Driver/   — HAL library (full)
 *   Drivers/CMSIS/                  — CMSIS core headers
 *
 * MANUALLY CREATED FILES:
 *   Core/Src/main.c                 — THIS FILE
 *   Core/Src/MPU6050.c
 *   Core/Src/ComplementaryFilter.c
 *   Core/Src/FeatureExtractor.c
 *   Core/Src/NeuralNetwork.c
 *   Core/Src/Serialization.c
 *   Core/Src/FederatedClient.c
 *   Core/Src/Utils.c
 *   Core/Inc/  (all headers above)
 *
 * @author  FedVibroSense Project
 * @version 1.0.0
 */

//#include "main.h"
#include "stm32f4xx_hal.h"

/* Custom module headers */
#include "Config.h"
#include "Utils.h"
#include "MPU6050.h"
#include "ComplementaryFilter.h"
#include "FeatureExtractor.h"
#include "NeuralNetwork.h"
#include "FederatedClient.h"
#include "Serialization.h"

#include <stdio.h>
#include <string.h>


I2C_HandleTypeDef  hi2c1;     /**< MPU-6050 I2C bus */
UART_HandleTypeDef huart1;    /**< FL communication UART (921600 baud) */
UART_HandleTypeDef huart2;    /**< Debug UART (115200 baud) */
TIM_HandleTypeDef  htim2;     /**< Free-running microsecond timer (32-bit) */
TIM_HandleTypeDef  htim3;     /**< 100 Hz sampling timer (period interrupt) */

/** IMU driver handle */
static MPU6050_Handle_t   s_mpu;

/** Complementary filter state */
static CF_State_t         s_cf_state;

/** Feature extractor (ring buffer + window state) */
static FE_State_t         s_fe;

/**
 * Neural network handle.
 * SRAM2 placement keeps the large W1 array (32 KB) out of main SRAM1
 * to reduce banking conflicts.
 * Uncomment __attribute__((section(".sram2"))) if your linker script
 * defines a .sram2 region for 0x10000000 (64 KB CCM on F405).
 *
 * CCM (Core Coupled Memory, 0x10000000, 64 KB) on STM32F405:
 *   - Zero-wait-state access from Cortex-M4
 *   - Ideal for the hot neural network weight matrix W1
 *   - NOT accessible by DMA (fine here, no DMA used for NN)
 */
/* __attribute__((section(".ccmram"))) */
static NN_Handle_t        s_nn;

/** Federated learning client */
static FLC_Handle_t       s_flc;

/** Scaled IMU sample output */
static MPU6050_Data_t     s_imu_data;

/** Complementary filter output per sample */
static CF_Output_t        s_cf_out;

/** Feature vector (500 floats, ~2 KB on stack is too large — static) */
static float              s_feature_vec[FEATURE_VECTOR_SIZE];

/** Set by TIM3 ISR at 100 Hz — signals time to read IMU */
volatile uint8_t g_sample_flag = 0U;

/** UART single-byte label command received */
volatile uint8_t g_uart_rx_byte = 0U;
volatile uint8_t g_uart_rx_ready = 0U;

static uint8_t s_current_label = CLASS_NORMAL;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

static void App_Init(void);
static void App_HandleUARTCommand(uint8_t cmd);
static void App_UpdateLEDs(uint8_t class_idx, uint8_t training);


int main(void)
{
    /* === EARLY DEBUG: Toggle all LEDs to show main() was entered === */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_Init = {0};
    GPIO_Init.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Init);
    
    /* Flash all LEDs to show main() started */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
    for (volatile int i = 0; i < 1000000; i++);  /* Busy wait ~1ms */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    
    /* --- 1. HAL initialization (sets SysTick at 1 kHz) ------------------ */
    HAL_Init();

    /* --- 2. Configure system clocks (168 MHz HSE PLL) ------------------- */
    SystemClock_Config();

    /* --- 3. Initialize peripherals -------------------------------------- */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();    /* Debug UART — init first for early logging */
    MX_USART1_UART_Init();    /* FL comm UART */
    MX_TIM2_Init();           /* Microsecond timer */
    MX_TIM3_Init();           /* 100 Hz sampling timer */

    /* --- 4. Application-level initialization ---------------------------- */
    App_Init();

    /* --- 5. Enable UART receive interrupt for label commands ------------ */
    HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U);

    /* --- 6. Test GPIO and UART to diagnose connectivity ---- */
    /* Toggle LED PD12 to show the board is running */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   /* LED on */
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); /* LED off */
    HAL_Delay(100);
    
    /* Try to send test message with error checking */
    const char *test_msg = "\r\n[TEST] UART TEST MESSAGE\r\n";
    HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, 
                                                       strlen(test_msg), 500);
    /* If UART failed, try again once more */
    if (uart_status != HAL_OK) {
        HAL_Delay(50);
        HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, strlen(test_msg), 500);
    }

    LOG_INF("=== FedVibroSense STM32F405 FL Client Started ===");
    LOG_INF("FEATURE_VECTOR_SIZE=%u, NN=%ux%ux%u, FL_LOCAL_EPOCHS=%u",
            FEATURE_VECTOR_SIZE, NN_INPUT_SIZE, NN_HIDDEN_SIZE, NN_OUTPUT_SIZE,
            FL_LOCAL_EPOCHS);
    Utils_PrintMemoryStats();

    /*
     * MAIN LOOP
     *
     * Structure:
     *   [100 Hz]  IMU read → CF update → ring push → (optional) inference
     *   [1 Hz]    Feature vector build → FL client tick
     *   [async]   UART command handling for labels and FL trigger
    */

    uint32_t window_count       = 0UL;  /* Number of complete 1-s windows */
    uint32_t last_window_ms     = 0UL;  /* Timestamp of last feature build */
    uint8_t  inference_pending  = 0U;   /* Flag: new feature vector ready */

    while (1) {

        /* --- 100 Hz: IMU sampling (flag set by TIM3 ISR) --------------- */
        if (g_sample_flag) {
            g_sample_flag = 0U;

            uint32_t t_start = Utils_GetMicros();

            /* Read IMU — if I2C fails, skip this sample */
            MPU6050_Status_t imu_ret = MPU6050_ReadScaled(&s_mpu, &s_imu_data);
            if (imu_ret != MPU6050_OK) {
                LOG_ERR("IMU read fail at sample_cnt=%lu", window_count);
                continue;
            }

            /* Update complementary filter */
            CF_Update(&s_cf_state, &s_imu_data, &s_cf_out);

            /* Push filter output to ring buffer */
            FE_Push(&s_fe, &s_cf_out);

            uint32_t t_imu_us = Utils_ElapsedMicros(t_start);
            LOG_VRB("IMU+CF+Push: %lu µs | pitch=%.2f roll=%.2f",
                    t_imu_us,
                    (double)s_cf_out.pitch,
                    (double)s_cf_out.roll);
        }

        /* --- Feature vector build (once per second) -------------------- */
        uint32_t now_ms = Utils_GetMillis();
        if (FE_IsWindowReady(&s_fe) &&
            (now_ms - last_window_ms >= FEATURE_WINDOW_SECONDS * 1000U)) {

            last_window_ms = now_ms;
            uint32_t t_feat = Utils_GetMicros();

            /* Build 500-float normalized feature vector */
            uint8_t ok = FE_BuildFeatureVector(&s_fe, s_feature_vec);
            if (!ok) {
                LOG_ERR("Feature build failed");
                continue;
            }
            uint32_t t_feat_us = Utils_ElapsedMicros(t_feat);

            /* Run inference on the new feature vector */
            uint32_t t_infer = Utils_GetMicros();
            NN_Forward(&s_nn, s_feature_vec);
            uint8_t pred = NN_Predict(&s_nn);
            uint32_t t_infer_us = Utils_ElapsedMicros(t_infer);

            window_count++;
            inference_pending = 1U;

            LOG_INF("Window #%lu | Pred=%u [N=%.3f I=%.3f L=%.3f] | "
                    "feat=%lu µs infer=%lu µs",
                    window_count, pred,
                    (double)s_nn.a2[CLASS_NORMAL],
                    (double)s_nn.a2[CLASS_IMBALANCE],
                    (double)s_nn.a2[CLASS_LOOSENESS],
                    t_feat_us, t_infer_us);

            /* Update LEDs with predicted class */
            App_UpdateLEDs(pred, 0U);

            /* Submit to FL client if in labeled mode -------------------- */
            if (s_flc.state == FLC_STATE_IDLE ||
                s_flc.state == FLC_STATE_COLLECTING) {

                uint8_t submitted = FLC_SubmitSample(
                    &s_flc, s_feature_vec, s_current_label);

                if (submitted) {
                    LOG_INF("FL: submitted window #%lu with label=%u "
                            "(%u/%u buffered)",
                            window_count, s_current_label,
                            s_flc.train_buf.count, FL_LOCAL_EPOCHS);
                }
            }
        }

        /* --- Federated Learning FSM tick ------------------------------- */
        if (s_flc.state == FLC_STATE_TRAINING ||
            s_flc.state == FLC_STATE_UPLOADING ||
            s_flc.state == FLC_STATE_DOWNLOADING ||
            s_flc.state == FLC_STATE_ERROR) {

            App_UpdateLEDs(s_current_label, 1U);   /* Training LED on */
            FLC_Tick(&s_flc);
            App_UpdateLEDs(s_current_label, 0U);
        }

        /* --- UART command handling -------------------------------------- */
        if (g_uart_rx_ready) {
            g_uart_rx_ready = 0U;
            App_HandleUARTCommand(g_uart_rx_byte);
            /* Re-arm UART interrupt for next byte */
            HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U);
        }
    }
    /* unreachable */
    return 0;
}

static void App_Init(void)
{
    /* Start microsecond timer */
    Utils_TimerInit();

    /* Start 100 Hz TIM3 interrupt */
    HAL_TIM_Base_Start_IT(&htim3);

    /* Seed PRNG deterministically for reproducible Xavier initialization */
    Utils_SeedRNG(0xABCD1234UL);

    /* Initialize neural network weights */
    NN_Init(&s_nn);

    /* Initialize FL client */
    FLC_Init(&s_flc, &s_nn, &huart1);

    /* Initialize complementary filter */
    CF_Init(&s_cf_state);

    /* Initialize feature extractor */
    FE_Init(&s_fe);

    /* Initialize MPU-6050 */
    MPU6050_Status_t mpu_ret = MPU6050_Init(&s_mpu, &hi2c1);
    if (mpu_ret != MPU6050_OK) {
        LOG_ERR("MPU6050 init FAILED (code %u) — check I2C wiring", mpu_ret);
        /* Blink error LED and halt */
        while (1) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);   /* Red LED */
            HAL_Delay(100U);
        }
    }

    /* Skip blocking gyro calibration for now so the main loop can run */
    LOG_INF("MPU6050 calibration skipped (temporary)");
    LOG_INF("App_Init complete. Waiting for IMU data...");
}

/* 
 * UART COMMAND HANDLER
 *
 * Commands (single ASCII byte via debug UART):
 *   '0' — set label = CLASS_NORMAL
 *   '1' — set label = CLASS_IMBALANCE
 *   '2' — set label = CLASS_LOOSENESS
 *   'u' — trigger immediate FL upload
 *   'r' — reset FL client
 *   's' — print status
 *   'w' — print NN weight statistics
 */

static void App_HandleUARTCommand(uint8_t cmd)
{
    switch (cmd) {
        case '0':
            s_current_label = CLASS_NORMAL;
            LOG_INF("Label set → NORMAL");
            break;
        case '1':
            s_current_label = CLASS_IMBALANCE;
            LOG_INF("Label set → IMBALANCE");
            break;
        case '2':
            s_current_label = CLASS_LOOSENESS;
            LOG_INF("Label set → LOOSENESS");
            break;
        case 'u':
            FLC_TriggerUpload(&s_flc);
            break;
        case 'r':
            FLC_Reset(&s_flc);
            break;
        case 's':
            FLC_PrintStatus(&s_flc);
            break;
        case 'w':
            NN_PrintWeightStats(&s_nn);
            break;
        default:
            LOG_VRB("Unknown cmd: 0x%02X", cmd);
            break;
    }
}

/* 
 * LED STATUS OUTPUT
 *
 * STM32F405 Discovery LEDs on PA4/5/6/7:
 *   PA4 Green  = CLASS_NORMAL
 *   PA5 Orange = CLASS_IMBALANCE
 *   PA6 Red    = CLASS_LOOSENESS
 *   PA7 Blue   = Training/FL active
 */

static void App_UpdateLEDs(uint8_t class_idx, uint8_t training)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (class_idx == CLASS_NORMAL)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (class_idx == CLASS_IMBALANCE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (class_idx == CLASS_LOOSENESS) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, training ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  TIM3 Period Elapsed callback — fires at 100 Hz.
 *         Sets g_sample_flag for the main loop.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        g_sample_flag = 1U;
    }
}

/**
 * @brief  UART RX Complete callback — single byte received.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        g_uart_rx_ready = 1U;
        /* Do NOT re-arm here — main loop does it after processing */
    }
}

/**
 * @brief  MPU-6050 data-ready EXTI callback (PA0 pin).
 *         Delegates to driver ISR handler.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        MPU6050_DataReadyISR(&s_mpu);
    }
}

/* 
 * PERIPHERAL INITIALIZATION FUNCTIONS
 *
 * These would normally be auto-generated by STM32CubeMX.
 * They are provided here for completeness and terminal builds.
 * In a CubeMX project, delete these and rely on the generated versions.
 *  */

/**
 * @brief  System Clock Configuration.
 *
 * Target: 168 MHz from 8 MHz HSE using PLL.
 *
 * PLL config:
 *   PLLM = 8    → VCO input = 8 MHz / 8 = 1 MHz
 *   PLLN = 336  → VCO output = 1 MHz × 336 = 336 MHz
 *   PLLP = 2    → System clock = 336 / 2 = 168 MHz
 *   PLLQ = 7    → USB/SDIO/RNG clock = 336 / 7 = 48 MHz
 *
 * Bus clocks:
 *   AHB  prescaler = 1  → HCLK = 168 MHz
 *   APB1 prescaler = 4  → PCLK1 = 42 MHz (timer clocks = 84 MHz)
 *   APB2 prescaler = 2  → PCLK2 = 84 MHz (timer clocks = 168 MHz)
 *
 * Flash latency: 5 wait states (required for 168 MHz, 3.3 V)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable power controller clock and set voltage scaling */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE oscillator and PLL */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 8U;
    RCC_OscInitStruct.PLL.PLLN            = 336U;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 7U;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select PLL as system clock, configure bus prescalers */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000U;   /* Fast mode: 400 kHz */
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0U;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = UART_FL_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = UART_DEBUG_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_TIM2_Init(void)
{
    /* TIM2: 32-bit free-running microsecond timer
     * APB1 timer clock = 84 MHz (2× PCLK1 when APB1 prescaler > 1)
     * Prescaler = 83 → timer ticks at 84 MHz / 84 = 1 MHz = 1 µs/tick */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83U;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFUL;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
}

static void MX_TIM3_Init(void)
{
    /* TIM3: 100 Hz periodic interrupt
     * APB1 timer clock = 84 MHz
     * Prescaler = 839, Period = 999:
     *   Freq = 84 MHz / (840 × 1000) = 100 Hz exactly */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 839U;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999U;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);
    /* NVIC: TIM3 at preemption priority 1 (below SysTick=0, above UART) */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD12/13/14/15 — LEDs (output push-pull) */
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    /* Start with all LEDs off */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
                      GPIO_PIN_RESET);

    /* PA0 — MPU-6050 INT pin (EXTI, falling edge, active low) */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /* PC13 — User button (label cycle, active low on Nucleo) */
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Note: I2C1 (PB8/PB9) and UART1/2 GPIO alt-function init
     * is handled by HAL_I2C_MspInit() and HAL_UART_MspInit()
     * in stm32f4xx_hal_msp.c (auto-generated by CubeMX) */
}
