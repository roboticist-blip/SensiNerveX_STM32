/**
 * @file    main.c
 * @brief   FedVibroSense — STM32F405 Federated Learning Vibration Client
 *
 * Application entry point and main control loop.
 *
 * Hardware:
 *   MCU:    STM32F405RGT6 @ 168 MHz, Cortex-M4F (WeAct Studio core board)
 *   Sensor: MPU-6050 on I2C1 (PB8=SCL, PB9=SDA)
 *   Debug:  UART2 (PA2=TX, PA3=RX) @ 115200 baud
 *   FL:     UART1 (PA9=TX, PA10=RX) @ 921600 baud
 *   SD:     SDIO 4-bit (PC8-11=D0-D3, PC12=CK, PD2=CMD) — see SDCard.h
 *   Label:  PC13 button (active low) cycles class: normal→imbalance→looseness
 *   Status: PA4/5/6 LEDs = class indicator, PA7 = training indicator
 *
 * Main loop architecture:
 *
 *   main() no longer hand-sequences each subsystem. It builds a table of
 *   AppModule_t entries in App_Init() (see AppModule.h) and the loop body
 *   is a single AppModule_TickAll() call. Registration order fixes the
 *   tick order, which mirrors the original hand-written sequence:
 *
 *     [100 Hz]  IMUSample     — MPU-6050 read → complementary filter → ring push
 *     [1 Hz]    FeatureWindow — feature vector build → NN inference →
 *                               SD log enqueue → FL sample submit
 *               FLClient      — FL FSM tick (no-op outside TRAINING/
 *                               UPLOADING/DOWNLOADING/ERROR) → SD round log
 *               DataLogger    — drains at most one queued SD row per tick
 *     [async]   UARTCommand   — label/FL/log commands from the debug UART
 *
 *   Adding a new subsystem (another sensor, a second storage backend, a
 *   wireless transport, ...) means writing one Init/Tick pair and one
 *   AppModule_Register() call in App_Init() — this loop body is done
 *   growing. See Config.h "SCALABILITY / MODULE FRAMEWORK".
 *
 *  UART label injection:
 *    Receive single byte: '0'=normal, '1'=imbalance, '2'=looseness, 'u'=upload,
 *    'l'=SD log stats, 'f'=force SD flush.
 * 
 * @author  FedVibroSense / SensiNerveX Project
 * @version 2.0.0
 */

#include "main.h"
#include "stm32f4xx_hal.h"

#include "Config.h"
#include "Utils.h"
#include "MPU6050.h"
#include "ComplementaryFilter.h"
#include "FeatureExtractor.h"
#include "NeuralNetwork.h"
#include "FederatedClient.h"
#include "Serialization.h"
#include "AppModule.h"
#include "DataLogger.h"

#include <stdio.h>
#include <string.h>


I2C_HandleTypeDef  hi2c1;     
UART_HandleTypeDef huart1;   
UART_HandleTypeDef huart2;    
TIM_HandleTypeDef  htim2;     
TIM_HandleTypeDef  htim3;     

static MPU6050_Handle_t   s_mpu;

static CF_State_t         s_cf_state;

static FE_State_t         s_fe;

static NN_Handle_t        s_nn;

static FLC_Handle_t       s_flc;

static MPU6050_Data_t     s_imu_data;

static CF_Output_t        s_cf_out;

static float              s_feature_vec[FEATURE_VECTOR_SIZE];

volatile uint8_t g_sample_flag = 0U;

volatile uint8_t g_uart_rx_byte = 0U;
volatile uint8_t g_uart_rx_ready = 0U;

static uint8_t s_current_label = CLASS_NORMAL;


static uint32_t s_window_count      = 0UL; 
static uint32_t s_imu_sample_count  = 0UL; 

static uint8_t s_latest_pred        = CLASS_NORMAL;
static uint32_t s_last_window_ms    = 0UL; 

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

static void Mod_ImuSample_Tick(void);
static void Mod_FeatureWindow_Tick(void);
static void Mod_FLC_Tick(void);
static void Mod_UartCommand_Tick(void);

static const AppModule_t s_mod_imu     = { .name = "IMUSample",     .init = NULL, .tick = Mod_ImuSample_Tick };
static const AppModule_t s_mod_feature = { .name = "FeatureWindow", .init = NULL, .tick = Mod_FeatureWindow_Tick };
static const AppModule_t s_mod_flc     = { .name = "FLClient",      .init = NULL, .tick = Mod_FLC_Tick };
static const AppModule_t s_mod_uart    = { .name = "UARTCommand",   .init = NULL, .tick = Mod_UartCommand_Tick };


int main(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_Init = {0};
    GPIO_Init.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Init);
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
    for (volatile int i = 0; i < 1000000; i++);  
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();    
    MX_USART1_UART_Init();   
    MX_TIM2_Init();           
    MX_TIM3_Init();           

    App_Init();

    HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
    HAL_Delay(100);
    
    const char *test_msg = "\r\n[TEST] UART TEST MESSAGE\r\n";
    HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, 
                                                       strlen(test_msg), 500);
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
     * MAIN LOOP — now driven entirely by the AppModule table built in
     * App_Init(). Each iteration is one AppModule_TickAll() call, which
     * runs every registered module's Tick() in registration order:
     *
     *   [100 Hz]  IMUSample     — IMU read → CF update → ring push
     *   [1 Hz]    FeatureWindow — feature vector build → inference → FL submit
     *             FLClient      — FL FSM tick (only does work in non-idle states)
     *             DataLogger    — drains at most one queued SD row per call
     *   [async]   UARTCommand   — label/FL/log commands from the debug UART
     *
     * See AppModule.h for why this replaced the previous hand-written
     * sequence, and Config.h "SCALABILITY / MODULE FRAMEWORK" for how to
     * add a new subsystem without touching this loop again.
     */
    LOG_INF("App: %u modules registered, entering main loop", AppModule_Count());

    while (1) {
        AppModule_TickAll();
    }
    /* unreachable */
    return 0;
}

static void Mod_ImuSample_Tick(void)
{
    if (!g_sample_flag) {
        return;
    }
    g_sample_flag = 0U;

    uint32_t t_start = Utils_GetMicros();

    MPU6050_Status_t imu_ret = MPU6050_ReadScaled(&s_mpu, &s_imu_data);
    if (imu_ret != MPU6050_OK) {
        LOG_ERR("IMU read fail at window_cnt=%lu", s_window_count);
        return;
    }

    CF_Update(&s_cf_state, &s_imu_data, &s_cf_out);

    FE_Push(&s_fe, &s_cf_out);

    s_imu_sample_count++;

    DataLogger_LogRawIMU(s_imu_sample_count, t_start,
                         s_imu_data.ax, s_imu_data.ay, s_imu_data.az,
                         s_imu_data.gx, s_imu_data.gy, s_imu_data.gz,
                         s_current_label, s_latest_pred);

    uint32_t t_imu_us = Utils_ElapsedMicros(t_start);
    LOG_VRB("IMU+CF+Push: %lu µs | pitch=%.2f roll=%.2f",
            t_imu_us,
            (double)s_cf_out.pitch,
            (double)s_cf_out.roll);
}

static void Mod_FeatureWindow_Tick(void)
{
    uint32_t now_ms = Utils_GetMillis();
    if (!(FE_IsWindowReady(&s_fe) &&
          (now_ms - s_last_window_ms >= FEATURE_WINDOW_SECONDS * 1000U))) {
        return;
    }

    s_last_window_ms = now_ms;
    uint32_t t_feat = Utils_GetMicros();

    uint8_t ok = FE_BuildFeatureVector(&s_fe, s_feature_vec);
    if (!ok) {
        LOG_ERR("Feature build failed");
        return;
    }
    uint32_t t_feat_us = Utils_ElapsedMicros(t_feat);

    uint32_t t_infer = Utils_GetMicros();
    NN_Forward(&s_nn, s_feature_vec);
    uint8_t pred = NN_Predict(&s_nn);
    uint32_t t_infer_us = Utils_ElapsedMicros(t_infer);

    s_latest_pred = pred;

    s_window_count++;

    LOG_INF("Window #%lu | Pred=%u [N=%.3f I=%.3f L=%.3f] | "
            "feat=%lu µs infer=%lu µs",
            s_window_count, pred,
            (double)s_nn.a2[CLASS_NORMAL],
            (double)s_nn.a2[CLASS_IMBALANCE],
            (double)s_nn.a2[CLASS_LOOSENESS],
            t_feat_us, t_infer_us);

    App_UpdateLEDs(pred, 0U);

    DataLogger_LogWindow(s_window_count, now_ms,
                          s_cf_out.pitch, s_cf_out.roll, pred,
                          s_nn.a2[CLASS_NORMAL], s_nn.a2[CLASS_IMBALANCE],
                          s_nn.a2[CLASS_LOOSENESS], s_current_label);

    DataLogger_LogRawWindow(s_window_count, now_ms,
                             s_current_label, pred,
                             s_nn.a2[CLASS_NORMAL], s_nn.a2[CLASS_IMBALANCE],
                             s_nn.a2[CLASS_LOOSENESS],
                             s_feature_vec, FEATURE_VECTOR_SIZE);

    if (s_flc.state == FLC_STATE_IDLE ||
        s_flc.state == FLC_STATE_COLLECTING) {

        uint8_t submitted = FLC_SubmitSample(
            &s_flc, s_feature_vec, s_current_label);

        if (submitted) {
            LOG_INF("FL: submitted window #%lu with label=%u "
                    "(%u/%u buffered)",
                    s_window_count, s_current_label,
                    s_flc.train_buf.count, FL_LOCAL_EPOCHS);
        }
    }
}

static void Mod_FLC_Tick(void)
{
    if (!(s_flc.state == FLC_STATE_TRAINING ||
          s_flc.state == FLC_STATE_UPLOADING ||
          s_flc.state == FLC_STATE_DOWNLOADING ||
          s_flc.state == FLC_STATE_ERROR)) {
        return;
    }

    uint32_t round_before = s_flc.round_count;

    App_UpdateLEDs(s_current_label, 1U);   
    FLC_Tick(&s_flc);
    App_UpdateLEDs(s_current_label, 0U);

    if (s_flc.round_count != round_before) {
        DataLogger_LogFLRound(s_flc.round_count, Utils_GetMillis(),
                               s_flc.last_round_loss, s_flc.total_samples,
                               s_flc.server_connected);
    }
}

static void Mod_UartCommand_Tick(void)
{
    if (!g_uart_rx_ready) {
        return;
    }
    g_uart_rx_ready = 0U;
    App_HandleUARTCommand(g_uart_rx_byte);
    HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U);
}

static void App_Init(void)
{
    Utils_TimerInit();

    HAL_TIM_Base_Start_IT(&htim3);

    Utils_SeedRNG(0xABCD1234UL);

    NN_Init(&s_nn);

    FLC_Init(&s_flc, &s_nn, &huart1);

    CF_Init(&s_cf_state);

    FE_Init(&s_fe);

    MPU6050_Status_t mpu_ret = MPU6050_Init(&s_mpu, &hi2c1);
    if (mpu_ret != MPU6050_OK) {
        LOG_ERR("MPU6050 init FAILED (code %u) — check I2C wiring", mpu_ret);
        while (1) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);  
            HAL_Delay(100U);
        }
    }

    /* Skip blocking gyro calibration for now so the main loop can run */
    LOG_INF("MPU6050 calibration skipped (temporary)");

    AppModule_Register(&s_mod_imu);
    AppModule_Register(&s_mod_feature);
    AppModule_Register(&s_mod_flc);
    AppModule_Register(&g_datalogger_module);   
    AppModule_Register(&s_mod_uart);

    AppModule_InitAll();   

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
 *   'l' — print SD data-logger statistics (rows written/dropped, errors)
 *   'f' — force an immediate SD flush (f_sync) of the open log file
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
        case 'l': {
            DataLogger_Stats_t st;
            DataLogger_GetStats(&st);
            LOG_INF("SD LOG:  mounted=%u written=%lu dropped=%lu write_err=%lu",
                    st.log_mounted, st.log_rows_written, st.log_rows_dropped, st.log_write_errors);
            LOG_INF("SD DATA: mounted=%u written=%lu write_err=%lu",
                    st.data_mounted, st.data_rows_written, st.data_write_errors);
            LOG_INF("SD IMU:  mounted=%u written=%lu dropped=%lu write_err=%lu remounts=%lu",
                    st.imu_mounted, st.imu_rows_written, st.imu_rows_dropped, st.imu_write_errors,
                    st.remount_count);
            break;
        }
        case 'f':
            DataLogger_Flush();
            LOG_INF("SD Log: forced flush complete");
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

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 8U;
    RCC_OscInitStruct.PLL.PLLN            = 336U;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 7U;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

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
    hi2c1.Init.ClockSpeed      = 400000U;   
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
    HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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