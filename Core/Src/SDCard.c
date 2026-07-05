/**
 * @file    SDCard.c
 * @brief   SDIO block-device driver implementation. See SDCard.h.
 */

#include "SDCard.h"
#include "Utils.h"
#include <string.h>

#define SDCARD_BLOCK_SIZE      512U

/** SD peripheral handle — local to this module. Nothing above SDCard.c
 *  (i.e. diskio.c, DataLogger.c, main.c) ever touches SD_HandleTypeDef
 *  directly, which is the point of the abstraction. */
static SD_HandleTypeDef s_hsd;
static uint8_t          s_initialized = 0U;

static void SDCard_GPIO_Init(void);
static void SDCard_GPIO_DeInit(void);

/* =========================================================================
 * CARD DETECT (optional — off by default, see Config.h)
 * ========================================================================= */

uint8_t SDCard_IsPresent(void)
{
#if (SD_DETECT_GPIO_ENABLE)
    /* Active-low detect switch, internal pull-up assumed enabled in GPIO init */
    return (HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_GPIO_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return 1U;
#endif
}

/* =========================================================================
 * GPIO / PERIPHERAL CLOCK CONFIG
 * ========================================================================= */

static void SDCard_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SDIO_CLK_ENABLE();

    /* PC8..PC12 = D0,D1,D2,D3,CK — AF12, push-pull, very high speed, pull-up
     * (external pull-ups are also present on most SD sockets; internal
     * pull-ups here guard against a card being removed mid-transfer). */
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PD2 = CMD — AF12 */
    gpio.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &gpio);

#if (SD_DETECT_GPIO_ENABLE)
    /* Optional card-detect input, active low, internal pull-up */
    GPIO_InitTypeDef det = {0};
    det.Pin  = SD_DETECT_GPIO_PIN;
    det.Mode = GPIO_MODE_INPUT;
    det.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD_DETECT_GPIO_PORT, &det);
#endif
}

static void SDCard_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
    __HAL_RCC_SDIO_CLK_DISABLE();
}

/* =========================================================================
 * INIT / DEINIT
 * ========================================================================= */

SDCard_Status_t SDCard_Init(void)
{
    if (!SDCard_IsPresent()) {
        LOG_ERR("SD: card-detect reports no card inserted");
        return SDCARD_ERR_NOT_PRESENT;
    }

    SDCard_GPIO_Init();

    memset(&s_hsd, 0, sizeof(s_hsd));
    s_hsd.Instance                 = SDIO;
    s_hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    s_hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    s_hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    s_hsd.Init.BusWide             = SDIO_BUS_WIDE_1B;   /* 1-bit during identification */
    s_hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    /* SDIO_CK = SDIOCLK / (2 + ClockDiv). ClockDiv=118 → 48 MHz/120 ≈ 400 kHz
     * for card identification, matching the SD spec's <400 kHz init clock. */
    s_hsd.Init.ClockDiv            = SDIO_INIT_CLK_DIV;

    if (HAL_SD_Init(&s_hsd) != HAL_OK) {
        LOG_ERR("SD: HAL_SD_Init failed");
        SDCard_GPIO_DeInit();
        return SDCARD_ERR_INIT;
    }

#if (SD_USE_4BIT_BUS)
    /* Switch to full speed + 4-bit wide bus now that the card is enumerated.
     * SDIO_TRANSFER_CLK_DIV (0) → 48 MHz / (0+2) = 24 MHz, within Class-10 spec. */
    s_hsd.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV;
    (void)HAL_SD_ConfigWideBusOperation(&s_hsd, SDIO_BUS_WIDE_4B);
#endif

    SDCard_Status_t rdy = SDCard_WaitReady(SD_INIT_TIMEOUT_MS);
    if (rdy != SDCARD_OK) {
        LOG_ERR("SD: card did not reach TRANSFER state after init");
        return rdy;
    }

    s_initialized = 1U;
    SDCard_PrintInfo();
    return SDCARD_OK;
}

void SDCard_DeInit(void)
{
    if (s_initialized) {
        HAL_SD_DeInit(&s_hsd);
    }
    SDCard_GPIO_DeInit();
    s_initialized = 0U;
}

SDCard_Status_t SDCard_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = Utils_GetMillis();
    while (HAL_SD_GetCardState(&s_hsd) != HAL_SD_CARD_TRANSFER) {
        if ((Utils_GetMillis() - t0) > timeout_ms) {
            return SDCARD_ERR_TIMEOUT;
        }
    }
    return SDCARD_OK;
}

/* =========================================================================
 * BLOCK I/O
 * ========================================================================= */

SDCard_Status_t SDCard_ReadBlocks(uint8_t *dst, uint32_t start_block, uint32_t count)
{
    if (!s_initialized) return SDCARD_ERR_INIT;

    if (HAL_SD_ReadBlocks(&s_hsd, dst, start_block, count, SD_INIT_TIMEOUT_MS) != HAL_OK) {
        LOG_ERR("SD: read failed at block %lu (%lu blocks)", start_block, count);
        return SDCARD_ERR_READ;
    }
    return SDCard_WaitReady(SD_INIT_TIMEOUT_MS);
}

SDCard_Status_t SDCard_WriteBlocks(const uint8_t *src, uint32_t start_block, uint32_t count)
{
    if (!s_initialized) return SDCARD_ERR_INIT;

    /* HAL_SD_WriteBlocks() takes a non-const pointer (DMA source) — the
     * driver never mutates caller data, so the cast is safe. */
    if (HAL_SD_WriteBlocks(&s_hsd, (uint8_t *)src, start_block, count, SD_INIT_TIMEOUT_MS) != HAL_OK) {
        LOG_ERR("SD: write failed at block %lu (%lu blocks)", start_block, count);
        return SDCARD_ERR_WRITE;
    }
    return SDCard_WaitReady(SD_INIT_TIMEOUT_MS);
}

uint32_t SDCard_GetBlockCount(void)
{
    HAL_SD_CardInfoTypeDef info;
    if (HAL_SD_GetCardInfo(&s_hsd, &info) != HAL_OK) {
        return 0U;
    }
    return info.LogBlockNbr;
}

void SDCard_PrintInfo(void)
{
    HAL_SD_CardInfoTypeDef info;
    if (HAL_SD_GetCardInfo(&s_hsd, &info) != HAL_OK) {
        LOG_ERR("SD: HAL_SD_GetCardInfo failed");
        return;
    }

    const char *type = (info.CardType == CARD_SDSC) ? "SDSC" :
                        (info.CardType == CARD_SDHC_SDXC) ? "SDHC/SDXC" : "UNKNOWN";

    uint32_t capacity_mb = (uint32_t)(((uint64_t)info.LogBlockNbr * info.LogBlockSize) / (1024ULL * 1024ULL));

    LOG_INF("SD: type=%s blocks=%lu block_size=%lu capacity=%lu MB",
            type, info.LogBlockNbr, info.LogBlockSize, capacity_mb);
    LOG_INF("CardType      = %lu", info.CardType);
LOG_INF("LogBlockNbr   = %lu", info.LogBlockNbr);
LOG_INF("LogBlockSize  = %lu", info.LogBlockSize);

#ifdef HAL_SD_CARDINFO_HAS_BLOCKNBR
LOG_INF("BlockNbr      = %lu", info.BlockNbr);
LOG_INF("BlockSize     = %lu", info.BlockSize);
#endif
}
